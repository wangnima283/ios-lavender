#!/usr/bin/env python3
"""Pre-flight check for shim/build.sh: kernel payload and appended DTB.

shim/build.sh packs a raw Mach-O XNU kernel plus the board's appended device
tree into an Android boot image.  A kernel built for the wrong machine, or a
DTB whose qcom,msm-id/qcom,board-id do not match the phone, makes the stock ABL
reject the image with "Failed to load/authenticate boot image: Load Error" -
which looks just like a broken kernel once it is on the device.  Catch it on
the host instead.

Usage:
    check_payload.py --board whyred
    check_payload.py --board whyred --kernel /tmp/kernel.development.whyred
    check_payload.py --board whyred --dtb resources/whyred_dtb.bin
"""
import argparse
import os
import struct
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

FDT_MAGIC = 0xD00DFEED
MACHO_MAGIC64 = 0xFEEDFACF      # on disk (little-endian): cf fa ed fe
CPU_TYPE_ARM64 = 0x0100000C
MH_EXECUTE = 2
MIN_KERNEL_BYTES = 1 << 20      # anything smaller is a stub, not an XNU kernel

# qcom,msm-id and qcom,board-id (first cell of each pair) that the phone's ABL
# matches against.  whyred: sdm636-mtp-whyred.dts has board-id 0x30008/0x10008;
# lavender: sdm660-mtp-lavender.dts has 0x20008.
BOARDS = {
    "whyred": {"msm_id": 0x159, "board_ids": (0x30008, 0x10008), "model": "SDM 636"},
    "lavender": {"msm_id": 0x13D, "board_ids": (0x20008,), "model": "SDM 660"},
    "violet": {"msm_id": 0x163, "board_ids": (0x22,), "model": "SDM 675"},
}


def board_paths(board):
    """Default kernel and DTB paths for a board, both relative to the repo."""
    return (os.path.join(REPO, "xnu", "out", "kernel.development.%s" % board),
            os.path.join(REPO, "resources", "%s_dtb.bin" % board))


def _align(base, pos):
    # FDT struct-block alignment is relative to the blob, and an appended DTB
    # can start at an unaligned file offset.
    return base + ((pos - base + 3) & ~3)


def _u32(blob):
    return [struct.unpack_from(">I", blob, i)[0] for i in range(0, len(blob) - len(blob) % 4, 4)]


def _string(data, start, end, what):
    nul = data.find(b"\0", start, end)
    if nul < 0:
        raise ValueError("unterminated %s at offset %#x" % (what, start))
    return data[start:nul].decode("ascii", "replace"), nul + 1


def parse_fdt(data, base=0):
    """Parse the flattened device tree blob at data[base:].

    Returns a dict with totalsize, model, msm_ids, board_ids and compatible.
    Raises ValueError with a readable message on a malformed blob.
    """
    if base < 0 or base + 40 > len(data):
        raise ValueError("no room for an FDT header at offset %#x" % base)
    (magic, totalsize, off_struct, off_strings, off_rsv, version, last_comp,
     boot_cpuid, size_strings, size_struct) = struct.unpack_from(">10I", data, base)
    if magic != FDT_MAGIC:
        raise ValueError("bad FDT magic %#010x at offset %#x (expected 0x%08x)"
                         % (magic, base, FDT_MAGIC))
    if totalsize < 40 or base + totalsize > len(data):
        raise ValueError("totalsize %d at offset %#x does not fit in %d bytes"
                         % (totalsize, base, len(data)))
    limit = base + totalsize
    if not 0 < off_struct < totalsize or not 0 <= off_strings < totalsize:
        raise ValueError("invalid struct/strings block offset at %#x" % base)

    pos = base + off_struct
    end_struct = min(limit, pos + size_struct) if size_struct else limit
    str_start = base + off_strings
    str_end = min(limit, str_start + size_strings) if size_strings else limit
    depth = 0
    props = {}
    while pos + 4 <= end_struct:
        token = struct.unpack_from(">I", data, pos)[0]
        pos += 4
        if token == 1:                                  # FDT_BEGIN_NODE
            _name, pos = _string(data, pos, limit, "node name")
            pos = _align(base, pos)
            depth += 1
        elif token == 3:                                # FDT_PROP
            if pos + 8 > limit:
                raise ValueError("truncated property header at offset %#x" % pos)
            length, nameoff = struct.unpack_from(">II", data, pos)
            pos += 8
            if pos + length > limit:
                raise ValueError("property at %#x runs past the blob" % pos)
            if depth == 1:
                if not 0 <= nameoff < str_end - str_start:
                    raise ValueError("property name offset %#x outside the strings "
                                     "block at %#x" % (nameoff, base))
                name, _ = _string(data, str_start + nameoff, str_end, "property name")
                props.setdefault(name, data[pos:pos + length])
            pos = _align(base, pos + length)
        elif token == 2:                                # FDT_END_NODE
            depth -= 1
        elif token == 4:                                # FDT_NOP
            continue
        elif token == 9:                                # FDT_END
            break
        else:
            raise ValueError("bad FDT token %#x at offset %#x" % (token, pos - 4 - base))
    else:
        raise ValueError("struct block at %#x has no FDT_END token" % base)

    compatible = props.get("compatible", b"")
    return {
        "offset": base,
        "totalsize": totalsize,
        "model": props.get("model", b"").rstrip(b"\0").decode("utf-8", "replace"),
        "msm_ids": _u32(props.get("qcom,msm-id", b""))[0::2],
        # board-id/msm-id are emitted as pairs (<id subtype>), keep the ids.
        "board_ids": _u32(props.get("qcom,board-id", b""))[0::2],
        "compatible": [c.decode("ascii", "replace")
                       for c in compatible.split(b"\0") if c],
    }


def scan_dtbs(data, start=0, end=None):
    """Walk a concatenated DTB table (Image.gz-dtb) from start to end."""
    end = len(data) if end is None else end
    entries = []
    pos = start
    while pos + 4 <= end and struct.unpack_from(">I", data, pos)[0] == FDT_MAGIC:
        entry = parse_fdt(data, pos)
        entries.append(entry)
        pos += entry["totalsize"]
    if pos < end and any(byte not in (0x00, 0xFF) for byte in data[pos:end]):
        raise ValueError("unparsed data after the last DTB at offset %#x" % pos)
    return entries, pos


def matching_dtbs(entries, board):
    """Entries whose msm-id/board-id the given board's ABL will accept."""
    spec = BOARDS[board]
    wanted_board = set(spec["board_ids"])
    return [e for e in entries
            if spec["msm_id"] in e["msm_ids"] and wanted_board & set(e["board_ids"])]


def describe(entry):
    msm = " ".join("%#x" % v for v in entry["msm_ids"]) or "-"
    bid = " ".join("%#x" % v for v in entry["board_ids"]) or "-"
    return "msm-id <%s> board-id <%s> %s" % (msm, bid, entry["model"] or "(no model)")


def check_kernel(path, board):
    """Validate a raw kernel payload; returns (size, [warnings])."""
    try:
        size = os.path.getsize(path)
        with open(path, "rb") as f:
            head = f.read(32)
    except OSError as exc:
        raise ValueError("cannot read %s: %s" % (path, exc.strerror))
    if size < 32 or len(head) < 32:
        raise ValueError("%s is too small (%d bytes) to be a kernel" % (path, size))
    magic, cputype, _sub, filetype, _ncmds, _sizeofcmds, _flags, _res = \
        struct.unpack("<IiiIIIII", head)
    if magic != MACHO_MAGIC64:
        raise ValueError("%s is not a 64-bit Mach-O kernel (magic %#010x); the "
                         "CI artifact kernel.development.%s is a raw Mach-O"
                         % (path, magic, board))
    if cputype != CPU_TYPE_ARM64:
        raise ValueError("%s has cputype %#x, expected arm64 (%#x)"
                         % (path, cputype & 0xFFFFFFFF, CPU_TYPE_ARM64))
    if filetype != MH_EXECUTE:
        raise ValueError("%s has Mach-O filetype %d, expected MH_EXECUTE (%d)"
                         % (path, filetype, MH_EXECUTE))
    warnings = []
    if size < MIN_KERNEL_BYTES:
        warnings.append("%s is only %d bytes; is that the real XNU kernel?"
                        % (path, size))
    return size, warnings


def check_dtb(path, board):
    """Validate the appended DTB for a board; returns a summary string."""
    try:
        with open(path, "rb") as f:
            data = f.read()
    except OSError as exc:
        raise ValueError("cannot read %s: %s" % (path, exc.strerror))
    if len(data) < 40:
        raise ValueError("%s is too small (%d bytes) to be a DTB" % (path, len(data)))
    if struct.unpack_from(">I", data, 0)[0] != FDT_MAGIC:
        raise ValueError("%s does not start with the FDT magic 0x%08x (a dtbo or "
                         "a boot image is not an appended DTB)" % (path, FDT_MAGIC))
    try:
        entries, _ = scan_dtbs(data)
    except ValueError as exc:
        raise ValueError("%s: %s" % (path, exc))
    hits = matching_dtbs(entries, board)
    if not hits:
        spec = BOARDS[board]
        wanted = ("qcom,msm-id %#x and qcom,board-id %s"
                  % (spec["msm_id"], " or ".join("%#x" % b for b in spec["board_ids"])))
        found = "; ".join(describe(e) for e in entries[:6]) or "none"
        raise ValueError("%s has no %s entry (wanted %s); found: %s"
                         % (path, board, wanted, found))
    best = hits[0]
    return "%s (%d DTB%s, board entry at +%#x: %s)" % (
        path, len(entries), "" if len(entries) == 1 else "s",
        best["offset"], describe(best))


def _size(n):
    return "%.1f KiB" % (n / 1024.0) if n < 1 << 20 else "%.1f MiB" % (n / (1 << 20))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--board", default="whyred", choices=sorted(BOARDS))
    ap.add_argument("--kernel", help="check this kernel payload "
                                     "(default: xnu/out/kernel.development.BOARD)")
    ap.add_argument("--dtb", help="check this appended DTB "
                                  "(default: resources/BOARD_dtb.bin)")
    ap.add_argument("--quiet", "-q", action="store_true")
    args = ap.parse_args(argv)

    def_kernel, def_dtb = board_paths(args.board)
    # With no explicit path both defaults are checked; otherwise only what the
    # caller asked for (build.sh validates the kernel and the DTB separately).
    check_kernel_path = args.kernel is not None or args.dtb is None
    check_dtb_path = args.dtb is not None or args.kernel is None
    kernel = args.kernel or def_kernel
    dtb = args.dtb or def_dtb
    errors = []

    if check_kernel_path:
        if not os.path.exists(kernel):
            errors.append("kernel payload not found: %s (download kernel.development.%s "
                          "from the xnu-kernel CI artifact)" % (kernel, args.board))
        else:
            try:
                size, warnings = check_kernel(kernel, args.board)
            except ValueError as exc:
                errors.append(str(exc))
            else:
                if not args.quiet:
                    print("ok   kernel %s: Mach-O arm64 executable, %s" % (kernel, _size(size)))
                for warning in warnings:
                    print("warn %s" % warning, file=sys.stderr)

    if check_dtb_path:
        if not os.path.exists(dtb):
            errors.append("appended DTB not found: %s (run tools/extract_appended_dtb.py "
                          "on a whyred boot/recovery image)" % dtb)
        else:
            try:
                summary = check_dtb(dtb, args.board)
            except ValueError as exc:
                errors.append(str(exc))
            else:
                if not args.quiet:
                    print("ok   dtb %s" % summary)

    for error in errors:
        print("error: %s" % error, file=sys.stderr)
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
