#!/usr/bin/env python3
"""Wrap an Actions-built lavender-shim bundle with the local Lavender DTB.

No compiler is used here. The XNU command line comes from the compiled
payload, so the Android header and the arguments XNU actually sees agree.
"""
import argparse
import gzip
import hashlib
import json
from pathlib import Path
import struct
import subprocess
import sys
import tempfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bundle', type=Path, help='downloaded lavender-shim artifact directory')
    parser.add_argument('dtb', type=Path, help='existing resources/lavender_dtb.bin')
    parser.add_argument('output', type=Path, help='new boot image (must not already exist)')
    args = parser.parse_args()
    if args.output.exists():
        parser.error(f'refusing to overwrite {args.output}')
    if not args.output.parent.is_dir():
        parser.error(f'output directory does not exist: {args.output.parent}')

    build_info = json.loads((args.bundle / 'build-info.json').read_text())
    parts = {}
    for name in ('Image.gz', 'payload.bin', 'ramdisk.bin', 'lavender.adt'):
        data = (args.bundle / name).read_bytes()
        if hashlib.sha256(data).hexdigest() != build_info['sha256'][name]:
            parser.error(f'{name}: checksum does not match the Actions build')
        parts[name] = data

    payload = parts['payload.bin']
    if len(payload) < 0x2a8 or payload[:8] != b'SWEETXNU':
        parser.error('invalid XNU payload header')
    version, nsegs = struct.unpack_from('<II', payload, 8)
    if version != 1 or not 1 <= nsegs <= 32:
        parser.error('unsupported payload version or segment count')
    phys_base, mem_size = struct.unpack_from('<QQ', payload, 0x38)
    if (phys_base, mem_size) != (0xa0000000, 0x40000000):
        parser.error('payload does not use the Lavender memory layout')
    command = payload[0x48:0x2a8]
    if b'\0' not in command:
        parser.error('payload command line is not NUL-terminated')
    cmdline = command.split(b'\0')[0].decode()
    if cmdline != build_info['cmdline']:
        parser.error('payload command line differs from the Actions build record')

    image = gzip.decompress(parts['Image.gz'])
    if image[56:60] != b'ARM\x64' or not image.endswith(payload):
        parser.error('compiled ARM64 Image does not contain this payload')
    dtb = args.dtb.read_bytes()
    if len(dtb) < 40 or dtb[:4] != b'\xd0\x0d\xfe\xed':
        parser.error('not a flattened device tree')
    if struct.unpack_from('>I', dtb, 4)[0] != len(dtb):
        parser.error('DTB total size does not match the supplied file')

    with tempfile.TemporaryDirectory(prefix='lavender-pack-', dir=args.output.parent) as work:
        kernel_path = Path(work) / 'Image.gz-dtb'
        kernel_path.write_bytes(parts['Image.gz'] + dtb)
        subprocess.run([
            sys.executable, str(Path(__file__).with_name('mkbootimg.py')),
            '--kernel', str(kernel_path), '--ramdisk', str(args.bundle / 'ramdisk.bin'),
            '--output', str(args.output), '--base', '0x00000000',
            '--kernel_offset', '0x00008000', '--ramdisk_offset', '0x02000000',
            '--tags_offset', '0x00000100', '--pagesize', '4096', '--header_version', '0',
            '--cmdline', cmdline,
        ], check=True)

    digest = hashlib.sha256(args.output.read_bytes()).hexdigest()
    args.output.with_name(args.output.name + '.sha256').write_text(f'{digest}  {args.output.name}\n')
    record = dict(build_info, dtb_sha256=hashlib.sha256(dtb).hexdigest(), boot_image_sha256=digest)
    args.output.with_name(args.output.name + '.json').write_text(json.dumps(record, indent=2) + '\n')
    print(f'{args.output}: sha256 {digest}')
    print(f'XNU command line ({len(cmdline.encode())}/607 bytes): {cmdline}')


if __name__ == '__main__':
    main()
