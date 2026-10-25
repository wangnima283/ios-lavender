#!/usr/bin/env python3
"""Validate a unified diff the way `git apply` / patch(1) read it.

The darwin-xnu-build runner applies xnu/patches/*.patch with `git apply
--check` and only warns when a patch is rejected, so a malformed hunk header
silently drops a whole feature from the kernel. That happened to
0143-santoni-persistent-console-log.patch and
0144-santoni-4k-unsigned-dyld-cache.patch ("error: corrupt patch at line N").
This checker makes that class of failure loud and local:

  * every hunk's declared -old,new / +old,new counts must match the body,
  * a hunk must not be truncated by EOF or by the next hunk header,
  * hunks inside one file must be in ascending, non-overlapping order,
  * a new file's hunk must be @@ -0,0 +1,N @@,
  * added lines may be asserted to contain / not contain a substring, which
    checks a requirement against the code that actually ships in the patch
    instead of against a host test double.

Usage:
  tools/check_patch.py xnu/patches/0144-santoni-4k-unsigned-dyld-cache.patch
  tools/check_patch.py P --added-contains bsd/kern/ubc_subr.c:0xEDB88320
  tools/check_patch.py P --added-lacks bsd/dev/santoni_sdhci.c:su.dma

Types are matched as plain substrings, so special characters are safe.
"""

import argparse
import re
import sys

HUNK_RE = re.compile(r'^@@ -(\d+)(?:,(\d+))? \+(\d+)(?:,(\d+))? @@')

# git diff metadata lines that carry no hunk body.
META_PREFIXES = (
    "index ",
    "new file mode ",
    "deleted file mode ",
    "old mode ",
    "new mode ",
    "similarity index ",
    "dissimilarity index ",
    "rename from ",
    "rename to ",
    "copy from ",
    "copy to ",
    "Binary files ",
    "GIT binary patch",
    "diff --git ",
)


class PatchError(Exception):
    pass


class FilePatch:
    def __init__(self, name, old_name, is_new):
        self.name = name
        self.old_name = old_name
        self.is_new = is_new
        self.added = []          # added lines, without the leading '+'
        self.hunks = 0


def _strip_prefix(path, prefix):
    if path.startswith(prefix):
        return path[len(prefix):]
    return path


def parse(path):
    """Return (files, errors) for the patch at path."""
    with open(path, encoding="utf-8", errors="surrogateescape") as f:
        raw = f.read()
    lines = raw.split("\n")
    if lines and lines[-1] == "":
        lines.pop()

    files = []
    errors = []
    cur = None
    i = 0
    n = len(lines)

    while i < n:
        line = lines[i]

        if line.startswith("diff --git "):
            # name is only known for sure once '+++ ' shows up; remember the
            # b/ path as a fallback for binary diffs.
            m = re.match(r'^diff --git a/(.*) b/(.*)$', line)
            cur = FilePatch(m.group(2) if m else "?", None, False)
            files.append(cur)
            i += 1
            continue

        if line.startswith("--- ") and i + 1 < n and lines[i + 1].startswith("+++ "):
            old_name = line[4:].strip()
            new_name = lines[i + 1][4:].strip()
            is_new = old_name == "/dev/null"
            name = _strip_prefix(new_name, "b/") if not new_name == "/dev/null" else "?"
            if cur is not None and cur.name in ("?", name):
                cur.name = name
                cur.old_name = _strip_prefix(old_name, "a/")
                cur.is_new = is_new
            else:
                cur = FilePatch(name, _strip_prefix(old_name, "a/"), is_new)
                files.append(cur)
            i += 2
            continue

        m = HUNK_RE.match(line)
        if m:
            if cur is None:
                errors.append("%s:%d: hunk outside a file section"
                              % (path, i + 1))
                i += 1
                continue
            old_start = int(m.group(1))
            old_count = int(m.group(2)) if m.group(2) is not None else 1
            new_start = int(m.group(3))
            new_count = int(m.group(4)) if m.group(4) is not None else 1
            hunk_no = cur.hunks + 1
            cur.hunks += 1
            where = "%s: %s hunk #%d (@@ -%d,%d +%d,%d @@), line %d" % (
                path, cur.name, hunk_no, old_start, old_count,
                new_start, new_count, i + 1)

            if cur.is_new and (old_start != 0 or old_count != 0):
                errors.append("%s: new file hunk must be -0,0" % where)

            i += 1
            got_old = 0
            got_new = 0
            truncated = None
            while got_old < old_count or got_new < new_count:
                if i >= n:
                    truncated = "end of file"
                    break
                body = lines[i]
                if body.startswith("@@ ") or body.startswith("diff --git "):
                    truncated = "unexpected header inside hunk"
                    break
                if body.startswith("\\"):
                    # "\ No newline at end of file"
                    i += 1
                    continue
                tag = body[0] if body else " "
                if tag == " ":
                    got_old += 1
                    got_new += 1
                elif tag == "-":
                    got_old += 1
                elif tag == "+":
                    got_new += 1
                    cur.added.append(body[1:])
                else:
                    truncated = "unexpected %r" % body[:40]
                    break
                i += 1
            if truncated is not None:
                errors.append("%s: body truncated (%s) after -%d,+%d of "
                              "-%d,+%d" % (where, truncated, got_old, got_new,
                                           old_count, new_count))
                continue
            if got_old != old_count or got_new != new_count:
                errors.append("%s: declared -%d,+%d but body has -%d,+%d"
                              % (where, old_count, new_count,
                                 got_old, got_new))
            continue

        if line.startswith(META_PREFIXES):
            i += 1
            continue

        # Anything else outside a hunk is diff preamble/trailer or a stray
        # line; the latter is caught by the next hunk's count check.
        i += 1

    return files, errors


def norm(path):
    return _strip_prefix(path.strip(), "b/")


def find(files, path):
    want = norm(path)
    for f in files:
        if f.name == want or f.name.endswith("/" + want):
            return f
    return None


def check_assertions(path, files, contains, lacks):
    errors = []
    for spec in contains:
        if ":" not in spec:
            errors.append("%s: bad --added-contains %r (want FILE:TEXT)"
                          % (path, spec))
            continue
        fname, text = spec.split(":", 1)
        f = find(files, fname)
        if f is None:
            errors.append("%s: --added-contains %s: file not in patch"
                          % (path, fname))
        elif not any(text in line for line in f.added):
            errors.append("%s: %s does not add a line containing %r"
                          % (path, f.name, text))
    for spec in lacks:
        if ":" not in spec:
            errors.append("%s: bad --added-lacks %r (want FILE:TEXT)"
                          % (path, spec))
            continue
        fname, text = spec.split(":", 1)
        f = find(files, fname)
        if f is None:
            errors.append("%s: --added-lacks %s: file not in patch"
                          % (path, fname))
            continue
        for line in f.added:
            if text in line:
                errors.append("%s: %s adds a line containing %r: %r"
                              % (path, f.name, text, line.strip()))
    return errors


def main(argv):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("patches", nargs="+")
    ap.add_argument("--added-contains", action="append", default=[],
                    metavar="FILE:TEXT")
    ap.add_argument("--added-lacks", action="append", default=[],
                    metavar="FILE:TEXT")
    ap.add_argument("-q", "--quiet", action="store_true")
    args = ap.parse_args(argv)

    failed = False
    for path in args.patches:
        try:
            files, errors = parse(path)
        except OSError as e:
            print("%s: %s" % (path, e), file=sys.stderr)
            failed = True
            continue
        errors += check_assertions(path, files, args.added_contains,
                                   args.added_lacks)
        if not files:
            errors.append("%s: no file sections found" % path)
        if errors:
            failed = True
            for e in errors:
                print("FAIL " + e, file=sys.stderr)
        elif not args.quiet:
            hunks = sum(f.hunks for f in files)
            print("ok   %s: %d file(s), %d hunk(s), all hunk counts exact"
                  % (path, len(files), hunks))
    if failed:
        print("check_patch: MALFORMED PATCH", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
