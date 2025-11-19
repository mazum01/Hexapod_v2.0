#!/usr/bin/env python3
import argparse
import base64
import fnmatch
import re
import sys
import time
from pathlib import Path
from typing import Optional, Tuple, Dict, List

import serial
import serial.tools.list_ports

# ----------- Protocol markers -----------
BEGIN_RAW_RE = re.compile(rb"---BEGIN RAW (.+?)---\r?\n")
END_RAW_RE   = re.compile(rb"\r?\n---END RAW (.+?)---\r?\n")
BEGIN_B64_RE = re.compile(rb"---BEGIN BASE64 (.+?)---\r?\n")
END_B64_RE   = re.compile(rb"\r?\n---END BASE64 (.+?)---\r?\n")

# ----------- Serial helpers -----------
def find_teensy_port(prefer: Optional[str] = None) -> str:
    if prefer and Path(prefer).exists():
        return prefer
    ports = list(serial.tools.list_ports.grep(r"ACM|Teensy|ttyACM"))
    if ports:
        return ports[0].device
    cand = "/dev/ttyACM0"
    if Path(cand).exists():
        return cand
    raise SystemExit("No Teensy serial port found. Use --port.")

def open_serial(port: str, baud: int, timeout: float = 1.0) -> serial.Serial:
    ser = serial.Serial(port=port, baudrate=baud, timeout=timeout)
    time.sleep(0.5)
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    return ser

def send_command(ser: serial.Serial, line: str) -> None:
    ser.write((line + "\n").encode("utf-8"))
    ser.flush()

def read_until_regex(ser: serial.Serial, pattern: re.Pattern, chunk=1024, max_wait=30.0):
    buf = bytearray()
    start = time.time()
    while True:
        b = ser.read(chunk)
        if b:
            buf.extend(b)
            m = pattern.search(buf)
            if m:
                return bytes(buf), m
        else:
            if (time.time() - start) > max_wait:
                raise TimeoutError("Timed out waiting for marker.")
        time.sleep(0.005)

# ----------- Downloaders -----------
def pull_raw(ser: serial.Serial, remote_path: str, local_root: Path) -> Path:
    send_command(ser, f"dl {remote_path}")
    buf, m_begin = read_until_regex(ser, BEGIN_RAW_RE)
    begin_path = m_begin.group(1).decode("utf-8", "replace")

    local_path = (local_root / begin_path.lstrip("/")).resolve()
    local_path.parent.mkdir(parents=True, exist_ok=True)

    with open(local_path, "wb") as f:
        start_idx = m_begin.end()
        tail = buf[start_idx:]
        while True:
            m_end = END_RAW_RE.search(tail)
            if m_end:
                f.write(tail[:m_end.start()])
                break
            if tail:
                f.write(tail)
            more = ser.read(4096)
            if not more:
                time.sleep(0.01)
                continue
            tail = more
    return local_path

def pull_b64(ser: serial.Serial, remote_path: str, local_root: Path) -> Path:
    send_command(ser, f"b64 {remote_path}")
    buf, m_begin = read_until_regex(ser, BEGIN_B64_RE)
    begin_path = m_begin.group(1).decode("utf-8", "replace")

    local_path = (local_root / begin_path.lstrip("/")).resolve()
    local_path.parent.mkdir(parents=True, exist_ok=True)

    b64_accum = bytearray()
    tail = buf[m_begin.end():]
    while True:
        m_end = END_B64_RE.search(tail)
        if m_end:
            b64_accum.extend(tail[:m_end.start()].replace(b"\r", b"").replace(b"\n", b""))
            data = base64.b64decode(b64_accum, validate=False)
            with open(local_path, "wb") as f:
                f.write(data)
            break
        if tail:
            b64_accum.extend(tail.replace(b"\r", b"").replace(b"\n", b""))
        more = ser.read(4096)
        if not more:
            time.sleep(0.01)
            continue
        tail = more
    return local_path

# ----------- Remote listing / wildcard expansion -----------
def get_all_files(ser: serial.Serial, root="/") -> List[str]:
    send_command(ser, f"lsr {root}")
    lines: List[str] = []
    start = time.time()
    while True:
        line = ser.readline()
        if not line:
            if time.time() - start > 10.0:
                break
            continue
        s = line.decode(errors="ignore").strip()
        if s == "===END LSR===":
            break
        if s and not s.startswith("---"):
            if not s.startswith("/"):
                s = "/" + s
            lines.append(s)
    return lines

def _normalize_pattern(p: str) -> List[str]:
    out = [p]
    if p.startswith("/"):
        out.append(p.lstrip("/"))
    else:
        out.append("/" + p)
    seen, res = set(), []
    for x in out:
        if x not in seen:
            seen.add(x); res.append(x)
    return res

def expand_wildcards(ser: serial.Serial, paths: List[str], ignore_case: bool, literal: bool) -> List[str]:
    if literal:
        return [(p if p.startswith("/") else "/" + p) for p in paths]

    needs_glob = any(("*" in p or "?" in p) for p in paths)
    if not needs_glob:
        return [(p if p.startswith("/") else "/" + p) for p in paths]

    remote_files = get_all_files(ser, "/")
    if ignore_case:
        remote_fold = [(f, f.lower()) for f in remote_files]
    else:
        remote_fold = [(f, f) for f in remote_files]

    expanded: List[str] = []
    for pat in paths:
        if "*" in pat or "?" in pat:
            variants = _normalize_pattern(pat)
            matches = set()
            if ignore_case:
                for v in variants:
                    v_fold = v.lower()
                    for orig, lower in remote_fold:
                        if fnmatch.fnmatch(lower, v_fold):
                            matches.add(orig)
            else:
                for v in variants:
                    matches.update(fnmatch.filter(remote_files, v))
            if not matches:
                print(f"[INFO] No remote matches for pattern: {pat}")
            expanded.extend(sorted(matches))
        else:
            expanded.append(pat if p.startswith("/") else "/" + pat)
    return sorted(set(expanded))

# ----------- Concatenation: rotation-aware (base-first, reverse option) -----------
ROT_BEFORE_EXT = re.compile(r"""^(?P<root>.+?)[._-](?P<idx>\d+)(?P<ext>\.[^.]+)$""")
ROT_AFTER_EXT  = re.compile(r"""^(?P<root>.+?)(?P<ext>\.[^.]+)[._-](?P<idx>\d+)$""")
LEADING_NUM    = re.compile(r"""^(?P<idx>\d+)[._-]?(?P<rootrest>.*)$""")

def parse_rotation_parts(basename: str) -> Optional[Tuple[str, int, str]]:
    m = ROT_BEFORE_EXT.match(basename)
    if m:
        return (m.group("root"), int(m.group("idx")), m.group("ext"))
    m = ROT_AFTER_EXT.match(basename)
    if m:
        return (m.group("root"), int(m.group("idx")), m.group("ext"))
    m = LEADING_NUM.match(basename)
    if m and m.group("rootrest"):
        idx = int(m.group("idx"))
        rr  = m.group("rootrest")
        dot = rr.rfind(".")
        if dot > 0:
            return (rr[:dot], idx, rr[dot:])
        else:
            return (rr, idx, "")
    return None

def split_root_ext(basename: str) -> Tuple[str, str]:
    i = basename.rfind(".")
    if i > 0:
        return basename[:i], basename[i:]
    return basename, ""

def group_rotated_with_base(local_paths: List[Path]) -> Dict[Tuple[Path, str, str], List[Tuple[Path, int]]]:
    groups: Dict[Tuple[Path, str, str], List[Tuple[Path, int]]] = {}
    rotated_keys: set = set()
    non_rotated: List[Path] = []

    for p in local_paths:
        rot = parse_rotation_parts(p.name)
        if rot:
            root, idx, ext = rot
            key = (p.parent, root, ext)
            rotated_keys.add(key)
            groups.setdefault(key, []).append((p, idx))
        else:
            non_rotated.append(p)

    for p in non_rotated:
        root, ext = split_root_ext(p.name)
        key = (p.parent, root, ext)
        if key in rotated_keys:
            groups.setdefault(key, []).append((p, 0))  # base

    for key in list(groups.keys()):
        groups[key].sort(key=lambda t: t[1])
    return groups

def concatenate_groups(groups: Dict[Tuple[Path, str, str], List[Tuple[Path, int]]],
                       dest_dir: Path, suffix: str, dry_run: bool, reverse: bool) -> List[Path]:
    results: List[Path] = []
    for (gdir, root, ext) in sorted(groups.keys(), key=lambda k: (str(k[0]), k[1], k[2])):
        entries = groups[(gdir, root, ext)]
        if reverse:
            entries = sorted(entries, key=lambda t: t[1], reverse=True)
        else:
            entries = sorted(entries, key=lambda t: t[1])

        out_name = f"{root}{ext}{suffix}" if suffix else f"{root}{ext}"
        out_path = dest_dir / out_name

        order = "descending" if reverse else "ascending"
        print(f"[CONCAT] {gdir}/{root}{ext} ({order}): {len(entries)} file(s) -> {out_path}")
        for p, idx in entries:
            label = "(base)" if idx == 0 else f"(part {idx})"
            print(f"         - {p.name} {label}")

        if dry_run:
            continue

        out_path.parent.mkdir(parents=True, exist_ok=True)
        with open(out_path, "wb") as outfh:
            for p, _idx in entries:
                with open(p, "rb") as infh:
                    while True:
                        chunk = infh.read(1024 * 256)
                        if not chunk:
                            break
                        outfh.write(chunk)
        results.append(out_path)
    return results

# ----------- CLI / main -----------
def load_manifest(paths_file: Path) -> List[str]:
    items = []
    with open(paths_file, "r", encoding="utf-8") as f:
        for line in f:
            s = line.strip()
            if not s or s.startswith("#"):
                continue
            items.append(s)
    return items

def main():
    ap = argparse.ArgumentParser(description="Download files from Teensy SD over USB serial (wildcards + concat).")
    ap.add_argument("paths", nargs="*", help="Remote SD paths or patterns (quote wildcards: 'logs/*.txt').")
    ap.add_argument("--manifest", "-m", type=str, help="Text file with remote paths/patterns (one per line).")
    ap.add_argument("--port", "-p", type=str, help="Serial port (e.g. /dev/ttyACM0). Auto-detect if omitted.")
    ap.add_argument("--baud", "-b", type=int, default=115200, help="Baud rate (default 115200).")
    ap.add_argument("--out", "-o", type=str, default="teensy_downloads", help="Local output directory.")
    ap.add_argument("--mode", choices=["raw", "b64"], default="raw", help="Use Teensy's 'dl' (raw) or 'b64' (base64).")
    ap.add_argument("--ignore-case", action="store_true", help="Case-insensitive wildcard matching.")
    ap.add_argument("--literal", action="store_true", help="Do not expand wildcards; treat args as exact remote paths.")
    ap.add_argument("--concat", action="store_true",
                    help="After download, concatenate rotated families (base first by default).")
    ap.add_argument("--concat-dir", type=str, default=None,
                    help="Directory to write concatenated files (default: same as --out).")
    ap.add_argument("--concat-suffix", type=str, default=".concat",
                    help="Suffix for concatenated outputs (use '' to overwrite base name).")
    ap.add_argument("--concat-dry-run", action="store_true",
                    help="Show grouping and order, but do not write concatenated files.")
    ap.add_argument("--concat-reverse", action="store_true",
                    help="Concatenate in reverse order (newest part first, base last).")
    args = ap.parse_args()

    if not args.paths and not args.manifest:
        ap.print_help(sys.stderr)
        sys.exit(2)

    port = find_teensy_port(args.port)
    ser = open_serial(port, args.baud, timeout=0.2)
    outdir = Path(args.out).resolve()
    outdir.mkdir(parents=True, exist_ok=True)

    requested: List[str] = []
    if args.paths:
        requested.extend(args.paths)
    if args.manifest:
        requested.extend(load_manifest(Path(args.manifest)))

    expanded = expand_wildcards(ser, requested, ignore_case=args.ignore_case, literal=args.literal)
    if not expanded:
        print("[WARN] No matching remote files found.")
        sys.exit(0)

    print(f"[INFO] Will download {len(expanded)} files...")
    downloaded_local_paths: List[Path] = []
    for remote_path in expanded:
        if not remote_path.startswith("/"):
            remote_path = "/" + remote_path
        try:
            if args.mode == "raw":
                local = pull_raw(ser, remote_path, outdir)
            else:
                local = pull_b64(ser, remote_path, outdir)
            print(f"[OK] {remote_path} -> {local}")
            downloaded_local_paths.append(local)
        except Exception as e:
            print(f"[FAIL] {remote_path} ({e})")
            ser.reset_input_buffer()
    ser.close()

    if args.concat:
        concat_dir = Path(args.concat_dir).resolve() if args.concat_dir else outdir
        groups = group_rotated_with_base(downloaded_local_paths)
        if not groups:
            print("[INFO] No rotated families detected; nothing to concatenate.")
            return
        results = concatenate_groups(groups, concat_dir, args.concat_suffix, args.concat_dry_run, args.concat_reverse)
        if args.concat_dry_run:
            print("[INFO] Dry run complete. No files written.")
        else:
            for outp in results:
                print(f"[DONE] Wrote {outp}")

if __name__ == "__main__":
    main()
