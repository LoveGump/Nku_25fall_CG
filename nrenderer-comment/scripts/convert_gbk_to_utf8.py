#!/usr/bin/env python3
"""
Convert likely GBK-encoded files to UTF-8.

Usage examples (PowerShell):
  # Dry run: only list candidates
  python .\scripts\convert_gbk_to_utf8.py --root "F:\\code\\CG\\nrenderer-comment" --dry-run

  # Actual conversion with backup
  python .\scripts\convert_gbk_to_utf8.py --root "F:\\code\\CG\\nrenderer-comment"

Requirements (recommended):
  pip install chardet

The script will by default search recursively and check common source/text extensions.
"""
from __future__ import annotations

import argparse
import shutil
import sys
from pathlib import Path
from typing import List, Optional, Tuple

try:
    import chardet
except Exception:
    chardet = None


DEFAULT_EXTS = [
    ".cpp", ".c", ".cc", ".h", ".hpp", ".txt", ".md", ".scn", ".ini",
    ".glsl", ".vert", ".frag", ".py", ".json"
]


def detect_encoding(buf: bytes, use_chardet: bool = True) -> Tuple[Optional[str], float]:
    """Return (encoding, confidence). encoding may be None."""
    if use_chardet and chardet is not None:
        res = chardet.detect(buf)
        return res.get("encoding"), float(res.get("confidence", 0.0))

    # Fallback: try utf-8 strict, then return None
    try:
        buf.decode("utf-8")
        return "utf-8", 1.0
    except Exception:
        return None, 0.0


def probably_gbk(buf: bytes, encoding: Optional[str], confidence: float) -> bool:
    """Decide whether buffer is likely GBK/GB18030-encoded.

    Heuristics:
    - If chardet reports an encoding that contains 'GB' or 'GB2312' or 'GB18030' -> True
    - If chardet not available: try decode as utf-8 strict -> if fails, mark True
    - If utf-8 decode yields replacement chars or length mismatch -> mark True
    """
    if encoding:
        enc_low = encoding.lower()
        if "gb" in enc_low or "cp936" in enc_low or "gb18030" in enc_low:
            return True
        if enc_low == "ascii":
            return False
        # if chardet says utf-8 with high confidence -> not GBK
        if enc_low == "utf-8" and confidence > 0.9:
            return False

    # Try strict utf-8 decode
    try:
        text = buf.decode("utf-8")
    except Exception:
        return True

    # If decoded utf-8 contains replacement char U+FFFD then it's suspicious
    if "\ufffd" in text:
        return True

    # Round-trip heuristic: re-encoding length mismatch
    try:
        rebytes = text.encode("utf-8")
        if len(rebytes) != len(buf):
            return True
    except Exception:
        return True

    return False


def iter_files(root: Path, exts: List[str], recursive: bool) -> List[Path]:
    files = []
    if recursive:
        for p in root.rglob("*"):
            if p.is_file() and (p.suffix.lower() in exts):
                files.append(p)
    else:
        for p in root.iterdir():
            if p.is_file() and (p.suffix.lower() in exts):
                files.append(p)
    return sorted(files)


def make_backup(root: Path, files: List[Path], backup_dir: Path) -> None:
    for f in files:
        rel = f.relative_to(root)
        dest = backup_dir.joinpath(rel)
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(str(f), str(dest))


def convert_file(path: Path, src_encoding: Optional[str]) -> bool:
    try:
        b = path.read_bytes()
        # prefer a specific encoding if provided, else try common GB fallback
        if src_encoding:
            try:
                text = b.decode(src_encoding)
            except Exception:
                # fallback
                text = b.decode("gb18030", errors="replace")
        else:
            text = b.decode("gb18030", errors="replace")

        # write as UTF-8 without BOM
        path.write_text(text, encoding="utf-8", newline="\n")
        return True
    except Exception as e:
        print(f"Failed to convert {path}: {e}")
        return False


def parse_exts(exts_arg: Optional[str]) -> List[str]:
    if not exts_arg:
        return [e.lower() for e in DEFAULT_EXTS]
    parts = [p.strip() for p in exts_arg.split(',') if p.strip()]
    normalized = []
    for p in parts:
        if not p.startswith('.'):
            p = '.' + p
        normalized.append(p.lower())
    return normalized


def main() -> int:
    ap = argparse.ArgumentParser(description="Detect and convert GBK-like files to UTF-8")
    ap.add_argument("--root", default=".", help="Root directory to scan")
    ap.add_argument("--exts", help="Comma separated extensions to include (e.g. .cpp,.h,.txt)")
    ap.add_argument("--recursive", action="store_true", default=True, help="Recursively scan directories")
    ap.add_argument("--no-recursive", dest="recursive", action="store_false", help="Do not scan recursively")
    ap.add_argument("--dry-run", action="store_true", help="Only list candidate files, do not modify")
    ap.add_argument("--backup", action="store_true", default=True, help="Create backup before converting")
    ap.add_argument("--backup-dir", help="Backup directory (defaults to <root>/backup/<timestamp>)")
    ap.add_argument("--min-confidence", type=float, default=0.6, help="Min chardet confidence to trust its encoding")
    args = ap.parse_args()

    root = Path(args.root).resolve()
    if not root.exists() or not root.is_dir():
        print("Root path does not exist or is not a directory:", root)
        return 2

    exts = parse_exts(args.exts)
    files = iter_files(root, exts, args.recursive)
    print(f"Scanning {len(files)} files under {root} ...")

    candidates = []
    for f in files:
        b = f.read_bytes()
        enc, conf = detect_encoding(b, use_chardet=(chardet is not None))
        if enc:
            enc_low = enc.lower()
        else:
            enc_low = None

        is_gbk = probably_gbk(b, enc_low, conf)
        if is_gbk:
            candidates.append((f, enc_low, conf))

    if not candidates:
        print("No GBK-like files detected among chosen extensions.")
        return 0

    print(f"Detected {len(candidates)} candidate file(s):")
    for p, enc, conf in candidates:
        print(f" - {p}   (detected: {enc}, conf={conf:.2f})")

    if args.dry_run:
        print("Dry run: no files modified. Remove --dry-run to perform conversion.")
        return 0

    # prepare backup
    import datetime
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    backup_dir = Path(args.backup_dir) if args.backup_dir else root.joinpath("backup", timestamp)
    if args.backup:
        print(f"Creating backup at {backup_dir} ...")
        backup_dir.mkdir(parents=True, exist_ok=True)
        make_backup(root, [p for p,_,_ in candidates], backup_dir)
        print("Backup completed.")

    # perform conversion
    converted = 0
    failed = 0
    for p, enc, conf in candidates:
        src_enc = None
        if enc and conf >= args.min_confidence:
            src_enc = enc
        else:
            # prefer gb18030 to cover gbk/gb2312
            src_enc = "gb18030"

        ok = convert_file(p, src_enc)
        if ok:
            converted += 1
            print(f"Converted: {p}")
        else:
            failed += 1

    print(f"Done. Converted: {converted}. Failed: {failed}. Backups at: {backup_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
