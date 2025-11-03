#!/usr/bin/env python3
import sys
import re
from pathlib import Path

"""
Arduino-style formatter (lightweight)

Goals for future changes (do not retro-format existing code unless explicitly run):
- Indent with 2 spaces (convert leading tabs to two spaces).
- Attach opening braces to the same line for control statements and function definitions (K&R / "attach" style):
    if (cond) {
      ...
    } else {
      ...
    }
- Ensure a single space before the opening brace when attached.

This script is intentionally conservative and only performs minimal, local rewrites to
help keep new edits aligned with Arduino's default Auto Format style.
"""

CONTROL_RE = re.compile(r'^(?P<indent>\s*)(?P<kw>(?:else\s+if|else|if|for|while|switch)\b)(?P<rest>.*)$')
ONLY_BRACE_RE = re.compile(r'^\s*\{\s*}\s*$|^\s*\{\s*$')
OPEN_BRACE_LINE_RE = re.compile(r'^\s*\{\s*$')

def normalize_leading_tabs(line: str) -> str:
    m = re.match(r'^(\t+)', line)
    if not m:
        return line
    tabs = len(m.group(1))
    return ('  ' * tabs) + line[len(m.group(1)):]

def is_control_line(line: str) -> bool:
    if not line.strip():
        return False
    return CONTROL_RE.match(line) is not None

def attach_brace_to_line(base_line: str) -> str:
    """Ensure exactly one space before an opening brace if present inline,
    or return the line unchanged (brace may be added by caller)."""
    # If there's already a brace, ensure one space before it
    no_comment = re.split(r'//', base_line, 1)[0]
    if '{' in no_comment:
        # Collapse spaces before '{' to a single space
        return re.sub(r'\s*\{', ' {', base_line)
    return base_line

def is_function_signature(line: str) -> bool:
    s = line.strip()
    if not s or s.startswith('#'):
        return False
    # Heuristic: not a control keyword, ends with ')' and not a declaration ending with ';'
    if CONTROL_RE.match(line):
        return False
    if s.endswith(')') and not s.endswith(');'):
        return True
    return False

def format_file(path: Path) -> bool:
    src = path.read_text(encoding='utf-8')
    lines = src.splitlines()
    out = []
    i = 0
    n = len(lines)
    while i < n:
        line = normalize_leading_tabs(lines[i])

        # Attach brace for control lines where the next non-empty line is a standalone '{'
        if is_control_line(line):
            # Look ahead to find next non-empty line index
            j = i + 1
            # Skip blank lines between statement and brace
            blanks = 0
            while j < n and lines[j].strip() == '':
                blanks += 1
                j += 1
            if j < n and OPEN_BRACE_LINE_RE.match(lines[j]):
                # Attach the brace to the control line with one space
                line = attach_brace_to_line(line.rstrip())
                if '{' not in line:
                    line = line.rstrip() + ' {'
                out.append(line)
                # Skip the blank lines and the brace line
                i = j + 1
                continue
            else:
                # If brace is inline already, normalize spacing before it
                line = attach_brace_to_line(line)
                out.append(line)
                i += 1
                continue

        # Attach brace for function signatures
        if is_function_signature(line):
            j = i + 1
            blanks = 0
            while j < n and lines[j].strip() == '':
                blanks += 1
                j += 1
            if j < n and OPEN_BRACE_LINE_RE.match(lines[j]):
                # Attach brace to function signature
                base = line.rstrip()
                if '{' not in base:
                    line = base + ' {'
                else:
                    line = attach_brace_to_line(base)
                out.append(line)
                i = j + 1
                continue

        # Default: keep line (with normalized leading tabs)
        out.append(line)
        i += 1

    result = '\n'.join(out) + '\n'
    if result != src:
        path.write_text(result, encoding='utf-8')
        return True
    return False

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print('Usage: format_c_style.py <file> [<file>...]')
        sys.exit(2)
    changed = []
    for p in sys.argv[1:]:
        path = Path(p)
        if not path.exists():
            print(f'File not found: {p}', file=sys.stderr)
            continue
        ok = format_file(path)
        if ok:
            changed.append(p)
    if changed:
        print('Formatted (Arduino style attach braces):', ', '.join(changed))
        sys.exit(0)
    else:
        print('No changes')
        sys.exit(0)
