"""Check that (), [] and {} are balanced in LispBM source files.

Skips ; comments, "strings" (with backslash escapes) and character
literals such as \\#( so brackets inside them are not counted.

Usage: python tools/check_lisp_parens.py [file ...]   (default: vesc/*.lbm)
Exit status is 0 when every file is balanced, 1 otherwise.
"""
import glob
import sys

PAIRS = {")": "(", "]": "[", "}": "{"}


def check(path):
    with open(path, encoding="utf-8") as f:
        text = f.read()

    stack = []  # (bracket, line, column)
    line, col = 1, 0
    in_string = in_comment = False
    i = 0
    while i < len(text):
        ch = text[i]
        col += 1
        if ch == "\n":
            line, col = line + 1, 0
            in_comment = False
        elif in_comment:
            pass
        elif in_string:
            if ch == "\\":
                i += 1  # skip the escaped character
                col += 1
            elif ch == '"':
                in_string = False
        elif ch == ";":
            in_comment = True
        elif ch == '"':
            in_string = True
        elif text.startswith("\\#", i):
            i += 2  # character literal: skip \# and the character after it
            col += 2
        elif ch in "([{":
            stack.append((ch, line, col))
        elif ch in PAIRS:
            if not stack:
                return f"{path}:{line}:{col}: unmatched '{ch}'"
            opener, o_line, o_col = stack.pop()
            if opener != PAIRS[ch]:
                return f"{path}:{line}:{col}: '{ch}' closes '{opener}' opened at {o_line}:{o_col}"
        i += 1

    if in_string:
        return f"{path}: unterminated string"
    if stack:
        opener, o_line, o_col = stack[-1]
        return f"{path}:{o_line}:{o_col}: '{opener}' is never closed ({len(stack)} open)"
    return None


def main(argv):
    paths = argv or sorted(p.replace("\\", "/") for p in glob.glob("vesc/*.lbm"))
    if not paths:
        print("no .lbm files found")
        return 1
    failed = False
    for path in paths:
        error = check(path)
        if error:
            print(error)
            failed = True
        else:
            print(f"{path}: OK")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
