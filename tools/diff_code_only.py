"""Show whether changes since a git ref touch code or only comments and whitespace.

For each changed C/C++ and LispBM file, both versions are reduced to their code
(comments removed, whitespace collapsed) and compared. A comment-only change
reports "comments only"; anything else reports "CODE CHANGED" and exits 1.

Usage: python tools/diff_code_only.py [ref]   (default ref: HEAD)
"""
import os
import re
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from lint_invariants import strip_cpp_comments, strip_lisp_comments  # noqa: E402

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
CPP_EXT = (".cpp", ".h", ".hpp", ".c")


def git(*args):
    return subprocess.run(["git", *args], cwd=ROOT, capture_output=True, text=True, check=True).stdout


def code_only(path, text):
    if path.endswith(".lbm"):
        text = strip_lisp_comments(text)
    else:
        text = strip_cpp_comments(text)
    # Collapse whitespace between tokens, but keep string literals exactly
    parts = re.split(r'("(?:[^"\\\n]|\\.)*")', text)
    return "".join(p if p.startswith('"') else " ".join(p.split()) + " " for p in parts).strip()


def main(argv):
    ref = argv[0] if argv else "HEAD"
    changed = [f for f in git("diff", "--name-only", ref, "--").splitlines()
               if f.endswith(CPP_EXT + (".lbm",)) and not f.startswith("third_party/")]
    if not changed:
        print(f"no C/C++ or LispBM changes since {ref}")
        return 0
    code_changed = False
    for path in changed:
        try:
            old = git("show", f"{ref}:{path}")
        except subprocess.CalledProcessError:
            old = ""
        full = os.path.join(ROOT, path)
        new = open(full, encoding="utf-8").read() if os.path.exists(full) else ""
        same = code_only(path, old) == code_only(path, new)
        code_changed |= not same
        print(f"{'comments only' if same else 'CODE CHANGED '}  {path}")
    return 1 if code_changed else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
