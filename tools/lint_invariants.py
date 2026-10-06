"""Check repository files for patterns that break project rules.

Rules:
  lisp-foc-beep        `foc-beep` in LispBM: blocks the calling thread.
  lisp-handbrake       `set-handbrake` / `canset-handbrake`: holds current at standstill.
  lisp-canset-offdelay `canset-current` with 3 arguments: VESC 7.00 decodes the
                       off-delay and current fields over CAN in the wrong order.
  cpp-delay            `delay(` in ESP32 control code (src/, lib/KukirinDisplay, lib/VESCBridge).
  cpp-heap             heap allocation in ESP32 control code (new, malloc, String, std containers).
  log-brackets         square brackets inside runtime log strings.
  emoji                emoji characters in any text file.
  numbered-heading     Markdown headings that start with a number or a letter label ("## 1.", "### A.").
  credential           strings that look like passwords, keys or tokens.
  personal-path        absolute paths into a user's home folder.

Comments are ignored for the code rules. A line can opt out of one rule with a
comment containing `lint-allow: <rule>`; use it only with a reason next to it.

Usage: python tools/lint_invariants.py [--root PATH] [file ...]
  (default: all tracked files outside third_party/ in the repository, or every file under --root)
Exit status is 0 when nothing is found, 1 otherwise.
"""
import os
import re
import subprocess
import sys

CONTROL_CODE_DIRS = ("src/", "lib/KukirinDisplay/", "lib/VESCBridge/")
CPP_EXT = (".cpp", ".h", ".hpp", ".c")
TEXT_EXT = CPP_EXT + (".lbm", ".py", ".md", ".ini", ".json", ".txt", ".yml", ".yaml", ".csv")

EMOJI = re.compile("[\U0001F000-\U0001FAFF☀-➿⬀-⯿️]")
NUMBERED_HEADING = re.compile(r"^#{1,6}\s+(\d+(\.\d+)*[.)]?|[A-Z][.)])\s")
CREDENTIAL = re.compile(
    r"(?i)\b(pass(word|wd)?|pwd|secret|api[_-]?key|access[_-]?token|auth[_-]?token|wifi[_-]?pass\w*)\b"
    r"\s*[:=]?\s*\(?\s*\"[^\"\s]{4,}\""
    r"|-----BEGIN [A-Z ]*PRIVATE KEY-----"
    r"|\bghp_[A-Za-z0-9]{36}\b|\bgithub_pat_[A-Za-z0-9_]{40,}\b|\bAKIA[0-9A-Z]{16}\b|\bsk-[A-Za-z0-9]{32,}\b")
PERSONAL_PATH = re.compile(r"(?i)\b[A-Z]:[\\/]+Users[\\/]+[^\\/\s\"']+|/home/[a-z_][\w-]*/|/Users/[A-Za-z][\w-]*/")
CPP_DELAY = re.compile(r"\bdelay\s*\(")
CPP_HEAP = re.compile(r"\bnew\s+[A-Za-z_]|\b(malloc|calloc|realloc)\s*\(|\bString\b|\bstd::(vector|string|map|list|deque)\b")
CPP_LOG_CALL = re.compile(r"\b(Serial\.(print|println|printf)|printf|send_debug_logf?|queue_vesc_log|log_[a-z]+)\s*\(")
STRING_LITERAL = re.compile(r'"((?:[^"\\]|\\.)*)"')


def tracked_files(root):
    try:
        out = subprocess.run(["git", "ls-files"], cwd=root, capture_output=True, text=True, check=True).stdout
        files = out.splitlines()
    except (OSError, subprocess.CalledProcessError):
        files = []
        for dirpath, dirnames, filenames in os.walk(root):
            dirnames[:] = [d for d in dirnames if not d.startswith(".")]
            files += [os.path.relpath(os.path.join(dirpath, f), root).replace("\\", "/") for f in filenames]
    return [f for f in files if not f.startswith("third_party/")]


def strip_cpp_comments(text):
    """Blank out // and /* */ comments, keeping strings and line numbers intact."""
    out, i, n = [], 0, len(text)
    while i < n:
        if text.startswith("//", i):
            j = text.find("\n", i)
            j = n if j < 0 else j
            out.append(" " * (j - i))
            i = j
        elif text.startswith("/*", i):
            j = text.find("*/", i + 2)
            j = n if j < 0 else j + 2
            out.append(re.sub(r"[^\n]", " ", text[i:j]))
            i = j
        elif text[i] in "\"'":
            q, j = text[i], i + 1
            while j < n and text[j] != q and text[j] != "\n":
                j += 2 if text[j] == "\\" else 1
            out.append(text[i:j + 1])
            i = j + 1
        else:
            out.append(text[i])
            i += 1
    return "".join(out)


def strip_lisp_comments(text):
    """Blank out ; comments outside strings."""
    lines = []
    for line in text.split("\n"):
        in_str, cut = False, len(line)
        for k, ch in enumerate(line):
            if ch == '"' and (k == 0 or line[k - 1] != "\\"):
                in_str = not in_str
            elif ch == ";" and not in_str:
                cut = k
                break
        lines.append(line[:cut] + " " * (len(line) - cut))
    return "\n".join(lines)


def lisp_forms(code, head):
    """Yield (line_number, argument_count) for each `(head ...)` form."""
    for m in re.finditer(r"\(" + re.escape(head) + r"(?=[\s)])", code):
        depth, args, in_token, i = 0, 0, False, m.end()
        while i < len(code):
            ch = code[i]
            if ch == '"':
                j = i + 1
                while j < len(code) and code[j] != '"':
                    j += 2 if code[j] == "\\" else 1
                if depth == 0:
                    args += 1
                i, in_token = j + 1, False
                continue
            if ch in "([{":
                if depth == 0:
                    args += 1
                depth += 1
                in_token = False
            elif ch in ")]}":
                if depth == 0:
                    break
                depth -= 1
            elif ch.isspace():
                in_token = False
            elif depth == 0 and not in_token:
                args += 1
                in_token = True
            i += 1
        yield code.count("\n", 0, m.start()) + 1, args


def allowed(raw_lines, line_no, rule):
    return f"lint-allow: {rule}" in raw_lines[line_no - 1]


def check_file(root, rel):
    path = os.path.join(root, rel)
    if not rel.endswith(TEXT_EXT) or not os.path.isfile(path):
        return []
    try:
        with open(path, encoding="utf-8") as f:
            text = f.read()
    except UnicodeDecodeError:
        return [(rel, 0, "encoding", "file is not valid UTF-8")]
    raw = text.split("\n")
    found = []

    def report(line_no, rule, message):
        if not allowed(raw, line_no, rule):
            found.append((rel, line_no, rule, message))

    for no, line in enumerate(raw, 1):
        if EMOJI.search(line):
            report(no, "emoji", "emoji character")
        if CREDENTIAL.search(line):
            report(no, "credential", "looks like a password, key or token")
        if PERSONAL_PATH.search(line):
            report(no, "personal-path", "absolute path into a home folder")
        if rel.endswith(".md") and NUMBERED_HEADING.match(line):
            report(no, "numbered-heading", "heading starts with a number or letter label")

    if rel.endswith(".lbm"):
        code = strip_lisp_comments(text)
        for no, line in enumerate(code.split("\n"), 1):
            if re.search(r"\(foc-beep\b", line):
                report(no, "lisp-foc-beep", "foc-beep blocks the calling thread")
            if re.search(r"\((can)?set-handbrake\b", line):
                report(no, "lisp-handbrake", "handbrake holds current at standstill")
            m = re.search(r"\(puts\s+\"([^\"]*)\"", line)
            if m and re.search(r"[\[\]]", m.group(1)):
                report(no, "log-brackets", "square brackets in a log string")
        for no, args in lisp_forms(code, "canset-current"):
            if args >= 3:
                report(no, "lisp-canset-offdelay", "canset-current with an off-delay argument")

    if rel.endswith(CPP_EXT):
        code = strip_cpp_comments(text)
        control = rel.startswith(CONTROL_CODE_DIRS)
        for no, line in enumerate(code.split("\n"), 1):
            if control and CPP_DELAY.search(line):
                report(no, "cpp-delay", "delay() in control code")
            if control and CPP_HEAP.search(line):
                report(no, "cpp-heap", "heap allocation in control code")
            if CPP_LOG_CALL.search(line):
                for literal in STRING_LITERAL.findall(line):
                    if re.search(r"[\[\]]", literal):
                        report(no, "log-brackets", "square brackets in a log string")
    return found


def main(argv):
    root = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
    if argv[:1] == ["--root"]:
        root, argv = os.path.abspath(argv[1]), argv[2:]
    files = [os.path.relpath(os.path.abspath(f), root).replace("\\", "/") for f in argv] if argv else tracked_files(root)
    findings = []
    for rel in sorted(files):
        findings += check_file(root, rel)
    findings.sort(key=lambda f: (f[0], f[1]))
    for rel, no, rule, message in findings:
        print(f"{rel}:{no}: {rule}: {message}")
    print(f"{len(findings)} finding(s)" if findings else f"no findings in {len(files)} files")
    return 1 if findings else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
