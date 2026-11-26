#!/usr/bin/env python3
"""Converts b43-asm sources (C preprocessor + b43-asm syntax) to b43-masm.

    b43asm2masm.py [--swap-jand] INPUT [OUTPUT]

Object-like #defines become EQU, function-like ones macro functions (or
statement macros when their body has several statements), #if/#ifdef/#else/
#endif the IF family, #include INCLUDE, %arch .arch, .initvals(x) an
INITVALS block. Statement-terminating semicolons are dropped. --swap-jand
exchanges jand and jnand, for sources written for upstream b43-asm, where
the two are named the other way round.
"""

import re
import sys

IDENT = r"[A-Za-z_][A-Za-z0-9_]*"


def split_code_comment(line, state):
    """Returns (code, comment) of a line; state['block'] tracks /* */."""
    code, comment, i = [], [], 0
    while i < len(line):
        if state["block"]:
            end = line.find("*/", i)
            if end < 0:
                comment.append(line[i:])
                return "".join(code), "".join(comment)
            comment.append(line[i:end + 2])
            i = end + 2
            state["block"] = False
            continue
        if line.startswith("/*", i):
            state["block"] = True
            continue
        if line.startswith("//", i):
            comment.append(line[i:])
            break
        if line[i] in "\"'":
            j = line.find(line[i], i + 1)
            j = len(line) - 1 if j < 0 else j
            code.append(line[i:j + 1])
            i = j + 1
            continue
        code.append(line[i])
        i += 1
    return "".join(code), "".join(comment)


def logical_lines(text):
    """Joins backslash continuations, keeping the number of the first line."""
    buf, first = [], None
    for n, line in enumerate(text.split("\n"), 1):
        if first is None:
            first = n
        if line.rstrip().endswith("\\"):
            buf.append(line.rstrip()[:-1])
            continue
        buf.append(line)
        yield first, buf
        buf, first = [], None
    if buf:
        yield first, buf


def convert_expr(text):
    return re.sub(r"\bdefined\s*\(\s*(" + IDENT + r")\s*\)", r"DEFINED(\1)", text)


def macro_body(name, params, body):
    """A function-like #define as a b43-masm macro."""
    out = ["%s MACRO %s" % (name, ", ".join(params))]
    asserts = []

    def take_assert(m):
        asserts.append(m.group(1))
        return "0"

    body = re.sub(r"%assert\s*\(((?:[^()]|\((?:[^()]|\([^()]*\))*\))*)\)", take_assert, body)
    stmts = [s.strip() for s in body.split(";") if s.strip()]
    for a in asserts:
        out.append("\t.ERRE %s, <%s: assertion failed>" % (a, name))
    if len(stmts) <= 1 and not re.search(IDENT + r"\s*:", body):
        out.append("\tEXITM <%s>" % (stmts[0] if stmts else ""))
    else:
        labels = re.findall(r"(" + IDENT + r")\s*:(?!=)", body)
        if labels:
            out.append("\tLOCAL " + ", ".join(dict.fromkeys(labels)))
        for s in stmts:
            for lab in labels:
                s = re.sub(r"\b%s\s*[+-](?=\s*(,|$))" % re.escape(lab), lab, s)
            m = re.match(r"(" + IDENT + r")\s*:\s*(.*)$", s)
            if m:
                out.append("%s:" % m.group(1))
                s = m.group(2)
            if s:
                out.append("\t" + s)
    out.append("\tENDM")
    return out


def convert(text, swap_jand):
    out = []
    state = {"block": False}
    open_iv = None
    for _, parts in logical_lines(text):
        raw = " ".join(p.strip() if k else p for k, p in enumerate(parts))
        was_block = state["block"]
        code, comment = split_code_comment(raw, state)
        stripped = code.strip()
        indent = code[:len(code) - len(code.lstrip())]
        pp = re.match(r"#\s*(\w+)\s*(.*)$", stripped)
        new = None
        if was_block and not stripped:
            out.append(raw)
            continue
        if pp:
            kw, arg = pp.group(1), pp.group(2).strip()
            if kw == "define":
                fm = re.match(r"(" + IDENT + r")\(([^)]*)\)\s*(.*)$", arg)
                if fm:
                    params = [p.strip() for p in fm.group(2).split(",") if p.strip()]
                    out.extend(macro_body(fm.group(1), params, fm.group(3)))
                    if comment.strip():
                        out.append(comment.strip())
                    continue
                m = re.match(r"(" + IDENT + r")\s*(.*)$", arg)
                new = "%s%s\tEQU %s" % (indent, m.group(1), m.group(2).strip() or "1")
            elif kw in ("ifdef", "ifndef"):
                new = "%s %s" % (kw.upper(), arg)
            elif kw == "if":
                new = "IF %s" % convert_expr(arg)
            elif kw == "elif":
                new = "ELSEIF %s" % convert_expr(arg)
            elif kw in ("else", "endif"):
                new = kw.upper()
            elif kw == "include":
                new = "INCLUDE %s" % arg
            elif kw == "undef":
                new = "PURGE %s" % arg
            else:
                raise SystemExit("unsupported preprocessor directive: #" + kw)
        else:
            s = re.sub(r";\s*$", "", code.rstrip())
            s = re.sub(r":\s*;", ":", s)
            s = re.sub(r"%arch\s+(\d+)", r".arch \1", s)
            if re.search(r"%start\b", s):
                raise SystemExit("%start: place the entry point at address 0 instead")
            if re.match(r"\s*\.text\s*$", s):
                s = "%s ENDS" % open_iv if open_iv else ""
                open_iv = None
            iv = re.match(r"\s*\.initvals\s*\(\s*(" + IDENT + r")\s*\)", s)
            if iv:
                if open_iv:
                    out.append("%s ENDS" % open_iv)
                open_iv = iv.group(1)
                s = "%s INITVALS" % open_iv
            raw = re.match(r"(\s*)@([0-9A-Fa-f]{1,4})\s+(.*)$", s)
            if raw:
                ops = [o.strip() for o in raw.group(3).split(",")]
                ops = ["RAW(0x%s)" % o[1:] if o.startswith("@") else o for o in ops]
                s = "%sINSN 0x%s, %s" % (raw.group(1), raw.group(2), ", ".join(ops))
            if swap_jand:
                s = re.sub(r"\b(jn?and)\b",
                           lambda m: "jnand" if m.group(1) == "jand" else "jand", s)
            new = s
        out.append(new + ((" " + comment) if comment and new.strip() else comment))
    if open_iv:
        out.append("%s ENDS" % open_iv)
    return "\n".join(out)


def main():
    args = sys.argv[1:]
    swap = "--swap-jand" in args
    args = [a for a in args if a != "--swap-jand"]
    if not args or len(args) > 2:
        sys.exit(__doc__)
    with open(args[0]) as f:
        text = convert(f.read(), swap)
    if len(args) == 2:
        with open(args[1], "w") as f:
            f.write(text)
    else:
        sys.stdout.write(text)


if __name__ == "__main__":
    main()
