#!/bin/sh
# Regression tests for b43-masm.
set -u
here=$(cd "$(dirname "$0")" && pwd)
top=$here/..
masm=$top/b43-masm
asmdir=$top/../assembler
interp=$top/../interpreter
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT
fail=0

pass() { printf 'ok    %s\n' "$1"; }
bad() { printf 'FAIL  %s\n' "$1"; fail=1; }

for f in "$here"/errors/*.asm; do
	name=errors/$(basename "$f" .asm)
	arch=$(sed -n 's/^; arch: //p' "$f")
	expect=$(sed -n 's/^; expect: //p' "$f")
	out=$("$masm" "$f" "$tmp/x.fw" -a "$arch" 2>&1)
	if [ $? -ne 0 ] && printf '%s' "$out" | grep -qF -- "$expect"; then
		pass "$name"
	else
		bad "$name: $out"
	fi
done

for f in "$here"/ok/*.asm; do
	for arch in 5 15; do
		name="ok/$(basename "$f" .asm) arch $arch"
		if out=$("$masm" "$f" "$tmp/f.fw" -a "$arch" -W -m "$tmp/f.map" \
			 -l "$tmp/f.lst" 2>&1); then
			pass "$name"
		else
			bad "$name: $out"
		fi
	done
done

if [ -x "$asmdir/b43-asm.bin" ] && command -v python3 >/dev/null; then
	for arch in 5 15; do
		for seed in 1 2 3; do
			python3 "$here/diffgen.py" "$arch" "$seed" 3000 "$tmp/o.asm" "$tmp/n.asm"
			"$masm" "$tmp/n.asm" "$tmp/n.fw" -f raw-le32 >/dev/null 2>&1
			if ! (cd "$asmdir" && ./b43-asm "$tmp/o.asm" "$tmp/o.fw" -f raw-le32 >/dev/null 2>&1); then
				bad "b43-asm fails, arch $arch seed $seed"
			elif cmp -s "$tmp/o.fw" "$tmp/n.fw"; then
				pass "same encoding as b43-asm, arch $arch seed $seed"
			else
				bad "encoding differs from b43-asm, arch $arch seed $seed"
			fi
		done
	done
else
	printf 'skip  encoding comparison (build ../assembler first)\n'
fi

if [ -f "$interp/psm5.py" ] && command -v python3 >/dev/null; then
	for f in "$here"/run/*.asm; do
		expect=$(sed -n 's/^; expect-shm: //p' "$f")
		for arch in 5 15; do
			name="run/$(basename "$f" .asm) arch $arch"
			if ! "$masm" "$f" "$tmp/p.bin" -a "$arch" -f raw-be32 >/dev/null 2>&1; then
				bad "$name: does not assemble"
				continue
			fi
			if (cd "$interp" && python3 - "$arch" "$tmp/p.bin" $expect) <<'PY'
import importlib, sys
mod = importlib.import_module("psm" + sys.argv[1])
m = mod.PSM(mod.load_program(sys.argv[2]))
try:
    m.run(start=0, max_steps=10000)
except Exception:
    pass
for kv in sys.argv[3:]:
    k, v = (int(x, 0) for x in kv.split("="))
    if m.shm[k] != v:
        sys.exit(1)
PY
			then
				pass "$name"
			else
				bad "$name"
			fi
		done
	done
fi

exit $fail
