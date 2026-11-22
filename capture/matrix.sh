#!/bin/sh
# matrix.sh DEBUGFS LO HI [OUT]: on the device, the full-address matrix. Every MMIO address 0x400-0x7FE and
# 0x800-0x9FE (SPR 0x000-0x2FF: 0x263, 0x265, 0x271 ... live above 0x7FE) except the
# data ports (0x422, 0x432, 0x540, 0x542, 0x560, 0x562) and the TX status FIFO (0x518-0x51E) is sampled together
# with FRAMELEN (0x418) by pdtrace in its history mode: 3000 rows, 1500 before the moment FRAMELEN reaches 6 (the
# mark) and 1500 after it. A capture is kept when FRAMELEN peaks in [LO, HI] after the mark (one frame of the
# traffic you send), up to 8 tries per address. Prints "addr ADDR PEAK" (PEAK 0: none caught) and the rows:
#   index tsf-low phydebug value framelen
# DEBUGFS is the phy's directory, e.g. /sys/kernel/debug/b43_next/phy0 (needs b43-next-capture.patch).
D=$1; LO=$2; HI=$3
for a in $(seq $((0x400)) 2 $((0x7FE))) $(seq $((0x800)) 2 $((0x9FE))); do
	h=$(printf 0x%03X $a)
	case $h in 0x422|0x432|0x518|0x51A|0x51C|0x51E|0x540|0x542|0x560|0x562) continue;; esac
	got=0
	for t in 1 2 3 4 5 6 7 8; do
		echo "3000 0x60000006 $h 0x0418" > $D/pdtrace 2>/dev/null
		cat $D/pdtrace > /tmp/matrix-cur.txt
		m=$(awk '$1 != "trig" && $1 >= 1500 { v = sprintf("%d", "0x" $5); if (v + 0 > m) m = v + 0 } END { print m + 0 }' /tmp/matrix-cur.txt)
		if [ $m -ge $LO ] && [ $m -le $HI ]; then got=$m; break; fi
	done
	echo "addr $h $got"; [ $got -gt 0 ] && grep -v trig /tmp/matrix-cur.txt
done
