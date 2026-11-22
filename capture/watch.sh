#!/bin/sh
# watch.sh DEBUGFS GROUPS US: on the device, regwatch over each line of the file GROUPS (MMIO addresses, e.g.
# "0x418 0x400 0x408 ..."; keep FRAMELEN 0x418 in every line as the time reference and about 6 others, so a pass
# stays near 10 us), US microseconds per line. Prints "== group ADDRS", the header, then one row per change:
#   tsf-us-since-the-group's-start address value       (the first rows: every address's starting value at 0;
#                                                        address ffff: a gap where interrupts were let through)
D=$1; G=$2; US=$3
while read g; do
	echo "== group $g"
	echo "$US $g" > $D/regwatch
	while true; do c=$(cat $D/regwatch); [ -z "$c" ] && break; echo "$c"; done
done < $G
