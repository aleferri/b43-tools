#!/bin/sh

basedir="$(dirname "$(realpath "$0")")"

srcdir="$basedir/.." # git repos root

die() { echo "$*"; exit 1; }

# Import the makerelease.lib
# https://bues.ch/cgit/misc.git/tree/makerelease.lib
for path in $(echo "$PATH" | tr ':' ' '); do
	[ -f "$MAKERELEASE_LIB" ] && break
	MAKERELEASE_LIB="$path/makerelease.lib"
done
[ -f "$MAKERELEASE_LIB" ] && . "$MAKERELEASE_LIB" || die "makerelease.lib not found."

hook_get_version()
{
	local file="$1/Makefile"
	version="$(cat "$file" | grep -e VERSION | head -n1 | cut -d' ' -f3)"
}

project=b43-fwcutter
srcsubdir=fwcutter
makerelease "$@"
