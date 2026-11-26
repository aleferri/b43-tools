qemu: a Broadcom board as a PCI device
=======================================

`bcm_board.c` is a QEMU PCI device with a board's IDs and BAR0 size that
knows nothing of the chip: BAR0, the vendor part of the configuration space
and the passing of the virtual clock go to a board served on a UNIX socket
(`interpreter/boardd.py`), and the board's interrupt line comes back with
every reply. A driver in the guest, b43 or wl, runs on the board as on the
card. The protocol is at the head of `bcm_board.c`.

It is for QEMU 8.2.2, Ubuntu's source (`qemu_8.2.2+ds.orig.tar.xz`):

    tar xf qemu_8.2.2+ds.orig.tar.xz && cd qemu-8.2.2
    patch -p1 < qemu-8.2.2.patch
    cp bcm_board.c hw/misc/
    mkdir build && cd build
    ../configure --target-list=x86_64-softmmu --enable-fdt=system \
        --disable-install-blobs
    ninja qemu-system-x86_64

The patch adds the device's Kconfig and meson entries, and keeps the build
from unpacking the EDK2 images, which the `+ds` tarball does not have.

    ./boardd.py --socket /tmp/board.sock --trace attach.txt --bar-size 0x8000 &
    qemu-system-x86_64 ... -device bcm-board,socket=/tmp/board.sock,\
        device-id=0x4365,subsys-vendor=0x103c,subsys-id=0x2230,revision=1,\
        bar-size=0x8000

`tick-us` (default 1000) is how much virtual time passes before the board is
told, between accesses or on a timer while the guest waits.
