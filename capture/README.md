# capture: register traces of the real chip, for vchip

vchip models the chip from what the ucode does and what the hardware answers. When the two disagree, for example a
frame that streams on the chip but is held in the model, you need the chip's own timeline. This directory has the tools
for that: a b43-next debugfs patch and the scripts that drive it.

The method is black-box. Run a ucode, the vendor's or your own, while traffic you control arrives. Read what the
registers do around each frame, and compare.

## The driver patch

`b43-next-capture.patch` (against b43-next main) adds two debugfs files to each phy's directory,
`/sys/kernel/debug/b43_next/phyN/`. Build the module with `B43_NEXT_DEBUG=y`.

- **`pdtrace`**: sample two MMIO addresses back to back, with interrupts off. Write `N MASK RA RB`. Each row read
  back is `index tsf-low phydebug value-of-RA value-of-RB`, printed only when something changes.
  - MASK bit 30: start when RB, having differed from MASK's low 16 bits, equals them. For example, RB = 0x418
    (FRAMELEN) and value 6 is the mark, 6 bytes into a frame.
  - MASK bits 30 and 29 together (history mode): the same trigger, but the N rows run from N/2 samples before it to
    N/2 after it.
  - With history, one sample takes about 4.5 µs on a BCM4352 behind PCIe.
- **`regwatch`**: log every change of a list of addresses. Write `US all` or `US 0xADDR 0xADDR ...`.
  - The addresses are read in passes for US microseconds, with interrupts off in bursts of at most 3 ms.
  - Each change is stored as `tsf-us address value`. Every address's starting value comes first, at time 0. Address
    `ffff` marks a gap where interrupts were let through.
  - Read the file repeatedly, one chunk per read, until it returns nothing. A new write rewinds.
  - Each read costs about 1.1 µs over PCIe, so a pass over 7 addresses takes about 10 µs. Watch about 7 addresses at
    a time.

## The procedure

1. **Traffic.** Load the ucode and associate a station. Send frames of one known size, such as pings every 30 ms.
   Their FRAMELEN peak (MPDU plus the 6-byte PLCP) singles them out from everyone else's traffic.
2. **The matrix.** `matrix.sh DEBUGFS LO HI > matrix.txt` covers every address in 0x400-0x7FE and 0x800-0x9FE
   (SPR 0x000-0x2FF), except the data ports
   and the TX status FIFO, whose reads would consume data. Each address gets 1500 samples before the mark and 1500
   after, on one frame with its FRAMELEN peak in [LO, HI]. It takes about 3 hours per ucode.
3. **The watch list.** `watchlist.py matrix.txt > groups.txt` lists the addresses that move around the frame, leaves
   out free-running timers, and groups them six per line with FRAMELEN first.
4. **The timelines.** `watch.sh DEBUGFS groups.txt 300000 > watch.txt` gives every change of every listed address
   with its time, about 10 µs per pass.
5. **Reading.** `frames.py watch.txt GROUP [N]` prints N frames of one group, every event relative to the mark.
   `matrixcmp.py vendor.txt yours.txt` lists the addresses whose behaviour differs between two matrices.
6. **Publishing.** Use `redact.py aa:bb:cc:dd:ee:ff ... < in > out` before sharing. The address-match registers
   (SPR 0x0E8-0x0ED) and shared memory hold the station and AP MAC addresses.

Run the same steps under each ucode you compare. A frame size the working ucode handles is the cleanest reference.

## Example

784.2 on a BCM4352 (core rev 42), one 600-byte ping frame (FRAMELEN peak 0x2A0), from `frames.py` (times in µs
from the mark):

    -10:418=0002 -10:408=0800 +0:418=0006 +11:418=0052 +11:408=0822 +21:418=00a6 ... +72:418=029e
    +72:408=0821 +72:40a=0001 +72:412=2c26 +72:414=4203 +82:418=02a0 +82:408=0800 +82:40a=0000

The FIFOCTL1 (0x408) write that reads back 0x0822 comes 11 µs after the mark, and FRAMELEN climbs at air rate.
The push (0x0821) comes at the frame's end, and 0x0800 follows 10 µs later. The captures of that chip are in
aleferri/b43-next-traces.
