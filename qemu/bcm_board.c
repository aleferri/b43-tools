/*
 * A Broadcom wireless board for QEMU, served by interpreter/boardd.py.
 *
 * The PCI function has the board's IDs and a BAR0 of its size, and knows
 * nothing of the chip: BAR0 accesses, accesses to the vendor part of the
 * configuration space (0x40-0xff: the BAR0 windows, SPROM control, GPIO...)
 * and the passing of the virtual clock go to the board over a UNIX socket,
 * one request per line, one reply per request; every reply carries the
 * level of the board's interrupt line.
 *
 *   W <off> <val> <bits>    BAR0 write         -> K <irq>
 *   R <off> <bits>          BAR0 read          -> V <val> <irq>
 *   P <off> <val> <bits>    config write       -> K <irq>
 *   p <off> <bits>          config read        -> V <val> <irq>
 *   T <us>                  time passes        -> K <irq>
 *   Q                       end                -> K <irq>
 *
 * Numbers in hex. Time goes before an access when tick-us of virtual time
 * have passed since the last T, and on a timer every tick-us, so that the
 * board runs while the guest waits for its interrupt.
 *
 *   -device bcm-board,socket=board.sock,device-id=0x4365,
 *           subsys-vendor=0x103c,subsys-id=0x2230,revision=1,bar-size=0x8000
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 */

#include "qemu/osdep.h"
#include "hw/pci/pci_device.h"
#include "hw/qdev-properties.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "qemu/module.h"
#include "qemu/sockets.h"
#include "qemu/timer.h"
#include "qom/object.h"

#define TYPE_BCM_BOARD "bcm-board"
OBJECT_DECLARE_SIMPLE_TYPE(BcmBoardState, BCM_BOARD)

#define VENDOR_CONFIG	0x40

struct BcmBoardState {
    PCIDevice parent_obj;
    MemoryRegion bar0;
    QEMUTimer *tick;

    char *socket;
    uint16_t device_id;
    uint16_t subsys_vendor;
    uint16_t subsys_id;
    uint8_t revision;
    uint32_t bar_size;
    uint32_t tick_us;

    FILE *rx, *tx;
    int64_t last_us;
};

/* Sends one request and reads its reply; the reply's value, 0 for a K. */
static uint32_t board_request(BcmBoardState *s, const char *fmt, ...)
    G_GNUC_PRINTF(2, 3);

static uint32_t board_request(BcmBoardState *s, const char *fmt, ...)
{
    char line[64];
    unsigned val = 0, irq = 0;
    va_list ap;

    if (!s->tx) {
        return ~0u;
    }
    va_start(ap, fmt);
    vfprintf(s->tx, fmt, ap);
    va_end(ap);
    fflush(s->tx);
    if (!fgets(line, sizeof(line), s->rx) ||
        !((line[0] == 'V' && sscanf(line, "V %x %x", &val, &irq) == 2) ||
          (line[0] == 'K' && sscanf(line, "K %x", &irq) == 1))) {
        error_report("bcm-board: the board stopped answering");
        fclose(s->tx);
        fclose(s->rx);
        s->tx = s->rx = NULL;
        return ~0u;
    }
    pci_set_irq(PCI_DEVICE(s), irq != 0);
    return val;
}

static void board_time(BcmBoardState *s)
{
    int64_t now = qemu_clock_get_us(QEMU_CLOCK_VIRTUAL);

    if (now - s->last_us >= s->tick_us) {
        board_request(s, "T %" PRIx64 "\n", now - s->last_us);
        s->last_us = now;
    }
}

static void board_tick(void *opaque)
{
    BcmBoardState *s = opaque;

    board_time(s);
    timer_mod(s->tick, qemu_clock_get_us(QEMU_CLOCK_VIRTUAL) + s->tick_us);
}

static uint64_t bar0_read(void *opaque, hwaddr addr, unsigned size)
{
    BcmBoardState *s = opaque;

    board_time(s);
    return board_request(s, "R %" HWADDR_PRIx " %x\n", addr, size * 8);
}

static void bar0_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    BcmBoardState *s = opaque;

    board_time(s);
    board_request(s, "W %" HWADDR_PRIx " %" PRIx64 " %x\n", addr, val,
                  size * 8);
}

static const MemoryRegionOps bar0_ops = {
    .read = bar0_read,
    .write = bar0_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl = { .min_access_size = 1, .max_access_size = 4 },
};

static uint32_t board_config_read(PCIDevice *pdev, uint32_t addr, int len)
{
    BcmBoardState *s = BCM_BOARD(pdev);

    if (addr < VENDOR_CONFIG) {
        return pci_default_read_config(pdev, addr, len);
    }
    return board_request(s, "p %x %x\n", addr, len * 8);
}

static void board_config_write(PCIDevice *pdev, uint32_t addr, uint32_t val,
                               int len)
{
    BcmBoardState *s = BCM_BOARD(pdev);

    if (addr < VENDOR_CONFIG) {
        pci_default_write_config(pdev, addr, val, len);
        return;
    }
    board_request(s, "P %x %x %x\n", addr, val, len * 8);
}

static void board_realize(PCIDevice *pdev, Error **errp)
{
    BcmBoardState *s = BCM_BOARD(pdev);
    uint8_t *c = pdev->config;
    int fd;

    if (!s->socket) {
        error_setg(errp, "bcm-board: socket= is required");
        return;
    }
    fd = unix_connect(s->socket, errp);
    if (fd < 0) {
        return;
    }
    s->rx = fdopen(fd, "r");
    s->tx = fdopen(dup(fd), "w");
    if (!s->rx || !s->tx) {
        error_setg_errno(errp, errno, "bcm-board: %s", s->socket);
        return;
    }

    pci_config_set_device_id(c, s->device_id);
    pci_config_set_revision(c, s->revision);
    pci_set_word(c + PCI_SUBSYSTEM_VENDOR_ID, s->subsys_vendor);
    pci_set_word(c + PCI_SUBSYSTEM_ID, s->subsys_id);
    c[PCI_INTERRUPT_PIN] = 1;

    memory_region_init_io(&s->bar0, OBJECT(s), &bar0_ops, s,
                          "bcm-board-bar0", s->bar_size);
    pci_register_bar(pdev, 0, PCI_BASE_ADDRESS_SPACE_MEMORY |
                     PCI_BASE_ADDRESS_MEM_TYPE_64, &s->bar0);

    s->last_us = qemu_clock_get_us(QEMU_CLOCK_VIRTUAL);
    s->tick = timer_new_us(QEMU_CLOCK_VIRTUAL, board_tick, s);
    timer_mod(s->tick, s->last_us + s->tick_us);
}

static void board_exit(PCIDevice *pdev)
{
    BcmBoardState *s = BCM_BOARD(pdev);

    timer_free(s->tick);
    board_request(s, "Q\n");
    if (s->tx) {
        fclose(s->tx);
        fclose(s->rx);
    }
}

static Property board_props[] = {
    DEFINE_PROP_STRING("socket", BcmBoardState, socket),
    DEFINE_PROP_UINT16("device-id", BcmBoardState, device_id, 0x4727),
    DEFINE_PROP_UINT16("subsys-vendor", BcmBoardState, subsys_vendor, 0x14e4),
    DEFINE_PROP_UINT16("subsys-id", BcmBoardState, subsys_id, 0),
    DEFINE_PROP_UINT8("revision", BcmBoardState, revision, 1),
    DEFINE_PROP_UINT32("bar-size", BcmBoardState, bar_size, 0x4000),
    DEFINE_PROP_UINT32("tick-us", BcmBoardState, tick_us, 1000),
    DEFINE_PROP_END_OF_LIST(),
};

static void board_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    k->realize = board_realize;
    k->exit = board_exit;
    k->config_read = board_config_read;
    k->config_write = board_config_write;
    k->vendor_id = 0x14e4;
    k->class_id = PCI_CLASS_NETWORK_OTHER;
    dc->desc = "Broadcom wireless board served by boardd.py";
    device_class_set_props(dc, board_props);
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
}

static const TypeInfo board_info = {
    .name = TYPE_BCM_BOARD,
    .parent = TYPE_PCI_DEVICE,
    .instance_size = sizeof(BcmBoardState),
    .class_init = board_class_init,
    .interfaces = (InterfaceInfo[]) {
        { INTERFACE_CONVENTIONAL_PCI_DEVICE },
        { },
    },
};

static void board_register_types(void)
{
    type_register_static(&board_info);
}

type_init(board_register_types)
