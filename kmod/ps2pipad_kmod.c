#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <asm/fiq.h>

static inline u32 ccnt_read(void) {
    u32 cc;
    asm volatile("mrc p15, 0, %0, c15, c12, 1" : "=r" (cc));
    return cc;
}

// UART stuff
#define UART_BASE 0x20201000
#define UART_DR 0x0
#define UART_FR 0x18

static void __iomem* uart_base;

static void dbgc(char c) {
    while (readl(uart_base + UART_FR) & (1 << 5)) {
        // nop
    }
    writel((u32)c, uart_base + UART_DR);
}

static void dbgprintf(const char* format, ...) {
    char buf[512];
    va_list ap;

    va_start(ap, format);
    vsnprintf(buf, sizeof(buf), format, ap);
    va_end(ap);
    for (size_t i = 0; buf[i]; i++)
        dbgc(buf[i]);
}

// GPIO FIQ stuff
#define GPIO_BASE 0x20200000
#define GPIO_GPLEV0 0x34 /* Pin Level */
#define GPIO_GPEDS0 0x40 /* Pin Event Detect Status */

static void __iomem* gpio_base;

static struct fiq_handler fh = {
    .name = "ps2pipad",
};
static unsigned long fiq_stack[1024];

extern char ps2pipad_fiq, ps2pipad_fiq_end;

#define PIN_ACK (1u << 7)
#define PIN_DAT (1u << 8)
#define PIN_CLK (1u << 9)
#define PIN_CMD (1u << 11)
#define PIN_ATN (1u << 25)

static void handle_pin_change(void) {
    uint8_t dat_bytes[128];
    uint8_t cmd_bytes[128];
    size_t byte_idx = 0;

    void __iomem* gplev0 = gpio_base + GPIO_GPLEV0;
    uint32_t pins = readl(gplev0);
    bool atn_initially_hi = false;
    uint32_t atn_start_cycles, atn_end_cycles;
    if (pins & PIN_ATN) {
        atn_initially_hi = true;
        atn_start_cycles = ccnt_read();
        while (readl(gplev0) & PIN_ATN)
            ;
        atn_end_cycles = ccnt_read();
    }

    while (1) {
        unsigned cmd_byte = 0;
        unsigned dat_byte = 0;
        for (size_t bit = 0; bit < 8; bit++) {
            // Wait for falling edge on CLK
            while ((pins = readl(gplev0)) & PIN_CLK) {
                if (pins & PIN_ATN) {
                    dbgprintf("ERR: ATN went high early 1 (byte %zu bit %zu cmd 0x%02x dat 0x%02x)\r\n", byte_idx, bit, cmd_byte, dat_byte);
                    goto out;
                }
                if (!(pins & PIN_ACK)) {
                    dbgprintf("ERR: ACK went low early 1 (byte %zu bit %zu cmd 0x%02x dat 0x%02x)\r\n", byte_idx, bit, cmd_byte, dat_byte);
                    goto out;
                }
            }
            // Here we would switch output

            // Wait for rising edge on CLK
            while (!((pins = readl(gplev0)) & PIN_CLK)) {
                if (pins & PIN_ATN) {
                    dbgprintf("ERR: ATN went high early 2 (byte %zu bit %zu cmd 0x%02x dat 0x%02x)\r\n", byte_idx, bit, cmd_byte, dat_byte);
                    goto out;
                }
                if (!(pins & PIN_ACK)) {
                    dbgprintf("ERR: ACK went low early 2 (byte %zu bit %zu cmd 0x%02x dat 0x%02x)\r\n", byte_idx, bit, cmd_byte, dat_byte);
                    goto out;
                }
            }
            cmd_byte >>= 1;
            dat_byte >>= 1;
            if (pins & PIN_CMD)
                cmd_byte |= 0x80;
            if (pins & PIN_DAT)
                dat_byte |= 0x80;
        }
        dat_bytes[byte_idx] = dat_byte;
        cmd_bytes[byte_idx] = cmd_byte;
        byte_idx++;

        // Wait for ACK to toggle or ATN go high
        while ((pins = readl(gplev0)) & PIN_ACK && !(pins & PIN_ATN)) {
        }
        while (!((pins = readl(gplev0)) & PIN_ACK) && !(pins & PIN_ATN)) {
        }
        if (pins & PIN_ATN)
            break;
    }
    if (atn_initially_hi) {
        dbgprintf("atn hi for %u cycles\r\n", atn_end_cycles - atn_start_cycles);
    }
#if 1
    dbgprintf("CMD: ");
    for (size_t i = 0; i < byte_idx; i++) {
        dbgprintf("%02x", cmd_bytes[i]);
    }
    dbgprintf("\r\n");
    dbgprintf("DAT: ");
    for (size_t i = 0; i < byte_idx; i++) {
        dbgprintf("%02x", dat_bytes[i]);
    }
    dbgprintf("\r\n");
#endif

    // ack interrupts
out:
    {
        uint32_t events = readl(gpio_base + GPIO_GPEDS0);
        writel(events, gpio_base + GPIO_GPEDS0);
    }
}

int init_module() {
    // Configure CCNT cycle counter
    asm volatile("mcr p15, 0, %0, c15, c9, 0\n" :: "r"(1));
    asm volatile("mcr p15, 0, %0, c15, c12, 0\n" :: "r"(1));
    pr_info("CCNT counter enabled with userspace access allowed\n");

    // Configure UART
    uart_base = ioremap(UART_BASE, 4096);
    if (!uart_base) {
        pr_err("Couldn't ioremap uart\n");
        return -ENODEV;
    }

    // Configure GPIO FIQ interrupt
    gpio_base = ioremap(GPIO_BASE, 4096);
    if (!gpio_base) {
        pr_err("Couldn't ioremap gpio\n");
        return -ENODEV;
    }

    dbgprintf("hi! uart: 0x%08x gpio: 0x%08x", uart_base, gpio_base);

    {
        struct pt_regs fiq_regs;
        void* fiq_handler = (void *)&ps2pipad_fiq;
        size_t fiq_handler_length = &ps2pipad_fiq_end - &ps2pipad_fiq;

        if (claim_fiq(&fh)) {
            pr_err("Can't claim FIQ");
            return -ENODEV;
        }

        set_fiq_handler(fiq_handler, fiq_handler_length);

        memset(&fiq_regs, 0, sizeof(fiq_regs));
        fiq_regs.ARM_r8 = (unsigned long)handle_pin_change;
        fiq_regs.ARM_sp = ((unsigned long)&fiq_stack) + sizeof(fiq_stack) - 4;
        set_fiq_regs(&fiq_regs);
        enable_fiq(73); // TODO don't hardcode

    }

    return 0;
}

void cleanup_module() {
}

MODULE_LICENSE("GPL");
