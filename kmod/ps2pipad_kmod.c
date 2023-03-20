#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <asm/fiq.h>

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

static void dbgprintf(const char* format, ...)
{
    char buf[512];
    va_list ap;

    va_start(ap, format);
    vsnprintf(buf, sizeof(buf), format, ap);
    va_end(ap);
    for (size_t i = 0; buf[i]; i++)
        dbgc(buf[i]);
    dbgc('\r');
    dbgc('\n');
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

static void handle_pin_change(void) {
    // ack interrupts
    uint32_t events = readl(gpio_base + GPIO_GPEDS0);
    writel(events, gpio_base + GPIO_GPEDS0);
    dbgprintf("events was: 0x%08x", events);
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
        fiq_regs.ARM_r0 = 0xABCDEF00;
        fiq_regs.ARM_r1 = 0xABCDEF01;
        fiq_regs.ARM_r2 = 0xABCDEF02;
        fiq_regs.ARM_r3 = 0xABCDEF03;
        fiq_regs.ARM_r4 = 0xABCDEF04;
        fiq_regs.ARM_r5 = 0xABCDEF05;
        fiq_regs.ARM_r6 = 0xABCDEF06;
        fiq_regs.ARM_r7 = 0xABCDEF07;
        fiq_regs.ARM_r8 = (unsigned long)handle_pin_change;
        fiq_regs.ARM_r9 = 0xABCDEF09;
        fiq_regs.ARM_r10 = 0xABCDEF10;
        fiq_regs.ARM_sp = ((unsigned long)&fiq_stack) + sizeof(fiq_stack) - 4;
        set_fiq_regs(&fiq_regs);
        enable_fiq(73); // TODO don't hardcode

    }

    return 0;
}

void cleanup_module() {
}

MODULE_LICENSE("GPL");
