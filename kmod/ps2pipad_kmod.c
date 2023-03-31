#include <linux/delay.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <asm/fiq.h>

struct user_comm {
    bool reset;
};
static struct user_comm* user_page;

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

#define PIN_ACK (1u << 7)
#define PIN_DAT (1u << 8)
#define PIN_CLK (1u << 9)
#define PIN_CMD (1u << 11)
#define PIN_ATN (1u << 25)

#define FSEL_ACK_MASK (0x7 << (3 * 7))
#define FSEL_ACK_OUT  (0x1 << (3 * 7))
#define FSEL_DAT_MASK (0x7 << (3 * 8))
#define FSEL_DAT_OUT  (0x1 << (3 * 8))

#define GPIO_BASE 0x20200000
#define GPIO_GPFSEL0 0x00 /* Function Select */
#define GPIO_GPCLR0  0x28 /* Output Clear */
#define GPIO_GPLEV0  0x34 /* Pin Level */
#define GPIO_GPEDS0  0x40 /* Pin Event Detect Status */

static void __iomem* gpio_base;

static struct fiq_handler fh = {
    .name = "ps2pipad",
};
static unsigned long fiq_stack[1024];

extern char ps2pipad_fiq, ps2pipad_fiq_end;

static u32 prev_fiq_start;
static u32 prev_fiq_end;

static uint8_t dat_bytes[256];
static uint8_t cmd_bytes[256];

#define MODE_DIGITAL 0x41
#define MODE_ANALOG 0x73
#define MODE_DS2_NATIVE 0x79

// State variables
static uint32_t mode;
static bool in_config;
static uint8_t vibration_map[6];
static bool analog_locked;
static uint32_t mask[3];

// State variables to be switched to at end of transaction
static uint32_t new_mode = MODE_DIGITAL;
static bool new_in_config = false;

// Verifier
static uint8_t dat_resp[256];
static bool check_in_config = false;

#define CHECK(val, exp) do_check(val, exp, #val)
static void do_check(uint32_t value, uint32_t expected, const char* what) {
    if (value != expected)
        dbgprintf("WRN: unexpected %s: 0x%02x, expecting: 0x%02x\r\n", what, value, expected);
}

static const u8 CMD_40_RESP[] = { 0x00, 0x00, 0x02, 0x00, 0x00, 0x5a };
static const u8 CMD_45_RESP_TEMPLATE[] = { 0x03, 0x02, 0x00, 0x02, 0x01, 0x00 };
static const u8 CMD_46_RESP_PAGE0[] = { 0x00, 0x00, 0x01, 0x02, 0x00, 0x0a };
static const u8 CMD_46_RESP_PAGE1[] = { 0x00, 0x00, 0x01, 0x01, 0x01, 0x14 };
static const u8 CMD_47_RESP[] = { 0x00, 0x00, 0x02, 0x00, 0x01, 0x00 };
static const u8 CMD_4C_RESP_PAGE0[] = { 0x00, 0x00, 0x00, 0x04, 0x00, 0x00 };
static const u8 CMD_4C_RESP_PAGE1[] = { 0x00, 0x00, 0x00, 0x07, 0x00, 0x00 };
static const u8 CMD_4F_RESP[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x5A };
#define CONFIG_RESP_SIZE 6

static void got_byte(size_t index) {
    // Commands without any data dependencies on parameters
    uint32_t cmd = cmd_bytes[1];
    if (index == 1) {
        switch (cmd) {
            case 0x40: {
                check_in_config = true;
                memcpy(dat_resp, CMD_40_RESP, CONFIG_RESP_SIZE);
                break;
            }
            case 0x41: {
                check_in_config = true;
                memset(dat_resp, 0, CONFIG_RESP_SIZE);
                if (mode != MODE_DIGITAL) {
                    dat_resp[0] = mask[0];
                    dat_resp[1] = mask[1];
                    dat_resp[2] = mask[2];
                    dat_resp[5] = 0x5a;
                }
                break;
            }

            case 0x43:
            // data dependency handled later
            // fall thru
            case 0x42: {
                // TODO:
                memset(dat_resp, 0, sizeof(dat_resp));
                break;
            }
            case 0x44: {
                // data bytes read later:
                check_in_config = true;
                memset(dat_resp, 0, CONFIG_RESP_SIZE);
                break;
            }
            case 0x45: {
                check_in_config = true;
                memcpy(dat_resp, CMD_45_RESP_TEMPLATE, CONFIG_RESP_SIZE);
                dat_resp[2] = mode != MODE_DIGITAL;
                break;
            }
            case 0x46:
            case 0x4c: {
                check_in_config = true;
                dat_resp[0] = 0;
                // data dependency handled later
                break;
            }

            case 0x47: {
                check_in_config = true;
                memcpy(dat_resp, CMD_47_RESP, CONFIG_RESP_SIZE);
                break;
            }

            case 0x4d: {
                check_in_config = true;
                memcpy(dat_resp, vibration_map, CONFIG_RESP_SIZE);
                break;
            }
            case 0x4f: {
                check_in_config = true;
                new_mode = MODE_DS2_NATIVE;
                memcpy(dat_resp, CMD_4F_RESP, CONFIG_RESP_SIZE);
                break;
            }
            default: {
                dbgprintf("unknown command 0x%02x\r\n", cmd_bytes[1]);
                memset(dat_resp, 0xff, sizeof(dat_resp));
                break;
            }
        }
    }

    // Constant queries have data dependencies
    if (index == 3) {
        switch (cmd) {
            case 0x43: {
                new_in_config = cmd_bytes[3] == 1;
                break;
            }
            case 0x46: {
                if (cmd_bytes[3] == 0)
                    memcpy(dat_resp, CMD_46_RESP_PAGE0, CONFIG_RESP_SIZE);
                else if (cmd_bytes[3] == 1)
                    memcpy(dat_resp, CMD_46_RESP_PAGE1, CONFIG_RESP_SIZE);
                else
                    dbgprintf("unknown cmd 46 page %u\r\n", cmd_bytes[3]);
                break;
            }
            case 0x4c: {
                if (cmd_bytes[3] == 0)
                    memcpy(dat_resp, CMD_4C_RESP_PAGE0, CONFIG_RESP_SIZE);
                else if (cmd_bytes[3] == 1)
                    memcpy(dat_resp, CMD_4C_RESP_PAGE1, CONFIG_RESP_SIZE);
                else
                    dbgprintf("unknown cmd 4c page %u\r\n", cmd_bytes[3]);
                break;
            }
        }
    }

    if (cmd == 0x44 && index == 3) {
        if (cmd_bytes[3])
            new_mode = MODE_ANALOG;
        else
            new_mode = MODE_DIGITAL;
    }
    if (cmd == 0x44 && index == 4) {
        analog_locked = cmd_bytes[4];
    }

    if (cmd == 0x4d && index >= 3 && index - 3 < 6) {
        vibration_map[index - 3] = cmd_bytes[index - 3];
    }

    if (cmd == 0x4f && index >= 3 && index - 3 < 3) {
        mask[index - 3] = cmd_bytes[index - 3];
    }
}

static void transaction_over(size_t len) {
    if (check_in_config && !new_in_config) {
        dbgprintf("expected to be in config mode\r\n");
    }

    in_config = new_in_config;
    mode = new_mode;
}

static void reset_state(void) {
    mode = MODE_DIGITAL;
    in_config = false;
    memset(vibration_map, 0xff, sizeof(vibration_map));
    analog_locked = false;
    mask[0] = 0xff;
    mask[1] = 0xff;
    mask[2] = 0x03;

    new_mode = mode;
    new_in_config = in_config;
}

static void handle_pin_change(void) {
    uint32_t fiq_start = ccnt_read();
    size_t byte_idx = 0;

    void __iomem* gplev0 = gpio_base + GPIO_GPLEV0;
    uint32_t pins = readl(gplev0);
    bool atn_initially_hi = false;
    uint32_t atn_start_cycles, atn_end_cycles;
    bool ended_with_ack = false;

    if (pins & PIN_ATN) {
        atn_initially_hi = true;
        atn_start_cycles = ccnt_read();
        while (readl(gplev0) & PIN_ATN)
            ;
        atn_end_cycles = ccnt_read();
    }
    if (user_page->reset) {
        //dbgprintf("reset\r\n");
        reset_state();
        user_page->reset = false;
    }

    size_t expected_len = 3;
    if (in_config) {
        expected_len = 9;
    } else if (mode == MODE_DIGITAL) {
        expected_len = 5;
    } else if (mode == MODE_ANALOG) {
        expected_len = 9;
    } else if (mode == MODE_DS2_NATIVE) {
        expected_len = 21;
    }
    check_in_config = false;

    for (byte_idx = 0; byte_idx < expected_len; byte_idx++) {
        unsigned cmd_byte = 0;
        unsigned dat_byte;

        switch (byte_idx) {
            case 0: dat_byte = 0xff; break;
            case 1: dat_byte = in_config ? 0xf3 : mode; break;
            case 2: dat_byte = 0x5a; break;
            default: dat_byte = dat_resp[byte_idx - 3];
        }
#ifndef SNIFF
        dat_bytes[byte_idx] = dat_byte;
#endif

        for (size_t bit = 0; bit < 8; bit++) {
            // Wait for falling edge on CLK
            while ((pins = readl(gplev0)) & PIN_CLK) {
                if (pins & PIN_ATN) {
                    // very spammy
                    //dbgprintf("ERR: ATN went high early 1 (byte %zu bit %zu cmd 0x%02x dat 0x%02x)\r\n", byte_idx, bit, cmd_byte, dat_byte);
                    goto out;
                }
#ifdef SNIFF
                if (!(pins & PIN_ACK)) {
                    dbgprintf("ERR: ACK went low early 1 (byte %zu bit %zu cmd 0x%02x dat 0x%02x)\r\n", byte_idx, bit, cmd_byte, dat_byte);
                    goto out;
                }
#endif
            }
            if (dat_byte & 1) {
                // Set input
                uint32_t fsel = readl(gpio_base + GPIO_GPFSEL0);
                fsel &= ~FSEL_DAT_MASK;
                writel(fsel, gpio_base + GPIO_GPFSEL0);
            } else {
                // Set output
                uint32_t fsel = readl(gpio_base + GPIO_GPFSEL0);
                fsel &= ~FSEL_DAT_MASK;
                fsel |= FSEL_DAT_OUT;
                writel(fsel, gpio_base + GPIO_GPFSEL0);
            }

            // Wait for rising edge on CLK
            while (!((pins = readl(gplev0)) & PIN_CLK)) {
                if (pins & PIN_ATN) {
                    dbgprintf("ERR: ATN went high early 2 (byte %zu bit %zu cmd 0x%02x dat 0x%02x)\r\n", byte_idx, bit, cmd_byte, dat_byte);
                    goto out;
                }
#ifdef SNIFF
                if (!(pins & PIN_ACK)) {
                    dbgprintf("ERR: ACK went low early 2 (byte %zu bit %zu cmd 0x%02x dat 0x%02x)\r\n", byte_idx, bit, cmd_byte, dat_byte);
                    goto out;
                }
#endif
            }
            cmd_byte >>= 1;
            dat_byte >>= 1;
            if (pins & PIN_CMD)
                cmd_byte |= 0x80;

#ifdef SNIFF
            if (pins & PIN_DAT)
               dat_byte |= 0x80;
#endif
        }
#ifdef SNIFF
        dat_bytes[byte_idx] = dat_byte;
#endif
        cmd_bytes[byte_idx] = cmd_byte;
        got_byte(byte_idx);

#ifdef SNIFF
        ended_with_ack = false;
        // Wait for ACK to toggle or ATN go high
        while ((pins = readl(gplev0)) & PIN_ACK && !(pins & PIN_ATN)) {
        }

        if (!(pins & PIN_ACK)) {
            ended_with_ack = true;
            while (!((pins = readl(gplev0)) & PIN_ACK) && !(pins & PIN_ATN)) {
            }
        }

        if (pins & PIN_ATN)
            break;
#else
    // Don't ack last byte
    if (byte_idx == expected_len - 1)
        break;

    uint32_t timer_start = ccnt_read();
    // Give some time before responding with ack
    while (ccnt_read() - timer_start < 2000)
        ;

    // Set ack output
    uint32_t fsel = readl(gpio_base + GPIO_GPFSEL0);
    fsel &= ~FSEL_ACK_MASK;
    fsel |= FSEL_ACK_OUT;
    writel(fsel, gpio_base + GPIO_GPFSEL0);

    // Hold ack low for a bit
    while (ccnt_read() - timer_start < 4000)
        ;

    // Set ack back to input
    fsel &= ~FSEL_ACK_MASK;
    writel(fsel, gpio_base + GPIO_GPFSEL0);
#endif
    }

out:
    transaction_over(byte_idx);

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
    dbgprintf("%c\r\n", ended_with_ack ? '+' : '-');
    dbgprintf("\r\n");
#else
    dbgprintf("%u %u %zu %02x %02x %c\r\n", prev_fiq_end - prev_fiq_start, fiq_start - prev_fiq_end, byte_idx, cmd_bytes[1], dat_bytes[1], ended_with_ack ? '+' : '-');
#endif

    // reset pin states just in case
    {
        uint32_t fsel = readl(gpio_base + GPIO_GPFSEL0);
        fsel &= ~FSEL_ACK_MASK;
        fsel &= ~FSEL_DAT_MASK;
        writel(fsel, gpio_base + GPIO_GPFSEL0);
    }

    // ack interrupts
    {
        uint32_t events = readl(gpio_base + GPIO_GPEDS0);
        writel(events, gpio_base + GPIO_GPEDS0);
    }
    uint32_t fiq_end = ccnt_read();
    prev_fiq_start = fiq_start;
    prev_fiq_end = fiq_end;
}

static int page_map_mmap( struct file *file, struct vm_area_struct *vma )
{
    if ((vma->vm_end - vma->vm_start) > PAGE_SIZE)
        return -EINVAL;

    remap_pfn_range(vma, vma->vm_start,
                    __pa(pde_data(file_inode(file))) >> PAGE_SHIFT,
                    PAGE_SIZE, vma->vm_page_prot);
    return 0;
}

static const struct proc_ops page_map_proc_ops = {
    //.proc_lseek = page_map_seek,
    //.proc_read = page_map_read,
    .proc_mmap = page_map_mmap,
};

int init_module() {
    // Configure proc entry
    struct proc_dir_entry* pde;
    user_page = (struct user_comm*)__get_free_page(GFP_KERNEL | __GFP_ZERO);
    if (!user_page) {
        pr_err("Couldn't allocate page\n");
        return -ENODEV;
    }

    pde = proc_create_data("ps2pipad", S_IFREG | 0644, NULL,
                            &page_map_proc_ops, user_page);
    if (!pde) {
        pr_err("Couldn't create proc entry\n");
        return -ENODEV;
    }
    proc_set_size(pde, PAGE_SIZE);

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

    {
        // Ensure inputs
        uint32_t fsel = readl(gpio_base + GPIO_GPFSEL0);
        fsel &= ~FSEL_ACK_MASK;
        fsel &= ~FSEL_DAT_MASK;
        writel(fsel, gpio_base + GPIO_GPFSEL0);

        // Ensure low state when switched to outputs
        writel(PIN_ACK | PIN_DAT, gpio_base + GPIO_GPCLR0);
    }

    dbgprintf("hi! uart: 0x%08x gpio: 0x%08x", uart_base, gpio_base);
    reset_state();

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
    disable_fiq(73);
    msleep(10);
    release_fiq(&fh);
    iounmap(gpio_base);
    iounmap(uart_base);
}

MODULE_LICENSE("GPL");
