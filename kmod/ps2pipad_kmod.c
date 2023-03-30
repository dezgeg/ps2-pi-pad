#include <linux/delay.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
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

static u32 prev_fiq_start;
static u32 prev_fiq_end;

static uint8_t dat_bytes[256];
static uint8_t cmd_bytes[256];

#define MODE_DIGITAL 0x41
#define MODE_ANALOG 0x73
#define MODE_DS2_NATIVE 0x79

// State variables
static uint32_t mode = MODE_DIGITAL;
static bool in_config = false;
static uint8_t vibration_map[] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
static bool analog_locked = false;
static uint32_t mask[] = { 0xff, 0xff, 0x03 };

// State variables to be switched to at end of transaction
static uint32_t new_mode = MODE_DIGITAL;
static bool new_in_config = false;

// Verifier
static uint8_t expected_dat_resp[256];
static bool compare_dat_resp = false;
static bool check_in_config = false;
static bool known_command = false;

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
    // Static field checks
    if (index == 0) {
        // Initialize
        compare_dat_resp = false;
        check_in_config = false;
        known_command = false;

        CHECK(cmd_bytes[0], 0x01);
        CHECK(dat_bytes[0], 0xff);
        return;
    }

    // if (index == 2) {
        // CHECK(cmd_bytes[2], 0x00);
        // CHECK(dat_bytes[2], 0x5a);
        // return;
    // }

    // Commands without any data dependencies on parameters
    uint32_t cmd = cmd_bytes[1];
    if (index == 1) {
        switch (cmd) {
            case 0x40: {
                compare_dat_resp = true;
                check_in_config = true;
                known_command = true;
                memcpy(expected_dat_resp, CMD_40_RESP, CONFIG_RESP_SIZE);
                break;
            }
            case 0x41: {
                compare_dat_resp = true;
                check_in_config = true;
                known_command = true;
                if (mode == MODE_DIGITAL) {
                    memset(expected_dat_resp, 0, CONFIG_RESP_SIZE);
                } else {
                    dbgprintf("todo cmd41 non digital\r\n");
                }
                break;
            }
            case 0x42: {
                // TODO:
                known_command = true;
                break;
            }
            case 0x44: {
                // data bytes read later:
                compare_dat_resp = true;
                check_in_config = true;
                known_command = true;
                memset(expected_dat_resp, 0, CONFIG_RESP_SIZE);
                break;
            }
            case 0x45: {
                check_in_config = true;
                known_command = true;
                memcpy(expected_dat_resp, CMD_45_RESP_TEMPLATE, CONFIG_RESP_SIZE);
                expected_dat_resp[2] = mode != MODE_DIGITAL;
                break;
            }
            case 0x4d: {
                check_in_config = true;
                known_command = true;
                memcpy(expected_dat_resp, vibration_map, CONFIG_RESP_SIZE);
                break;
            }
            case 0x4f: {
                check_in_config = true;
                known_command = true;
                compare_dat_resp = true;
                new_mode = MODE_DS2_NATIVE;
                memcpy(expected_dat_resp, CMD_4F_RESP, CONFIG_RESP_SIZE);
                break;
            }
        }
    }

    // Constant queries have data dependencies
    if (index == 3) {
        switch (cmd) {
            case 0x43: {
                new_in_config = cmd_bytes[3] == 1;
                known_command = true;
                break;
            }
            case 0x46: {
                compare_dat_resp = true;
                check_in_config = true;
                known_command = true;
                if (cmd_bytes[3] == 0)
                    memcpy(expected_dat_resp, CMD_46_RESP_PAGE0, CONFIG_RESP_SIZE);
                else if (cmd_bytes[3] == 1)
                    memcpy(expected_dat_resp, CMD_46_RESP_PAGE1, CONFIG_RESP_SIZE);
                else
                    dbgprintf("unknown cmd 46 page %u\r\n", cmd_bytes[3]);
                break;
            }
            case 0x4c: {
                compare_dat_resp = true;
                check_in_config = true;
                known_command = true;
                if (cmd_bytes[3] == 0)
                    memcpy(expected_dat_resp, CMD_4C_RESP_PAGE0, CONFIG_RESP_SIZE);
                else if (cmd_bytes[3] == 1)
                    memcpy(expected_dat_resp, CMD_4C_RESP_PAGE1, CONFIG_RESP_SIZE);
                else
                    dbgprintf("unknown cmd 4c page %u\r\n", cmd_bytes[3]);
                break;
            }
            case 0x47: {
                compare_dat_resp = true;
                check_in_config = true;
                known_command = true;
                CHECK(dat_bytes[4], 0x00);
                memcpy(expected_dat_resp, CMD_47_RESP, CONFIG_RESP_SIZE);
                break;
            }


        }
    }

    if (cmd == 0x44 && index == 3) {
        if (cmd_bytes[3])
            mode = MODE_ANALOG;
        else
            mode = MODE_DIGITAL;
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
    if (!known_command) {
        dbgprintf("unknown command 0x%02x\r\n", cmd_bytes[1]);
    }
    if (check_in_config && !in_config) {
        dbgprintf("expected to be in config mode\r\n");
    }
    size_t expected_len = 3;
    if (in_config) {
        expected_len = 9;
    } else if (mode == MODE_DIGITAL) {
        expected_len = 5;
    } else if (mode == MODE_ANALOG) {
        expected_len = 5;
    } else if (mode == MODE_DS2_NATIVE) {
        expected_len = 21;
    }
    CHECK(len, expected_len);
    if (compare_dat_resp) {
        if (memcmp(dat_bytes + 3, expected_dat_resp, expected_len - 3)) {
            dbgprintf("mismatch\r\n");
        }
    }

    in_config = new_in_config;
    new_mode = mode;
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
        got_byte(byte_idx);
        byte_idx++;

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
    }

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

    // ack interrupts
out:
    {
        uint32_t events = readl(gpio_base + GPIO_GPEDS0);
        writel(events, gpio_base + GPIO_GPEDS0);
    }
    uint32_t fiq_end = ccnt_read();
    prev_fiq_start = fiq_start;
    prev_fiq_end = fiq_end;
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
    disable_fiq(73);
    msleep(10);
    release_fiq(&fh);
    iounmap(gpio_base);
    iounmap(uart_base);
}

MODULE_LICENSE("GPL");
