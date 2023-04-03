#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <errno.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <stdbool.h>
#include <linux/joystick.h>

#include "uapi.h"

// Order in bitmap
enum {
    PAD_SELECT,
    PAD_L3,
    PAD_R3,
    PAD_START,
    PAD_U,
    PAD_R,
    PAD_D,
    PAD_L,
    PAD_L2,
    PAD_R2,
    PAD_L1,
    PAD_R1,
    PAD_TRIANGLE,
    PAD_O,
    PAD_X,
    PAD_SQUARE,
};

// Order in packet
enum {
    ANALOG_RX,
    ANALOG_RY,
    ANALOG_LX,
    ANALOG_LY,
    // Hack, DS4 needs translation from axis to button
    DPAD_X,
    DPAD_Y,
};

// Linux button number to PAD_ enum - for DualShock4
static ssize_t linux_button_to_pad_button(uint32_t button) {
    switch (button) {
        case 0: return PAD_X;
        case 1: return PAD_O;
        case 2: return PAD_TRIANGLE;
        case 3: return PAD_SQUARE;
        case 4: return PAD_L1;
        case 5: return PAD_R1;
        case 6: return PAD_L2;
        case 7: return PAD_R2;
        case 8: return PAD_SELECT;
        case 9: return PAD_START;
        // case 10: PAD_ANALOG; if implemented
        case 11: return PAD_L3;
        case 12: return PAD_R3;
    }
    return -1;
};

// Linux axis number to ANALOG_ enum - for DualShock4
static ssize_t linux_axis_to_pad_axis(uint32_t axis) {
    switch (axis) {
        case 0: return ANALOG_LX;
        case 1: return ANALOG_LY;
        case 3: return ANALOG_RX;
        case 4: return ANALOG_RY;
        case 6: return DPAD_X;
        case 7: return DPAD_Y;
    }
    return -1;
};

// Index in pressures array
static ssize_t pad_button_to_pressure_index(uint32_t button) {
    switch (button) {
        case PAD_R: return 0;
        case PAD_L: return 1;
        case PAD_U: return 2;
        case PAD_D: return 3;
        case PAD_TRIANGLE: return 4;
        case PAD_O: return 5;
        case PAD_X: return 6;
        case PAD_SQUARE: return 7;
        case PAD_L1: return 8;
        case PAD_R1: return 9;
        case PAD_L2: return 10;
        case PAD_R2: return 11;
    }
    return 0;
};

int main(int argc, char **argv)
{
    int fd = open("/proc/ps2pipad", O_RDWR);
    assert(fd >= 0);

    volatile struct ps2pipad_uapi* user_page = (volatile struct ps2pipad_uapi*)mmap(0, getpagesize(), PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0);

    fd = -1;
    while (1) {
        if (fd < 0) {
            fd = open("/dev/input/js0", O_RDONLY);
        }
        if (fd < 0) {
            continue;
        }
        struct js_event e;
        int ret = read(fd, &e, sizeof(e));
        if (ret == -1 && errno == ENODEV) {
            // Controller got unplugged
            close(fd);
            continue;
        }
        if (e.type & JS_EVENT_AXIS) {
            ssize_t ix = linux_axis_to_pad_axis(e.number);
            if (ix < 0) {
                continue;
            } else if (ix == DPAD_X) {
                user_page->resp.buttons |= (1 << PAD_L) | (1 << PAD_R);
                user_page->resp.pressures[pad_button_to_pressure_index(PAD_L)] = 0;
                user_page->resp.pressures[pad_button_to_pressure_index(PAD_R)] = 0;
                if (e.value < 0) {
                    user_page->resp.buttons &= ~(1 << PAD_L);
                    user_page->resp.pressures[pad_button_to_pressure_index(PAD_L)] = 0xff;
                } else if (e.value > 0) {
                    user_page->resp.buttons &= ~(1 << PAD_R);
                    user_page->resp.pressures[pad_button_to_pressure_index(PAD_R)] = 0xff;
                }
            } else if (ix == DPAD_Y) {
                user_page->resp.buttons |= (1 << PAD_U) | (1 << PAD_D);
                user_page->resp.pressures[pad_button_to_pressure_index(PAD_U)] = 0;
                user_page->resp.pressures[pad_button_to_pressure_index(PAD_D)] = 0;
                if (e.value < 0) {
                    user_page->resp.buttons &= ~(1 << PAD_U);
                    user_page->resp.pressures[pad_button_to_pressure_index(PAD_U)] = 0xff;
                } else if (e.value > 0) {
                    user_page->resp.buttons &= ~(1 << PAD_D);
                    user_page->resp.pressures[pad_button_to_pressure_index(PAD_D)] = 0xff;
                }
            } else {
                user_page->resp.analogs[linux_axis_to_pad_axis(e.number)] = 128 + (e.value >> 8);
            }
        } else if (e.type & JS_EVENT_BUTTON) {
            ssize_t ix = linux_button_to_pad_button(e.number);
            if (ix < 0)
                continue;

            if (e.value) {
                user_page->resp.buttons &= ~(1 << ix);
                user_page->resp.pressures[pad_button_to_pressure_index(ix)] = 0xff;
            } else {
                user_page->resp.buttons |= 1 << ix;
                user_page->resp.pressures[pad_button_to_pressure_index(ix)] = 0x0;
            }
        }
    }
}
