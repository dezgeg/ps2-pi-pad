#pragma once

struct pad_response {
    uint16_t buttons;
    int8_t analogs[4];
    uint8_t pressures[12];
} __attribute__((packed));

struct ps2pipad_uapi {
    struct pad_response resp;

    bool reset;
};

