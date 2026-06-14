// Default little-endian so bitfields fill LSB-first (first member = bit 0).
// `#pragma endian big` would flip the bit order to MSB-first and mis-decode
// CFG/INT_CFG/STATUS (e.g. CFG 0x84 reading als_en = 0 instead of 1). The
// three 16-bit fields are MSB-first in the blob, so they are tagged `be` below.
#pragma endian little

// A bitfield renders only as its decomposed fields, so the raw composite byte
// (e.g. CFG = 0x84) is never shown. Reg8<T> overlays a raw u8 on the same byte
// as the bitfield T, so the Pattern Data view shows both: raw = 0x84 alongside
// the decoded bits. u8 alone would display decimal (132), so format it as hex.
// builtin::std::format is the compiled-in formatter (what std::io::format wraps),
// so this needs no import and no installed standard library.
fn hex8(u8 v) {
    return builtin::std::format("0x{:02X}", v);
};

union Reg8<T> {
    u8 raw [[format("hex8")]];
    T   bits;
};

enum PwrMode : u8 {
    Off = 0x00, Sleep = 0x01, OneShot = 0x02, Continuous = 0x03,
};

// Bitfields are LSB-first: first member = bit 0.
bitfield CFG {        // 0x06
    padding  : 1;     // bit 0
    als_time : 2;     // bits 2:1
    als_gain : 2;     // bits 4:3
    padding  : 2;     // bits 6:5  (temp_en/hum_en — other channels)
    als_en   : 1;     // bit 7
};

bitfield INT_CFG {    // 0x07
    padding   : 5;
    int_pol   : 1;    // bit 5
    int_latch : 1;    // bit 6
    int_en    : 1;    // bit 7
};

bitfield STATUS {     // 0x0C  (clear-on-read on the real bus; safe to read here)
    als_int : 1;      // bit 0
    padding : 2;      // bits 2:1  (temp_rdy/hum_rdy)
    als_rdy : 1;      // bit 3
    padding : 4;
};

struct EnvComboAls {
    u8      who_am_i;     // 0x00  expect 0xEB
    padding[3];           // 0x01-0x03  temp MSB/LSB + humidity (not ALS)
    be u16  als_out;      // 0x04-0x05  latched ALS result (MSB-first)
    Reg8<CFG>     cfg;        // 0x06  raw = 0x84 + decoded bits
    Reg8<INT_CFG> int_cfg;    // 0x07
    be u16  als_th_low;   // 0x08-0x09  falling threshold (MSB-first)
    be u16  als_th_high;  // 0x0A-0x0B  rising threshold (MSB-first)
    Reg8<STATUS>  status;     // 0x0C
    padding[3];           // 0x0D-0x0F  temp/hum calibration (not ALS)
    u8      cal_again;    // 0x10  ALS gain trim
    u8      cal_atime;    // 0x11  non-zero = ALS integration-time override
    PwrMode pwr_mode;     // 0x12
};

EnvComboAls regs @ 0x00;