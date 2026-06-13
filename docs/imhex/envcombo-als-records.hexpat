#pragma description ENV-COMBO ALS triggered-buffer records (16-byte: light + timestamp)
#pragma endian little

import std.mem;
import std.time;

fn format_timestamp(s64 ns) {
    return std::time::format(std::time::to_utc(ns / 1000000000),
                             "%Y-%m-%d %H:%M:%S UTC");
};

struct Sample {
    u16 light;
    padding[6];
    s64 timestamp_ns [[format("format_timestamp")]];
};

Sample samples[while(!std::mem::eof())] @ 0x00;
