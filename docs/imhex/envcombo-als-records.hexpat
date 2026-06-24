#pragma description ENV-COMBO ALS triggered-buffer records (16-byte: light + timestamp)
#pragma endian little

import std.mem;
import std.time;

// Fixed UTC+3 offset to match the device's TZ='IST-3' (current Israel DST
// offset, set in recipes-core/envcombo-timezone/files/timezone.sh). Not
// DST-aware -- revisit if that file's offset changes for winter (UTC+2).
fn format_timestamp(s64 ns) {
    return std::time::format(std::time::to_utc(ns / 1000000000 + 3 * 60 * 60),
                             "%Y-%m-%d %H:%M:%S IST");
};

struct Sample {
    u16 light;
    padding[6];
    s64 timestamp_ns [[format("format_timestamp")]];
};

Sample samples[while(!std::mem::eof())] @ 0x00;
