#pragma description ENV-COMBO ALS triggered-buffer records (16-byte: light + timestamp)
#pragma endian little

import std.mem;

struct Sample {
    u16 light;
    padding[6];
    s64 timestamp_ns;
};

Sample samples[while(!std::mem::eof())] @ 0x00;
