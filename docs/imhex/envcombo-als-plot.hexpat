#pragma description ENV-COMBO ALS samples - line plot of light vs sample index
#pragma endian little

import std.mem;

u16 light[while(!std::mem::eof())] @ 0x00 [[hex::visualize("line_plot", this)]];
