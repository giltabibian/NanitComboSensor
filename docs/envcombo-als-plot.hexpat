#pragma description ENV-COMBO ALS samples - line plot of light vs sample index
#pragma endian little

import std.mem;

// For this pattern, capture the triggered buffer with ONLY the illuminance
// scan element enabled (timestamp OFF), so every record is a bare little-
// endian u16 and the whole file is a contiguous stream of light values:
//
//   cd /sys/bus/iio/devices/iio:device0
//   echo 1 > scan_elements/in_illuminance_en
//   echo 0 > scan_elements/in_timestamp_en      # <-- pure u16 records
//   echo envcombo-dev0 > trigger/current_trigger
//   echo 256 > buffer/length
//   echo 1 > buffer/enable
//   dd if=/dev/iio:device0 of=/tmp/als_lights.bin bs=2 count=256
//   echo 0 > buffer/enable
//
// Open als_lights.bin with this pattern; ImHex draws the ramp in the
// visualizer pane (Pattern Editor -> the line_plot view).

u16 light[while(!std::mem::eof())] @ 0x00 [[hex::visualize("line_plot", light)]];
