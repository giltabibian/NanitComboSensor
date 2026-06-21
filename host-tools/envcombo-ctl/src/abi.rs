//! Sysfs/debugfs ABI surface of the envcombo driver + simulator, and the
//! commands used to read it remotely. Offsets/bitfields mirror
//! `recipes-kernel/envcombo-driver/files/envcombo.c` (ENVCOMBO_REG_*) and
//! `recipes-kernel/envcombo-sim/files/i2c-envcombo-sim.c` (`enum { R_WHO_AM_I,
//! ... REG_COUNT }`).

pub const DEBUGFS_REGS: &str = "/sys/kernel/debug/envcombo-sim/regs";
pub const REG_COUNT: usize = 0x13;

pub mod off {
    pub const WHO_AM_I: usize = 0x00;
    pub const TEMP_MSB: usize = 0x01;
    pub const HUMIDITY: usize = 0x03;
    pub const ALS_MSB: usize = 0x04;
    pub const CFG: usize = 0x06;
    pub const INT_CFG: usize = 0x07;
    pub const ALS_TH_LOW_MSB: usize = 0x08;
    pub const ALS_TH_HIGH_MSB: usize = 0x0A;
    pub const STATUS: usize = 0x0C;
    pub const CAL_TOFF_MSB: usize = 0x0D;
    pub const CAL_HOFF: usize = 0x0F;
    pub const CAL_AGAIN: usize = 0x10;
    pub const CAL_ATIME: usize = 0x11;
    pub const PWR_MODE: usize = 0x12;
}

pub const GAIN_TABLE: [u32; 4] = [1, 4, 16, 64];
pub const TIME_TABLE_MS: [u32; 4] = [50, 100, 200, 400];

fn be16(regs: &[u8; REG_COUNT], offset: usize) -> u16 {
    (regs[offset] as u16) << 8 | regs[offset + 1] as u16
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PwrMode {
    Off,
    Sleep,
    OneShot,
    Continuous,
}

impl PwrMode {
    fn decode(byte: u8) -> Self {
        match byte & 0x03 {
            0 => PwrMode::Off,
            1 => PwrMode::Sleep,
            2 => PwrMode::OneShot,
            _ => PwrMode::Continuous,
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            PwrMode::Off => "OFF",
            PwrMode::Sleep => "SLEEP",
            PwrMode::OneShot => "ONE-SHOT",
            PwrMode::Continuous => "CONTINUOUS",
        }
    }
}

/// Field-by-field decode of the simulator's 19-byte debugfs register file.
pub struct Decoded {
    pub who_am_i: u8,
    pub temp: i16,
    pub humidity: u8,
    pub als: u16,
    pub als_en: bool,
    pub gain_idx: u8,
    pub time_idx: u8,
    pub int_en: bool,
    pub int_latch: bool,
    pub int_pol: bool,
    pub als_th_low: u16,
    pub als_th_high: u16,
    pub als_int: bool,
    pub temp_rdy: bool,
    pub hum_rdy: bool,
    pub als_rdy: bool,
    pub cal_toff: i16,
    pub cal_hoff: i8,
    pub cal_again: u8,
    pub cal_atime: u8,
    pub pwr_mode: PwrMode,
}

pub fn decode(regs: &[u8; REG_COUNT]) -> Decoded {
    let cfg = regs[off::CFG];
    let int_cfg = regs[off::INT_CFG];
    let status = regs[off::STATUS];
    Decoded {
        who_am_i: regs[off::WHO_AM_I],
        temp: be16(regs, off::TEMP_MSB) as i16,
        humidity: regs[off::HUMIDITY],
        als: be16(regs, off::ALS_MSB),
        als_en: cfg & 0x80 != 0,
        gain_idx: (cfg >> 3) & 0x03,
        time_idx: (cfg >> 1) & 0x03,
        int_en: int_cfg & 0x80 != 0,
        int_latch: int_cfg & 0x40 != 0,
        int_pol: int_cfg & 0x20 != 0,
        als_th_low: be16(regs, off::ALS_TH_LOW_MSB),
        als_th_high: be16(regs, off::ALS_TH_HIGH_MSB),
        als_int: status & 0x01 != 0,
        temp_rdy: status & 0x02 != 0,
        hum_rdy: status & 0x04 != 0,
        als_rdy: status & 0x08 != 0,
        cal_toff: be16(regs, off::CAL_TOFF_MSB) as i16,
        cal_hoff: regs[off::CAL_HOFF] as i8,
        cal_again: regs[off::CAL_AGAIN],
        cal_atime: regs[off::CAL_ATIME],
        pwr_mode: PwrMode::decode(regs[off::PWR_MODE]),
    }
}

/// `IIO_UNMOD_EVENT_CODE(IIO_LIGHT, 0, IIO_EV_TYPE_THRESH, IIO_EV_DIR_EITHER)`
/// -- the only event code this driver ever pushes (see
/// `envcombo_irq_thread` in envcombo.c): chan_type=IIO_LIGHT(6) in bits
/// [39:32], every other field (type, direction, channel, modifier) zero.
pub const EXPECTED_EVENT_ID: u64 = 6u64 << 32;

pub fn describe_event_id(id: u64) -> String {
    if id == EXPECTED_EVENT_ID {
        "ALS threshold crossing (EITHER)".to_string()
    } else {
        format!("unrecognized event id 0x{id:016x}")
    }
}

/// Device discovered at `/sys/bus/iio/devices/iio:deviceN`.
#[derive(Clone, Debug)]
pub struct Device {
    pub base: String,    // .../iio:deviceN
    pub chardev: String, // /dev/iio:deviceN
}

/// Shell one-liner that finds the envcombo IIO device, run once after
/// connect. `-n 1`, not the GNU shorthand `-1` -- the target's BusyBox
/// `head` doesn't accept the latter.
pub const DISCOVER_CMD: &str =
    "grep -l envcombo /sys/bus/iio/devices/iio:device*/name 2>/dev/null | head -n 1";

/// Shell loop that kills every process whose `/proc/<pid>/comm` matches
/// `name`, signal 9. Deliberately not `killall`/`pkill -f`: BusyBox builds
/// vary in which of those applets they include at all (this image's has
/// neither `pkill` nor `pgrep`, discovered the hard way), whereas `/proc`,
/// `cat`, `kill` and basic shell are guaranteed.
fn kill_by_comm_cmd(name: &str) -> String {
    format!(
        "for p in /proc/[0-9]*; do \
           n=$(cat $p/comm 2>/dev/null); \
           [ \"$n\" = \"{name}\" ] && kill -9 ${{p#/proc/}} 2>/dev/null; \
         done; true"
    )
}

/// Forcibly kills any stray `dd` left holding `/dev/iio:deviceN` open from
/// a previous buffer-stream session. The IIO core only allows one opener
/// of the chardev at a time, and `dd` blocked in `read()` on the *device*
/// fd (not the SSH channel) doesn't notice the channel died until its next
/// I/O on the device -- which, if the buffer stalled, may never come. Run
/// this before opening the chardev for a new buffer stream so a leftover
/// `dd` from an earlier session can't cause a confusing EBUSY.
///
/// Targets only `dd`, never `envcombo-evtcat`: an event monitor may be
/// running at the same time and must not be collateral damage here (it
/// only needs the chardev briefly, to bootstrap its event fd -- see
/// kill_stray_evtcat_cmd's doc comment for the full picture).
pub fn kill_stray_dd_cmd() -> String {
    kill_by_comm_cmd("dd")
}

/// Same idea as `kill_stray_dd_cmd`, for a stale `envcombo-evtcat` left
/// over from a previous event-monitor session. Targets only
/// `envcombo-evtcat`, never `dd` -- a buffer stream may be running at the
/// same time and must not be killed just because the event monitor is
/// (re)starting.
///
/// Ordering note: the event monitor only needs `/dev/iio:deviceN` open
/// *briefly*, to issue `IIO_GET_EVENT_FD_IOCTL`, then it operates entirely
/// off the independent event fd that returns -- so starting the event
/// monitor first and the buffer stream second lets both run concurrently.
/// Starting the buffer stream first means its `dd` holds the chardev open
/// continuously, and the event monitor's later open() will fail with
/// EBUSY until that `dd` stops.
pub fn kill_stray_evtcat_cmd() -> String {
    kill_by_comm_cmd("envcombo-evtcat")
}

pub fn device_from_discovery(output: &str) -> Option<Device> {
    let name_path = output.trim();
    let base = name_path.strip_suffix("/name")?;
    if !valid_device_base(base) {
        return None;
    }
    let leaf = base.rsplit('/').next()?;
    Some(Device {
        base: base.to_string(),
        chardev: format!("/dev/{leaf}"),
    })
}

fn valid_device_base(base: &str) -> bool {
    base.strip_prefix("/sys/bus/iio/devices/iio:device")
        .is_some_and(|n| !n.is_empty() && n.chars().all(|c| c.is_ascii_digit()))
}

/// One remote round trip that reads every auto-refreshed attribute plus the
/// debugfs register snapshot. Sections are separated by a lone "---" line so
/// `parse_snapshot` can split a single blob of text back into fields.
pub fn snapshot_cmd(dev: &Device) -> String {
    let b = &dev.base;
    format!(
        "cat {b}/in_illuminance_raw 2>/dev/null; echo ---; \
         cat {b}/in_illuminance_scale 2>/dev/null; echo ---; \
         cat {b}/in_illuminance_hardwaregain 2>/dev/null; echo ---; \
         cat {b}/in_illuminance_hardwaregain_available 2>/dev/null; echo ---; \
         cat {b}/in_illuminance_integration_time 2>/dev/null; echo ---; \
         cat {b}/in_illuminance_integration_time_available 2>/dev/null; echo ---; \
         cat {b}/events/in_illuminance_thresh_rising_value 2>/dev/null; echo ---; \
         cat {b}/events/in_illuminance_thresh_falling_value 2>/dev/null; echo ---; \
         cat {b}/events/in_illuminance_thresh_either_en 2>/dev/null; echo ---; \
         cat {b}/scan_elements/in_illuminance_en 2>/dev/null; echo ---; \
         cat {b}/scan_elements/in_timestamp_en 2>/dev/null; echo ---; \
         cat {b}/buffer/enable 2>/dev/null; echo ---; \
         cat {b}/buffer/length 2>/dev/null; echo ---; \
         hexdump -ve '1/1 \"%02x\"' {regs} 2>/dev/null",
        b = b,
        regs = DEBUGFS_REGS
    )
}

#[derive(Default, Clone)]
pub struct Snapshot {
    pub raw: Option<u16>,
    pub scale: Option<f64>,
    pub gain: Option<u32>,
    pub gain_avail: Vec<u32>,
    pub int_time: Option<f64>,
    pub int_time_avail: Vec<f64>,
    pub thresh_rising: Option<u32>,
    pub thresh_falling: Option<u32>,
    pub either_en: Option<bool>,
    pub scan_illum_en: Option<bool>,
    pub scan_ts_en: Option<bool>,
    pub buffer_enabled: Option<bool>,
    pub buffer_len: Option<u32>,
    pub regs: Option<[u8; REG_COUNT]>,
}

fn parse_bool01(s: &str) -> Option<bool> {
    match s {
        "0" => Some(false),
        "1" => Some(true),
        _ => None,
    }
}

fn parse_hex_regs(s: &str) -> Option<[u8; REG_COUNT]> {
    if s.len() != REG_COUNT * 2 {
        return None;
    }
    let mut out = [0u8; REG_COUNT];
    for (i, byte) in out.iter_mut().enumerate() {
        *byte = u8::from_str_radix(&s[i * 2..i * 2 + 2], 16).ok()?;
    }
    Some(out)
}

pub fn parse_snapshot(text: &str) -> Snapshot {
    let parts: Vec<&str> = text.split("\n---\n").collect();
    let get = |i: usize| parts.get(i).map(|s| s.trim()).unwrap_or("");
    let list_u32 = |s: &str| s.split_whitespace().filter_map(|t| t.parse().ok()).collect();
    let list_f64 = |s: &str| s.split_whitespace().filter_map(|t| t.parse().ok()).collect();

    Snapshot {
        raw: get(0).parse().ok(),
        scale: get(1).parse().ok(),
        gain: get(2).parse().ok(),
        gain_avail: list_u32(get(3)),
        int_time: get(4).parse().ok(),
        int_time_avail: list_f64(get(5)),
        thresh_rising: get(6).parse().ok(),
        thresh_falling: get(7).parse().ok(),
        either_en: parse_bool01(get(8)),
        scan_illum_en: parse_bool01(get(9)),
        scan_ts_en: parse_bool01(get(10)),
        buffer_enabled: parse_bool01(get(11)),
        buffer_len: get(12).parse().ok(),
        regs: parse_hex_regs(get(13)),
    }
}
