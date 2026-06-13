// SPDX-License-Identifier: MIT
//! End-to-end test harness for the envcombo IIO driver.
//!
//! Exercises the driver through its userspace ABI (sysfs, the IIO chardev
//! and event interface) and cross-checks driver-side state against the
//! simulator's raw register file exposed in debugfs, so every assertion
//! can actually fail if the driver misbehaves.
//!
//! Each test is self-isolating: test_module_loading reloads both modules
//! for a deterministic starting point, and every device test calls
//! reset_baseline() first, so the suite does not depend on a fresh boot, on
//! earlier manual interaction, or on the order the tests run in.

use std::fmt::Write as _;
use std::fs;
use std::io::Read;
use std::os::unix::io::{AsRawFd, FromRawFd, OwnedFd};
use std::path::{Path, PathBuf};
use std::process::Command;
use std::thread::sleep;
use std::time::{Duration, Instant};

// Simulator register file layout (debugfs blob, one byte per register).
const REG_CFG: usize = 0x06;
const REG_INT_CFG: usize = 0x07;
const REG_ALS_TH_LOW_MSB: usize = 0x08;
const REG_ALS_TH_HIGH_MSB: usize = 0x0A;
const REG_CAL_ALS_GAIN: usize = 0x10;
const REG_CAL_ALS_TIME: usize = 0x11;
const REG_PWR_MODE: usize = 0x12;
const REG_COUNT: usize = 0x13;

// CFG/INT_CFG bits the driver provisions at probe.
const CFG_ALS_EN: u8 = 0x80;
const INT_CFG_EN: u8 = 0x80;
const INT_CFG_LATCH: u8 = 0x40;

const PWR_MODE_MASK: u8 = 0x03;
const PWR_SLEEP: u8 = 0x01;
const PWR_CONTINUOUS: u8 = 0x03;

const CFG_GAIN_SHIFT: u8 = 3;
const CFG_GAIN_MASK: u8 = 0x03 << CFG_GAIN_SHIFT;
const CFG_TIME_SHIFT: u8 = 1;
const CFG_TIME_MASK: u8 = 0x03 << CFG_TIME_SHIFT;

// With the driver defaults (gain x1, integration time 200 ms) the simulator
// produces DIV_ROUND_CLOSEST(ticks, 4) for ticks in 0..=999, i.e. at most 250.
const MAX_RAW_DEFAULT_CFG: u16 = 250;

// Continuous-mode conversion period of the simulator.
const CONV_PERIOD: Duration = Duration::from_millis(200);

// From the kernel UAPI (include/uapi/linux/iio): _IOR('i', 0x90, int).
const IIO_GET_EVENT_FD_IOCTL: libc::c_ulong = 0x8004_6990;
// IIO_UNMOD_EVENT_CODE(IIO_LIGHT=6, chan 0, IIO_EV_TYPE_THRESH=0,
// IIO_EV_DIR_EITHER=0): only the channel type field is non-zero.
const EXPECTED_EVENT_ID: u64 = 6u64 << 32;

const SIM_MODULE: &str = "i2c-envcombo-sim";
const DRV_MODULE: &str = "envcombo";
const DEBUGFS_REGS: &str = "/sys/kernel/debug/envcombo-sim/regs";

type TestResult = Result<(), String>;

macro_rules! check {
    ($cond:expr, $($msg:tt)+) => {
        if !($cond) {
            return Err(format!($($msg)+));
        }
    };
}

/// Tiny seeded PRNG (splitmix64) so the test order can be shuffled
/// reproducibly without pulling in an external crate (the Yocto build is
/// offline).
struct Rng(u64);

impl Rng {
    fn new(seed: u64) -> Self {
        Rng(seed)
    }
    fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }
    /// Index in 0..n (n must be > 0).
    fn below(&mut self, n: usize) -> usize {
        (self.next_u64() % n as u64) as usize
    }
}

/// Seed for the test-order shuffle: `--seed=N`, else `ENVCOMBO_TEST_SEED=N`,
/// else derived from the clock. Always printed so any failing ordering can
/// be replayed.
fn parse_seed() -> u64 {
    for arg in std::env::args().skip(1) {
        if let Some(v) = arg.strip_prefix("--seed=") {
            if let Ok(n) = v.parse() {
                return n;
            }
        }
    }
    if let Ok(v) = std::env::var("ENVCOMBO_TEST_SEED") {
        if let Ok(n) = v.parse() {
            return n;
        }
    }
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_nanos() as u64)
        .unwrap_or(0x9E37_79B9_7F4A_7C15)
}

fn main() {
    let tests: &[(&str, fn(&mut Ctx) -> TestResult)] = &[
        ("module-loading", test_module_loading),
        ("direct-als-read", test_direct_read),
        ("gain-and-integration-time", test_gain_and_time),
        ("gain-response", test_gain_response),
        ("time-response", test_time_response),
        ("factory-gain-cal", test_factory_gain_cal),
        ("factory-atime-override", test_factory_atime_override),
        ("threshold-attributes", test_threshold_attrs),
        ("threshold-events", test_events),
        ("triggered-buffer", test_buffer),
        ("buffer-no-timestamp", test_buffer_no_timestamp),
        ("buffer-timestamp-accuracy", test_timestamp),
        ("power-fsm-transitions", test_power_fsm),
        ("module-unloading", test_unload),
        ("kernel-log-health", test_kernel_log),
    ];

    // module-loading (first) discovers the device and forces a clean slate;
    // module-unloading and kernel-log-health (last two) must run after every
    // device test. Only the device tests in between are order-independent, so
    // those are the ones we shuffle -- a reproducible way to prove the
    // per-test isolation holds regardless of order. Replay a specific order
    // with --seed=N or ENVCOMBO_TEST_SEED=N.
    let seed = parse_seed();
    let mut order: Vec<usize> = (0..tests.len()).collect();
    let (mid_start, mid_end) = (1, tests.len() - 2);
    let mut rng = Rng::new(seed);
    for i in (mid_start + 1..mid_end).rev() {
        let j = mid_start + rng.below(i - mid_start + 1);
        order.swap(i, j);
    }

    let mut ctx = Ctx::default();
    let mut failures = 0;

    println!(
        "envcombo driver test harness ({} tests, seed {})",
        tests.len(),
        seed
    );
    for (pos, &idx) in order.iter().enumerate() {
        let (name, test) = tests[idx];
        match test(&mut ctx) {
            Ok(()) => println!("[{}/{}] {:<28} PASS", pos + 1, tests.len(), name),
            Err(e) => {
                failures += 1;
                println!("[{}/{}] {:<28} FAIL: {}", pos + 1, tests.len(), name, e);
            }
        }
    }

    if failures == 0 {
        println!("RESULT: all {} tests passed (seed {})", tests.len(), seed);
    } else {
        println!(
            "RESULT: {} of {} tests FAILED (seed {})",
            failures,
            tests.len(),
            seed
        );
    }
    std::process::exit(if failures == 0 { 0 } else { 1 });
}

#[derive(Default)]
struct Ctx {
    /// /sys/bus/iio/devices/iio:deviceN for the envcombo device.
    dev: PathBuf,
    /// Matching /dev/iio:deviceN chardev.
    chardev: PathBuf,
    /// Name of the driver's own trigger ("envcombo-devN").
    trigger: String,
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

fn sysfs_read(path: &Path) -> Result<String, String> {
    fs::read_to_string(path)
        .map(|s| s.trim().to_string())
        .map_err(|e| format!("read {}: {}", path.display(), e))
}

fn sysfs_write(path: &Path, val: &str) -> Result<(), String> {
    fs::write(path, val).map_err(|e| format!("write {} <- {:?}: {}", path.display(), val, e))
}

fn attr(ctx: &Ctx, name: &str) -> PathBuf {
    ctx.dev.join(name)
}

fn read_u32(path: &Path) -> Result<u32, String> {
    let s = sysfs_read(path)?;
    s.parse()
        .map_err(|e| format!("{}: bad integer {:?}: {}", path.display(), s, e))
}

fn read_f64(path: &Path) -> Result<f64, String> {
    let s = sysfs_read(path)?;
    s.parse()
        .map_err(|e| format!("{}: bad float {:?}: {}", path.display(), s, e))
}

fn run(cmd: &str, args: &[&str]) -> Result<(), String> {
    let out = Command::new(cmd)
        .args(args)
        .output()
        .map_err(|e| format!("spawn {}: {}", cmd, e))?;
    if out.status.success() {
        Ok(())
    } else {
        Err(format!(
            "{} {} failed: {}",
            cmd,
            args.join(" "),
            String::from_utf8_lossy(&out.stderr).trim()
        ))
    }
}

fn module_loaded(name: &str) -> bool {
    Path::new("/sys/module").join(name.replace('-', "_")).exists()
}

/// Raw simulator register dump; reading it has no side effects (unlike a
/// STATUS read over I2C), so it is safe to use for state assertions.
fn sim_regs() -> Result<[u8; REG_COUNT], String> {
    let mut buf = [0u8; REG_COUNT];
    let mut f = fs::File::open(DEBUGFS_REGS)
        .map_err(|e| format!("open {}: {} (is debugfs mounted?)", DEBUGFS_REGS, e))?;
    f.read_exact(&mut buf)
        .map_err(|e| format!("read {}: {}", DEBUGFS_REGS, e))?;
    Ok(buf)
}

fn sim_reg16(regs: &[u8; REG_COUNT], msb: usize) -> u16 {
    ((regs[msb] as u16) << 8) | regs[msb + 1] as u16
}

fn power_mode() -> Result<u8, String> {
    Ok(sim_regs()?[REG_PWR_MODE] & PWR_MODE_MASK)
}

fn expect_power_mode(expected: u8, when: &str) -> TestResult {
    let mode = power_mode()?;
    check!(
        mode == expected,
        "power mode {} {:#04x}, expected {:#04x}",
        when,
        mode,
        expected
    );
    Ok(())
}

/// Poll a non-blocking fd and read whatever is available into `buf`.
/// Ok(None) on timeout, a wakeup without POLLIN, or EAGAIN; Ok(Some(n))
/// when bytes were read. EINTR is retried, error conditions reported
/// distinctly so they cannot masquerade as data or flaky failures.
fn poll_read_nonblock(
    fd: libc::c_int,
    buf: &mut [u8],
    timeout_ms: libc::c_int,
) -> Result<Option<usize>, String> {
    let mut pfd = libc::pollfd {
        fd,
        events: libc::POLLIN,
        revents: 0,
    };
    let ret = loop {
        let ret = unsafe { libc::poll(&mut pfd, 1, timeout_ms) };
        let err = std::io::Error::last_os_error();
        if ret < 0 && err.kind() == std::io::ErrorKind::Interrupted {
            continue;
        }
        check!(ret >= 0, "poll: {}", err);
        break ret;
    };
    if ret == 0 {
        return Ok(None);
    }
    check!(
        pfd.revents & (libc::POLLERR | libc::POLLHUP | libc::POLLNVAL) == 0,
        "fd error condition (revents {:#x})",
        pfd.revents
    );
    if pfd.revents & libc::POLLIN == 0 {
        return Ok(None);
    }

    loop {
        let n = unsafe { libc::read(fd, buf.as_mut_ptr().cast(), buf.len()) };
        if n >= 0 {
            return Ok(Some(n as usize));
        }
        let err = std::io::Error::last_os_error();
        match err.kind() {
            std::io::ErrorKind::Interrupted => continue,
            std::io::ErrorKind::WouldBlock => return Ok(None),
            _ => return Err(format!("read: {}", err)),
        }
    }
}

/// Read a POSIX clock as nanoseconds, matching the i64 ns units the IIO
/// core stamps buffered samples with.
fn clock_now_ns(clk: libc::clockid_t) -> Result<i64, String> {
    let mut ts = libc::timespec {
        tv_sec: 0,
        tv_nsec: 0,
    };
    let ret = unsafe { libc::clock_gettime(clk, &mut ts) };
    check!(
        ret == 0,
        "clock_gettime({}): {}",
        clk,
        std::io::Error::last_os_error()
    );
    Ok(ts.tv_sec as i64 * 1_000_000_000 + ts.tv_nsec as i64)
}

/// The clock the IIO core uses to stamp this device's samples, so the test
/// can compare timestamps against the *same* reference the driver does.
/// Defaults to realtime (the IIO core default) when the attribute is absent.
fn iio_timestamp_clock(ctx: &Ctx) -> Result<(libc::clockid_t, String), String> {
    let path = attr(ctx, "current_timestamp_clock");
    let name = if path.exists() {
        sysfs_read(&path)?
    } else {
        "realtime".to_string()
    };
    let id = match name.as_str() {
        "realtime" => libc::CLOCK_REALTIME,
        "realtime_coarse" => libc::CLOCK_REALTIME_COARSE,
        "monotonic" => libc::CLOCK_MONOTONIC,
        "monotonic_raw" => libc::CLOCK_MONOTONIC_RAW,
        "monotonic_coarse" => libc::CLOCK_MONOTONIC_COARSE,
        "boottime" => libc::CLOCK_BOOTTIME,
        "tai" => libc::CLOCK_TAI,
        other => return Err(format!("unknown timestamp clock {:?}", other)),
    };
    Ok((id, name))
}

fn find_iio_device() -> Result<(PathBuf, PathBuf), String> {
    let dir = fs::read_dir("/sys/bus/iio/devices")
        .map_err(|e| format!("read /sys/bus/iio/devices: {}", e))?;
    for entry in dir.flatten() {
        let name = entry.file_name().to_string_lossy().into_owned();
        if !name.starts_with("iio:device") {
            continue;
        }
        if sysfs_read(&entry.path().join("name")).as_deref() == Ok("envcombo") {
            return Ok((entry.path(), PathBuf::from("/dev").join(name)));
        }
    }
    Err("no IIO device named \"envcombo\" found".into())
}

fn find_trigger() -> Result<String, String> {
    let dir = fs::read_dir("/sys/bus/iio/devices")
        .map_err(|e| format!("read /sys/bus/iio/devices: {}", e))?;
    for entry in dir.flatten() {
        if !entry.file_name().to_string_lossy().starts_with("trigger") {
            continue;
        }
        let name = sysfs_read(&entry.path().join("name"))?;
        if name.starts_with("envcombo-dev") {
            return Ok(name);
        }
    }
    Err("no envcombo-dev* trigger found".into())
}

/// Wait for the IIO device to (dis)appear after module load/unload.
fn wait_device(present: bool, timeout: Duration) -> Result<Option<(PathBuf, PathBuf)>, String> {
    let deadline = Instant::now() + timeout;
    loop {
        match find_iio_device() {
            Ok(found) if present => return Ok(Some(found)),
            Err(_) if !present => return Ok(None),
            _ => {}
        }
        if Instant::now() >= deadline {
            return Err(format!(
                "IIO device still {} after {:?}",
                if present { "absent" } else { "present" },
                timeout
            ));
        }
        sleep(Duration::from_millis(100));
    }
}

/// Force both modules to reload so the suite starts from a deterministic
/// state regardless of prior boot or manual activity. Reloading the driver
/// resets its CFG to the probe defaults; reloading the simulator clears its
/// internal latches (the ALS threshold edge-state in particular, which no
/// sysfs write can reach).
fn reload_modules() -> TestResult {
    // Best-effort: detach the trigger and stop the buffer first so no
    // consumer reference keeps the driver module busy across rmmod.
    if let Ok((dev, _)) = find_iio_device() {
        let _ = sysfs_write(&dev.join("buffer/enable"), "0");
        let _ = sysfs_write(&dev.join("trigger/current_trigger"), "\n");
    }
    if module_loaded(DRV_MODULE) {
        run("rmmod", &[DRV_MODULE])?;
        wait_device(false, Duration::from_secs(5))?;
    }
    if module_loaded(SIM_MODULE) {
        run("rmmod", &[SIM_MODULE])?;
    }
    run("modprobe", &[SIM_MODULE])?;
    run("modprobe", &[DRV_MODULE])?;
    Ok(())
}

/// Set one simulator register byte through the debugfs blob. Used to inject
/// factory-calibration values (CAL_AGAIN/CAL_ATIME) that have no sysfs path.
/// Read-modify-write of the whole 19-byte image so no seek is needed; the
/// reconstructed ALS/threshold/STATUS bytes are ignored on the write side, so
/// rewriting them is harmless.
fn sim_write_reg(offset: usize, val: u8) -> TestResult {
    use std::io::Write;
    let mut regs = sim_regs()?;
    regs[offset] = val;
    let mut f = fs::OpenOptions::new()
        .write(true)
        .open(DEBUGFS_REGS)
        .map_err(|e| format!("open {} for write: {}", DEBUGFS_REGS, e))?;
    f.write_all(&regs)
        .map_err(|e| format!("write {} <- reg[{:#04x}]={:#04x}: {}", DEBUGFS_REGS, offset, val, e))
}

/// Reload only the driver (the simulator, and any debugfs register edits made
/// to it, stay in place) so it re-probes and re-reads the factory-calibration
/// registers, then re-discover the possibly-renumbered IIO device into `ctx`.
fn reload_driver(ctx: &mut Ctx) -> TestResult {
    if let Ok((dev, _)) = find_iio_device() {
        let _ = sysfs_write(&dev.join("buffer/enable"), "0");
        let _ = sysfs_write(&dev.join("trigger/current_trigger"), "\n");
    }
    if module_loaded(DRV_MODULE) {
        run("rmmod", &[DRV_MODULE])?;
        wait_device(false, Duration::from_secs(5))?;
    }
    run("modprobe", &[DRV_MODULE])?;
    let (dev, chardev) =
        wait_device(true, Duration::from_secs(5))?.expect("wait_device(present) returned device");
    ctx.trigger = find_trigger()?;
    ctx.dev = dev;
    ctx.chardev = chardev;
    Ok(())
}

/// Bring the device to a known baseline so each test is independent of the
/// order tests run in and of any state earlier tests (or manual poking) left
/// behind: consumers off, default gain/time, full-range (effectively
/// disabled) thresholds. sysfs only -- the simulator's edge latch is handled
/// by reload_modules() at startup and by test_events itself.
fn reset_baseline(ctx: &Ctx) -> TestResult {
    // Drop any active consumers first so the device returns to SLEEP and
    // scan-element writes are permitted again.
    let _ = sysfs_write(&attr(ctx, "buffer/enable"), "0");
    let _ = sysfs_write(&attr(ctx, "events/in_illuminance_thresh_either_en"), "0");
    let _ = sysfs_write(&attr(ctx, "scan_elements/in_illuminance_en"), "0");
    let _ = sysfs_write(&attr(ctx, "scan_elements/in_timestamp_en"), "0");

    // Probe defaults: gain x1, integration time 200 ms.
    sysfs_write(&attr(ctx, "in_illuminance_hardwaregain"), "1")?;
    sysfs_write(&attr(ctx, "in_illuminance_integration_time"), "0.2")?;

    // Open the threshold window fully. Raise the high bound before lowering
    // the low bound so the window stays well-formed at every step.
    sysfs_write(&attr(ctx, "events/in_illuminance_thresh_rising_value"), "65535")?;
    sysfs_write(&attr(ctx, "events/in_illuminance_thresh_falling_value"), "0")?;
    Ok(())
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

fn test_module_loading(ctx: &mut Ctx) -> TestResult {
    // Start from a guaranteed-clean slate so the suite does not depend on a
    // fresh boot or on whatever state earlier manual interaction left behind.
    reload_modules()?;
    for module in [SIM_MODULE, DRV_MODULE] {
        check!(module_loaded(module), "module {} did not load", module);
    }

    // debugfs is needed for register-level assertions throughout.
    if !Path::new(DEBUGFS_REGS).exists() {
        let _ = run("mount", &["-t", "debugfs", "debugfs", "/sys/kernel/debug"]);
    }
    sim_regs()?;

    let (dev, chardev) = wait_device(true, Duration::from_secs(5))?
        .expect("wait_device(present) returned device");
    check!(chardev.exists(), "{} missing", chardev.display());
    ctx.trigger = find_trigger()?;
    ctx.dev = dev;
    ctx.chardev = chardev;

    // Probe must leave the device idle in SLEEP.
    expect_power_mode(PWR_SLEEP, "after probe")?;

    // Probe must also provision the device registers: interrupt enabled and
    // latched, and CFG with ALS enabled at the default gain/time indices
    // (gain x1 = idx 0, 200 ms = idx 2 -> 0x80 | (2 << 1) = 0x84).
    let regs = sim_regs()?;
    check!(
        regs[REG_INT_CFG] == INT_CFG_EN | INT_CFG_LATCH,
        "INT_CFG {:#04x} after probe, expected {:#04x}",
        regs[REG_INT_CFG],
        INT_CFG_EN | INT_CFG_LATCH
    );
    check!(
        regs[REG_CFG] == CFG_ALS_EN | (2 << CFG_TIME_SHIFT),
        "CFG {:#04x} after probe, expected {:#04x} (ALS on, gain x1, 200 ms)",
        regs[REG_CFG],
        CFG_ALS_EN | (2 << CFG_TIME_SHIFT)
    );
    Ok(())
}

fn test_direct_read(ctx: &mut Ctx) -> TestResult {
    reset_baseline(ctx)?;
    let raw_attr = attr(ctx, "in_illuminance_raw");

    // Each sysfs read runs a one-shot conversion; the device must be back
    // in SLEEP afterwards, and values must stay in the range the default
    // gain/time configuration allows.
    let mut samples = Vec::new();
    for _ in 0..3 {
        let v = read_u32(&raw_attr)?;
        check!(
            v <= MAX_RAW_DEFAULT_CFG as u32,
            "raw value {} exceeds max {} for default config",
            v,
            MAX_RAW_DEFAULT_CFG
        );
        expect_power_mode(PWR_SLEEP, "after one-shot read")?;
        samples.push(v);
        sleep(Duration::from_millis(600));
    }

    // The simulated light level ramps continuously; three reads 600 ms
    // apart returning the same value means conversions are not running.
    check!(
        !(samples[0] == samples[1] && samples[1] == samples[2]),
        "three spaced reads all returned {}; one-shot conversions appear stale",
        samples[0]
    );
    Ok(())
}

fn test_gain_and_time(ctx: &mut Ctx) -> TestResult {
    reset_baseline(ctx)?;
    let gain_attr = attr(ctx, "in_illuminance_hardwaregain");
    let time_attr = attr(ctx, "in_illuminance_integration_time");

    let avail = sysfs_read(&attr(ctx, "in_illuminance_hardwaregain_available"))?;
    check!(avail == "1 4 16 64", "gain_available {:?}", avail);

    // Valid gain write must be visible in sysfs, in the device CFG
    // register, and in the reported scale (scale = 1/gain).
    sysfs_write(&gain_attr, "4")?;
    check!(read_u32(&gain_attr)? == 4, "gain readback != 4");
    let cfg = sim_regs()?[REG_CFG];
    check!(
        (cfg & CFG_GAIN_MASK) >> CFG_GAIN_SHIFT == 1,
        "CFG {:#04x}: gain field not set to index 1",
        cfg
    );
    let scale = read_f64(&attr(ctx, "in_illuminance_scale"))?;
    check!((scale - 0.25).abs() < 1e-6, "scale {} != 0.25 at gain 4", scale);

    // scale must be 1/gain for every supported gain (CAL_AGAIN defaults to 1).
    for (g, expect) in [(1, 1.0), (4, 0.25), (16, 0.0625), (64, 0.015_625)] {
        sysfs_write(&gain_attr, &g.to_string())?;
        let s = read_f64(&attr(ctx, "in_illuminance_scale"))?;
        check!((s - expect).abs() < 1e-9, "scale {} != {} at gain {}", s, expect, g);
    }
    sysfs_write(&gain_attr, "4")?; // restore for the rejected-write check below

    // Invalid gain must be rejected and leave state untouched.
    check!(
        sysfs_write(&gain_attr, "3").is_err(),
        "gain write of unsupported value 3 succeeded"
    );
    check!(read_u32(&gain_attr)? == 4, "gain changed by rejected write");

    let avail = sysfs_read(&attr(ctx, "in_illuminance_integration_time_available"))?;
    let times: Vec<f64> = avail
        .split_whitespace()
        .map(|t| t.parse().map_err(|e| format!("bad time {:?}: {}", t, e)))
        .collect::<Result<_, _>>()?;
    check!(
        times == [0.05, 0.1, 0.2, 0.4],
        "integration_time_available {:?}",
        avail
    );

    sysfs_write(&time_attr, "0.4")?;
    check!(
        (read_f64(&time_attr)? - 0.4).abs() < 1e-6,
        "integration_time readback != 0.4"
    );
    let cfg = sim_regs()?[REG_CFG];
    check!(
        (cfg & CFG_TIME_MASK) >> CFG_TIME_SHIFT == 3,
        "CFG {:#04x}: time field not set to index 3",
        cfg
    );

    check!(
        sysfs_write(&time_attr, "0.075").is_err(),
        "integration_time write of unsupported 0.075 succeeded"
    );
    check!(
        (read_f64(&time_attr)? - 0.4).abs() < 1e-6,
        "integration_time changed by rejected write"
    );

    // No explicit restore needed: reset_baseline() re-establishes the
    // defaults at the start of every test.
    Ok(())
}

/// Functional gain check. test_gain_and_time proves the gain *setting*
/// reaches the CFG register and the reported scale, but not that gain
/// actually amplifies the measurement. Here we sweep all gains against live
/// readings and assert two complementary properties:
///   1. the raw value grows with gain (the simulator's reading is
///      proportional to gain), catching a gain that is ignored or inverted;
///   2. the gain-compensated value `raw * scale` is gain-independent, i.e.
///      the reported scale correctly undoes the gain.
/// A one-shot conversion is latched synchronously during the I2C write, so a
/// back-to-back sweep sees an essentially frozen signal. The only degenerate
/// ramp positions -- sitting at zero, or wrapping mid-sweep -- show up as a
/// non-increasing raw sequence, so we briefly retry those rather than risk a
/// flaky failure; a genuine gain fault is non-increasing on *every* attempt
/// and still fails once the retries are exhausted.
fn test_gain_response(ctx: &mut Ctx) -> TestResult {
    reset_baseline(ctx)?; // gain 1, integration time 200 ms, thresholds open
    let gain_attr = attr(ctx, "in_illuminance_hardwaregain");
    let scale_attr = attr(ctx, "in_illuminance_scale");
    let raw_attr = attr(ctx, "in_illuminance_raw");
    const GAINS: [u32; 4] = [1, 4, 16, 64];

    // One burst of one-shot reads, one per gain, taken as close together as
    // possible so the ramp barely moves between them. Returns (gain, raw,
    // gain-compensated value) per gain.
    let sweep = || -> Result<Vec<(u32, u32, f64)>, String> {
        let mut out = Vec::with_capacity(GAINS.len());
        for g in GAINS {
            sysfs_write(&gain_attr, &g.to_string())?;
            let scale = read_f64(&scale_attr)?;
            let raw = read_u32(&raw_attr)?;
            out.push((g, raw, raw as f64 * scale));
        }
        Ok(out)
    };
    let increasing = |d: &[(u32, u32, f64)]| d.windows(2).all(|w| w[1].1 > w[0].1);

    let mut data = sweep()?;
    for _ in 0..5 {
        if increasing(&data) {
            break;
        }
        sleep(Duration::from_millis(60)); // advance the ramp off the boundary
        data = sweep()?;
    }
    check!(
        increasing(&data),
        "raw did not increase with gain: {:?}",
        data.iter().map(|&(g, r, _)| (g, r)).collect::<Vec<_>>()
    );

    // scale must undo the gain: every compensated reading lands on the same
    // physical value (spread bounded by per-gain rounding plus the tiny ramp
    // drift across the burst).
    let min = data.iter().map(|&(_, _, c)| c).fold(f64::INFINITY, f64::min);
    let max = data.iter().map(|&(_, _, c)| c).fold(f64::NEG_INFINITY, f64::max);
    check!(
        max - min <= 3.0,
        "gain-compensated readings disagree by {:.2} (raw*scale per gain: {:?})",
        max - min,
        data.iter().map(|&(g, _, c)| (g, c)).collect::<Vec<_>>()
    );

    expect_power_mode(PWR_SLEEP, "after gain-sweep one-shot reads")?;
    Ok(())
}

/// Functional integration-time check, the time analogue of test_gain_response.
/// gain-and-integration-time proves the time *setting* lands in CFG; this
/// proves it changes the measurement: the simulator divides the reading by
/// time_ms/50, so a longer integration time yields a smaller raw value, and
/// `raw * (time_ms / 50)` recovers the same underlying signal regardless of
/// time. Degenerate ramp positions surface as a non-decreasing raw sequence
/// and are retried, exactly as in the gain test.
fn test_time_response(ctx: &mut Ctx) -> TestResult {
    reset_baseline(ctx)?; // gain 1, integration time 200 ms
    let time_attr = attr(ctx, "in_illuminance_integration_time");
    let raw_attr = attr(ctx, "in_illuminance_raw");
    // (sysfs value, divisor = time_ms / 50), ascending in time.
    const TIMES: [(&str, u32); 4] = [("0.05", 1), ("0.1", 2), ("0.2", 4), ("0.4", 8)];

    let sweep = || -> Result<Vec<(u32, u32, f64)>, String> {
        let mut out = Vec::with_capacity(TIMES.len());
        for (t, div) in TIMES {
            sysfs_write(&time_attr, t)?;
            let raw = read_u32(&raw_attr)?;
            out.push((div, raw, raw as f64 * div as f64));
        }
        Ok(out)
    };
    // Longer time -> smaller reading, so raw must strictly decrease.
    let decreasing = |d: &[(u32, u32, f64)]| d.windows(2).all(|w| w[1].1 < w[0].1);

    let mut data = sweep()?;
    for _ in 0..8 {
        if decreasing(&data) {
            break;
        }
        sleep(Duration::from_millis(100)); // climb the ramp clear of rounding ties
        data = sweep()?;
    }
    check!(
        decreasing(&data),
        "raw did not decrease with integration time: {:?}",
        data.iter().map(|&(div, r, _)| (div, r)).collect::<Vec<_>>()
    );

    // raw * (time/50) recovers the same signal. The slack is wider than the
    // gain test's: the longest time divides by 8, so its rounding error is
    // amplified by 8 when un-scaled.
    let min = data.iter().map(|&(_, _, c)| c).fold(f64::INFINITY, f64::min);
    let max = data.iter().map(|&(_, _, c)| c).fold(f64::NEG_INFINITY, f64::max);
    check!(
        max - min <= 8.0,
        "time-compensated readings disagree by {:.2} (raw*div per time: {:?})",
        max - min,
        data.iter().map(|&(div, _, c)| (div, c)).collect::<Vec<_>>()
    );

    expect_power_mode(PWR_SLEEP, "after integration-time sweep reads")?;
    Ok(())
}

/// Factory gain calibration (CAL_AGAIN). The effective gain -- and therefore
/// the reported scale -- is the configured gain times CAL_AGAIN, but every
/// other test runs with CAL_AGAIN = 1 so the multiply is never exercised.
/// Inject a multiplier via debugfs, reload the driver so it re-reads the
/// factory register at probe, and confirm scale folds it in. The override is
/// always undone and the driver reloaded so the rest of the suite sees a
/// normal device.
fn test_factory_gain_cal(ctx: &mut Ctx) -> TestResult {
    const MULT: u32 = 2;
    sim_write_reg(REG_CAL_ALS_GAIN, MULT as u8)?;
    let result = (|| -> TestResult {
        reload_driver(ctx)?;
        reset_baseline(ctx)?; // configured gain x1 -> effective gain = MULT
        let scale_attr = attr(ctx, "in_illuminance_scale");

        let scale = read_f64(&scale_attr)?;
        check!(
            (scale - 1.0 / MULT as f64).abs() < 1e-9,
            "scale {} != {} with CAL_AGAIN={} at gain 1",
            scale,
            1.0 / MULT as f64,
            MULT
        );
        // At configured gain 4 the effective gain is 4*MULT -> scale 1/(4*MULT).
        sysfs_write(&attr(ctx, "in_illuminance_hardwaregain"), "4")?;
        let scale = read_f64(&scale_attr)?;
        check!(
            (scale - 1.0 / (4 * MULT) as f64).abs() < 1e-9,
            "scale {} != {} with CAL_AGAIN={} at gain 4",
            scale,
            1.0 / (4 * MULT) as f64,
            MULT
        );
        Ok(())
    })();
    // Always restore, even if the body failed, so later tests are unaffected.
    let restore_res = sim_write_reg(REG_CAL_ALS_GAIN, 1).and_then(|()| reload_driver(ctx));
    match (result, restore_res) {
        (Ok(()), Ok(())) => Ok(()),
        (Err(e), Ok(())) => Err(e),
        (Ok(()), Err(r)) => Err(format!("restore failed: {}", r)),
        (Err(e), Err(r)) => Err(format!("{}; restore failed: {}", e, r)),
    }
}

/// Factory integration-time override (CAL_ATIME). A non-zero CAL_ATIME at
/// probe must fix the integration time to that value and make it read-only,
/// with the available list collapsed to the single forced value. The whole
/// branch is otherwise only exercised by hand, so inject it via debugfs,
/// reload the driver, check the user-visible effects, then restore.
fn test_factory_atime_override(ctx: &mut Ctx) -> TestResult {
    sim_write_reg(REG_CAL_ALS_TIME, 100)?; // force 100 ms
    let result = (|| -> TestResult {
        reload_driver(ctx)?;
        let time_attr = attr(ctx, "in_illuminance_integration_time");

        check!(
            (read_f64(&time_attr)? - 0.1).abs() < 1e-6,
            "integration_time {:?} not forced to 0.1 by CAL_ATIME",
            sysfs_read(&time_attr)?
        );
        let avail = sysfs_read(&attr(ctx, "in_illuminance_integration_time_available"))?;
        check!(
            avail == "0.100000",
            "integration_time_available {:?}, expected only the forced 0.100000",
            avail
        );
        // Read-only: a write must be refused and leave the value unchanged.
        check!(
            sysfs_write(&time_attr, "0.2").is_err(),
            "integration_time write accepted despite factory override"
        );
        check!(
            (read_f64(&time_attr)? - 0.1).abs() < 1e-6,
            "integration_time changed by a write that should have been refused"
        );
        Ok(())
    })();
    let restore = sim_write_reg(REG_CAL_ALS_TIME, 0).and_then(|()| reload_driver(ctx));
    result?;
    restore
}

fn test_threshold_attrs(ctx: &mut Ctx) -> TestResult {
    reset_baseline(ctx)?;
    let rising = attr(ctx, "events/in_illuminance_thresh_rising_value");
    let falling = attr(ctx, "events/in_illuminance_thresh_falling_value");

    sysfs_write(&falling, "50")?;
    sysfs_write(&rising, "200")?;
    check!(read_u32(&falling)? == 50, "falling readback != 50");
    check!(read_u32(&rising)? == 200, "rising readback != 200");

    // The values must land in the device threshold registers (big-endian).
    let regs = sim_regs()?;
    let (lo, hi) = (
        sim_reg16(&regs, REG_ALS_TH_LOW_MSB),
        sim_reg16(&regs, REG_ALS_TH_HIGH_MSB),
    );
    check!(lo == 50, "device TH_LOW {} != 50", lo);
    check!(hi == 200, "device TH_HIGH {} != 200", hi);

    // Invalid configurations must be rejected without side effects:
    // inverted window, out-of-range, negative.
    check!(
        sysfs_write(&rising, "10").is_err(),
        "rising 10 below falling 50 was accepted"
    );
    check!(
        sysfs_write(&falling, "300").is_err(),
        "falling 300 above rising 200 was accepted"
    );
    check!(
        sysfs_write(&rising, "70000").is_err(),
        "rising 70000 above 16-bit range was accepted"
    );
    check!(sysfs_write(&falling, "-1").is_err(), "falling -1 was accepted");

    let regs = sim_regs()?;
    check!(
        sim_reg16(&regs, REG_ALS_TH_LOW_MSB) == 50
            && sim_reg16(&regs, REG_ALS_TH_HIGH_MSB) == 200,
        "device thresholds changed by rejected writes"
    );

    // The boundary case low == high is a valid (degenerate) window and must
    // be accepted: equality is allowed, only a strict inversion is rejected.
    sysfs_write(&falling, "100")?;
    sysfs_write(&rising, "100")?;
    check!(read_u32(&falling)? == 100, "falling readback != 100 (low == high)");
    check!(read_u32(&rising)? == 100, "rising readback != 100 (low == high)");
    let regs = sim_regs()?;
    check!(
        sim_reg16(&regs, REG_ALS_TH_LOW_MSB) == 100
            && sim_reg16(&regs, REG_ALS_TH_HIGH_MSB) == 100,
        "device thresholds not set to the degenerate low == high window"
    );
    Ok(())
}

/// Get the IIO event fd for the device.
fn open_event_fd(chardev: &Path) -> Result<OwnedFd, String> {
    let dev = fs::File::open(chardev).map_err(|e| format!("open {}: {}", chardev.display(), e))?;
    let mut event_fd: libc::c_int = -1;
    let ret = unsafe { libc::ioctl(dev.as_raw_fd(), IIO_GET_EVENT_FD_IOCTL, &mut event_fd) };
    check!(
        ret == 0 && event_fd >= 0,
        "IIO_GET_EVENT_FD_IOCTL failed: {}",
        std::io::Error::last_os_error()
    );
    Ok(unsafe { OwnedFd::from_raw_fd(event_fd) })
}

/// Wait for one iio_event_data record; Ok(None) on timeout.
fn read_event(fd: &OwnedFd, timeout: Duration) -> Result<Option<(u64, i64)>, String> {
    let mut pfd = libc::pollfd {
        fd: fd.as_raw_fd(),
        events: libc::POLLIN,
        revents: 0,
    };
    let ret = loop {
        let ret = unsafe { libc::poll(&mut pfd, 1, timeout.as_millis() as libc::c_int) };
        let err = std::io::Error::last_os_error();
        if ret < 0 && err.kind() == std::io::ErrorKind::Interrupted {
            continue;
        }
        check!(ret >= 0, "poll: {}", err);
        break ret;
    };
    if ret == 0 {
        return Ok(None);
    }
    check!(
        pfd.revents & (libc::POLLERR | libc::POLLHUP | libc::POLLNVAL) == 0,
        "event fd error condition (revents {:#x})",
        pfd.revents
    );
    check!(
        pfd.revents & libc::POLLIN != 0,
        "poll woke without POLLIN (revents {:#x})",
        pfd.revents
    );

    // Assemble the full 16-byte iio_event_data record; tolerate short
    // reads and EINTR rather than misreporting them as protocol errors.
    let mut buf = [0u8; 16];
    let mut got = 0usize;
    while got < buf.len() {
        let n = unsafe {
            libc::read(
                fd.as_raw_fd(),
                buf[got..].as_mut_ptr().cast(),
                buf.len() - got,
            )
        };
        if n < 0 {
            let err = std::io::Error::last_os_error();
            if err.kind() == std::io::ErrorKind::Interrupted {
                continue;
            }
            return Err(format!("event read after {} bytes: {}", got, err));
        }
        check!(n != 0, "event fd EOF after {} of 16 bytes", got);
        got += n as usize;
    }
    let id = u64::from_ne_bytes(buf[0..8].try_into().unwrap());
    let ts = i64::from_ne_bytes(buf[8..16].try_into().unwrap());
    Ok(Some((id, ts)))
}

fn expect_event(fd: &OwnedFd, timeout: Duration, what: &str) -> Result<i64, String> {
    match read_event(fd, timeout)? {
        Some((id, ts)) => {
            check!(
                id == EXPECTED_EVENT_ID,
                "{}: event id {:#x}, expected {:#x} (light/thresh/either)",
                what,
                id,
                EXPECTED_EVENT_ID
            );
            check!(ts > 0, "{}: non-positive timestamp {}", what, ts);
            Ok(ts)
        }
        None => Err(format!("{}: no event within {:?}", what, timeout)),
    }
}

fn test_events(ctx: &mut Ctx) -> TestResult {
    reset_baseline(ctx)?;

    let enable = attr(ctx, "events/in_illuminance_thresh_either_en");
    let rising = attr(ctx, "events/in_illuminance_thresh_rising_value");
    let falling = attr(ctx, "events/in_illuminance_thresh_falling_value");
    let raw = attr(ctx, "in_illuminance_raw");
    let fd = open_event_fd(&ctx.chardev)?;

    // Run the body with guaranteed teardown: a mid-test failure must not
    // leave events enabled (that would pin the device in CONTINUOUS and
    // cascade into the buffer/power-fsm tests).
    let result = (|| -> TestResult {
        // The device only signals on the in-window -> out-of-window
        // transition, so each crossing below is created deterministically:
        // first park the level inside a window (clearing any latched
        // edge-state from earlier use), then move a threshold past it.
        sysfs_write(&enable, "1")?;
        check!(read_u32(&enable)? == 1, "enable readback != 1");
        expect_power_mode(PWR_CONTINUOUS, "with events enabled")?;

        // Full-range window: a few conversions observe the level in-window,
        // so the simulator's edge latch is cleared regardless of how the
        // device was last used. Then avoid arming right as the 0..250
        // sawtooth wraps (which would briefly undo the crossing) by sampling
        // a non-peak level; the wait is bounded so a stalled simulator fails
        // via the expect_event timeout below rather than hanging here.
        sleep(3 * CONV_PERIOD);
        while read_event(&fd, Duration::ZERO)?.is_some() {} // drain
        let deadline = Instant::now() + Duration::from_secs(12);
        let mut level = read_u32(&raw)?;
        while level > 200 && Instant::now() < deadline {
            sleep(CONV_PERIOD);
            level = read_u32(&raw)?;
        }

        // Upward crossing: drop the high threshold just below the sampled
        // level so the next conversion is already out-of-window above.
        sysfs_write(&rising, &format!("{}", level.saturating_sub(2)))?;
        let first_ts = expect_event(&fd, Duration::from_secs(10), "above-window crossing")?;

        // Re-arm downward: widen again, let a conversion see the level back
        // in range to clear the latch, then raise the low threshold above it.
        sysfs_write(&rising, "65535")?;
        sleep(3 * CONV_PERIOD);
        while read_event(&fd, Duration::ZERO)?.is_some() {}
        let level = read_u32(&raw)?;
        sysfs_write(&falling, &format!("{}", level + 100))?;
        let second_ts = expect_event(&fd, Duration::from_secs(10), "below-window crossing")?;
        check!(
            second_ts > first_ts,
            "event timestamps not increasing: {} then {}",
            first_ts,
            second_ts
        );

        // Back in range: no further events may arrive.
        sysfs_write(&falling, "0")?;
        sysfs_write(&rising, "65535")?;
        sleep(3 * CONV_PERIOD);
        while read_event(&fd, Duration::ZERO)?.is_some() {}
        check!(
            read_event(&fd, 5 * CONV_PERIOD)?.is_none(),
            "spurious event while level inside the threshold window"
        );

        expect_power_mode(PWR_CONTINUOUS, "events still enabled before teardown")?;
        Ok(())
    })();

    // Always disable events, then report the inner result.
    let teardown = sysfs_write(&enable, "0");
    result?;
    teardown?;
    check!(read_u32(&enable)? == 0, "enable readback != 0");
    expect_power_mode(PWR_SLEEP, "after disabling events")?;
    Ok(())
}

/// Triggered buffer with the timestamp scan element *disabled*. test_buffer
/// always captures 16-byte timestamped records; this proves the other layout
/// the driver must support -- bare 2-byte little-endian light samples, which
/// is what the ImHex line_plot capture in the docs relies on. If the driver
/// wrongly kept emitting the 16-byte record, the s64 timestamp bytes would
/// read back as wildly out-of-range u16 values, so the in-range check below
/// is what actually pins the record size.
fn test_buffer_no_timestamp(ctx: &mut Ctx) -> TestResult {
    reset_baseline(ctx)?;
    let scan_en = attr(ctx, "scan_elements/in_illuminance_en");
    let ts_en = attr(ctx, "scan_elements/in_timestamp_en");
    let buf_en = attr(ctx, "buffer/enable");

    sysfs_write(&scan_en, "1")?;
    sysfs_write(&ts_en, "0")?; // timestamp OFF -> pure u16 records
    sysfs_write(&attr(ctx, "trigger/current_trigger"), &ctx.trigger)?;
    sysfs_write(&attr(ctx, "buffer/length"), "64")?;
    sysfs_write(&buf_en, "1")?;

    let result = (|| -> TestResult {
        expect_power_mode(PWR_CONTINUOUS, "with buffer enabled")?;

        use std::os::unix::fs::OpenOptionsExt;
        let file = fs::OpenOptions::new()
            .read(true)
            .custom_flags(libc::O_NONBLOCK)
            .open(&ctx.chardev)
            .map_err(|e| format!("open {}: {}", ctx.chardev.display(), e))?;
        let mut data = Vec::new();
        let deadline = Instant::now() + Duration::from_secs(8);
        while data.len() < 8 * 2 && Instant::now() < deadline {
            let mut chunk = [0u8; 256];
            if let Some(n) = poll_read_nonblock(file.as_raw_fd(), &mut chunk, 1000)? {
                data.extend_from_slice(&chunk[..n]);
            }
        }
        check!(
            data.len() >= 8 * 2,
            "only {} bytes in 8 s, expected >= 16 (8 u16 samples)",
            data.len()
        );
        check!(
            data.len() % 2 == 0,
            "odd byte count {} -- not whole 2-byte records",
            data.len()
        );

        let samples: Vec<u16> = data
            .chunks_exact(2)
            .map(|s| u16::from_le_bytes([s[0], s[1]]))
            .collect();
        for &light in &samples {
            check!(
                light <= MAX_RAW_DEFAULT_CFG,
                "buffered u16 sample {} out of range (16-byte records leaking in?)",
                light
            );
        }
        check!(
            samples.iter().any(|&v| v != samples[0]),
            "all buffered samples identical; conversions not running"
        );
        Ok(())
    })();

    let mut teardown = String::new();
    for (path, val) in [(&buf_en, "0"), (&scan_en, "0")] {
        if let Err(e) = sysfs_write(path, val) {
            let _ = write!(teardown, "; teardown: {}", e);
        }
    }
    result.map_err(|e| e + &teardown)?;
    check!(teardown.is_empty(), "buffer teardown failed{}", teardown);
    expect_power_mode(PWR_SLEEP, "after disabling buffer")?;
    Ok(())
}

fn test_buffer(ctx: &mut Ctx) -> TestResult {
    reset_baseline(ctx)?;
    let scan_en = attr(ctx, "scan_elements/in_illuminance_en");
    let ts_en = attr(ctx, "scan_elements/in_timestamp_en");
    let buf_en = attr(ctx, "buffer/enable");

    let scan_type = sysfs_read(&attr(ctx, "scan_elements/in_illuminance_type"))?;
    check!(
        scan_type == "le:u16/16>>0",
        "unexpected scan type {:?}",
        scan_type
    );

    sysfs_write(&scan_en, "1")?;
    sysfs_write(&ts_en, "1")?;
    sysfs_write(&attr(ctx, "trigger/current_trigger"), &ctx.trigger)?;
    sysfs_write(&attr(ctx, "buffer/length"), "64")?;
    sysfs_write(&buf_en, "1")?;

    let result = (|| -> TestResult {
        expect_power_mode(PWR_CONTINUOUS, "with buffer enabled")?;

        // A direct read in continuous mode returns the most recent *latched*
        // conversion, and a freshly-enabled buffer can still hold a sample
        // latched under the previous test's gain/time (the simulator keeps the
        // last value until the next conversion). Wait for one fresh conversion
        // at the reset baseline before the range check so it judges current
        // data, not stale carry-over.
        expect_buffer_alive(ctx, Duration::from_secs(3))?;

        // Direct reads must keep working alongside the buffer.
        let v = read_u32(&attr(ctx, "in_illuminance_raw"))?;
        check!(
            v <= MAX_RAW_DEFAULT_CFG as u32,
            "raw read during buffering out of range: {}",
            v
        );

        // Samples are interrupt-paced at one per conversion period:
        // collect some and validate layout, cadence and monotonicity.
        // Non-blocking + poll so a silent trigger fails the test instead
        // of hanging the harness on a blocking read.
        use std::os::unix::fs::OpenOptionsExt;
        let file = fs::OpenOptions::new()
            .read(true)
            .custom_flags(libc::O_NONBLOCK)
            .open(&ctx.chardev)
            .map_err(|e| format!("open {}: {}", ctx.chardev.display(), e))?;
        let mut data = Vec::new();
        let deadline = Instant::now() + Duration::from_secs(8);
        while data.len() < 8 * 16 && Instant::now() < deadline {
            let mut chunk = [0u8; 256];
            if let Some(n) = poll_read_nonblock(file.as_raw_fd(), &mut chunk, 1000)? {
                data.extend_from_slice(&chunk[..n]);
            }
        }
        check!(
            data.len() >= 8 * 16,
            "only {} bytes ({} samples) in 8 s, expected >= 8 samples",
            data.len(),
            data.len() / 16
        );

        let mut prev_ts: Option<i64> = None;
        let mut deltas = Vec::new();
        for sample in data.chunks_exact(16) {
            let light = u16::from_le_bytes(sample[0..2].try_into().unwrap());
            let ts = i64::from_le_bytes(sample[8..16].try_into().unwrap());
            check!(
                light <= MAX_RAW_DEFAULT_CFG,
                "buffered sample {} out of range",
                light
            );
            if let Some(prev) = prev_ts {
                check!(ts > prev, "timestamps not monotonic: {} -> {}", prev, ts);
                deltas.push(ts - prev);
            }
            prev_ts = Some(ts);
        }

        // Median spacing must reflect the 200 ms interrupt cadence.
        deltas.sort_unstable();
        let median_ms = deltas[deltas.len() / 2] / 1_000_000;
        check!(
            (100..=400).contains(&median_ms),
            "median sample spacing {} ms, expected ~200 ms",
            median_ms
        );
        Ok(())
    })();

    // Always tear down, then report the inner result.
    let mut teardown = String::new();
    for (path, val) in [(&buf_en, "0"), (&scan_en, "0"), (&ts_en, "0")] {
        if let Err(e) = sysfs_write(path, val) {
            let _ = write!(teardown, "; teardown: {}", e);
        }
    }
    result.map_err(|e| e + &teardown)?;
    check!(teardown.is_empty(), "buffer teardown failed{}", teardown);

    expect_power_mode(PWR_SLEEP, "after disabling buffer")?;
    Ok(())
}

/// Validate the *absolute* accuracy of buffered-sample timestamps, not just
/// their spacing. test_buffer already proves monotonicity and the ~200 ms
/// cadence, but a delta-only check passes even if the driver stamps on the
/// wrong clock, emits a constant/zero timestamp, or carries a fixed offset.
/// Here we bracket the read with clock_gettime() on the very clock the IIO
/// core advertises for this device and require every fresh sample's stamp to
/// fall inside that wall-clock window.
fn test_timestamp(ctx: &mut Ctx) -> TestResult {
    reset_baseline(ctx)?;
    let (clk, clk_name) = iio_timestamp_clock(ctx)?;

    set_buffer(ctx, true)?;
    let result = (|| -> TestResult {
        expect_power_mode(PWR_CONTINUOUS, "with buffer enabled")?;

        use std::os::unix::fs::OpenOptionsExt;
        let file = fs::OpenOptions::new()
            .read(true)
            .custom_flags(libc::O_NONBLOCK)
            .open(&ctx.chardev)
            .map_err(|e| format!("open {}: {}", ctx.chardev.display(), e))?;

        // Drain the backlog so every sample we judge below was stamped after
        // we start watching the clock.
        loop {
            let mut chunk = [0u8; 256];
            match poll_read_nonblock(file.as_raw_fd(), &mut chunk, 0)? {
                Some(n) if n > 0 => continue,
                _ => break,
            }
        }

        // Bracket fresh samples with the device's own clock.
        let before = clock_now_ns(clk)?;
        let mut data = Vec::new();
        let deadline = Instant::now() + Duration::from_secs(4);
        while data.len() < 4 * 16 && Instant::now() < deadline {
            let mut chunk = [0u8; 256];
            if let Some(n) = poll_read_nonblock(file.as_raw_fd(), &mut chunk, 1000)? {
                data.extend_from_slice(&chunk[..n]);
            }
        }
        let after = clock_now_ns(clk)?;
        check!(
            data.len() >= 16,
            "no buffered sample within 4 s (clock {})",
            clk_name
        );

        // Low slack: a sample could have been latched in the gap between the
        // drain and `before`, so allow one conversion period there. High
        // slack: a stamp is taken before read() returns, so `after` already
        // bounds it; a small margin only absorbs clock-read scheduling slop.
        let lo = before - CONV_PERIOD.as_nanos() as i64 - 50_000_000;
        let hi = after + 50_000_000;
        for sample in data.chunks_exact(16) {
            let ts = i64::from_le_bytes(sample[8..16].try_into().unwrap());
            check!(ts != 0, "zero timestamp on clock {}", clk_name);
            let off_ms = if ts < lo {
                (lo - ts) / 1_000_000
            } else {
                (ts - hi) / 1_000_000
            };
            check!(
                (lo..=hi).contains(&ts),
                "timestamp {} outside read window [{}, {}] on clock {} (off by {} ms)",
                ts,
                lo,
                hi,
                clk_name,
                off_ms
            );
        }
        Ok(())
    })();

    let teardown_res = set_buffer(ctx, false);
    match result {
        Ok(()) => teardown_res?,
        Err(e) => {
            if let Err(t) = teardown_res {
                return Err(format!("{}; teardown: {}", e, t));
            }
            return Err(e);
        }
    }
    expect_power_mode(PWR_SLEEP, "after disabling buffer")?;
    Ok(())
}

fn set_buffer(ctx: &Ctx, on: bool) -> TestResult {
    if on {
        sysfs_write(&attr(ctx, "scan_elements/in_illuminance_en"), "1")?;
        sysfs_write(&attr(ctx, "scan_elements/in_timestamp_en"), "1")?;
        sysfs_write(&attr(ctx, "trigger/current_trigger"), &ctx.trigger)?;
        sysfs_write(&attr(ctx, "buffer/enable"), "1")?;
    } else {
        sysfs_write(&attr(ctx, "buffer/enable"), "0")?;
        sysfs_write(&attr(ctx, "scan_elements/in_illuminance_en"), "0")?;
        sysfs_write(&attr(ctx, "scan_elements/in_timestamp_en"), "0")?;
    }
    Ok(())
}

/// Wait until at least one full buffered sample can be read, proving the
/// interrupt -> trigger -> buffer data path is currently alive.
fn expect_buffer_alive(ctx: &Ctx, timeout: Duration) -> TestResult {
    use std::os::unix::fs::OpenOptionsExt;
    let file = fs::OpenOptions::new()
        .read(true)
        .custom_flags(libc::O_NONBLOCK)
        .open(&ctx.chardev)
        .map_err(|e| format!("open {}: {}", ctx.chardev.display(), e))?;
    // Drain any backlog so only samples produced from now on count.
    loop {
        let mut chunk = [0u8; 64];
        match poll_read_nonblock(file.as_raw_fd(), &mut chunk, 0)? {
            Some(n) if n > 0 => continue,
            _ => break,
        }
    }

    let deadline = Instant::now() + timeout;
    let mut got = 0usize;
    while got < 16 && Instant::now() < deadline {
        let mut chunk = [0u8; 64];
        if let Some(n) = poll_read_nonblock(file.as_raw_fd(), &mut chunk, 500)? {
            got += n;
        }
    }
    check!(got >= 16, "no buffered sample within {:?}", timeout);
    Ok(())
}

/// Power-state machine transitions across *combinations* of users. The
/// single-user transitions are covered by the event/buffer tests; this
/// exercises the cases where one of two active users goes away and the
/// device must stay in CONTINUOUS for the remaining one.
fn test_power_fsm(ctx: &mut Ctx) -> TestResult {
    reset_baseline(ctx)?;
    let ev_en = attr(ctx, "events/in_illuminance_thresh_either_en");
    let raw = attr(ctx, "in_illuminance_raw");

    // reset_baseline left thresholds spanning the full range, so enabling
    // the event here produces no crossings; only the power state matters.
    expect_power_mode(PWR_SLEEP, "at start")?;

    sysfs_write(&ev_en, "1")?;
    expect_power_mode(PWR_CONTINUOUS, "with events on")?;

    set_buffer(ctx, true)?;
    let result = (|| -> TestResult {
        expect_power_mode(PWR_CONTINUOUS, "with events+buffer on")?;

        // A direct read while users are active must use the latched data
        // path and leave the device in CONTINUOUS (no one-shot detour).
        read_u32(&raw)?;
        expect_power_mode(PWR_CONTINUOUS, "after raw read while active")?;

        // Key transition: dropping one of two users keeps the device
        // running, and the surviving data path still delivers samples.
        sysfs_write(&ev_en, "0")?;
        expect_power_mode(PWR_CONTINUOUS, "with buffer on after events off")?;
        expect_buffer_alive(ctx, Duration::from_secs(3))?;

        // And the mirror image: buffer off while events are back on.
        sysfs_write(&ev_en, "1")?;
        Ok(())
    })();
    let teardown = set_buffer(ctx, false);
    result?;
    teardown?;

    expect_power_mode(PWR_CONTINUOUS, "with events on after buffer off")?;

    // Last user gone: back to SLEEP, and the one-shot path must work again.
    sysfs_write(&ev_en, "0")?;
    expect_power_mode(PWR_SLEEP, "with all users off")?;
    read_u32(&raw)?;
    expect_power_mode(PWR_SLEEP, "after one-shot from sleep")?;
    Ok(())
}

fn test_unload(ctx: &mut Ctx) -> TestResult {
    // Detach the trigger so no consumer references linger.
    let _ = sysfs_write(&attr(ctx, "trigger/current_trigger"), "\n");

    run("rmmod", &[DRV_MODULE])?;
    check!(!module_loaded(DRV_MODULE), "{} still loaded after rmmod", DRV_MODULE);
    wait_device(false, Duration::from_secs(5))?;

    run("rmmod", &[SIM_MODULE])?;
    check!(!module_loaded(SIM_MODULE), "{} still loaded after rmmod", SIM_MODULE);
    check!(
        !Path::new(DEBUGFS_REGS).exists(),
        "simulator debugfs entry survived unload"
    );

    // Reload to prove the cycle is clean and leave the system usable.
    run("modprobe", &[SIM_MODULE])?;
    run("modprobe", &[DRV_MODULE])?;
    let (dev, chardev) = wait_device(true, Duration::from_secs(5))?
        .expect("wait_device(present) returned device");
    ctx.dev = dev;
    ctx.chardev = chardev;
    expect_power_mode(PWR_SLEEP, "after reload")?;
    Ok(())
}

/// The whole run (including the unload/reload cycle) must not have tripped
/// any kernel diagnostics: a WARN or refcount splat is a driver bug even
/// when the operation that caused it appears to succeed.
fn test_kernel_log(_ctx: &mut Ctx) -> TestResult {
    let out = Command::new("dmesg")
        .output()
        .map_err(|e| format!("run dmesg: {}", e))?;
    let log = String::from_utf8_lossy(&out.stdout);

    let bad: Vec<&str> = log
        .lines()
        .filter(|l| {
            ["WARNING:", "BUG:", "Oops", "refcount_t:", "Call trace:", "kernel NULL pointer"]
                .iter()
                .any(|p| l.contains(p))
        })
        .collect();
    check!(
        bad.is_empty(),
        "kernel log contains {} diagnostic line(s), first: {:?}",
        bad.len(),
        bad[0]
    );
    Ok(())
}
