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
const REG_ALS_TH_LOW_MSB: usize = 0x08;
const REG_ALS_TH_HIGH_MSB: usize = 0x0A;
const REG_PWR_MODE: usize = 0x12;
const REG_COUNT: usize = 0x13;

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

fn main() {
    let tests: &[(&str, fn(&mut Ctx) -> TestResult)] = &[
        ("module-loading", test_module_loading),
        ("direct-als-read", test_direct_read),
        ("gain-and-integration-time", test_gain_and_time),
        ("threshold-attributes", test_threshold_attrs),
        ("threshold-events", test_events),
        ("triggered-buffer", test_buffer),
        ("power-fsm-transitions", test_power_fsm),
        ("module-unloading", test_unload),
        ("kernel-log-health", test_kernel_log),
    ];

    let mut ctx = Ctx::default();
    let mut failures = 0;

    println!("envcombo driver test harness ({} tests)", tests.len());
    for (i, (name, test)) in tests.iter().enumerate() {
        match test(&mut ctx) {
            Ok(()) => println!("[{}/{}] {:<28} PASS", i + 1, tests.len(), name),
            Err(e) => {
                failures += 1;
                println!("[{}/{}] {:<28} FAIL: {}", i + 1, tests.len(), name, e);
            }
        }
    }

    if failures == 0 {
        println!("RESULT: all {} tests passed", tests.len());
    } else {
        println!("RESULT: {} of {} tests FAILED", failures, tests.len());
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
