// SPDX-License-Identifier: MIT
//! Streams raw `iio_event_data` records from an IIO chardev's event fd.
//!
//! A plain shell can't issue `IIO_GET_EVENT_FD_IOCTL`, so remote tooling
//! that only has a shell (e.g. a GUI driving the device over SSH) needs a
//! tiny local helper to do the ioctl and forward the resulting byte stream.
//! Each record is the kernel's 16-byte `struct iio_event_data` (u64 id +
//! i64 timestamp, native-endian), written to stdout exactly as read.
//!
//! This process blocks in poll() waiting for hardware events and has no
//! self-timeout; it relies on the caller tearing down the channel it was
//! spawned on (e.g. an SSH exec channel close sends SIGHUP) to exit.

use std::env;
use std::fs::File;
use std::io::{self, Write};
use std::os::unix::io::{AsRawFd, FromRawFd, OwnedFd};
use std::process::ExitCode;
use std::time::Duration;

// From the kernel UAPI (include/uapi/linux/iio): _IOR('i', 0x90, int).
const IIO_GET_EVENT_FD_IOCTL: libc::c_ulong = 0x8004_6990;

// A previous instance of this process, killed when its caller (e.g. an SSH
// channel) was torn down, may not have actually released the chardev yet:
// SIGKILL is prompt, but it's blocked in poll() on the *event* fd, not on
// anything tied to the channel, so it has no chance to notice the channel
// died until it next touches it -- the kill is what actually reclaims the
// chardev, and that reclaim isn't instantaneous from this process's point
// of view. Retry EBUSY for up to ~2s rather than failing immediately.
fn open_event_fd(chardev: &str) -> Result<OwnedFd, String> {
    let mut last_err = io::Error::new(io::ErrorKind::Other, "never attempted");
    for attempt in 0..20 {
        if attempt > 0 {
            std::thread::sleep(Duration::from_millis(100));
        }
        match try_open_event_fd(chardev) {
            Ok(fd) => return Ok(fd),
            Err(e) => {
                let busy = e.kind() == io::ErrorKind::ResourceBusy;
                last_err = e;
                if !busy {
                    break;
                }
            }
        }
    }
    Err(last_err.to_string())
}

fn try_open_event_fd(chardev: &str) -> io::Result<OwnedFd> {
    let dev = File::open(chardev)
        .map_err(|e| io::Error::new(e.kind(), format!("open {chardev}: {e}")))?;
    let mut event_fd: libc::c_int = -1;
    let ret = unsafe { libc::ioctl(dev.as_raw_fd(), IIO_GET_EVENT_FD_IOCTL, &mut event_fd) };
    if ret != 0 || event_fd < 0 {
        let err = io::Error::last_os_error();
        return Err(io::Error::new(
            err.kind(),
            format!("IIO_GET_EVENT_FD_IOCTL on {chardev} failed: {err}"),
        ));
    }
    Ok(unsafe { OwnedFd::from_raw_fd(event_fd) })
}

fn poll_readable(fd: &OwnedFd) -> Result<bool, String> {
    let mut pfd = libc::pollfd {
        fd: fd.as_raw_fd(),
        events: libc::POLLIN,
        revents: 0,
    };
    loop {
        let ret = unsafe { libc::poll(&mut pfd, 1, -1) };
        if ret < 0 {
            let err = io::Error::last_os_error();
            if err.kind() == io::ErrorKind::Interrupted {
                continue;
            }
            return Err(format!("poll: {err}"));
        }
        break;
    }
    if pfd.revents & (libc::POLLERR | libc::POLLHUP | libc::POLLNVAL) != 0 {
        return Ok(false);
    }
    Ok(pfd.revents & libc::POLLIN != 0)
}

fn read_record(fd: &OwnedFd, buf: &mut [u8; 16]) -> Result<bool, String> {
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
            let err = io::Error::last_os_error();
            if err.kind() == io::ErrorKind::Interrupted {
                continue;
            }
            return Err(format!("read after {got} bytes: {err}"));
        }
        if n == 0 {
            return Ok(false); // EOF
        }
        got += n as usize;
    }
    Ok(true)
}

fn run() -> Result<(), String> {
    let chardev = env::args()
        .nth(1)
        .ok_or_else(|| format!("usage: {} <iio-chardev>", env!("CARGO_PKG_NAME")))?;

    let fd = open_event_fd(&chardev)?;
    let mut stdout = io::stdout();
    let mut buf = [0u8; 16];

    loop {
        if !poll_readable(&fd)? {
            return Ok(()); // device gone (POLLHUP/POLLERR) -- exit cleanly
        }
        if !read_record(&fd, &mut buf)? {
            return Ok(()); // EOF
        }
        if stdout.write_all(&buf).is_err() || stdout.flush().is_err() {
            return Ok(()); // far end closed (e.g. SSH channel torn down)
        }
    }
}

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("envcombo-evtcat: {e}");
            ExitCode::FAILURE
        }
    }
}
