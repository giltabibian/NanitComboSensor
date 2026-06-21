//! Blocking ssh2 I/O, always off the egui UI thread.
//!
//! Each one-off command opens its own short-lived connection on a fresh
//! background thread (`exec_async`) -- simpler and safer than multiplexing
//! one shared `ssh2::Session` across concurrent callers, at the cost of a
//! fresh TCP+SSH handshake per call (cheap over the loopback-forwarded
//! QEMU link this targets). Long-lived reads (event monitor, buffer
//! capture) keep one connection for their duration; since a blocked
//! libssh2 read can't otherwise be interrupted from another thread,
//! cancelling one shuts down the underlying socket out from under it.

use ssh2::{KeyboardInteractivePrompt, Prompt, Session};
use std::io::Read;
use std::net::{Shutdown, TcpStream, ToSocketAddrs};
use std::sync::mpsc::Sender;
use std::thread::JoinHandle;
use std::time::Duration;

#[derive(Clone, Debug)]
pub struct Config {
    pub host: String,
    pub port: u16,
    pub user: String,
    pub password: String,
}

impl Default for Config {
    fn default() -> Self {
        Config {
            host: "localhost".to_string(),
            port: 2222,
            user: "root".to_string(),
            password: String::new(),
        }
    }
}

fn connect(cfg: &Config) -> Result<(Session, TcpStream), String> {
    let addr = format!("{}:{}", cfg.host, cfg.port);
    let socket_addr = addr
        .to_socket_addrs()
        .map_err(|e| format!("resolve {addr}: {e}"))?
        .next()
        .ok_or_else(|| format!("no address for {addr}"))?;
    let tcp = TcpStream::connect_timeout(&socket_addr, Duration::from_secs(5))
        .map_err(|e| format!("connect {addr}: {e}"))?;
    let shutdown_handle = tcp.try_clone().map_err(|e| format!("dup socket: {e}"))?;

    let mut sess = Session::new().map_err(|e| format!("session init: {e}"))?;
    sess.set_tcp_stream(tcp);
    sess.handshake().map_err(|e| format!("handshake: {e}"))?;
    authenticate(&sess, &cfg.user, &cfg.password)?;
    Ok((sess, shutdown_handle))
}

struct PasswordPrompter<'a> {
    password: &'a str,
}

impl KeyboardInteractivePrompt for PasswordPrompter<'_> {
    fn prompt<'a>(&mut self, _username: &str, _instructions: &str, prompts: &[Prompt<'a>]) -> Vec<String> {
        prompts.iter().map(|_| self.password.to_string()).collect()
    }
}

/// Tries every auth method the server actually offers, in roughly the order
/// a normal `ssh` client would. A server permitting blank passwords (e.g.
/// dropbear with `allow-empty-password`) may grant access outright via the
/// `none` method, via plain `password`, or -- common with PAM-backed
/// servers -- only via `keyboard-interactive` even though it looks like it
/// supports `password`. Going straight to `userauth_password` only covers
/// the middle case, which is what silently broke this against dropbear.
fn authenticate(sess: &Session, user: &str, password: &str) -> Result<(), String> {
    let methods = match sess.auth_methods(user) {
        Ok(m) => m.to_string(),
        Err(_) if sess.authenticated() => return Ok(()), // "none" already succeeded
        Err(e) => return Err(format!("query auth methods for {user}: {e}")),
    };
    if sess.authenticated() {
        return Ok(());
    }

    if methods.is_empty() || methods.contains("password") {
        let _ = sess.userauth_password(user, password);
        if sess.authenticated() {
            return Ok(());
        }
    }
    if methods.contains("keyboard-interactive") {
        let mut prompter = PasswordPrompter { password };
        let _ = sess.userauth_keyboard_interactive(user, &mut prompter);
        if sess.authenticated() {
            return Ok(());
        }
    }
    Err(format!(
        "auth as {user} failed (server offered: {})",
        if methods.is_empty() { "none" } else { &methods }
    ))
}

fn drain_channel(channel: &mut ssh2::Channel) -> Result<Vec<u8>, String> {
    let mut out = Vec::new();
    channel
        .read_to_end(&mut out)
        .map_err(|e| format!("read: {e}"))?;
    let _ = channel.wait_close();
    let status = channel.exit_status().unwrap_or(0);
    if status != 0 {
        let mut err = String::new();
        let _ = channel.stderr().read_to_string(&mut err);
        return Err(format!("exited {status}: {}", err.trim()));
    }
    Ok(out)
}

/// Run `cmd` to completion over a fresh connection, returning its stdout.
pub fn exec_once(cfg: &Config, cmd: &str) -> Result<Vec<u8>, String> {
    let (sess, _tcp) = connect(cfg)?;
    let mut channel = sess
        .channel_session()
        .map_err(|e| format!("open channel: {e}"))?;
    channel
        .exec(cmd)
        .map_err(|e| format!("exec {cmd:?}: {e}"))?;
    drain_channel(&mut channel)
}

/// Spawn `exec_once` on a background thread; the result arrives on `reply`.
pub fn exec_async(cfg: Config, cmd: String, reply: Sender<Result<Vec<u8>, String>>) {
    std::thread::spawn(move || {
        let _ = reply.send(exec_once(&cfg, &cmd));
    });
}

/// A cancellable long-running remote command whose raw stdout is streamed to
/// `sink` as it arrives (the event monitor, a live buffer capture, ...).
pub struct Stream {
    shutdown_handle: TcpStream,
    join: Option<JoinHandle<()>>,
}

impl Stream {
    pub fn stop(mut self) {
        let _ = self.shutdown_handle.shutdown(Shutdown::Both);
        if let Some(j) = self.join.take() {
            let _ = j.join();
        }
    }
}

/// Connects, execs `cmd`, and returns once the channel is open; reading then
/// continues on a background thread until EOF, an error, or `Stream::stop`.
pub fn start_stream(cfg: &Config, cmd: &str, sink: Sender<Vec<u8>>) -> Result<Stream, String> {
    let (sess, shutdown_handle) = connect(cfg)?;
    let mut channel = sess
        .channel_session()
        .map_err(|e| format!("open channel: {e}"))?;
    channel
        .exec(cmd)
        .map_err(|e| format!("exec {cmd:?}: {e}"))?;

    let join = std::thread::spawn(move || {
        let mut buf = [0u8; 4096];
        loop {
            match channel.read(&mut buf) {
                Ok(0) => break,
                Ok(n) => {
                    if sink.send(buf[..n].to_vec()).is_err() {
                        break; // UI side gave up listening
                    }
                }
                Err(_) => break, // includes the socket shutdown from Stream::stop()
            }
        }
        let _ = channel.close();
    });

    Ok(Stream {
        shutdown_handle,
        join: Some(join),
    })
}
