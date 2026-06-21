//! egui panels driving the envcombo userspace ABI over SSH.

use crate::{abi, ssh};
use eframe::egui;
use egui_plot::{HLine, Legend, Line, LineStyle, Plot, PlotPoints};
use std::collections::VecDeque;
use std::sync::mpsc::{self, Receiver, TryRecvError};
use std::time::{Duration, Instant};

const SNAPSHOT_INTERVAL: Duration = Duration::from_millis(1000);
const EVENT_LOG_CAP: usize = 200;
const PLOT_CAP: usize = 500;

#[derive(PartialEq, Clone, Copy)]
enum ConnStatus {
    Disconnected,
    Connecting,
    Connected,
    Failed,
}

enum ExportKind {
    Regs,
    Buffer,
}

enum EventLogEntry {
    Marker(String),
    Event {
        seq: u64,
        label: String,
        ts_ns: i64,
        /// in_illuminance_raw, read on demand right after the event arrives
        /// (cheap while CONTINUOUS -- it's just the latched value, no fresh
        /// conversion) since the event record itself carries no sample.
        raw: Option<u16>,
    },
}

struct PendingAction {
    label: String,
    rx: Receiver<Result<Vec<u8>, String>>,
}

pub struct EnvComboCtl {
    host_buf: String,
    port_buf: String,
    user_buf: String,
    password_buf: String,

    cfg: ssh::Config,
    status: ConnStatus,
    status_msg: String,
    connect_rx: Option<Receiver<Result<Vec<u8>, String>>>,
    device: Option<abi::Device>,

    pending: Vec<PendingAction>,

    snapshot: abi::Snapshot,
    snapshot_inflight: bool,
    snapshot_rx: Option<Receiver<Result<Vec<u8>, String>>>,
    last_snapshot: Instant,

    thresh_rising_buf: String,
    thresh_falling_buf: String,
    event_log: VecDeque<EventLogEntry>,
    event_stream: Option<ssh::Stream>,
    event_rx: Option<Receiver<ssh::StreamEvent>>,
    event_partial: Vec<u8>,
    next_event_seq: u64,
    pending_event_reads: Vec<(u64, Receiver<Result<Vec<u8>, String>>)>,

    buffer_len_buf: String,
    plot_points: VecDeque<[f64; 2]>,
    plot_record_len: usize,
    sample_index: f64,
    buffer_stream: Option<ssh::Stream>,
    buffer_rx: Option<Receiver<ssh::StreamEvent>>,
    buffer_partial: Vec<u8>,

    export_dir: String,
    export_count_buf: String,
    export_status: String,
    export_inflight: bool,
    export_kind: Option<ExportKind>,
    export_rx: Option<Receiver<Result<Vec<u8>, String>>>,
}

impl Default for EnvComboCtl {
    fn default() -> Self {
        let cfg = ssh::Config::default();
        Self {
            host_buf: cfg.host.clone(),
            port_buf: cfg.port.to_string(),
            user_buf: cfg.user.clone(),
            password_buf: cfg.password.clone(),

            cfg,
            status: ConnStatus::Disconnected,
            status_msg: String::new(),
            connect_rx: None,
            device: None,

            pending: Vec::new(),

            snapshot: abi::Snapshot::default(),
            snapshot_inflight: false,
            snapshot_rx: None,
            last_snapshot: Instant::now() - SNAPSHOT_INTERVAL,

            thresh_rising_buf: String::new(),
            thresh_falling_buf: String::new(),
            event_log: VecDeque::new(),
            event_stream: None,
            event_rx: None,
            event_partial: Vec::new(),
            next_event_seq: 0,
            pending_event_reads: Vec::new(),

            buffer_len_buf: "64".to_string(),
            plot_points: VecDeque::new(),
            plot_record_len: 0,
            sample_index: 0.0,
            buffer_stream: None,
            buffer_rx: None,
            buffer_partial: Vec::new(),

            export_dir: ".".to_string(),
            export_count_buf: "64".to_string(),
            export_status: String::new(),
            export_inflight: false,
            export_kind: None,
            export_rx: None,
        }
    }
}

impl eframe::App for EnvComboCtl {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        self.poll_connect();
        self.drain_pending();
        self.poll_snapshot();
        self.poll_event_stream();
        self.poll_pending_event_reads();
        self.poll_buffer_stream();
        self.poll_export();

        egui::TopBottomPanel::top("connection").show(ctx, |ui| self.connection_bar(ui));

        egui::CentralPanel::default().show(ctx, |ui| {
            if self.device.is_none() {
                ui.label("Not connected. Enter connection details above and click Connect.");
                if !self.status_msg.is_empty() {
                    ui.label(&self.status_msg);
                }
                return;
            }
            // One page, collapsible sections instead of tabs -- nothing is
            // ever out of reach behind a tab switch, but the ones you're
            // not using right now can be folded out of the way.
            egui::ScrollArea::vertical().show(ui, |ui| {
                collapsing_section(ui, "Channel", true, |ui| self.channel_section(ui));
                collapsing_section(ui, "Events", true, |ui| self.events_section(ui));
                collapsing_section(ui, "Buffer", true, |ui| self.buffer_section(ui));
                collapsing_section(ui, "State", false, |ui| self.state_section(ui));
                collapsing_section(ui, "Simulator", false, |ui| self.simulator_section(ui));
                collapsing_section(ui, "Capture / Export", false, |ui| {
                    self.capture_section(ui)
                });
            });
        });

        let live = self.event_stream.is_some() || self.buffer_stream.is_some();
        ctx.request_repaint_after(Duration::from_millis(if live { 80 } else { 250 }));
    }
}

impl EnvComboCtl {
    fn fire(&mut self, label: &str, cmd: String) {
        let (tx, rx) = mpsc::channel();
        ssh::exec_async(self.cfg.clone(), cmd, tx);
        self.pending.push(PendingAction {
            label: label.to_string(),
            rx,
        });
    }

    fn drain_pending(&mut self) {
        let mut still = Vec::new();
        let mut any_done = false;
        for p in self.pending.drain(..) {
            match p.rx.try_recv() {
                Ok(Ok(_)) => any_done = true,
                Ok(Err(e)) => {
                    self.status_msg = format!("{}: {e}", p.label);
                    any_done = true;
                }
                Err(TryRecvError::Empty) => still.push(p),
                Err(TryRecvError::Disconnected) => {}
            }
        }
        self.pending = still;
        if any_done {
            // Pull the change forward instead of waiting for the next tick.
            self.last_snapshot = Instant::now() - SNAPSHOT_INTERVAL;
        }
    }

    fn connect(&mut self) {
        self.cfg = ssh::Config {
            host: self.host_buf.clone(),
            port: self.port_buf.trim().parse().unwrap_or(2222),
            user: self.user_buf.clone(),
            password: self.password_buf.clone(),
        };
        self.status = ConnStatus::Connecting;
        self.status_msg.clear();
        let (tx, rx) = mpsc::channel();
        ssh::exec_async(self.cfg.clone(), abi::DISCOVER_CMD.to_string(), tx);
        self.connect_rx = Some(rx);
    }

    fn poll_connect(&mut self) {
        let Some(rx) = &self.connect_rx else {
            return;
        };
        match rx.try_recv() {
            Ok(Ok(bytes)) => {
                let text = String::from_utf8_lossy(&bytes).into_owned();
                match abi::device_from_discovery(&text) {
                    Some(dev) => {
                        self.device = Some(dev);
                        self.status = ConnStatus::Connected;
                        self.last_snapshot = Instant::now() - SNAPSHOT_INTERVAL;
                    }
                    None => {
                        self.status = ConnStatus::Failed;
                        self.status_msg =
                            "connected, but no envcombo IIO device found (modules loaded?)"
                                .to_string();
                    }
                }
                self.connect_rx = None;
            }
            Ok(Err(e)) => {
                self.status = ConnStatus::Failed;
                self.status_msg = e;
                self.connect_rx = None;
            }
            Err(TryRecvError::Empty) => {}
            Err(TryRecvError::Disconnected) => self.connect_rx = None,
        }
    }

    fn disconnect(&mut self) {
        if let Some(s) = self.event_stream.take() {
            s.stop();
        }
        if let Some(s) = self.buffer_stream.take() {
            s.stop();
        }
        self.event_rx = None;
        self.buffer_rx = None;
        self.device = None;
        self.status = ConnStatus::Disconnected;
        self.status_msg.clear();
    }

    fn poll_snapshot(&mut self) {
        let Some(device) = self.device.clone() else {
            return;
        };
        if let Some(rx) = &self.snapshot_rx {
            match rx.try_recv() {
                Ok(Ok(bytes)) => {
                    let text = String::from_utf8_lossy(&bytes).into_owned();
                    self.snapshot = abi::parse_snapshot(&text);
                    self.snapshot_inflight = false;
                    self.snapshot_rx = None;
                }
                Ok(Err(e)) => {
                    self.status_msg = format!("snapshot: {e}");
                    self.snapshot_inflight = false;
                    self.snapshot_rx = None;
                }
                Err(TryRecvError::Empty) => {}
                Err(TryRecvError::Disconnected) => {
                    self.snapshot_inflight = false;
                    self.snapshot_rx = None;
                }
            }
        }
        if !self.snapshot_inflight && self.last_snapshot.elapsed() >= SNAPSHOT_INTERVAL {
            let (tx, rx) = mpsc::channel();
            ssh::exec_async(self.cfg.clone(), abi::snapshot_cmd(&device), tx);
            self.snapshot_rx = Some(rx);
            self.snapshot_inflight = true;
            self.last_snapshot = Instant::now();
        }
    }

    fn connection_bar(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            ui.label("Host:");
            ui.add(egui::TextEdit::singleline(&mut self.host_buf).desired_width(110.0));
            ui.label("Port:");
            ui.add(egui::TextEdit::singleline(&mut self.port_buf).desired_width(50.0));
            ui.label("User:");
            ui.add(egui::TextEdit::singleline(&mut self.user_buf).desired_width(60.0));
            ui.label("Password:");
            ui.add(
                egui::TextEdit::singleline(&mut self.password_buf)
                    .password(true)
                    .desired_width(90.0),
            );

            let busy = self.status == ConnStatus::Connecting;
            if ui
                .add_enabled(!busy, egui::Button::new("Connect"))
                .clicked()
            {
                self.connect();
            }
            if ui
                .add_enabled(self.device.is_some(), egui::Button::new("Disconnect"))
                .clicked()
            {
                self.disconnect();
            }

            match self.status {
                ConnStatus::Disconnected => {
                    ui.label("⚪ disconnected");
                }
                ConnStatus::Connecting => {
                    ui.spinner();
                    ui.label("connecting...");
                }
                ConnStatus::Connected => {
                    let base = self.device.as_ref().map(|d| d.base.as_str()).unwrap_or("");
                    ui.colored_label(egui::Color32::GREEN, "●");
                    ui.label(format!("connected — {base}"));
                }
                ConnStatus::Failed => {
                    ui.colored_label(egui::Color32::RED, "●");
                    ui.label(&self.status_msg);
                }
            }
        });
    }

    fn channel_section(&mut self, ui: &mut egui::Ui) {
        let device = self.device.clone();
        let s = self.snapshot.clone();

        ui.label(format!(
            "in_illuminance_raw: {}",
            s.raw.map(|v| v.to_string()).unwrap_or("-".into())
        ));
        ui.label(format!(
            "in_illuminance_scale: {}",
            s.scale.map(|v| format!("{v:.6}")).unwrap_or("-".into())
        ));
        if let (Some(raw), Some(scale)) = (s.raw, s.scale) {
            ui.label(format!("≈ {:.3} (raw × scale)", raw as f64 * scale));
        }

        ui.separator();
        ui.horizontal(|ui| {
            ui.label("hardwaregain:");
            for g in &s.gain_avail {
                let selected = s.gain == Some(*g);
                if ui.selectable_label(selected, format!("x{g}")).clicked() && !selected {
                    if let Some(d) = &device {
                        self.fire(
                            "set hardwaregain",
                            format!("echo {g} > {}/in_illuminance_hardwaregain", d.base),
                        );
                    }
                }
            }
        });

        ui.horizontal(|ui| {
            ui.label("integration_time:");
            let locked = s
                .regs
                .map(|r| abi::decode(&r).cal_atime != 0)
                .unwrap_or(false);
            if locked {
                let ms = s.regs.map(|r| abi::decode(&r).cal_atime).unwrap_or(0);
                ui.label(format!("locked by factory CAL_ATIME = {ms} ms (read-only)"))
                    .on_hover_text(
                        "CAL_ATIME is non-zero, so write_raw on this attribute returns -EOPNOTSUPP",
                    );
            } else {
                for t in &s.int_time_avail {
                    let selected = s.int_time.map(|c| (c - t).abs() < 1e-6).unwrap_or(false);
                    if ui.selectable_label(selected, format!("{t:.3}s")).clicked() && !selected {
                        if let Some(d) = &device {
                            self.fire(
                                "set integration_time",
                                format!("echo {t} > {}/in_illuminance_integration_time", d.base),
                            );
                        }
                    }
                }
            }
        });
    }

    fn events_section(&mut self, ui: &mut egui::Ui) {
        let device = self.device.clone();
        let s = self.snapshot.clone();

        ui.heading("Threshold window");
        ui.horizontal(|ui| {
            ui.label("falling (low):");
            ui.add(egui::TextEdit::singleline(&mut self.thresh_falling_buf).desired_width(60.0));
            ui.label("rising (high):");
            ui.add(egui::TextEdit::singleline(&mut self.thresh_rising_buf).desired_width(60.0));
            if ui.button("Apply").clicked() {
                let low = self.thresh_falling_buf.trim().parse::<u32>();
                let high = self.thresh_rising_buf.trim().parse::<u32>();
                match (&device, low, high) {
                    (Some(d), Ok(low), Ok(high)) if low <= high => {
                        // Falling first, mirroring docs/user-guide.adoc -- keeps
                        // the window well-formed at every intermediate step.
                        self.fire(
                            "set thresh_falling",
                            format!(
                                "echo {low} > {}/events/in_illuminance_thresh_falling_value",
                                d.base
                            ),
                        );
                        self.fire(
                            "set thresh_rising",
                            format!(
                                "echo {high} > {}/events/in_illuminance_thresh_rising_value",
                                d.base
                            ),
                        );
                    }
                    _ => self.status_msg = "rejected locally: need falling <= rising".to_string(),
                }
            }
            if ui.button("Sync from device").clicked() {
                self.thresh_falling_buf =
                    s.thresh_falling.map(|v| v.to_string()).unwrap_or_default();
                self.thresh_rising_buf =
                    s.thresh_rising.map(|v| v.to_string()).unwrap_or_default();
            }
        });
        ui.label(format!(
            "device reports: falling={} rising={}",
            s.thresh_falling.map(|v| v.to_string()).unwrap_or("-".into()),
            s.thresh_rising.map(|v| v.to_string()).unwrap_or("-".into()),
        ));

        ui.separator();
        let mut either_en = s.either_en.unwrap_or(false);
        if ui
            .checkbox(&mut either_en, "thresh_either_en (drives CONTINUOUS while on)")
            .changed()
        {
            if let Some(d) = &device {
                self.fire(
                    "toggle either_en",
                    format!(
                        "echo {} > {}/events/in_illuminance_thresh_either_en",
                        either_en as u8,
                        d.base
                    ),
                );
            }
        }

        ui.separator();
        ui.heading("Live event log");
        ui.horizontal(|ui| {
            if self.event_stream.is_none() {
                if ui.button("Start monitor").clicked() {
                    self.start_event_monitor();
                }
            } else if ui.button("Stop monitor").clicked() {
                self.stop_event_monitor();
            }
            if ui.button("Clear log").clicked() {
                self.event_log.clear();
            }
        });
        egui::ScrollArea::vertical()
            .max_height(220.0)
            .stick_to_bottom(true)
            .show(ui, |ui| {
                for entry in &self.event_log {
                    match entry {
                        EventLogEntry::Marker(text) => {
                            ui.monospace(text);
                        }
                        EventLogEntry::Event { label, ts_ns, raw, .. } => {
                            let light = raw
                                .map(|v| v.to_string())
                                .unwrap_or_else(|| "reading...".to_string());
                            ui.monospace(format!(
                                "{label} light={light} ts={ts_ns} ns (~{:.3}s epoch)",
                                *ts_ns as f64 / 1e9
                            ));
                        }
                    }
                }
            });
    }

    fn start_event_monitor(&mut self) {
        let Some(device) = self.device.clone() else {
            return;
        };
        let (tx, rx) = mpsc::channel();
        let cmd = format!(
            "{}; sleep 0.2; envcombo-evtcat {}",
            abi::kill_stray_streams_cmd(),
            device.chardev
        );
        match ssh::start_stream(&self.cfg, &cmd, tx) {
            Ok(stream) => {
                self.event_stream = Some(stream);
                self.event_rx = Some(rx);
                self.event_partial.clear();
                self.push_event_entry(EventLogEntry::Marker("-- monitor started --".to_string()));
            }
            Err(e) => self.status_msg = format!("event monitor: {e}"),
        }
    }

    fn stop_event_monitor(&mut self) {
        if let Some(s) = self.event_stream.take() {
            s.stop();
        }
        self.event_rx = None;
        // Belt-and-suspenders: make sure the remote process is actually
        // dead, not just disconnected from -- see kill_stray_streams_cmd.
        self.fire("cleanup", abi::kill_stray_streams_cmd());
        self.push_event_entry(EventLogEntry::Marker("-- monitor stopped --".to_string()));
    }

    fn poll_event_stream(&mut self) {
        let Some(device) = self.device.clone() else {
            return;
        };
        let Some(rx) = &self.event_rx else {
            return;
        };
        let mut entries = Vec::new();
        let mut closed = None;
        while let Ok(event) = rx.try_recv() {
            match event {
                ssh::StreamEvent::Data(chunk) => {
                    self.event_partial.extend_from_slice(&chunk);
                    while self.event_partial.len() >= 16 {
                        let record: Vec<u8> = self.event_partial.drain(..16).collect();
                        let id = u64::from_le_bytes(record[0..8].try_into().unwrap());
                        let ts_ns = i64::from_le_bytes(record[8..16].try_into().unwrap());
                        let seq = self.next_event_seq;
                        self.next_event_seq += 1;
                        entries.push(EventLogEntry::Event {
                            seq,
                            label: abi::describe_event_id(id),
                            ts_ns,
                            raw: None,
                        });

                        // The event record carries no sample value, so fetch the
                        // currently-latched in_illuminance_raw right away -- while
                        // CONTINUOUS (true here, since an event just fired) that's
                        // a cheap read of the same value the crossing check used,
                        // not a fresh conversion.
                        let (tx, read_rx) = mpsc::channel();
                        ssh::exec_async(
                            self.cfg.clone(),
                            format!("cat {}/in_illuminance_raw", device.base),
                            tx,
                        );
                        self.pending_event_reads.push((seq, read_rx));
                    }
                }
                ssh::StreamEvent::Closed { status, stderr } => closed = Some((status, stderr)),
            }
        }
        for entry in entries {
            self.push_event_entry(entry);
        }
        if let Some((status, stderr)) = closed {
            self.event_stream = None;
            self.event_rx = None;
            let msg = if status == 0 {
                "-- monitor process exited --".to_string()
            } else {
                format!(
                    "-- monitor process exited (status {status}): {} --",
                    if stderr.is_empty() { "no output" } else { &stderr }
                )
            };
            self.push_event_entry(EventLogEntry::Marker(msg));
        }
    }

    fn poll_pending_event_reads(&mut self) {
        let mut still = Vec::new();
        for (seq, rx) in self.pending_event_reads.drain(..) {
            match rx.try_recv() {
                Ok(result) => {
                    let value = result.ok().and_then(|bytes| {
                        String::from_utf8_lossy(&bytes).trim().parse::<u16>().ok()
                    });
                    for entry in self.event_log.iter_mut() {
                        if let EventLogEntry::Event { seq: s, raw, .. } = entry {
                            if *s == seq {
                                *raw = value;
                                break;
                            }
                        }
                    }
                }
                Err(TryRecvError::Empty) => still.push((seq, rx)),
                Err(TryRecvError::Disconnected) => {}
            }
        }
        self.pending_event_reads = still;
    }

    fn push_event_entry(&mut self, entry: EventLogEntry) {
        self.event_log.push_back(entry);
        while self.event_log.len() > EVENT_LOG_CAP {
            self.event_log.pop_front();
        }
    }

    fn buffer_section(&mut self, ui: &mut egui::Ui) {
        let device = self.device.clone();
        let s = self.snapshot.clone();

        ui.heading("Scan elements & buffer");
        ui.horizontal(|ui| {
            let mut illum = s.scan_illum_en.unwrap_or(false);
            if ui.checkbox(&mut illum, "in_illuminance_en").changed() {
                if let Some(d) = &device {
                    self.fire(
                        "scan_elements illuminance",
                        format!(
                            "echo {} > {}/scan_elements/in_illuminance_en",
                            illum as u8, d.base
                        ),
                    );
                }
            }
            let mut ts = s.scan_ts_en.unwrap_or(false);
            if ui.checkbox(&mut ts, "in_timestamp_en").changed() {
                if let Some(d) = &device {
                    self.fire(
                        "scan_elements timestamp",
                        format!(
                            "echo {} > {}/scan_elements/in_timestamp_en",
                            ts as u8, d.base
                        ),
                    );
                }
            }
        });
        ui.horizontal(|ui| {
            ui.label("buffer/length:");
            ui.add(egui::TextEdit::singleline(&mut self.buffer_len_buf).desired_width(60.0));
            if ui.button("Set").clicked() {
                if let (Some(d), Ok(len)) =
                    (&device, self.buffer_len_buf.trim().parse::<u32>())
                {
                    self.fire(
                        "set buffer length",
                        format!("echo {len} > {}/buffer/length", d.base),
                    );
                }
            }
        });
        ui.label(format!(
            "device reports: buffer/enable={} buffer/length={}",
            s.buffer_enabled.map(|v| (v as u8).to_string()).unwrap_or("-".into()),
            s.buffer_len.map(|v| v.to_string()).unwrap_or("-".into()),
        ));

        ui.horizontal(|ui| {
            if self.buffer_stream.is_none() {
                if ui.button("Enable buffer + start live plot").clicked() {
                    self.start_buffer_stream();
                }
            } else if ui.button("Disable buffer / stop plot").clicked() {
                self.stop_buffer_stream(true);
            }
        });

        ui.separator();
        ui.heading("Light vs. sample (live)");
        let points: PlotPoints = self.plot_points.iter().copied().collect::<Vec<_>>().into();
        let falling = s.thresh_falling;
        let rising = s.thresh_rising;
        Plot::new("light_plot")
            .height(260.0)
            .legend(Legend::default())
            .show(ui, |plot_ui| {
                plot_ui.line(Line::new(points).name("light"));
                if let Some(low) = falling {
                    plot_ui.hline(
                        HLine::new(low as f64)
                            .name("falling")
                            .color(egui::Color32::LIGHT_BLUE)
                            .style(LineStyle::dashed_loose()),
                    );
                }
                if let Some(high) = rising {
                    plot_ui.hline(
                        HLine::new(high as f64)
                            .name("rising")
                            .color(egui::Color32::LIGHT_RED)
                            .style(LineStyle::dashed_loose()),
                    );
                }
            });
    }

    fn start_buffer_stream(&mut self) {
        let Some(device) = self.device.clone() else {
            return;
        };
        let ts_on = self.snapshot.scan_ts_en.unwrap_or(false);
        let record_len = if ts_on { 16 } else { 2 };
        let cmd = format!(
            // in_illuminance_en is forced on here rather than left to the
            // checkbox above: the IIO core rejects buffer/enable=1 with no
            // scan elements active, and that failure is otherwise silent
            // (chained with ';', not '&&') -- `dd` would then just block
            // forever with no data and no visible error. The kill_stray
            // line clears out any `dd`/evtcat left over from a previous
            // session that's still holding the chardev open (see
            // kill_stray_streams_cmd) -- otherwise this `dd` would fail
            // with EBUSY instead of starting.
            "{kill_stray}; sleep 0.2; \
             echo 1 > {base}/scan_elements/in_illuminance_en; \
             echo envcombo-dev0 > {base}/trigger/current_trigger 2>/dev/null; \
             echo 1 > {base}/buffer/enable; \
             dd if={chardev} bs={record_len} 2>/dev/null",
            kill_stray = abi::kill_stray_streams_cmd(),
            base = device.base,
            chardev = device.chardev,
        );

        let (tx, rx) = mpsc::channel();
        match ssh::start_stream(&self.cfg, &cmd, tx) {
            Ok(stream) => {
                self.buffer_stream = Some(stream);
                self.buffer_rx = Some(rx);
                self.buffer_partial.clear();
                self.plot_points.clear();
                self.plot_record_len = record_len;
                self.sample_index = 0.0;
            }
            Err(e) => self.status_msg = format!("buffer stream: {e}"),
        }
    }

    fn stop_buffer_stream(&mut self, also_disable_device_buffer: bool) {
        if let Some(s) = self.buffer_stream.take() {
            s.stop();
        }
        self.buffer_rx = None;
        if also_disable_device_buffer {
            if let Some(d) = &self.device {
                self.fire("disable buffer", format!("echo 0 > {}/buffer/enable", d.base));
            }
        }
        // Belt-and-suspenders: make sure the remote dd is actually dead,
        // not just disconnected from -- see kill_stray_streams_cmd.
        self.fire("cleanup", abi::kill_stray_streams_cmd());
    }

    fn poll_buffer_stream(&mut self) {
        let Some(rx) = &self.buffer_rx else {
            return;
        };
        let record_len = self.plot_record_len;
        let mut closed = None;
        while let Ok(event) = rx.try_recv() {
            match event {
                ssh::StreamEvent::Data(chunk) if record_len > 0 => {
                    self.buffer_partial.extend_from_slice(&chunk);
                    while self.buffer_partial.len() >= record_len {
                        let record: Vec<u8> = self.buffer_partial.drain(..record_len).collect();
                        let light = u16::from_le_bytes([record[0], record[1]]);
                        self.plot_points.push_back([self.sample_index, light as f64]);
                        self.sample_index += 1.0;
                        while self.plot_points.len() > PLOT_CAP {
                            self.plot_points.pop_front();
                        }
                    }
                }
                ssh::StreamEvent::Data(_) => {}
                ssh::StreamEvent::Closed { status, stderr } => closed = Some((status, stderr)),
            }
        }
        if let Some((status, stderr)) = closed {
            self.buffer_stream = None;
            self.buffer_rx = None;
            if status != 0 {
                self.status_msg = format!(
                    "buffer stream exited (status {status}): {}",
                    if stderr.is_empty() { "no output" } else { &stderr }
                );
            }
        }
    }

    fn state_section(&mut self, ui: &mut egui::Ui) {
        ui.weak("Power-mode / status, decoded from the simulator's debugfs registers");
        let Some(regs) = self.snapshot.regs else {
            ui.label("no register snapshot yet");
            return;
        };
        let d = abi::decode(&regs);
        ui.label(format!("PWR_MODE: {}", d.pwr_mode.label()));
        ui.separator();
        ui.label(format!(
            "STATUS: ALS_INT={} TEMP_RDY={} HUM_RDY={} ALS_RDY={}",
            d.als_int as u8, d.temp_rdy as u8, d.hum_rdy as u8, d.als_rdy as u8
        ));
        ui.separator();
        ui.label(format!(
            "CFG: als_en={} gain=x{} time={}ms",
            d.als_en as u8,
            abi::GAIN_TABLE[d.gain_idx as usize],
            abi::TIME_TABLE_MS[d.time_idx as usize]
        ));
        ui.label(format!(
            "INT_CFG: en={} latch={} pol={}",
            d.int_en as u8, d.int_latch as u8, d.int_pol as u8
        ));
        ui.label(format!(
            "ALS_TH_LOW={} ALS_TH_HIGH={}",
            d.als_th_low, d.als_th_high
        ));
        ui.separator();
        ui.label(
            "This view never writes debugfs -- read-only cross-check only, \
             same rule the test harness and docs/debugging.adoc follow.",
        );
    }

    fn simulator_section(&mut self, ui: &mut egui::Ui) {
        ui.weak("Full 19-byte debugfs register file, read-only");
        let Some(regs) = self.snapshot.regs else {
            ui.label("no register snapshot yet");
            return;
        };
        let d = abi::decode(&regs);

        egui::Grid::new("regs_grid").striped(true).show(ui, |ui| {
            ui.strong("offset");
            ui.strong("bytes");
            ui.strong("field");
            ui.strong("decoded");
            ui.end_row();

            let row = |ui: &mut egui::Ui, off: usize, len: usize, name: &str, decoded: String| {
                ui.monospace(format!("0x{off:02X}"));
                ui.monospace(
                    regs[off..off + len]
                        .iter()
                        .map(|b| format!("{b:02x}"))
                        .collect::<Vec<_>>()
                        .join(" "),
                );
                ui.label(name);
                ui.label(decoded);
                ui.end_row();
            };

            row(ui, abi::off::WHO_AM_I, 1, "WHO_AM_I", format!("0x{:02X}", d.who_am_i));
            row(
                ui,
                abi::off::TEMP_MSB,
                2,
                "TEMP (out of driver scope)",
                d.temp.to_string(),
            );
            row(
                ui,
                abi::off::HUMIDITY,
                1,
                "HUMIDITY (out of driver scope)",
                d.humidity.to_string(),
            );
            row(
                ui,
                abi::off::CFG,
                1,
                "CFG",
                format!(
                    "als_en={} gain_idx={} time_idx={}",
                    d.als_en as u8, d.gain_idx, d.time_idx
                ),
            );
            row(
                ui,
                abi::off::INT_CFG,
                1,
                "INT_CFG",
                format!(
                    "en={} latch={} pol={}",
                    d.int_en as u8, d.int_latch as u8, d.int_pol as u8
                ),
            );
            row(ui, abi::off::ALS_MSB, 2, "ALS", d.als.to_string());
            row(ui, abi::off::ALS_TH_LOW_MSB, 2, "ALS_TH_LOW", d.als_th_low.to_string());
            row(ui, abi::off::ALS_TH_HIGH_MSB, 2, "ALS_TH_HIGH", d.als_th_high.to_string());
            row(
                ui,
                abi::off::STATUS,
                1,
                "STATUS",
                format!("ALS_INT={} ALS_RDY={}", d.als_int as u8, d.als_rdy as u8),
            );
            row(
                ui,
                abi::off::CAL_TOFF_MSB,
                2,
                "CAL_TOFF (out of driver scope)",
                d.cal_toff.to_string(),
            );
            row(
                ui,
                abi::off::CAL_HOFF,
                1,
                "CAL_HOFF (out of driver scope)",
                d.cal_hoff.to_string(),
            );
            row(ui, abi::off::CAL_AGAIN, 1, "CAL_AGAIN", d.cal_again.to_string());
            row(
                ui,
                abi::off::CAL_ATIME,
                1,
                "CAL_ATIME",
                format!(
                    "{} ms{}",
                    d.cal_atime,
                    if d.cal_atime != 0 { " (overrides integration_time)" } else { "" }
                ),
            );
            row(ui, abi::off::PWR_MODE, 1, "PWR_MODE", d.pwr_mode.label().to_string());
        });
    }

    fn capture_section(&mut self, ui: &mut egui::Ui) {
        ui.label(
            "Pulls bytes straight over the open SSH connection to a local file -- \
             replaces the manual nc-listener dance in docs/debugging.adoc.",
        );

        ui.horizontal(|ui| {
            ui.label("export directory:");
            ui.add(egui::TextEdit::singleline(&mut self.export_dir).desired_width(220.0));
        });

        ui.separator();
        if ui.button("Save register snapshot (regs.bin)").clicked() {
            self.export_regs();
        }

        ui.separator();
        ui.horizontal(|ui| {
            ui.label("record count:");
            ui.add(egui::TextEdit::singleline(&mut self.export_count_buf).desired_width(60.0));
            let ts_on = self.snapshot.scan_ts_en.unwrap_or(false);
            ui.label(if ts_on {
                "(16-byte records: light + timestamp)"
            } else {
                "(2-byte records: light only)"
            });
            if ui.button("Capture buffer records").clicked() {
                self.export_buffer();
            }
        });

        if self.export_inflight {
            ui.spinner();
        }
        if !self.export_status.is_empty() {
            ui.label(&self.export_status);
        }

        ui.separator();
        ui.heading("Modules");
        ui.horizontal(|ui| {
            if ui.button("modprobe i2c-envcombo-sim && envcombo").clicked() {
                self.fire(
                    "modprobe",
                    "modprobe i2c-envcombo-sim && modprobe envcombo".to_string(),
                );
            }
            if ui.button("rmmod envcombo && i2c_envcombo_sim").clicked() {
                self.fire(
                    "rmmod",
                    "rmmod envcombo && rmmod i2c_envcombo_sim".to_string(),
                );
            }
        });
    }

    fn export_regs(&mut self) {
        let (tx, rx) = mpsc::channel();
        ssh::exec_async(
            self.cfg.clone(),
            format!("hexdump -ve '1/1 \"%02x\"' {}", abi::DEBUGFS_REGS),
            tx,
        );
        self.export_rx = Some(rx);
        self.export_kind = Some(ExportKind::Regs);
        self.export_inflight = true;
        self.export_status = "capturing register snapshot...".to_string();
    }

    fn export_buffer(&mut self) {
        let Some(device) = self.device.clone() else {
            return;
        };
        let Ok(n) = self.export_count_buf.trim().parse::<u32>() else {
            self.export_status = "invalid record count".to_string();
            return;
        };
        let ts_on = self.snapshot.scan_ts_en.unwrap_or(false);
        let record_len = if ts_on { 16 } else { 2 };
        let cmd = format!(
            // in_illuminance_en forced on, kill_stray prepended -- see
            // start_buffer_stream for why both are needed.
            "{kill_stray}; sleep 0.2; \
             echo 1 > {base}/scan_elements/in_illuminance_en; \
             echo envcombo-dev0 > {base}/trigger/current_trigger 2>/dev/null; \
             echo {len} > {base}/buffer/length; \
             echo 1 > {base}/buffer/enable; \
             dd if={chardev} bs={record_len} count={n} 2>/dev/null; \
             echo 0 > {base}/buffer/enable",
            kill_stray = abi::kill_stray_streams_cmd(),
            base = device.base,
            len = n.max(1),
            chardev = device.chardev,
        );
        let (tx, rx) = mpsc::channel();
        ssh::exec_async(self.cfg.clone(), cmd, tx);
        self.export_rx = Some(rx);
        self.export_kind = Some(ExportKind::Buffer);
        self.export_inflight = true;
        self.export_status = format!(
            "capturing {n} records (~{:.0}s at 200ms/sample while CONTINUOUS)...",
            n as f64 * 0.2
        );
    }

    fn poll_export(&mut self) {
        let Some(rx) = &self.export_rx else {
            return;
        };
        match rx.try_recv() {
            Ok(Ok(bytes)) => {
                let kind = self.export_kind.take();
                let named = match kind {
                    Some(ExportKind::Regs) => {
                        let hex_str = String::from_utf8_lossy(&bytes).trim().to_string();
                        decode_hex(&hex_str).map(|raw| ("regs.bin", raw))
                    }
                    Some(ExportKind::Buffer) => Some(("buffer_records.bin", bytes)),
                    None => None,
                };
                match named {
                    Some((name, raw)) => {
                        let path = std::path::Path::new(&self.export_dir).join(name);
                        match std::fs::write(&path, &raw) {
                            Ok(()) => {
                                self.export_status =
                                    format!("wrote {} ({} bytes)", path.display(), raw.len())
                            }
                            Err(e) => {
                                self.export_status = format!("write {}: {e}", path.display())
                            }
                        }
                    }
                    None => self.export_status = "capture returned no usable data".to_string(),
                }
                self.export_inflight = false;
                self.export_rx = None;
            }
            Ok(Err(e)) => {
                self.export_status = format!("capture failed: {e}");
                self.export_inflight = false;
                self.export_rx = None;
            }
            Err(TryRecvError::Empty) => {}
            Err(TryRecvError::Disconnected) => {
                self.export_inflight = false;
                self.export_rx = None;
            }
        }
    }
}

fn collapsing_section(
    ui: &mut egui::Ui,
    title: &str,
    default_open: bool,
    add_contents: impl FnOnce(&mut egui::Ui),
) {
    egui::CollapsingHeader::new(egui::RichText::new(title).heading())
        .default_open(default_open)
        .show(ui, add_contents);
    ui.add_space(4.0);
}

fn decode_hex(s: &str) -> Option<Vec<u8>> {
    if s.len() % 2 != 0 {
        return None;
    }
    (0..s.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&s[i..i + 2], 16).ok())
        .collect()
}
