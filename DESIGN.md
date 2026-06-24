# Design Notes

The architecture and design documentation lives in [`docs/`](docs/index.html),
authored in AsciiDoc and rendered to HTML. The documentation root links to
every page:

1. [Architecture & Design](docs/architecture.html) — hardware model and
   register map, driver components and locking, the power-mode state machine,
   ALS usage flows, the userspace ABI, and the design decisions with their
   tradeoffs.
2. [Test Harness](docs/test-harness.html) — the Rust end-to-end test harness:
   test plan, what each test asserts and how it can fail, and the cross-check
   strategy against the simulator's debugfs register file.
3. [Installation, Build & Run](docs/user-guide.html) — adding the layer,
   building the image, booting it in QEMU, exercising every feature from the
   shell, and running the test harness.
4. [Manual Debugging & Binary Inspection](docs/debugging.html) — host-side
   decoding of the device's binary interfaces: register file, captured ALS
   buffers, full timestamped records, the SMBus transaction waveforms, and
   live I2C traffic capture. **Recommended:** this page walks through how I
   tackled bugs and inspected the driver's functionality on a running system.
5. [Host GUI (envcombo-ctl)](docs/envcombo-ctl.html) — bonus tooling: a
   Rust/egui desktop app that drives the full driver ABI over SSH instead of
   by hand.
