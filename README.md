# meta-nanit-envcombo

Yocto layer for the ENV-COMBO sensor driver assignment. Targets QEMU
qemuarm64 on Yocto 5.0 (scarthgap).

Unpack this layer at the poky root (so that `meta-nanit-envcombo/` sits
alongside `meta/`, `bitbake/`, etc.). The image recipe already includes
`envcombo-test`.

## Quick start

```bash
# From the poky root directory
source oe-init-build-env build

# Add the layer
bitbake-layers add-layer ../meta-nanit-envcombo

# Set machine in local.conf
# MACHINE ?= "qemuarm64"

# Build the image
bitbake envcombo-image

# Boot
runqemu qemuarm64 nographic slirp
# Login: root (no password)

# Both modules autoload at boot; just run the tests
envcombo-test
```

## Documentation

Full documentation lives in [`docs/`](docs/index.adoc), authored in AsciiDoc
and rendered to HTML alongside the sources. The pages embed SVG diagrams,
flow charts, and screenshots that GitHub's web view does not render reliably —
**for the intended experience, clone the repo and open
[`docs/index.html`](docs/index.html) in a browser.**

0. [Background](docs/background.adoc) — IIO framework primer (what IIO is,
   driver structure, sysfs interface, buffering/triggers, IIO vs hwmon) and
   I2C/SMBus overview (two-wire bus, SMBus helpers, 16-bit word transactions).
1. [Architecture & Design](docs/architecture.adoc) — hardware/register map,
   driver components and locking, the power-mode FSM, ALS usage flows, the
   userspace ABI, and the design decisions with their tradeoffs.
2. [Test Harness](docs/test-harness.adoc) — test plan, block diagram of the
   harness/driver/simulator stack, and what each of the 15 tests asserts.
3. [Installation, Build & Run](docs/user-guide.adoc) — every build, boot,
   and manual-verification command, step by step.
4. [Manual Debugging & Binary Inspection](docs/debugging.adoc) — decoding the
   device register file and captured ALS buffers with ImHex, reading
   timestamped records, and the SMBus transaction timing diagrams.
5. [Host GUI (envcombo-ctl)](docs/envcombo-ctl.adoc) — a Rust/egui desktop
   app, run on the host, that drives the full driver ABI over SSH: live
   channel/event/buffer control plus one-click register/buffer capture for
   ImHex. Bonus tooling, not part of the bitbake build.

To re-render after editing (requires `asciidoctor` and `plantuml`):

```bash
plantuml -tsvg -o svg docs/diagrams/*.puml   # diagrams -> docs/diagrams/svg/
asciidoctor docs/*.adoc                      # pages    -> docs/*.html
```

## Layer contents

```
meta-nanit-envcombo/
├── conf/layer.conf
├── recipes-kernel/
│   ├── envcombo-driver/          # IIO driver module (your code)
│   ├── envcombo-sim/             # I2C bus & device simulator (provided)
│   └── linux/                    # Kernel config fragment
├── recipes-core/images/
│   └── envcombo-image.bb         # Image recipe (now also enables SSH)
├── recipes-utils/
│   ├── envcombo-test/             # Rust test harness recipe + source
│   └── envcombo-evtcat/          # On-target event-fd helper for the host GUI
└── host-tools/envcombo-ctl/      # Host-side GUI (not part of the bitbake build)
```
