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

Full documentation lives in [`docs/`](docs/index.adoc) (rendered HTML
alongside the sources):

1. [Architecture & Design](docs/architecture.adoc) — register map, driver
   components, power FSM, usage flows, ABI, design decisions.
2. [Test Harness](docs/test-harness.adoc) — test plan, block diagram of the
   harness/driver/simulator stack, per-test assertions.
3. [Installation, Build & Run](docs/user-guide.adoc) — every build, boot,
   and manual-verification command, step by step.

## Layer contents

```
meta-nanit-envcombo/
├── conf/layer.conf
├── recipes-kernel/
│   ├── envcombo-driver/          # IIO driver module (your code)
│   ├── envcombo-sim/             # I2C bus & device simulator (provided)
│   └── linux/                    # Kernel config fragment
├── recipes-core/images/
│   └── envcombo-image.bb         # Image recipe
└── recipes-utils/envcombo-test/
    ├── envcombo-test_0.1.0.bb    # Test harness recipe (Rust/Cargo)
    └── files/envcombo-test/      # Test harness source
```
