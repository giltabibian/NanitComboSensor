SUMMARY = "ENV-COMBO driver test image"
LICENSE = "MIT"

inherit core-image

require recipes-core/images/core-image-minimal.bb

# SSH access for the host-side envcombo-ctl GUI (host-tools/envcombo-ctl).
# allow-root-login / allow-empty-password are the actual knobs: OE-core's
# rootfs postcommands rewrite /etc/default/dropbear's DROPBEAR_EXTRA_ARGS
# (shipped as "-w", disallow root) based on these IMAGE_FEATURES. Setting
# DROPBEAR_EXTRA_ARGS directly from this recipe is a no-op -- it lives in
# the image recipe's own namespace, not the dropbear package's.
IMAGE_FEATURES += "ssh-server-dropbear allow-root-login allow-empty-password"

IMAGE_INSTALL += " \
    kernel-modules \
    kernel-module-envcombo \
    kernel-module-i2c-envcombo-sim \
    envcombo-test \
    envcombo-evtcat \
"
