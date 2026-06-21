SUMMARY = "ENV-COMBO driver test image"
LICENSE = "MIT"

inherit core-image

require recipes-core/images/core-image-minimal.bb

# SSH access for the host-side envcombo-ctl GUI (host-tools/envcombo-ctl).
# -B allows the image's blank root password to keep working over SSH;
# dropbear refuses empty-password logins by default otherwise.
IMAGE_FEATURES += "ssh-server-dropbear"
DROPBEAR_EXTRA_ARGS = "-B"

IMAGE_INSTALL += " \
    kernel-modules \
    kernel-module-envcombo \
    kernel-module-i2c-envcombo-sim \
    envcombo-test \
    envcombo-evtcat \
"
