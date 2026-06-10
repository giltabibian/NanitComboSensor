SUMMARY = "ENV-COMBO driver test image"
LICENSE = "MIT"

inherit core-image

require recipes-core/images/core-image-minimal.bb

IMAGE_INSTALL += " \
    kernel-modules \
    kernel-module-envcombo \
    kernel-module-i2c-envcombo-sim \
    envcombo-test \
"
