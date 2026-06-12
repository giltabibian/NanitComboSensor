SUMMARY = "ENV-COMBO IIO sensor driver"
DESCRIPTION = "IIO driver for the ENV-COMBO temperature/humidity/ambient light sensor"
LICENSE = "GPL-2.0-only"
LIC_FILES_CHKSUM = "file://envcombo.c;beginline=1;endline=1;md5=50d2ba0afecd20f74c12a4bdbcfcfe61"

inherit module

SRC_URI = "file://envcombo.c \
           file://Makefile"

S = "${WORKDIR}"

RPROVIDES:${PN} += "kernel-module-envcombo"

# Load at boot; binds as soon as the simulator registers the i2c client,
# regardless of which module comes up first.
KERNEL_MODULE_AUTOLOAD += "envcombo"
