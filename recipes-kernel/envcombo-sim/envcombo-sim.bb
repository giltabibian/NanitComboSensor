SUMMARY = "ENV-COMBO I2C device simulator"
DESCRIPTION = "Kernel module that simulates the ENV-COMBO I2C sensor device"
LICENSE = "GPL-2.0-only"
LIC_FILES_CHKSUM = "file://i2c-envcombo-sim.c;beginline=1;endline=1;md5=50d2ba0afecd20f74c12a4bdbcfcfe61"

inherit module

SRC_URI = "file://i2c-envcombo-sim.c \
           file://Makefile"

S = "${WORKDIR}"

RPROVIDES:${PN} += "kernel-module-i2c-envcombo-sim"
