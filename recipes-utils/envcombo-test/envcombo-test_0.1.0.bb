SUMMARY = "ENV-COMBO IIO driver test harness"
DESCRIPTION = "Userspace test harness that exercises the envcombo IIO driver end-to-end."
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"

SRC_URI = "file://envcombo-test"

S = "${WORKDIR}/envcombo-test"

inherit cargo

require ${BPN}-crates.inc

RDEPENDS:${PN} = "envcombo-driver envcombo-sim"
