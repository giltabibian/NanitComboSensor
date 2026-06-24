SUMMARY = "ENV-COMBO IIO event-fd cat helper"
DESCRIPTION = "Opens an IIO chardev's event fd via IIO_GET_EVENT_FD_IOCTL and streams raw iio_event_data records to stdout, for tools that can't issue the ioctl themselves (e.g. a remote shell)."
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"

SRC_URI = "file://envcombo-evtcat"

S = "${WORKDIR}/envcombo-evtcat"

inherit cargo

require ${BPN}-crates.inc

RDEPENDS:${PN} = "envcombo-driver"
