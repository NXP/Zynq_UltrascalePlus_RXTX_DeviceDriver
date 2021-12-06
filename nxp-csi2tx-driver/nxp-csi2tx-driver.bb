SUMMARY = "Recipe for  build an external nxp-csi2tx-driver Linux kernel module"
SECTION = "PETALINUX/modules"
LICENSE = "GPLv2"
LIC_FILES_CHKSUM = "file://COPYING;md5=12f884d2ae1ff87c09e5b7ccc2c4ca7e"

inherit module

INHIBIT_PACKAGE_STRIP = "1"

SRC_URI = "file://Makefile \
           file://nxp-csi2tx-driver.c \
           file://nxp-csi2tx-driver.h \
	   file://COPYING \
          "

S = "${WORKDIR}"
KERNEL_MODULE_AUTOLOAD += "nxp-csi2tx-driver"

# The inherit of module.bbclass will automatically name module packages with
# "kernel-module-" prefix as required by the oe-core build environment.
