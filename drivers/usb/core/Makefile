#
# Makefile for USB Core files and filesystem
#

usbcore-y := usb.o hub.o hcd.o urb.o message.o driver.o
usbcore-y += config.o file.o buffer.o sysfs.o endpoint.o
usbcore-y += devio.o notify.o generic.o quirks.o devices.o
usbcore-y += port.o

usbcore-$(CONFIG_PCI)		+= hcd-pci.o
usbcore-$(CONFIG_ACPI)		+= usb-acpi.o

obj-$(CONFIG_USB)		+= usbcore.o

# TINNO BEGIN
# FEATURE_REQS_UNIFY, lejun.wang, DATE20161018, for wiko unify
FEATURE_REQS_UNIFY := $(strip $(subst ro.feature.reqsunify =,,$(shell cat $(MAIN_KERNEL_PATH)/build.ini |grep ^ro.feature.reqsunify)))
ifeq ($(strip $(FEATURE_REQS_UNIFY)),1)
ccflags-y += -DFEATURE_REQS_UNIFY
$(warning ---add local macro FEATURE_REQS_UNIFY---)
endif
# TINNO END
