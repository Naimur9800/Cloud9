#
# Copyright (C) 2015 MediaTek Inc.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#

#
# for USB OTG silicon based on Mentor Graphics INVENTRA designs
#
include $(srctree)/drivers/misc/mediatek/Makefile.custom
ccflags-$(CONFIG_USB_MTK_HDRC) += -I$(srctree)/drivers/misc/mediatek/usb20

obj-$(CONFIG_USB_MTK_HDRC)	:= usb20.o usb20_phy.o
obj-$(CONFIG_USB_MTK_HDRC)  += usb20_host.o
obj-$(CONFIG_USBIF_COMPLIANCE)   += usb20_otg_if.o

# TINNO BEGIN
# FEATURE_REQS_UNIFY, lejun.wang, DATE20161018, for wiko unify
FEATURE_REQS_UNIFY := $(strip $(subst ro.feature.reqsunify =,,$(shell cat $(MAIN_KERNEL_PATH)/build.ini |grep ^ro.feature.reqsunify)))
ifeq ($(strip $(FEATURE_REQS_UNIFY)),1)
ccflags-y += -DFEATURE_REQS_UNIFY
$(warning ---add local macro FEATURE_REQS_UNIFY---)
endif
# TINNO END
