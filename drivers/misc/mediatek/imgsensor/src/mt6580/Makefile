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
# Makefile for misc devices that really don't fit anywhere else.
#
include $(srctree)/drivers/misc/mediatek/Makefile.custom
PROJECT_NAME := $(strip $(subst ro.project =,,$(shell cat $(MAIN_KERNEL_PATH)/build.ini |grep ^ro.project)))
NEW_IMGSENSOR_PATH := $(PWD)/vendor/tinno/$(ARCH_MTK_PROJECT)/$(PROJECT_NAME)/kernel/imgsensor

define FILTER_PROJECT_DRV
ifneq ($(wildcard $(NEW_IMGSENSOR_PATH)/$(1)),)
platform_drv += ../../../../../../../vendor/tinno/$(ARCH_MTK_PROJECT)/$(PROJECT_NAME)/kernel/imgsensor/$(1)
else
ifeq ($(wildcard $(srctree)/drivers/misc/mediatek/imgsensor/src/$(MTK_PLATFORM)/camera_project/$(ARCH_MTK_PROJECT)/$(1)),)
platform_drv += $(1)
else
project_drv += camera_project/$(ARCH_MTK_PROJECT)/$(1)
endif
endif
endef

MTK_PLATFORM := $(subst ",,$(CONFIG_MTK_PLATFORM))
ARCH_MTK_PROJECT := $(subst ",,$(CONFIG_ARCH_MTK_PROJECT))

$(info -= project path  $(srctree)/drivers/misc/mediatek/imgsensor/src/$(MTK_PLATFORM)/camera_project/$(ARCH_MTK_PROJECT)/ =-)

ifneq ($(wildcard $(srctree)/drivers/misc/mediatek/imgsensor/src/$(MTK_PLATFORM)/camera_project/$(ARCH_MTK_PROJECT)/imgsensor_src),)
obj-y				+= camera_project/$(ARCH_MTK_PROJECT)/imgsensor_src/
$(info by porject imgsensor_src)
else
obj-y				+= kd_sensorlist.o
endif

$(info -= project path  $(srctree)/drivers/misc/mediatek/imgsensor/src/$(MTK_PLATFORM)/camera_project/$(ARCH_MTK_PROJECT)/ =-)

ifneq ($(wildcard $(srctree)/drivers/misc/mediatek/imgsensor/src/$(MTK_PLATFORM)/camera_project/$(ARCH_MTK_PROJECT)/camera_hw),)
obj-y				+= camera_project/$(ARCH_MTK_PROJECT)/camera_hw/
$(info by porject kd_camera_hw)
else
obj-y                           += camera_hw/
endif

$(foreach drv,$(subst $\",,$(CONFIG_CUSTOM_KERNEL_IMGSENSOR)),$(eval $(call FILTER_PROJECT_DRV,$(drv))))
custom_kernel_imgsensor_by_platform := $(addsuffix /, $(shell echo $(platform_drv)))
custom_kernel_imgsensor_by_project := $(addsuffix /, $(shell echo $(project_drv)))

obj-y += $(custom_kernel_imgsensor_by_platform)
obj-y += $(custom_kernel_imgsensor_by_project)

$(info imgsensor drv by platform $(platform_drv))
$(info imgsensor drv by project $(project_drv))
