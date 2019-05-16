include $(srctree)/drivers/misc/mediatek/Makefile.custom


obj-$(CONFIG_MTK_TINNO_HALL)	+= hall_device.o

# TINNO BEGIN
# FEATURE_REQS_UNIFY, lejun.wang, DATE20161018, for wiko unify
FEATURE_REQS_UNIFY := $(strip $(subst ro.feature.reqsunify =,,$(shell cat $(MAIN_KERNEL_PATH)/build.ini |grep ^ro.feature.reqsunify)))
ifeq ($(strip $(FEATURE_REQS_UNIFY)),1)
ccflags-y += -DFEATURE_REQS_UNIFY
$(warning ---add local macro FEATURE_REQS_UNIFY---)
endif
# TINNO END
