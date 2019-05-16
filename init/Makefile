#
# Makefile for the linux kernel.
#

subdir-ccflags-y += -Werror
ccflags-y += -Idrivers/misc/mediatek/mtprof/

obj-y                          := main.o version.o mounts.o
obj-y                          += noinitramfs.o
obj-$(CONFIG_BLK_DEV_INITRD)   += initramfs.o
obj-$(CONFIG_GENERIC_CALIBRATE_DELAY) += calibrate.o

ifneq ($(CONFIG_ARCH_INIT_TASK),y)
obj-y                          += init_task.o
endif

mounts-y			:= do_mounts.o
mounts-$(CONFIG_BLK_DEV_RAM)	+= do_mounts_rd.o
mounts-$(CONFIG_BLK_DEV_INITRD)	+= do_mounts_initrd.o
mounts-$(CONFIG_BLK_DEV_MD)	+= do_mounts_md.o
mounts-$(CONFIG_BLK_DEV_DM)	+= do_mounts_dm.o

# dependencies on generated files need to be listed explicitly
$(obj)/version.o: include/generated/compile.h

# TINNO BEGIN
# FEATURE_REQS_UNIFY, lejun.wang, DATE20161018, for wiko unify
FEATURE_REQS_UNIFY := $(strip $(subst ro.feature.reqsunify =,,$(shell cat $(MAIN_KERNEL_PATH)/build.ini |grep ^ro.feature.reqsunify)))
ifeq ($(strip $(FEATURE_REQS_UNIFY)),1)
$(warning -wlj--PWD= $(PWD)-------)
$(warning -wlj--FEATURE_REQS_UNIFY= $(FEATURE_REQS_UNIFY))
ccflags-y += -DFEATURE_REQS_UNIFY
$(warning -wlj---ccflags-y += -DFEATURE_REQS_UNIFY---)
else
FEATURE_REQS_UNIFY := $(strip $(subst ro.feature.reqsunify =,,$(shell cat $(MAIN_KERNEL_PATH)/build.ini |grep ^ro.feature.reqsunify)))
ifeq ($(strip $(FEATURE_REQS_UNIFY)),1)
$(warning -wlj--PWD= $(PWD)-------)
$(warning -wlj--FEATURE_REQS_UNIFY= $(FEATURE_REQS_UNIFY))
ccflags-y += -DFEATURE_REQS_UNIFY
$(warning ---add local macro FEATURE_REQS_UNIFY---)
endif
endif
# TINNO END

# compile.h changes depending on hostname, generation number, etc,
# so we regenerate it always.
# mkcompile_h will make sure to only update the
# actual file if its content has changed.

       chk_compile.h = :
 quiet_chk_compile.h = echo '  CHK     $@'
silent_chk_compile.h = :
include/generated/compile.h: FORCE
	@$($(quiet)chk_compile.h)
	$(Q)$(CONFIG_SHELL) $(srctree)/scripts/mkcompile_h $@ \
	"$(UTS_MACHINE)" "$(CONFIG_SMP)" "$(CONFIG_PREEMPT)" "$(CC) $(KBUILD_CFLAGS)"
