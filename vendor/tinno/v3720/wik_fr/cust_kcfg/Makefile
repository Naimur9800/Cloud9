# guomingyi20150113 modify.
###############################################################################################################################################
# 以下开关用于控制kernel层中的行为，以满足订单的特殊需求.
# 以下定义根据各个原型实际使用器件不同而不同,本文件以s4710原型为例.
# 如果订单项目不需要某个器件(如 m-sensor)，就直接去掉以下注释，同时修改ProjectConfig.mk中的定义
# 如果订单项目需要增加某个kernel层的修改，可以定义一个宏：ccflags-y += -DCONFIG_PROJECT_S4710_TRUNK 来实现控制.
###############################################################################################################################################

# m-sensor.
# CONFIG_MTK_AKM09911 := n

# p-sensor.
# CONFIG_MTK_LTR553 := n

# g-sensor.
# CONFIG_MTK_KXTJ2_1009 := n

###############################################################################################################################################
# user define.
PROJECT_NAME := $(strip $(subst ro.target =,,$(shell cat $(MAIN_KERNEL_PATH)/build.ini |grep ^ro.target)))
ccflags-y += -DCONFIG_PROJECT_$(PROJECT_NAME)_TRUNK

PRJ_DEF_FLAGS := $(strip $(subst PRJ_DEF_FLAGS =,,$(shell cat $(MAIN_KERNEL_PATH)/build.ini |grep ^PRJ_DEF_FLAGS)))
ifneq ($(strip $(PRJ_DEF_FLAGS)),)
ccflags-y += -D$(PRJ_DEF_FLAGS)
$(warning ---ccflags-y += -D$(PRJ_DEF_FLAGS)---)
endif

# m-sensor.
CONFIG_CUSTOM_KERNEL_MAGNETOMETER=y
# CONFIG_MTK_AKM09911_NEW=y
#CONFIG_MTK_AKM09916_NEW=y

#hall
CONFIG_MTK_TINNO_HALL=y

#GYRO
# CONFIG_CUSTOM_KERNEL_GYROSCOPE=y
# CONFIG_MTK_MPU6515G=y
# CONFIG_MTK_MPU6515A=y

# no GYRO open BMA222E
#CONFIG_MTK_BMA222E_NEW=y



#单软多硬 for BLU
#ccflags-y += -DCONFIG_PROJECT_V3720_SINGLE_SW_MULTI_HW











