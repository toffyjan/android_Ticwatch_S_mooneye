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

#$(call all-subdir-src-or-makefile)
MTK_PLATFORM := $(subst ",,$(CONFIG_MTK_PLATFORM))
subdir-ccflags-y += -Werror
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/hibcore
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat
subdir-ccflags-y += -I$(srctree)/drivers/mmc/host/mediatek/$(MTK_PLATFORM)

ifeq ($(CONFIG_ARCH_MT2601),y)
include $(srctree)/drivers/misc/mediatek/Makefile.custom
endif


obj-$(CONFIG_MTK_BASE_POWER) += base/
#obj-$(CONFIG_MEDIATEK_SOLUTION) += sched/
#obj-$(CONFIG_MEDIATEK_SOLUTION) += mach/
obj-$(CONFIG_MTK_SERIAL)	+= uart/
obj-$(CONFIG_MTK_SYS_CIRQ) += cirq/

obj-y += mmp/
obj-$(CONFIG_MTK_AUXADC)	+= auxadc/
obj-$(CONFIG_MTK_KEYPAD)	+= keypad/
obj-$(CONFIG_MTK_M4U)	+= m4u/
obj-$(CONFIG_MTK_GPT_SCHEME_SUPPORT)	+= partition/
obj-$(CONFIG_MTK_PWM)	    += pwm/

obj-$(CONFIG_CUSTOM_KERNEL_SENSORHUB) += sensorhub/

ifeq ($(CONFIG_MTK_I2C_EXTENSION),y)
obj-$(CONFIG_MTK_I2C)  += i2c/
endif
obj-$(CONFIG_MTK_LEDS)	+= leds/
obj-$(CONFIG_MTK_VIBRATOR)	+= vibrator/
#obj-$(CONFIG_MTK_SM100)	+= tc1_interface/vib_drv/
obj-$(CONFIG_MTK_ACCDET)	+= accdet/
obj-y += irtx/
obj-$(CONFIG_MTK_RTC)	+= rtc/
obj-$(CONFIG_REG_MONITOR_DRIVER) += reg_monitor/
obj-$(CONFIG_MTK_PMIC)	+= power/
ifeq ($(CONFIG_MTK_GAUGE_VERSION),30)
obj-y	+= power/
endif
obj-$(CONFIG_MTK_PMIC_NEW_ARCH)	+= pmic/
obj-$(CONFIG_MTK_PMIC_WRAP_HAL) += pmic_wrap/
obj-$(CONFIG_USB_MTK_HDRC) += usb20/
obj-$(CONFIG_MTK_USBFSH) += usb11/
obj-$(CONFIG_MTK_GPIO) += gpio/
obj-$(CONFIG_MTK_MT6306_GPIO_SUPPORT) += gpio/
obj-$(CONFIG_SECURITY_SELINUX)	+= selinux_warning/
obj-$(CONFIG_MTK_MEM) += mem/
obj-$(CONFIG_MTK_MEMORY_LOWPOWER) += memory-lowpower/
obj-$(CONFIG_MTK_BOOT) += boot/
obj-y += devinfo/
obj-$(CONFIG_CRYPTO_DEV_DX_CC63P) += dxcc/
#obj-$(CONFIG_MTK_LCM) += lcm/
obj-$(CONFIG_MTK_LCM) += ext_disp/
obj-$(CONFIG_MTK_FB) += video/
obj-$(CONFIG_MTK_DYNAMIC_FPS_FRAMEWORK_SUPPORT) += dfrc/
obj-$(CONFIG_MTK_GPU_SUPPORT) += gpu/
obj-$(CONFIG_MTK_SMI_EXT)	+= smi/
obj-$(CONFIG_MTK_SYNC) += sync/
obj-$(CONFIG_MTK_FREQ_HOPPING) += freqhopping/
#obj-$(CONFIG_MTK_DRAMC) += dramc/
#obj-$(CONFIG_MTK_SYNC) += sync/
obj-$(CONFIG_MTK_FREQ_HOPPING) += freqhopping/
obj-$(CONFIG_MTK_SECURITY_SW_SUPPORT) +=masp/
obj-$(CONFIG_MTK_BLOCK_TAG) +=blocktag/
#obj-$(CONFIG_MEDIATEK_SOLUTION)  += sched/
obj-y += cameraisp/src/
obj-$(CONFIG_MTK_CCU)	+= ccu/src/
obj-$(CONFIG_MTK_CCCI_DEVICES)	+=  ccci_util/
obj-$(CONFIG_MTK_NET_CCMNI) +=  ccmni/
obj-$(CONFIG_MTK_ECCCI_DRIVER) +=  eccci/

#obj-$(CONFIG_MTK_HIBERNATION) += hibcore/

obj-$(CONFIG_MTK_COMBO) += connectivity/
obj-$(CONFIG_MTK_BTIF) += btif/

obj-$(CONFIG_MTK_RAM_CONSOLE)	+= ram_console/
obj-$(CONFIG_RT_REGMAP) += rt-regmap/


obj-y += performance/

# gator driver is platform-independent
ifneq ($(wildcard $(srctree)/$(src)/gator-driver/),)
obj-$(CONFIG_GATOR_DRIVER) += gator-driver/
endif # gator-driver folder exist


obj-$(CONFIG_THERMAL)   += thermal/

obj-$(CONFIG_MEDIATEK_SOLUTION)	+= aee/
#obj-$(CONFIG_MTK_MLOG) += mlog/
obj-$(CONFIG_MTK_BTCVSD) += btcvsd/
#obj-$(CONFIG_MTK_DYNAMIC_BOOST) += dynamic_boost/

#widevine drm
obj-y += sspm/
ifneq ($(wildcard $(srctree)/drivers/misc/mediatek/cpu_stress/),)
obj-$(CONFIG_MTK_CPU_STRESS)	+= cpu_stress/
endif
obj-$(CONFIG_USB)	+= usb_boost/
#obj-$(CONFIG_RT_REGMAP) += rt-regmap/
ifneq ($(CONFIG_ARCH_MT2601),y)
obj-$(CONFIG_MEDIATEK_SOLUTION)	+= cmo/
endif
obj-$(CONFIG_MTK_SMARTPA_SOUND)	+= smartpa/
#obj-y += mtprintk/
obj-$(CONFIG_MTK_SOUND)	+= sound/
