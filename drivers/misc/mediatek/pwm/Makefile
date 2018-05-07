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
ccflags-y += -I$(srctree)/drivers/misc/mediatek/pwm/$(CONFIG_MTK_PLATFORM)/include

ifneq ($(CONFIG_ARCH_MT2601),y)
obj-$(CONFIG_MTK_PWM)	+= mtk_pwm.o
else
obj-$(CONFIG_MTK_PWM)	+= mt_pwm.o
endif

obj-y += $(subst ",,$(CONFIG_MTK_PLATFORM))/

