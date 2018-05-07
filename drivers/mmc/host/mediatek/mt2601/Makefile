#
# Copyright (C) 2017 MediaTek Inc.
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

#add kernel source code path as head file search path
MTK_PLATFORM := $(subst ",,$(CONFIG_MTK_PLATFORM))
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat
ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/$(MTK_PLATFORM)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/pmic/
#ccflags-y += -I$(srctree)/drivers/misc/mediatek/pmic/include/mt6335
#workaround for include mmc/card/queue.h and mmc/core/core.h
ccflags-y += -I$(srctree)/drivers
ccflags-y += -I$(srctree)/drivers/mmc/host/mediatek/mt2601

obj-$(CONFIG_MMC_MTK_PRO) += msdc.o

include $(srctree)/drivers/misc/mediatek/Makefile.custom

ccflags-y += -I$(srctree)/drivers

obj-$(CONFIG_MTK_MMC)		+= msdc.o

msdc-y += dbg.o
msdc-y += sd.o mt_sd_misc.o

# kdump/kpainc interface move to lk
ifeq ($(CONFIG_MTK_AEE_IPANIC),y)
msdc-y += mt_dump.o
endif

