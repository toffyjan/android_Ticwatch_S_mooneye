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

ifneq ($(CONFIG_ARCH_MT2601),y)
# to fix error in mt2601
obj-$(CONFIG_MTK_CACHE_DUMP) += icache_dump.o
endif
obj-$(CONFIG_MTK_MEMCFG) += mtk_memcfg.o
obj-$(CONFIG_MTK_MEMCFG) += mtk_memcfg_reserve_info.o
ifneq ($(CONFIG_ARM64), y)
# For arm32
obj-y += mtk_common_hooks.o
else
############################ For arm64 #############################
obj-y += mtk_common_hooks_64.o
endif

obj-y += mtk_meminfo.o
MTK_PLATFORM_DIR=$(srctree)/drivers/misc/mediatek/mem/$(subst ",,$(CONFIG_MTK_PLATFORM))
mtkplatform_exist=$(shell if [ -d ${MTK_PLATFORM_DIR} ]; then echo "y"; else echo "n"; fi;)
ifeq ($(mtkplatform_exist), y)
obj-y += $(subst ",,$(CONFIG_MTK_PLATFORM))/
else
obj-y += platform-default/
endif
