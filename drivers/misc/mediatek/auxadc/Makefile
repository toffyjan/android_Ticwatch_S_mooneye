MTK_PLATFORM:= $(subst ",,$(CONFIG_MTK_PLATFORM))

ccflags-y += -I$(srctree)/drivers/misc/mediatek/auxadc/$(MTK_PLATFORM)

ifneq ($(CONFIG_ARCH_MT2601),y)
obj-$(CONFIG_MTK_AUXADC) += mtk_auxadc.o
else
obj-$(CONFIG_MTK_AUXADC) += mt_auxadc.o
obj-y += $(subst ",,$(CONFIG_MTK_PLATFORM))/
endif
