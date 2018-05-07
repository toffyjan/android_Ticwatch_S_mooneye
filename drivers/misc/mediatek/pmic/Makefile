
obj-$(CONFIG_MTK_EXTBUCK) += extbuck/
obj-$(CONFIG_MTK_PMIC_COMMON) += common/

obj-$(CONFIG_MTK_AUXADC_INTF) += common/upmu_auxadc.o

ifneq (,$(findstring evb,$(CONFIG_ARCH_MTK_PROJECT)))
	subdir-ccflags-y += -DMTK_EVB_PLATFORM
endif

ccflags-y += -I$(srctree)/drivers/power/mediatek/battery
obj-$(CONFIG_MTK_SMART_BATTERY) += mtk_battery_adc_intf.o mtk_gauge_class.o mtk_gauge_coulomb_service.o
