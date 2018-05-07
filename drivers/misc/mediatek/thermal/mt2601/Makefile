include $(srctree)/drivers/misc/mediatek/Makefile.custom
ccflags-y  += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
obj-  := dummy.o

ifeq ($(CONFIG_THERMAL), y)
else
    obj-y += mtk_thermal_dummy.o
endif

obj-$(CONFIG_THERMAL) += mtk_thermal_platform.o
ifneq ($(CONFIG_MTK_COMBO_WIFI),)
    ifeq ($(CONFIG_MTK_COMBO_CHIP),"MT6630")
        obj-$(CONFIG_THERMAL) += mtk_ts_wmt.o
    endif
endif

# Only included in non-EVB projects
ifeq ($(CONFIG_POWER_EXT), y)
else
    ifeq ($(CONFIG_MTK_SMART_BATTERY), y)
        ifeq ($(CONFIG_MTK_NCP1851_SUPPORT),y)
            obj-$(CONFIG_THERMAL) += mtk_cooler_bcct.o
        else
            ifeq ($(CONFIG_MTK_BQ24196_SUPPORT),y) 
                obj-$(CONFIG_THERMAL) += mtk_cooler_bcct.o
            else
                # FAN5405 or PMIC6320 pulse charging or new charger IC
                obj-$(CONFIG_THERMAL) += mtk_cooler_bcct.o
            endif
        endif
    endif
endif



ifeq ($(CONFIG_THERMAL),y)
    obj-y += mtk_ts_cpu.o
    obj-y += mtk_ts_pmic.o
    obj-y += mtk_ts_battery.o
endif
                                          
