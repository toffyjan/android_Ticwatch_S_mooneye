include $(srctree)/drivers/misc/mediatek/Makefile.custom
subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/include
subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/pmic/include

obj-$(CONFIG_MTK_SMART_BATTERY) += battery_meter.o battery_common.o battery_ssb.o linear_charging.o switch_charging.o

