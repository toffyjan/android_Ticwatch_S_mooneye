
include $(srctree)/drivers/misc/mediatek/Makefile.custom

# Module project related part

obj-$(CONFIG_MTK_KXTJ2_1009)   +=  accelerometer/KXTJ2_1009/
obj-$(CONFIG_MTK_MPU3050C)   +=  gyroscope/mpu3050c/
obj-$(CONFIG_MTK_AKM8963)	+=  magnetometer/akm8963/
obj-$(CONFIG_TOUCHSCREEN_ATMEL_MXT144U) += touchpanel/MXT144U/

ifneq ($(CONFIG_ARCH_MTK_PROJECT),)
obj-y	+= core/
obj-y	+= dct/dct/
endif

obj-y	+=  lcm/
obj-y	+=  vibrator/
obj-y	+=  leds/
obj-y	+=  keypad/
obj-y	+=  sound/
obj-y	+=  accdet/
