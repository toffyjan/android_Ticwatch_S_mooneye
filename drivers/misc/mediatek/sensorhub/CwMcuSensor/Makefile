#
# Makefile for the input misc drivers.
#

# Each configuration option enables a list of files.
include $(srctree)/drivers/misc/mediatek/Makefile.custom
ccflags-y += -I$(MTK_PROJECT_PATH_ROOT)/sensorhub/cwmcusensor
ccflags-y += -I$(srctree)/drivers/spi/mediatek/$(MTK_PLATFORM)

obj-y += CwMcuSensor.o CwMcuSensor_factory.o CwMcuBus.o
