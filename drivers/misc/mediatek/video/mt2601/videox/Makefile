include $(srctree)/drivers/misc/mediatek/Makefile.custom

ccflags-y += -I$(srctree)/drivers/staging/android/ion
ccflags-y += -I$(srctree)/drivers/staging/android/ion/mtk

obj-$(CONFIG_MTK_FB) += disp_drv.o \
                        mtkfb_console.o \
                        disp_assert_layer.o \
                        mtkfb.o \
                        mtkfb_vsync.o \
                        disp_debug.o \
                        disp_sync.o \
                        disp_mgr.o

#ifeq ($(MTK_DISP_CONFIG_SUPPORT), yes)
obj-$(CONFIG_MTK_FB) += fbconfig_kdebug.o
#endif

ifeq ($(CONFIG_MMPROFILE),y)
ccflags-y += -DDEFAULT_MMP_ENABLE
endif

obj-$(CONFIG_MTK_FB) += lcd_drv.o \
                        dpi_drv.o \
                        dsi_drv.o \
                        disp_drv_dbi.o \
                        disp_drv_dpi.o \
                        disp_drv_dsi.o \
                        disp_hal.o \
                        disp_intr.o

ifeq ($(CONFIG_MTK_GPU_SUPPORT), y)
ccflags-y += -DHWGPU_SUPPORT
endif
