#
# Makefile for the linux kernel.
#

include $(srctree)/drivers/misc/mediatek/Makefile.custom


ccflags-y += -I$(srctree)/
#ccflags-y += -Werror

obj-y := core.o irq.o timer.o mt_dbg.o eint.o eint_drv.o
obj-y += mt_devs.o mt_gpt.o board.o mt_pm_init.o
obj-y += ca7_timer.o
obj-y += mt_reg_dump.o
obj-y += mt_ahb_abt.o
obj-$(CONFIG_SMP) += mt-smp.o hotplug.o mt-headsmp.o mt_hotplug.o mt_hotplug_mechanism.o
obj-y += mt_freqhopping.o
obj-$(CONFIG_MT65XX_TRACER) += pmu_v7.o mon_interface.o mt_mon.o mt_emi_bm.o
obj-y += emi_bwl.o emi_mpu.o
obj-y += mt_pm_ldo.o
obj-y += innercache.o mt_cache_dump.o

obj-y += BOOT.o
BOOT-y:= mt_boot.o

# MT2601 GDMA is controlled by CONNSYS
#obj-y += mt_dma.o

obj-y += mt_spm.o mt_spm_pcm.o mt_spm_suspend.o mt_spm_dpidle.o mt_spm_mcdi.o mt_spm_mtcmos.o mt_spm_wdt.o spm_fs.o
obj-y += mt_sleep.o
obj-y += mt_cpufreq.o
obj-y += mt_gpufreq.o
obj-y += mt_cpu_ss.o
obj-y += mt_dormant.o cpu_dormant.o
obj-y += mt_dcm.o mt_idle.o
obj-y += mt_clkmgr.o mt_clkmgr_common.o
obj-y += mt_ptp.o
obj-y += sbchk_base.o
obj-$(CONFIG_MTK_CIRQ) += mt_cirq.o
obj-$(CONFIG_MTK_MCI) += mt_mci.o
obj-y += mtk_ccci_helper.o
obj-y += cache-mtk-v7.o
obj-$(CONFIG_FIQ_GLUE) += fiq_smp_call.o

# Hibernation
AFLAGS_swsusp.o	:= -DTEXT_OFFSET=$(TEXT_OFFSET)
#obj-$(CONFIG_HIBERNATION)  += cpu_hibernate.o swsusp.o

ifneq ($(MTK_TABLET_HARDWARE), )
ccflags-y += -DMTK_TABLET_PLATFORM=\"$(shell echo $(MTK_TABLET_HARDWARE) | tr a-z A-Z )\"
endif

KBUILD_AFLAGS	:=$(KBUILD_AFLAGS:-msoft-float=-Wa,-mfpu=neon)
#KT: modify for slt vfp test
#KBUILD_AFLAGS	:=$(KBUILD_AFLAGS:-msoft-float=-Wa,-mfpu=neon-vfpv4)

# Module project related part
obj-y += $(ARCH_MTK_PROJECT)/

