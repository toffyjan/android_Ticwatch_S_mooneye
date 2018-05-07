/*
* Copyright (C) 2011-2014 MediaTek Inc.
*
* This program is free software: you can redistribute it and/or modify it under the terms of the
* GNU General Public License version 2 as published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
* without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
* See the GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with this program.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include <linux/mali/mali_utgard.h>
#include "mali_kernel_common.h"
#include "mali_osk.h"
#include "platform_pmm.h"
#include "mach/mt_gpufreq.h"
#include <asm/atomic.h>
#include "arm_core_scaling.h"

#if defined(CONFIG_MALI400_PROFILING)
#include "mali_osk_profiling.h"
#endif

extern unsigned long (*mtk_thermal_get_gpu_loading_fp)(void);
extern unsigned int (*mtk_get_gpu_loading_fp)(void);

static int bPoweroff;

/* / #define __POWER_CLK_CTRL_SYNC__ */
/* / For MFG sub-system clock control API */
#include <mach/mt_clkmgr.h>
#include <mach/mt_spm.h>
#include <linux/kernel.h>

#define reg_read(addr) (*(volatile unsigned int*)(addr))

static unsigned int current_sample_utilization;

#if defined(__MALI_CORE_SCALING_ENABLE__)
   extern int mali_core_scaling_enable;
#endif

static DEFINE_SPINLOCK(mali_pwr_lock);

#define mfg_pwr_lock(flags) \
do { \
    spin_lock_irqsave(&mali_pwr_lock, flags); \
} while (0)

#define mfg_pwr_unlock(flags) \
do { \
    spin_unlock_irqrestore(&mali_pwr_lock, flags); \
} while (0)


void mali_pmm_init(void)
{
    MALI_DEBUG_PRINT(1, ("%s\n", __func__));
    /* / mtk_thermal_get_gpu_loading_fp = gpu_get_current_utilization; */
    atomic_set((atomic_t *)&bPoweroff, 1);
    mali_platform_power_mode_change(MALI_POWER_MODE_ON);
}

void mali_pmm_deinit(void)
{
    MALI_DEBUG_PRINT(1, ("%s\n", __func__));

    mali_platform_power_mode_change(MALI_POWER_MODE_DEEP_SLEEP);
}


/* this function will be called periodically with sampling period 200ms~1000ms */
void mali_pmm_utilization_handler(struct mali_gpu_utilization_data *data)
{
	current_sample_utilization = (unsigned int)data->utilization_gpu;

	if (0 == current_sample_utilization || 256 <= current_sample_utilization) {
		MALI_DEBUG_PRINT(1, ("%s: GPU utilization=%d\n", __func__, current_sample_utilization));
	}

#if defined(__MALI_CORE_SCALING_ENABLE__)
	if (1 == mali_core_scaling_enable) {
		mali_core_scaling_update(data);
	}
#endif
}

unsigned long gpu_get_current_utilization(void)
{
    return (current_sample_utilization * 100)/256;
}



void g3d_power_domain_control(int bpower_on)
{
   if (bpower_on)
   {
      MALI_DEBUG_PRINT(2, ("enable_subsys \n"));
      /* enable_subsys(SYS_MFG, "G3D_MFG"); */
   }
   else
   {
      MALI_DEBUG_PRINT(2, ("disable_subsys_force \n"));
      /* disable_subsys(SYS_MFG, "G3D_MFG"); */
   }
}



void mali_platform_power_mode_change(mali_power_mode power_mode)
{
   unsigned long flags;
   switch (power_mode)
   {
      case MALI_POWER_MODE_ON:
	 MALI_DEBUG_PRINT(3, ("Mali platform: Got MALI_POWER_MODE_ON event, %s\n",
			      atomic_read((atomic_t *)&bPoweroff) ? "powering on" : "already on"));

	 if (atomic_read((atomic_t *)&bPoweroff) == 1)
	 {
            MALI_DEBUG_PRINT(3, ("[+]MFG enable_clock \n"));
	    mfg_pwr_lock(flags);
	    if (!clock_is_on(MT_CG_MFG_PDN_BG3D_SW_CG))
	    {
	       enable_clock(MT_CG_MFG_PDN_BG3D_SW_CG, "G3D_DRV");
	       /* / enable WHPLL and set the GPU freq. to 500MHz */
	       if (get_gpu_level() != GPU_LEVEL_0) {
		   clkmux_sel(MT_CLKMUX_MFG_MUX_SEL, MT_CG_GPU_500P5M_EN, "G3D_DRV");
	       }
	    }
	    mfg_pwr_unlock(flags);
            MALI_DEBUG_PRINT(3, ("[-]MFG enable_clock \n"));

#if defined(CONFIG_MALI400_PROFILING)
	    _mali_osk_profiling_add_event(MALI_PROFILING_EVENT_TYPE_SINGLE |
		  MALI_PROFILING_EVENT_CHANNEL_GPU |
		  MALI_PROFILING_EVENT_REASON_SINGLE_GPU_FREQ_VOLT_CHANGE, 500,
		  1200/1000, 0, 0, 0);

#endif
	    atomic_set((atomic_t *)&bPoweroff, 0);
	 }
	 break;
      case MALI_POWER_MODE_LIGHT_SLEEP:
      case MALI_POWER_MODE_DEEP_SLEEP:
	 MALI_DEBUG_PRINT(3, ("Mali platform: Got %s event, %s\n", power_mode ==
		  MALI_POWER_MODE_LIGHT_SLEEP ?  "MALI_POWER_MODE_LIGHT_SLEEP" :
		  "MALI_POWER_MODE_DEEP_SLEEP",  atomic_read((atomic_t *)&bPoweroff) ? "already off" : "powering off"));

	 if (atomic_read((atomic_t *)&bPoweroff) == 0)
	 {
            MALI_DEBUG_PRINT(3, ("[+]MFG disable_clock \n"));
	    mfg_pwr_lock(flags);
	    if (clock_is_on(MT_CG_MFG_PDN_BG3D_SW_CG))
	    {
	       disable_clock(MT_CG_MFG_PDN_BG3D_SW_CG, "G3D_DRV");
	    }
	    mfg_pwr_unlock(flags);
            MALI_DEBUG_PRINT(3, ("[-]MFG disable_clock \n"));

#if defined(CONFIG_MALI400_PROFILING)
	    _mali_osk_profiling_add_event(MALI_PROFILING_EVENT_TYPE_SINGLE |
		  MALI_PROFILING_EVENT_CHANNEL_GPU |
		  MALI_PROFILING_EVENT_REASON_SINGLE_GPU_FREQ_VOLT_CHANGE, 0, 0, 0, 0, 0);
#endif
	    atomic_set((atomic_t *)&bPoweroff, 1);
	 }

	 break;
   }
}

void mali_platform_dump_all_log(char *buf)
{
#define DISP_REG_CONFIG_MMSYS_CG_CON0 (MMSYS_CONFIG_BASE + 0x100)
#define DISP_REG_CONFIG_MMSYS_CG_CON1 (MMSYS_CONFIG_BASE + 0x110)
    volatile unsigned int mfg_mm_clk_sta, pwr_sta, pwr_sta_s;
    unsigned long flags;
    unsigned int mfg_pdn_clk_on, mfg_mm_clk_on, mmsys_cg_con0, mmsys_cg_con1;
    int bPwr;
    const char dmpstr[] = "dump GPU: bPwroff=%d, [mfg_pdn_clk_on=%d, mfg_mm_clk_on=%d, status=0x%08x] [power: status=0x%08x, status_s=0x%08x]\n";
    const char dmpstr1[] = "DISP_REG_CONFIG_MMSYS_CG_CON0=%X, DISP_REG_CONFIG_MMSYS_CG_CON1=%X\n";

    mfg_pwr_lock(flags);

    pwr_sta = reg_read(SPM_PWR_STATUS);
    pwr_sta_s = reg_read(SPM_PWR_STATUS_S);

    mfg_pdn_clk_on = clock_is_on(MT_CG_MFG_PDN_BG3D_SW_CG);
    mfg_mm_clk_on = clock_is_on(MT_CG_MFG_MM_SW_CG);
    mfg_mm_clk_sta = reg_read(CLK_GATING_CTRL0);
    bPwr = atomic_read((atomic_t *)&bPoweroff);
    mmsys_cg_con0 = reg_read(DISP_REG_CONFIG_MMSYS_CG_CON0);
    mmsys_cg_con1 = reg_read(DISP_REG_CONFIG_MMSYS_CG_CON1);

    mfg_pwr_unlock(flags);

    if (buf)
    {
	char tmp[256];

	sprintf(buf, dmpstr, bPwr, mfg_pdn_clk_on, mfg_mm_clk_on, mfg_mm_clk_sta, pwr_sta, pwr_sta_s);
	sprintf(tmp, dmpstr1, mmsys_cg_con0, mmsys_cg_con1);
	strcat(buf, tmp);
    }
    else
    {
	MALI_PRINTF((dmpstr, bPwr, mfg_pdn_clk_on, mfg_mm_clk_on, mfg_mm_clk_sta, pwr_sta, pwr_sta_s));
	MALI_PRINTF((dmpstr1, mmsys_cg_con0, mmsys_cg_con1));
    }
}
