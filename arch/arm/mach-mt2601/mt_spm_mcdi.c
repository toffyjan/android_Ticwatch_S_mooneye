#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/proc_fs.h>
#include <linux/platform_device.h>
#include <linux/earlysuspend.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/err.h>
#include <linux/delay.h>

#include <mach/irqs.h>
#include <mach/mt_spm.h>
#include <mach/mt_dormant.h>
#include <mach/mt_gpt.h>
#include <mach/mt_spm_pcm.h>
#include <mach/mt_irq.h>
#include <mach/mt_spm_api.h>
#include <mach/mt_clkmgr.h>


/* #include <mach/env.h> // require from hibboot flag */
/* #include <asm/hardware/gic.h> */
#include <linux/irqchip/arm-gic.h>


/* =================================== */
#if defined(CONFIG_MT6572_FPGA_CA7)
#define SPM_MCDI_BYPASS_SYSPWREQ 0	/* for FPGA attach jtag */
#else
#define SPM_MCDI_BYPASS_SYSPWREQ 1
#endif
#ifdef SPM_MCDI_FUNC
#define MCDI_KICK_PCM 1
#else
#define MCDI_KICK_PCM 0
#endif

#define SPM_MCDI_LDVT_EN

#define clc_debug spm_debug
#define clc_notice spm_notice
/* =================================== */


DEFINE_SPINLOCK(spm_sodi_lock);

s32 spm_sodi_disable_counter = 0;
u32 MCDI_Test_Mode = 0;


#define WFI_OP        4
#define WFI_L2C      5
#define WFI_SCU      6
#define WFI_MM      16
#define WFI_MD      19

#define PCM_MCDI_LEN              (441)
#define MCDI_pcm_pc_0      0
#define MCDI_pcm_pc_1      13
#define MCDI_pcm_pc_2      MCDI_pcm_pc_0
#define MCDI_pcm_pc_3      MCDI_pcm_pc_1

#define PCM_MCDI_VEC0        EVENT_VEC(WAKE_ID_26M_WAKE, 1, 0, MCDI_pcm_pc_0)	/* MD-wake event */
#define PCM_MCDI_VEC1        EVENT_VEC(WAKE_ID_26M_SLP, 1, 0, MCDI_pcm_pc_1)	/* MD-sleep event */
#define PCM_MCDI_VEC2        EVENT_VEC(WAKE_ID_AP_WAKE, 1, 0, MCDI_pcm_pc_2)
#define PCM_MCDI_VEC3        EVENT_VEC(WAKE_ID_AP_SLEEP, 1, 0, MCDI_pcm_pc_3)


/* extern int mt_irq_mask_all(struct mtk_irq_mask *mask); */
/* extern int mt_irq_mask_restore(struct mtk_irq_mask *mask); */
/* extern int mt_SPI_mask_all(struct mtk_irq_mask *mask); */
/* extern int mt_SPI_mask_restore(struct mtk_irq_mask *mask); */
extern void mt_irq_mask_for_sleep(unsigned int irq);
extern void mt_irq_unmask_for_sleep(unsigned int irq);
#if defined(SPM_MCDI_LDVT_EN)
extern void spm_mcdi_LDVT_mcdi(void);
extern void spm_mcdi_LDVT_sodi(void);
#endif
extern char *get_env(char *name);

/*
extern void //mt_cirq_enable(void);
extern void //mt_cirq_disable(void);
extern void //mt_cirq_clone_gic(void);
extern void //mt_cirq_flush(void);
extern void //mt_cirq_mask(unsigned int cirq_num);
*/
extern spinlock_t spm_lock;
extern u32 En_SPM_MCDI;

/* static struct mtk_irq_mask MCDI_cpu_irq_mask; */

/* TODO: need check */
#if SPM_MCDI_BYPASS_SYSPWREQ
#define WAKE_SRC_FOR_MCDI                     \
	(WAKE_SRC_PCM_TIMER | WAKE_SRC_GPT | WAKE_SRC_THERM | WAKE_SRC_CIRQ | WAKE_SRC_CPU0_IRQ | WAKE_SRC_CPU1_IRQ | WAKE_SRC_SYSPWREQ)
#else
#define WAKE_SRC_FOR_MCDI                     \
	(WAKE_SRC_PCM_TIMER | WAKE_SRC_GPT | WAKE_SRC_THERM | WAKE_SRC_CIRQ | WAKE_SRC_CPU0_IRQ | WAKE_SRC_CPU1_IRQ)
#endif


SPM_PCM_CONFIG pcm_config_mcdi = {
	.scenario = SPM_PCM_MCDI,
	.spm_turn_off_26m = false,
	.pcm_firmware_len = PCM_MCDI_LEN,
	.pcm_pwrlevel = PWR_LVNA,
	.spm_request_uart_sleep = false,
	.sodi_en = false,
	.pcm_vsr = {PCM_MCDI_VEC0, PCM_MCDI_VEC1, PCM_MCDI_VEC2, PCM_MCDI_VEC3, 0, 0, 0, 0},

	/* spm_write(SPM_AP_STANBY_CON, ((0x0<<WFI_OP) | (0x1<<WFI_L2C) | (0x1<<WFI_SCU)));  // operand or, mask l2c, mask scu */

	/*Wake up event mask */
	.md_mask = MDCONN_MASK,	/* mask MD1 and MD2 */
	.mm_mask = MMALL_MASK,	/* mask DISP and MFG */

	/*AP Sleep event mask */
	.wfi_scu_mask = true,	/* check SCU idle */
	.wfi_l2c_mask = true,	/* check L2C idle */
	.wfi_op = REDUCE_OR,
	.wfi_sel = {true, true},

	.timer_val_ms = 0 * 1000,
	.wake_src = WAKE_SRC_FOR_MCDI,
	.infra_pdn = false,	/* keep INFRA/DDRPHY power */
	.cpu_pdn = false,
	/* debug related */
	.reserved = 0x0,
	.dbg_wfi_cnt = 0,
	.wakesta_idx = 0
};

/* extern u32 cpu_pdn_cnt; */
void spm_mcdi_wfi(void)
{
	volatile u32 core_id;
	/* u32 clc_counter; */
	/* unsigned long flags; */
	/* u32 temp_address; */

	core_id = (u32) smp_processor_id();


	if (core_id == 0) {

		if (MCDI_Test_Mode == 1) {
			/* clc_notice("SPM_FC1_PWR_CON %x, cpu_pdn_cnt %d.\n",spm_read(SPM_FC1_PWR_CON),cpu_pdn_cnt); */
			clc_notice("core_%d set wfi_sel.\n", core_id);
		}

		spm_wfi_sel(pcm_config_mcdi.wfi_sel, SPM_CORE1_WFI_SEL_SW_MASK);

		spm_mcdi_poll_mask(core_id, pcm_config_mcdi.wfi_sel);
		if (MCDI_Test_Mode == 1) {
			clc_notice("core_%d mask polling done.\n", core_id);
		}
		wfi_with_sync();	/* enter wfi */

		/* spm_get_wakeup_status(&pcm_config_mcdi); */

		if (MCDI_Test_Mode == 1)
			clc_notice("core_%d exit wfi.\n", core_id);

		/* if(MCDI_Test_Mode == 1) */
		/* mdelay(10);  // delay 10 ms */


	} else {		/* Core 1 Keep original IRQ */

		if (MCDI_Test_Mode == 1) {
			clc_notice("core_%d set wfi_sel.\n", core_id);
		}
		spm_wfi_sel(pcm_config_mcdi.wfi_sel, SPM_CORE0_WFI_SEL_SW_MASK);

		/* //clc_notice("core_%d enter wfi.\n", core_id); */
		spm_mcdi_poll_mask(core_id, pcm_config_mcdi.wfi_sel);
		if (MCDI_Test_Mode == 1) {
			clc_notice("core_%d mask polling done.\n", core_id);
		}
		if (!cpu_power_down(DORMANT_MODE)) {
			switch_to_amp();

			/* do not add code here */
			wfi_with_sync();
		}
		switch_to_smp();
		spm_get_wakeup_status(&pcm_config_mcdi);
		cpu_check_dormant_abort();
		if (MCDI_Test_Mode == 1)
			clc_notice("core_%d exit wfi.\n", core_id);
#if 0
		if (MCDI_Test_Mode == 1) {
			/* read/clear XGPT status: 72 need to confirm */
			if (core_id == 1) {
				gpt_check_and_ack_irq(GPT4);
#if 0
				if (((spm_read(0xf0008004) >> 0) & 0x1) == 0x1) {
					spm_write(0xf0008008, (0x1 << 0));
				}
#endif
			}
			/* mdelay(10);  // delay 10 ms */
		}
#endif
	}

}



/* ============================================================================== */

void spm_disable_sodi(void)
{
	spin_lock(&spm_sodi_lock);

	spm_sodi_disable_counter++;
	clc_debug("spm_disable_sodi() : spm_sodi_disable_counter = 0x%x\n",
		  spm_sodi_disable_counter);

	if (spm_sodi_disable_counter > 0) {
		spm_direct_disable_sodi();
	}

	spin_unlock(&spm_sodi_lock);
}

void spm_enable_sodi(void)
{
	spin_lock(&spm_sodi_lock);

	spm_sodi_disable_counter--;
	clc_debug("spm_enable_sodi() : spm_sodi_disable_counter = 0x%x\n",
		  spm_sodi_disable_counter);

	if (spm_sodi_disable_counter <= 0) {
		spm_direct_enable_sodi();
	}

	spin_unlock(&spm_sodi_lock);
}

void spm_disable_sodi_user(void)
{
	if (pcm_config_mcdi.sodi_en == false)
		return;
	else
		spm_disable_sodi();
	pcm_config_mcdi.sodi_en = false;
}

void spm_enable_sodi_user(void)
{
	if (pcm_config_mcdi.sodi_en == true)
		return;
	else
		spm_enable_sodi();
	pcm_config_mcdi.sodi_en = true;
}

bool spm_is_sodi_user_en(void)
{
	return pcm_config_mcdi.sodi_en;
}

#if 0
static int spm_mcdi_probe(struct platform_device *pdev)
{
	int hibboot = 0;
	hibboot = get_env("hibboot") == NULL ? 0 : simple_strtol(get_env("hibboot"), NULL, 10);

	/* set SPM_MP_CORE0_AUX */
	spm_mcdi_init_core_mux();


#if MCDI_KICK_PCM

	clc_notice("spm_mcdi_probe start.\n");
	if (1 == hibboot) {
		clc_notice("[%s] skip spm_go_to_MCDI due to hib boot\n", __func__);
	} else {
		spm_go_to_MCDI();
		if (pcm_config_mcdi.sodi_en == true)
			spm_direct_enable_sodi();
		else
			spm_disable_sodi();
	}

#endif

	return 0;

}

static void spm_mcdi_early_suspend(struct early_suspend *h)
{
#if MCDI_KICK_PCM
	clc_notice("spm_mcdi_early_suspend start.\n");
	spm_leave_MCDI();
#endif

}

static void spm_mcdi_late_resume(struct early_suspend *h)
{
#if MCDI_KICK_PCM
	clc_notice("spm_mcdi_late_resume start.\n");
	spm_go_to_MCDI();
#endif
}

static struct platform_driver mtk_spm_mcdi_driver = {
	.remove = NULL,
	.shutdown = NULL,
	.probe = spm_mcdi_probe,
	.suspend = NULL,
	.resume = NULL,
	.driver = {
		   .name = "mtk-spm-mcdi",
		   },
};

static struct early_suspend mtk_spm_mcdi_early_suspend_driver = {
	.level = EARLY_SUSPEND_LEVEL_DISABLE_FB + 251,
	.suspend = spm_mcdi_early_suspend,
	.resume = spm_mcdi_late_resume,
};

/***************************
* show current SPM-MCDI stauts
****************************/
static int spm_mcdi_debug_read(char *buf, char **start, off_t off, int count, int *eof, void *data)
{
	int len = 0;
	char *p = buf;

	if (En_SPM_MCDI)
		p += sprintf(p, "SPM MCDI+Thermal Protect enabled.\n");
	else
		p += sprintf(p, "SPM MCDI disabled, Thermal Protect only.\n");

	len = p - buf;
	return len;
}

/************************************
* set SPM-MCDI stauts by sysfs interface
*************************************/
static ssize_t spm_mcdi_debug_write(struct file *file, const char *buffer, unsigned long count,
				    void *data)
{
	int enabled = 0;

	if (sscanf(buffer, "%d", &enabled) == 1) {
		if (enabled == 0) {
			spm_leave_MCDI();
		} else if (enabled == 1) {
			spm_go_to_MCDI();
		} else if (enabled == 2) {
			clc_notice("spm_mcdi_LDVT_sodi() (argument_0 = %d)\n", enabled);
			spm_mcdi_LDVT_sodi();
		} else if (enabled == 3) {
			clc_notice("spm_mcdi_LDVT_mcdi() (argument_0 = %d)\n", enabled);
			spm_mcdi_LDVT_mcdi();
		} else if (enabled == 4) {
			En_SPM_MCDI = 1;
		} else {
			clc_notice("bad argument_0!! (argument_0 = %d)\n", enabled);
		}
	} else {
		clc_notice("bad argument_1!!\n");
	}

	return count;

}

/************************************
* set SPM-SODI Enable by sysfs interface
*************************************/
static ssize_t spm_user_sodi_en(struct file *file, const char *buffer, unsigned long count,
				void *data)
{
	int enabled = 0;

	if (sscanf(buffer, "%d", &enabled) == 1) {
		if (enabled == 0) {
			spm_disable_sodi();
		} else if (enabled == 1) {
			spm_enable_sodi();
		}
	} else {
		clc_notice("bad argument_1!!\n");
	}

	return count;
}

#endif
static void __exit spm_mcdi_exit(void)
{
	clc_notice("Exit SPM-MCDI\n\r");
}


u32 En_SPM_MCDI = 0;
void spm_check_core_status_before(u32 target_core)
{
	u32 target_core_temp, hotplug_out_core_id;
	volatile u32 core_id;

	if (En_SPM_MCDI != 1) {
		return;
	}

	core_id = (u32) smp_processor_id();

	target_core_temp = target_core & 0xf;

	hotplug_out_core_id =
	    ((spm_read(SPM_MP_CORE0_AUX) & 0x1) << 0) | ((spm_read(SPM_MP_CORE1_AUX) & 0x1) << 1);

	target_core_temp &= (~hotplug_out_core_id);

	/* clc_notice("issue IPI, spm_check_core_status_before = 0x%x\n", target_core_temp); */

	if (target_core_temp == 0x0) {
		return;
	}

	/* set IPI SPM register ================================================== */

	switch (core_id) {
	case 0:
		spm_write(SPM_MP_CORE0_AUX, (spm_read(SPM_MP_CORE0_AUX) | (target_core_temp << 1)));
		break;
	case 1:
		spm_write(SPM_MP_CORE1_AUX, (spm_read(SPM_MP_CORE1_AUX) | (target_core_temp << 1)));
		break;

	default:
		break;
	}

}


void spm_check_core_status_after(u32 target_core)
{

	u32 target_core_temp, clc_counter, spm_core_pws, hotplug_out_core_id;
	volatile u32 core_id;

	if (En_SPM_MCDI != 1) {
		return;
	}

	core_id = (u32) smp_processor_id();

	target_core_temp = target_core & 0xf;

	hotplug_out_core_id =
	    ((spm_read(SPM_MP_CORE0_AUX) & 0x1) << 0) | ((spm_read(SPM_MP_CORE1_AUX) & 0x1) << 1);

	target_core_temp &= (~hotplug_out_core_id);

	/* clc_notice("issue IPI, spm_check_core_status_after = 0x%x\n", target_core_temp); */

	if (target_core_temp == 0x0) {
		return;
	}

	/* check CPU wake up ============================================== */

	clc_counter = 0;

	while (1) {
		/* power_state => 1: power down */
		spm_core_pws = ((spm_read(SPM_FC0_PWR_CON) == 0x4d) ? 0 : 1) | ((spm_read(SPM_FC1_PWR_CON) == 0x4d) ? 0 : 2);	/* power_state => 1: power down */

		if ((target_core_temp & ((~spm_core_pws) & 0xf)) == target_core_temp) {
			break;
		}

		clc_counter++;

		if (clc_counter >= 100) {
			spm_notice
			    ("spm_check_core_status_after : check CPU wake up failed.(0x%x, 0x%x)\n",
			     target_core_temp, ((~spm_core_pws) & 0xf));
			break;
		}
	}

	/* clear IPI SPM register ================================================== */

	switch (core_id) {
	case 0:
		spm_write(SPM_MP_CORE0_AUX,
			  (spm_read(SPM_MP_CORE0_AUX) & (~(target_core_temp << 1))));
		break;
	case 1:
		spm_write(SPM_MP_CORE1_AUX,
			  (spm_read(SPM_MP_CORE1_AUX) & (~(target_core_temp << 1))));
		break;
	default:
		break;
	}

}


void spm_hot_plug_in_before(u32 target_core)
{

	spm_notice("spm_hot_plug_in_before()........ target_core = 0x%x\n", target_core);

	switch (target_core) {
	case 0:
		spm_write(SPM_MP_CORE0_AUX, (spm_read(SPM_MP_CORE0_AUX) & (~0x1U)));
		break;
	case 1:
		spm_write(SPM_MP_CORE1_AUX, (spm_read(SPM_MP_CORE1_AUX) & (~0x1U)));
		break;
	default:
		break;
	}

}

void spm_hot_plug_out_after(u32 target_core)
{

	spm_notice("spm_hot_plug_out_after()........ target_core = 0x%x\n", target_core);

	switch (target_core) {

	case 0:
		spm_write(SPM_MP_CORE0_AUX, (spm_read(SPM_MP_CORE0_AUX) | 0x1));
		break;
	case 1:
		spm_write(SPM_MP_CORE1_AUX, (spm_read(SPM_MP_CORE1_AUX) | 0x1));
		break;
	default:
		break;
	}

}

void spm_direct_disable_sodi(void)
{
	u32 clc_temp;

	clc_temp = spm_read(SPM_CLK_CON);
	clc_temp |= (0x1 << 13);

	spm_write(SPM_CLK_CON, clc_temp);
}

void spm_direct_enable_sodi(void)
{
	u32 clc_temp;

	clc_temp = spm_read(SPM_CLK_CON);
	clc_temp &= 0xffffdfff;	/* ~(0x1<<13); */

	spm_write(SPM_CLK_CON, clc_temp);
}

void spm_mcdi_init_core_mux(void)
{
	/* set SPM_MP_CORE0_AUX */
	spm_write(SPM_MP_CORE0_AUX, 0x0);
	spm_write(SPM_MP_CORE1_AUX, 0x0);

}

void spm_mcdi_clean(void)
{
	u32 spm_counter;
	u32 spm_core_pws, hotplug_out_core_id;

	/* trigger cpu wake up event */
	spm_write(SPM_SLEEP_CPU_WAKEUP_EVENT, 0x1);

	/* polling SPM_SLEEP_ISR_STATUS =========================== */
	spm_counter = 0;

	while (((spm_read(SPM_SLEEP_ISR_STATUS) >> 3) & 0x1) == 0x0) {
		if (spm_counter >= 10000) {
			/* set cpu wake up event = 0 */
			spm_write(SPM_SLEEP_CPU_WAKEUP_EVENT, 0x0);
			return;
		}
		spm_counter++;
	}

	/* set cpu wake up event = 0 */
	spm_write(SPM_SLEEP_CPU_WAKEUP_EVENT, 0x0);

	/* clean SPM_SLEEP_ISR_STATUS ============================ */
	spm_write(SPM_SLEEP_ISR_MASK, 0x0008);
	spm_write(SPM_SLEEP_ISR_STATUS, 0x0018);

	/* disable IO output for regiser 0 and 7 */
	spm_write(SPM_PCM_PWR_IO_EN, 0x0);

	/* print spm debug log =================================== */
	spm_pcm_dump_regs();

	/* clean wakeup event raw status */
	spm_write(SPM_SLEEP_WAKEUP_EVENT_MASK, 0xffffffff);

	/* check dram controller setting ============================== */
/* spm_check_dramc_for_pcm();//72 SPM don't modify DRAM setting, so must remive it, remove it latter */

	/* check cpu power ====================================== */
	spm_core_pws = ((spm_read(SPM_FC0_PWR_CON) == 0x4d) ? 0 : 1) | ((spm_read(SPM_FC1_PWR_CON) == 0x4d) ? 0 : 2);	/* power_state => 1: power down */

	hotplug_out_core_id = ((spm_read(SPM_MP_CORE0_AUX) & 0x1) << 0) | ((spm_read(SPM_MP_CORE1_AUX) & 0x1) << 1);	/* 1: hotplug out */


	if (spm_core_pws != hotplug_out_core_id) {
		spm_notice("spm_leave_MCDI : failed_1.(0x%x, 0x%x)\r\n", spm_core_pws,
			   hotplug_out_core_id);
	}
}

void spm_mcdi_poll_mask(u32 core_id, bool core_wfi_sel[])
{
	while ((spm_read(SPM_SLEEP_CPU_IRQ_MASK) & (0x1 << core_id)) == 0);
}

/* MODULE_DESCRIPTION("MT6589 SPM-Idle Driver v0.1"); */
