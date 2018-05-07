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

#define ENABLE_LCD_INTERRUPT 1

#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/module.h>
#include <linux/dma-mapping.h>

#include "ddp_reg.h"
#include "ddp_debug.h"
#include "ddp_hal.h"
#include "lcd_reg.h"
#include "lcd_drv.h"

#include "disp_drv_log.h"
#include "disp_drv_platform.h"

#include <linux/hrtimer.h>

#if ENABLE_LCD_INTERRUPT
#include <linux/interrupt.h>
#include <linux/wait.h>
#include "ddp_drv.h"
#include <mach/irqs.h>
#include "mtkfb.h"
static wait_queue_head_t _lcd_wait_queue;
#endif
static wait_queue_head_t _vsync_wait_queue;
static bool lcd_vsync;
static bool wait_lcd_vsync;
static struct hrtimer hrtimer_vsync;
#define VSYNC_US_TO_NS(x) (x * 1000)
unsigned int vsync_timer = 0;
#include "disp_debug.h"
#include <asm/current.h>
#include <asm/pgtable.h>
#include <asm/page.h>
#include <mt-plat/sync_write.h>

#ifdef OUTREG32
#undef OUTREG32
#define OUTREG32(x, y) mt_reg_sync_writel(y, x)
#endif

#ifdef OUTREG16
#undef OUTREG16
#define OUTREG16(x, y) mt_reg_sync_writew(y, x)
#endif

#ifdef OUTREG8
#undef OUTREG8
#define OUTREG8(x, y) mt_reg_sync_writeb(y, x)
#endif

#ifndef OUTREGBIT
#define OUTREGBIT(TYPE, REG, bit, value) \
	do { \
		TYPE r = *((TYPE *)&INREG32(&REG)); \
		r.bit = value; \
		OUTREG32(&REG, AS_UINT32(&r)); \
	} while (0)
#endif

#define LCD_OUTREG32(addr, data) \
	{ OUTREG32(addr, data); }

#define LCD_OUTREG16(addr, data) \
	{ OUTREG16(addr, data); }

#define LCD_OUTREG8(addr, data) \
	{ OUTREG8(addr, data); }

#define LCD_MASKREG32(addr, mask, data) \
	{ MASKREG32(addr, mask, data); }

#define LCD_IODRV_REG_DATA              (UINT32 *)(IO_CFG_LEFT_BASE + 0x080)
#define LCD_IODRV_REG_MSBDATA           (UINT32 *)(IO_CFG_RIGHT_BASE + 0x060)
#define LCD_IODRV_REG_CTRL              (UINT32 *)(IO_CFG_RIGHT_BASE + 0x060)
#define LCD_IODRV_REG_CTRL_MX           (UINT32 *)(IO_CFG_BOTTOM_BASE + 0x060)
#define LCD_IODRV_REG_SPI               (UINT32 *)(IO_CFG_TOP_BASE + 0x0D0)

#define GPIO_REG_GPIO_MODE5             (UINT32 *)(GPIO_BASE + 0x350)
#define GPIO_46_MODE                    ((DISP_REG_GET(GPIO_REG_GPIO_MODE5) >> 24) & 0x7)

void dbi_log_enable(int enable)
{
	dbi_drv_dbg_log = enable;
	dbi_drv_dbg_func_log = enable;
	DBI_DRV_INFO("lcd log %s\n", enable ? "enabled" : "disabled");
}

static PLCD_REGS const LCD_REG = (PLCD_REGS) (DISP_DBI_BASE);
static const UINT32 TO_BPP[LCD_FB_FORMAT_NUM] = { 2, 3, 4 };

unsigned int wait_time = 0;
typedef struct {
	LCD_FB_FORMAT fbFormat;
	UINT32 fbPitchInBytes;
	LCD_REG_SIZE roiWndSize;
	LCD_OUTPUT_MODE outputMode;
	LCD_REGS regBackup;
	void (*pIntCallback) (DISP_INTERRUPT_EVENTS);
} LCD_CONTEXT;

static LCD_CONTEXT _lcdContext;
static int wst_step_LCD = -1;	/* for LCD&FM de-sense */
static bool is_get_default_write_cycle = FALSE;
static unsigned int default_write_cycle;
static UINT32 default_wst;
/* UI layer, default set to 3 */
extern OVL_CONFIG_STRUCT cached_layer_config[DDP_OVL_LAYER_MUN];
extern LCM_PARAMS *lcm_params;
extern LCM_DRIVER *lcm_drv;

/* --------------------------------------------------------------------------- */
/* Local Functions */
/* --------------------------------------------------------------------------- */
LCD_STATUS LCD_Set_DrivingCurrent(LCM_PARAMS *lcm_params)
{
	if (lcm_params->type == LCM_TYPE_DBI) {
		switch (lcm_params->dbi.io_driving_current) {
		case LCM_DRIVING_CURRENT_4MA:
			MASKREG32(LCD_IODRV_REG_DATA, 0x00000C00, 0x00000000);
			break;
		case LCM_DRIVING_CURRENT_8MA:
		case LCM_DRIVING_CURRENT_DEFAULT:
			MASKREG32(LCD_IODRV_REG_DATA, 0x00000C00, 0x00000400);
			break;
		case LCM_DRIVING_CURRENT_12MA:
			MASKREG32(LCD_IODRV_REG_DATA, 0x00000C00, 0x00000800);
			break;
		case LCM_DRIVING_CURRENT_16MA:
			MASKREG32(LCD_IODRV_REG_DATA, 0x00000C00, 0x00000C00);
			break;
		default:
			MASKREG32(LCD_IODRV_REG_DATA, 0x00000C00, 0x00000400);
			DISP_LOG_PRINT("[WARNING] Driving current settings incorrect!\n");
			DISP_LOG_PRINT("dbi.io_driving_current must be 4 mA / 8 mA / 12 mA / 16 mA.\n");
			DISP_LOG_PRINT("Set to default (8 mA).\n");
			break;
		}

		if (GPIO_46_MODE == GPIO_MODE_03) {	/* LPD_MX */
			switch (lcm_params->dbi.ctrl_io_driving_current) {
			case LCM_DRIVING_CURRENT_2MA:
				MASKREG32(LCD_IODRV_REG_CTRL_MX, 0x000000C7, 0x00000000);
				break;
			case LCM_DRIVING_CURRENT_4MA:
			case LCM_DRIVING_CURRENT_DEFAULT:
				MASKREG32(LCD_IODRV_REG_CTRL_MX, 0x000000C7, 0x00000041);
				break;
			case LCM_DRIVING_CURRENT_6MA:
				MASKREG32(LCD_IODRV_REG_CTRL_MX, 0x000000C7, 0x00000082);
				break;
			case LCM_DRIVING_CURRENT_8MA:
				MASKREG32(LCD_IODRV_REG_CTRL_MX, 0x000000C7, 0x000000C3);
				break;
			case LCM_DRIVING_CURRENT_10MA:
				MASKREG32(LCD_IODRV_REG_CTRL_MX, 0x000000C7, 0x000000C4);
				break;
			case LCM_DRIVING_CURRENT_12MA:
				MASKREG32(LCD_IODRV_REG_CTRL_MX, 0x000000C7, 0x000000C5);
				break;
			case LCM_DRIVING_CURRENT_14MA:
				MASKREG32(LCD_IODRV_REG_CTRL_MX, 0x000000C7, 0x000000C6);
				break;
			case LCM_DRIVING_CURRENT_16MA:
				MASKREG32(LCD_IODRV_REG_CTRL_MX, 0x000000C7, 0x000000C7);
				break;
			default:
				MASKREG32(LCD_IODRV_REG_CTRL_MX, 0x000000C7, 0x00000041);
				DISP_LOG_PRINT("[WARNING] Driving current settings incorrect!\n");
				DISP_LOG_PRINT("dbi.io_driving_current must be 2 mA / 4 mA / 6 mA / 8 mA / 10 mA / 12 mA / 14 mA / 16 mA.\n");
				DISP_LOG_PRINT("Set to default (4 mA).\n");
				break;
			}
		} else {
			switch (lcm_params->dbi.ctrl_io_driving_current) {
			case LCM_DRIVING_CURRENT_4MA:
				MASKREG32(LCD_IODRV_REG_CTRL, 0x00000007, 0x00000000);
				break;
			case LCM_DRIVING_CURRENT_8MA:
			case LCM_DRIVING_CURRENT_DEFAULT:
				MASKREG32(LCD_IODRV_REG_CTRL, 0x00000007, 0x00000001);
				break;
			case LCM_DRIVING_CURRENT_12MA:
				MASKREG32(LCD_IODRV_REG_CTRL, 0x00000007, 0x00000002);
				break;
			case LCM_DRIVING_CURRENT_16MA:
				MASKREG32(LCD_IODRV_REG_CTRL, 0x00000007, 0x00000003);
				break;
			case LCM_DRIVING_CURRENT_20MA:
				MASKREG32(LCD_IODRV_REG_CTRL, 0x00000007, 0x00000004);
				break;
			case LCM_DRIVING_CURRENT_24MA:
				MASKREG32(LCD_IODRV_REG_CTRL, 0x00000007, 0x00000005);
				break;
			case LCM_DRIVING_CURRENT_28MA:
				MASKREG32(LCD_IODRV_REG_CTRL, 0x00000007, 0x00000006);
				break;
			case LCM_DRIVING_CURRENT_32MA:
				MASKREG32(LCD_IODRV_REG_CTRL, 0x00000007, 0x00000007);
				break;
			default:
				MASKREG32(LCD_IODRV_REG_CTRL, 0x00000007, 0x00000001);
				DISP_LOG_PRINT("[WARNING] Driving current settings incorrect!\n");
				DISP_LOG_PRINT("dbi.io_driving_current must be 4 mA / 8 mA / 12 mA / 16 mA / 20 mA / 24 mA / 28 mA / 32 mA.\n");
				DISP_LOG_PRINT("Set to default (8 mA).\n");
				break;
			}
		}
	}

	if (lcm_params->type == LCM_TYPE_DPI) {
		switch (lcm_params->dpi.io_driving_current) {
		case LCM_DRIVING_CURRENT_4MA:
			MASKREG32(LCD_IODRV_REG_DATA, 0x00000C00, 0x00000000);
			break;
		case LCM_DRIVING_CURRENT_8MA:
		case LCM_DRIVING_CURRENT_DEFAULT:
			MASKREG32(LCD_IODRV_REG_DATA, 0x00000C00, 0x00000400);
			break;
		case LCM_DRIVING_CURRENT_12MA:
			MASKREG32(LCD_IODRV_REG_DATA, 0x00000C00, 0x00000800);
			break;
		case LCM_DRIVING_CURRENT_16MA:
			MASKREG32(LCD_IODRV_REG_DATA, 0x00000C00, 0x00000C00);
			break;
		default:
			MASKREG32(LCD_IODRV_REG_DATA, 0x00000C00, 0x00000400);
			DISP_LOG_PRINT("[WARNING] Driving current settings incorrect!\n");
			DISP_LOG_PRINT("dbi.io_driving_current must be 4 mA / 8 mA / 12 mA / 16 mA.\n");
			DISP_LOG_PRINT("Set to default (8 mA).\n");
			break;
		}

		switch (lcm_params->dpi.ctrl_io_driving_current) {
		case LCM_DRIVING_CURRENT_4MA:
			MASKREG32(LCD_IODRV_REG_CTRL, 0x00000007, 0x00000000);
			break;
		case LCM_DRIVING_CURRENT_8MA:
		case LCM_DRIVING_CURRENT_DEFAULT:
			MASKREG32(LCD_IODRV_REG_CTRL, 0x00000007, 0x00000001);
			break;
		case LCM_DRIVING_CURRENT_12MA:
			MASKREG32(LCD_IODRV_REG_CTRL, 0x00000007, 0x00000002);
			break;
		case LCM_DRIVING_CURRENT_16MA:
			MASKREG32(LCD_IODRV_REG_CTRL, 0x00000007, 0x00000003);
			break;
		case LCM_DRIVING_CURRENT_20MA:
			MASKREG32(LCD_IODRV_REG_CTRL, 0x00000007, 0x00000004);
			break;
		case LCM_DRIVING_CURRENT_24MA:
			MASKREG32(LCD_IODRV_REG_CTRL, 0x00000007, 0x00000005);
			break;
		case LCM_DRIVING_CURRENT_28MA:
			MASKREG32(LCD_IODRV_REG_CTRL, 0x00000007, 0x00000006);
			break;
		case LCM_DRIVING_CURRENT_32MA:
			MASKREG32(LCD_IODRV_REG_CTRL, 0x00000007, 0x00000007);
			break;
		default:
			MASKREG32(LCD_IODRV_REG_CTRL, 0x00000007, 0x00000001);
			DISP_LOG_PRINT("[WARNING] Driving current settings incorrect!\n");
			DISP_LOG_PRINT("dbi.io_driving_current must be 4 mA / 8 mA / 12 mA / 16 mA / 20 mA / 24 mA / 28 mA / 32 mA.\n");
			DISP_LOG_PRINT("Set to default (8 mA).\n");
			break;
		}

		if (lcm_params->dpi.format == LCM_DPI_FORMAT_RGB888) {
			switch (lcm_params->dpi.msb_io_driving_current) {
			case LCM_DRIVING_CURRENT_2MA:
				MASKREG32(LCD_IODRV_REG_MSBDATA, 0x0003C000, 0x00000000);
				break;
			case LCM_DRIVING_CURRENT_4MA:
			case LCM_DRIVING_CURRENT_DEFAULT:
				MASKREG32(LCD_IODRV_REG_MSBDATA, 0x0003C000, 0x00001400);
				break;
			case LCM_DRIVING_CURRENT_6MA:
				MASKREG32(LCD_IODRV_REG_MSBDATA, 0x0003C000, 0x00002800);
				break;
			case LCM_DRIVING_CURRENT_8MA:
				MASKREG32(LCD_IODRV_REG_MSBDATA, 0x0003C000, 0x00003C00);
				break;
			default:
				MASKREG32(LCD_IODRV_REG_MSBDATA, 0x0003C000, 0x00001400);
				DISP_LOG_PRINT("[WARNING] Driving current settings incorrect!\n");
				DISP_LOG_PRINT("dbi.io_driving_current must be 2 mA / 4 mA / 6 mA / 8 mA.\n");
				DISP_LOG_PRINT("Set to default (4 mA).\n");
				break;
			}
		}

		if (lcm_params->ctrl == LCM_CTRL_SERIAL_DBI) {
			switch (lcm_params->dbi.io_driving_current) {
			case LCM_DRIVING_CURRENT_2MA:
				MASKREG32(LCD_IODRV_REG_SPI, 0x0000C0C0, 0x00000000);
				break;
			case LCM_DRIVING_CURRENT_4MA:
			case LCM_DRIVING_CURRENT_DEFAULT:
				MASKREG32(LCD_IODRV_REG_SPI, 0x0000C0C0, 0x00004040);
				break;
			case LCM_DRIVING_CURRENT_6MA:
				MASKREG32(LCD_IODRV_REG_SPI, 0x0000C0C0, 0x00008080);
				break;
			case LCM_DRIVING_CURRENT_8MA:
				MASKREG32(LCD_IODRV_REG_SPI, 0x0000C0C0, 0x0000C0C0);
				break;
			default:
				MASKREG32(LCD_IODRV_REG_SPI, 0x0000C0C0, 0x00004040);
				DISP_LOG_PRINT("[WARNING] Driving current settings incorrect!\n");
				DISP_LOG_PRINT("dbi.io_driving_current must be 2 mA / 4 mA / 6 mA / 8 mA.\n");
				DISP_LOG_PRINT("Set to default (4 mA).\n");
				break;
			}
		}
	}

	return LCD_STATUS_OK;
}

static void _LCD_RDMA0_IRQ_Handler(unsigned int param)
{
	if (_lcdContext.pIntCallback) {
		if (param & 4) {
			/* frame end interrupt */
			MMProfileLogEx(MTKFB_MMP_Events.ScreenUpdate, MMProfileFlagEnd, param, 0);
			_lcdContext.pIntCallback(DISP_LCD_SCREEN_UPDATE_END_INT);
		}
		if (param & 8) {
			/* abnormal EOF interrupt */
			MMProfileLogEx(MTKFB_MMP_Events.ScreenUpdate, MMProfileFlagEnd, param, 0);
		}
		if (param & 2) {
			/* frame start interrupt */
			MMProfileLogEx(MTKFB_MMP_Events.ScreenUpdate, MMProfileFlagStart, param, 0);
			_lcdContext.pIntCallback(DISP_LCD_SCREEN_UPDATE_START_INT);
		}
		if (param & 0x20) {
			/* target line interrupt */
			_lcdContext.pIntCallback(DISP_LCD_TARGET_LINE_INT);
			_lcdContext.pIntCallback(DISP_LCD_VSYNC_INT);
		}
	}
}

static void _LCD_MUTEX_IRQ_Handler(unsigned int param)
{
	if (_lcdContext.pIntCallback) {
		if (param & 1) {
			/* mutex0 register update interrupt */
			_lcdContext.pIntCallback(DISP_LCD_REG_COMPLETE_INT);
		}
	}
}

#if ENABLE_LCD_INTERRUPT
static irqreturn_t _LCD_InterruptHandler(int irq, void *dev_id)
{
	LCD_REG_INTERRUPT status = LCD_REG->INT_STATUS;
	MMProfileLogEx(DDP_MMP_Events.ROT_IRQ, MMProfileFlagPulse, AS_UINT32(&status), 0);

	if (status.COMPLETED) {
		/* write clear COMPLETED interrupt */
		status.COMPLETED = 1;

#ifdef CONFIG_MTPROF_APPLAUNCH	/* eng enable, user disable */
		DISP_LOG_PRINT("[AppLaunch] LCD frame buffer update done !\n");
#endif
		wake_up_interruptible(&_lcd_wait_queue);

		if (_lcdContext.pIntCallback)
			_lcdContext.pIntCallback(DISP_LCD_TRANSFER_COMPLETE_INT);

		DBG_OnLcdDone();
	}

	if (status.CMDQ_COMPLETED) {
		/* The last screen update has finished. */
		if (_lcdContext.pIntCallback)
			_lcdContext.pIntCallback(DISP_LCD_CDMQ_COMPLETE_INT);

		DBG_OnLcdDone();

		status.CMDQ_COMPLETED = 1;

		wake_up_interruptible(&_lcd_wait_queue);

		/* if (_lcdContext.pIntCallback) */
		/*     _lcdContext.pIntCallback(DISP_LCD_CDMQ_COMPLETE_INT); */
	}

	if (status.TE) {	/* this is TE mode 0 interrupt */
		DBG_OnTeDelayDone();

		/* Write clear TE */
		status.TE = 1;

		if (_lcdContext.pIntCallback)
			_lcdContext.pIntCallback(DISP_LCD_SYNC_INT);

		if (wait_lcd_vsync) {	/* judge if wait vsync */
			if (-1 != hrtimer_try_to_cancel(&hrtimer_vsync)) {
				lcd_vsync = true;

				/* hrtimer_try_to_cancel(&hrtimer_vsync); */

				wake_up_interruptible(&_vsync_wait_queue);
			}
			/* DISP_LOG_PRINT("TE signal, and wake up\n"); */
		}

		DBG_OnTeDelayDone();
	}

	if (status.HTT)
		status.HTT = 1;
	if (status.SYNC)
		status.SYNC = 1;
	LCD_OUTREG32(&LCD_REG->INT_STATUS, ~(AS_UINT32(&status)));

	return IRQ_HANDLED;
}
#endif

static BOOL _IsEngineBusy(void)
{
	LCD_REG_STATUS status;

	status = LCD_REG->STATUS;
	if (status.BUSY)
		return TRUE;

	return FALSE;
}

static void _WaitForLCDEngineComplete(void)
{
	do {
		if ((INREG32(&LCD_REG->INT_STATUS) & 0x1) == 0x1)
			break;
	} while (1);
}

LCD_STATUS LCD_WaitForEngineNotBusy(void)
{
	int timeOut;
	static int passCount;
#if ENABLE_LCD_INTERRUPT
	static const long WAIT_TIMEOUT = 2 * HZ;	/* 2 sec */
#endif

	timeOut = 200;

#if ENABLE_LCD_INTERRUPT
	if (in_interrupt()) {
		/* perform busy waiting if in interrupt context */
		while (_IsEngineBusy()) {
			msleep(1);

			if (--timeOut < 0) {
				pr_err("[LCD] Wait for LCD engine not busy timeout!!!\n");
				LCD_DumpRegisters();

				LCD_SetSwReset();

				return LCD_STATUS_ERROR;
			}
		}
	} else {
		int retry = 0;
		long ret;

		do {
			ret = -1;
			if (LCD_REG->STATUS.BUSY) {
				ret = wait_event_interruptible_timeout(_lcd_wait_queue, !_IsEngineBusy(), WAIT_TIMEOUT);

				if ((0 == ret) && _IsEngineBusy()) {
					pr_err("[LCD] Wait for LCD engine not busy timeout!!!\n");
					LCD_DumpRegisters();

					if ((LCD_REG->STATUS.WAIT_SYNC) && (passCount < 3)) {
						int ct;

						pr_err("[LCD] reason is LCD can't wait TE signal!!!\n");
						LCD_TE_Enable(FALSE);
						for (ct = 0; ct < 100; ct++)
							DISP_LOG_PRINT("====== Force disable TE ======\n");
					}

					LCD_SetSwReset();

					LCD_OUTREG32(&LCD_REG->START, 0);
					LCD_OUTREG32(&LCD_REG->START, (1 << 15));

					retry++;
				}
			}
		} while ((ret == 0) && (retry < 3));

		if ((ret == 0) && (retry == 3)) {
			return LCD_STATUS_ERROR;
		} else {
			if (passCount < 3)
				passCount++;
		}
	}
#else
	while (_IsEngineBusy()) {
		udelay(100);

		if (--timeOut < 0) {
			pr_err("[LCD] Wait for LCD engine not busy timeout!!!\n");
			LCD_DumpRegisters();

			if (LCD_REG->STATUS.WAIT_SYNC) {
				DISP_LOG_PRINT("reason is LCD can't wait TE signal!!!\n");
				LCD_TE_Enable(FALSE);
			}

			LCD_SetSwReset();
			OUTREG32(&LCD_REG->INT_STATUS, 0x0);

			return LCD_STATUS_ERROR;
		}
	}
	OUTREG32(&LCD_REG->INT_STATUS, 0x0);
#endif

	return LCD_STATUS_OK;
}

unsigned int vsync_wait_time = 0;
void LCD_WaitTE(void)
{
	wait_lcd_vsync = true;
	hrtimer_start(&hrtimer_vsync, ktime_set(0, VSYNC_US_TO_NS(vsync_timer)), HRTIMER_MODE_REL);
	wait_event_interruptible(_vsync_wait_queue, lcd_vsync);
	lcd_vsync = false;
	wait_lcd_vsync = false;
}

enum hrtimer_restart lcd_te_hrtimer_func(struct hrtimer *timer)
{
	if (wait_lcd_vsync) {
		lcd_vsync = true;
		wake_up_interruptible(&_vsync_wait_queue);
	}

	return HRTIMER_NORESTART;
}

void LCD_InitVSYNC(unsigned int vsync_interval)
{
	ktime_t ktime;

	vsync_timer = vsync_interval;
	ktime = ktime_set(0, VSYNC_US_TO_NS(vsync_timer));
	hrtimer_init(&hrtimer_vsync, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	hrtimer_vsync.function = lcd_te_hrtimer_func;
}

LCD_STATUS LCD_BackupRegisters(void)
{
	LCD_REGS *regs = &(_lcdContext.regBackup);
	UINT32 i;

	LCD_OUTREG32(&regs->INT_ENABLE, AS_UINT32(&LCD_REG->INT_ENABLE));
	LCD_OUTREG32(&regs->SERIAL_CFG, AS_UINT32(&LCD_REG->SERIAL_CFG));

	for (i = 0; i < ARY_SIZE(LCD_REG->SIF_TIMING); ++i)
		LCD_OUTREG32(&regs->SIF_TIMING[i], AS_UINT32(&LCD_REG->SIF_TIMING[i]));

	for (i = 0; i < ARY_SIZE(LCD_REG->PARALLEL_CFG); ++i)
		LCD_OUTREG32(&regs->PARALLEL_CFG[i], AS_UINT32(&LCD_REG->PARALLEL_CFG[i]));

	LCD_OUTREG32(&regs->TEARING_CFG, AS_UINT32(&LCD_REG->TEARING_CFG));
	LCD_OUTREG32(&regs->PARALLEL_DW, AS_UINT32(&LCD_REG->PARALLEL_DW));
	LCD_OUTREG32(&regs->CALC_HTT, AS_UINT32(&LCD_REG->CALC_HTT));
	LCD_OUTREG32(&regs->SYNC_LCM_SIZE, AS_UINT32(&LCD_REG->SYNC_LCM_SIZE));
	LCD_OUTREG32(&regs->SYNC_CNT, AS_UINT32(&LCD_REG->SYNC_CNT));
	LCD_OUTREG32(&regs->SMI_CON, AS_UINT32(&LCD_REG->SMI_CON));

	LCD_OUTREG32(&regs->WROI_CONTROL, AS_UINT32(&LCD_REG->WROI_CONTROL));
	LCD_OUTREG32(&regs->WROI_CMD_ADDR, AS_UINT32(&LCD_REG->WROI_CMD_ADDR));
	LCD_OUTREG32(&regs->WROI_DATA_ADDR, AS_UINT32(&LCD_REG->WROI_DATA_ADDR));
	LCD_OUTREG32(&regs->WROI_SIZE, AS_UINT32(&LCD_REG->WROI_SIZE));

	LCD_OUTREG32(&regs->SRC_CON, AS_UINT32(&LCD_REG->SRC_CON));
	LCD_OUTREG32(&regs->SRC_ADD, AS_UINT32(&LCD_REG->SRC_ADD));
	LCD_OUTREG32(&regs->SRC_PITCH, AS_UINT32(&LCD_REG->SRC_PITCH));

	LCD_OUTREG32(&regs->ULTRA_CON, AS_UINT32(&LCD_REG->ULTRA_CON));
	LCD_OUTREG32(&regs->DBI_ULTRA_TH, AS_UINT32(&LCD_REG->DBI_ULTRA_TH));

	return LCD_STATUS_OK;
}

LCD_STATUS LCD_RestoreRegisters(void)
{
	LCD_REGS *regs = &(_lcdContext.regBackup);
	UINT32 i;

	LCD_OUTREG32(&LCD_REG->INT_ENABLE, AS_UINT32(&regs->INT_ENABLE));
	LCD_OUTREG32(&LCD_REG->SERIAL_CFG, AS_UINT32(&regs->SERIAL_CFG));

	for (i = 0; i < ARY_SIZE(LCD_REG->SIF_TIMING); ++i)
		LCD_OUTREG32(&LCD_REG->SIF_TIMING[i], AS_UINT32(&regs->SIF_TIMING[i]));

	for (i = 0; i < ARY_SIZE(LCD_REG->PARALLEL_CFG); ++i)
		LCD_OUTREG32(&LCD_REG->PARALLEL_CFG[i], AS_UINT32(&regs->PARALLEL_CFG[i]));

	LCD_OUTREG32(&LCD_REG->TEARING_CFG, AS_UINT32(&regs->TEARING_CFG));
	LCD_OUTREG32(&LCD_REG->PARALLEL_DW, AS_UINT32(&regs->PARALLEL_DW));
	LCD_OUTREG32(&LCD_REG->CALC_HTT, AS_UINT32(&regs->CALC_HTT));
	LCD_OUTREG32(&LCD_REG->SYNC_LCM_SIZE, AS_UINT32(&regs->SYNC_LCM_SIZE));
	LCD_OUTREG32(&LCD_REG->SYNC_CNT, AS_UINT32(&regs->SYNC_CNT));
	LCD_OUTREG32(&LCD_REG->SMI_CON, AS_UINT32(&regs->SMI_CON));

	LCD_OUTREG32(&LCD_REG->WROI_CONTROL, AS_UINT32(&regs->WROI_CONTROL));
	LCD_OUTREG32(&LCD_REG->WROI_CMD_ADDR, AS_UINT32(&regs->WROI_CMD_ADDR));
	LCD_OUTREG32(&LCD_REG->WROI_DATA_ADDR, AS_UINT32(&regs->WROI_DATA_ADDR));
	LCD_OUTREG32(&LCD_REG->WROI_SIZE, AS_UINT32(&regs->WROI_SIZE));

	LCD_OUTREG32(&LCD_REG->SRC_CON, AS_UINT32(&regs->SRC_CON));
	LCD_OUTREG32(&LCD_REG->SRC_ADD, AS_UINT32(&regs->SRC_ADD));
	LCD_OUTREG32(&LCD_REG->SRC_PITCH, AS_UINT32(&regs->SRC_PITCH));

	LCD_OUTREG32(&LCD_REG->ULTRA_CON, AS_UINT32(&regs->ULTRA_CON));
	LCD_OUTREG32(&LCD_REG->DBI_ULTRA_TH, AS_UINT32(&regs->DBI_ULTRA_TH));

	return LCD_STATUS_OK;
}

static void _ResetBackupedLCDRegisterValues(void)
{
	LCD_REGS *regs = &_lcdContext.regBackup;

	memset((void *)regs, 0, sizeof(LCD_REGS));
}

/* --------------------------------------------------------------------------- */
/* LCD Controller API Implementations */
/* --------------------------------------------------------------------------- */
LCD_STATUS LCD_Init(BOOL isLcdPoweredOn)
{
	LCD_STATUS ret = LCD_STATUS_OK;

	memset(&_lcdContext, 0, sizeof(_lcdContext));

	/* LCD controller would NOT reset register as default values */
	/* Do it by SW here */
	if (isLcdPoweredOn)
		LCD_BackupRegisters();
	else
		_ResetBackupedLCDRegisterValues();

	ret = LCD_PowerOn();
	ASSERT(ret == LCD_STATUS_OK);

	LCD_OUTREG32(&LCD_REG->SYNC_LCM_SIZE, 0x00010001);
	LCD_OUTREG32(&LCD_REG->SYNC_CNT, 0x1);

#if ENABLE_LCD_INTERRUPT
	init_waitqueue_head(&_lcd_wait_queue);
	init_waitqueue_head(&_vsync_wait_queue);

	if (request_irq(MT_DISP_DBI_IRQ_ID, _LCD_InterruptHandler, IRQF_TRIGGER_LOW, MTKFB_DRIVER, NULL) < 0) {
		DBI_DRV_WRAN("[LCD][ERROR] fail to request LCD irq\n");
		return LCD_STATUS_ERROR;
	}

	OUTREGBIT(LCD_REG_INTERRUPT, LCD_REG->INT_ENABLE, COMPLETED, 1);
	OUTREGBIT(LCD_REG_INTERRUPT, LCD_REG->INT_ENABLE, CMDQ_COMPLETED, 1);
	OUTREGBIT(LCD_REG_INTERRUPT, LCD_REG->INT_ENABLE, HTT, 1);
	OUTREGBIT(LCD_REG_INTERRUPT, LCD_REG->INT_ENABLE, SYNC, 1);
	OUTREGBIT(LCD_REG_INTERRUPT, LCD_REG->INT_ENABLE, TE, 1);

	disp_register_irq(DISP_MODULE_RDMA0, _LCD_RDMA0_IRQ_Handler);
#endif

	return LCD_STATUS_OK;
}

LCD_STATUS LCD_Deinit(void)
{
	LCD_STATUS ret = LCD_PowerOff();

	ASSERT(ret == LCD_STATUS_OK);
	return LCD_STATUS_OK;
}

static BOOL s_isLcdPowerOn = FALSE;

LCD_STATUS LCD_PowerOn(void)
{
	DBI_DRV_FUNC("[%s]:enter\n", __func__);

	if (!s_isLcdPowerOn) {
		int ret = 0;

		DBI_DRV_INFO("lcd will be power on\n");
		if (!clock_is_on(MT_CG_DISP_DBI_IF_SW_CG))
			ret = enable_clock(MT_CG_DISP_DBI_IF_SW_CG, "LCD");
		if (!clock_is_on(MT_CG_DBI_PAD0_SW_CG))
			ret += enable_clock(MT_CG_DBI_PAD0_SW_CG, "LCD");
		if (!clock_is_on(MT_CG_DBI_PAD1_SW_CG))
			ret += enable_clock(MT_CG_DBI_PAD1_SW_CG, "LCD");
		if (!clock_is_on(MT_CG_DBI_PAD2_SW_CG))
			ret += enable_clock(MT_CG_DBI_PAD2_SW_CG, "LCD");
		if (!clock_is_on(MT_CG_DBI_PAD3_SW_CG))
			ret += enable_clock(MT_CG_DBI_PAD3_SW_CG, "LCD");
		if (!clock_is_on(MT_CG_DISP_DBI_ENGINE_SW_CG))
			ret += enable_clock(MT_CG_DISP_DBI_ENGINE_SW_CG, "LCD");
		if (!clock_is_on(MT_CG_DISP_DBI_SMI_SW_CG))
			ret += enable_clock(MT_CG_DISP_DBI_SMI_SW_CG, "LCD");

		if (ret > 0)
			pr_err("[LCD] LCD power manager API return FALSE\n");

		s_isLcdPowerOn = TRUE;
	}

	return LCD_STATUS_OK;
}

LCD_STATUS LCD_PowerOff(void)
{
	DBI_DRV_FUNC("[%s]:enter\n", __func__);

	if (s_isLcdPowerOn) {
		int ret = 0;

		LCD_OUTREG32(&LCD_REG->INT_ENABLE, 0x0);
		LCD_OUTREG32(&LCD_REG->INT_STATUS, 0x0);

		DBI_DRV_INFO("lcd will be power off\n");
		if (clock_is_on(MT_CG_DISP_DBI_SMI_SW_CG))
			ret += disable_clock(MT_CG_DISP_DBI_SMI_SW_CG, "LCD");
		if (clock_is_on(MT_CG_DISP_DBI_ENGINE_SW_CG))
			ret += disable_clock(MT_CG_DISP_DBI_ENGINE_SW_CG, "LCD");
		if (clock_is_on(MT_CG_DBI_PAD3_SW_CG))
			ret = disable_clock(MT_CG_DBI_PAD3_SW_CG, "LCD");
		if (clock_is_on(MT_CG_DBI_PAD2_SW_CG))
			ret += disable_clock(MT_CG_DBI_PAD2_SW_CG, "LCD");
		if (clock_is_on(MT_CG_DBI_PAD1_SW_CG))
			ret += disable_clock(MT_CG_DBI_PAD1_SW_CG, "LCD");
		if (clock_is_on(MT_CG_DBI_PAD0_SW_CG))
			ret += disable_clock(MT_CG_DBI_PAD0_SW_CG, "LCD");
		if (clock_is_on(MT_CG_DISP_DBI_IF_SW_CG))
			ret += disable_clock(MT_CG_DISP_DBI_IF_SW_CG, "LCD");

		if (ret > 0)
			pr_err("[LCD] LCD power manager API return FALSE\n");

		s_isLcdPowerOn = FALSE;
	}

	return LCD_STATUS_OK;
}

LCD_STATUS LCD_WaitForNotBusy(void)
{
	LCD_WaitForEngineNotBusy();
	return LCD_STATUS_OK;
}

LCD_STATUS LCD_EnableInterrupt(DISP_INTERRUPT_EVENTS eventID)
{
#if ENABLE_LCD_INTERRUPT
	switch (eventID) {
	case DISP_LCD_TRANSFER_COMPLETE_INT:
		OUTREGBIT(LCD_REG_INTERRUPT, LCD_REG->INT_ENABLE, COMPLETED, 1);
		break;

	case DISP_LCD_CDMQ_COMPLETE_INT:
		OUTREGBIT(LCD_REG_INTERRUPT, LCD_REG->INT_ENABLE, CMDQ_COMPLETED, 1);
		break;

	case DISP_LCD_HTT_INT:
		OUTREGBIT(LCD_REG_INTERRUPT, LCD_REG->INT_ENABLE, HTT, 1);
		break;

	case DISP_LCD_SYNC_INT:
		OUTREGBIT(LCD_REG_INTERRUPT, LCD_REG->INT_ENABLE, SYNC, 1);
		break;

	case DISP_LCD_TE_INT:
		OUTREGBIT(LCD_REG_INTERRUPT, LCD_REG->INT_ENABLE, TE, 1);
		break;

	case DISP_LCD_VSYNC_INT:
		disp_register_irq(DISP_MODULE_RDMA0, _LCD_RDMA0_IRQ_Handler);
		break;

	case DISP_LCD_SCREEN_UPDATE_START_INT:
		disp_register_irq(DISP_MODULE_RDMA0, _LCD_RDMA0_IRQ_Handler);
		break;

	case DISP_LCD_SCREEN_UPDATE_END_INT:
		disp_register_irq(DISP_MODULE_RDMA0, _LCD_RDMA0_IRQ_Handler);
		break;

	case DISP_LCD_TARGET_LINE_INT:
		disp_register_irq(DISP_MODULE_RDMA0, _LCD_RDMA0_IRQ_Handler);
		break;

	case DISP_LCD_REG_COMPLETE_INT:
		/* wake_up_interruptible(&_dsi_reg_update_wq); */
		disp_register_irq(DISP_MODULE_MUTEX, _LCD_MUTEX_IRQ_Handler);
		break;

	default:
		return LCD_STATUS_ERROR;
	}
#endif

	return LCD_STATUS_OK;
}

LCD_STATUS LCD_SetInterruptCallback(void (*pCB) (DISP_INTERRUPT_EVENTS))
{
	_lcdContext.pIntCallback = pCB;

	return LCD_STATUS_OK;
}

/* -------------------- LCD Controller Interface -------------------- */
LCD_STATUS LCD_ConfigParallelIF(LCD_IF_ID id,
				LCD_IF_PARALLEL_BITS ifDataWidth,
				LCD_IF_PARALLEL_CLK_DIV clkDivisor,
				UINT32 writeSetup,
				UINT32 writeHold,
				UINT32 writeWait,
				UINT32 readSetup,
				UINT32 readHold,
				UINT32 readLatency,
				UINT32 waitPeriod,
				UINT32 chw)
{
	ASSERT(id <= LCD_IF_PARALLEL_2);
	ASSERT(writeSetup <= 16U);
	ASSERT(writeHold <= 16U);
	ASSERT(writeWait <= 64U);
	ASSERT(readSetup <= 16U);
	ASSERT(readHold <= 16U);
	ASSERT(readLatency <= 64U);
	ASSERT(chw <= 16U);

	if (0 == writeHold)
		writeHold = 1;
	if (0 == writeWait)
		writeWait = 1;
	if (0 == readLatency)
		readLatency = 1;

	LCD_WaitForEngineNotBusy();

	/* (1) Config Data Width */
	{
		LCD_REG_PCNFDW pcnfdw = LCD_REG->PARALLEL_DW;

		switch (id) {
		case LCD_IF_PARALLEL_0:
			pcnfdw.PCNF0_DW = (UINT32) ifDataWidth;
			pcnfdw.PCNF0_CHW = chw;
			break;

		case LCD_IF_PARALLEL_1:
			pcnfdw.PCNF1_DW = (UINT32) ifDataWidth;
			pcnfdw.PCNF1_CHW = chw;
			break;

		case LCD_IF_PARALLEL_2:
			pcnfdw.PCNF2_DW = (UINT32) ifDataWidth;
			pcnfdw.PCNF2_CHW = chw;
			break;

		default:
			ASSERT(0);
		};

		LCD_OUTREG32(&LCD_REG->PARALLEL_DW, AS_UINT32(&pcnfdw));
	}

	/* (2) Config Timing */
	{
		UINT32 i;
		LCD_REG_PCNF config;

		i = (UINT32) id - LCD_IF_PARALLEL_0;
		config = LCD_REG->PARALLEL_CFG[i];

		config.C2WS = writeSetup;
		config.C2WH = writeHold;
		config.WST = writeWait;
		config.C2RS = readSetup;
		config.C2RH = readHold;
		config.RLT = readLatency;

		LCD_OUTREG32(&LCD_REG->PARALLEL_CFG[i], AS_UINT32(&config));
	}

	return LCD_STATUS_OK;
}

LCD_STATUS LCD_ConfigIfFormat(LCD_IF_FMT_COLOR_ORDER order,
			      LCD_IF_FMT_TRANS_SEQ transSeq,
			      LCD_IF_FMT_PADDING padding,
			      LCD_IF_FORMAT format,
			      LCD_IF_WIDTH busWidth)
{
	LCD_REG_WROI_CON ctrl = LCD_REG->WROI_CONTROL;

	ctrl.RGB_ORDER = order;
	ctrl.BYTE_ORDER = transSeq;
	ctrl.PADDING = padding;
	ctrl.DATA_FMT = (UINT32) format;
	ctrl.IF_FMT = (UINT32) busWidth;
	ctrl.IF_24 = 0;

	if (busWidth == LCD_IF_WIDTH_24_BITS)
		ctrl.IF_24 = 1;
	LCD_OUTREG32(&LCD_REG->WROI_CONTROL, AS_UINT32(&ctrl));

	return LCD_STATUS_OK;
}

LCD_STATUS LCD_ConfigSerialIF(LCD_IF_ID id,
			      LCD_IF_SERIAL_BITS bits,
			      UINT32 three_wire,
			      UINT32 sdi,
			      BOOL first_pol,
			      BOOL sck_def,
			      UINT32 div2,
			      UINT32 hw_cs,
			      UINT32 css,
			      UINT32 csh,
			      UINT32 rd_1st,
			      UINT32 rd_2nd,
			      UINT32 wr_1st,
			      UINT32 wr_2nd)
{
	LCD_REG_SCNF config;
	LCD_REG_SIF_TIMING sif_timing;
	unsigned int offset = 0;
	unsigned int sif_id = 0;

	ASSERT(id >= LCD_IF_SERIAL_0 && id <= LCD_IF_SERIAL_1);

	LCD_WaitForEngineNotBusy();

	memset(&config, 0, sizeof(config));

	if (id == LCD_IF_SERIAL_1) {
		offset = 8;
		sif_id = 1;
	}

	LCD_MASKREG32(&config, 0x07 << offset, bits << offset);
	LCD_MASKREG32(&config, 0x08 << offset, three_wire << (offset + 3));
	LCD_MASKREG32(&config, 0x10 << offset, sdi << (offset + 4));
	LCD_MASKREG32(&config, 0x20 << offset, first_pol << (offset + 5));
	LCD_MASKREG32(&config, 0x40 << offset, sck_def << (offset + 6));
	LCD_MASKREG32(&config, 0x80 << offset, div2 << (offset + 7));

	config.HW_CS = hw_cs;
	/* config.SIZE_0 = bits; */
	LCD_OUTREG32(&LCD_REG->SERIAL_CFG, AS_UINT32(&config));

	sif_timing.WR_2ND = wr_2nd;
	sif_timing.WR_1ST = wr_1st;
	sif_timing.RD_2ND = rd_2nd;
	sif_timing.RD_1ST = rd_1st;
	sif_timing.CSH = csh;
	sif_timing.CSS = css;

	LCD_OUTREG32(&LCD_REG->SIF_TIMING[sif_id], AS_UINT32(&sif_timing));

	return LCD_STATUS_OK;
}

LCD_STATUS LCD_SetSwReset(void)
{
	OUTREGBIT(LCD_REG_START, LCD_REG->START, RESET, 1);
	OUTREGBIT(LCD_REG_START, LCD_REG->START, RESET, 0);

	return LCD_STATUS_OK;
}

LCD_STATUS LCD_SetResetSignal(BOOL high)
{
	UINT32 reset = high ? 1 : 0;

	LCD_OUTREG32(&LCD_REG->RESET, reset);

	return LCD_STATUS_OK;
}

LCD_STATUS LCD_SetChipSelect(BOOL high)
{
	if (high)
		OUTREGBIT(LCD_REG_SIF_CS, LCD_REG->SIF_CS, CS0, 1);
	else
		OUTREGBIT(LCD_REG_SIF_CS, LCD_REG->SIF_CS, CS0, 0);

	return LCD_STATUS_OK;
}

/* -------------------- Command Queue -------------------- */

LCD_STATUS LCD_CmdQueueEnable(BOOL enabled)
{
	LCD_REG_WROI_CON ctrl;

	/* LCD_WaitForEngineNotBusy(); */

	ctrl = LCD_REG->WROI_CONTROL;
	ctrl.ENC = enabled ? 1 : 0;
	LCD_OUTREG32(&LCD_REG->WROI_CONTROL, AS_UINT32(&ctrl));

	return LCD_STATUS_OK;
}

LCD_STATUS LCD_CmdQueueWrite(UINT32 *cmds, UINT32 cmdCount)
{
	LCD_REG_WROI_CON ctrl;
	UINT32 i;

	ASSERT(cmdCount < ARY_SIZE(LCD_REG->CMDQ));

	/* LCD_WaitForEngineNotBusy(); */
	ctrl = LCD_REG->WROI_CONTROL;
	ctrl.COMMAND = cmdCount - 1;
	LCD_OUTREG32(&LCD_REG->WROI_CONTROL, AS_UINT32(&ctrl));

	for (i = 0; i < cmdCount; ++i)
		LCD_OUTREG32(&LCD_REG->CMDQ[i], cmds[i]);

	return LCD_STATUS_OK;
}

/* -------------------- Layer Configurations -------------------- */
LCD_STATUS LCD_LayerEnable(LCD_LAYER_ID id, BOOL enable)
{
	if (LCD_LAYER_ALL == id)
		return LCD_STATUS_OK;

	cached_layer_config[id].layer_en = enable;
	cached_layer_config[id].isDirty = true;

	return LCD_STATUS_OK;
}

LCD_STATUS LCD_LayerSetAddress(LCD_LAYER_ID id, UINT32 address)
{
	cached_layer_config[id].addr = address;
	cached_layer_config[id].isDirty = true;

	return LCD_STATUS_OK;
}

UINT32 LCD_DisableAllLayer(UINT32 vram_start, UINT32 vram_end)
{
	int id;
	int layer_enable = 0;
	DISP_LOG_PRINT("[LCD] %s(%d, %d)\n", __func__, vram_start, vram_end);

	for (id = 0; id < DDP_OVL_LAYER_MUN; id++) {
		if (cached_layer_config[id].layer_en == 0)
			continue;

		if (cached_layer_config[id].addr >= vram_start &&
		    cached_layer_config[id].addr < vram_end) {
			DISP_LOG_PRINT("[LCD] not disable(%d)\n", id);
			layer_enable |= (1 << id);
			continue;
		}

		DISP_LOG_PRINT("[LCD] disable(%d)\n", id);
		cached_layer_config[id].layer_en = 0;
		cached_layer_config[id].isDirty = true;
	}
	return layer_enable;
}

LCD_STATUS LCD_LayerSetSize(LCD_LAYER_ID id, UINT32 width, UINT32 height)
{
	return LCD_STATUS_OK;
}

LCD_STATUS LCD_LayerSetPitch(LCD_LAYER_ID id, UINT32 pitch)
{
	cached_layer_config[id].src_pitch = pitch;
	return LCD_STATUS_OK;
}

LCD_STATUS LCD_LayerSetOffset(LCD_LAYER_ID id, UINT32 x, UINT32 y)
{
	return LCD_STATUS_OK;
}

LCD_STATUS LCD_LayerSetFormat(LCD_LAYER_ID id, LCD_LAYER_FORMAT format)
{
	cached_layer_config[id].fmt = format;
	return LCD_STATUS_OK;
}

LCD_STATUS LCD_LayerSetSourceColorKey(LCD_LAYER_ID id, BOOL enable, UINT32 colorKey)
{
	cached_layer_config[id].key = colorKey;
	cached_layer_config[id].keyEn = enable;
	return LCD_STATUS_OK;
}

LCD_STATUS LCD_SetRoiWindow(UINT32 x, UINT32 y, UINT32 width, UINT32 height)
{
	LCD_REG_SIZE size;

	size.WIDTH = (UINT16) width;
	size.HEIGHT = (UINT16) height;

	LCD_OUTREG32(&LCD_REG->WROI_SIZE, AS_UINT32(&size));
	_lcdContext.roiWndSize = size;

	return LCD_STATUS_OK;
}

/* -------------------- Tearing Control -------------------- */

LCD_STATUS LCD_TE_Enable(BOOL enable)
{
	LCD_REG_TECON tecon = LCD_REG->TEARING_CFG;

	tecon.ENABLE = enable ? 1 : 0;
	LCD_OUTREG32(&LCD_REG->TEARING_CFG, AS_UINT32(&tecon));

	return LCD_STATUS_OK;
}

LCD_STATUS LCD_TE_SetMode(LCD_TE_MODE mode)
{
	LCD_REG_TECON tecon = LCD_REG->TEARING_CFG;

	tecon.MODE = (LCD_TE_MODE_VSYNC_OR_HSYNC == mode) ? 1 : 0;
	LCD_OUTREG32(&LCD_REG->TEARING_CFG, AS_UINT32(&tecon));

	return LCD_STATUS_OK;
}

LCD_STATUS LCD_TE_SetEdgePolarity(BOOL polarity)
{
	LCD_REG_TECON tecon = LCD_REG->TEARING_CFG;

	tecon.EDGE_SEL = (polarity ? 1 : 0);
	LCD_OUTREG32(&LCD_REG->TEARING_CFG, AS_UINT32(&tecon));

	return LCD_STATUS_OK;
}

LCD_STATUS LCD_TE_ConfigVHSyncMode(UINT32 hsDelayCnt,
				   UINT32 vsWidthCnt,
				   LCD_TE_VS_WIDTH_CNT_DIV vsWidthCntDiv)
{
/*
    LCD_REG_TECON tecon = LCD_REG->TEARING_CFG;
    tecon.HS_MCH_CNT = (hsDelayCnt ? hsDelayCnt - 1 : 0);
    tecon.VS_WLMT = (vsWidthCnt ? vsWidthCnt - 1 : 0);
    tecon.VS_CNT_DIV = vsWidthCntDiv;
    LCD_OUTREG32(&LCD_REG->TEARING_CFG, AS_UINT32(&tecon));
*/

	return LCD_STATUS_OK;
}

/* -------------------- Operations -------------------- */

LCD_STATUS LCD_SelectWriteIF(LCD_IF_ID id)
{
	LCD_REG_CMD_ADDR cmd_addr;
	LCD_REG_DAT_ADDR dat_addr;

	switch (id) {
	case LCD_IF_PARALLEL_0:
		cmd_addr.addr = 0;
		break;
	case LCD_IF_PARALLEL_1:
		cmd_addr.addr = 2;
		break;
	case LCD_IF_PARALLEL_2:
		cmd_addr.addr = 4;
		break;
	case LCD_IF_SERIAL_0:
		cmd_addr.addr = 8;
		break;
	case LCD_IF_SERIAL_1:
		cmd_addr.addr = 0xA;
		break;
	default:
		ASSERT(0);
	}
	dat_addr.addr = cmd_addr.addr + 1;
	LCD_OUTREG16(&LCD_REG->WROI_CMD_ADDR, AS_UINT16(&cmd_addr));
	LCD_OUTREG16(&LCD_REG->WROI_DATA_ADDR, AS_UINT16(&dat_addr));

	return LCD_STATUS_OK;
}

static inline void _LCD_WriteIF(DWORD baseAddr, UINT32 value, LCD_IF_MCU_WRITE_BITS bits)
{
	switch (bits) {
	case LCD_IF_MCU_WRITE_8BIT:
		LCD_OUTREG8((UINT8 *) baseAddr, value);
		break;

	case LCD_IF_MCU_WRITE_16BIT:
		LCD_OUTREG16((UINT16 *) baseAddr, value);
		break;

	case LCD_IF_MCU_WRITE_32BIT:
		LCD_OUTREG32((UINT32 *) baseAddr, value);
		break;

	default:
		ASSERT(0);
	}
}

LCD_STATUS LCD_WriteIF(LCD_IF_ID id, LCD_IF_A0_MODE a0,
		       UINT32 value, LCD_IF_MCU_WRITE_BITS bits)
{
	DWORD baseAddr = 0;

	switch (id) {
	case LCD_IF_PARALLEL_0:
		baseAddr = (DWORD) &LCD_REG->PCMD0;
		break;
	case LCD_IF_PARALLEL_1:
		baseAddr = (DWORD) &LCD_REG->PCMD1;
		break;
	case LCD_IF_PARALLEL_2:
		baseAddr = (DWORD) &LCD_REG->PCMD2;
		break;
	case LCD_IF_SERIAL_0:
		baseAddr = (DWORD) &LCD_REG->SCMD0;
		break;
	case LCD_IF_SERIAL_1:
		baseAddr = (DWORD) &LCD_REG->SCMD1;
		break;
	default:
		ASSERT(0);
	}

	if (LCD_IF_A0_HIGH == a0)
		baseAddr += LCD_A0_HIGH_OFFSET;

	_LCD_WriteIF(baseAddr, value, bits);

	return LCD_STATUS_OK;
}

static inline UINT32 _LCD_ReadIF(DWORD baseAddr, LCD_IF_MCU_WRITE_BITS bits)
{
	switch (bits) {
	case LCD_IF_MCU_WRITE_8BIT:
		return (UINT32) INREG8(baseAddr);

	case LCD_IF_MCU_WRITE_16BIT:
		return (UINT32) INREG16(baseAddr);

	case LCD_IF_MCU_WRITE_32BIT:
		return (UINT32) INREG32(baseAddr);

	default:
		ASSERT(0);
	}
}

LCD_STATUS LCD_ReadIF(LCD_IF_ID id, LCD_IF_A0_MODE a0,
		      UINT32 *value, LCD_IF_MCU_WRITE_BITS bits)
{
	DWORD baseAddr = 0;

	if (NULL == value)
		return LCD_STATUS_ERROR;

	switch (id) {
	case LCD_IF_PARALLEL_0:
		baseAddr = (DWORD) &LCD_REG->PCMD0;
		break;
	case LCD_IF_PARALLEL_1:
		baseAddr = (DWORD) &LCD_REG->PCMD1;
		break;
	case LCD_IF_PARALLEL_2:
		baseAddr = (DWORD) &LCD_REG->PCMD2;
		break;
	case LCD_IF_SERIAL_0:
		baseAddr = (DWORD) &LCD_REG->SCMD0;
		break;
	case LCD_IF_SERIAL_1:
		baseAddr = (DWORD) &LCD_REG->SCMD1;
		break;
	default:
		ASSERT(0);
	}

	if (LCD_IF_A0_HIGH == a0)
		baseAddr += LCD_A0_HIGH_OFFSET;

	*value = _LCD_ReadIF(baseAddr, bits);

	return LCD_STATUS_OK;
}

bool LCD_IsLayerEnable(LCD_LAYER_ID id)
{
	ASSERT(id <= LCD_LAYER_NUM);
	return (bool) (cached_layer_config[id].layer_en);
}

extern struct mutex OverlaySettingMutex;
LCD_STATUS LCD_StartTransfer(BOOL blocking, BOOL isMutexLocked)
{
	DBI_DRV_FUNC("[%s]:enter\n", __func__);

	LCD_WaitForEngineNotBusy();
	DBG_OnTriggerLcd();

	if (!isMutexLocked) {
		disp_path_get_mutex();
		mutex_lock(&OverlaySettingMutex);
	}

	LCD_OUTREG32(&LCD_REG->START, 0);
	LCD_OUTREG32(&LCD_REG->START, (1 << 15));

	if (!isMutexLocked) {
		mutex_unlock(&OverlaySettingMutex);
		disp_path_release_mutex();
	}

	if (blocking)
		_WaitForLCDEngineComplete();

	return LCD_STATUS_OK;
}

LCD_STATUS LCD_DumpRegisters(void)
{
	UINT32 i;

	DISP_LOG_PRINT("[LCD] ---------- Start dump LCD registers ----------\n");

	for (i = 0; i < offsetof(LCD_REGS, DBI_ULTRA_TH); i += 4)
		DISP_LOG_PRINT("LCD+%04x : 0x%08x\n", i, INREG32(DISP_DBI_BASE + i));

	return LCD_STATUS_OK;
}

LCD_STATUS LCD_FMDesense_Query(void)
{
	return LCD_STATUS_OK;
}

LCD_STATUS LCD_FM_Desense(LCD_IF_ID id, unsigned long freq)
{
	UINT32 a, b;
	UINT32 c, d;
	UINT32 wst, c2wh, chw, write_cycles;
	LCD_REG_PCNF config;
	/* LCD_REG_WROI_CON ctrl; */
	LCD_REG_PCNFDW pcnfdw;

	LCD_OUTREG32(&config, AS_UINT32(&LCD_REG->PARALLEL_CFG[(UINT32) id]));
	DBI_DRV_INFO("[enter LCD_FM_Desense]:parallel IF = 0x%x, ctrl = 0x%x\n",
		id, INREG32(&LCD_REG->PARALLEL_CFG[(UINT32) id]));
	wst = config.WST;
	c2wh = config.C2WH;

	LCD_OUTREG32(&pcnfdw, AS_UINT32(&LCD_REG->PARALLEL_DW));

	switch (id) {
	case LCD_IF_PARALLEL_0:
		chw = pcnfdw.PCNF0_CHW;
		break;
	case LCD_IF_PARALLEL_1:
		chw = pcnfdw.PCNF1_CHW;
		break;
	case LCD_IF_PARALLEL_2:
		chw = pcnfdw.PCNF2_CHW;
		break;
	default:
		ASSERT(0);
	}

	a = 13000 - freq * 10 - 20;
	b = 13000 - freq * 10 + 20;
	write_cycles = wst + c2wh + chw + 2;
	c = (a * write_cycles) % 13000;
	d = (b * write_cycles) % 13000;
	a = (a * write_cycles) / 13000;
	b = (b * write_cycles) / 13000;

	if ((b > a) || (c == 0) || (d == 0)) {	/* need modify setting to avoid interference */
		DBI_DRV_INFO("[LCD_FM_Desense] need to modify lcd setting, freq = %ld\n", freq);
		wst -= wst_step_LCD;
		wst_step_LCD = 0 - wst_step_LCD;

		config.WST = wst;
		LCD_WaitForNotBusy();
		LCD_OUTREG32(&LCD_REG->PARALLEL_CFG[(UINT32) id], AS_UINT32(&config));
	} else {
		DBI_DRV_INFO("[LCD_FM_Desense] not need to modify lcd setting, freq = %ld\n", freq);
	}
	DBI_DRV_INFO("[leave LCD_FM_Desense]:parallel = 0x%x\n", INREG32(&LCD_REG->PARALLEL_CFG[(UINT32) id]));

	return LCD_STATUS_OK;
}

LCD_STATUS LCD_Reset_WriteCycle(LCD_IF_ID id)
{
	LCD_REG_PCNF config;
	UINT32 wst;

	DBI_DRV_INFO("[enter LCD_Reset_WriteCycle]:parallel = 0x%x\n", INREG32(&LCD_REG->PARALLEL_CFG[(UINT32) id]));

	if (wst_step_LCD > 0) {	/* have modify lcd setting, so when fm turn off, we must decrease wst to default setting */
		DBI_DRV_INFO("[LCD_Reset_WriteCycle] need to reset lcd setting\n");
		LCD_OUTREG32(&config, AS_UINT32(&LCD_REG->PARALLEL_CFG[(UINT32) id]));
		wst = config.WST;
		wst -= wst_step_LCD;
		wst_step_LCD = 0 - wst_step_LCD;

		config.WST = wst;
		LCD_WaitForNotBusy();
		LCD_OUTREG32(&LCD_REG->PARALLEL_CFG[(UINT32) id], AS_UINT32(&config));
	} else {
		DBI_DRV_INFO("[LCD_Reset_WriteCycle] parallel is default setting, not need to reset it\n");
	}
	DBI_DRV_INFO("[leave LCD_Reset_WriteCycle]:parallel = 0x%x\n", INREG32(&LCD_REG->PARALLEL_CFG[(UINT32) id]));

	return LCD_STATUS_OK;
}

LCD_STATUS LCD_Get_Default_WriteCycle(LCD_IF_ID id, unsigned int *write_cycle)
{
	UINT32 wst, c2wh, chw;
	LCD_REG_PCNF config;
	/* LCD_REG_WROI_CON ctrl; */
	LCD_REG_PCNFDW pcnfdw;

	if (is_get_default_write_cycle) {
		*write_cycle = default_write_cycle;
		return LCD_STATUS_OK;
	}

	LCD_OUTREG32(&config, AS_UINT32(&LCD_REG->PARALLEL_CFG[(UINT32) id]));
	DBI_DRV_INFO("[enter LCD_Get_Default_WriteCycle]:parallel IF = 0x%x, ctrl = 0x%x\n",
		id, INREG32(&LCD_REG->PARALLEL_CFG[(UINT32) id]));
	wst = config.WST;
	c2wh = config.C2WH;

	LCD_OUTREG32(&pcnfdw, AS_UINT32(&LCD_REG->PARALLEL_DW));

	switch (id) {
	case LCD_IF_PARALLEL_0:
		chw = pcnfdw.PCNF0_CHW;
		break;
	case LCD_IF_PARALLEL_1:
		chw = pcnfdw.PCNF1_CHW;
		break;
	case LCD_IF_PARALLEL_2:
		chw = pcnfdw.PCNF2_CHW;
		break;
	default:
		ASSERT(0);
	}
	*write_cycle = wst + c2wh + chw + 2;
	default_write_cycle = *write_cycle;
	default_wst = wst;
	is_get_default_write_cycle = TRUE;
	DBI_DRV_INFO("[leave LCD_Get_Default_WriteCycle]:Default_Write_Cycle = %d\n", *write_cycle);

	return LCD_STATUS_OK;
}

LCD_STATUS LCD_Get_Current_WriteCycle(LCD_IF_ID id, unsigned int *write_cycle)
{
	UINT32 wst, c2wh, chw;
	LCD_REG_PCNF config;
	/* LCD_REG_WROI_CON ctrl; */
	LCD_REG_PCNFDW pcnfdw;

	LCD_OUTREG32(&config, AS_UINT32(&LCD_REG->PARALLEL_CFG[(UINT32) id]));
	DBI_DRV_INFO("[enter LCD_Get_Current_WriteCycle]:parallel IF = 0x%x, ctrl = 0x%x\n",
		id, INREG32(&LCD_REG->PARALLEL_CFG[(UINT32) id]));
	wst = config.WST;
	c2wh = config.C2WH;

	LCD_OUTREG32(&pcnfdw, AS_UINT32(&LCD_REG->PARALLEL_DW));
	switch (id) {
	case LCD_IF_PARALLEL_0:
		chw = pcnfdw.PCNF0_CHW;
		break;
	case LCD_IF_PARALLEL_1:
		chw = pcnfdw.PCNF1_CHW;
		break;
	case LCD_IF_PARALLEL_2:
		chw = pcnfdw.PCNF2_CHW;
		break;
	default:
		ASSERT(0);
	}

	*write_cycle = wst + c2wh + chw + 2;
	DBI_DRV_INFO("[leave LCD_Get_Current_WriteCycle]:Default_Write_Cycle = %d\n", *write_cycle);

	return LCD_STATUS_OK;
}

LCD_STATUS LCD_Change_WriteCycle(LCD_IF_ID id, unsigned int write_cycle)
{
	UINT32 wst;
	LCD_REG_PCNF config;

	LCD_OUTREG32(&config, AS_UINT32(&LCD_REG->PARALLEL_CFG[(UINT32) id]));
	DBI_DRV_INFO("[enter LCD_Change_WriteCycle]:parallel IF = 0x%x, ctrl = 0x%x\n",
		INREG32(&LCD_REG->PARALLEL_CFG[(UINT32) id]), INREG32(&LCD_REG->WROI_CONTROL));

	DBI_DRV_INFO("[LCD_Change_WriteCycle] modify lcd setting\n");
	wst = write_cycle - default_write_cycle + default_wst;

	config.WST = wst;
	LCD_WaitForNotBusy();
	LCD_OUTREG32(&LCD_REG->PARALLEL_CFG[(UINT32) id], AS_UINT32(&config));
	DBI_DRV_INFO("[leave LCD_Change_WriteCycle]:parallel = 0x%x\n", INREG32(&LCD_REG->PARALLEL_CFG[(UINT32) id]));

	return LCD_STATUS_OK;
}

LCD_STATUS LCD_read_lcm_fb(unsigned char *buffer)
{
	LCD_WaitForNotBusy();

	/* if read_fb not impl, should return info */
	if (lcm_drv->read_fb)
		lcm_drv->read_fb(buffer);

	return LCD_STATUS_OK;
}

unsigned int LCD_Check_LCM(UINT32 color)
{
	unsigned int ret = 1;
	unsigned char buffer[60] = { 0 };
	unsigned int i = 0;

	if (lcm_drv->ata_check) {
		ret = lcm_drv->ata_check(buffer);
	} else {
		LCD_read_lcm_fb(buffer);
		for (i = 0; i < 60; i++)
			DISP_LOG_PRINT("%d\n", buffer[i]);

		for (i = 0; i < 60; i += 3) {
			DISP_LOG_PRINT("read pixel = 0x%x,", (buffer[i] << 16) | (buffer[i + 1] << 8) | (buffer[i + 2]));
			if (((buffer[i] << 16) | (buffer[i + 1] << 8) | (buffer[i + 2])) != (color & 0xFFFFFF)) {
				ret = 0;
				break;
			}
		}
	}

	return ret;
}

LCD_STATUS LCD_MIPI_PowerOff(void)
{
	int ret = 0;

	if (clock_is_on(MT_CG_MIPI_26M_DBG_EN))
		ret += disable_clock(MT_CG_MIPI_26M_DBG_EN, "DSI");
	if (ret > 0)
		pr_err("[LCD] power manager API return FALSE\n");

	return LCD_STATUS_OK;
}
