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

#include <generated/autoconf.h>
#include <linux/module.h>
#include <linux/mm.h>
#include <linux/init.h>
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/kthread.h>
#include <linux/vmalloc.h>
#include <linux/disp_assert_layer.h>
#include <linux/semaphore.h>
#include <linux/mutex.h>
#include <linux/leds-mt65xx.h>
#include <linux/version.h>
#include <linux/sched.h>
#include <asm/uaccess.h>
#include <asm/atomic.h>
#include <asm/mach-types.h>
#include <asm/cacheflush.h>
#include <asm/io.h>

#include <mach/dma.h>
#include <mach/irqs.h>
#include <linux/dma-mapping.h>

#include "mach/mt_boot.h"

#include "disp_debug.h"
#include "disp_drv.h"
#include "ddp_hal.h"
#include "disp_drv_log.h"
#include "disp_hal.h"

#include "mtkfb.h"
#include "mtkfb_console.h"
#include "mtkfb_info.h"
#include "ddp_ovl.h"
#include "disp_drv_platform.h"
#include <mt-plat/aee.h>
/* Fence Sync Object */
#if defined(MTK_FB_SYNC_SUPPORT)
#include "disp_sync.h"
#endif
#include "disp_mgr.h"
#include "disp_session.h"

/* #define FPGA_DEBUG_PAN */

unsigned int EnableVSyncLog = 0;

static u32 MTK_FB_XRES;
static u32 MTK_FB_YRES;
static u32 MTK_FB_BPP;
static u32 MTK_FB_PAGES;
static u32 fb_xres_update;
static u32 fb_yres_update;

#define MTK_FB_XRESV (ALIGN_TO(MTK_FB_XRES, disphal_get_fb_alignment()))
#define MTK_FB_YRESV (ALIGN_TO(MTK_FB_YRES, disphal_get_fb_alignment()) * MTK_FB_PAGES)	/* For page flipping */
#define MTK_FB_BYPP  ((MTK_FB_BPP + 7) >> 3)
#define MTK_FB_LINE  (ALIGN_TO(MTK_FB_XRES, disphal_get_fb_alignment()) * MTK_FB_BYPP)
#define MTK_FB_SIZE  (MTK_FB_LINE * ALIGN_TO(MTK_FB_YRES, disphal_get_fb_alignment()))

#define MTK_FB_SIZEV (MTK_FB_LINE * ALIGN_TO(MTK_FB_YRES, disphal_get_fb_alignment()) * MTK_FB_PAGES)

#define CHECK_RET(expr) \
	do { \
		int ret = (expr); \
		ASSERT(0 == ret); \
	} while (0)

void mtkfb_log_enable(int enable)
{
	mtkfb_dbg_log = enable;
	mtkfb_dbg_info_log = enable;
	mtkfb_dbg_fence_log = enable;
	mtkfb_dbg_func_log = enable;
	MTKFB_INFO("mtkfb log %s\n", enable ? "enabled" : "disabled");
}

void mtkfb_clear_lcm(void);
/* --------------------------------------------------------------------------- */
/* local variables */
/* --------------------------------------------------------------------------- */
#ifdef MTKFB_FPGA_ONLY
static BOOL mtkfb_enable_mmu = FALSE;
#else
static BOOL mtkfb_enable_mmu = TRUE;
#endif

#ifdef CONFIG_MTK_AEE_POWERKEY_HANG_DETECT
unsigned int screen_update_cnt = 0;
#endif
unsigned int fb_pa = 0;
unsigned int decouple_addr = 0;	/* It's PA = MVA after m4u mapping */
unsigned int decouple_size = 0;

static const struct timeval FRAME_INTERVAL = { 0, 30000 };	/* 33ms */

atomic_t has_pending_update = ATOMIC_INIT(0);
struct fb_overlay_layer video_layerInfo;
UINT32 dbr_backup = 0;
UINT32 dbg_backup = 0;
UINT32 dbb_backup = 0;
static unsigned int video_rotation;
static UINT32 mtkfb_using_layer_type = LAYER_2D;
static bool hwc_force_fb_enabled = true;
bool is_ipoh_bootup = false;
struct fb_info *mtkfb_fbi;
struct fb_overlay_layer fb_layer_context;

/* This mutex is used to prevent tearing due to page flipping when adbd is
   reading the front buffer
*/
DEFINE_SEMAPHORE(sem_flipping);
DEFINE_SEMAPHORE(sem_early_suspend);
DEFINE_SEMAPHORE(sem_overlay_buffer);

extern OVL_CONFIG_STRUCT cached_layer_config[DDP_OVL_LAYER_MUN];
DEFINE_MUTEX(OverlaySettingMutex);
atomic_t OverlaySettingDirtyFlag = ATOMIC_INIT(0);
atomic_t OverlaySettingApplied = ATOMIC_INIT(0);
unsigned int PanDispSettingPending = 0;
unsigned int PanDispSettingDirty = 0;
unsigned int PanDispSettingApplied = 0;

DECLARE_WAIT_QUEUE_HEAD(reg_update_wq);

unsigned int need_esd_check = 0;
DECLARE_WAIT_QUEUE_HEAD(esd_check_wq);

extern unsigned int disp_running;
extern wait_queue_head_t disp_done_wq;

DEFINE_MUTEX(ScreenCaptureMutex);

BOOL is_early_suspended = FALSE;
BOOL is_lcm_always_on = FALSE;
unsigned int display_power_state = DISP_POWER_MODE_NORMAL;
unsigned int start_update_state = 0;
static int sem_flipping_cnt = 1;
static int sem_early_suspend_cnt = 1;
static int sem_overlay_buffer_cnt = 1;
static int vsync_cnt;
static int blank_skip_count;

extern BOOL dispsys_dynamic_cg_control_enable;
extern LCM_PARAMS *lcm_params;
extern disp_session_config disp_config;
extern unsigned int is_video_mode_running;
extern unsigned int isAEEEnabled;
BLOCKING_NOTIFIER_HEAD(ambient_mode_notifier);
int ambient_mode_notifier_register(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&ambient_mode_notifier, nb);
}
EXPORT_SYMBOL(ambient_mode_notifier_register);

int ambient_mode_notifier_unregister(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&ambient_mode_notifier, nb);
}
EXPORT_SYMBOL(ambient_mode_notifier_unregister);

/* --------------------------------------------------------------------------- */
/* local function declarations */
/* --------------------------------------------------------------------------- */

static int mtkfb_set_overlay_layer(struct fb_info *info,
				   struct fb_overlay_layer *layerInfo,
				   unsigned int locked);
static int mtkfb_get_overlay_layer_info(struct fb_overlay_layer_info *layerInfo);
static int mtkfb_update_screen(struct fb_info *info);
static void mtkfb_update_screen_impl(void);
unsigned int mtkfb_fm_auto_test(void);

static void mtkfb_late_resume(void);
static void mtkfb_early_suspend(void);

/* --------------------------------------------------------------------------- */
/* Timer Routines */
/* --------------------------------------------------------------------------- */
static struct task_struct *screen_update_task;
static struct task_struct *esd_recovery_task;
unsigned int lcd_fps = 6000;

void mtkfb_pan_disp_test(void)
{
	MTKFB_FUNC();
	if (down_interruptible(&sem_flipping)) {
		MTKFB_WRAN("[fb driver] can't get semaphore:%d\n", __LINE__);
		return;
	}
	sem_flipping_cnt--;
	pr_warn("[MTKFB] wait sem_flipping\n");
	if (down_interruptible(&sem_early_suspend)) {
		MTKFB_WRAN("[fb driver] can't get semaphore:%d\n", __LINE__);
		sem_flipping_cnt++;
		up(&sem_flipping);
		return;
	}
	sem_early_suspend_cnt--;

	pr_warn("[MTKFB] wait sem_early_suspend\n");
	if (down_interruptible(&sem_overlay_buffer)) {
		MTKFB_WRAN("[fb driver] can't get semaphore,%d\n", __LINE__);
		sem_early_suspend_cnt++;
		up(&sem_early_suspend);

		sem_flipping_cnt++;
		up(&sem_flipping);
		return;
	}
	sem_overlay_buffer_cnt--;
	pr_warn("[MTKFB] wait sem_overlay_buffer\n");
	if (is_early_suspended)
		goto end;

end:
	sem_overlay_buffer_cnt++;
	sem_early_suspend_cnt++;
	sem_flipping_cnt++;
	up(&sem_overlay_buffer);
	up(&sem_early_suspend);
	up(&sem_flipping);
}

void mtkfb_show_sem_cnt(void)
{
	MTKFB_INFO("[FB driver: sem cnt = %d, %d, %d. fps = %d, vsync_cnt = %d\n", sem_overlay_buffer_cnt, sem_early_suspend_cnt, sem_flipping_cnt, lcd_fps, vsync_cnt);
	MTKFB_INFO("[FB driver: sem cnt = %d, %d, %d\n", sem_overlay_buffer.count, sem_early_suspend.count, sem_flipping.count);
}

void mtkfb_hang_test(bool en)
{
	MTKFB_FUNC();
	if (en) {
		if (down_interruptible(&sem_flipping)) {
			MTKFB_WRAN("[fb driver] can't get semaphore:%d\n", __LINE__);
			return;
		}
		sem_flipping_cnt--;
	} else {
		sem_flipping_cnt++;
		up(&sem_flipping);
	}
}

BOOL esd_kthread_pause = TRUE;

void esd_recovery_pause(BOOL en)
{
	esd_kthread_pause = en;
}

static int esd_recovery_kthread(void *data)
{
	/* struct sched_param param = { .sched_priority = RTPM_PRIO_SCRN_UPDATE }; */
	/* sched_setscheduler(current, SCHED_RR, &param); */
	MTKFB_INFO("enter esd_recovery_kthread()\n");
	for (;;) {
		if (kthread_should_stop())
			break;

		MTKFB_INFO("sleep start in esd_recovery_kthread()\n");
		msleep(2000);	/* 2s */
		MTKFB_INFO("sleep ends in esd_recovery_kthread()\n");

		if (!esd_kthread_pause) {
			if (is_early_suspended) {
				MTKFB_INFO("is_early_suspended in esd_recovery_kthread()\n");
				continue;
			}
			/* execute ESD check and recover flow */
			MTKFB_INFO("DISP_EsdCheck starts\n");
			need_esd_check = 1;
			wait_event_interruptible(esd_check_wq, !need_esd_check);
			MTKFB_INFO("DISP_EsdCheck ends\n");
		}
	}

	MTKFB_INFO("exit esd_recovery_kthread()\n");
	return 0;
}

/*
 * ---------------------------------------------------------------------------
 *  mtkfb_set_lcm_inited() will be called in mt6516_board_init()
 * ---------------------------------------------------------------------------
 */
BOOL is_lcm_inited = FALSE;
void mtkfb_set_lcm_inited(BOOL inited)
{
	is_lcm_inited = inited;
}

unsigned long long fb_address_lk = 0;
unsigned long long fb_size_lk = 0;
void mtkfb_set_fb_lk(unsigned long long address, unsigned long long size)
{
	fb_address_lk = address;
	fb_size_lk = size;
}

unsigned int mtkfb_fb_lk_copy_size(void)
{
	return DISP_GetFBRamSize() / DISP_GetPages();
}

/*
 * ---------------------------------------------------------------------------
 * fbdev framework callbacks and the ioctl interface
 * ---------------------------------------------------------------------------
 */
/* Called each time the mtkfb device is opened */
static int mtkfb_open(struct fb_info *info, int user)
{
	NOT_REFERENCED(info);
	NOT_REFERENCED(user);

	MSG_FUNC_ENTER();
	MSG_FUNC_LEAVE();
	return 0;
}

/* Called when the mtkfb device is closed. We make sure that any pending
 * gfx DMA operations are ended, before we return. */
static int mtkfb_release(struct fb_info *info, int user)
{
	NOT_REFERENCED(info);
	NOT_REFERENCED(user);

	MSG_FUNC_ENTER();
	MSG_FUNC_LEAVE();
	return 0;
}

/* Store a single color palette entry into a pseudo palette or the hardware
 * palette if one is available. For now we support only 16bpp and thus store
 * the entry only to the pseudo palette.
 */
static int mtkfb_setcolreg(u_int regno, u_int red, u_int green,
			   u_int blue, u_int transp,
			   struct fb_info *info)
{
	int r = 0;
	unsigned bpp, m;

	NOT_REFERENCED(transp);

	MSG_FUNC_ENTER();

	bpp = info->var.bits_per_pixel;
	m = 1 << bpp;
	if (regno >= m) {
		r = -EINVAL;
		goto exit;
	}

	switch (bpp) {
	case 16:
		/* RGB 565 */
		((u32 *) (info->pseudo_palette))[regno] =
			((red & 0xF800) |
			((green & 0xFC00) >> 5) |
			((blue & 0xF800) >> 11));
		break;
	case 32:
		/* ARGB8888 */
		((u32 *) (info->pseudo_palette))[regno] =
			(0xff000000) |
			((red & 0xFF00) << 8) |
			((green & 0xFF00)) |
			((blue & 0xFF00) >> 8);
		break;

		/* TODO: RGB888, BGR888, ABGR8888 */

	default:
		ASSERT(0);
	}

exit:
	MSG_FUNC_LEAVE();
	return r;
}

static void mtkfb_update_screen_impl(void)
{
	BOOL down_sem = FALSE;
	MMProfileLog(MTKFB_MMP_Events.UpdateScreenImpl, MMProfileFlagStart);
	if (down_interruptible(&sem_overlay_buffer)) {
		MTKFB_WRAN("[FB Driver] can't get semaphore in mtkfb_update_screen_impl()\n");
	} else {
		down_sem = TRUE;
		sem_overlay_buffer_cnt--;
	}

	if (!is_early_suspended)
		DISP_CHECK_RET(DISP_UpdateScreen(0, 0, fb_xres_update, fb_yres_update));

	if (down_sem) {
		sem_overlay_buffer_cnt++;
		up(&sem_overlay_buffer);
	}
	MMProfileLog(MTKFB_MMP_Events.UpdateScreenImpl, MMProfileFlagEnd);
}

static int mtkfb_update_screen(struct fb_info *info)
{
	if (down_interruptible(&sem_early_suspend)) {
		MTKFB_WRAN("[FB Driver] can't get semaphore in mtkfb_update_screen()\n");
		return -ERESTARTSYS;
	}
	sem_early_suspend_cnt--;

	mtkfb_update_screen_impl();

/* End: */
	sem_early_suspend_cnt++;
	up(&sem_early_suspend);
	return 0;
}

static unsigned int BL_level;
static BOOL BL_set_level_resume = FALSE;
int mtkfb_set_backlight_level(unsigned int level)
{
	MTKFB_INFO("mtkfb_set_backlight_level:%d\n", level);
	if (down_interruptible(&sem_flipping)) {
		MTKFB_WRAN("[FB Driver] can't get semaphore:%d\n", __LINE__);
		return -ERESTARTSYS;
	}
	sem_flipping_cnt--;
	if (down_interruptible(&sem_early_suspend)) {
		MTKFB_WRAN("[FB Driver] can't get semaphore:%d\n", __LINE__);
		sem_flipping_cnt++;
		up(&sem_flipping);
		return -ERESTARTSYS;
	}

	sem_early_suspend_cnt--;
	if (is_early_suspended) {
		BL_level = level;
		BL_set_level_resume = TRUE;
		MTKFB_WRAN("[FB driver] set backlight level but FB has been suspended\n");
		goto End;
	}
	DISP_SetBacklight(level);
	BL_set_level_resume = FALSE;
End:
	sem_flipping_cnt++;
	sem_early_suspend_cnt++;
	up(&sem_early_suspend);
	up(&sem_flipping);
	return 0;
}

int mtkfb_set_backlight_mode(unsigned int mode)
{
	if (down_interruptible(&sem_flipping)) {
		MTKFB_WRAN("[FB Driver] can't get semaphore:%d\n", __LINE__);
		return -ERESTARTSYS;
	}
	sem_flipping_cnt--;
	if (down_interruptible(&sem_early_suspend)) {
		MTKFB_WRAN("[FB Driver] can't get semaphore:%d\n", __LINE__);
		sem_flipping_cnt++;
		up(&sem_flipping);
		return -ERESTARTSYS;
	}

	sem_early_suspend_cnt--;
	if (is_early_suspended)
		goto End;

	DISP_SetBacklight_mode(mode);
End:
	sem_flipping_cnt++;
	sem_early_suspend_cnt++;
	up(&sem_early_suspend);
	up(&sem_flipping);
	return 0;
}

int mtkfb_set_backlight_pwm(int div)
{
	if (down_interruptible(&sem_flipping)) {
		MTKFB_WRAN("[FB Driver] can't get semaphore:%d\n", __LINE__);
		return -ERESTARTSYS;
	}
	sem_flipping_cnt--;
	if (down_interruptible(&sem_early_suspend)) {
		MTKFB_WRAN("[FB Driver] can't get semaphore:%d\n", __LINE__);
		sem_flipping_cnt++;
		up(&sem_flipping);
		return -ERESTARTSYS;
	}
	sem_early_suspend_cnt--;
	if (is_early_suspended)
		goto End;
	DISP_SetPWM(div);
End:
	sem_flipping_cnt++;
	sem_early_suspend_cnt++;
	up(&sem_early_suspend);
	up(&sem_flipping);
	return 0;
}

int mtkfb_get_backlight_pwm(int div, unsigned int *freq)
{
	DISP_GetPWM(div, freq);
	return 0;
}

void mtkfb_waitVsync(void)
{
	if (is_early_suspended) {
		if (!is_lcm_always_on)
			MTKFB_WRAN("[MTKFB_VSYNC]:mtkfb has suspend, return directly\n");
		msleep(10);
		return;
	}
	vsync_cnt++;
	DISP_WaitVSYNC();
	vsync_cnt--;
	return;
}

/* Used for HQA test */
/*-------------------------------------------------------------
Note: The using scenario must be
1. switch normal mode to factory mode when LCD screen is on
2. switch factory mode to normal mode(optional)
-------------------------------------------------------------*/
static struct fb_var_screeninfo fbi_var_backup;
static struct fb_fix_screeninfo fbi_fix_backup;
static BOOL need_restore = FALSE;
static int mtkfb_set_par(struct fb_info *fbi);
void mtkfb_switch_normal_to_factory(void)
{
	if (down_interruptible(&sem_flipping)) {
		MTKFB_WRAN("[FB Driver] can't get semaphore:%d\n", __LINE__);
		return;
	}
	sem_flipping_cnt--;
	if (down_interruptible(&sem_early_suspend)) {
		MTKFB_WRAN("[FB Driver] can't get semaphore:%d\n", __LINE__);
		sem_flipping_cnt++;
		up(&sem_flipping);
		return;
	}
	sem_early_suspend_cnt--;
	if (is_early_suspended)
		goto EXIT;

	if (mtkfb_fbi) {
		memcpy(&fbi_var_backup, &mtkfb_fbi->var, sizeof(fbi_var_backup));
		memcpy(&fbi_fix_backup, &mtkfb_fbi->fix, sizeof(fbi_fix_backup));
		need_restore = TRUE;
	}

EXIT:
	sem_early_suspend_cnt++;
	sem_flipping_cnt++;
	up(&sem_early_suspend);
	up(&sem_flipping);
}

/* Used for HQA test */
void mtkfb_switch_factory_to_normal(void)
{
	BOOL need_set_par = FALSE;
	if (down_interruptible(&sem_flipping)) {
		MTKFB_WRAN("[FB Driver] can't get semaphore in mtkfb_switch_factory_to_normal()\n");
		return;
	}
	sem_flipping_cnt--;
	if (down_interruptible(&sem_early_suspend)) {
		MTKFB_WRAN("[FB Driver] can't get semaphore in mtkfb_switch_factory_to_normal()\n");
		sem_flipping_cnt++;
		up(&sem_flipping);
		return;
	}

	sem_early_suspend_cnt--;
	if (is_early_suspended)
		goto EXIT;

	if ((mtkfb_fbi) && (need_restore)) {
		memcpy(&mtkfb_fbi->var, &fbi_var_backup, sizeof(fbi_var_backup));
		memcpy(&mtkfb_fbi->fix, &fbi_fix_backup, sizeof(fbi_fix_backup));
		need_restore = FALSE;
		need_set_par = TRUE;
	}

EXIT:
	sem_early_suspend_cnt++;
	sem_flipping_cnt++;
	up(&sem_early_suspend);
	up(&sem_flipping);
	if (need_set_par) {
		int ret;
		ret = mtkfb_set_par(mtkfb_fbi);
		if (ret != 0)
			MTKFB_WRAN("failed to mtkfb_set_par\n");
	}
}

static bool first_update = true;
static bool first_enable_esd = true;
static bool no_update;
static int cnt = 3;
static int mtkfb_pan_display_impl(struct fb_var_screeninfo *var, struct fb_info *info)
{
	UINT32 offset;
	UINT32 paStart;
	char *vaStart, *vaEnd;
	int ret = 0;
	int wait_ret = 0;
	int i;
	disp_job *job;
	if (first_update && no_update) {
		first_update = false;
		return ret;
	}
	MMProfileLog(MTKFB_MMP_Events.PanDisplay, MMProfileFlagStart);
	if (0 != cnt) {
		MTKFB_WRAN("LCD:%dx%d\n", MTK_FB_XRES, MTK_FB_YRES);
		cnt--;
	}

	MSG(ARGU, "xoffset=%u, yoffset=%u, xres=%u, yres=%u, xresv=%u, yresv=%u\n",
	    var->xoffset, var->yoffset,
	    info->var.xres, info->var.yres,
	    info->var.xres_virtual,
	    info->var.yres_virtual);

	if (down_interruptible(&sem_flipping)) {
		MTKFB_WRAN("[FB Driver] can't get semaphore in mtkfb_pan_display_impl()\n");
		MMProfileLogMetaString(MTKFB_MMP_Events.PanDisplay, MMProfileFlagEnd, "Can't get semaphore in mtkfb_pan_display_impl()");
		return -ERESTARTSYS;
	}
	sem_flipping_cnt--;

	info->var.yoffset = var->yoffset;

	offset = var->yoffset * info->fix.line_length;
	paStart = fb_pa + offset;
	vaStart = info->screen_base + offset;
	vaEnd = vaStart + info->var.yres * info->fix.line_length;

	job = disp_deque_job(disp_config.session_id);
	mutex_lock(&job->lock);
	for (i = 0; i < HW_OVERLAY_COUNT; i++) {
		if (isAEEEnabled && i == ASSERT_LAYER) {
			job->input[i].dirty = 0;
		} else {
			job->input[i].layer_id = i;
			job->input[i].layer_enable = 0;
			job->input[i].dirty = 1;
		}
	}
	job->input[FB_LAYER].layer_id = FB_LAYER;
	job->input[FB_LAYER].address = paStart;
	/* job->input[layer].vaddr = (unsigned int)vaStart; */
	job->input[FB_LAYER].layer_enable = 1;
	job->input[FB_LAYER].dirty = 1;
	job->input[FB_LAYER].width = var->xres;
	job->input[FB_LAYER].height = var->yres;
	{
		unsigned int layerpitch;
		unsigned int src_pitch = ALIGN_TO(var->xres, disphal_get_fb_alignment());
		switch (var->bits_per_pixel) {
		case 16:
			job->input[FB_LAYER].format = eRGB565;
			layerpitch = 2;
			job->input[FB_LAYER].alpha_enable = FALSE;
			break;
		case 24:
			job->input[FB_LAYER].format = eRGB888;
			layerpitch = 3;
			job->input[FB_LAYER].alpha_enable = FALSE;
			break;
		case 32:
			job->input[FB_LAYER].format = (0 == var->blue.offset) ? ePARGB8888 : ePABGR8888;
			layerpitch = 4;
			job->input[FB_LAYER].alpha_enable = TRUE;
			break;
		default:
			MTKFB_WRAN("Invalid color format bpp: 0x%d\n", var->bits_per_pixel);
			job->input[FB_LAYER].dirty = 0;
			mutex_unlock(&job->lock);
			MMProfileLogEx(MTKFB_MMP_Events.PanDisplay, MMProfileFlagEnd, 0, var->bits_per_pixel);
			return -1;
		}
		job->input[FB_LAYER].alpha = 0xFF;
		job->input[FB_LAYER].pitch = src_pitch * layerpitch;
	}
	mutex_unlock(&job->lock);

	MMProfileLogStructure(MTKFB_MMP_Events.PanDisplay, MMProfileFlagPulse, &job->input[FB_LAYER], input_config);
	PanDispSettingPending = 1;
	PanDispSettingDirty = 1;
	PanDispSettingApplied = 0;

	ret = disp_enque_job(disp_config.session_id);
	is_ipoh_bootup = false;

	if (DISP_IsDecoupleMode())
		DISP_StartOverlayTransfer();
	else
		ret = mtkfb_update_screen(info);

	/* NOTICE: un-interruptible wait here for m4u callback */
	wait_ret = wait_event_timeout(reg_update_wq, PanDispSettingApplied, HZ / 10);
	MTKFB_INFO("[WaitQ] wait_event_interruptible() ret = %d, %d\n", wait_ret, __LINE__);

	sem_flipping_cnt++;
	up(&sem_flipping);
	if (first_enable_esd) {
		esd_recovery_pause(FALSE);
		first_enable_esd = false;
	}
	MMProfileLog(MTKFB_MMP_Events.PanDisplay, MMProfileFlagEnd);
#ifdef CONFIG_MTK_AEE_POWERKEY_HANG_DETECT
	screen_update_cnt++;
#endif

	return ret;
}

static int mtkfb_pan_display_proxy(struct fb_var_screeninfo *var, struct fb_info *info)
{
#ifdef CONFIG_MTPROF_APPLAUNCH	/* eng enable, user disable */
	DISP_LOG_PRINT("[AppLaunch] mtkfb_pan_display_proxy.\n");
#endif
	return mtkfb_pan_display_impl(var, info);
}

/* Set fb_info.fix fields and also updates fbdev.
 * When calling this fb_info.var must be set up already.
 */
static void set_fb_fix(struct mtkfb_device *fbdev)
{
	struct fb_info *fbi = fbdev->fb_info;
	struct fb_fix_screeninfo *fix = &fbi->fix;
	struct fb_var_screeninfo *var = &fbi->var;
	struct fb_ops *fbops = fbi->fbops;

	strncpy(fix->id, MTKFB_DRIVER, sizeof(fix->id));
	fix->type = FB_TYPE_PACKED_PIXELS;

	switch (var->bits_per_pixel) {
	case 16:
	case 24:
	case 32:
		fix->visual = FB_VISUAL_TRUECOLOR;
		break;
	case 1:
	case 2:
	case 4:
	case 8:
		fix->visual = FB_VISUAL_PSEUDOCOLOR;
		break;
	default:
		ASSERT(0);
	}

	fix->accel = FB_ACCEL_NONE;
	fix->line_length = ALIGN_TO(var->xres_virtual, disphal_get_fb_alignment()) * var->bits_per_pixel / 8;
	fix->smem_len = fbdev->fb_size_in_byte;
	fix->smem_start = fbdev->fb_pa_base;

	fix->xpanstep = 0;
	fix->ypanstep = 1;

	fbops->fb_fillrect = cfb_fillrect;
	fbops->fb_copyarea = cfb_copyarea;
	fbops->fb_imageblit = cfb_imageblit;
}

/* Check values in var, try to adjust them in case of out of bound values if
 * possible, or return error.
 */
static int mtkfb_check_var(struct fb_var_screeninfo *var, struct fb_info *fbi)
{
	unsigned int bpp;
	unsigned long max_frame_size;
	unsigned long line_size;

	struct mtkfb_device *fbdev = (struct mtkfb_device *)fbi->par;

	MSG_FUNC_ENTER();

	MSG(ARGU, "xres=%u, yres=%u, xres_virtual=%u, yres_virtual=%u, xoffset=%u, yoffset=%u, bits_per_pixel=%u)\n",
	    var->xres, var->yres, var->xres_virtual, var->yres_virtual,
	    var->xoffset, var->yoffset, var->bits_per_pixel);

	bpp = var->bits_per_pixel;

	if (bpp != 16 && bpp != 24 && bpp != 32) {
		MTKFB_WRAN("[%s]unsupported bpp: %d", __func__, bpp);
		return -1;
	}

	switch (var->rotate) {
	case 0:
	case 180:
		var->xres = MTK_FB_XRES;
		var->yres = MTK_FB_YRES;
		break;
	case 90:
	case 270:
		var->xres = MTK_FB_YRES;
		var->yres = MTK_FB_XRES;
		break;
	default:
		return -1;
	}

	if (var->xres_virtual < var->xres)
		var->xres_virtual = var->xres;
	if (var->yres_virtual < var->yres)
		var->yres_virtual = var->yres;

	max_frame_size = fbdev->fb_size_in_byte;
	line_size = var->xres_virtual * bpp / 8;

	if (line_size * var->yres_virtual > max_frame_size) {
		/* Try to keep yres_virtual first */
		line_size = max_frame_size / var->yres_virtual;
		var->xres_virtual = line_size * 8 / bpp;
		if (var->xres_virtual < var->xres) {
			/* Still doesn't fit. Shrink yres_virtual too */
			var->xres_virtual = var->xres;
			line_size = var->xres * bpp / 8;
			var->yres_virtual = max_frame_size / line_size;
		}
	}
	if (var->xres + var->xoffset > var->xres_virtual)
		var->xoffset = var->xres_virtual - var->xres;
	if (var->yres + var->yoffset > var->yres_virtual)
		var->yoffset = var->yres_virtual - var->yres;

	if (16 == bpp) {
		var->red.offset = 11;
		var->red.length = 5;
		var->green.offset = 5;
		var->green.length = 6;
		var->blue.offset = 0;
		var->blue.length = 5;
		var->transp.offset = 0;
		var->transp.length = 0;
	} else if (24 == bpp) {
		var->red.length = var->green.length = var->blue.length = 8;
		var->transp.length = 0;

		/* Check if format is RGB565 or BGR565 */

		ASSERT(8 == var->green.offset);
		ASSERT(16 == var->red.offset + var->blue.offset);
		ASSERT(16 == var->red.offset || 0 == var->red.offset);
	} else if (32 == bpp) {
		var->red.length = var->green.length =
			var->blue.length = var->transp.length = 8;

		/* Check if format is ARGB565 or ABGR565 */

		ASSERT(8 == var->green.offset && 24 == var->transp.offset);
		ASSERT(16 == var->red.offset + var->blue.offset);
		ASSERT(16 == var->red.offset || 0 == var->red.offset);
	}

	var->red.msb_right = var->green.msb_right =
		var->blue.msb_right = var->transp.msb_right = 0;

	if (var->activate & FB_ACTIVATE_NO_UPDATE)
		no_update = true;
	else
		no_update = false;

	var->activate = FB_ACTIVATE_NOW;

	var->height = DISP_GetPhysicalHeight();
	var->width = DISP_GetPhysicalWidth();
	var->grayscale = 0;
	var->nonstd = 0;

	var->pixclock = UINT_MAX;
	var->left_margin = UINT_MAX;
	var->right_margin = UINT_MAX;
	var->upper_margin = UINT_MAX;
	var->lower_margin = UINT_MAX;
	var->hsync_len = UINT_MAX;
	var->vsync_len = UINT_MAX;

	var->vmode = FB_VMODE_NONINTERLACED;
	var->sync = 0;

	MSG_FUNC_LEAVE();
	return 0;
}

/* Switch to a new mode. The parameters for it has been check already by
 * mtkfb_check_var.
 */
static int mtkfb_set_par(struct fb_info *fbi)
{
	struct fb_var_screeninfo *var = &fbi->var;
	struct mtkfb_device *fbdev = (struct mtkfb_device *)fbi->par;
	struct fb_overlay_layer fb_layer;
	u32 bpp = var->bits_per_pixel;

	MSG_FUNC_ENTER();
	/* No need for IPO-H reboot, or white screen flash will happen */
	if (is_ipoh_bootup && (lcm_params->type == LCM_TYPE_DSI && lcm_params->dsi.mode != CMD_MODE)) {
		MTKFB_WRAN("mtkfb_set_par return in IPOH!!!\n");
		goto Done;
	}
	memset(&fb_layer, 0, sizeof(struct fb_overlay_layer));
	switch (bpp) {
	case 16:
		fb_layer.src_fmt = MTK_FB_FORMAT_RGB565;
		fb_layer.src_use_color_key = 1;
		fb_layer.src_color_key = 0xFF000000;
		break;

	case 24:
		fb_layer.src_use_color_key = 1;
		fb_layer.src_fmt = (0 == var->blue.offset) ?
				MTK_FB_FORMAT_RGB888 :
				MTK_FB_FORMAT_BGR888;
		fb_layer.src_color_key = 0xFF000000;
		break;

	case 32:
		fb_layer.src_use_color_key = 0;
		fb_layer.src_fmt = (0 == var->blue.offset) ?
				MTK_FB_FORMAT_ARGB8888 :
				MTK_FB_FORMAT_ABGR8888;
		fb_layer.src_color_key = 0;
		break;

	default:
		fb_layer.src_fmt = MTK_FB_FORMAT_UNKNOWN;
		MTKFB_WRAN("[%s]unsupported bpp: %d", __func__, bpp);
		return -1;
	}

	/* If the framebuffer format is NOT changed, nothing to do */
	if (fb_layer.src_fmt == fbdev->layer_format[FB_LAYER])
		goto Done;

	/* else, begin change display mode */
	set_fb_fix(fbdev);

	fb_layer.layer_id = FB_LAYER;
	fb_layer.layer_enable = 1;
	fb_layer.src_base_addr = (void *)((unsigned long)fbdev->fb_va_base + var->yoffset * fbi->fix.line_length);
	fb_layer.src_phy_addr = (void *)(fb_pa + var->yoffset * fbi->fix.line_length);
	fb_layer.src_direct_link = 0;
	fb_layer.src_offset_x = fb_layer.src_offset_y = 0;
	/* fb_layer.src_width = fb_layer.tgt_width = fb_layer.src_pitch = var->xres; */
#if defined(HWGPU_SUPPORT)
	fb_layer.src_pitch = ALIGN_TO(var->xres, MTK_FB_ALIGNMENT);
#else
	if (get_boot_mode() == META_BOOT || get_boot_mode() == FACTORY_BOOT ||
	    get_boot_mode() == ADVMETA_BOOT || get_boot_mode() == RECOVERY_BOOT)
		fb_layer.src_pitch = ALIGN_TO(var->xres, MTK_FB_ALIGNMENT);
	else
		fb_layer.src_pitch = var->xres;
#endif
	fb_layer.src_width = fb_layer.tgt_width = var->xres;
	fb_layer.src_height = fb_layer.tgt_height = var->yres;
	fb_layer.tgt_offset_x = fb_layer.tgt_offset_y = 0;

	/* fb_layer.src_color_key = 0; */
	fb_layer.layer_rotation = MTK_FB_ORIENTATION_0;
	fb_layer.layer_type = LAYER_2D;

	if (!no_update)
		mtkfb_set_overlay_layer(fbi, &fb_layer, false);

	/* backup fb_layer information. */
	memcpy(&fb_layer_context, &fb_layer, sizeof(fb_layer));

Done:
	MSG_FUNC_LEAVE();
	return 0;
}

static int mtkfb_soft_cursor(struct fb_info *info, struct fb_cursor *cursor)
{
	NOT_REFERENCED(info);
	NOT_REFERENCED(cursor);

	return 0;
}

extern DAL_STATUS DAL_Dynamic_Change_FB_Layer(unsigned int isAEEEnabled);
static int mtkfb_set_overlay_layer(struct fb_info *info, struct fb_overlay_layer *layerInfo, unsigned int locked)
{
	struct mtkfb_device *fbdev = (struct mtkfb_device *)info->par;

	unsigned int layerpitch;
	unsigned int layerbpp;
	unsigned int id = (layerInfo->layer_id >= OVL_LAYER_NUM ? 0 : layerInfo->layer_id);
	int enable = layerInfo->layer_enable ? 1 : 0;
	int ret = 0;
	disp_job *job = disp_deque_job(disp_config.session_id);
	mutex_lock(&job->lock);
	job->input[id].layer_id = id;

	MMProfileLogEx(MTKFB_MMP_Events.SetOverlayLayer, MMProfileFlagStart, (id << 24) | (enable << 16) | layerInfo->next_buff_idx, (unsigned int)layerInfo->src_phy_addr);

	MTKFB_INFO("L%d set_overlay:%d,%d\n", layerInfo->layer_id, layerInfo->layer_enable, layerInfo->next_buff_idx);

	/* Update Layer Enable Bits and Layer Config Dirty Bits */
	if ((((fbdev->layer_enable >> id) & 1) ^ enable)) {
		fbdev->layer_enable ^= (1 << id);
		fbdev->layer_config_dirty |= MTKFB_LAYER_ENABLE_DIRTY;
	}

	/* Update Layer Format and Layer Config Dirty Bits */
	if (fbdev->layer_format[id] != layerInfo->src_fmt) {
		fbdev->layer_format[id] = layerInfo->src_fmt;
		fbdev->layer_config_dirty |= MTKFB_LAYER_FORMAT_DIRTY;
	}

	if (!enable) {
		job->input[id].layer_enable = enable;
		job->input[id].dirty = true;
		ret = 0;
		goto LeaveOverlayMode;
	}

	switch (layerInfo->src_fmt) {
	case MTK_FB_FORMAT_YUV422:
		job->input[id].format = eYUY2;
		layerpitch = 2;
		layerbpp = 24;
		break;

	case MTK_FB_FORMAT_RGB565:
		job->input[id].format = eRGB565;
		layerpitch = 2;
		layerbpp = 16;
		break;

	case MTK_FB_FORMAT_RGB888:
		job->input[id].format = eRGB888;
		layerpitch = 3;
		layerbpp = 24;
		break;
	case MTK_FB_FORMAT_BGR888:
		job->input[id].format = eBGR888;
		layerpitch = 3;
		layerbpp = 24;
		break;

	case MTK_FB_FORMAT_ARGB8888:
		job->input[id].format = ePARGB8888;
		layerpitch = 4;
		layerbpp = 32;
		break;
	case MTK_FB_FORMAT_ABGR8888:
		job->input[id].format = ePABGR8888;
		layerpitch = 4;
		layerbpp = 32;
		break;
	case MTK_FB_FORMAT_XRGB8888:
		job->input[id].format = eARGB8888;
		layerpitch = 4;
		layerbpp = 32;
		break;
	case MTK_FB_FORMAT_XBGR8888:
		job->input[id].format = eABGR8888;
		layerpitch = 4;
		layerbpp = 32;
		break;
	case MTK_FB_FORMAT_UYVY:
		job->input[id].format = eUYVY;
		layerpitch = 2;
		layerbpp = 16;
		break;
	default:
		MTKFB_WRAN("Invalid color format: 0x%x\n", layerInfo->src_fmt);
		ret = -EFAULT;
		goto LeaveOverlayMode;
	}
	job->input[id].security = layerInfo->security;
#if defined(MTK_FB_SYNC_SUPPORT)
	if (layerInfo->src_phy_addr != NULL)
		job->input[id].address = (unsigned int)layerInfo->src_phy_addr;
	else
		job->input[id].address = disp_sync_query_buffer_mva(job->group_id, layerInfo->layer_id, (unsigned int)layerInfo->next_buff_idx);
#else
	job->input[id].address = (unsigned int)layerInfo->src_phy_addr;
#endif
	job->input[id].index = layerInfo->next_buff_idx;

	/* set Alpha blending */
	if (layerInfo->alpha_enable) {
		job->input[id].alpha_enable = TRUE;
		job->input[id].alpha = layerInfo->alpha;
	} else {
		job->input[id].alpha_enable = FALSE;
	}
	if (MTK_FB_FORMAT_ARGB8888 == layerInfo->src_fmt ||
	    MTK_FB_FORMAT_ABGR8888 == layerInfo->src_fmt) {
		job->input[id].alpha_enable = TRUE;
		job->input[id].alpha = 0xff;
	}

	/* xuecheng, for slt debug */
	if (!strcmp(current->comm, "display_slt")) {
		job->input[id].alpha_enable = FALSE;
		isAEEEnabled = 1;
		DAL_Dynamic_Change_FB_Layer(isAEEEnabled);	/* default_ui_ layer coniig to changed_ui_layer */
	}

	/* set src width, src height */
	job->input[id].src_x = layerInfo->src_offset_x;
	job->input[id].src_y = layerInfo->src_offset_y;
	job->input[id].dst_x = layerInfo->tgt_offset_x;
	job->input[id].dst_y = layerInfo->tgt_offset_y;

	if (layerInfo->src_width != layerInfo->tgt_width || layerInfo->src_height != layerInfo->tgt_height)
		MTKFB_WRAN("OVL cannot support clip:src(%d,%d), dst(%d,%d)\n", layerInfo->src_width, layerInfo->src_height, layerInfo->tgt_width, layerInfo->tgt_height);
	job->input[id].width = layerInfo->tgt_width;
	job->input[id].height = layerInfo->tgt_height;
	job->input[id].pitch = layerInfo->src_pitch * layerpitch;

#if defined(CONFIG_MTK_LCM_PHYSICAL_ROTATION)
	if (0 == strncmp(CONFIG_MTK_LCM_PHYSICAL_ROTATION, "180", 3)) {
		layerInfo->layer_rotation = (layerInfo->layer_rotation + MTK_FB_ORIENTATION_180) % 4;
		layerInfo->tgt_offset_x = MTK_FB_XRES - (layerInfo->tgt_offset_x + layerInfo->tgt_width);
		layerInfo->tgt_offset_y = MTK_FB_YRES - (layerInfo->tgt_offset_y + layerInfo->tgt_height);
	}
#endif

	video_rotation = layerInfo->video_rotation;

	/* set color key */
	job->input[id].color_key = layerInfo->src_color_key;
	job->input[id].color_key_enable = layerInfo->src_use_color_key;

	job->input[id].layer_enable = enable;
	job->input[id].dirty = TRUE;

LeaveOverlayMode:
	/* Lock/unlock same as RCU to reduce R/W conflicting time */
	mutex_unlock(&job->lock);

	MSG_FUNC_LEAVE();
	MMProfileLog(MTKFB_MMP_Events.SetOverlayLayer, MMProfileFlagEnd);

	return ret;
}

static int mtkfb_get_overlay_layer_info(struct fb_overlay_layer_info *layerInfo)
{
	DISP_LAYER_INFO layer;

	if (layerInfo->layer_id >= DDP_OVL_LAYER_MUN)
		return 0;

	layer.id = layerInfo->layer_id;
	DISP_GetLayerInfo(&layer);
	layerInfo->layer_enabled = layer.hw_en;
	layerInfo->curr_en = layer.curr_en;
	layerInfo->next_en = layer.next_en;
	layerInfo->hw_en = layer.hw_en;
	layerInfo->curr_idx = layer.curr_idx;
	layerInfo->next_idx = layer.next_idx;
	layerInfo->hw_idx = layer.hw_idx;
	layerInfo->curr_identity = layer.curr_identity;
	layerInfo->next_identity = layer.next_identity;
	layerInfo->hw_identity = layer.hw_identity;
	layerInfo->curr_conn_type = layer.curr_conn_type;
	layerInfo->next_conn_type = layer.next_conn_type;
	layerInfo->hw_conn_type = layer.hw_conn_type;

#if defined(CONFIG_MTK_LCM_PHYSICAL_ROTATION)
	if (0 == strncmp(CONFIG_MTK_LCM_PHYSICAL_ROTATION, "180", 3)) {
		layerInfo->layer_rotation = MTK_FB_ORIENTATION_180;
	} else if (0 == strncmp(CONFIG_MTK_LCM_PHYSICAL_ROTATION, "90", 2)) {
		layerInfo->layer_rotation = MTK_FB_ORIENTATION_90;
	} else if (0 == strncmp(CONFIG_MTK_LCM_PHYSICAL_ROTATION, "270", 3)) {
		layerInfo->layer_rotation = MTK_FB_ORIENTATION_270;
	} else
#endif
	{
		layerInfo->layer_rotation = MTK_FB_ORIENTATION_0;
	}
#if 0
	MTKFB_INFO("[FB Driver] mtkfb_get_overlay_layer_info():id=%u, layer en=%u, next_en=%u, curr_en=%u, hw_en=%u, next_idx=%u, curr_idx=%u, hw_idx=%u\n",
		   layerInfo->layer_id,
		   layerInfo->layer_enabled,
		   layerInfo->next_en,
		   layerInfo->curr_en,
		   layerInfo->hw_en,
		   layerInfo->next_idx,
		   layerInfo->curr_idx,
		   layerInfo->hw_idx);
#endif
	MMProfileLogEx(MTKFB_MMP_Events.LayerInfo[layerInfo->layer_id], MMProfileFlagPulse, (layerInfo->next_idx << 16) + ((layerInfo->curr_idx) & 0xFFFF),
		       (layerInfo->hw_idx << 16) + (layerInfo->next_en << 8) + (layerInfo->curr_en << 4) + layerInfo->hw_en);
	return 0;
}

static atomic_t capture_ui_layer_only = ATOMIC_INIT(0);	/* when capturing framebuffer ,whether capture ui layer only */
void mtkfb_capture_fb_only(bool enable)
{
	atomic_set(&capture_ui_layer_only, enable);
}

#include <mt-plat/aee.h>
#define mtkfb_aee_print(string, args...) do { \
	aee_kernel_warning_api(__FILE__, __LINE__, DB_OPT_MMPROFILE_BUFFER, "sf-mtkfb blocked", string, ##args); \
} while (0)

void mtkfb_dump_layer_info(void)
{
	unsigned int i;
	MTKFB_INFO("[mtkfb] start dump layer info, early_suspend=%d\n", is_early_suspended);
	MTKFB_INFO("[mtkfb] cache(next):\n");
	for (i = 0; i < 4; i++) {
		MTKFB_INFO("[mtkfb] layer=%d, layer_en=%d, idx=%d, fmt=%d, addr=0x%x, %d, %d, %d\n",
			   cached_layer_config[i].layer,	/* layer */
			   cached_layer_config[i].layer_en,
			   cached_layer_config[i].buff_idx,
			   cached_layer_config[i].fmt,
			   cached_layer_config[i].addr,	/* addr */
			   cached_layer_config[i].identity,
			   cached_layer_config[i].connected_type,
			   cached_layer_config[i].security);
	}
}

mtk_dispif_info_t dispif_info[MTKFB_MAX_DISPLAY_COUNT];

static int mtkfb_ioctl(struct fb_info *info, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	DISP_STATUS ret = 0;
	int r = 0;

	/* M: dump debug mmprofile log info */
	MMProfileLogEx(MTKFB_MMP_Events.IOCtrl, MMProfileFlagPulse, _IOC_NR(cmd), arg);
	/* MTKFB_INFO("mtkfb_ioctl, info=0x%08x, cmd=0x%08x, arg=0x%08x\n", (unsigned int)info, (unsigned int)cmd, (unsigned int)arg); */

	switch (cmd) {
	case MTKFB_GET_FRAMEBUFFER_MVA:
		return copy_to_user(argp, &fb_pa, sizeof(fb_pa)) ? -EFAULT : 0;

	case MTKFB_GET_DISPLAY_IF_INFORMATION:
	{
		int displayid = 0;
		if (copy_from_user(&displayid, (void __user *)arg, sizeof(displayid))) {
			MTKFB_WRAN("[FB]: copy_from_user failed! line:%d\n", __LINE__);
			return -EFAULT;
		}
		MTKFB_INFO("%s, display_id=%d\n", __func__, displayid);
		if ((displayid < 0) || (displayid >= MTKFB_MAX_DISPLAY_COUNT)) {
			MTKFB_WRAN("[FB]: invalid display id:%d\n", displayid);
			return -EFAULT;
		}
		dispif_info[displayid].physicalHeight = DISP_GetPhysicalHeight();
		dispif_info[displayid].physicalWidth = DISP_GetPhysicalWidth();
		if (copy_to_user((void __user *)arg, &(dispif_info[displayid]), sizeof(mtk_dispif_info_t))) {
			MTKFB_WRAN("[FB]: copy_to_user failed! line:%d\n", __LINE__);
			r = -EFAULT;
		}
		return r;
	}

	case MTKFB_POWEROFF:
		if (is_early_suspended)
			return r;
		mtkfb_early_suspend();
		return r;

	case MTKFB_POWERON:
		if (!is_early_suspended)
			return r;
		mtkfb_late_resume();
		DISP_WaitVSYNC();
		mt65xx_leds_brightness_set(MT65XX_LED_TYPE_LCD, LED_HALF);
		return r;

	case MTKFB_GET_POWERSTATE:
	{
		unsigned long power_state;

		if (is_early_suspended == TRUE)
			power_state = 0;
		else
			power_state = 1;

		return copy_to_user(argp, &power_state, sizeof(power_state)) ? -EFAULT : 0;
	}

	case MTKFB_CONFIG_IMMEDIATE_UPDATE:
		MTKFB_INFO("[%s] MTKFB_CONFIG_IMMEDIATE_UPDATE, enable = %lu\n", __func__, arg);
		if (down_interruptible(&sem_early_suspend)) {
			MTKFB_WRAN("[mtkfb_ioctl] can't get semaphore:%d\n", __LINE__);
			return -ERESTARTSYS;
		}
		sem_early_suspend_cnt--;
		DISP_WaitForLCDNotBusy();
		ret = DISP_ConfigImmediateUpdate((BOOL) arg);
		sem_early_suspend_cnt++;
		up(&sem_early_suspend);
		return r;

	case MTKFB_GET_OVERLAY_LAYER_INFO:
	{
		struct fb_overlay_layer_info layerInfo;
		MTKFB_INFO(" mtkfb_ioctl():MTKFB_GET_OVERLAY_LAYER_INFO\n");

		if (copy_from_user(&layerInfo, (void __user *)arg, sizeof(layerInfo))) {
			MTKFB_WRAN("[FB]: copy_from_user failed! line:%d\n", __LINE__);
			return -EFAULT;
		}
		if (mtkfb_get_overlay_layer_info(&layerInfo) < 0) {
			MTKFB_WRAN("[FB]: Failed to get overlay layer info\n");
			return -EFAULT;
		}
		if (copy_to_user((void __user *)arg, &layerInfo, sizeof(layerInfo))) {
			MTKFB_WRAN("[FB]: copy_to_user failed! line:%d\n", __LINE__);
			r = -EFAULT;
		}
		return r;
	}

	case MTKFB_SET_OVERLAY_LAYER:
	{
		struct fb_overlay_layer layerInfo;
		memset((void*)&layerInfo, 0, sizeof(layerInfo));
		MTKFB_INFO(" mtkfb_ioctl():MTKFB_SET_OVERLAY_LAYER\n");

		if (copy_from_user(&layerInfo, (void __user *)arg, sizeof(layerInfo))) {
			MTKFB_WRAN("[FB]: copy_from_user failed! line:%d\n", __LINE__);
			r = -EFAULT;
		} else {
			/* in early suspend mode ,will not update buffer index, info SF by return value */
			if (is_early_suspended == TRUE) {
				MTKFB_WRAN("[FB] error, set overlay in early suspend ,skip!\n");
				return MTKFB_ERROR_IS_EARLY_SUSPEND;
			}

			mtkfb_set_overlay_layer(info, &layerInfo, false);
			disp_enque_job(disp_config.session_id);
			if (DISP_IsDecoupleMode())
				DISP_StartOverlayTransfer();
			if (is_ipoh_bootup) {
				int i;
				for (i = 0; i < DDP_OVL_LAYER_MUN; i++)
					cached_layer_config[i].isDirty = 1;
				is_ipoh_bootup = false;
			}
#ifdef CONFIG_MTK_AEE_POWERKEY_HANG_DETECT
			screen_update_cnt++;
#endif
		}
		return r;
	}

	case MTKFB_ERROR_INDEX_UPDATE_TIMEOUT:
		MTKFB_INFO("[DDP] mtkfb_ioctl():MTKFB_ERROR_INDEX_UPDATE_TIMEOUT\n");
		/* call info dump function here */
		mtkfb_dump_layer_info();
		return r;

	case MTKFB_ERROR_INDEX_UPDATE_TIMEOUT_AEE:
		MTKFB_INFO("[DDP] mtkfb_ioctl():MTKFB_ERROR_INDEX_UPDATE_TIMEOUT\n");
		/* call info dump function here */
		mtkfb_dump_layer_info();
		mtkfb_aee_print("surfaceflinger-mtkfb blocked");
		return r;

	case MTKFB_SET_VIDEO_LAYERS:
	{
		struct mmp_fb_overlay_layers {
			struct fb_overlay_layer Layer0;
			struct fb_overlay_layer Layer1;
			struct fb_overlay_layer Layer2;
			struct fb_overlay_layer Layer3;
		};
		struct fb_overlay_layer layerInfo[VIDEO_LAYER_COUNT];

		MTKFB_INFO(" mtkfb_ioctl():MTKFB_SET_VIDEO_LAYERS\n");
		MMProfileLog(MTKFB_MMP_Events.SetVideoLayers, MMProfileFlagStart);
		if (copy_from_user(&layerInfo, (void __user *)arg, sizeof(layerInfo))) {
			MTKFB_WRAN("[FB]: copy_from_user failed! line:%d\n", __LINE__);
			MMProfileLogMetaString(MTKFB_MMP_Events.SetVideoLayers, MMProfileFlagEnd, "Copy_from_user failed!");
			r = -EFAULT;
		} else {
			int32_t i;

			for (i = 0; i < VIDEO_LAYER_COUNT; ++i)
				mtkfb_set_overlay_layer(info, &layerInfo[i], true);
			disp_enque_job(disp_config.session_id);
			if (DISP_IsDecoupleMode())
				DISP_StartOverlayTransfer();
			is_ipoh_bootup = false;
			MMProfileLogStructure(MTKFB_MMP_Events.SetVideoLayers, MMProfileFlagEnd, layerInfo, struct mmp_fb_overlay_layers);
		}
#ifdef CONFIG_MTK_AEE_POWERKEY_HANG_DETECT
		screen_update_cnt++;
#endif

		return r;
	}

	case MTKFB_SET_MULTIPLE_LAYERS:
	{
		struct mmp_fb_overlay_layers {
			struct fb_overlay_layer Layer0;
			struct fb_overlay_layer Layer1;
			struct fb_overlay_layer Layer2;
			struct fb_overlay_layer Layer3;
		};
		struct fb_overlay_layer layerInfo[HW_OVERLAY_COUNT];

		MTKFB_INFO(" mtkfb_ioctl():MTKFB_SET_MULTIPLE_LAYERS\n");
		MMProfileLog(MTKFB_MMP_Events.SetMultipleLayers, MMProfileFlagStart);

		if (copy_from_user(&layerInfo, (void __user *)arg, sizeof(layerInfo))) {
			MTKFB_WRAN("[FB]: copy_from_user failed! line:%d\n", __LINE__);
			MMProfileLogMetaString(MTKFB_MMP_Events.SetMultipleLayers, MMProfileFlagEnd, "Copy_from_user failed!");
			r = -EFAULT;
		} else {
			int32_t i, layerId;

			for (i = 0; i < HW_OVERLAY_COUNT; ++i) {
				layerId = layerInfo[i].layer_id;
				if (layerInfo[i].layer_id >= HW_OVERLAY_COUNT)
					continue;
				if (is_early_suspended) {
					MTKFB_WRAN("in early suspend layer(0x%x),idx(%d)!\n", layerId << 16 | layerInfo[i].layer_enable, layerInfo[i].next_buff_idx);
					/* mtkfb_release_layer_fence(layerInfo[i].layer_id); */
					disp_sync_release(disp_config.session_id, layerInfo[i].layer_id);
				} else {
					mtkfb_set_overlay_layer(info, &layerInfo[i], true);
				}
			}
			disp_enque_job(disp_config.session_id);

			if (DISP_IsDecoupleMode())
				DISP_StartOverlayTransfer();
			is_ipoh_bootup = false;
			MMProfileLogStructure(MTKFB_MMP_Events.SetMultipleLayers, MMProfileFlagEnd, layerInfo, struct mmp_fb_overlay_layers);
#ifdef CONFIG_MTK_AEE_POWERKEY_HANG_DETECT
			screen_update_cnt++;
#endif
		}

		return r;
	}

	case MTKFB_TRIG_OVERLAY_OUT:
		MTKFB_INFO(" mtkfb_ioctl():MTKFB_TRIG_OVERLAY_OUT\n");
		MMProfileLog(MTKFB_MMP_Events.TrigOverlayOut, MMProfileFlagPulse);
		return mtkfb_update_screen(info);

#if defined(MTK_FB_SYNC_SUPPORT)
	case MTKFB_PREPARE_OVERLAY_BUFFER:
	{
		struct fb_overlay_buffer overlay_buffer;

		if (copy_from_user(&overlay_buffer, (void __user *)arg, sizeof(overlay_buffer))) {
			MTKFB_WRAN("[FB Driver]: copy_from_user failed! line:%d\n", __LINE__);
			r = -EFAULT;
		} else {
			if (overlay_buffer.layer_en) {
				if (disp_sync_prepare_buffer((struct disp_buffer_info_t *)&overlay_buffer) != SYNC_STATUS_OK) {
					overlay_buffer.fence_fd = DISP_INVALID_FENCE_FD;	/* invalid fd */
					overlay_buffer.index = 0;
					r = -EFAULT;
				}
			} else {
				overlay_buffer.fence_fd = DISP_INVALID_FENCE_FD;	/* invalid fd */
				overlay_buffer.index = 0;
			}
			if (copy_to_user((void __user *)arg, &overlay_buffer, sizeof(overlay_buffer))) {
				MTKFB_WRAN("[FB Driver]: copy_to_user failed! line:%d\n", __LINE__);
				r = -EFAULT;
			}
		}

		return r;
	}
#endif				/* #if defined(MTK_FB_SYNC_SUPPORT) */

	case MTKFB_SET_ORIENTATION:
		MTKFB_INFO("[MTKFB] Set Orientation: %lu\n", arg);
		/* surface flinger orientation definition of 90 and 270 */
		/* is different than DISP_TV_ROT */
		if (arg & 0x1)
			arg ^= 0x2;
		arg *= 90;

		return 0;

	case MTKFB_GET_INTERFACE_TYPE:
	{
		unsigned long lcm_type = lcm_params->type;

		MTKFB_INFO("[MTKFB] MTKFB_GET_INTERFACE_TYPE\n");

		MTKFB_INFO("[MTKFB EM]MTKFB_GET_INTERFACE_TYPE is %ld\n", lcm_type);

		return copy_to_user(argp, &lcm_type, sizeof(lcm_type)) ? -EFAULT : 0;
	}

	case MTKFB_GET_DEFAULT_UPDATESPEED:
	{
		unsigned int speed;
		MTKFB_INFO("[MTKFB] get default update speed\n");
		DISP_Get_Default_UpdateSpeed(&speed);

		MTKFB_INFO("[MTKFB EM]MTKFB_GET_DEFAULT_UPDATESPEED is %d\n", speed);
		return copy_to_user(argp, &speed, sizeof(speed)) ? -EFAULT : 0;
	}

	case MTKFB_GET_CURR_UPDATESPEED:
	{
		unsigned int speed;
		MTKFB_INFO("[MTKFB] get current update speed\n");
		DISP_Get_Current_UpdateSpeed(&speed);

		MTKFB_INFO("[MTKFB EM]MTKFB_GET_CURR_UPDATESPEED is %d\n", speed);
		return copy_to_user(argp, &speed, sizeof(speed)) ? -EFAULT : 0;
	}

	case MTKFB_CHANGE_UPDATESPEED:
	{
		unsigned int speed;
		MTKFB_INFO("[MTKFB] change update speed\n");

		if (copy_from_user(&speed, (void __user *)arg, sizeof(speed))) {
			MTKFB_WRAN("[FB]: copy_from_user failed! line:%d\n", __LINE__);
			r = -EFAULT;
		} else {
			DISP_Change_Update(speed);

			MTKFB_INFO("[MTKFB EM]MTKFB_CHANGE_UPDATESPEED is %d\n", speed);
		}
		return r;
	}

	case MTKFB_FBLAYER_ENABLE:
	{
		BOOL enable;
		if (copy_from_user(&enable, (void __user *)argp, sizeof(BOOL))) {
			MTKFB_WRAN("[FB]: copy_from_user failed! line:%d\n", __LINE__);
			r = -EFAULT;
		} else {
			MTKFB_INFO("[FB]: FDLAYER_ENABLE:%d\n", enable);

			hwc_force_fb_enabled = (enable ? true : false);

			mutex_lock(&OverlaySettingMutex);
			cached_layer_config[FB_LAYER].layer_en = enable;
			if (mtkfb_using_layer_type != LAYER_2D)
				cached_layer_config[FB_LAYER + 1].layer_en = enable;
			cached_layer_config[FB_LAYER].isDirty = true;
			atomic_set(&OverlaySettingDirtyFlag, 1);
			atomic_set(&OverlaySettingApplied, 0);
			mutex_unlock(&OverlaySettingMutex);
		}

		return r;
	}

	case MTKFB_FACTORY_AUTO_TEST:
	{
		unsigned int result = 0;
		MTKFB_INFO("factory mode: lcm auto test\n");
		result = mtkfb_fm_auto_test();
		return copy_to_user(argp, &result, sizeof(result)) ? -EFAULT : 0;
	}

	case MTKFB_AEE_LAYER_EXIST:
		/* MTKFB_INFO("[MTKFB] isAEEEnabled=%d\n", isAEEEnabled); */
		return copy_to_user(argp, &isAEEEnabled, sizeof(isAEEEnabled)) ? -EFAULT : 0;

	case MTKFB_LOCK_FRONT_BUFFER:
		return 0;

	case MTKFB_UNLOCK_FRONT_BUFFER:
		return 0;

/* ============================================================================= */
/* Multiple Display Support */
/* ================ */
	case DISP_IOCTL_CREATE_SESSION:
	{
		struct mmp_session_config {
			unsigned int type;
			unsigned int device;
			unsigned int mode;
			unsigned int session;
		};
		disp_session_config config;
		if (copy_from_user(&config, argp, sizeof(config))) {
			MTKFB_WRAN("[FB]: copy_from_user failed!\n");
			return -EFAULT;
		}
		MMProfileLogStructure(MTKFB_MMP_Events.CreateSession, MMProfileFlagStart, &config, disp_session_config);
		if (disp_create_session(&config) != DCP_STATUS_OK)
			r = -EFAULT;
		if (copy_to_user(argp, &config, sizeof(config))) {
			MTKFB_WRAN("[FB]: copy_to_user failed!\n");
			r = -EFAULT;
		}
		MMProfileLogEx(MTKFB_MMP_Events.CreateSession, MMProfileFlagEnd, config.session_id, disp_config.session_id);
		return r;
	}

	case DISP_IOCTL_DESTROY_SESSION:
	{
		disp_session_config config;

		if (copy_from_user(&config, argp, sizeof(config))) {
			MTKFB_WRAN("[FB]: copy_from_user failed!\n");
			return -EFAULT;
		}
		MMProfileLogStructure(MTKFB_MMP_Events.DestroySession, MMProfileFlagStart, &config, disp_session_config);
		if (disp_destroy_session(&config) != DCP_STATUS_OK)
			r = -EFAULT;
		MMProfileLogEx(MTKFB_MMP_Events.DestroySession, MMProfileFlagEnd, config.session_id, disp_config.session_id);
		return r;
	}

	case DISP_IOCTL_TRIGGER_SESSION:
	{
		disp_session_config config;
		if (copy_from_user(&config, argp, sizeof(config))) {
			MTKFB_WRAN("[FB]: copy_from_user failed!\n");
			return -EFAULT;
		}
		MMProfileLogStructure(MTKFB_MMP_Events.TriggerSession, MMProfileFlagStart, &config, disp_session_config);

		mutex_lock(&ScreenCaptureMutex);
		if (down_interruptible(&sem_early_suspend)) {
			pr_err("[FB Driver] can't get semaphore in DISP_IOCTL_TRIGGER_SESSION()\n");
			mutex_unlock(&ScreenCaptureMutex);
			return -EFAULT;
		}
		sem_early_suspend_cnt--;

		if ((DISP_SESSION_TYPE(config.session_id) == DISP_SESSION_PRIMARY) && (config.present_fence_idx != 0))
			disp_sync_set_present_fence(config.present_fence_idx);

		if (is_early_suspended) {
			if (!is_lcm_always_on) {
				int i = 0;

				disp_sync_present_fence_inc(disp_config.session_id);

				for (i = 0; i < HW_OVERLAY_COUNT; i++) {
					disp_sync_release(disp_config.session_id, i);
					if (!((i == DISP_DEFAULT_UI_LAYER_ID) && isAEEEnabled)) {
						cached_layer_config[i].layer_en = 0;
						cached_layer_config[i].isDirty = 0;
					}
				}

				MTKFB_WRAN("disp_cancel_job\n");
				disp_cancel_job(config.session_id);
				sem_early_suspend_cnt++;
				up(&sem_early_suspend);
				mutex_unlock(&ScreenCaptureMutex);
				return r;
			}
		}

		disp_enque_job(config.session_id);

		if (is_early_suspended)
			DISP_TriggerSessionEarlySuspend();

		sem_early_suspend_cnt++;
		up(&sem_early_suspend);
		mutex_unlock(&ScreenCaptureMutex);

		if (start_update_state) {
			DISP_StartConfigUpdate();
			start_update_state = 0;
		}

		if (DISP_IsDecoupleMode())
			DISP_StartOverlayTransfer();
		MMProfileLogEx(MTKFB_MMP_Events.TriggerSession, MMProfileFlagEnd, config.session_id, disp_config.session_id);
		return r;
	}

	case DISP_IOCTL_PREPARE_INPUT_BUFFER:
	case DISP_IOCTL_PREPARE_OUTPUT_BUFFER:
	{
		disp_buffer_info info;

		if (copy_from_user(&info, (void __user *)arg, sizeof(info))) {
			MTKFB_WRAN("[FB]: copy_from_user failed!\n");
			return -EFAULT;
		}
		MMProfileLogStructure(MTKFB_MMP_Events.PrepareInput, MMProfileFlagStart, &info, disp_buffer_info);
		if (info.layer_en) {
			if (disp_sync_prepare_buffer(&info) != SYNC_STATUS_OK) {
				info.fence_fd = DISP_INVALID_FENCE_FD;	/* invalid fd */
				info.index = 0;
				r = -EFAULT;
			}
		} else {
			info.fence_fd = DISP_INVALID_FENCE_FD;	/* invalid fd */
			info.index = 0;
		}
		if (copy_to_user((void __user *)arg, &info, sizeof(info))) {
			MTKFB_WRAN("[FB]: copy_to_user failed!\n");
			r = -EFAULT;
		}
		MMProfileLogEx(MTKFB_MMP_Events.PrepareInput, MMProfileFlagEnd, disp_config.session_id, r);
		return r;
	}

	case DISP_IOCTL_SET_INPUT_BUFFER:
	{
		disp_session_input_config layerInfo;

		if (copy_from_user(&layerInfo, (void __user *)arg, sizeof(layerInfo))) {
			MTKFB_WRAN("[FB]: copy_from_user failed!\n");
			return -EFAULT;
		}
		MMProfileLogStructure(MTKFB_MMP_Events.SetInput, MMProfileFlagStart, &layerInfo, disp_session_input_config);

#if defined(CONFIG_MTK_LCM_PHYSICAL_ROTATION)
		if (0 == strncmp(CONFIG_MTK_LCM_PHYSICAL_ROTATION, "180", 3)) {
			int i;

			for (i = 0; i < MAX_INPUT_CONFIG; i++) {
				if (layerInfo.config[i].layer_enable) {
					layerInfo.config[i].layer_rotation = (layerInfo.config[i].layer_rotation + MTK_FB_ORIENTATION_180) % 4;
					layerInfo.config[i].tgt_offset_x = MTK_FB_XRES - (layerInfo.config[i].tgt_offset_x + layerInfo.config[i].tgt_width);
					layerInfo.config[i].tgt_offset_y = MTK_FB_YRES - (layerInfo.config[i].tgt_offset_y + layerInfo.config[i].tgt_height);
				}
			}
		}
#endif

		if (disp_set_session_input(&layerInfo) != DCP_STATUS_OK)
			r = -EFAULT;
		is_ipoh_bootup = false;
		MMProfileLogEx(MTKFB_MMP_Events.SetInput, MMProfileFlagEnd, disp_config.session_id, r);
		return r;
	}

	case DISP_IOCTL_SET_OUTPUT_BUFFER:
	{
		disp_session_output_config layerInfo;

		if (copy_from_user(&layerInfo, (void __user *)arg, sizeof(layerInfo))) {
			MTKFB_WRAN("[FB]: copy_from_user failed!\n");
			return -EFAULT;
		}
		MMProfileLogStructure(MTKFB_MMP_Events.SetOutput, MMProfileFlagStart, &layerInfo, disp_session_output_config);
		if (disp_set_session_output(&layerInfo) != DCP_STATUS_OK)
			r = -EFAULT;
		MMProfileLogEx(MTKFB_MMP_Events.SetOutput, MMProfileFlagEnd, disp_config.session_id, r);
		return r;
	}

	case DISP_IOCTL_GET_SESSION_INFO:
	{
		disp_session_info info;
		int dev = 0;

		if (copy_from_user(&info, argp, sizeof(info))) {
			MTKFB_WRAN("[FB]: copy_from_user failed!\n");
			return -EFAULT;
		}
		dev = DISP_SESSION_DEV(info.session_id);
		info.displayFormat = dispif_info[dev].displayFormat;
		info.displayHeight = dispif_info[dev].displayHeight;
		info.displayMode = dispif_info[dev].displayMode;
		info.displayType = dispif_info[dev].displayType;
		info.displayWidth = dispif_info[dev].displayWidth;
		info.isConnected = dispif_info[dev].isConnected;
		info.isHwVsyncAvailable = dispif_info[dev].isHwVsyncAvailable;
		info.maxLayerNum = isAEEEnabled ? (HW_OVERLAY_COUNT - 1) : HW_OVERLAY_COUNT;
		info.physicalHeight = dispif_info[dev].physicalHeight;
		info.physicalWidth = dispif_info[dev].physicalWidth;
		info.vsyncFPS = dispif_info[dev].vsyncFPS;
		MMProfileLogStructure(MTKFB_MMP_Events.GetDispInfo, MMProfileFlagPulse, &info, disp_session_info);

		if (copy_to_user(argp, &info, sizeof(info))) {
			MTKFB_WRAN("[FB]: copy_to_user failed!\n");
			r = -EFAULT;
		}
		return r;
	}

	case DISP_IOCTL_GET_PRESENT_FENCE:
		return disp_sync_prepare_present_fence(arg);

/* ////////////////////////////////////////////// */

	case MTKFB_LCM_ALWAYS_ON_ENABLE:
	{
		BOOL alwaysOn;

		if (copy_from_user(&alwaysOn, (void __user *)argp, sizeof(BOOL))) {
			MTKFB_INFO("[FB]: copy_from_user failed! line:%d\n", __LINE__);
			r = -EFAULT;
		} else {
			if ((lcm_params->type == LCM_TYPE_DBI) || (lcm_params->type == LCM_TYPE_DSI && lcm_params->dsi.mode == CMD_MODE)) {
				if ((is_lcm_always_on != alwaysOn) && is_early_suspended) {
					DISP_LateResume();
					is_lcm_always_on = alwaysOn;
					DISP_EarlySuspend();
				}
				else {
					is_lcm_always_on = alwaysOn;
				}

			}
			DISP_LOG_PRINT("[FB]: MTKFB_LCM_ALWAYS_ON_ENABLE: %d\n", is_lcm_always_on);
		}

		return r;
	}

	case MTKFB_SET_DISPLAY_POWER_MODE:
	{
		unsigned int state;
		BOOL alwaysOn = FALSE;

		if (copy_from_user(&state, (void __user *)argp, sizeof(unsigned int))) {
			MTKFB_INFO("[FB]: copy_from_user failed! line:%d\n", __LINE__);
			r = -EFAULT;
		} else {
			display_power_state = state;
			if ((lcm_params->type == LCM_TYPE_DBI) || (lcm_params->type == LCM_TYPE_DSI && lcm_params->dsi.mode == CMD_MODE)) {
				if (display_power_state != DISP_POWER_MODE_OFF)
					alwaysOn = TRUE;

				if ((is_lcm_always_on != alwaysOn) && is_early_suspended) {
					DISP_LateResume();
					is_lcm_always_on = alwaysOn;
					DISP_EarlySuspend();
				}
				else {
					is_lcm_always_on = alwaysOn;
				}
			}
			blocking_notifier_call_chain(&ambient_mode_notifier, display_power_state, NULL);
			DISP_LOG_PRINT("[FB]: MTKFB_SET_DISPLAY_POWER_MODE: %d %d\n", display_power_state, is_lcm_always_on);
		}

		return r;
	}

/* ////////////////////////////////////////////// */
	default:
		pr_err("mtkfb_ioctl Not support, info=0x%08x, cmd=0x%08x, arg=0x%08x, num=%d\n", (unsigned int)info, (unsigned int)cmd, (unsigned int)arg, _IOC_NR(cmd));
		return -EINVAL;
	}
}

static int mtkfb_fbinfo_modify(struct fb_info *info)
{
	struct fb_var_screeninfo var;
	int r = 0;

	memcpy(&var, &(info->var), sizeof(var));
	var.activate = FB_ACTIVATE_NOW;
	var.bits_per_pixel = 32;
	var.transp.offset = 24;
	var.transp.length = 8;
	var.red.offset = 0;
	var.red.length = 8;
	var.green.offset = 8;
	var.green.length = 8;
	var.blue.offset = 16;
	var.blue.length = 8;
	var.yoffset = var.yres;

	r = mtkfb_check_var(&var, info);
	if (r != 0)
		MTKFB_WRAN("failed to mtkfb_check_var\n");

	info->var = var;

	r = mtkfb_set_par(info);
	if (r != 0)
		MTKFB_WRAN("failed to mtkfb_set_par\n");

	if (!DISP_IsDecoupleMode())
		r = disp_enque_job(disp_config.session_id);

	return r;
}

UINT32 color = 0;
unsigned int mtkfb_fm_auto_test(void)
{
	unsigned int result = 0;
	unsigned int bls_enable = 0;
	unsigned int i = 0;
	UINT32 fbVirAddr;
	UINT32 fbsize;
	int r = 0;
	unsigned int *fb_buffer;
	struct mtkfb_device *fbdev = (struct mtkfb_device *)mtkfb_fbi->par;
	struct fb_var_screeninfo var;
	extern bool needStartEngine;

	mtkfb_pan_display_impl(&mtkfb_fbi->var, mtkfb_fbi);

	fbVirAddr = (UINT32) fbdev->fb_va_base;
	fb_buffer = (unsigned int *)fbVirAddr;

	memcpy(&var, &(mtkfb_fbi->var), sizeof(var));
	var.activate = FB_ACTIVATE_NOW;
	var.bits_per_pixel = 32;
	var.transp.offset = 24;
	var.transp.length = 8;
	var.red.offset = 16;
	var.red.length = 8;
	var.green.offset = 8;
	var.green.length = 8;
	var.blue.offset = 0;
	var.blue.length = 8;

	r = mtkfb_check_var(&var, mtkfb_fbi);
	if (r != 0)
		MTKFB_WRAN("failed to mtkfb_check_var\n");

	mtkfb_fbi->var = var;
	r = mtkfb_set_par(mtkfb_fbi);
	if (r != 0)
		MTKFB_WRAN("failed to mtkfb_set_par\n");

	if (color == 0)
		color = 0xFF00FF00;
	fbsize = ALIGN_TO(DISP_GetScreenWidth(), disphal_get_fb_alignment()) * DISP_GetScreenHeight() * MTK_FB_PAGES;

	DISP_DispsysPowerEnable(TRUE);

	bls_enable = DISP_BLS_Query();

	MTKFB_INFO("BLS is enable %d\n", bls_enable);
	if (bls_enable == 1)
		DISP_BLS_Enable(false);

	for (i = 0; i < fbsize; i++)
		*fb_buffer++ = color;

	for (i = 0; i < DDP_OVL_LAYER_MUN; i++)
		cached_layer_config[i].isDirty = 1;
	needStartEngine = TRUE;
	mtkfb_update_screen(NULL);
	if (DISP_IsDecoupleMode())
		DISP_StartOverlayTransfer();

	msleep(100);

	if (down_interruptible(&sem_early_suspend)) {
		MTKFB_WRAN("[FB Driver] can't get semaphore in mtkfb_fm_auto_test()\n");
		return result;
	}
	DISP_PrepareSuspend();
	/* Wait for disp finished. */
	if (wait_event_interruptible_timeout(disp_done_wq, !disp_running, HZ / 10) == 0)
		MTKFB_WRAN("[FB Driver] Wait disp finished timeout in early_suspend\n");

	result = DISP_AutoTest();

	up(&sem_early_suspend);
	if (result == 0)
		MTKFB_WRAN("ATA LCM failed\n");
	else
		MTKFB_INFO("ATA LCM passed\n");

	if (bls_enable == 1)
		DISP_BLS_Enable(true);
	return result;
}

static int mtkfb_blank(int blank_mode, struct fb_info *info)
{
	switch (blank_mode) {
	case FB_BLANK_UNBLANK:
	case FB_BLANK_NORMAL:
		mtkfb_late_resume();
		break;
	case FB_BLANK_VSYNC_SUSPEND:
	case FB_BLANK_HSYNC_SUSPEND:
		break;
	case FB_BLANK_POWERDOWN:
		if (blank_skip_count > 0) {
			blank_skip_count--;
			break;
		}
		mtkfb_early_suspend();
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

/* Callback table for the frame buffer framework. Some of these pointers
 * will be changed according to the current setting of fb_info->accel_flags.
 */
static struct fb_ops mtkfb_ops = {
	.owner = THIS_MODULE,
	.fb_open = mtkfb_open,
	.fb_release = mtkfb_release,
	.fb_setcolreg = mtkfb_setcolreg,
	.fb_pan_display = mtkfb_pan_display_proxy,
	.fb_fillrect = cfb_fillrect,
	.fb_copyarea = cfb_copyarea,
	.fb_imageblit = cfb_imageblit,
	.fb_cursor = mtkfb_soft_cursor,
	.fb_check_var = mtkfb_check_var,
	.fb_set_par = mtkfb_set_par,
	.fb_ioctl = mtkfb_ioctl,
	.fb_blank = mtkfb_blank,
};

/*
 * ---------------------------------------------------------------------------
 * Sysfs interface
 * ---------------------------------------------------------------------------
 */

static int mtkfb_register_sysfs(struct mtkfb_device *fbdev)
{
	NOT_REFERENCED(fbdev);

	return 0;
}

static void mtkfb_unregister_sysfs(struct mtkfb_device *fbdev)
{
	NOT_REFERENCED(fbdev);
}

/*
 * ---------------------------------------------------------------------------
 * LDM callbacks
 * ---------------------------------------------------------------------------
 */
/* Initialize system fb_info object and set the default video mode.
 * The frame buffer memory already allocated by lcddma_init
 */
static int mtkfb_fbinfo_init(struct fb_info *info)
{
	struct mtkfb_device *fbdev = (struct mtkfb_device *)info->par;
	struct fb_var_screeninfo var;
	int r = 0;

	MSG_FUNC_ENTER();

	BUG_ON(!fbdev->fb_va_base);
	info->fbops = &mtkfb_ops;
	info->flags = FBINFO_FLAG_DEFAULT;
	info->screen_base = (char *)fbdev->fb_va_base;
	info->screen_size = fbdev->fb_size_in_byte;
	info->pseudo_palette = fbdev->pseudo_palette;
	if (!DISP_IsDecoupleMode())
		cached_layer_config[FB_LAYER].alpha = 0xFF;
	r = fb_alloc_cmap(&info->cmap, 16, 0);
	if (r != 0)
		MTKFB_WRAN("unable to allocate color map memory\n");

	/* setup the initial video mode (RGB565) */

	memset(&var, 0, sizeof(var));

	var.xres = MTK_FB_XRES;
	var.yres = MTK_FB_YRES;
	var.xres_virtual = MTK_FB_XRESV;
	var.yres_virtual = MTK_FB_YRESV;

	var.bits_per_pixel = 16;

	var.red.offset = 11;
	var.red.length = 5;
	var.green.offset = 5;
	var.green.length = 6;
	var.blue.offset = 0;
	var.blue.length = 5;

	var.width = DISP_GetPhysicalWidth();
	var.height = DISP_GetPhysicalHeight();

	var.activate = FB_ACTIVATE_NOW;

	r = mtkfb_check_var(&var, info);
	if (r != 0)
		MTKFB_WRAN("failed to mtkfb_check_var\n");

	info->var = var;

	r = mtkfb_set_par(info);
	if (r != 0)
		MTKFB_WRAN("failed to mtkfb_set_par\n");

	MSG_FUNC_LEAVE();
	return r;
}

/* Release the fb_info object */
static void mtkfb_fbinfo_cleanup(struct mtkfb_device *fbdev)
{
	MSG_FUNC_ENTER();

	fb_dealloc_cmap(&fbdev->fb_info->cmap);

	MSG_FUNC_LEAVE();
}

void mtkfb_disable_non_fb_layer(void)
{
	int id;
	unsigned int dirty = 0;
	for (id = 0; id < DDP_OVL_LAYER_MUN; id++) {
		if (cached_layer_config[id].layer_en == 0)
			continue;

		if (cached_layer_config[id].addr >= fb_pa &&
		    cached_layer_config[id].addr < (fb_pa + DISP_GetVRamSize()))
			continue;

		DISP_LOG_PRINT("[LCD] disable(%d)\n", id);
		cached_layer_config[id].layer_en = 0;
		cached_layer_config[id].isDirty = true;
		dirty = 1;
	}
	if (dirty) {
		memset(mtkfb_fbi->screen_base, 0, DISP_GetFBRamSize());
		mtkfb_pan_display_impl(&mtkfb_fbi->var, mtkfb_fbi);
	}
}

int m4u_reclaim_mva_callback_ovl(int moduleID, unsigned int va, unsigned int size, unsigned int mva)
{
	int id;
	unsigned int dirty = 0;
	MMProfileLogEx(MTKFB_MMP_Events.Debug, MMProfileFlagStart, mva, size);
	for (id = 0; id < DDP_OVL_LAYER_MUN; id++) {
		if (cached_layer_config[id].layer_en == 0)
			continue;

		if (cached_layer_config[id].addr >= mva &&
		    cached_layer_config[id].addr < (mva + size)) {
			MTKFB_WRAN("Warning: m4u required to disable layer id=%d\n", id);
			cached_layer_config[id].layer_en = 0;
			cached_layer_config[id].isDirty = 1;
			dirty = 1;
		}
	}
	if (dirty) {
		MTKFB_INFO(KERN_INFO "Warning: m4u_reclaim_mva_callback_ovl. mva=0x%08X size=0x%X dirty=%d\n", mva, size, dirty);
		memset(mtkfb_fbi->screen_base, 0, DISP_GetVRamSize());
		mtkfb_pan_display_impl(&mtkfb_fbi->var, mtkfb_fbi);
	}
	MMProfileLogEx(MTKFB_MMP_Events.Debug, MMProfileFlagEnd, dirty, 0);
	return 0;
}

#define RGB565_TO_ARGB8888(x) \
	(((((x) & 0x1F) << 3) | (((x) & 0x1C) >> 2)) | \
	 (((x) & 0x7E0) << 5) | (((x) & 0x600) >> 1) | \
	 (((x) & 0xF800) << 8) | (((x) & 0xE000) << 3) | \
	 (0xFF << 24))

#define RGB565_TO_ABGR8888(x) \
	(((((x) & 0x1F) << 19) | (((x) & 0x1C) << 14)) | \
	 (((x) & 0x7E0) << 5) | (((x) & 0x600) >> 1) | \
	 (((x) & 0xF800) >> 8) | (((x) & 0xE000) >> 13) | \
	 (0xFF << 24))

/* Free driver resources. Can be called to rollback an aborted initialization
 * sequence.
 */
static void mtkfb_free_resources(struct mtkfb_device *fbdev, int state)
{
	int r = 0;

	switch (state) {
	case MTKFB_ACTIVE:
		r = unregister_framebuffer(fbdev->fb_info);
		ASSERT(0 == r);
		/* lint -fallthrough */
	case 5:
		mtkfb_unregister_sysfs(fbdev);
		/* lint -fallthrough */
	case 4:
		mtkfb_fbinfo_cleanup(fbdev);
		/* lint -fallthrough */
	case 3:
		DISP_CHECK_RET(DISP_Deinit());
		/* lint -fallthrough */
	case 2:
		dma_free_coherent(0, fbdev->fb_size_in_byte, fbdev->fb_va_base, fbdev->fb_pa_base);
		/* lint -fallthrough */
	case 1:
		dev_set_drvdata(fbdev->dev, NULL);
		framebuffer_release(fbdev->fb_info);
		/* lint -fallthrough */
	case 0:
		/* nothing to free */
		break;
	default:
		BUG();
	}
}

extern char *saved_command_line;
char mtkfb_lcm_name[256] = { 0 };

BOOL mtkfb_find_lcm_driver(void)
{
	BOOL ret = FALSE;
	char *p, *q;

	p = strstr(saved_command_line, "lcm=");
	if (p == NULL) {
		/* we can't find lcm string in the command line, the lk should be old version */
		return DISP_SelectDevice(NULL);
	}

	p += 4;
	if ((p - saved_command_line) > strlen(saved_command_line + 1)) {
		ret = FALSE;
		goto done;
	}

	MTKFB_INFO("%s, %s\n", __func__, p);
	q = p;
	while (*q != ' ' && *q != '\0')
		q++;

	memset((void *)mtkfb_lcm_name, 0, sizeof(mtkfb_lcm_name));
	strncpy((char *)mtkfb_lcm_name, (const char *)p, (int)(q - p));

	MTKFB_INFO("%s, %s\n", __func__, mtkfb_lcm_name);
	if (DISP_SelectDevice(mtkfb_lcm_name))
		ret = TRUE;

done:
	return ret;
}

void disp_get_fb_address(UINT32 *fbVirAddr, UINT32 *fbPhysAddr)
{
	struct mtkfb_device *fbdev = (struct mtkfb_device *)mtkfb_fbi->par;

	*fbVirAddr = (UINT32) fbdev->fb_va_base + mtkfb_fbi->var.yoffset * mtkfb_fbi->fix.line_length;
	*fbPhysAddr = (UINT32) fbdev->fb_pa_base + mtkfb_fbi->var.yoffset * mtkfb_fbi->fix.line_length;
}

static void mtkfb_fb_565_to_8888(struct fb_info *fb_info)
{
	unsigned int xres = fb_info->var.xres;
	unsigned int yres = fb_info->var.yres;
	unsigned int x_virtual = ALIGN_TO(xres, disphal_get_fb_alignment());

	unsigned int fbsize = x_virtual * yres * 2;

	unsigned short *s = (unsigned short *)fb_info->screen_base;
	unsigned int *d = (unsigned int *)(fb_info->screen_base + fbsize * 2);
	unsigned short src_rgb565 = 0;
	int j = 0;
	int k = 0;
	int wait_ret = 0;

	MTKFB_INFO("mtkfb_fb_565_to_8888 xres=%d yres=%d fbsize=0x%X x_virtual=%d s=0x%08X d=0x%08X\n",
		   xres, yres, fbsize, x_virtual, (unsigned int)s, (unsigned int)d);

	for (j = 0; j < yres; ++j) {
		for (k = 0; k < xres; ++k) {
			src_rgb565 = *s++;
			*d++ = RGB565_TO_ABGR8888(src_rgb565);
		}
		d += (ALIGN_TO(xres, MTK_FB_ALIGNMENT) - xres);
		s += (ALIGN_TO(xres, disphal_get_fb_alignment()) - xres);
	}

	mtkfb_fbinfo_modify(fb_info);
	wait_ret = wait_event_interruptible_timeout(reg_update_wq, atomic_read(&OverlaySettingApplied), HZ / 10);
	MTKFB_INFO("[WaitQ] wait_event_interruptible() ret = %d, %d\n", wait_ret, __LINE__);

	s = (unsigned short *)fb_info->screen_base;
	d = (unsigned int *)(fb_info->screen_base + fbsize * 2);
	memcpy(s, d, fbsize * 2);
}

#ifdef FPGA_DEBUG_PAN
static int update_test_kthread(void *data)
{
	/* struct sched_param param = { .sched_priority = RTPM_PRIO_SCRN_UPDATE }; */
	/* sched_setscheduler(current, SCHED_RR, &param); */
	unsigned int i = 0;
	for (;;) {
		if (kthread_should_stop())
			break;
		msleep(1000);	/* 2s */
		MTKFB_INFO("update test thread work\n");
		mtkfb_fbi->var.yoffset = (i % 2) * DISP_GetScreenHeight();
		i++;
		mtkfb_pan_display_impl(&mtkfb_fbi->var, mtkfb_fbi);
	}

	MTKFB_INFO("exit esd_recovery_kthread()\n");
	return 0;
}
#endif

#if defined(MTK_OVL_DECOUPLE_SUPPORT)
static void mtkfb_fb_565_to_888(void *fb_va)
{
	unsigned int xres = DISP_GetScreenWidth();
	unsigned int yres = DISP_GetScreenHeight();
	/* unsigned int x_virtual = ALIGN_TO(xres,disphal_get_fb_alignment()); */

	unsigned short *s = (unsigned short *)(fb_va);
	unsigned char *d = (unsigned char *)(fb_va + DISP_GetFBRamSize() + DAL_GetLayerSize());
	unsigned short src_rgb565 = 0;
	int j = 0;
	int k = 0;

	MTKFB_INFO("555_to_888, s = 0x%x, d=0x%x\n", (unsigned int)s, (unsigned int)d);
	for (j = 0; j < yres; ++j) {
		for (k = 0; k < xres; ++k) {
			src_rgb565 = *s++;
			*d++ = (((src_rgb565 & 0x1F) << 3) | ((src_rgb565 & 0x1C) >> 2));
			*d++ = (((src_rgb565 & 0x7E0) >> 3) | ((src_rgb565 & 0x600) >> 9));
			*d++ = (((src_rgb565 & 0xF800) >> 8) | ((src_rgb565 & 0xE000) >> 13));
		}
		s += (ALIGN_TO(xres, disphal_get_fb_alignment()) - xres);
	}
}
#endif

#if defined(DFO_USE_NEW_API)
extern int dfo_query(const char *s, unsigned long *v);
#else
#include <mach/dfo_boot.h>
static disp_dfo_item_t disp_dfo_setting[] = {
	{"LCM_FAKE_WIDTH", 0},
	{"LCM_FAKE_HEIGHT", 0},
	{"DISP_DEBUG_SWITCH", 0}
};

#define MT_DISP_DFO_DEBUG
#ifdef MT_DISP_DFO_DEBUG
#define disp_dfo_printf(fmt, args...) pr_warn("[DISP/DFO]"fmt, ##args)
#else
#define disp_dfo_printf(fmt, args...) ()
#endif

/* this function will be called in mt_fixup()@mt_devs.c. which will send DFO information organized as tag_dfo_boot struct. */
/* because lcm_params isn't inited here, so we will change lcm_params later in mtkfb_probe. */
unsigned int mtkfb_parse_dfo_setting(void *dfo_tbl, int num)
{
	char *disp_name = NULL;
	/* int  *disp_value; */
	char *tag_name;
	int tag_value;
	int i, j;
	tag_dfo_boot *dfo_data;

	disp_dfo_printf("enter mtkfb_parse_dfo_setting\n");

	if (dfo_tbl == NULL)
		return -1;

	dfo_data = (tag_dfo_boot *) dfo_tbl;
	for (i = 0; i < (sizeof(disp_dfo_setting) / sizeof(disp_dfo_item_t)); i++) {
		disp_name = disp_dfo_setting[i].name;

		for (j = 0; j < num; j++) {
			tag_name = dfo_data->name[j];
			tag_value = dfo_data->value[j];
			if (!strcmp(disp_name, tag_name)) {
				disp_dfo_setting[i].value = tag_value;
				disp_dfo_printf("%s = [DEC]%d [HEX]0x%08x\n", disp_dfo_setting[i].name, disp_dfo_setting[i].value, disp_dfo_setting[i].value);
			}
		}
	}

	disp_dfo_printf("leave mtkfb_parse_dfo_setting\n");

	return 0;
}

int mtkfb_get_dfo_setting(const char *string, unsigned int *value)
{
	char *disp_name;
	int disp_value;
	int i;

	if (string == NULL)
		return -1;

	for (i = 0; i < (sizeof(disp_dfo_setting) / sizeof(disp_dfo_item_t)); i++) {
		disp_name = disp_dfo_setting[i].name;
		disp_value = disp_dfo_setting[i].value;
		if (!strcmp(disp_name, string)) {
			*value = disp_value;
			disp_dfo_printf("%s = [DEC]%d [HEX]0x%08x\n", disp_name, disp_value, disp_value);
			return 0;
		}
	}

	return 0;
}
#endif

/* Called by LDM binding to probe and attach a new device.
 * Initialization sequence:
 *   1. allocate system fb_info structure
 *      select panel type according to machine type
 *   2. init LCD panel
 *   3. init LCD controller and LCD DMA
 *   4. init system fb_info structure
 *   5. init gfx DMA
 *   6. enable LCD panel
 *      start LCD frame transfer
 *   7. register system fb_info structure
 */
static int mtkfb_probe(struct platform_device *pdev)
{
	struct device *dev;
	struct mtkfb_device *fbdev = NULL;
	struct fb_info *fbi;
	int init_state;
	int r = 0;
	unsigned int lcm_fake_width = 0;
	unsigned int lcm_fake_height = 0;
	char *p = NULL;

	MSG_FUNC_ENTER();

	if (get_boot_mode() == META_BOOT || get_boot_mode() == FACTORY_BOOT ||
	    get_boot_mode() == ADVMETA_BOOT || get_boot_mode() == RECOVERY_BOOT)
		first_update = false;

	MTKFB_INFO("%s, %s\n", __func__, saved_command_line);
	p = strstr(saved_command_line, "fps=");
	if (p == NULL) {
		lcd_fps = 6000;
		MTKFB_WRAN("[FB driver]can not get fps from lk\n");
	} else {
		p += 4;
		lcd_fps = simple_strtol(p, NULL, 10);
		if (0 == lcd_fps)
			lcd_fps = 6000;
	}

	if (DISP_IsContextInited() == FALSE) {
		if (mtkfb_find_lcm_driver()) {
			MTKFB_INFO("%s, we have found the lcm - %s\n", __func__, mtkfb_lcm_name);
		} else if (DISP_DetectDevice() != DISP_STATUS_OK) {
			MTKFB_WRAN("[mtkfb] detect device fail, maybe caused by the two reasons below:\n");
			MTKFB_WRAN("\t\t1.no lcm connected\n");
			MTKFB_WRAN("\t\t2.we can't support this lcm\n");
		}
	}
#ifdef MTKFB_FPGA_ONLY
	is_lcm_inited = FALSE;
#endif

#if defined(DFO_USE_NEW_API)
	if ((0 == dfo_query("LCM_FAKE_WIDTH", &lcm_fake_width)) && (0 == dfo_query("LCM_FAKE_HEIGHT", &lcm_fake_height))) {
#else
	if ((0 == mtkfb_get_dfo_setting("LCM_FAKE_WIDTH", &lcm_fake_width)) && (0 == mtkfb_get_dfo_setting("LCM_FAKE_HEIGHT", &lcm_fake_height))) {
#endif
		MTKFB_INFO("[DFO] LCM_FAKE_WIDTH=%d, LCM_FAKE_HEIGHT=%d\n", lcm_fake_width, lcm_fake_height);
		if (lcm_fake_width && lcm_fake_height) {
			if (DISP_STATUS_OK != DISP_Change_LCM_Resolution(lcm_fake_width, lcm_fake_height))
				MTKFB_WRAN("[DISP/DFO]WARNING!!! Change LCM Resolution FAILED!!!\n");
		}
	}

	MTK_FB_XRES = DISP_GetScreenWidth();
	MTK_FB_YRES = DISP_GetScreenHeight();
	fb_xres_update = MTK_FB_XRES;
	fb_yres_update = MTK_FB_YRES;

	MTKFB_INFO("[MTKFB] XRES=%d, YRES=%d\n", MTK_FB_XRES, MTK_FB_YRES);

	MTK_FB_BPP = DISP_GetScreenBpp();
	MTK_FB_PAGES = DISP_GetPages();

	if (DISP_IsLcmFound() && DISP_EsdRecoverCapbility()) {
		esd_recovery_task = kthread_create(esd_recovery_kthread, NULL, "esd_recovery_kthread");

		if (IS_ERR(esd_recovery_task))
			MTKFB_WRAN("ESD recovery task create fail\n");
		else
			wake_up_process(esd_recovery_task);
	}
	init_state = 0;

	if (pdev->num_resources != 1) {
		MTKFB_WRAN("probed for an unknown device\n");
		r = -ENODEV;
		goto cleanup;
	}

	dev = &(pdev->dev);
	fbi = framebuffer_alloc(sizeof(struct mtkfb_device), dev);
	if (!fbi) {
		MTKFB_WRAN("unable to allocate memory for device info\n");
		r = -ENOMEM;
		goto cleanup;
	}
	mtkfb_fbi = fbi;

	fbdev = (struct mtkfb_device *)fbi->par;
	fbdev->fb_info = fbi;
	fbdev->dev = dev;
	fbdev->layer_format = vmalloc(sizeof(MTK_FB_FORMAT) * HW_OVERLAY_COUNT);
	if (!fbdev->layer_format) {
		pr_err("[MTKFB] vmalloc failed, %d\n", __LINE__);
		r = -ENOMEM;
		goto cleanup;
	}
	memset(fbdev->layer_format, 0, sizeof(MTK_FB_FORMAT) * HW_OVERLAY_COUNT);

	dev_set_drvdata(dev, fbdev);

	init_state++;		/* 1 */

	/* Allocate and initialize video frame buffer */

	fbdev->fb_size_in_byte = MTK_FB_SIZEV;
	{
		struct resource *res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

		/* ASSERT(DISP_GetVRamSize() <= (res->end - res->start + 1)); */
		struct resource res_lk;
		unsigned int pa_lk;
		unsigned int va_lk;
		unsigned int dma_pa_lk;
		unsigned int fbsize_copy = mtkfb_fb_lk_copy_size();

		if (fb_address_lk) {
			res_lk.start = fb_address_lk;
			res_lk.end = fb_address_lk + fbsize_copy - 1;
			disphal_enable_mmu(FALSE);
			disphal_allocate_fb(&res_lk, &pa_lk, &va_lk, &dma_pa_lk);
		}

		disphal_enable_mmu(mtkfb_enable_mmu);
		disphal_allocate_fb(res, &fbdev->fb_pa_base, (unsigned int *)&fbdev->fb_va_base, &fb_pa);

		if (fb_address_lk) {
			DISP_LOG_PRINT("[wwy]fbsize_copy=%d\n", fbsize_copy);
			memcpy((void *)fbdev->fb_va_base, (void *)(va_lk), fbsize_copy);
			iounmap((void *)va_lk);
		}
#if 0
		DISP_LOG_PRINT("[MTKFB] max RAM = %x, actual RAM = %x\n", get_max_DRAM_size(), get_actual_DRAM_size());
		if (get_max_DRAM_size() < get_actual_DRAM_size()) {
			void *fb_va_base_lk;
			unsigned int fb_pa_base_lk = get_phys_offset() + get_actual_DRAM_size() - (res->end - res->start + 1);

			DISP_LOG_PRINT("Boot logo address shift, copy framebuffer from 0x%08X to 0x%08X\n", fb_pa_base_lk, fbdev->fb_pa_base);
			fb_va_base_lk = ioremap_nocache(fb_pa_base_lk, res->end - res->start + 1);
			memcpy(fbdev->fb_va_base, fb_va_base_lk, res->end - res->start + 1);
			iounmap(fb_va_base_lk);
		}
#endif

#if defined(MTK_OVL_DECOUPLE_SUPPORT)
		fbdev->ovl_pa_base = fbdev->fb_pa_base + DISP_GetFBRamSize() + DAL_GetLayerSize();
		fbdev->ovl_va_base = fbdev->fb_va_base + DISP_GetFBRamSize() + DAL_GetLayerSize();
		fbdev->ovl_size_in_byte = DISP_GetOVLRamSize();

		decouple_addr = fbdev->ovl_pa_base;
		decouple_size = fbdev->ovl_size_in_byte;
		/* Copy lk logo buffer to decouple buffer */
		mtkfb_fb_565_to_888((void *)fbdev->fb_va_base);
		MTKFB_INFO("fb_pa_base=0x%08x, fb_pa=0x%08x, decouple_addr=0x%08x\n", fbdev->fb_pa_base, fb_pa, decouple_addr);
#endif
#ifdef MTKFB_FPGA_ONLY
		memset((void *)fbdev->fb_va_base, 0x88, (res->end - res->start + 1));
#endif
	}

	MTKFB_INFO("[FB Driver] fbdev->fb_pa_base = %x, fbdev->fb_va_base = %x\n", fbdev->fb_pa_base, (unsigned int)(fbdev->fb_va_base));

	if (!fbdev->fb_va_base) {
		MTKFB_WRAN("unable to allocate memory for frame buffer\n");
		r = -ENOMEM;
		goto cleanup;
	}
	init_state++;		/* 2 */

#if defined(MTK_FB_SYNC_SUPPORT)
	disp_sync_init(disp_config.session_id);
#endif
	/* Initialize Display Driver PDD Layer */
	if (DISP_STATUS_OK != DISP_Init((DWORD) fbdev->fb_va_base, (DWORD) fb_pa, is_lcm_inited)) {
		r = -1;
		goto cleanup;
	}

	init_state++;		/* 3 */

	/* Register to system */

	r = mtkfb_fbinfo_init(fbi);
	if (r)
		goto cleanup;
	init_state++;		/* 4 */

	r = mtkfb_register_sysfs(fbdev);
	if (r)
		goto cleanup;
	init_state++;		/* 5 */

	r = register_framebuffer(fbi);
	if (r != 0) {
		MTKFB_WRAN("register_framebuffer failed\n");
		goto cleanup;
	}

	fbdev->state = MTKFB_ACTIVE;

	/********************************************/

	mtkfb_fb_565_to_8888(fbi);

	/********************************************/
	MSG(INFO, "MTK framebuffer initialized vram=%lu\n", fbdev->fb_size_in_byte);
#ifdef FPGA_DEBUG_PAN
	{
		unsigned int cnt = 0;
		unsigned int *fb_va = (unsigned int *)fbdev->fb_va_base;
		unsigned int fbsize = DISP_GetScreenHeight() * DISP_GetScreenWidth();

		for (cnt = 0; cnt < fbsize; cnt++)
			*(fb_va++) = 0xFFFF0000;
		for (cnt = 0; cnt < fbsize; cnt++)
			*(fb_va++) = 0xFF00FF00;
	}

	MTKFB_INFO("memset done\n");
	{
		struct task_struct *update_test_task = NULL;

		update_test_task = kthread_create(update_test_kthread, NULL, "update_test_kthread");

		if (IS_ERR(update_test_task))
			MTKFB_WRAN("update test task create fail\n");
		else
			wake_up_process(update_test_task);
	}
#endif
	MSG_FUNC_LEAVE();
	return 0;

cleanup:
	mtkfb_free_resources(fbdev, init_state);

	MSG_FUNC_LEAVE();
	return r;
}

/* Called when the device is being detached from the driver */
static int mtkfb_remove(struct device *dev)
{
	struct mtkfb_device *fbdev = dev_get_drvdata(dev);
	enum mtkfb_state saved_state = fbdev->state;

	MSG_FUNC_ENTER();
	/* FIXME: wait till completion of pending events */

	fbdev->state = MTKFB_DISABLED;
	mtkfb_free_resources(fbdev, saved_state);

	MSG_FUNC_LEAVE();
	return 0;
}

/* PM suspend */
static int mtkfb_suspend(struct device *pdev, pm_message_t mesg)
{
	NOT_REFERENCED(pdev);
	MSG_FUNC_ENTER();
	MTKFB_INFO("[FB Driver] mtkfb_suspend(): 0x%x\n", mesg.event);
	MSG_FUNC_LEAVE();
	return 0;
}

bool mtkfb_is_suspend(void)
{
	return is_early_suspended;
}

static void mtkfb_shutdown(struct device *pdev)
{
	if (dispsys_dynamic_cg_control_enable) {
		if (is_lcm_always_on) {
			DISP_LateResume();
			is_lcm_always_on = FALSE;
		}

		DISP_EarlySuspend();
		return;
	}

	MTKFB_INFO("[FB Driver] mtkfb_shutdown()\n");

	if (!lcd_fps)
		msleep(30);
	else
		msleep(2 * 100000 / lcd_fps);	/* Delay 2 frames. */

	if (is_early_suspended) {
		MTKFB_WRAN("mtkfb has been power off\n");
		return;
	}

	if (down_interruptible(&sem_early_suspend)) {
		MTKFB_WRAN("[FB Driver] can't get semaphore in mtkfb_shutdown()\n");
		return;
	}
	sem_early_suspend_cnt--;

	is_early_suspended = TRUE;
	DISP_PrepareSuspend();
	/* Wait for disp finished. */
	if (wait_event_interruptible_timeout(disp_done_wq, !disp_running, HZ / 10) == 0)
		MTKFB_WRAN("[FB Driver] Wait disp finished timeout in shut_down\n");
	DISP_CHECK_RET(DISP_PanelEnable(FALSE));
	DISP_CHECK_RET(DISP_PowerEnable(FALSE));

	DISP_CHECK_RET(DISP_PauseVsync(TRUE));
	disp_path_clock_off("mtkfb");
	sem_early_suspend_cnt++;
	up(&sem_early_suspend);

	MTKFB_INFO("[FB Driver] leave mtkfb_shutdown\n");
}

void mtkfb_clear_lcm(void)
{
	int i;
	unsigned int layer_status[DDP_OVL_LAYER_MUN] = { 0 };

	disp_job *job = disp_deque_job(disp_config.session_id);
	mutex_lock(&job->lock);

	for (i = 0; i < FB_LAYER; i++) {
		job->input[i].layer_enable = 0;
		job->input[i].dirty = 1;
	}
	mutex_unlock(&job->lock);
	disp_enque_job(disp_config.session_id);
	if (DISP_IsDecoupleMode()) {
		DISP_StartOverlayTransfer();
	} else {
		/* Need to config cached_layer_config for clear lcm purpose in DDlink mode */
		for (i = 0; i < HW_OVERLAY_COUNT; i++) {
			layer_status[i] = cached_layer_config[i].layer_en;
			cached_layer_config[i].layer_en = 0;
			cached_layer_config[i].isDirty = 1;
		}
	}
	DISP_CHECK_RET(DISP_UpdateScreen(0, 0, fb_xres_update, fb_yres_update));
	DISP_CHECK_RET(DISP_UpdateScreen(0, 0, fb_xres_update, fb_yres_update));
	if (!lcd_fps)
		msleep(30);
	else
		msleep(200000 / lcd_fps);	/* Delay 1 frame. */
	DISP_WaitForLCDNotBusy();
}

static void mtkfb_early_suspend(void)
{
	int i = 0;

	if (dispsys_dynamic_cg_control_enable) {
		DISP_EarlySuspend();
		return;
	}

	MSG_FUNC_ENTER();

	MTKFB_INFO("[FB Driver] enter early_suspend\n");

	mutex_lock(&ScreenCaptureMutex);

	if (!lcd_fps)
		msleep(30);
	else
		msleep(2 * 100000 / lcd_fps);	/* Delay 2 frames. */

	if (down_interruptible(&sem_early_suspend)) {
		MTKFB_WRAN("[FB Driver] can't get semaphore in mtkfb_early_suspend()\n");
		mutex_unlock(&ScreenCaptureMutex);
		return;
	}

	sem_early_suspend_cnt--;
	/* MMProfileLogEx(MTKFB_MMP_Events.EarlySuspend, MMProfileFlagStart, 0, 0); */

	if (is_early_suspended) {
		is_early_suspended = TRUE;
		sem_early_suspend_cnt++;
		up(&sem_early_suspend);
		MTKFB_WRAN("[FB driver] has been suspended\n");
		mutex_unlock(&ScreenCaptureMutex);
		return;
	}

	MMProfileLog(MTKFB_MMP_Events.EarlySuspend, MMProfileFlagStart);
	is_early_suspended = TRUE;

	DISP_PrepareSuspend();
	/* Wait for disp finished. */
	if (wait_event_interruptible_timeout(disp_done_wq, !disp_running, HZ / 10) == 0)
		MTKFB_WRAN("[FB Driver] Wait disp finished timeout in early_suspend\n");
#if defined(MTK_FB_SYNC_SUPPORT)
	for (i = 0; i < HW_OVERLAY_COUNT; i++) {
		disp_sync_release(disp_config.session_id, i);
		MTKFB_INFO("[FB driver] layer%d release fences\n", i);
	}
#endif
	DISP_CHECK_RET(DISP_PanelEnable(FALSE));
	DISP_CHECK_RET(DISP_PowerEnable(FALSE));

	DISP_CHECK_RET(DISP_PauseVsync(TRUE));
	disp_path_clock_off("mtkfb");
#if defined(MTK_FB_SYNC_SUPPORT)
	/* disp_sync_deinit(); */
#endif
	/* MMProfileLogEx(MTKFB_MMP_Events.EarlySuspend, MMProfileFlagEnd, 0, 0); */
	sem_early_suspend_cnt++;
	up(&sem_early_suspend);
	mutex_unlock(&ScreenCaptureMutex);

	MTKFB_INFO("[FB Driver] leave early_suspend\n");

	MSG_FUNC_LEAVE();
	aee_kernel_wdt_kick_Powkey_api("mtkfb_early_suspend", WDT_SETBY_Display);
}

/* PM resume */
static int mtkfb_resume(struct device *pdev)
{
	NOT_REFERENCED(pdev);
	MSG_FUNC_ENTER();
	MTKFB_INFO("[FB Driver] mtkfb_resume()\n");
	MSG_FUNC_LEAVE();
	return 0;
}

static void mtkfb_late_resume(void)
{
	if (dispsys_dynamic_cg_control_enable) {
		DISP_LateResume();
		return;
	}

	MSG_FUNC_ENTER();

	MTKFB_INFO("[FB Driver] enter late_resume\n");
	mutex_lock(&ScreenCaptureMutex);
	if (down_interruptible(&sem_early_suspend)) {
		MTKFB_WRAN("[FB Driver] can't get semaphore in mtkfb_late_resume()\n");
		mutex_unlock(&ScreenCaptureMutex);
		return;
	}
	sem_early_suspend_cnt--;
	/* MMProfileLogEx(MTKFB_MMP_Events.EarlySuspend, MMProfileFlagStart, 0, 0); */

	MMProfileLog(MTKFB_MMP_Events.EarlySuspend, MMProfileFlagEnd);
	if (is_ipoh_bootup) {
		atomic_set(&OverlaySettingDirtyFlag, 0);
		is_video_mode_running = true;
		disp_path_clock_on("ipoh_mtkfb");
	} else {
		if (is_early_suspended == FALSE) {
			is_early_suspended = FALSE;
			sem_early_suspend_cnt++;
			up(&sem_early_suspend);
			MTKFB_WRAN("[FB driver] has been resumed\n");
			mutex_unlock(&ScreenCaptureMutex);
			return;
		}

		disp_path_clock_on("mtkfb");
	}
	MTKFB_INFO("[FB LR] 1\n");
	DISP_CHECK_RET(DISP_PauseVsync(FALSE));
	MTKFB_INFO("[FB LR] 2\n");
	DISP_CHECK_RET(DISP_PowerEnable(TRUE));
	MTKFB_INFO("[FB LR] 3\n");
	DISP_CHECK_RET(DISP_PanelEnable(TRUE));
	MTKFB_INFO("[FB LR] 4\n");

	is_early_suspended = FALSE;

	if (is_ipoh_bootup)
		DISP_StartConfigUpdate();
	else
		mtkfb_clear_lcm();

	sem_early_suspend_cnt++;
	up(&sem_early_suspend);
	mutex_unlock(&ScreenCaptureMutex);

	if (BL_set_level_resume) {
		mtkfb_set_backlight_level(BL_level);
		BL_set_level_resume = FALSE;
	}

	MTKFB_INFO("[FB Driver] leave late_resume\n");

	MSG_FUNC_LEAVE();
	aee_kernel_wdt_kick_Powkey_api("mtkfb_late_resume", WDT_SETBY_Display);
}

/*---------------------------------------------------------------------------*/
#ifdef CONFIG_PM
/*---------------------------------------------------------------------------*/
int mtkfb_pm_suspend(struct device *device)
{
	struct platform_device *pdev = to_platform_device(device);
	BUG_ON(pdev == NULL);

	return mtkfb_suspend((struct device *)pdev, PMSG_SUSPEND);
}

int mtkfb_pm_resume(struct device *device)
{
	struct platform_device *pdev = to_platform_device(device);
	BUG_ON(pdev == NULL);

	return mtkfb_resume((struct device *)pdev);
}

#ifdef DEFAULT_MMP_ENABLE
void MMProfileStart(int start);
#endif
int mtkfb_pm_restore_noirq(struct device *device)
{
#ifdef DEFAULT_MMP_ENABLE
	MMProfileStart(0);
	MMProfileStart(1);
#endif

	disphal_pm_restore_noirq(device);
	is_ipoh_bootup = true;
	return 0;
}

int mtkfb_pm_restore_early(struct device *device)
{
	/* sometime disp_path_clock  will control i2c, when IPOH, */
	/* there is no irq in mtkfb_pm_restore_noirq , i2c will timeout. */
	/* so move this to  resore early. */
	disp_path_clock_on("ipoh_mtkfb");
	return 0;
}

/*---------------------------------------------------------------------------*/
#else				/* CONFIG_PM */
/*---------------------------------------------------------------------------*/
#define mtkfb_pm_suspend NULL
#define mtkfb_pm_resume  NULL
#define mtkfb_pm_restore_noirq NULL
#define mtkfb_pm_restore_early NULL
/*---------------------------------------------------------------------------*/
#endif				/* CONFIG_PM */
/*---------------------------------------------------------------------------*/
const struct dev_pm_ops mtkfb_pm_ops = {
	.suspend = mtkfb_pm_suspend,
	.resume = mtkfb_pm_resume,
	.freeze = mtkfb_pm_suspend,
	.thaw = mtkfb_pm_resume,
	.poweroff = mtkfb_pm_suspend,
	.restore = mtkfb_pm_resume,
	.restore_noirq = mtkfb_pm_restore_noirq,
	.restore_early = mtkfb_pm_restore_early,
};

static struct platform_driver mtkfb_driver = {
	.probe = mtkfb_probe,
	.driver = {
		   .name = MTKFB_DRIVER,
#ifdef CONFIG_PM
		   .pm = &mtkfb_pm_ops,
#endif
		   .bus = &platform_bus_type,
		   .remove = mtkfb_remove,
		   .suspend = NULL,
		   .resume = NULL,
		   .shutdown = mtkfb_shutdown,
		   },
};

#ifdef DEFAULT_MMP_ENABLE
void MMProfileEnable(int enable);
void MMProfileStart(int start);
void init_mtkfb_mmp_events(void);
void init_ddp_mmp_events(void);
#endif

static struct notifier_block healthd_bl_fb_notifier;
static int healthd_bl_fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	INT32 blank;

	MTKFB_INFO("healthd_bl_fb_notifier_callback\n");

	if (event != FB_EVENT_BLANK)
		return 0;

	blank = *(INT32 *)evdata->data;
	MTKFB_INFO("fb_notify(blank=%d)\n", blank);

	switch (blank) {
	case FB_BLANK_UNBLANK:
		mtkfb_clear_lcm();
		DISP_WaitVSYNC();
		mt65xx_leds_brightness_set(MT65XX_LED_TYPE_LCD, LED_HALF);
		break;
	case FB_BLANK_POWERDOWN:
		mt65xx_leds_brightness_set(MT65XX_LED_TYPE_LCD, LED_OFF);
		break;
	default:
		break;
	}
	return 0;
}

/* Register both the driver and the device */
int __init mtkfb_init(void)
{
	int r = 0;

	MSG_FUNC_ENTER();

#ifdef DEFAULT_MMP_ENABLE
	MMProfileEnable(1);
	init_mtkfb_mmp_events();
	init_ddp_mmp_events();
	MMProfileStart(1);
#endif

	/* Register the driver with LDM */

	if (platform_driver_register(&mtkfb_driver)) {
		MTKFB_WRAN("failed to register mtkfb driver\n");
		r = -ENODEV;
		goto exit;
	}

	if (get_boot_mode() == RECOVERY_BOOT)
		blank_skip_count = 1;

	healthd_bl_fb_notifier.notifier_call = healthd_bl_fb_notifier_callback;
	if ((get_boot_mode() == KERNEL_POWER_OFF_CHARGING_BOOT) ||
	    (get_boot_mode() == LOW_POWER_OFF_CHARGING_BOOT)) {
		blank_skip_count = 1;
		r = fb_register_client(&healthd_bl_fb_notifier);
		if (r)
			MTKFB_WRAN("register healthd_bl_fb_notifier failed! ret(%d)\n", r);
	}

	DBG_Init();

	ConfigPara_Init();	/* In order to Trigger Display Customization Tool.. */

exit:
	MSG_FUNC_LEAVE();
	return r;
}

static void __exit mtkfb_cleanup(void)
{
	MSG_FUNC_ENTER();

	platform_driver_unregister(&mtkfb_driver);

	if ((get_boot_mode() == KERNEL_POWER_OFF_CHARGING_BOOT) ||
	    (get_boot_mode() == LOW_POWER_OFF_CHARGING_BOOT))
		fb_unregister_client(&healthd_bl_fb_notifier);

	kthread_stop(screen_update_task);
	if (esd_recovery_task)
		kthread_stop(esd_recovery_task);

	DBG_Deinit();

	ConfigPara_Deinit();	/* clean up Display Customization Tool... */

	MSG_FUNC_LEAVE();
}

module_init(mtkfb_init);
module_exit(mtkfb_cleanup);

MODULE_DESCRIPTION("MEDIATEK framebuffer driver");
MODULE_AUTHOR("Zaikuo Wang <zaikuo.wang@mediatek.com>");
MODULE_LICENSE("GPL");
