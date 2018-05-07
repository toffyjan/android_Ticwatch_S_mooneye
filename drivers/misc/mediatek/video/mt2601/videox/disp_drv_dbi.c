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

#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/semaphore.h>
#include <mach/m4u.h>
#include <linux/delay.h>

#include "disp_drv.h"
#include "disp_drv_platform.h"
#include "disp_drv_log.h"

#include "lcd_drv.h"
#include "lcm_drv.h"
#include "disp_debug.h"

/* --------------------------------------------------------------------------- */
/* Private Variables */
/* --------------------------------------------------------------------------- */

extern LCM_DRIVER *lcm_drv;
extern LCM_PARAMS *lcm_params;
extern BOOL DISP_IsDecoupleMode(void);

/* --------------------------------------------------------------------------- */
/* Private Functions */
/* --------------------------------------------------------------------------- */

static BOOL disp_drv_dbi_init_context(void)
{
	if (lcm_drv != NULL && lcm_params != NULL)
		return TRUE;
	else
		DISP_LOG_PRINT("%s, lcm_drv=0x%08x, lcm_params=0x%08x\n", __func__, (unsigned int)lcm_drv, (unsigned int)lcm_params);

	DISP_LOG_PRINT("%s, lcm_drv=0x%08x\n", __func__, (unsigned int)lcm_drv);
	if (NULL == lcm_drv) {
		pr_err("%s, lcm_drv is NULL\n", __func__);
		return FALSE;
	}

	lcm_drv->get_params(lcm_params);

	return TRUE;
}

static void init_lcd(BOOL isLcdPoweredOn)
{
	/* Config LCD Controller */
	LCD_CHECK_RET(LCD_Init(isLcdPoweredOn));

	LCD_CHECK_RET(LCD_LayerEnable(LCD_LAYER_ALL, FALSE));
	LCD_CHECK_RET(LCD_SetRoiWindow(0, 0, lcm_params->width, lcm_params->height));
}

static DISP_STATUS lcd_config_ddp(UINT32 fbPA)
{
	struct disp_path_config_struct config = { 0 };

	if (DISP_IsDecoupleMode())
		config.srcModule = DISP_MODULE_RDMA0;
	else
		config.srcModule = DISP_MODULE_OVL;

	config.bgROI.x = 0;
	config.bgROI.y = 0;
	config.bgROI.width = lcm_params->width;
	config.bgROI.height = lcm_params->height;
	config.bgColor = 0x0;	/* background color */

	config.pitch = ALIGN_TO(lcm_params->width, MTK_FB_ALIGNMENT) * 2;
	config.srcROI.x = 0;
	config.srcROI.y = 0;
	config.srcROI.height = lcm_params->height;
	config.srcROI.width = lcm_params->width;
	config.ovl_config.source = OVL_LAYER_SOURCE_MEM;

	config.ovl_config.layer = DDP_OVL_LAYER_MUN - 1;
	config.ovl_config.layer_en = 1;
	config.ovl_config.fmt = eRGB565;
	config.ovl_config.addr = fbPA;
	config.ovl_config.source = OVL_LAYER_SOURCE_MEM;
	config.ovl_config.src_x = 0;
	config.ovl_config.src_y = 0;
	config.ovl_config.dst_x = 0;	/* ROI */
	config.ovl_config.dst_y = 0;
	config.ovl_config.dst_w = lcm_params->width;
	config.ovl_config.dst_h = lcm_params->height;
	config.ovl_config.src_pitch = ALIGN_TO(lcm_params->width, MTK_FB_ALIGNMENT) * 2;	/* pixel number */
	config.ovl_config.keyEn = 0;
	config.ovl_config.key = 0xFF;	/* color key */
	config.ovl_config.aen = 0;	/* alpha enable */
	config.ovl_config.alpha = 0;

	LCD_LayerSetAddress(DDP_OVL_LAYER_MUN - 1, fbPA);
	LCD_LayerSetFormat(DDP_OVL_LAYER_MUN - 1, LCD_LAYER_FORMAT_RGB565);
	LCD_LayerSetOffset(DDP_OVL_LAYER_MUN - 1, 0, 0);
	LCD_LayerSetSize(DDP_OVL_LAYER_MUN - 1, lcm_params->width, lcm_params->height);
	LCD_LayerSetPitch(DDP_OVL_LAYER_MUN - 1, ALIGN_TO(lcm_params->width, MTK_FB_ALIGNMENT) * 2);
	LCD_LayerEnable(DDP_OVL_LAYER_MUN - 1, TRUE);

	config.dstModule = DISP_MODULE_DBI;	/* DISP_MODULE_WDMA1 */
	config.outFormat = RDMA_OUTPUT_FORMAT_ARGB;
	disp_path_config(&config);

	/* Config FB_Layer port to be physical. */
	{
		M4U_PORT_STRUCT portStruct;

		portStruct.ePortID = M4U_PORT_LCD_OVL;	/* hardware port ID, defined in M4U_PORT_ID_ENUM */
		portStruct.Virtuality = 1;
		portStruct.Security = 0;
		portStruct.domain = 3;	/* domain : 0 1 2 3 */
		portStruct.Distance = 1;
		portStruct.Direction = 0;
		m4u_config_port(&portStruct);
	}
	/* hook m4u debug callback function */
	m4u_set_tf_callback(M4U_CLNTMOD_DISP, &disp_m4u_dump_reg);

	DISP_LOG_PRINT("%s, config done\n", __func__);

	return DISP_STATUS_OK;
}

static void init_lcd_te_control(void)
{
	const LCM_DBI_PARAMS *dbi = &(lcm_params->dbi);

	/* The board may not connect to LCM in META test mode,
	   force disalbe TE to avoid blocked in LCD controller
	 */
	/* but for lk, the boot mode selection is done after lcd init, so we have to disable te always in lk. */
	LCD_CHECK_RET(LCD_TE_Enable(FALSE));
	if (!DISP_IsLcmFound())
		return;

	if (LCM_DBI_TE_MODE_DISABLED == dbi->te_mode) {
		LCD_CHECK_RET(LCD_TE_Enable(FALSE));
		return;
	}

	if (LCM_DBI_TE_MODE_VSYNC_ONLY == dbi->te_mode) {
		LCD_CHECK_RET(LCD_TE_SetMode(LCD_TE_MODE_VSYNC_ONLY));
	} else if (LCM_DBI_TE_MODE_VSYNC_OR_HSYNC == dbi->te_mode) {
		LCD_CHECK_RET(LCD_TE_SetMode(LCD_TE_MODE_VSYNC_OR_HSYNC));
		LCD_CHECK_RET(LCD_TE_ConfigVHSyncMode(dbi->te_hs_delay_cnt,
						      dbi->te_vs_width_cnt,
						      (LCD_TE_VS_WIDTH_CNT_DIV) dbi->te_vs_width_cnt_div));
	} else
		ASSERT(0);

	LCD_CHECK_RET(LCD_TE_SetEdgePolarity(dbi->te_edge_polarity));
	LCD_CHECK_RET(LCD_TE_Enable(TRUE));
}

static void init_io_driving_current(void)
{
	LCD_CHECK_RET(LCD_Set_DrivingCurrent(lcm_params));
}

/* --------------------------------------------------------------------------- */
/* DBI Display Driver Public Functions */
/* --------------------------------------------------------------------------- */
static DISP_STATUS dbi_init(UINT32 fbVA, UINT32 fbPA, BOOL isLcmInited)
{
	if (!disp_drv_dbi_init_context())
		return DISP_STATUS_NOT_IMPLEMENTED;

	LCD_MIPI_PowerOff();

	lcd_config_ddp(fbPA);

	init_io_driving_current();
	init_lcd(isLcmInited);

	if (NULL != lcm_drv->init && !isLcmInited)
		lcm_drv->init();

	init_lcd_te_control();

	return DISP_STATUS_OK;
}

static DISP_STATUS dbi_enable_power(BOOL enable)
{
	if (enable) {
		LCD_CHECK_RET(LCD_PowerOn());
		LCD_RestoreRegisters();
		init_io_driving_current();
	} else {
		LCD_WaitForEngineNotBusy();
		LCD_BackupRegisters();
		LCD_CHECK_RET(LCD_PowerOff());
	}
	return DISP_STATUS_OK;
}

static DISP_STATUS dbi_update_screen(BOOL isMuextLocked)
{
	LCD_CHECK_RET(LCD_StartTransfer(FALSE, isMuextLocked));

	return DISP_STATUS_OK;
}

static UINT32 dbi_get_working_buffer_size(void)
{
	return 0;
}

static UINT32 dbi_get_working_buffer_bpp(void)
{
	return 0;
}

static PANEL_COLOR_FORMAT dbi_get_panel_color_format(void)
{
	disp_drv_dbi_init_context();

	switch (lcm_params->dbi.data_format.format) {
	case LCM_DBI_FORMAT_RGB332:
		return PANEL_COLOR_FORMAT_RGB332;
	case LCM_DBI_FORMAT_RGB444:
		return PANEL_COLOR_FORMAT_RGB444;
	case LCM_DBI_FORMAT_RGB565:
		return PANEL_COLOR_FORMAT_RGB565;
	case LCM_DBI_FORMAT_RGB666:
		return PANEL_COLOR_FORMAT_RGB666;
	case LCM_DBI_FORMAT_RGB888:
		return PANEL_COLOR_FORMAT_RGB888;
	default:
		ASSERT(0);
	}
	return PANEL_COLOR_FORMAT_RGB888;
}

static UINT32 dbi_get_dithering_bpp(void)
{
	return PANEL_COLOR_FORMAT_TO_BPP(dbi_get_panel_color_format());
}

DISP_STATUS dbi_capture_framebuffer(UINT32 pvbuf, UINT32 bpp)
{
	return DISP_STATUS_OK;
}

static BOOL dbi_esd_check(void)
{
	BOOL result = false;

	result = lcm_drv->esd_check();

	return result;
}

const DISP_IF_DRIVER *DISP_GetDriverDBI(void)
{
	static const DISP_IF_DRIVER DBI_DISP_DRV = {
		.init = dbi_init,
		.enable_power = dbi_enable_power,
		.update_screen = dbi_update_screen,

		.get_working_buffer_size = dbi_get_working_buffer_size,
		.get_working_buffer_bpp = dbi_get_working_buffer_bpp,
		.get_panel_color_format = dbi_get_panel_color_format,
		.init_te_control = init_lcd_te_control,
		.get_dithering_bpp = dbi_get_dithering_bpp,
		.capture_framebuffer = dbi_capture_framebuffer,
		.esd_check = dbi_esd_check,
	};

	return &DBI_DISP_DRV;
}
