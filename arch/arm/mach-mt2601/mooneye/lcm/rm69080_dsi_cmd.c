/*
* Copyright (C) 2014 MediaTek Inc.
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

#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/cdev.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include "lcm_drv.h"

/* --------------------------------------------------------------------------- */
/* Local Constants */
/* --------------------------------------------------------------------------- */

#define FRAME_WIDTH  (400)
#define FRAME_HEIGHT (400)

#define PHYSICAL_WIDTH  (35)	/* mm */
#define PHYSICAL_HEIGHT (35)	/* mm */

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#define LCM_PRINT pr_warn

/* --------------------------------------------------------------------------- */
/* Local Variables */
/* --------------------------------------------------------------------------- */

static LCM_UTIL_FUNCS lcm_util;
static struct class *lcm_class;
static int lcm_init_flag;
static int hl;

static void lcm_enter_hl(void);
static void lcm_exit_hl(void);

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n)           (lcm_util.udelay(n))
#define MDELAY(n)           (lcm_util.mdelay(n))

#define LCM_ID             (0x00)

/* --------------------------------------------------------------------------- */
/* Local Functions */
/* --------------------------------------------------------------------------- */

#define dsi_set_cmdq(pdata, queue_size, force_update)       lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define read_reg_v2(cmd, buffer, buffer_size)               lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

/* --------------------------------------------------------------------------- */
/* LCM Driver Implementations */
/* --------------------------------------------------------------------------- */
static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_get_params(LCM_PARAMS *params)
{
	memset(params, 0, sizeof(LCM_PARAMS));

	params->type = LCM_TYPE_DSI;
	params->width = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

	params->physical_width = PHYSICAL_WIDTH;
	params->physical_height = PHYSICAL_HEIGHT;

	params->dsi.mode = CMD_MODE;
	params->dsi.LANE_NUM = LCM_ONE_LANE;
	params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;
	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.PLL_CLOCK = 221;
}

static void init_lcm_registers(void)
{
	unsigned int data_array[16];

	data_array[0] = 0x05FE1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00051500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x07FE1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x6D071500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0AFE1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1B1C1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00FE1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00351500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00511500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00110500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(150);

	data_array[0] = 0x00290500;
	dsi_set_cmdq(data_array, 1, 1);
}

static void lcm_init(void)
{
	int ret;
	SET_RESET_PIN(1);
	SET_RESET_PIN(0);
	MDELAY(1);
	SET_RESET_PIN(1);
	MDELAY(5);

	lcm_init_flag = 1;

	init_lcm_registers();
}

static void lcm_suspend(void)
{
	unsigned int data_array[16];

	data_array[0] = 0x00280500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00100500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(120);

	data_array[0] = 0x014F1500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(10);

	SET_RESET_PIN(1);
	SET_RESET_PIN(0);
	MDELAY(1);
	SET_RESET_PIN(1);
	MDELAY(5);
}

static void lcm_resume(void)
{
	lcm_init();
}

static void lcm_update(unsigned int x, unsigned int y, unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = (x0 >> 8) & 0xFF;
	unsigned char x0_LSB = x0 & 0xFF;
	unsigned char x1_MSB = (x1 >> 8) & 0xFF;
	unsigned char x1_LSB = x1 & 0xFF;
	unsigned char y0_MSB = (y0 >> 8) & 0xFF;
	unsigned char y0_LSB = y0 & 0xFF;
	unsigned char y1_MSB = (y1 >> 8) & 0xFF;
	unsigned char y1_LSB = y1 & 0xFF;

	unsigned int data_array[16];

	data_array[0] = 0x00053902;
	data_array[1] = (x1_MSB << 24) | (x0_LSB << 16) | (x0_MSB << 8) | 0x2A;
	data_array[2] = x1_LSB;
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0] = 0x00053902;
	data_array[1] = (y1_MSB << 24) | (y0_LSB << 16) | (y0_MSB << 8) | 0x2B;
	data_array[2] = y1_LSB;
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0] = 0x002C3909;
	dsi_set_cmdq(data_array, 1, 0);

}

static void lcm_setbacklight(unsigned int level)
{
	unsigned int data_array[16];

	LCM_PRINT("lcm_setbacklight = %d\n", level);
	if (lcm_init_flag == 1) {
		lcm_init_flag = 0;
		data_array[0] = 0x20531500;
		dsi_set_cmdq(data_array, 1, 1);
	}

	if (level > 255)
		level = 255;

	if (level == 255) {
		hl = 1;
		lcm_enter_hl();
	} else {
		if (hl == 1) {
			hl = 0;
			lcm_exit_hl();
		}

		data_array[0] = 0x00FE1500;
		dsi_set_cmdq(data_array, 1, 1);
		data_array[0] = 0x00511500 | (level << 24);
		dsi_set_cmdq(data_array, 1, 1);
	}
}

static unsigned int lcm_compare_id(void)
{
	unsigned char buffer[3] = { 0 };
	unsigned int data_array[16];

	SET_RESET_PIN(1);
	SET_RESET_PIN(0);
	MDELAY(1);
	SET_RESET_PIN(1);
	MDELAY(5);

	data_array[0] = 0x00033700;
	dsi_set_cmdq(data_array, 1, 1);

	read_reg_v2(0x04, buffer, 3);

	return LCM_ID == buffer[1];
}

static void lcm_read_fb(unsigned char *buffer)
{
	unsigned int array[2];

	array[0] = 0x000A3700;
	dsi_set_cmdq(array, 1, 1);

	read_reg_v2(0x2E, buffer, 10);
	read_reg_v2(0x3E, buffer + 10, 10);
	read_reg_v2(0x3E, buffer + 10 * 2, 10);
	read_reg_v2(0x3E, buffer + 10 * 3, 10);
	read_reg_v2(0x3E, buffer + 10 * 4, 10);
	read_reg_v2(0x3E, buffer + 10 * 5, 10);
}

static void lcm_enter_idle(void)
{
	unsigned int data_array[16];

	data_array[0] = 0x00390500;
	dsi_set_cmdq(data_array, 1, 1);
}

static void lcm_exit_idle(void)
{
	unsigned int data_array[16];
	data_array[0] = 0x00380500;
	dsi_set_cmdq(data_array, 1, 1);
}

static void lcm_enter_hl(void)
{
	unsigned int data_array[16];

	data_array[0] = 0x05fe1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x01c01500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x17c11500;
	dsi_set_cmdq(data_array, 1, 1);
}

static void lcm_exit_hl(void)
{
	unsigned int data_array[16];

	data_array[0] = 0x05fe1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x05c01500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x12c11500;
	dsi_set_cmdq(data_array, 1, 1);
}

/* --------------------------------------------------------------------------- */
/* Get LCM Driver Hooks */
/* --------------------------------------------------------------------------- */
LCM_DRIVER rm69080_dsi_cmd_lcm_drv = {
	.name = "rm69080_dsi_cmd",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params = lcm_get_params,
	.init = lcm_init,
	.suspend = lcm_suspend,
	.resume = lcm_resume,
	.set_backlight = lcm_setbacklight,
	.compare_id = lcm_compare_id,
	.update = lcm_update,
	.read_fb = lcm_read_fb,
	.enter_idle = lcm_enter_idle,
	.exit_idle = lcm_exit_idle,
};
