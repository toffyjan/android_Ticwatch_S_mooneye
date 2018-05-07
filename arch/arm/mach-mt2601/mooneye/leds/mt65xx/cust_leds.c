/*
* Copyright (C) 2011-2015 MediaTek Inc.
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

#include <cust_leds.h>
#include <cust_leds_def.h>
#include <mach/mt_pwm.h>

#include <linux/kernel.h>
#include <mach/pmic_mt6329_hw_bank1.h>
#include <mach/pmic_mt6329_sw_bank1.h>
#include <mach/pmic_mt6329_hw.h>
#include <mach/pmic_mt6329_sw.h>
#include <mach/upmu_common_sw.h>
#include <mach/upmu_hw.h>

extern int mtkfb_set_backlight_level(unsigned int level);
/* extern int mtkfb_set_backlight_pwm(int div); */
/* extern int disp_bls_set_backlight(unsigned int level); */
/*
#define ERROR_BL_LEVEL 0xFFFFFFFF

unsigned int brightness_mapping(unsigned int level)
{
	return ERROR_BL_LEVEL;
}
*/
unsigned int brightness_mapping(unsigned int level)
{
	unsigned int mapped_level;

	mapped_level = level;

	return mapped_level;
}

unsigned int Cust_SetBacklight(int level, int div)
{
	return mtkfb_set_backlight_level(level);
}

static struct cust_mt65xx_led cust_led_list[MT65XX_LED_TYPE_TOTAL] = {
	{"red",                MT65XX_LED_MODE_NONE, -1, {0} },
	{"green",              MT65XX_LED_MODE_NONE, -1, {0} },
	{"blue",               MT65XX_LED_MODE_NONE, -1, {0} },
	{"jogball-backlight",  MT65XX_LED_MODE_NONE, -1, {0} },
	{"keyboard-backlight", MT65XX_LED_MODE_NONE, -1, {0} },
	{"button-backlight",   MT65XX_LED_MODE_NONE, -1, {0} },
	{"lcd-backlight",      MT65XX_LED_MODE_CUST_LCM, Cust_SetBacklight, {0} },
};

struct cust_mt65xx_led *get_cust_led_list(void)
{
	return cust_led_list;
}
