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

#include <lcm_drv.h>

extern LCM_DRIVER rm69080_dsi_cmd_lcm_drv;
extern LCM_DRIVER rm67160_dsi_cmd_lcm_drv;

LCM_DRIVER *lcm_driver_list[] = {
#if defined(RM69080_DSI_CMD)
	&rm69080_dsi_cmd_lcm_drv,
#endif
#if defined(RM67160_DSI_CMD)
	&rm67160_dsi_cmd_lcm_drv,
#endif
};

#define LCM_COMPILE_ASSERT(condition) LCM_COMPILE_ASSERT_X(condition, __LINE__)
#define LCM_COMPILE_ASSERT_X(condition, line) LCM_COMPILE_ASSERT_XX(condition, line)
#define LCM_COMPILE_ASSERT_XX(condition, line) char assertion_failed_at_line_##line[(condition) ? 1 : -1]

unsigned int lcm_count = sizeof(lcm_driver_list) / sizeof(LCM_DRIVER *);
LCM_COMPILE_ASSERT(0 != sizeof(lcm_driver_list) / sizeof(LCM_DRIVER *));
