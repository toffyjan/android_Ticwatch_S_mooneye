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

#ifndef __DISP_DRV_PLATFORM_H__
#define __DISP_DRV_PLATFORM_H__

#include <linux/dma-mapping.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_reg_base.h>
#include <mach/mt_clkmgr.h>
#include <mach/mt_irq.h>

/* LCD HW feature options */
/* #define MTK_LCD_HW_3D_SUPPORT */

#define ALIGN_TO(x, n)   (((x) + ((n) - 1)) & ~((n) - 1))
#define MTK_FB_ALIGNMENT 32
#define MTK_FB_SYNC_SUPPORT

/* #define MTK_OVL_DECOUPLE_SUPPORT */

#endif				/* __DISP_DRV_PLATFORM_H__ */
