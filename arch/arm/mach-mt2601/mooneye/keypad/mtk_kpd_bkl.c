/*
 * Copyright (C) 2010 MediaTek, Inc.
 *
 * Author: Terry Chang <terry.chang@mediatek.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <mach/mt_typedefs.h>
#include <mtk_kpd.h>		/* custom file */

#if KPD_DRV_CTRL_BACKLIGHT
void kpd_enable_backlight(void)
{
}

void kpd_disable_backlight(void)
{
}
#endif

/* for META tool */
void kpd_set_backlight(bool onoff, void *val1, void *val2)
{
}
