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

#include <mach/mt_reg_base.h>
#include <mt-plat/sync_write.h>
#include <mach/bus_fabric.h>

void ap_md_bus_config(void)
{
	unsigned int top_axi_bus_ctrl;

	top_axi_bus_ctrl = readl(TOP_AXI_BUS_CTRL);
	top_axi_bus_ctrl &= 0xFFFFF5AD;
	writel(top_axi_bus_ctrl, TOP_AXI_BUS_CTRL);
}
EXPORT_SYMBOL(ap_md_bus_config);
