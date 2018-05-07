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

#include <linux/pm.h>
#include <linux/mm.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/setup.h>
#include <mach/mt_reg_base.h>
#include <mach/memory.h>
#include <mach/irqs.h>

extern struct smp_operations mt_smp_ops;

extern void arm_machine_restart(enum reboot_mode, const char *cmd);
extern struct sys_timer mt6572_timer;
extern void mt_power_off(void);
extern void mt_fixup(struct tag *tags, char **cmdline);
extern void mt_reserve(void);
extern void __init mt_timer_init(void);

static void __init mt_init(void)
{
	pm_power_off = mt_power_off;
	panic_on_oops = 1;
}

static struct map_desc mt_io_desc[] __initdata = {
	/* MM Subsys */
	{
	 .virtual = MMSYS_CONFIG_BASE,
	 .pfn = __phys_to_pfn(IO_VIRT_TO_PHYS(MMSYS_CONFIG_BASE)),
	 .length = SZ_128K,
	 .type = MT_DEVICE},

	/* G3D Sys */
	{
	 .virtual = G3D_CONFIG_BASE,
	 .pfn = __phys_to_pfn(IO_VIRT_TO_PHYS(G3D_CONFIG_BASE)),
	 .length = SZ_128K,
	 .type = MT_DEVICE},

	/* Perisys */
	{
	 .virtual = AP_DMA_BASE,
	 .pfn = __phys_to_pfn(IO_VIRT_TO_PHYS(AP_DMA_BASE)),
	 .length = SZ_2M,
	 .type = MT_DEVICE},

	/* infrasys AO */
	{
	 .virtual = TOPCKGEN_BASE,
	 .pfn = __phys_to_pfn(IO_VIRT_TO_PHYS(TOPCKGEN_BASE)),
	 .length = SZ_128K,
	 .type = MT_DEVICE},

	/* infrasys */
	{
	 .virtual = APARM_BASE,
	 .pfn = __phys_to_pfn(IO_VIRT_TO_PHYS(APARM_BASE)),
	 .length = SZ_1M,
	 .type = MT_DEVICE},

	/* Connsys */
	{
	 .virtual = CONN_BT_PKV_BASE,
	 .pfn = __phys_to_pfn(IO_VIRT_TO_PHYS(CONN_BT_PKV_BASE)),
	 .length = SZ_1M,
	 .type = MT_DEVICE},

	/* Ram Console */
	{
	 .virtual = RAM_CONSOLE_BASE,
	 .pfn = __phys_to_pfn(0x01000000),
	 .length = SZ_16K,
	 .type = MT_UNCACHED},

#ifdef CONFIG_MT6572_FPGA_CA7
	/* Dummy Region */
	{
	 .virtual = 0xF6000000,
	 .pfn = __phys_to_pfn(IO_VIRT_TO_PHYS(0xF6000000)),
	 .length = SZ_2M,
	 .type = MT_DEVICE},
#endif				/* CONFIG_MT6572_FPGA_CA7 */

	/* Device Info */
	{
	 .virtual = DEVINFO_BASE,
	 .pfn = __phys_to_pfn(0x08000000),
	 .length = SZ_16K,
	 .type = MT_DEVICE}
};

static void __init mt_map_io(void)
{
	iotable_init(mt_io_desc, ARRAY_SIZE(mt_io_desc));
}

MACHINE_START(MT2601, "MT2601")
    .atag_offset = 0x00000100,
    .map_io = mt_map_io,
    .smp = smp_ops(mt_smp_ops),
    .init_irq = mt_init_irq,
    .init_time = mt_timer_init,
    .init_machine = mt_init,
    .fixup = mt_fixup,
    .restart = arm_machine_restart,
    .reserve = mt_reserve,
MACHINE_END
