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

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/cpu.h>
#include <linux/smp.h>
#include <linux/irqchip/arm-gic.h>
#include <asm/system.h>
#include <asm/mach/irq.h>
#include "mach/mt_cirq.h"
#include "mach/mt_reg_base.h"
#include "mach/irqs.h"
#include <mt-plat/sync_write.h>
#include "mach/mt_sleep.h"


/*
 * Definition
 */

#define CIRQ_DEBUG   0
#if (CIRQ_DEBUG == 1)
#ifdef CTP
#define dbgmsg dbg_print
#else
#define dbgmsg printk
#endif
#define print_func() do { \
    dbgmsg("in %s\n", __func__); \
} while (0)
#else
#define dbgmsg(...)
#define print_func() do { } while (0)
#endif


/*
 * Define Data Structure
 */
struct mt_cirq_driver {
	struct device_driver driver;
	const struct platform_device_id *id_table;
};


/*
 * Define Global Variable
 */
static struct mt_cirq_driver mt_cirq_drv = {
	.driver = {
		   .name = "cirq",
		   .bus = &platform_bus_type,
		   .owner = THIS_MODULE,
		   },
	.id_table = NULL,
};


/*
 * Define Function
 */

/*
 * mt_cirq_read_status:
 * @cirq_num:
 */
unsigned int mt_cirq_read_status(unsigned int cirq_num)
{
	print_func();
	return 0;
}

/*
 * mt_cirq_get_status: Get the specified SYS_CIRQ status
 * @cirq_num: the SYS_CIRQ number to get
 */
unsigned int mt_cirq_get_status(unsigned int cirq_num)
{
	unsigned int st;
	unsigned int bit = 1 << (cirq_num % 32);

	if (cirq_num < CIRQ_MAX_CHANNEL) {
		st = readl((volatile void __iomem *)((cirq_num / 32) * 4 + CIRQ_STA_BASE));
		return (st & bit);
	} else {
		dbgmsg("[CIRQ] get_status number %d exceeds the max number!\n", cirq_num);
		return 0;
	}
}

/*
 * mt_cirq_ack_all: Ack all the interrupt on SYS_CIRQ
 */
void mt_cirq_ack_all(void)
{
	unsigned int i;

	for (i = 0; i < CIRQ_CTRL_REG_NUM; i++) {
		writel_relaxed(0xFFFFFFFF, (volatile void __iomem *)(CIRQ_ACK_BASE + (i * 4)));
	}
	dsb();

	return;
}

/*
 * mt_cirq_ack: Ack the interrupt on SYS_CIRQ
 * @cirq_num: the SYS_CIRQ number to ack
 */
void mt_cirq_ack(unsigned int cirq_num)
{
	unsigned int bit = 1 << (cirq_num % 32);

	print_func();

	if (cirq_num < CIRQ_MAX_CHANNEL) {
		mt_reg_sync_writel(bit, (cirq_num / 32) * 4 + CIRQ_ACK_BASE);
	} else {
		dbgmsg("[CIRQ] ack number %d exceeds the max number!\n", cirq_num);
	}

	return;
}

/*
 * mt_cirq_get_mask: Get the specified SYS_CIRQ mask
 * @cirq_num: the SYS_CIRQ number to get
 */
unsigned int mt_cirq_get_mask(unsigned int cirq_num)
{
	unsigned int st;
	unsigned int bit = 1 << (cirq_num % 32);

	if (cirq_num < CIRQ_MAX_CHANNEL) {
		st = readl((volatile void __iomem *)((cirq_num / 32) * 4 + CIRQ_MASK_BASE));
		return (st & bit);
	} else {
		dbgmsg("[CIRQ] get_mask number %d exceeds the max number!\n", cirq_num);
		return 0;
	}
}

/*
 * mt_cirq_mask_all: Mask all interrupts on SYS_CIRQ.
 */
void mt_cirq_mask_all(void)
{
	unsigned int i;

	for (i = 0; i < CIRQ_CTRL_REG_NUM; i++) {
		writel_relaxed(0xFFFFFFFF, (volatile void __iomem *)(CIRQ_MASK_SET_BASE + (i * 4)));
	}
	dsb();

	return;
}

/*
 * mt_cirq_unmask_all: Unmask all interrupts on SYS_CIRQ.
 */
void mt_cirq_unmask_all(void)
{
	unsigned int i;

	for (i = 0; i < CIRQ_CTRL_REG_NUM; i++) {
		writel_relaxed(0xFFFFFFFF, (volatile void __iomem *)(CIRQ_MASK_CLR_BASE + (i * 4)));
	}
	dsb();

	return;
}

/*
 * mt_cirq_mask: Mask the specified SYS_CIRQ.
 * @cirq_num: the SYS_CIRQ number to mask
 */
void mt_cirq_mask(unsigned int cirq_num)
{
	unsigned int bit = 1 << (cirq_num % 32);

	print_func();

	if (cirq_num < CIRQ_MAX_CHANNEL) {
		mt_reg_sync_writel(bit, (cirq_num / 32) * 4 + CIRQ_MASK_SET_BASE);
		/* mt_reg_sync_writel(bit, (cirq_num / 32) * 4 + CIRQ_ACK_BASE); */
	} else {
		dbgmsg("[CIRQ] mask number %d exceeds the max number!\n", cirq_num);
	}

	return;
}

/*
 * mt_cirq_unmask: Unmask the specified SYS_CIRQ.
 * @cirq_num: the SYS_CIRQ number to unmask
 */
void mt_cirq_unmask(unsigned int cirq_num)
{
	unsigned int bit = 1 << (cirq_num % 32);

	print_func();

	if (cirq_num < CIRQ_MAX_CHANNEL) {
		/* mt_reg_sync_writel(bit, (cirq_num / 32) * 4 + CIRQ_ACK_BASE); */
		mt_reg_sync_writel(bit, (cirq_num / 32) * 4 + CIRQ_MASK_CLR_BASE);
	} else {
		dbgmsg("[CIRQ] unmask number %d exceeds the max number!\n", cirq_num);
	}

	return;
}

/*
 * mt_cirq_get_sens: Get the specified SYS_CIRQ sensitivity
 * @cirq_num: the SYS_CIRQ number to get
 */
unsigned int mt_cirq_get_sens(unsigned int cirq_num)
{
	unsigned int st;
	unsigned int bit = 1 << (cirq_num % 32);

	if (cirq_num < CIRQ_MAX_CHANNEL) {
		st = readl((volatile void __iomem *)((cirq_num / 32) * 4 + CIRQ_SENS_BASE));
		return (st & bit);
	} else {
		dbgmsg("[CIRQ] get_sens number %d exceeds the max number!\n", cirq_num);
		return 0;
	}
}

/*
 * mt_cirq_set_sens: Set the sensitivity for the specified SYS_CIRQ number.
 * @cirq_num: the SYS_CIRQ number to set
 * @sens: sensitivity to set
 */
void mt_cirq_set_sens(unsigned int cirq_num, unsigned int sens)
{
	unsigned int base;
	unsigned int bit = 1 << (cirq_num % 32);

	print_func();

	if (cirq_num < CIRQ_MAX_CHANNEL) {
		if (sens == MT_EDGE_SENSITIVE) {
			base = (cirq_num / 32) * 4 + CIRQ_SENS_CLR_BASE;
		} else if (sens == MT_LEVEL_SENSITIVE) {
			base = (cirq_num / 32) * 4 + CIRQ_SENS_SET_BASE;
		} else {
			dbgmsg("[CIRQ] set_sens invalid sensitivity value %d\n", sens);
			return;
		}
	} else {
		dbgmsg("[CIRQ] set_sens number %d exceeds the max number!\n", cirq_num);
		return;
	}

	mt_reg_sync_writel(bit, base);
	dbgmsg("[CIRQ] set_sens number %d with sens %d; addr:%x, bit:%d\n", cirq_num, sens, base,
	       bit);

	return;
}

/*
 * mt_cirq_get_pol: Get the specified SYS_CIRQ polarity
 * @cirq_num: the SYS_CIRQ number to get
 */
unsigned int mt_cirq_get_pol(unsigned int cirq_num)
{
	unsigned int st;
	unsigned int bit = 1 << (cirq_num % 32);

	if (cirq_num < CIRQ_MAX_CHANNEL) {
		st = readl((volatile void __iomem *)((cirq_num / 32) * 4 + CIRQ_POL_BASE));
		return (st & bit);
	} else {
		dbgmsg("[CIRQ] get_pol number %d exceeds the max number!\n", cirq_num);
		return 0;
	}
}

/*
 * mt_cirq_set_pol: Set the polarity for the specified SYS_CIRQ number.
 * @cirq_num: the SYS_CIRQ number to set
 * @pol: polarity to set
 */
void mt_cirq_set_pol(unsigned int cirq_num, unsigned int pol)
{
	unsigned int base;
	unsigned int bit = 1 << (cirq_num % 32);

	print_func();

	if (cirq_num < CIRQ_MAX_CHANNEL) {
		if (pol == MT_CIRQ_POL_NEG) {
			base = (cirq_num / 32) * 4 + CIRQ_POL_CLR_BASE;
		} else if (pol == MT_CIRQ_POL_POS) {
			base = (cirq_num / 32) * 4 + CIRQ_POL_SET_BASE;
		} else {
			dbgmsg("[CIRQ] set_pol invalid polarity value %d\n", pol);
			return;
		}
	} else {
		dbgmsg("[CIRQ] set_pol number %d exceeds the max number!\n", cirq_num);
		return;
	}

	mt_reg_sync_writel(bit, base);
	dbgmsg("[CIRQ] set_pol number %d with pol %d; addr:%x, bit:%d\n", cirq_num, pol, base, bit);

	return;
}

/*
 * mt_cirq_enable: Enable SYS_CIRQ
 */
void mt_cirq_enable(void)
{
	unsigned int st;

	print_func();

	mt_cirq_ack_all();

	st = readl((volatile void __iomem *)CIRQ_CON);
	st |= (CIRQ_CON_EN << CIRQ_CON_EN_BITS);
	mt_reg_sync_writel((st & CIRQ_CON_BITS_MASK), CIRQ_CON);

	return;
}

/*
 * mt_cirq_disable: Disable SYS_CIRQ
 */
void mt_cirq_disable(void)
{
	unsigned int st;

	print_func();

	st = readl((volatile void __iomem *)CIRQ_CON);
	st &= ~(CIRQ_CON_EN << CIRQ_CON_EN_BITS);
	mt_reg_sync_writel((st & CIRQ_CON_BITS_MASK), CIRQ_CON);

	return;
}

/*
 * mt_cirq_disable: Flush interrupt from SYS_CIRQ to GIC
 */
void mt_cirq_flush(void)
{
	unsigned int i;
	unsigned int st;

	print_func();

	for (i = 0; i < CIRQ_CTRL_REG_NUM; i++) {
		st = readl((volatile void __iomem *)(CIRQ_STA_BASE + (i * 4)));

		mt_reg_sync_writel(st << (CIRQ_TO_IRQ_NUM(0) % 32),
				       GIC_DIST_BASE + GIC_DIST_PENDING_SET +
				       (CIRQ_TO_IRQ_NUM(i * 32) / 32 * 4));
		if (CIRQ_TO_IRQ_NUM(0) % 32 != 0) {
			if (i != CIRQ_CTRL_REG_NUM - 1
			    || (CIRQ_TO_IRQ_NUM(CIRQ_MAX_CHANNEL - 1) % 32) <
			    (CIRQ_TO_IRQ_NUM(0) % 32))
				mt_reg_sync_writel(st >> (32 - (CIRQ_TO_IRQ_NUM(0) % 32)),
						       GIC_DIST_BASE + GIC_DIST_PENDING_SET +
						       (CIRQ_TO_IRQ_NUM((i + 1) * 32) / 32 * 4));
		}
	}
	mt_cirq_ack_all();

	return;
}

void mt_cirq_clone_pol(void)
{
	unsigned int cirq_num, irq_num;
	unsigned int st;
	unsigned int bit;

	print_func();

	for (cirq_num = 0; cirq_num < CIRQ_MAX_CHANNEL; cirq_num++) {
		irq_num = CIRQ_TO_IRQ_NUM(cirq_num);

		if (cirq_num == 0 || irq_num % 32 == 0) {
			st = readl((volatile void __iomem *)(INT_POL_CTL0 + ((irq_num - GIC_PRIVATE_SIGNALS) / 32 * 4)));
			dbgmsg("[CIRQ] clone_pol read pol 0x%08x at cirq %d (irq %d)\n", st,
			       cirq_num, irq_num);
		}

		bit = 0x1 << ((irq_num - GIC_PRIVATE_SIGNALS) % 32);

		if (st & bit) {
			mt_cirq_set_pol(cirq_num, MT_CIRQ_POL_NEG);
			dbgmsg("[CIRQ] clone_pol set cirq %d (irq %d) as negative\n", cirq_num,
			       irq_num);
		} else {
			mt_cirq_set_pol(cirq_num, MT_CIRQ_POL_POS);
			dbgmsg("[CIRQ] clone_pol set cirq %d (irq %d) as postive\n", cirq_num,
			       irq_num);
		}
	}

	return;
}

void mt_cirq_clone_sens(void)
{
	unsigned int cirq_num, irq_num;
	unsigned int st;
	unsigned int bit;

	print_func();

	for (cirq_num = 0; cirq_num < CIRQ_MAX_CHANNEL; cirq_num++) {
		irq_num = CIRQ_TO_IRQ_NUM(cirq_num);

		if (cirq_num == 0 || irq_num % 16 == 0) {
			st = readl((volatile void __iomem *)(GIC_DIST_BASE + GIC_DIST_CONFIG + (irq_num / 16 * 4)));
			dbgmsg("[CIRQ] clone_sens read sens 0x%08x at cirq %d (irq %d)\n", st,
			       cirq_num, irq_num);
		}

		bit = 0x2 << ((irq_num % 16) * 2);

		if (st & bit) {
			mt_cirq_set_sens(cirq_num, MT_EDGE_SENSITIVE);
			dbgmsg("[CIRQ] clone_sens set cirq %d (irq %d) as edge\n", cirq_num,
			       irq_num);
		} else {
			mt_cirq_set_sens(cirq_num, MT_LEVEL_SENSITIVE);
			dbgmsg("[CIRQ] clone_sens set cirq %d (irq %d) as level\n", cirq_num,
			       irq_num);
		}
	}

	return;
}

void mt_cirq_clone_mask(void)
{
	unsigned int cirq_num, irq_num;
	unsigned int st;
	unsigned int bit;

	print_func();

	for (cirq_num = 0; cirq_num < CIRQ_MAX_CHANNEL; cirq_num++) {
		irq_num = CIRQ_TO_IRQ_NUM(cirq_num);

		if (cirq_num == 0 || irq_num % 32 == 0) {
			st = readl((volatile void __iomem *)(GIC_DIST_BASE + GIC_DIST_ENABLE_SET + (irq_num / 32 * 4)));
			dbgmsg("[CIRQ] clone_mask read enable 0x%08x at cirq %d (irq %d)\n", st,
			       cirq_num, irq_num);
		}

		bit = 0x1 << (irq_num % 32);

		if (st & bit) {
			mt_cirq_unmask(cirq_num);
			dbgmsg("[CIRQ] clone_mask unmask cirq %d (irq %d)\n", cirq_num, irq_num);
		} else {
			mt_cirq_mask(cirq_num);
			dbgmsg("[CIRQ] clone_mask mask cirq %d (irq %d)\n", cirq_num, irq_num);
		}
	}

	return;
}

/*
 * mt_cirq_clone_gic: Copy the setting from GIC to SYS_CIRQ
 */
void mt_cirq_clone_gic(void)
{
	mt_cirq_clone_pol();
	mt_cirq_clone_sens();
	mt_cirq_clone_mask();

	return;
}

/*
 * mt_cirq_init: SYS_CIRQ init function
 * always return 0
 */
int mt_cirq_init(void)
{
	int ret;

	printk(KERN_DEBUG "CIRQ init...\n");

	ret = driver_register(&mt_cirq_drv.driver);
	if (ret == 0)
		printk(KERN_DEBUG "CIRQ init done...\n");

	return 0;
}

/*
 * mt_cirq_exit:
 * always return 0
 */
int mt_cirq_exit(void)
{
	return 0;
}
arch_initcall(mt_cirq_init);
EXPORT_SYMBOL(mt_cirq_enable);
EXPORT_SYMBOL(mt_cirq_disable);
EXPORT_SYMBOL(mt_cirq_clone_gic);
EXPORT_SYMBOL(mt_cirq_flush);
