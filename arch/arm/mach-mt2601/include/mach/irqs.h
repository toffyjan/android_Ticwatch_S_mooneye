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

#ifndef __IRQS_H__
#define __IRQS_H__

#include "mt_irq.h"

/*
 * Define constants.
 */
#define FIQ_START 0
#define CPU_BRINGUP_SGI 1
#define FIQ_SMP_CALL_SGI 13
#define FIQ_DBG_SGI 14
#define GIC_CPU_AIAR 0x20
#define GIC_CPU_AEOI 0x24

#ifdef NR_IRQS
#undef NR_IRQS
#endif
#define NR_IRQS (NR_MT_IRQ_LINE+220)

#ifndef MT_EDGE_SENSITIVE
#define  MT_EDGE_SENSITIVE      (0)
#endif
#ifndef MT_LEVEL_SENSITIVE
#define  MT_LEVEL_SENSITIVE     (1)
#endif
#define MT_POLARITY_LOW     0
#define MT_POLARITY_HIGH    1
#define MT65xx_EDGE_SENSITIVE   MT_EDGE_SENSITIVE
#define MT65xx_LEVEL_SENSITIVE  MT_LEVEL_SENSITIVE
#define MT65xx_POLARITY_LOW     MT_POLARITY_LOW
#define MT65xx_POLARITY_HIGH    MT_POLARITY_HIGH

#if !defined(__ASSEMBLY__)

/*
 * Define data structures.
 */

enum {
	IRQ_MASK_HEADER = 0xF1F1F1F1,
	IRQ_MASK_FOOTER = 0xF2F2F2F2
};

struct mtk_irq_mask {
	unsigned int header;	/* for error checking */
	__u32 mask0;
	__u32 mask1;
	__u32 mask2;
	__u32 mask3;
	__u32 mask4;
	unsigned int footer;	/* for error checking */
};
typedef void (*fiq_isr_handler) (void *arg, void *regs, void *svc_sp);

/*
 * Define function prototypes.
 *  */
extern void mt_init_irq(void);
extern int mt_irq_is_active(const unsigned int irq);
extern int request_fiq(int irq, fiq_isr_handler handler, unsigned long irq_flags, void *arg);
#if defined(CONFIG_FIQ_GLUE)
extern void trigger_sw_irq(int irq);
extern int mt_enable_fiq(int irq);
extern int mt_disable_fiq(int irq);
#else
#define trigger_sw_irq(__irq) (0)
#define mt_enable_fiq(__irq) (0)
#define mt_disable_fiq(__irq) (0)
#endif
extern void mt_enable_ppi(int irq);
#endif				/* !__ASSEMBLY__ */

#endif				/* !_IRQS_H__ */
