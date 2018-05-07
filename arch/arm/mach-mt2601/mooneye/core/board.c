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

/* system header files */
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/mutex.h>
#include <linux/uaccess.h>
#include <linux/syscalls.h>
#include <linux/mtd/nand.h>

#include <asm/irq.h>
#include <asm/io.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/irq.h>
#include <asm/mach/map.h>
#include <asm/mach/time.h>
#include <asm/setup.h>
#include <asm/atomic.h>

#include <mach/system.h>
#include <mach/board.h>
#include <mach/mt_gpio.h>
#include <mach/mt_bt.h>
#include <mach/eint.h>
#include <mach/mtk_rtc.h>
#include <mach/mt_typedefs.h>
#include <cust_gpio_usage.h>
#include <cust_eint.h>
#include "board-custom.h"
#if defined(CONFIG_MTK_COMBO) || defined(CONFIG_MTK_COMBO_MODULE)
#include <mt-plat/mtk_wcn_cmb_stub.h>
#endif

#if defined(CONFIG_MTK_WCN_CMB_SDIO_SLOT)
static sdio_irq_handler_t mtk_wcn_cmb_sdio_eirq_handler;
int mtk_wcn_sdio_irq_flag_set(int falg);
static atomic_t sdio_irq_enable_flag;
static pm_callback_t mtk_wcn_cmb_sdio_pm_cb;
static void *mtk_wcn_cmb_sdio_pm_data;
static void *mtk_wcn_cmb_sdio_eirq_data;

const static u32 mtk_wcn_cmb_sdio_eint_pin = GPIO_WIFI_EINT_PIN;
const static u32 mtk_wcn_cmb_sdio_eint_num = CUST_EINT_WIFI_NUM;
const static u32 mtk_wcn_cmb_sdio_eint_m_eint = GPIO_WIFI_EINT_PIN_M_EINT;
const static u32 mtk_wcn_cmb_sdio_eint_m_gpio = GPIO_WIFI_EINT_PIN_M_GPIO;
/*
index: port number of combo chip (1:SDIO1, 2:SDIO2, no SDIO0)
value: slot power status of  (0:off, 1:on, 0xFF:invalid)
*/
#if (CONFIG_MTK_WCN_CMB_SDIO_SLOT == 0)
static unsigned char combo_port_pwr_map[4] = { 0x0, 0xFF, 0xFF, 0xFF };
#elif (CONFIG_MTK_WCN_CMB_SDIO_SLOT == 1)
static unsigned char combo_port_pwr_map[4] = { 0xFF, 0x0, 0xFF, 0xFF };
#elif (CONFIG_MTK_WCN_CMB_SDIO_SLOT == 2)
static unsigned char combo_port_pwr_map[4] = { 0xFF, 0xFF, 0x0, 0xFF };
#elif (CONFIG_MTK_WCN_CMB_SDIO_SLOT == 3)
static unsigned char combo_port_pwr_map[4] = { 0xFF, 0xFF, 0xFF, 0x0 };
#else
#error "unsupported CONFIG_MTK_WCN_CMB_SDIO_SLOT" CONFIG_MTK_WCN_CMB_SDIO_SLOT
#endif
#else
static sdio_irq_handler_t mt_wifi_irq_handler;
static pm_message_t mt_wifi_pm_state = {.event = PM_EVENT_HIBERNATE };

static pm_callback_t mt_wifi_pm_cb;
static void *mt_wifi_pm_data;
static void *mt_wifi_irq_data;
static int mt_wifi_pm_late_cb;
#endif
/*=======================================================================*/
/* Board Specific Devices Power Management                               */
/*=======================================================================*/

/*=======================================================================*/
/* Board Specific Devices                                                */
/*=======================================================================*/

/*=======================================================================*/
/* Board Specific Devices Init                                           */
/*=======================================================================*/

#ifdef CONFIG_MTK_BT_SUPPORT
void mt_bt_power_on(void)
{
	printk(KERN_INFO "+mt_bt_power_on\n");

#if defined(CONFIG_MTK_COMBO) || defined(CONFIG_MTK_COMBO_MODULE)
	/* combo chip product */
	/*
	 * Ignore rfkill0/state call. Controll BT power on/off through device /dev/stpbt.
	 */
#else
	/* standalone product */
#endif

	printk(KERN_INFO "-mt_bt_power_on\n");
}
EXPORT_SYMBOL(mt_bt_power_on);

void mt_bt_power_off(void)
{
	printk(KERN_INFO "+mt_bt_power_off\n");

#if defined(CONFIG_MTK_COMBO) || defined(CONFIG_MTK_COMBO_MODULE)
	/* combo chip product */
	/*
	 * Ignore rfkill0/state call. Controll BT power on/off through device /dev/stpbt.
	 */
#else
	/* standalone product */
#endif

	printk(KERN_INFO "-mt_bt_power_off\n");
}
EXPORT_SYMBOL(mt_bt_power_off);

int mt_bt_suspend(pm_message_t state)
{
	printk(KERN_INFO "+mt_bt_suspend\n");
	printk(KERN_INFO "-mt_bt_suspend\n");
	return MT_BT_OK;
}

int mt_bt_resume(pm_message_t state)
{
	printk(KERN_INFO "+mt_bt_resume\n");
	printk(KERN_INFO "-mt_bt_resume\n");
	return MT_BT_OK;
}
#endif


#if defined(CONFIG_MTK_WCN_CMB_SDIO_SLOT)
void mtk_wcn_cmb_sdio_enable_eirq(void)
{
	mt65xx_eint_unmask(mtk_wcn_cmb_sdio_eint_num);	/* CUST_EINT_WIFI_NUM */
}

void mtk_wcn_cmb_sdio_disable_eirq(void)
{
	mt65xx_eint_mask(mtk_wcn_cmb_sdio_eint_num);	/* CUST_EINT_WIFI_NUM */
}

void mtk_wcn_cmb_sdio_eirq_handler_stub(void)
{
	if ((NULL != mtk_wcn_cmb_sdio_eirq_handler) &&(0 != atomic_read(&sdio_irq_enable_flag))) {
		mtk_wcn_cmb_sdio_eirq_handler(mtk_wcn_cmb_sdio_eirq_data);
	}
}

void mtk_wcn_cmb_sdio_request_eirq(sdio_irq_handler_t irq_handler, void *data)
{
	mtk_wcn_sdio_irq_flag_set(0);
	mtk_wcn_cmb_sdio_eirq_data = data;
	mtk_wcn_cmb_sdio_eirq_handler = irq_handler;
	mt65xx_eint_set_sens(mtk_wcn_cmb_sdio_eint_num, CUST_EINT_WIFI_SENSITIVE);	/*CUST_EINT_WIFI_NUM */
	mt65xx_eint_set_hw_debounce(mtk_wcn_cmb_sdio_eint_num, CUST_EINT_WIFI_DEBOUNCE_CN);	/*CUST_EINT_WIFI_NUM */
	mt_set_gpio_pull_enable(mtk_wcn_cmb_sdio_eint_pin, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(mtk_wcn_cmb_sdio_eint_pin, GPIO_PULL_UP);
	mt65xx_eint_registration(mtk_wcn_cmb_sdio_eint_num /*CUST_EINT_WIFI_NUM */ ,
				 CUST_EINT_WIFI_DEBOUNCE_EN,
				 CUST_EINT_WIFI_POLARITY, mtk_wcn_cmb_sdio_eirq_handler_stub, 0);
	mt65xx_eint_mask(mtk_wcn_cmb_sdio_eint_num);	/*CUST_EINT_WIFI_NUM */

}

void mtk_wcn_cmb_sdio_register_pm(pm_callback_t pm_cb, void *data)
{
	printk(KERN_INFO "mtk_wcn_cmb_sdio_register_pm (0x%p, 0x%p)\n", pm_cb, data);
	/* register pm change callback */
	mtk_wcn_cmb_sdio_pm_cb = pm_cb;
	mtk_wcn_cmb_sdio_pm_data = data;
}

void mtk_wcn_cmb_sdio_on(int sdio_port_num)
{
	pm_message_t state = {.event = PM_EVENT_USER_RESUME };

	printk(KERN_INFO "mtk_wcn_cmb_sdio_on (%d)\n", sdio_port_num);

	/* 1. disable sdio eirq */
	mtk_wcn_cmb_sdio_disable_eirq();
	mt_set_gpio_mode(mtk_wcn_cmb_sdio_eint_pin, mtk_wcn_cmb_sdio_eint_m_eint);	/* EINT mode */

	/* 2. call sd callback */
	if (mtk_wcn_cmb_sdio_pm_cb) {
		/* printk(KERN_INFO "mtk_wcn_cmb_sdio_pm_cb(PM_EVENT_USER_RESUME, 0x%p, 0x%p)\n", mtk_wcn_cmb_sdio_pm_cb, mtk_wcn_cmb_sdio_pm_data); */
		mtk_wcn_cmb_sdio_pm_cb(state, mtk_wcn_cmb_sdio_pm_data);
	} else {
		printk(KERN_WARNING "mtk_wcn_cmb_sdio_on no sd callback!!\n");
	}
}

void mtk_wcn_cmb_sdio_off(int sdio_port_num)
{
	pm_message_t state = {.event = PM_EVENT_USER_SUSPEND };

	printk(KERN_INFO "mtk_wcn_cmb_sdio_off (%d)\n", sdio_port_num);

	/* 1. call sd callback */
	if (mtk_wcn_cmb_sdio_pm_cb) {
		/* printk(KERN_INFO "mtk_wcn_cmb_sdio_off(PM_EVENT_USER_SUSPEND, 0x%p, 0x%p)\n", mtk_wcn_cmb_sdio_pm_cb, mtk_wcn_cmb_sdio_pm_data); */
		mtk_wcn_cmb_sdio_pm_cb(state, mtk_wcn_cmb_sdio_pm_data);
	} else {
		printk(KERN_WARNING "mtk_wcn_cmb_sdio_off no sd callback!!\n");
	}

	/* 2. disable sdio eirq */
	mtk_wcn_cmb_sdio_disable_eirq();
	/*printk(KERN_INFO "[mt6620] set WIFI_EINT input pull down\n"); */
	mt_set_gpio_mode(mtk_wcn_cmb_sdio_eint_pin, mtk_wcn_cmb_sdio_eint_m_gpio);	/* GPIO mode */
	mt_set_gpio_dir(mtk_wcn_cmb_sdio_eint_pin, GPIO_DIR_IN);
	mt_set_gpio_pull_select(mtk_wcn_cmb_sdio_eint_pin, GPIO_PULL_UP);
	mt_set_gpio_pull_enable(mtk_wcn_cmb_sdio_eint_pin, GPIO_PULL_ENABLE);
}

int board_sdio_ctrl(unsigned int sdio_port_num, unsigned int on)
{
#if defined(CONFIG_MTK_WCN_CMB_SDIO_SLOT)
	sdio_port_num = CONFIG_MTK_WCN_CMB_SDIO_SLOT;
	printk(KERN_WARNING "mt_combo_sdio_ctrl: force set sdio port to (%d)\n", sdio_port_num);
#endif
	if ((sdio_port_num >= 4) || (combo_port_pwr_map[sdio_port_num] == 0xFF)) {
		/* invalid sdio port number or slot mapping */
		printk(KERN_WARNING "mt_mtk_wcn_cmb_sdio_ctrl invalid port(%d, %d)\n",
		       sdio_port_num, combo_port_pwr_map[sdio_port_num]);
		return -1;
	}
	/*printk(KERN_INFO "mt_mtk_wcn_cmb_sdio_ctrl (%d, %d)\n", sdio_port_num, on); */

	if (!combo_port_pwr_map[sdio_port_num] && on) {
		printk(KERN_INFO "board_sdio_ctrl force off before on\n");
		mtk_wcn_cmb_sdio_off(sdio_port_num);
		combo_port_pwr_map[sdio_port_num] = 0;
		/* off -> on */
		mtk_wcn_cmb_sdio_on(sdio_port_num);
		combo_port_pwr_map[sdio_port_num] = 1;
	} else if (combo_port_pwr_map[sdio_port_num] && !on) {
		/* on -> off */
		mtk_wcn_cmb_sdio_off(sdio_port_num);
		combo_port_pwr_map[sdio_port_num] = 0;
	} else {
		return -2;
	}
	return 0;
}
EXPORT_SYMBOL(board_sdio_ctrl);

int mtk_wcn_sdio_irq_flag_set(int flag)
{

	if (0 != flag) {
		atomic_set(&sdio_irq_enable_flag, 1);
	} else {
		atomic_set(&sdio_irq_enable_flag, 0);
	}
	printk(KERN_INFO "sdio_irq_enable_flag:%d\n", atomic_read(&sdio_irq_enable_flag));

	return atomic_read(&sdio_irq_enable_flag);
}
EXPORT_SYMBOL(mtk_wcn_sdio_irq_flag_set);
#endif				/* end of defined(CONFIG_MTK_WCN_CMB_SDIO_SLOT) */


/* Board Specific Devices                                                */
/*=======================================================================*/

/*=======================================================================*/
/* Board Specific Devices Init                                           */
/*=======================================================================*/

/*=======================================================================*/
/* Board Devices Capability                                              */
/*=======================================================================*/
/* #define MSDC_SDCARD_FLAG  (MSDC_SYS_SUSPEND | MSDC_CD_PIN_EN | MSDC_REMOVABLE | MSDC_HIGHSPEED| MSDC_UHS1 |MSDC_DDR) */
/* #define MSDC_SDCARD_FLAG  (MSDC_SYS_SUSPEND | MSDC_REMOVABLE | MSDC_HIGHSPEED| MSDC_UHS1 |MSDC_DDR) */

/* Please enable/disable SD card MSDC_CD_PIN_EN for customer request */
#define MSDC_SDIO_FLAG    (MSDC_EXT_SDIO_IRQ | MSDC_HIGHSPEED | MSDC_PM_KEEP_POWER)

#ifdef CONFIG_EARLY_LINUX_PORTING
#define MSDC_SDCARD_FLAG  (MSDC_SYS_SUSPEND | MSDC_REMOVABLE | MSDC_HIGHSPEED)
#define MSDC_EMMC_FLAG	  (MSDC_SYS_SUSPEND | MSDC_HIGHSPEED)
#else
#define MSDC_SDCARD_FLAG  (MSDC_SYS_SUSPEND | MSDC_CD_PIN_EN | MSDC_REMOVABLE | MSDC_HIGHSPEED | MSDC_UHS1 | MSDC_DDR)
#define MSDC_EMMC_FLAG    (MSDC_SYS_SUSPEND | MSDC_HIGHSPEED | MSDC_UHS1 | MSDC_DDR)
#endif

#if defined(CFG_DEV_MSDC0)
#if defined(CONFIG_MTK_EMMC_SUPPORT)
struct msdc_hw msdc0_hw = {
#ifdef CONFIG_EARLY_LINUX_PORTING
	.clk_src = 0,
#else
	.clk_src = MSDC_CLKSRC_200MHZ,
#endif
	.cmd_edge = MSDC_SMPL_FALLING,
	.rdata_edge = MSDC_SMPL_FALLING,
	.wdata_edge = MSDC_SMPL_FALLING,
	.clk_drv = 2,		/* 6mA */
	.cmd_drv = 2,		/* 6mA */
	.dat_drv = 2,		/* 6mA */
	.clk_drv_sd_18 = 3,	/* 8mA for HS200 */
	.cmd_drv_sd_18 = 3,	/* 8mA for HS200 */
	.dat_drv_sd_18 = 3,	/* 8mA for HS200 */
	.data_pins = 8,
	.data_offset = 0,
	.flags = MSDC_EMMC_FLAG,
	.dat0rddly = 0,
	.dat1rddly = 0,
	.dat2rddly = 0,
	.dat3rddly = 0,
	.dat4rddly = 0,
	.dat5rddly = 0,
	.dat6rddly = 0,
	.dat7rddly = 0,
	.datwrddly = 0,
	.cmdrrddly = 0,
	.cmdrddly = 0,
	.host_function = MSDC_EMMC,
	.boot = MSDC_BOOT_EN,
	.cd_level = MSDC_CD_LOW,
};
#endif
#endif

#if defined(CFG_DEV_MSDC1)
#if defined(CONFIG_MTK_COMBO_COMM_SDIO)
struct msdc_hw msdc1_hw = {
	.clk_src = MSDC_CLKSRC_200MHZ,
	.cmd_edge = MSDC_SMPL_FALLING,
	.rdata_edge = MSDC_SMPL_FALLING,
	.wdata_edge = MSDC_SMPL_FALLING,
	.clk_drv = 4,		/* 20mA for SD20, 15mA for SDR50, DDR50 */
	.cmd_drv = 4,		/* 20mA for SD20, 15mA for SDR50, DDR50 */
	.dat_drv = 4,		/* 20mA for SD20, 15mA for SDR50, DDR50 */
	.clk_drv_sd_18 = 5,	/* 18mA for SDR104 */
	.cmd_drv_sd_18 = 5,	/* 18mA for SDR104 */
	.dat_drv_sd_18 = 5,	/* 18mA for SDR104 */
	.data_pins = 4,
	.data_offset = 0,
	/* MT6620 use External IRQ, wifi uses high speed. here wifi manage his own suspend and resume, does not support hot plug */
	.flags = MSDC_SDIO_FLAG,	/* MSDC_SYS_SUSPEND | MSDC_WP_PIN_EN | MSDC_CD_PIN_EN | MSDC_REMOVABLE, */
	.dat0rddly = 0,
	.dat1rddly = 0,
	.dat2rddly = 0,
	.dat3rddly = 0,
	.dat4rddly = 0,
	.dat5rddly = 0,
	.dat6rddly = 0,
	.dat7rddly = 0,
	.datwrddly = 0,
	.cmdrrddly = 0,
	.cmdrddly = 0,
	.host_function = MSDC_SDIO,
	.boot = 0,
	.request_sdio_eirq = mtk_wcn_cmb_sdio_request_eirq,
	.enable_sdio_eirq = mtk_wcn_cmb_sdio_enable_eirq,
	.disable_sdio_eirq = mtk_wcn_cmb_sdio_disable_eirq,
	.register_pm = mtk_wcn_cmb_sdio_register_pm,
};
#else
struct msdc_hw msdc1_hw = {
	.clk_src = MSDC_CLKSRC_200MHZ,
	.cmd_edge = MSDC_SMPL_FALLING,
	.rdata_edge = MSDC_SMPL_FALLING,
	.wdata_edge = MSDC_SMPL_FALLING,
	.clk_drv = 4,		/* 20mA for SD20, 15mA for SDR50, DDR50 */
	.cmd_drv = 4,		/* 20mA for SD20, 15mA for SDR50, DDR50 */
	.dat_drv = 4,		/* 20mA for SD20, 15mA for SDR50, DDR50 */
	.clk_drv_sd_18 = 5,	/* 18mA for SDR104 */
	.cmd_drv_sd_18 = 5,	/* 18mA for SDR104 */
	.dat_drv_sd_18 = 5,	/* 18mA for SDR104 */
	.data_pins = 4,
	.data_offset = 0,
	.flags = MSDC_SDCARD_FLAG,
	.dat0rddly = 0,
	.dat1rddly = 0,
	.dat2rddly = 0,
	.dat3rddly = 0,
	.dat4rddly = 0,
	.dat5rddly = 0,
	.dat6rddly = 0,
	.dat7rddly = 0,
	.datwrddly = 0,
	.cmdrrddly = 0,
	.cmdrddly = 0,
	.host_function = MSDC_SD,
	.boot = 0,
	.cd_level = MSDC_CD_LOW,
};
#endif
#endif

/* NAND Driver */
#if defined(CONFIG_MTK_MTD_NAND)
struct mtk_nand_host_hw mtk_nand_hw = {
	.nfi_bus_width = 8,
	.nfi_access_timing = NFI_DEFAULT_ACCESS_TIMING,
	.nfi_cs_num = NFI_CS_NUM,
	.nand_sec_size = 512,
	.nand_sec_shift = 9,
	.nand_ecc_size = 2048,
	.nand_ecc_bytes = 32,
	.nand_ecc_mode = NAND_ECC_HW,
};
#endif
