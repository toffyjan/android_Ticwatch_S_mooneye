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

#include <linux/string.h>
#include <linux/uaccess.h>

#include <linux/debugfs.h>

#include <mt-plat/aee.h>

#include <linux/dma-mapping.h>
#include "ddp_debug.h"
#include "ddp_reg.h"
#include "ddp_bls.h"
#include "ddp_color.h"
#include "ddp_drv.h"
#include "ddp_wdma.h"
#include "ddp_hal.h"
#include "ddp_path.h"

#include "disp_drv_ddp.h"
#include <mach/m4u.h>

/* --------------------------------------------------------------------------- */
/* Debug Options */
/* --------------------------------------------------------------------------- */

struct DDP_MMP_Events_t DDP_MMP_Events;

struct dentry *debugfs = NULL;
unsigned int gUltraLevel = 4;	/* RDMA ultra aggressive level */
unsigned int gEnableUltra = 0;

static const long int DEFAULT_LOG_FPS_WND_SIZE = 30;

unsigned char pq_debug_flag = 0;
unsigned char aal_debug_flag = 0;

#if defined(DDP_DRV_DBG_ON)
unsigned int ddp_drv_dbg_log = 0;	/* default off, use "adb shell "echo ddp_drv_dbg_log:1 > sys/kernel/debug/dispsys" to enable */
unsigned int ddp_drv_irq_log = 0;	/* default off */
unsigned int ddp_drv_info_log = 1;	/* default on, important msg, not err */
unsigned int ddp_drv_err_log = 1;	/* default on, err msg */
#else
unsigned int ddp_drv_dbg_log = 0;
unsigned int ddp_drv_irq_log = 0;
unsigned int ddp_drv_info_log = 1;
unsigned int ddp_drv_err_log = 1;
#endif

#if defined(LCD_DRV_DBG_ON)
size_t dbi_drv_dbg_log = true;
size_t dbi_drv_dbg_func_log = false;	/* function entry log, default off */
size_t dsi_drv_dbg_log = true;
size_t dsi_drv_dbg_func_log = false;	/* function entry log, default off */
#else
size_t dbi_drv_dbg_log = false;
size_t dbi_drv_dbg_func_log = false;	/* function entry log, default off */
size_t dsi_drv_dbg_log = false;
size_t dsi_drv_dbg_func_log = false;	/* function entry log, default off */
#endif

static char STR_HELP[] =
"\n"
"USAGE\n"
"        echo [ACTION]... > dispsys\n"
"\n"
"ACTION\n"
"       ddp_drv_dbg_log:0|1\n"
"\n"
"       ddp_drv_irq_log:0|1\n"
"\n"
"       backlight:level\n"
"\n"
"       dump_aal:arg\n"
"\n"
"       mmp\n"
"\n"
"       dump_reg:moduleID\n"
"\n"
"       dpfd_ut1:channel\n"
;

void init_ddp_mmp_events(void)
{
	if (DDP_MMP_Events.DDP == 0) {
		DDP_MMP_Events.DDP = MMProfileRegisterEvent(MMP_RootEvent, "DDP");
		DDP_MMP_Events.MutexParent = MMProfileRegisterEvent(DDP_MMP_Events.DDP, "Mutex");
		DDP_MMP_Events.Mutex[0] = MMProfileRegisterEvent(DDP_MMP_Events.MutexParent, "Mutex0");
		DDP_MMP_Events.Mutex[1] = MMProfileRegisterEvent(DDP_MMP_Events.MutexParent, "Mutex1");
		DDP_MMP_Events.Mutex[2] = MMProfileRegisterEvent(DDP_MMP_Events.MutexParent, "Mutex2");
		DDP_MMP_Events.Mutex[3] = MMProfileRegisterEvent(DDP_MMP_Events.MutexParent, "Mutex3");
		DDP_MMP_Events.Mutex[4] = MMProfileRegisterEvent(DDP_MMP_Events.MutexParent, "Mutex4");
		DDP_MMP_Events.Mutex[5] = MMProfileRegisterEvent(DDP_MMP_Events.MutexParent, "Mutex5");
		DDP_MMP_Events.BackupReg = MMProfileRegisterEvent(DDP_MMP_Events.DDP, "BackupReg");
		DDP_MMP_Events.DDP_IRQ = MMProfileRegisterEvent(DDP_MMP_Events.DDP, "DDP_IRQ");
		DDP_MMP_Events.SCL_IRQ = MMProfileRegisterEvent(DDP_MMP_Events.DDP_IRQ, "SCL_IRQ");
		DDP_MMP_Events.ROT_IRQ = MMProfileRegisterEvent(DDP_MMP_Events.DDP_IRQ, "ROT_IRQ");
		DDP_MMP_Events.OVL_IRQ = MMProfileRegisterEvent(DDP_MMP_Events.DDP_IRQ, "OVL_IRQ");
		DDP_MMP_Events.WDMA0_IRQ = MMProfileRegisterEvent(DDP_MMP_Events.DDP_IRQ, "WDMA0_IRQ");
		DDP_MMP_Events.WDMA1_IRQ = MMProfileRegisterEvent(DDP_MMP_Events.DDP_IRQ, "WDMA1_IRQ");
		DDP_MMP_Events.RDMA0_IRQ = MMProfileRegisterEvent(DDP_MMP_Events.DDP_IRQ, "RDMA0_IRQ");
		DDP_MMP_Events.RDMA1_IRQ = MMProfileRegisterEvent(DDP_MMP_Events.DDP_IRQ, "RDMA1_IRQ");
		DDP_MMP_Events.COLOR_IRQ = MMProfileRegisterEvent(DDP_MMP_Events.DDP_IRQ, "COLOR_IRQ");
		DDP_MMP_Events.BLS_IRQ = MMProfileRegisterEvent(DDP_MMP_Events.DDP_IRQ, "BLS_IRQ");
		DDP_MMP_Events.TDSHP_IRQ = MMProfileRegisterEvent(DDP_MMP_Events.DDP_IRQ, "TDSHP_IRQ");
		DDP_MMP_Events.CMDQ_IRQ = MMProfileRegisterEvent(DDP_MMP_Events.DDP_IRQ, "CMDQ_IRQ");
		DDP_MMP_Events.Mutex_IRQ = MMProfileRegisterEvent(DDP_MMP_Events.DDP_IRQ, "Mutex_IRQ");
		DDP_MMP_Events.WAIT_INTR = MMProfileRegisterEvent(DDP_MMP_Events.DDP, "WAIT_IRQ");
		DDP_MMP_Events.Debug = MMProfileRegisterEvent(DDP_MMP_Events.DDP, "Debug");

		MMProfileEnableEventRecursive(DDP_MMP_Events.MutexParent, 1);
		MMProfileEnableEventRecursive(DDP_MMP_Events.BackupReg, 1);
		/* MMProfileEnableEventRecursive(DDP_MMP_Events.DDP_IRQ, 1); */
		MMProfileEnableEventRecursive(DDP_MMP_Events.WAIT_INTR, 1);
	}
}

/* --------------------------------------------------------------------------- */
/* Command Processor */
/* --------------------------------------------------------------------------- */
static char dbg_buf[2048];
extern void mtkfb_dump_layer_info(void);
extern unsigned int gNeedToRecover;

static void process_dbg_opt(const char *opt)
{
	char *buf = dbg_buf + strlen(dbg_buf);
	if (0 == strncmp(opt, "ddp_drv_dbg_log:", 16)) {
		char *p = (char *)opt + 16;
		unsigned int enable = (unsigned int)simple_strtoul(p, &p, 10);
		if (enable)
			ddp_drv_dbg_log = 1;
		else
			ddp_drv_dbg_log = 0;

		sprintf(buf, "ddp_drv_dbg_log: %d\n", ddp_drv_dbg_log);
	} else if (0 == strncmp(opt, "ddp_drv_irq_log:", 16)) {
		char *p = (char *)opt + 16;
		unsigned int enable = (unsigned int)simple_strtoul(p, &p, 10);
		if (enable)
			ddp_drv_irq_log = 1;
		else
			ddp_drv_irq_log = 0;

		sprintf(buf, "ddp_drv_irq_log: %d\n", ddp_drv_irq_log);
	} else if (0 == strncmp(opt, "backlight:", 10)) {
		char *p = (char *)opt + 10;
		unsigned int level = (unsigned int)simple_strtoul(p, &p, 10);

		if (level) {
			disp_bls_set_backlight(level);
			sprintf(buf, "backlight: %d\n", level);
		} else {
			goto Error;
		}
	} else if (0 == strncmp(opt, "dump_reg:", 9)) {
		char *p = (char *)opt + 9;
		unsigned int module = (unsigned int)simple_strtoul(p, &p, 10);
		DDP_DRV_INFO("process_dbg_opt, module=%d\n", module);
		if (module < DISP_MODULE_MAX) {
			disp_dump_reg(module);
			sprintf(buf, "dump_reg: %d\n", module);
		} else {
			DDP_DRV_INFO("process_dbg_opt2, module=%d\n", module);
			goto Error;
		}
	} else if (0 == strncmp(opt, "dump_aal:", 9)) {
		char *p = (char *)opt + 9;
		unsigned int arg = (unsigned int)simple_strtoul(p, &p, 10);
		if (arg == 0) {
			int i;
			unsigned int hist[LUMA_HIST_BIN];
			disp_get_hist(hist);
			for (i = 0; i < LUMA_HIST_BIN; i++) {
				DDP_DRV_DBG("LUMA_HIST_%02d: %d\n", i, hist[i]);
				sprintf(dbg_buf + strlen(dbg_buf), "LUMA_HIST_%2d: %d\n", i, hist[i]);
			}
		} else if (arg == 1) {
			int i;
			DISP_AAL_PARAM param;

			GetUpdateMutex();
			memcpy(&param, get_aal_config(), sizeof(DISP_AAL_PARAM));
			ReleaseUpdateMutex();

			DDP_DRV_DBG("pwmDuty: %lu\n", param.pwmDuty);
			sprintf(dbg_buf + strlen(dbg_buf), "pwmDuty: %lu\n", param.pwmDuty);
			for (i = 0; i < LUMA_CURVE_POINT; i++) {
				DDP_DRV_DBG("lumaCurve[%02d]: %lu\n", i, param.lumaCurve[i]);
				sprintf(dbg_buf + strlen(dbg_buf), "lumaCurve[%02d]: %lu\n", i, param.lumaCurve[i]);
			}
		}
	} else if (0 == strncmp(opt, "debug:", 6)) {
		char *p = (char *)opt + 6;
		unsigned int enable = (unsigned int)simple_strtoul(p, &p, 10);
		if (enable == 1) {
			DISP_MSG("[DDP] debug=1, trigger AEE\n");
			aee_kernel_exception("DDP-TEST-ASSERT", "[DDP] DDP-TEST-ASSERT");
		} else if (enable == 2) {
			ddp_mem_test();
		} else if (enable == 3) {
			ddp_mem_test2();
		} else if (enable == 5) {
			DISP_MSG("SMI_LARB_MON_REQ0=0x%x, SMI_LARB_MON_REQ1=0x%x, SMI_0=0x%x, SMI_600=0x%x, SMI_604=0x%x, SMI_610=0x%x, SMI_614=0x%x, \
				color_h_cnt=%d, color_line_cnt=%d, ovl_add_con=0x%x, ovl_ctrl_flow=0x%x\n",
				DISP_REG_GET(0xF4010450),
				DISP_REG_GET(0xF4010454),
				DISP_REG_GET(0xF4010000),
				DISP_REG_GET(0xF4010600),
				DISP_REG_GET(0xF4010604),
				DISP_REG_GET(0xF4010610),
				DISP_REG_GET(0xF4010614),
				DISP_REG_GET(0xF400B404),
				DISP_REG_GET(0xF400B408),
				DISP_REG_GET(DISP_REG_OVL_ADDCON_DBG),
				DISP_REG_GET(DISP_REG_OVL_FLOW_CTRL_DBG));
			sprintf(dbg_buf + strlen(dbg_buf), "SMI_LARB_MON_REQ0=0x%x, SMI_LARB_MON_REQ1=0x%x, SMI_0=0x%x, SMI_600=0x%x, SMI_604=0x%x, SMI_610=0x%x, SMI_614=0x%x, \
				color_h_cnt=%d, color_line_cnt=%d, ovl_add_con=0x%x, ovl_ctrl_flow=0x%x\n",
				DISP_REG_GET(0xF4010450),
				DISP_REG_GET(0xF4010454),
				DISP_REG_GET(0xF4010000),
				DISP_REG_GET(0xF4010600),
				DISP_REG_GET(0xF4010604),
				DISP_REG_GET(0xF4010610),
				DISP_REG_GET(0xF4010614),
				DISP_REG_GET(0xF400B404),
				DISP_REG_GET(0xF400B408),
				DISP_REG_GET(DISP_REG_OVL_ADDCON_DBG),
				DISP_REG_GET(DISP_REG_OVL_FLOW_CTRL_DBG));
		} else if (enable == 6) {
			mtkfb_dump_layer_info();
		} else if (enable == 7) {
			gNeedToRecover = 1;
		}
		else if ((enable >= 11) && (enable <= 15)) {
			gEnableUltra = 1;
			gUltraLevel = enable - 11;
			sprintf(buf, "gUltraLevel: %d, DISP_REG_RDMA_MEM_GMC_SETTING_0=0x%x, DISP_REG_RDMA_FIFO_CON=0x%x\n",
				gUltraLevel,
				DISP_REG_GET(DISP_REG_RDMA_MEM_GMC_SETTING_0),
				DISP_REG_GET(DISP_REG_RDMA_FIFO_CON));
			DISP_MSG("ddp debug set gUltraLevel = %d, DISP_REG_RDMA_MEM_GMC_SETTING_0=0x%x, DISP_REG_RDMA_FIFO_CON=0x%x\n",
				gUltraLevel,
				DISP_REG_GET(DISP_REG_RDMA_MEM_GMC_SETTING_0),
				DISP_REG_GET(DISP_REG_RDMA_FIFO_CON));
		} else if (enable == 21) {
			sprintf(buf, "base:\n\
				config f4+0 \n\
				ovl 7\n\
				rdma 8\n\
				rdma1 12\n\
				wdma 9\n\
				bls a\n\
				color b\n\
				dsi c\n\
				dpi d\n\
				mm_mutex e\n\
				mm_cmdq f\n\
				smi_larb0 10\n\
				smi_common 11\n");
		}
	} else if (0 == strncmp(opt, "mmp", 3)) {
		init_ddp_mmp_events();
	} else if (0 == strncmp(opt, "pqon", 4)) {
		pq_debug_flag = 0;
		sprintf(buf, "Turn on PQ %d\n", pq_debug_flag);
	} else if (0 == strncmp(opt, "pqoff", 5)) {
		pq_debug_flag = 1;
		sprintf(buf, "Turn off PQ %d\n", pq_debug_flag);
	} else if (0 == strncmp(opt, "pqdemo", 6)) {
		pq_debug_flag = 2;
		sprintf(buf, "Turn on PQ (demo) %d\n", pq_debug_flag);
	} else if (0 == strncmp(opt, "pqstop", 6)) {
		pq_debug_flag = 3;
		sprintf(buf, "Stop mutex update %d\n", pq_debug_flag);
	} else if (0 == strncmp(opt, "aalon", 5)) {
		aal_debug_flag = 0;
		sprintf(buf, "resume aal update %d\n", aal_debug_flag);
	} else if (0 == strncmp(opt, "aaloff", 6)) {
		aal_debug_flag = 1;
		sprintf(buf, "suspend aal update %d\n", aal_debug_flag);
	} else if (0 == strncmp(opt, "color_win:", 10)) {
		char *p = (char *)opt + 10;
		unsigned int sat_upper, sat_lower, hue_upper, hue_lower;
		sat_upper = (unsigned int)simple_strtoul(p, &p, 10);
		p++;
		sat_lower = (unsigned int)simple_strtoul(p, &p, 10);
		p++;
		hue_upper = (unsigned int)simple_strtoul(p, &p, 10);
		p++;
		hue_lower = (unsigned int)simple_strtoul(p, &p, 10);
		DDP_DRV_INFO("Set color_win: %u, %u, %u, %u\n", sat_upper, sat_lower, hue_upper, hue_lower);
		disp_color_set_window(sat_upper, sat_lower, hue_upper, hue_lower);
	} else if (0 == strncmp(opt, "dither:", 7)) {
		if (0 == strncmp(opt + 7, "on", 2))
			DISP_REG_SET(DISP_REG_BLS_DITHER(0), 0x00000001);
		else if (0 == strncmp(opt + 7, "off", 3))
			DISP_REG_SET(DISP_REG_BLS_DITHER(0), 0x00000000);
		else
			goto Error;
	} else {
		goto Error;
	}

	return;

Error:
	DDP_DRV_ERR("parse command error!\n%s\n\n%s", opt, STR_HELP);
}

static void process_dbg_cmd(char *cmd)
{
	char *tok;

	DDP_DRV_DBG("cmd: %s\n", cmd);
	memset(dbg_buf, 0, sizeof(dbg_buf));
	while ((tok = strsep(&cmd, " ")) != NULL)
		process_dbg_opt(tok);
}

/* --------------------------------------------------------------------------- */
/* Debug FileSystem Routines */
/* --------------------------------------------------------------------------- */

static ssize_t debug_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static char cmd_buf[512];

static ssize_t debug_read(struct file *file,
			  char __user *ubuf, size_t count, loff_t *ppos)
{
	if (strlen(dbg_buf))
		return simple_read_from_buffer(ubuf, count, ppos, dbg_buf, strlen(dbg_buf));
	else
		return simple_read_from_buffer(ubuf, count, ppos, STR_HELP, strlen(STR_HELP));
}

static ssize_t debug_write(struct file *file,
			   const char __user *ubuf, size_t count, loff_t *ppos)
{
	const int debug_bufmax = sizeof(cmd_buf) - 1;
	size_t ret;

	ret = count;

	if (count > debug_bufmax)
		count = debug_bufmax;

	if (copy_from_user(&cmd_buf, ubuf, count))
		return -EFAULT;

	cmd_buf[count] = 0;

	process_dbg_cmd(cmd_buf);

	return ret;
}

static const struct file_operations debug_fops = {
	.read = debug_read,
	.write = debug_write,
	.open = debug_open,
};

void ddp_debug_init(void)
{
	debugfs = debugfs_create_file("dispsys",
				      S_IFREG | S_IRUGO, NULL, (void *)0, &debug_fops);
}

void ddp_debug_exit(void)
{
	debugfs_remove(debugfs);
}

#include <linux/vmalloc.h>
#define DDP_TEST_WIDTH 64
#define DDP_TEST_HEIGHT 64
#define DDP_TEST_BPP 3
#define DDP_MUTEX_FOR_ROT_SCL_WDMA 1
extern unsigned char data_rgb888_64x64[12288];
extern unsigned char data_rgb888_64x64_golden[12288];
int ddp_mem_test2(void)
{
	int result = 0;
	unsigned int *pSrc;
	unsigned int *pDst;

	pSrc = vmalloc(DDP_TEST_WIDTH * DDP_TEST_HEIGHT * DDP_TEST_BPP);
	if (pSrc == 0) {
		DISP_MSG("[DDP] error: dma_alloc_coherent error!  dma memory not available.\n");
		return 0;
	} else {
		DISP_MSG("[ddp] pSrc=0x%x\n", (unsigned int)pSrc);
	}
	memcpy((void *)pSrc, data_rgb888_64x64, DDP_TEST_WIDTH * DDP_TEST_HEIGHT * DDP_TEST_BPP);

	pDst = vmalloc(DDP_TEST_WIDTH * DDP_TEST_HEIGHT * DDP_TEST_BPP);
	if (pDst == 0) {
		DISP_MSG("[DDP] error: dma_alloc_coherent error!  dma memory not available.\n");
		return 0;
	} else {
		DISP_MSG("[ddp] pDst=0x%x\n", (unsigned int)pDst);
	}
	memset((void *)pDst, 0, DDP_TEST_WIDTH * DDP_TEST_HEIGHT * DDP_TEST_BPP);

	/* config port to virtual */
	{
		M4U_PORT_STRUCT sPort;

		sPort.ePortID = M4U_PORT_LCD_W;
		sPort.Virtuality = 1;
		sPort.Security = 0;
		sPort.Distance = 1;
		sPort.Direction = 0;
		m4u_config_port(&sPort);
	}

	/* result verify */
	{
		unsigned int diff_cnt = 0;
		unsigned int t = 0;
		unsigned int size = DDP_TEST_WIDTH * DDP_TEST_HEIGHT * DDP_TEST_BPP;
		for (t = 0; t < size; t++) {
			if (*((unsigned char *)pSrc + t) != *((unsigned char *)data_rgb888_64x64 + t)) {
				diff_cnt++;
				DISP_MSG("t=%d, diff_cnt=%d, dst=0x%x, gold=0x%x\n",
				       t,
				       diff_cnt,
				       *((unsigned char *)pSrc + t),
				       *((unsigned char *)data_rgb888_64x64 + t));
			}

		}
		if (diff_cnt == 0)
			DISP_MSG("ddp_mem_test src compare result: success\n");
		else {
			DISP_MSG("[DDP] error: ddp_mem_test src compare result: fail\n");
			DISP_MSG("detail, %d, %d, %%%d\n", diff_cnt, size, diff_cnt * 100 / size);
			result = -1;
		}
	}

	{
		unsigned int diff_cnt = 0;
		unsigned int t = 0;
		unsigned int size = DDP_TEST_WIDTH * DDP_TEST_HEIGHT * DDP_TEST_BPP;
		for (t = 0; t < size; t++) {
			if (*((unsigned char *)pDst + t) != *((unsigned char *)data_rgb888_64x64_golden + t)) {
				diff_cnt++;
				DISP_MSG("t=%d, diff_cnt=%d, dst=0x%x, gold=0x%x\n",
				       t,
				       diff_cnt,
				       *((unsigned char *)pDst + t),
				       *((unsigned char *)data_rgb888_64x64_golden + t));
			}

		}
		if (diff_cnt == 0)
			DISP_MSG("ddp_mem_test result: success\n");
		else {
			DISP_MSG("[DDP] error: ddp_mem_test result: fail\n");
			DISP_MSG("detail, %d, %d, %%%d\n", diff_cnt, size, diff_cnt * 100 / size);
			result = -1;
		}
	}

	/* dealloc memory */
	vfree(pSrc);
	vfree(pDst);

	return result;
}

int ddp_mem_test(void)
{
	int result = 0;
	struct disp_path_config_struct config;
	unsigned int *pSrc;
	unsigned char *pSrcPa;
	unsigned int *pDst;
	unsigned char *pDstPa;

	pSrc = dma_alloc_coherent(NULL, DDP_TEST_WIDTH * DDP_TEST_HEIGHT * DDP_TEST_BPP, (dma_addr_t *) &pSrcPa, GFP_KERNEL);
	if (pSrc == 0 || pSrcPa == 0) {
		DISP_MSG("dma_alloc_coherent error!  dma memory not available.\n");
		return 0;
	} else {
		DISP_MSG("[ddp] pSrc=0x%x, pSrcPa=0x%x\n", (unsigned int)pSrc, (unsigned int)pSrcPa);
	}
	memcpy((void *)pSrc, data_rgb888_64x64, DDP_TEST_WIDTH * DDP_TEST_HEIGHT * DDP_TEST_BPP);

	pDst = dma_alloc_coherent(NULL, DDP_TEST_WIDTH * DDP_TEST_HEIGHT * DDP_TEST_BPP, (dma_addr_t *) &pDstPa, GFP_KERNEL);
	if (pDst == 0 || pDstPa == 0) {
		DISP_MSG("dma_alloc_coherent error!  dma memory not available.\n");
		return 0;
	} else {
		DISP_MSG("[ddp] pDst=0x%x, pDstPa=0x%x\n", (unsigned int)pDst, (unsigned int)pDstPa);
	}
	memset((void *)pDst, 0, DDP_TEST_WIDTH * DDP_TEST_HEIGHT * DDP_TEST_BPP);

	/* config port to physical */
	{
		M4U_PORT_STRUCT sPort;

		sPort.ePortID = M4U_PORT_LCD_W;
		sPort.Virtuality = 0;
		sPort.Security = 0;
		sPort.Distance = 1;
		sPort.Direction = 0;
		m4u_config_port(&sPort);
	}

	config.srcModule = DISP_MODULE_OVL;
	config.addr = (unsigned int)pSrcPa;
	config.inFormat = eRGB888;
	config.pitch = DDP_TEST_WIDTH;
	config.srcROI.x = 0;
	config.srcROI.y = 0;
	config.srcROI.width = DDP_TEST_WIDTH;
	config.srcROI.height = DDP_TEST_HEIGHT;
	config.srcWidth = DDP_TEST_WIDTH;
	config.srcHeight = DDP_TEST_HEIGHT;
	config.dstModule = DISP_MODULE_WDMA0;
	config.outFormat = eRGB888;
	config.dstAddr = (unsigned int)pDstPa;
	config.dstWidth = DDP_TEST_WIDTH;
	config.dstHeight = DDP_TEST_HEIGHT;
	config.dstPitch = DDP_TEST_WIDTH;

	disp_path_get_mutex_(DDP_MUTEX_FOR_ROT_SCL_WDMA);
	disp_path_config_(&config, DDP_MUTEX_FOR_ROT_SCL_WDMA);

	DISP_MSG("*after ddp test config start: -------------------\n");
	disp_dump_reg(DISP_MODULE_OVL);
	disp_dump_reg(DISP_MODULE_WDMA0);
	disp_dump_reg(DISP_MODULE_CONFIG);
	DISP_MSG("*after ddp test config end: ---------------------\n");

	disp_path_release_mutex_(DDP_MUTEX_FOR_ROT_SCL_WDMA);
	if (DISP_REG_GET(DISP_REG_CONFIG_MUTEX1) != 0)
		DISP_REG_SET(DISP_REG_CONFIG_MUTEX1, 0);

	DISP_MSG("ddp_mem_test wdma wait done...\n");
	WDMAWait(0);
	DISP_MSG("ddp_mem_test wdma done!\n");

	if (0) {		/* compare source */
		unsigned int diff_cnt = 0;
		unsigned int t = 0;
		unsigned int size = DDP_TEST_WIDTH * DDP_TEST_HEIGHT * DDP_TEST_BPP;
		for (t = 0; t < size; t++) {
			if (*((unsigned char *)pSrc + t) != *((unsigned char *)data_rgb888_64x64 + t)) {
				diff_cnt++;
				DISP_MSG("t=%d, diff_cnt=%d, dst=0x%x, gold=0x%x\n",
				       t,
				       diff_cnt,
				       *((unsigned char *)pSrc + t),
				       *((unsigned char *)data_rgb888_64x64 + t));
			}

		}
		if (diff_cnt == 0)
			DISP_MSG("ddp_mem_test src compare result: success\n");
		else {
			DISP_MSG("ddp_mem_test src compare result: fail\n");
			DISP_MSG("detail, %d, %d, %%%d\n", diff_cnt, size, diff_cnt * 100 / size);
			result = -1;
		}
	}

	if (1) {		/* compare dst */
		unsigned int diff_cnt = 0;
		unsigned int t = 0;
		unsigned int size = DDP_TEST_WIDTH * DDP_TEST_HEIGHT * DDP_TEST_BPP;
		for (t = 0; t < size; t++) {
			if (*((unsigned char *)pDst + t) != *((unsigned char *)data_rgb888_64x64_golden + t)) {
				diff_cnt++;
				DISP_MSG("t=%d, diff_cnt=%d, dst=0x%x, gold=0x%x\n",
				       t,
				       diff_cnt,
				       *((unsigned char *)pDst + t),
				       *((unsigned char *)data_rgb888_64x64_golden + t));
			}

		}
		if (diff_cnt == 0)
			DISP_MSG("ddp_mem_test result: success\n");
		else {
			DISP_MSG("ddp_mem_test result: fail\n");
			DISP_MSG("detail, %d, %d, %%%d\n", diff_cnt, size, diff_cnt * 100 / size);
			result = -1;
		}
	}

	/* print out dst buffer to save as golden */
	if (0) {
		unsigned int t = 0;
		unsigned int size = DDP_TEST_WIDTH * DDP_TEST_HEIGHT * DDP_TEST_BPP;

		for (t = 0; t < size; t++) {
			DISP_MSG("0x%x, ", *((unsigned char *)pDst + t));
			if ((t + 1) % 12 == 0)
				DISP_MSG("\n%05d: ", (t + 1) / 12);
		}
	}

	/* dealloc memory */
	dma_free_coherent(NULL, DDP_TEST_WIDTH * DDP_TEST_HEIGHT * DDP_TEST_BPP, pSrc, (dma_addr_t) &pSrcPa);
	dma_free_coherent(NULL, DDP_TEST_WIDTH * DDP_TEST_HEIGHT * DDP_TEST_BPP, pDst, (dma_addr_t) &pDstPa);

	return result;
}
