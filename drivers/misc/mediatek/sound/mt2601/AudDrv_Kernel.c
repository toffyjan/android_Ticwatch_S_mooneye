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
/*******************************************************************************
 *
 * Filename:
 * ---------
 *   AudDrv_Kernelc
 *
 * Project:
 * --------
 *   MT2601  Audio Driver Kernel Function
 *
 * Description:
 * ------------
 *   Audio register
 *
 * Author:
 * -------
 * Chipeng Chang
 *
 *------------------------------------------------------------------------------
 * $Revision: #1 $
 * $Modtime:$
 * $Log:$
 *
 *
 *******************************************************************************/


/*****************************************************************************
 *                     C O M P I L E R   F L A G S
 *****************************************************************************/


/*****************************************************************************
 *                E X T E R N A L   R E F E R E N C E S
 *****************************************************************************/

#define CONFIG_MTK_DEEP_IDLE

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/completion.h>
#include <linux/mm.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/vmalloc.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/sched.h>
#include <linux/wakelock.h>
#include <linux/semaphore.h>
#include <linux/jiffies.h>
#include <linux/proc_fs.h>
#include <linux/string.h>
#include <linux/mutex.h>
#include <linux/xlog.h>
#include <mach/irqs.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/div64.h>
#include <mt-plat/aee.h>
#include <mach/pmic_mt6323_sw.h>
#include <mach/upmu_common.h>
#include <mach/upmu_hw.h>

#include <mach/mt_gpio.h>
#include <mach/mt_typedefs.h>

#include <linux/seq_file.h>

#ifdef CONFIG_MTK_DEEP_IDLE
#include <mach/mt_clkmgr.h>
#include <mach/mt_idle.h>
#endif

#include "AudDrv_Common.h"
#include "AudDrv_Def.h"
#include "AudDrv_Afe.h"
#include "AudDrv_Ana.h"
#include "AudDrv_Clk.h"
#include "AudDrv_ioctl.h"
#include "AudDrv_Kernel.h"

#include "yusu_android_speaker.h"
#if defined(CONFIG_MTK_COMBO) || defined(CONFIG_MTK_COMBO_MODULE)
#include <mt-plat/mtk_wcn_cmb_stub.h>
#endif

#if defined(MTK_MT5192) || defined(MTK_MT5193)
extern int cust_matv_gpio_on(void);
extern int cust_matv_gpio_off(void);
#endif

/* #define GPIO_WR32(addr, data)   __raw_writel(data, addr) */
/* #define GPIO_RD32(addr)         __raw_readl(addr) */

/*****************************************************************************
*           DEFINE AND CONSTANT
******************************************************************************
*/
#define do_posix_clock_monotonic_gettime(ts) ktime_get_ts(ts)
#define AUDDRV_NAME   "MediaTek Audio Driver"
#define AUDDRV_AUTHOR "MediaTek WCX"

#define AUDDRV_DL1_MAX_BUFFER_LENGTH (0x4000)
#ifndef AUD_AFE_MCU_IRQ_LINE
#define AUD_AFE_MCU_IRQ_LINE    MT_AFE_IRQ_MCU_ID
/* MT_AFE_IRQ_MCU_ID is defined in mt_irq.h */
#endif
#define MASK_ALL		  (0xFFFFFFFF)

#define AFE_INT_TIMEOUT       (10)
#define AFE_UL_TIMEOUT       (10)

/*****************************************************************************
*           V A R I A B L E     D E L A R A T I O N
*******************************************************************************/

static char       auddrv_name[]       = "AudDrv_driver_device";
static u64        AudDrv_dmamask      = 0xffffffffUL;

static volatile bool   AudDrvSuspendStatus; /* is suspend flag */
static bool   AudIrqReset; /* flag when irq to reset */
static bool   AuddrvSpkStatus;
static bool   AuddrvAeeEnable;
static volatile kal_uint8	 Afe_irq_status;

#define WriteArrayMax (6)
#define WriteWarningTrigger (3)
static int WriteArrayIndex;
static unsigned int WriteRecordArray[WriteArrayMax] = {0};

static DEFINE_SPINLOCK(auddrv_lock);
static DEFINE_SPINLOCK(auddrv_irqstatus_lock);
static DEFINE_SPINLOCK(auddrv_SphCtlState_lock);
static DEFINE_SPINLOCK(auddrv_DLCtl_lock);
static DEFINE_SPINLOCK(auddrv_ULInCtl_lock);
static DEFINE_SPINLOCK(auddrv_clk_isr_lock);

/* hold for not let system go into suspend mode */
struct wake_lock  Audio_wake_lock;
struct wake_lock  Audio_record_wake_lock;

/* wait queue flag */
static kal_uint32 DL1_wait_queue_flag;
DECLARE_WAIT_QUEUE_HEAD(DL1_Wait_Queue);
static kal_uint32 DL1_Interrupt_Interval;
static kal_uint32 DL1_Interrupt_Interval_Limit;

/* wait queue flag */
static kal_uint32 DL2_wait_queue_flag;
DECLARE_WAIT_QUEUE_HEAD(DL2_Wait_Queue);

/* VUL quene */
static kal_uint32 VUL_wait_queue_flag;
DECLARE_WAIT_QUEUE_HEAD(VUL_Wait_Queue);

/* AWB quene */
static kal_uint32 AWB_wait_queue_flag;
DECLARE_WAIT_QUEUE_HEAD(AWB_Wait_Queue);

/* MOD_DAI quene */
static kal_uint32 MODDAI_wait_queue_flag;
DECLARE_WAIT_QUEUE_HEAD(MODDAI_Wait_Queue);

/* amp mutex lock */
static DEFINE_MUTEX(gamp_mutex);
static DEFINE_MUTEX(AnaClk_mutex);


/*****************************************************************************
*           FUNCTION     D E L A R A T I O N
*******************************************************************************/
void CheckPowerState(void);
bool GetHeadPhoneState(void);
void Auddrv_Check_Irq(void);
static bool Auddrv_CheckRead_MemIF_Fp(int MEM_Type);
AFE_MEM_CONTROL_T *Auddrv_Find_MemIF_Fp(struct file *fp);
void Auddrv_Handle_DL_Mem_context(AFE_MEM_CONTROL_T *Mem_Block);

/* PMIC AUX ADC function */
int PMIC_IMM_GetOneChannelValue(int dwChannel, int deCount, int trimd);
static void CheckInterruptTiming(void);
static void ClearInterruptTiming(void);
/* here is counter of clock , user extern , clock counter is maintain in auddrv_clk */

extern int        Aud_Core_Clk_cntr;
extern int        Aud_AFE_Clk_cntr;
extern int        Aud_ADC_Clk_cntr;
extern int        Aud_I2S_Clk_cntr;
extern int        Aud_ANA_Clk_cntr;
extern int        Aud_LineIn_Clk_cntr;
extern int        Aud_HDMI_Clk_cntr;
extern int        Afe_Mem_Pwr_on;
extern int        Aud_AFE_Clk_cntr;
int  Aud_Ext_Mem_Flag = 0; /* bits indicate MEMIF_BUFFER_TYPE. 0:MEM_DL1, 1:MEM_DL2, 2:MEM_VUL, 3:MEM_DAI, 4:MEM_I2S, 5:MEM_AWB, 6:MEM_MOD_DAI */
int  Aud_Int_Mem_Flag = 0; /* bits indicate MEMIF_BUFFER_TYPE. 0:MEM_DL1, 1:MEM_DL2, 2:MEM_VUL, 3:MEM_DAI, 4:MEM_I2S, 5:MEM_AWB, 6:MEM_MOD_DAI */

static bool CheckNullPointer(void *pointer)
{
    if (pointer == NULL)
    {
        pr_debug("CheckNullPointer pointer = NULL\n");
        return true;
    }
    return false;
}

static bool CheckSize(uint32 size)
{
    if ((size) == 0)
    {
        pr_debug("CheckSize size = 0\n");
        return true;
    }
    return false;
}

static kal_uint32 AudDrv_SampleRateIndexConvert(kal_uint32 SampleRateIndex)
{
    switch (SampleRateIndex)
    {
        case 0x0:
            return 8000;
        case 0x1:
            return 11025;
        case 0x2:
            return 12000;
        case 0x4:
            return 16000;
        case 0x5:
            return 22050;
        case 0x6:
            return 24000;
        case 0x8:
            return 32000;
        case 0x9:
            return 44100;
        case 0xa:
            return 48000;
        default:
            pr_debug("AudDrv_SampleRateIndexConvert SampleRateIndex = %d\n", SampleRateIndex);
            return 44100;
    }
    return 0;
}

static void AudDrv_getDLInterval(void)
{
    kal_uint32 samplerate = Afe_Get_Reg(AFE_IRQ_CON);
    kal_uint32 InterruptSample = Afe_Get_Reg(AFE_IRQ_CNT1);
    samplerate = (samplerate >> 4) & 0x0000000f;
    samplerate = AudDrv_SampleRateIndexConvert(samplerate);
    DL1_Interrupt_Interval = ((InterruptSample * 1000) / samplerate) + 1;
    DL1_Interrupt_Interval_Limit = DL1_Interrupt_Interval * 11 / 8;
    /* PRINTK_AUDDRV("DL1_Interrupt_Interval = %d DL1_Interrupt_Interval_Limit = %d\n",DL1_Interrupt_Interval,DL1_Interrupt_Interval_Limit); */
}

static void power_init(void)
{
    /* uint32_t chip_version = upmu_get_cid(); */
    upmu_set_rg_clksq_en_aud(1);
    /* Follow Audio Downlink Power on procedure (*Put this sequence in system power on sequence) */
    Ana_Set_Reg(AUDTOP_CON0, 0x0002, 0x000F); /* Set UL PGA L MUX as open */
    Ana_Set_Reg(AUDTOP_CON1, 0x0020, 0x00F0); /* Set UL PGA R MUX as open */
    Ana_Set_Reg(AUDTOP_CON5, 0x1114, 0xFFFF); /* Set audio DAC Bias to 50% */
    Ana_Set_Reg(AUDTOP_CON6, 0x37A2, 0xFFFF);
    Ana_Set_Reg(AUDTOP_CON6, 0x37E2, 0xFFFF); /* Enable the depop MUX of HP drivers */
    upmu_set_rg_clksq_en_aud(0);
}

bool GetHeadPhoneState(void)
{
    uint32 HPAna_reg = 0;
    if (Aud_ANA_Clk_cntr == true)
    {
	HPAna_reg = Ana_Get_Reg(AUDTOP_CON4);
	if (HPAna_reg & 0x60)
	{
	    return true;
	}
    }
    return false;
}

void AudioWayEnable(void)
{
    /*    volatile uint32 address = 0xF0001200;
	volatile uint32 *AudioWayEnable = (volatile uint32*)address;
	volatile uint32 value = 0xF0001200;
	value = (*AudioWayEnable);
	value =(value&0xffffff7f);
	//PRINTK_AUDDRV("AudioWayEnable value = %x\n",value);
	mt_reg_sync_writel(value,AudioWayEnable);*/
}

void AudioWayDisable(void)
{
    /*    volatile uint32 address = 0xF0001200;
	volatile uint32 *AudioWayEnable = (volatile uint32*)address;
	volatile uint32 value = 0xF0001200;
	value = (*AudioWayEnable);
	value |= 0x80;
	//PRINTK_AUDDRV("AudioWayDisable value = %x\n",value);
	mt_reg_sync_writel(value,AudioWayEnable); */
}

void SaveWriteWaitEvent(unsigned int  t2)
{
    WriteRecordArray[WriteArrayIndex] = t2; /* in ms */
    WriteArrayIndex++;
    if (WriteArrayIndex >= WriteArrayMax)
    {
	WriteArrayIndex -= WriteArrayMax;
    }
}

void ResetWriteWaitEvent(void)
{
    int i = 0;
    for (i = 0; i < WriteArrayMax; i++)
    {
	WriteRecordArray[i] = 0;
    }
    /* also reset hardware */
    WriteArrayIndex = 0;
}

void CheckWriteWaitEvent(void)
{
    int i = 0;
    int OverTimeCounter = 0;
    unsigned int DL1_Interrupt_Interval_ns = DL1_Interrupt_Interval_Limit * 1000000;
    for (i = 0; i < WriteArrayMax; i++)
    {
	/* pr_debug("WriteRecordArray[%d] = %d ",i ,WriteRrecordArray[i]); */
	if (WriteRecordArray[i] > DL1_Interrupt_Interval_ns)
	{
	    OverTimeCounter++;
	}
    }
    /* pr_debug("DL1_Interrupt_Interval_Limit = %d DL1_Interrupt_Interval_ns = %d\n",DL1_Interrupt_Interval_Limit,DL1_Interrupt_Interval_ns); */

    if (OverTimeCounter >= WriteWarningTrigger)
    {
	/* xlog_printk(ANDROID_LOG_ERROR, "Sound", "Audio Dump FTrace, OverTimeCounter=%d n", OverTimeCounter); */
	pr_err("Audio Dump FTrace, OverTimeCounter=%d\n", OverTimeCounter);
	if (AuddrvAeeEnable)
	{
	    aee_kernel_exception_api(__FILE__, __LINE__, DB_OPT_FTRACE, "Audio is blocked", "audio blocked dump ftrace");
	}
	ResetWriteWaitEvent();
	/* AudIrqReset = true; */
    }
}


/****************************************************************************
 * FUNCTION
 *  AudDrv_Read_Procmem
 *
 * DESCRIPTION
 *  dump AFE/Analog register
 *  cat /proc/Audio
 *
 * PARAMETERS
 *
 *
 * RETURNS
 *  length
 *
 ***************************************************************************** */

static int AudDrv_Read_Procmem(struct seq_file *buf, void *v)
{
    int len = 0;
    pr_debug("+AudDrv_Read_Procmem\n");
    AudDrv_Clk_On();

    seq_printf(buf+len , "Afe_Mem_Pwr_on =0x%x\n", Afe_Mem_Pwr_on);
    seq_printf(buf+len , "Aud_AFE_Clk_cntr = 0x%x\n", Aud_AFE_Clk_cntr);
    seq_printf(buf+len , "Aud_ANA_Clk_cntr = 0x%x\n", Aud_ANA_Clk_cntr);
    seq_printf(buf+len , "Aud_HDMI_Clk_cntr = 0x%x\n", Aud_HDMI_Clk_cntr);
    seq_printf(buf+len , "Aud_I2S_Clk_cntr = 0x%x\n", Aud_I2S_Clk_cntr);
    seq_printf(buf+len , "Aud_Int_Mem_Flag = 0x%x\n", Aud_Int_Mem_Flag);
    seq_printf(buf+len , "Aud_Ext_Mem_Flag = 0x%x\n", Aud_Ext_Mem_Flag);
    seq_printf(buf+len , "AuddrvSpkStatus = 0x%x\n", AuddrvSpkStatus);

    seq_printf(buf+len , "AUDIO_TOP_CON0  = 0x%x\n", Afe_Get_Reg(AUDIO_AFE_TOP_CON0));
    seq_printf(buf+len , "AUDIO_TOP_CON1  = 0x%x\n", Afe_Get_Reg(AUDIO_AFE_TOP_CON1));
    seq_printf(buf+len , "AUDIO_TOP_CON2  = 0x%x\n", Afe_Get_Reg(AUDIO_AFE_TOP_CON2));
    seq_printf(buf+len , "AUDIO_TOP_CON3  = 0x%x\n", Afe_Get_Reg(AUDIO_AFE_TOP_CON3));
    seq_printf(buf+len , "AFE_DAC_CON0  = 0x%x\n", Afe_Get_Reg(AFE_DAC_CON0));
    seq_printf(buf+len , "AFE_DAC_CON1  = 0x%x\n", Afe_Get_Reg(AFE_DAC_CON1));
    seq_printf(buf+len , "AFE_I2S_CON  = 0x%x\n", Afe_Get_Reg(AFE_I2S_CON));
    seq_printf(buf+len , "AFE_CONN0  = 0x%x\n", Afe_Get_Reg(AFE_CONN0));
    seq_printf(buf+len , "AFE_CONN1  = 0x%x\n", Afe_Get_Reg(AFE_CONN1));
    seq_printf(buf+len , "AFE_CONN2  = 0x%x\n", Afe_Get_Reg(AFE_CONN2));
    seq_printf(buf+len , "AFE_CONN3  = 0x%x\n", Afe_Get_Reg(AFE_CONN3));
    seq_printf(buf+len , "AFE_CONN4  = 0x%x\n", Afe_Get_Reg(AFE_CONN4));
    seq_printf(buf+len , "AFE_I2S_CON1  = 0x%x\n", Afe_Get_Reg(AFE_I2S_CON1));
    seq_printf(buf+len , "AFE_I2S_CON2  = 0x%x\n", Afe_Get_Reg(AFE_I2S_CON2));
    seq_printf(buf+len , "AFE_I2S_CON3  = 0x%x\n", Afe_Get_Reg(AFE_I2S_CON3));

    seq_printf(buf+len , "AFE_DL1_BASE  = 0x%x\n", Afe_Get_Reg(AFE_DL1_BASE));
    seq_printf(buf+len , "AFE_DL1_CUR  = 0x%x\n", Afe_Get_Reg(AFE_DL1_CUR));
    seq_printf(buf+len , "AFE_DL1_END  = 0x%x\n", Afe_Get_Reg(AFE_DL1_END));
    seq_printf(buf+len , "AFE_DL2_BASE  = 0x%x\n", Afe_Get_Reg(AFE_DL2_BASE));
    seq_printf(buf+len , "AFE_DL2_CUR  = 0x%x\n", Afe_Get_Reg(AFE_DL2_CUR));
    seq_printf(buf+len , "AFE_DL2_END  = 0x%x\n", Afe_Get_Reg(AFE_DL2_END));
    seq_printf(buf+len , "AFE_AWB_BASE  = 0x%x\n", Afe_Get_Reg(AFE_AWB_BASE));
    seq_printf(buf+len , "AFE_AWB_END  = 0x%x\n", Afe_Get_Reg(AFE_AWB_END));
    seq_printf(buf+len , "AFE_AWB_CUR  = 0x%x\n", Afe_Get_Reg(AFE_AWB_CUR));
    seq_printf(buf+len , "AFE_VUL_BASE  = 0x%x\n", Afe_Get_Reg(AFE_VUL_BASE));
    seq_printf(buf+len , "AFE_VUL_END  = 0x%x\n", Afe_Get_Reg(AFE_VUL_END));
    seq_printf(buf+len , "AFE_VUL_CUR  = 0x%x\n", Afe_Get_Reg(AFE_VUL_CUR));

    seq_printf(buf+len , "AFE_MEMIF_MON0  = 0x%x\n", Afe_Get_Reg(AFE_MEMIF_MON0));
    seq_printf(buf+len , "AFE_MEMIF_MON1  = 0x%x\n", Afe_Get_Reg(AFE_MEMIF_MON1));
    seq_printf(buf+len , "AFE_MEMIF_MON2  = 0x%x\n", Afe_Get_Reg(AFE_MEMIF_MON2));
    seq_printf(buf+len , "AFE_MEMIF_MON4  = 0x%x\n", Afe_Get_Reg(AFE_MEMIF_MON4));

    seq_printf(buf+len , "AFE_ADDA_DL_SRC2_CON0  = 0x%x\n", Afe_Get_Reg(AFE_ADDA_DL_SRC2_CON0));
    seq_printf(buf+len , "AFE_ADDA_DL_SRC2_CON1  = 0x%x\n", Afe_Get_Reg(AFE_ADDA_DL_SRC2_CON1));
    seq_printf(buf+len , "AFE_ADDA_UL_SRC_CON0  = 0x%x\n", Afe_Get_Reg(AFE_ADDA_UL_SRC_CON0));
    seq_printf(buf+len , "AFE_ADDA_UL_SRC_CON1  = 0x%x\n", Afe_Get_Reg(AFE_ADDA_UL_SRC_CON1));
    seq_printf(buf+len , "AFE_ADDA_TOP_CON0  = 0x%x\n", Afe_Get_Reg(AFE_ADDA_TOP_CON0));
    seq_printf(buf+len , "AFE_ADDA_UL_DL_CON0  = 0x%x\n", Afe_Get_Reg(AFE_ADDA_UL_DL_CON0));
    seq_printf(buf+len , "AFE_ADDA_SRC_DEBUG  = 0x%x\n", Afe_Get_Reg(AFE_ADDA_SRC_DEBUG));
    seq_printf(buf+len , "AFE_ADDA_SRC_DEBUG_MON0  = 0x%x\n", Afe_Get_Reg(AFE_ADDA_SRC_DEBUG_MON0));
    seq_printf(buf+len , "AFE_ADDA_SRC_DEBUG_MON1  = 0x%x\n", Afe_Get_Reg(AFE_ADDA_SRC_DEBUG_MON1));
    seq_printf(buf+len , "AFE_ADDA_NEWIF_CFG0  = 0x%x\n", Afe_Get_Reg(AFE_ADDA_NEWIF_CFG0));
    seq_printf(buf+len , "AFE_ADDA_NEWIF_CFG1  = 0x%x\n", Afe_Get_Reg(AFE_ADDA_NEWIF_CFG1));

    seq_printf(buf+len , "AFE_SIDETONE_DEBUG  = 0x%x\n", Afe_Get_Reg(AFE_SIDETONE_DEBUG));
    seq_printf(buf+len , "AFE_SIDETONE_MON  = 0x%x\n", Afe_Get_Reg(AFE_SIDETONE_MON));
    seq_printf(buf+len , "AFE_SIDETONE_CON0  = 0x%x\n", Afe_Get_Reg(AFE_SIDETONE_CON0));
    seq_printf(buf+len , "AFE_SIDETONE_COEFF  = 0x%x\n", Afe_Get_Reg(AFE_SIDETONE_COEFF));
    seq_printf(buf+len , "AFE_SIDETONE_CON1  = 0x%x\n", Afe_Get_Reg(AFE_SIDETONE_CON1));
    seq_printf(buf+len , "AFE_SIDETONE_GAIN  = 0x%x\n", Afe_Get_Reg(AFE_SIDETONE_GAIN));
    seq_printf(buf+len , "AFE_SGEN_CON0  = 0x%x\n", Afe_Get_Reg(AFE_SGEN_CON0));

    seq_printf(buf+len , "AFE_TOP_CON0  = 0x%x\n", Afe_Get_Reg(AFE_TOP_CON0));
    seq_printf(buf+len , "AFE_ADDA_PREDIS_CON0  = 0x%x\n", Afe_Get_Reg(AFE_ADDA_PREDIS_CON0));
    seq_printf(buf+len , "AFE_ADDA_PREDIS_CON1  = 0x%x\n", Afe_Get_Reg(AFE_ADDA_PREDIS_CON1));

    seq_printf(buf+len , "AFE_IRQ_CON  = 0x%x\n", Afe_Get_Reg(AFE_IRQ_CON));
    seq_printf(buf+len , "AFE_IRQ_STATUS  = 0x%x\n", Afe_Get_Reg(AFE_IRQ_STATUS));
    seq_printf(buf+len , "AFE_IRQ_CLR  = 0x%x\n", Afe_Get_Reg(AFE_IRQ_CLR));
    seq_printf(buf+len , "AFE_IRQ_CNT1  = 0x%x\n", Afe_Get_Reg(AFE_IRQ_CNT1));
    seq_printf(buf+len , "AFE_IRQ_CNT2  = 0x%x\n", Afe_Get_Reg(AFE_IRQ_CNT2));
    seq_printf(buf+len , "AFE_IRQ_MON2  = 0x%x\n", Afe_Get_Reg(AFE_IRQ_MON2));
    seq_printf(buf+len , "AFE_IRQ1_CNT_MON  = 0x%x\n", Afe_Get_Reg(AFE_IRQ1_CNT_MON));
    seq_printf(buf+len , "AFE_IRQ2_CNT_MON  = 0x%x\n", Afe_Get_Reg(AFE_IRQ2_CNT_MON));
    seq_printf(buf+len , "AFE_IRQ1_EN_CNT_MON  = 0x%x\n", Afe_Get_Reg(AFE_IRQ1_EN_CNT_MON));

    seq_printf(buf+len , "AFE_MEMIF_MAXLEN  = 0x%x\n", Afe_Get_Reg(AFE_MEMIF_MAXLEN));

    seq_printf(buf+len , "AFE_GAIN1_CON0  = 0x%x\n", Afe_Get_Reg(AFE_GAIN1_CON0));
    seq_printf(buf+len , "AFE_GAIN1_CON1  = 0x%x\n", Afe_Get_Reg(AFE_GAIN1_CON1));
    seq_printf(buf+len , "AFE_GAIN1_CON2  = 0x%x\n", Afe_Get_Reg(AFE_GAIN1_CON2));
    seq_printf(buf+len , "AFE_GAIN1_CON3  = 0x%x\n", Afe_Get_Reg(AFE_GAIN1_CON3));
    seq_printf(buf+len , "AFE_GAIN1_CONN  = 0x%x\n", Afe_Get_Reg(AFE_GAIN1_CONN));
    seq_printf(buf+len , "AFE_GAIN1_CUR  = 0x%x\n", Afe_Get_Reg(AFE_GAIN1_CUR));

    seq_printf(buf+len , "AFE_GAIN2_CON0  = 0x%x\n", Afe_Get_Reg(AFE_GAIN2_CON0));
    seq_printf(buf+len , "AFE_GAIN2_CON1  = 0x%x\n", Afe_Get_Reg(AFE_GAIN2_CON1));
    seq_printf(buf+len , "AFE_GAIN2_CON2  = 0x%x\n", Afe_Get_Reg(AFE_GAIN2_CON2));
    seq_printf(buf+len , "AFE_GAIN2_CON3  = 0x%x\n", Afe_Get_Reg(AFE_GAIN2_CON3));
    seq_printf(buf+len , "AFE_GAIN2_CONN  = 0x%x\n", Afe_Get_Reg(AFE_GAIN2_CONN));
    seq_printf(buf+len , "AFE_GAIN2_CONN2  = 0x%x\n", Afe_Get_Reg(AFE_GAIN2_CONN2));
    seq_printf(buf+len , "AFE_GAIN2_CUR  = 0x%x\n", Afe_Get_Reg(AFE_GAIN2_CUR));

    seq_printf(buf+len , "FPGA_CFG0  = 0x%x\n", Afe_Get_Reg(FPGA_CFG0));
    seq_printf(buf+len , "FPGA_CFG1  = 0x%x\n", Afe_Get_Reg(FPGA_CFG1));

    seq_printf(buf+len , "AFE_ASRC_CON0  = 0x%x\n", Afe_Get_Reg(AFE_ASRC_CON0));
    seq_printf(buf+len , "AFE_ASRC_CON1  = 0x%x\n", Afe_Get_Reg(AFE_ASRC_CON1));
    seq_printf(buf+len , "AFE_ASRC_CON2  = 0x%x\n", Afe_Get_Reg(AFE_ASRC_CON2));
    seq_printf(buf+len , "AFE_ASRC_CON3  = 0x%x\n", Afe_Get_Reg(AFE_ASRC_CON3));
    seq_printf(buf+len , "AFE_ASRC_CON4  = 0x%x\n", Afe_Get_Reg(AFE_ASRC_CON4));
    seq_printf(buf+len , "AFE_ASRC_CON5  = 0x%x\n", Afe_Get_Reg(AFE_ASRC_CON5));
    seq_printf(buf+len , "AFE_ASRC_CON6  = 0x%x\n", Afe_Get_Reg(AFE_ASRC_CON6));
    seq_printf(buf+len , "AFE_ASRC_CON7  = 0x%x\n", Afe_Get_Reg(AFE_ASRC_CON7));
    seq_printf(buf+len , "AFE_ASRC_CON8  = 0x%x\n", Afe_Get_Reg(AFE_ASRC_CON8));
    seq_printf(buf+len , "AFE_ASRC_CON9  = 0x%x\n", Afe_Get_Reg(AFE_ASRC_CON9));
    seq_printf(buf+len , "AFE_ASRC_CON10 = 0x%x\n", Afe_Get_Reg(AFE_ASRC_CON10));
    seq_printf(buf+len , "AFE_ASRC_CON11 = 0x%x\n", Afe_Get_Reg(AFE_ASRC_CON11));

    seq_printf(buf+len , "PCM2_INTF_CON  = 0x%x\n", Afe_Get_Reg(PCM2_INTF_CON));

    seq_printf(buf+len , "AFE_ASRC_CON13  = 0x%x\n", Afe_Get_Reg(AFE_ASRC_CON13));
    seq_printf(buf+len , "AFE_ASRC_CON14  = 0x%x\n", Afe_Get_Reg(AFE_ASRC_CON14));
    seq_printf(buf+len , "AFE_ASRC_CON15  = 0x%x\n", Afe_Get_Reg(AFE_ASRC_CON15));
    seq_printf(buf+len , "AFE_ASRC_CON16  = 0x%x\n", Afe_Get_Reg(AFE_ASRC_CON16));
    seq_printf(buf+len , "AFE_ASRC_CON17  = 0x%x\n", Afe_Get_Reg(AFE_ASRC_CON17));
    seq_printf(buf+len , "AFE_ASRC_CON18  = 0x%x\n", Afe_Get_Reg(AFE_ASRC_CON18));
    seq_printf(buf+len , "AFE_ASRC_CON19  = 0x%x\n", Afe_Get_Reg(AFE_ASRC_CON19));
    seq_printf(buf+len , "AFE_ASRC_CON20  = 0x%x\n", Afe_Get_Reg(AFE_ASRC_CON20));
    seq_printf(buf+len , "AFE_ASRC_CON21  = 0x%x\n", Afe_Get_Reg(AFE_ASRC_CON21));

    seq_printf(buf+len , "ABB_AFE_CON0  = 0x%x\n", Ana_Get_Reg(ABB_AFE_CON0));
    seq_printf(buf+len , "ABB_AFE_CON1  = 0x%x\n", Ana_Get_Reg(ABB_AFE_CON1));
    seq_printf(buf+len , "ABB_AFE_CON2  = 0x%x\n", Ana_Get_Reg(ABB_AFE_CON2));
    seq_printf(buf+len , "ABB_AFE_CON3  = 0x%x\n", Ana_Get_Reg(ABB_AFE_CON3));
    seq_printf(buf+len , "ABB_AFE_CON4  = 0x%x\n", Ana_Get_Reg(ABB_AFE_CON4));
    seq_printf(buf+len , "ABB_AFE_CON5  = 0x%x\n", Ana_Get_Reg(ABB_AFE_CON5));
    seq_printf(buf+len , "ABB_AFE_CON6  = 0x%x\n", Ana_Get_Reg(ABB_AFE_CON6));
    seq_printf(buf+len , "ABB_AFE_CON7  = 0x%x\n", Ana_Get_Reg(ABB_AFE_CON7));
    seq_printf(buf+len , "ABB_AFE_CON8  = 0x%x\n", Ana_Get_Reg(ABB_AFE_CON8));
    seq_printf(buf+len , "ABB_AFE_CON9  = 0x%x\n", Ana_Get_Reg(ABB_AFE_CON9));
    seq_printf(buf+len , "ABB_AFE_CON10 = 0x%x\n", Ana_Get_Reg(ABB_AFE_CON10));
    seq_printf(buf+len , "ABB_AFE_CON11 = 0x%x\n", Ana_Get_Reg(ABB_AFE_CON11));

    seq_printf(buf+len , "ABB_AFE_STA0  = 0x%x\n", Ana_Get_Reg(ABB_AFE_STA0));
    seq_printf(buf+len , "ABB_AFE_STA1  = 0x%x\n", Ana_Get_Reg(ABB_AFE_STA1));
    seq_printf(buf+len , "ABB_AFE_STA2  = 0x%x\n", Ana_Get_Reg(ABB_AFE_STA2));

    seq_printf(buf+len , "AFE_UP8X_FIFO_CFG0  = 0x%x\n", Ana_Get_Reg(AFE_UP8X_FIFO_CFG0));
    seq_printf(buf+len , "AFE_UP8X_FIFO_LOG_MON0  = 0x%x\n", Ana_Get_Reg(AFE_UP8X_FIFO_LOG_MON0));
    seq_printf(buf+len , "AFE_UP8X_FIFO_LOG_MON1  = 0x%x\n", Ana_Get_Reg(AFE_UP8X_FIFO_LOG_MON1));
    seq_printf(buf+len , "AFE_PMIC_NEWIF_CFG0  = 0x%x\n", Ana_Get_Reg(AFE_PMIC_NEWIF_CFG0));
    seq_printf(buf+len , "AFE_PMIC_NEWIF_CFG1  = 0x%x\n", Ana_Get_Reg(AFE_PMIC_NEWIF_CFG1));
    seq_printf(buf+len , "AFE_PMIC_NEWIF_CFG2  = 0x%x\n", Ana_Get_Reg(AFE_PMIC_NEWIF_CFG2));
    seq_printf(buf+len , "AFE_PMIC_NEWIF_CFG3  = 0x%x\n", Ana_Get_Reg(AFE_PMIC_NEWIF_CFG3));

    seq_printf(buf+len , "AFE_TOP_PMIC_CON0  = 0x%x\n", Ana_Get_Reg(AFE_TOP_PMIC_CON0));
    seq_printf(buf+len , "ABB_MON_DEBUG0  = 0x%x\n", Ana_Get_Reg(ABB_MON_DEBUG0));

    seq_printf(buf+len , "TOP_CKPDN0  = 0x%x\n", Ana_Get_Reg(TOP_CKPDN0));
    seq_printf(buf+len , "TOP_CKPDN0_CLR  = 0x%x\n", Ana_Get_Reg(TOP_CKPDN0_CLR));
    seq_printf(buf+len , "TOP_CKPDN1  = 0x%x\n", Ana_Get_Reg(TOP_CKPDN1));
    seq_printf(buf+len , "TOP_CKCON1  = 0x%x\n", Ana_Get_Reg(TOP_CKCON1));

    seq_printf(buf+len , "SPK_CON0  = 0x%x\n", Ana_Get_Reg(SPK_CON0));
    seq_printf(buf+len , "SPK_CON1  = 0x%x\n", Ana_Get_Reg(SPK_CON1));
    seq_printf(buf+len , "SPK_CON2  = 0x%x\n", Ana_Get_Reg(SPK_CON2));
    seq_printf(buf+len , "SPK_CON6  = 0x%x\n", Ana_Get_Reg(SPK_CON6));
    seq_printf(buf+len , "SPK_CON7  = 0x%x\n", Ana_Get_Reg(SPK_CON7));
    seq_printf(buf+len , "SPK_CON8  = 0x%x\n", Ana_Get_Reg(SPK_CON8));
    seq_printf(buf+len , "SPK_CON9  = 0x%x\n", Ana_Get_Reg(SPK_CON9));
    seq_printf(buf+len , "SPK_CON10  = 0x%x\n", Ana_Get_Reg(SPK_CON10));
    seq_printf(buf+len , "SPK_CON11  = 0x%x\n", Ana_Get_Reg(SPK_CON11));
    seq_printf(buf+len , "SPK_CON12  = 0x%x\n", Ana_Get_Reg(SPK_CON12));

    seq_printf(buf+len , "AUDTOP_CON0  = 0x%x\n", Ana_Get_Reg(AUDTOP_CON0));
    seq_printf(buf+len , "AUDTOP_CON1  = 0x%x\n", Ana_Get_Reg(AUDTOP_CON1));
    seq_printf(buf+len , "AUDTOP_CON2  = 0x%x\n", Ana_Get_Reg(AUDTOP_CON2));
    seq_printf(buf+len , "AUDTOP_CON3  = 0x%x\n", Ana_Get_Reg(AUDTOP_CON3));
    seq_printf(buf+len , "AUDTOP_CON4  = 0x%x\n", Ana_Get_Reg(AUDTOP_CON4));
    seq_printf(buf+len , "AUDTOP_CON5  = 0x%x\n", Ana_Get_Reg(AUDTOP_CON5));
    seq_printf(buf+len , "AUDTOP_CON6  = 0x%x\n", Ana_Get_Reg(AUDTOP_CON6));
    seq_printf(buf+len , "AUDTOP_CON7  = 0x%x\n", Ana_Get_Reg(AUDTOP_CON7));
    seq_printf(buf+len , "AUDTOP_CON8  = 0x%x\n", Ana_Get_Reg(AUDTOP_CON8));
    seq_printf(buf+len , "AUDTOP_CON9  = 0x%x\n", Ana_Get_Reg(AUDTOP_CON9));
    pr_debug("AudDrv_Read_Procmem len = %d\n", len);

    AudDrv_Clk_Off();

    if (len > 4096)
    {
	pr_err("Audio Dump FTrace, AudDrv_Read_Procmem len=%d\n", len);
	aee_kernel_exception_api(__FILE__, __LINE__, DB_OPT_FTRACE, "AudDrv_Read_Procmem OverFlow", "AudDrv_Read_Procmem OverFlow");
    }

    pr_debug("-AudDrv_Read_Procmem\n");
    return len;
}

static int Auddrv_proc_open(struct inode *inode, struct file *file)
{
   return single_open(file, AudDrv_Read_Procmem, NULL);
}

static const struct file_operations auddrv_proc_fops = {
    .open    = Auddrv_proc_open,
    .read    = seq_read,
    .llseek  = seq_lseek,
    .release = seq_release,
};

void Auddrv_Handle_Mem_context(AFE_MEM_CONTROL_T *Mem_Block)
{
    kal_uint32 HW_Cur_ReadIdx = 0;
    kal_uint32 HW_Begin_Idx = 0;
    kal_uint32 HW_End_Idx = 0;
    kal_int32 Hw_Get_bytes = 0;
    AFE_BLOCK_T  *mBlock = NULL;
    unsigned long flags;

    if (Mem_Block == NULL || AudDrvSuspendStatus == true)
    {
        pr_debug("[Auddrv] Mem_Block ==[NULL] || AudDrvSuspendStatus = [%d]\n", AudDrvSuspendStatus);
        return;
    }

    switch (Mem_Block->MemIfNum)
    {
        case MEM_VUL:
            HW_Cur_ReadIdx = Afe_Get_Reg(AFE_VUL_CUR);
            HW_Begin_Idx = Afe_Get_Reg(AFE_VUL_BASE);
            HW_End_Idx = Afe_Get_Reg(AFE_VUL_END);
            break;
        case MEM_AWB:
            HW_Cur_ReadIdx = Afe_Get_Reg(AFE_AWB_CUR);
            HW_Begin_Idx = Afe_Get_Reg(AFE_AWB_BASE);
            HW_End_Idx = Afe_Get_Reg(AFE_AWB_END);
            break;
        case MEM_MOD_DAI:
            HW_Cur_ReadIdx = Afe_Get_Reg(AFE_MOD_PCM_CUR);
            HW_Begin_Idx = Afe_Get_Reg(AFE_MOD_PCM_BASE);
            HW_End_Idx = Afe_Get_Reg(AFE_MOD_PCM_END);
            break;
    }
    mBlock = &Mem_Block->rBlock;

    if (CheckSize(HW_Cur_ReadIdx))
    {
        pr_debug("AudDrv: HW_Cur_ReadIdx 0");
        return;
    }

    if (HW_Cur_ReadIdx < HW_Begin_Idx || HW_Cur_ReadIdx > HW_End_Idx)
    {
        PRINTK_AUDDRV("AudDrv: HW_Cur_ReadIdx over range");
        return;
    }

    if (mBlock->pucVirtBufAddr  == NULL)
    {
        pr_debug("AudDrv: Auddrv_Handle_Mem_context pucVirtBufAddr NULL");
        return;
    }

    /* HW already fill in */
    Hw_Get_bytes = (HW_Cur_ReadIdx - mBlock->pucPhysBufAddr) - mBlock->u4WriteIdx;
    if (Hw_Get_bytes < 0)
    {
        Hw_Get_bytes = (Hw_Get_bytes + mBlock->u4BufferSize) % mBlock->u4BufferSize;
    }

    /*
    PRINTK_AUDDRV("Auddrv_Handle_Mem_context Hw_Get_bytes:%x, HW_Cur_ReadIdx:%x, u4DMAReadIdx:%x, u4WriteIdx:0x%x, pucPhysBufAddr:%x Mem_Block->MemIfNum = %d\n",
      Hw_Get_bytes,HW_Cur_ReadIdx,mBlock->u4DMAReadIdx,mBlock->u4WriteIdx,mBlock->pucPhysBufAddr,Mem_Block->MemIfNum);*/

    spin_lock_irqsave(&auddrv_ULInCtl_lock, flags);

    mBlock->u4WriteIdx  += Hw_Get_bytes;
    mBlock->u4WriteIdx  %= mBlock->u4BufferSize;
    mBlock->u4DataRemained += Hw_Get_bytes;

    /* buffer overflow */
    if (mBlock->u4DataRemained > mBlock->u4BufferSize)
    {
        pr_debug("AudDrv=>+UL_Handling overflow ");
        pr_debug("AudDrv=>DataRemained:0x%x R:0x%x W:0x%x BufSize:0x%x Hw_Get_bytes:0x%x ", mBlock->u4DataRemained, mBlock->u4DMAReadIdx, mBlock->u4WriteIdx, mBlock->u4BufferSize, Hw_Get_bytes);
        pr_debug("AudDrv=>HW_Cur_ReadIdx:0x%x pucPhysBufAddr:0x%x", HW_Cur_ReadIdx, mBlock->pucPhysBufAddr);

        mBlock->u4DataRemained = mBlock->u4BufferSize / 2;
        mBlock->u4DMAReadIdx = mBlock->u4WriteIdx - mBlock->u4BufferSize / 2;
        if (mBlock->u4DMAReadIdx < 0)
        {
            mBlock->u4DMAReadIdx = (mBlock->u4DMAReadIdx + mBlock->u4BufferSize) % mBlock->u4BufferSize;
        }
    }
    spin_unlock_irqrestore(&auddrv_ULInCtl_lock, flags);

    switch (Mem_Block->MemIfNum)
    {
        case MEM_VUL:
            VUL_wait_queue_flag = 1;
            wake_up_interruptible(&VUL_Wait_Queue);
            break;
        case MEM_AWB:
            AWB_wait_queue_flag = 1;
            wake_up_interruptible(&AWB_Wait_Queue);
            break;
        case MEM_MOD_DAI:
            MODDAI_wait_queue_flag = 1;
            wake_up_interruptible(&MODDAI_Wait_Queue);
            break;
        default:
            break;
    }
}


/*****************************************************************************
 * FUNCTION
 *  Auddrv_DL1_Interrupt_Handler
 *
 * DESCRIPTION
 * update hardware pointer and send event to write thread to copy data into hardware buffer
 *
 * PARAMETERS
 *  None
 *
 * RETURNS
 *  None
 *
 *****************************************************************************
*/

void Auddrv_UL_Interrupt_Handler(void)  /* irq2 ISR handler */
{
    unsigned long flags;
    kal_uint32 Afe_Dac_Con0 = Afe_Get_Reg(AFE_DAC_CON0);
    AFE_MEM_CONTROL_T *Mem_Block = NULL;
    spin_lock_irqsave(&auddrv_irqstatus_lock, flags);
    if (Afe_Dac_Con0 & 0x8)
    {
	/* handle VUL Context */
	Mem_Block = &VUL_Control_context;
	Auddrv_Handle_Mem_context(Mem_Block);
    }
    if (Afe_Dac_Con0 & 0x40)
    {
	/* handle AWB Context */
	Mem_Block = &AWB_Control_context;
	Auddrv_Handle_Mem_context(Mem_Block);
    }
    if (Afe_Dac_Con0 & 0x80)
    {
	/* handle MODDAI context */
	Mem_Block = &MODDAI_Control_context;
	Auddrv_Handle_Mem_context(Mem_Block);
    }
    spin_unlock_irqrestore(&auddrv_irqstatus_lock, flags);
}

/*****************************************************************************
 * FUNCTION
 *  Auddrv_DL1_Interrupt_Handler
 *
 * DESCRIPTION
 * update hardware pointer and send event to write thread to copy data into hardware buffer
 *
 * PARAMETERS
 *  None
 *
 * RETURNS
 *  None
 *
 *****************************************************************************
 */
void Auddrv_DL_Interrupt_Handler(void)  /* irq1 ISR handler */
{
    unsigned long flags;
    kal_uint32 Afe_Dac_Con0 = Afe_Get_Reg(AFE_DAC_CON0);
    AFE_MEM_CONTROL_T *Mem_Block = NULL;
    spin_lock_irqsave(&auddrv_irqstatus_lock, flags);
    if (Afe_Dac_Con0 & 0x2)
    {
        // handle DL1 Context
        pr_debug("%s  AFE_dL1_Control_context\n",__func__);
        Mem_Block = &AFE_dL1_Control_context;
        Auddrv_Handle_DL_Mem_context(&AFE_dL1_Control_context);
    }
    if (Afe_Dac_Con0 & 0x4)
    {
        // handle DL2 Context
        pr_debug("%s  AFE_dL2_Control_context\n",__func__);
        Mem_Block = &AFE_dL2_Control_context;
        Auddrv_Handle_DL_Mem_context(&AFE_dL2_Control_context);
    }
    spin_unlock_irqrestore(&auddrv_irqstatus_lock, flags);

}

void Auddrv_Handle_DL_Mem_context(AFE_MEM_CONTROL_T *Mem_Block)
{
    //unsigned long flags = 0;
    kal_int32 Afe_consumed_bytes = 0;
    kal_int32 HW_memory_index = 0;
    kal_int32 HW_Cur_ReadIdx = 0;

    AFE_MEM_CONTROL_T  *pAfe_MEM_ConTrol = NULL;
    AFE_BLOCK_T  *Afe_Block = NULL;

    // check which memif need to be read
    pAfe_MEM_ConTrol = Mem_Block;
    Afe_Block = &(pAfe_MEM_ConTrol->rBlock);

    pr_debug("%s pAfe_MEM_ConTrol->MemIfNum = %d Mem_Block->MemIfNum = %d\n ", __func__,pAfe_MEM_ConTrol->MemIfNum,Mem_Block->MemIfNum);

    if (pAfe_MEM_ConTrol == NULL)
    {
        pr_debug("cannot find MEM control !!!!!!!\n");
        return;
    }

    if (Afe_Block->u4BufferSize <= 0)
    {
        pr_debug("Afe_Block->u4BufferSize \n");
        return;
    }

    if (AudDrvSuspendStatus == true)
    {
        HW_Cur_ReadIdx = 0;
    }
    else
    {
        // wait up write thread
        switch (pAfe_MEM_ConTrol->MemIfNum)
        {
            case MEM_DL1:
            {
                HW_Cur_ReadIdx = Afe_Get_Reg(AFE_DL1_CUR);
                break;
            }
            case MEM_DL2:
            {
                HW_Cur_ReadIdx = Afe_Get_Reg(AFE_DL2_CUR);
                break;
            }
            default:
                pr_debug("Auddrv_Handle_DL_Mem_context cannot find match MemIfNum!!!\n");
                HW_Cur_ReadIdx =0;
        }
    }
    /* if(HW_Cur_ReadIdx == 0) */
    if (HW_Cur_ReadIdx < Afe_Block->pucPhysBufAddr || HW_Cur_ReadIdx >= (Afe_Block->pucPhysBufAddr+Afe_Block->u4BufferSize))
    {
        pr_debug("[Auddrv][Warn] HW_Cur_ReadIdx ==0x%x, PhyAddr=0x%x, VirAddr = %p\n", HW_Cur_ReadIdx, Afe_Block->pucPhysBufAddr, Afe_Block->pucVirtBufAddr);
        pr_debug("[Auddrv][Warn] AFE_DL1_BASE ==0x%x, AFE_DL1_END=0x%x\n", Afe_Get_Reg(AFE_DL1_BASE), Afe_Get_Reg(AFE_DL1_END));
        HW_Cur_ReadIdx = Afe_Block->pucPhysBufAddr;
    }
    HW_memory_index = (HW_Cur_ReadIdx - Afe_Block->pucPhysBufAddr);
    pr_debug("[Auddrv] HW_Cur_ReadIdx=0x%x HW_memory_index = 0x%x Afe_Block->pucPhysBufAddr = 0x%x\n",
    HW_Cur_ReadIdx,HW_memory_index,Afe_Block->pucPhysBufAddr);
    /* get hw consume bytes */
    if (HW_memory_index > Afe_Block->u4DMAReadIdx)
    {
        Afe_consumed_bytes = HW_memory_index - Afe_Block->u4DMAReadIdx;
    }
    else
    {
        Afe_consumed_bytes = Afe_Block->u4BufferSize + HW_memory_index - Afe_Block->u4DMAReadIdx;
    }

    if ((Afe_consumed_bytes & 0x3) != 0) {
        pr_debug("[Auddrv] MEMIF address is not aligned 4 bytes\n");
    }

    if (Afe_Block->u4DataRemained < Afe_consumed_bytes || Afe_Block->u4DataRemained <= 0 || Afe_Block->u4DataRemained  > Afe_Block->u4BufferSize || AudIrqReset)
    {
        /* buffer underflow --> clear  whole buffer */
        pr_debug("AudDrv=>+DL_Handling underflow ");
        pr_debug("AudDrv=>DataRemained:0x%x R:0x%x W:0x%x BufSize:0x%x Consumed_bytes:0x%x ", Afe_Block->u4DataRemained, Afe_Block->u4DMAReadIdx, Afe_Block->u4WriteIdx, Afe_Block->u4BufferSize, Afe_consumed_bytes);
        pr_debug("AudDrv=>HW_memory_index:0x%x pucPhysBufAddr:0x%x AudIrqReset:0x%x", HW_memory_index, Afe_Block->pucPhysBufAddr, AudIrqReset);
        memset(Afe_Block->pucVirtBufAddr, 0, Afe_Block->u4BufferSize);
        pr_debug("AudDrv=>-DL_Handling underflow ");
        Afe_Block->u4DMAReadIdx  = HW_memory_index;
        Afe_Block->u4WriteIdx  = Afe_Block->u4DMAReadIdx;
        Afe_Block->u4DataRemained = Afe_Block->u4BufferSize;
        AudIrqReset = false;
    }
    else
    {
        pr_debug("+DL_Handling normal ReadIdx:%x ,DataRemained:%x, WriteIdx:%x\n",
        Afe_Block->u4DMAReadIdx,Afe_Block->u4DataRemained,Afe_Block->u4WriteIdx);
        Afe_Block->u4DataRemained -= Afe_consumed_bytes;
        Afe_Block->u4DMAReadIdx += Afe_consumed_bytes;
        Afe_Block->u4DMAReadIdx %= Afe_Block->u4BufferSize;
        pr_debug("-DL_Handling normal ReadIdx:%x ,DataRemained:%x, WriteIdx:%x\n",
        Afe_Block->u4DMAReadIdx,Afe_Block->u4DataRemained,Afe_Block->u4WriteIdx);
    }
    pr_debug("Auddrv_Handle_DL_Mem_context_+lock1");
    pr_debug("Auddrv_Handle_DL_Mem_context_-lock1");
    /* wait up write thread */
    switch (pAfe_MEM_ConTrol->MemIfNum)
    {
        case MEM_DL1:
        {
            DL1_wait_queue_flag = 1;
            wake_up_interruptible(&DL1_Wait_Queue);
            break;
        }
        case MEM_DL2:
        {
            DL2_wait_queue_flag = 1;
            wake_up_interruptible(&DL2_Wait_Queue);
            pr_debug("Auddrv_Handle_DL_Mem_context_+lock2");
            pr_debug("Auddrv_Handle_DL_Mem_context_-lock2");

            break;
        }
        default:
            pr_err("cannot find matcg MemIfNum!!!\n");
    }
}

static unsigned long long Irq_time_t1 = 0, Irq_time_t2;
static void CheckInterruptTiming(void)
{
    if (Irq_time_t1 == 0)
    {
	Irq_time_t1 = sched_clock(); /* in ns (10^9) */
    }
    else
    {
	Irq_time_t2 = Irq_time_t1;
	Irq_time_t1 = sched_clock(); /* in ns (10^9) */
	if ((Irq_time_t1 > Irq_time_t2) && DL1_Interrupt_Interval_Limit)
	{
	    /*
	    PRINTK_AUDDRV("CheckInterruptTiming  Irq_time_t2 t2 = %llu Irq_time_t1 = %llu Irq_time_t1 - Irq_time_t2 = %llu  DL1_Interrupt_Interval_Limit = %d\n",
		Irq_time_t2,Irq_time_t1, Irq_time_t1 - Irq_time_t2,DL1_Interrupt_Interval_Limit);*/
	    Irq_time_t2 = Irq_time_t1 - Irq_time_t2;
	    if (Irq_time_t2 > DL1_Interrupt_Interval_Limit * 1000000)
	    {
		pr_debug("CheckInterruptTiming interrupt may be blocked Irq_time_t2 = %llu DL1_Interrupt_Interval_Limit = %d\n",
			      Irq_time_t2, DL1_Interrupt_Interval_Limit);
	    }
	}
    }
}

static void ClearInterruptTiming(void)
{
    Irq_time_t1 = 0;
    Irq_time_t2 = 0;
}

/*****************************************************************************
 * FUNCTION
 *  AudDrv_IRQ_handler / AudDrv_magic_tasklet
 *
 * DESCRIPTION
 *  IRQ handler
 *
 *****************************************************************************
 */
static irqreturn_t AudDrv_IRQ_handler(int irq, void *dev_id)
{
    unsigned long flags = 0;
    kal_uint32 volatile u4RegValue = 0;
    kal_uint32 volatile u4RegValue2 = 0;
    spin_lock_irqsave(&auddrv_clk_isr_lock, flags);

    if (Aud_AFE_Clk_cntr > 0)
    {
    u4RegValue = Afe_Get_Reg(AFE_IRQ_STATUS);
	u4RegValue2 = Afe_Get_Reg(AFE_IRQ_CON);
    }
    u4RegValue &= 0xf;
    u4RegValue2 &= 0x3;
    pr_debug("AudDrv_IRQ_handler AFE_IRQ_MCU_STATUS = %x\n",u4RegValue);
    /* here is error handle , for interrupt is trigger but not status , clear all interrupt with bit 6 */
    if (u4RegValue == 0 || Aud_AFE_Clk_cntr == 0 || u4RegValue2 == 0)
    {
	pr_debug("AudDrv u4RegValue == %d Aud_AFE_Clk_cntr = %d u4RegValue2 = %d\n", u4RegValue, Aud_AFE_Clk_cntr, u4RegValue2);
	AudioWayDisable();
	/*
	AudDrv_Clk_On();
	Afe_Set_Reg(AFE_IRQ_CLR, 1 << 6 , 0xff);
	Afe_Set_Reg(AFE_IRQ_CLR, 1 , 0xff);
	Afe_Set_Reg(AFE_IRQ_CLR, 1 << 1 , 0xff);
	Afe_Set_Reg(AFE_IRQ_CLR, 1 << 2 , 0xff);
	Afe_Set_Reg(AFE_IRQ_CLR, 1 << 3 , 0xff);
	Afe_Set_Reg(AFE_IRQ_CLR, 1 << 4 , 0xff);
	Afe_Set_Reg(AFE_IRQ_CLR, 1 << 5 , 0xff);

	AudDrv_Clk_Off();
	*/
	AudDrv_Clk_On_ClrISRStatus();
	pr_debug("IRQ Handle Exception");
	goto AudDrv_IRQ_handler_exit;
    }
    CheckInterruptTiming();
    if (u4RegValue & INTERRUPT_IRQ1_MCU)
    {
	Auddrv_DL_Interrupt_Handler();
    }
    if (u4RegValue & INTERRUPT_IRQ2_MCU)
    {
	Auddrv_UL_Interrupt_Handler();
    }
    if (u4RegValue & INTERRUPT_IRQ_MCU_DAI_SET)
    {

    }
    if (u4RegValue & INTERRUPT_IRQ_MCU_DAI_RST)
    {

    }

    /* clear irq */
    Afe_Set_Reg(AFE_IRQ_CLR, u4RegValue , 0xff);

AudDrv_IRQ_handler_exit:
    spin_unlock_irqrestore(&auddrv_clk_isr_lock, flags);
    return IRQ_HANDLED;
}

/*****************************************************************************
 * PLATFORM DRIVER FUNCTION:
 *
 *  AudDrv_probe / AudDrv_suspend / AudDrv_resume / AudDrv_shutdown / AudDrv_remove
 *
 * DESCRIPTION
 *  Linus Platform Driver
 *
 *****************************************************************************
 */

static int AudDrv_probe(struct platform_device *dev)
{
    int ret = 0;
    pr_debug("+AudDrv_probe\n");

    pr_debug("+request_irq\n");
   ret = request_irq(AUD_AFE_MCU_IRQ_LINE, AudDrv_IRQ_handler, IRQF_TRIGGER_LOW/*IRQF_TRIGGER_FALLING*/, "Afe_ISR_Handle", dev);
    if (ret < 0)
    {
        pr_err("AudDrv_probe request_irq Fail\n");
    }

    /* init */
    memset((void *)&AFE_dL1_Control_context, 0, sizeof(AFE_MEM_CONTROL_T));
    memset((void *)&AFE_dL2_Control_context, 0, sizeof(AFE_MEM_CONTROL_T));
    memset((void *)&AWB_Control_context, 0, sizeof(AFE_MEM_CONTROL_T));
    memset((void *)&VUL_Control_context, 0, sizeof(AFE_MEM_CONTROL_T));
    memset((void *)&MODDAI_Control_context, 0, sizeof(AFE_MEM_CONTROL_T));
    memset((void *)&Suspend_reg, 0, sizeof(AudAfe_Suspend_Reg));
    memset((void *)&SPH_Ctrl_State, 0, sizeof(SPH_Control));
    AFE_dL1_Control_context.MemIfNum = MEM_DL1;
    AFE_dL2_Control_context.MemIfNum = MEM_DL2;
    AWB_Control_context.MemIfNum = MEM_AWB;
    VUL_Control_context.MemIfNum = MEM_VUL;
    MODDAI_Control_context.MemIfNum = MEM_MOD_DAI;
    memset((void *)&SPH_Ctrl_State, 0, sizeof(SPH_Control));

#ifdef AUDIO_MEMORY_SRAM
    AFE_SRAM_ADDRESS = ioremap_nocache(AFE_INTERNAL_SRAM_PHY_BASE, 0x10000);
    pr_debug("AFE_BASE_ADDRESS = %p AFE_SRAM_ADDRESS = %p\n", AFE_BASE_ADDRESS, AFE_SRAM_ADDRESS);
#endif

#ifdef AUDIO_MEM_IOREMAP
    AFE_BASE_ADDRESS = ioremap_nocache(AUDIO_HW_PHYSICAL_BASE, 0x10000);
#endif

    pr_debug("-AudDrv_probe\n");
    Speaker_Init();
    if (Auddrv_First_bootup == true)
    {
        power_init();
    }
    else
    {

    }
    return 0;
}

static void AudDrv_Store_reg_AFE(void)
{
   AudDrv_Clk_On();

   /* Suspend_reg.Suspend_AUDIO_TOP_CON0                =   Afe_Get_Reg( AUDIO_AFE_TOP_CON0      ); */
   Suspend_reg.Suspend_AUDIO_TOP_CON1                =   Afe_Get_Reg(AUDIO_AFE_TOP_CON1);
   Suspend_reg.Suspend_AUDIO_TOP_CON2                =   Afe_Get_Reg(AUDIO_AFE_TOP_CON2);
   Suspend_reg.Suspend_AUDIO_TOP_CON3                =   Afe_Get_Reg(AUDIO_AFE_TOP_CON3);
   Suspend_reg.Suspend_AFE_DAC_CON0                  =   Afe_Get_Reg(AFE_DAC_CON0);
   Suspend_reg.Suspend_AFE_DAC_CON1                  =   Afe_Get_Reg(AFE_DAC_CON1);
   Suspend_reg.Suspend_AFE_I2S_CON                   =   Afe_Get_Reg(AFE_I2S_CON);

   Suspend_reg.Suspend_AFE_CONN0                     =   Afe_Get_Reg(AFE_CONN0);
   Suspend_reg.Suspend_AFE_CONN1                     =   Afe_Get_Reg(AFE_CONN1);
   Suspend_reg.Suspend_AFE_CONN2                     =   Afe_Get_Reg(AFE_CONN2);
   Suspend_reg.Suspend_AFE_CONN3                     =   Afe_Get_Reg(AFE_CONN3);
   Suspend_reg.Suspend_AFE_CONN4                     =   Afe_Get_Reg(AFE_CONN4);

   Suspend_reg.Suspend_AFE_I2S_CON1                  =   Afe_Get_Reg(AFE_I2S_CON1);
   Suspend_reg.Suspend_AFE_I2S_CON2                  =   Afe_Get_Reg(AFE_I2S_CON2);
   Suspend_reg.Suspend_AFE_I2S_CON3                  =   Afe_Get_Reg(AFE_I2S_CON3);

   Suspend_reg.Suspend_AFE_DL1_BASE                  =   Afe_Get_Reg(AFE_DL1_BASE);
   Suspend_reg.Suspend_AFE_DL1_CUR                   =   Afe_Get_Reg(AFE_DL1_CUR);
   Suspend_reg.Suspend_AFE_DL1_END                   =   Afe_Get_Reg(AFE_DL1_END);
   Suspend_reg.Suspend_AFE_DL2_BASE                  =   Afe_Get_Reg(AFE_DL2_BASE);
   Suspend_reg.Suspend_AFE_DL2_CUR                   =   Afe_Get_Reg(AFE_DL2_CUR);
   Suspend_reg.Suspend_AFE_DL2_END                   =   Afe_Get_Reg(AFE_DL2_END);
   Suspend_reg.Suspend_AFE_AWB_BASE                  =   Afe_Get_Reg(AFE_AWB_BASE);
   Suspend_reg.Suspend_AFE_AWB_END                   =   Afe_Get_Reg(AFE_AWB_END);
   Suspend_reg.Suspend_AFE_AWB_CUR                   =   Afe_Get_Reg(AFE_AWB_CUR);
   Suspend_reg.Suspend_AFE_VUL_BASE                  =   Afe_Get_Reg(AFE_VUL_BASE);
   Suspend_reg.Suspend_AFE_VUL_END                   =   Afe_Get_Reg(AFE_VUL_END);
   Suspend_reg.Suspend_AFE_VUL_CUR                   =   Afe_Get_Reg(AFE_VUL_CUR);

   Suspend_reg.Suspend_AFE_MEMIF_MON0                =   Afe_Get_Reg(AFE_MEMIF_MON0);
   Suspend_reg.Suspend_AFE_MEMIF_MON1                =   Afe_Get_Reg(AFE_MEMIF_MON1);
   Suspend_reg.Suspend_AFE_MEMIF_MON2                =   Afe_Get_Reg(AFE_MEMIF_MON2);
   Suspend_reg.Suspend_AFE_MEMIF_MON4                =   Afe_Get_Reg(AFE_MEMIF_MON4);

   Suspend_reg.Suspend_AFE_ADDA_DL_SRC2_CON0         =   Afe_Get_Reg(AFE_ADDA_DL_SRC2_CON0);
   Suspend_reg.Suspend_AFE_ADDA_DL_SRC2_CON1         =   Afe_Get_Reg(AFE_ADDA_DL_SRC2_CON1);
   Suspend_reg.Suspend_AFE_ADDA_UL_SRC_CON0          =   Afe_Get_Reg(AFE_ADDA_UL_SRC_CON0);
   Suspend_reg.Suspend_AFE_ADDA_UL_SRC_CON1          =   Afe_Get_Reg(AFE_ADDA_UL_SRC_CON1);
   Suspend_reg.Suspend_AFE_ADDA_TOP_CON0             =   Afe_Get_Reg(AFE_ADDA_TOP_CON0);
   Suspend_reg.Suspend_AFE_ADDA_UL_DL_CON0           =   Afe_Get_Reg(AFE_ADDA_UL_DL_CON0);
   Suspend_reg.Suspend_AFE_ADDA_SRC_DEBUG            =   Afe_Get_Reg(AFE_ADDA_SRC_DEBUG);
   Suspend_reg.Suspend_AFE_ADDA_SRC_DEBUG_MON0       =   Afe_Get_Reg(AFE_ADDA_SRC_DEBUG_MON0);
   Suspend_reg.Suspend_AFE_ADDA_SRC_DEBUG_MON1       =   Afe_Get_Reg(AFE_ADDA_SRC_DEBUG_MON1);
   Suspend_reg.Suspend_AFE_ADDA_NEWIF_CFG0           =   Afe_Get_Reg(AFE_ADDA_NEWIF_CFG0);
   Suspend_reg.Suspend_AFE_ADDA_NEWIF_CFG1           =   Afe_Get_Reg(AFE_ADDA_NEWIF_CFG1);

   Suspend_reg.Suspend_AFE_SIDETONE_DEBUG            =   Afe_Get_Reg(AFE_SIDETONE_DEBUG);
   Suspend_reg.Suspend_AFE_SIDETONE_MON              =   Afe_Get_Reg(AFE_SIDETONE_MON);
   Suspend_reg.Suspend_AFE_SIDETONE_CON0             =   Afe_Get_Reg(AFE_SIDETONE_CON0);
   Suspend_reg.Suspend_AFE_SIDETONE_COEFF            =   Afe_Get_Reg(AFE_SIDETONE_COEFF);
   Suspend_reg.Suspend_AFE_SIDETONE_CON1             =   Afe_Get_Reg(AFE_SIDETONE_CON1);
   Suspend_reg.Suspend_AFE_SIDETONE_GAIN             =   Afe_Get_Reg(AFE_SIDETONE_GAIN);
   Suspend_reg.Suspend_AFE_SGEN_CON0                 =   Afe_Get_Reg(AFE_SGEN_CON0);

   Suspend_reg.Suspend_AFE_TOP_CON0                  =   Afe_Get_Reg(AFE_TOP_CON0);

   Suspend_reg.Suspend_AFE_ADDA_PREDIS_CON0          =   Afe_Get_Reg(AFE_ADDA_PREDIS_CON0);
   Suspend_reg.Suspend_AFE_ADDA_PREDIS_CON1          =   Afe_Get_Reg(AFE_ADDA_PREDIS_CON1);
   Suspend_reg.Suspend_AFE_MOD_PCM_BASE              =   Afe_Get_Reg(AFE_MOD_PCM_BASE);
   Suspend_reg.Suspend_AFE_MOD_PCM_END               =   Afe_Get_Reg(AFE_MOD_PCM_END);
   Suspend_reg.Suspend_AFE_MOD_PCM_CUR               =   Afe_Get_Reg(AFE_MOD_PCM_CUR);

   Suspend_reg.Suspend_AFE_IRQ_CON                   =   Afe_Get_Reg(AFE_IRQ_CON);
   Suspend_reg.Suspend_AFE_IRQ_STATUS                =   Afe_Get_Reg(AFE_IRQ_STATUS);
   Suspend_reg.Suspend_AFE_IRQ_CLR                   =   Afe_Get_Reg(AFE_IRQ_CLR);
   Suspend_reg.Suspend_AFE_IRQ_CNT1                  =   Afe_Get_Reg(AFE_IRQ_CNT1);
   Suspend_reg.Suspend_AFE_IRQ_CNT2                  =   Afe_Get_Reg(AFE_IRQ_CNT2);
   Suspend_reg.Suspend_AFE_IRQ_MON2                  =   Afe_Get_Reg(AFE_IRQ_MON2);
   Suspend_reg.Suspend_AFE_IRQ1_CNT_MON              =   Afe_Get_Reg(AFE_IRQ1_CNT_MON);
   Suspend_reg.Suspend_AFE_IRQ2_CNT_MON              =   Afe_Get_Reg(AFE_IRQ2_CNT_MON);
   Suspend_reg.Suspend_AFE_IRQ1_EN_CNT_MON           =   Afe_Get_Reg(AFE_IRQ1_EN_CNT_MON);
   Suspend_reg.Suspend_AFE_MEMIF_MAXLEN              =   Afe_Get_Reg(AFE_MEMIF_MAXLEN);
   Suspend_reg.Suspend_AFE_MEMIF_PBUF_SIZE           =   Afe_Get_Reg(AFE_MEMIF_PBUF_SIZE);

   Suspend_reg.Suspend_AFE_GAIN1_CON0                =   Afe_Get_Reg(AFE_GAIN1_CON0);
   Suspend_reg.Suspend_AFE_GAIN1_CON1                =   Afe_Get_Reg(AFE_GAIN1_CON1);
   Suspend_reg.Suspend_AFE_GAIN1_CON2                =   Afe_Get_Reg(AFE_GAIN1_CON2);
   Suspend_reg.Suspend_AFE_GAIN1_CON3                =   Afe_Get_Reg(AFE_GAIN1_CON3);
   Suspend_reg.Suspend_AFE_GAIN1_CONN                =   Afe_Get_Reg(AFE_GAIN1_CONN);
   Suspend_reg.Suspend_AFE_GAIN1_CUR                 =   Afe_Get_Reg(AFE_GAIN1_CUR);
   Suspend_reg.Suspend_AFE_GAIN2_CON0                =   Afe_Get_Reg(AFE_GAIN2_CON0);
   Suspend_reg.Suspend_AFE_GAIN2_CON1                =   Afe_Get_Reg(AFE_GAIN2_CON1);
   Suspend_reg.Suspend_AFE_GAIN2_CON2                =   Afe_Get_Reg(AFE_GAIN2_CON2);
   Suspend_reg.Suspend_AFE_GAIN2_CON3                =   Afe_Get_Reg(AFE_GAIN2_CON3);
   Suspend_reg.Suspend_AFE_GAIN2_CONN                =   Afe_Get_Reg(AFE_GAIN2_CONN);
   Suspend_reg.Suspend_AFE_GAIN2_CONN2               =   Afe_Get_Reg(AFE_GAIN2_CONN2);
   Suspend_reg.Suspend_AFE_GAIN2_CUR                 =   Afe_Get_Reg(AFE_GAIN2_CUR);

   Suspend_reg.Suspend_AFE_ASRC_CON0                 =   Afe_Get_Reg(AFE_ASRC_CON0);
   Suspend_reg.Suspend_AFE_ASRC_CON1                 =   Afe_Get_Reg(AFE_ASRC_CON1);
   Suspend_reg.Suspend_AFE_ASRC_CON2                 =   Afe_Get_Reg(AFE_ASRC_CON2);
   Suspend_reg.Suspend_AFE_ASRC_CON3                 =   Afe_Get_Reg(AFE_ASRC_CON3);
   Suspend_reg.Suspend_AFE_ASRC_CON4                 =   Afe_Get_Reg(AFE_ASRC_CON4);
   Suspend_reg.Suspend_AFE_ASRC_CON5                 =   Afe_Get_Reg(AFE_ASRC_CON5);
   Suspend_reg.Suspend_AFE_ASRC_CON6                 =   Afe_Get_Reg(AFE_ASRC_CON6);
   Suspend_reg.Suspend_AFE_ASRC_CON7                 =   Afe_Get_Reg(AFE_ASRC_CON7);
   Suspend_reg.Suspend_AFE_ASRC_CON8                 =   Afe_Get_Reg(AFE_ASRC_CON8);
   Suspend_reg.Suspend_AFE_ASRC_CON9                 =   Afe_Get_Reg(AFE_ASRC_CON9);
   Suspend_reg.Suspend_AFE_ASRC_CON10                =   Afe_Get_Reg(AFE_ASRC_CON10);
   Suspend_reg.Suspend_AFE_ASRC_CON11                =   Afe_Get_Reg(AFE_ASRC_CON11);

   Suspend_reg.Suspend_PCM2_INTF_CON                 =   Afe_Get_Reg(PCM2_INTF_CON);
   Suspend_reg.Suspend_AFE_ASRC_CON13                =   Afe_Get_Reg(AFE_ASRC_CON13);
   Suspend_reg.Suspend_AFE_ASRC_CON14                =   Afe_Get_Reg(AFE_ASRC_CON14);
   Suspend_reg.Suspend_AFE_ASRC_CON15                =   Afe_Get_Reg(AFE_ASRC_CON15);
   Suspend_reg.Suspend_AFE_ASRC_CON16                =   Afe_Get_Reg(AFE_ASRC_CON16);
   Suspend_reg.Suspend_AFE_ASRC_CON17                =   Afe_Get_Reg(AFE_ASRC_CON17);
   Suspend_reg.Suspend_AFE_ASRC_CON18                =   Afe_Get_Reg(AFE_ASRC_CON18);
   Suspend_reg.Suspend_AFE_ASRC_CON19                =   Afe_Get_Reg(AFE_ASRC_CON19);
   Suspend_reg.Suspend_AFE_ASRC_CON20                =   Afe_Get_Reg(AFE_ASRC_CON20);
   Suspend_reg.Suspend_AFE_ASRC_CON21                =   Afe_Get_Reg(AFE_ASRC_CON21);

   AudDrv_Clk_Off();
   pr_debug("-AudDrv_Store_reg\n");
}

static void AudDrv_Recover_reg_AFE(void)
{
    pr_debug(KERN_EMERG "+AudDrv_resume");
    AudDrv_Clk_On();
    /* Digital register setting */
    /* Afe_Set_Reg( AUDIO_AFE_TOP_CON1      ,Suspend_reg.Suspend_AUDIO_TOP_CON1      ,MASK_ALL); */
    Afe_Set_Reg(AUDIO_AFE_TOP_CON2      , Suspend_reg.Suspend_AUDIO_TOP_CON2      , MASK_ALL);
    Afe_Set_Reg(AUDIO_AFE_TOP_CON3      , Suspend_reg.Suspend_AUDIO_TOP_CON3      , MASK_ALL);
    Afe_Set_Reg(AFE_DAC_CON0            , Suspend_reg.Suspend_AFE_DAC_CON0            , MASK_ALL);
    Afe_Set_Reg(AFE_DAC_CON1            , Suspend_reg.Suspend_AFE_DAC_CON1            , MASK_ALL);
    Afe_Set_Reg(AFE_I2S_CON             , Suspend_reg.Suspend_AFE_I2S_CON             , MASK_ALL);

    Afe_Set_Reg(AFE_CONN0               , Suspend_reg.Suspend_AFE_CONN0               , MASK_ALL);
    Afe_Set_Reg(AFE_CONN1               , Suspend_reg.Suspend_AFE_CONN1               , MASK_ALL);
    Afe_Set_Reg(AFE_CONN2               , Suspend_reg.Suspend_AFE_CONN2               , MASK_ALL);
    Afe_Set_Reg(AFE_CONN3               , Suspend_reg.Suspend_AFE_CONN3               , MASK_ALL);
    Afe_Set_Reg(AFE_CONN4               , Suspend_reg.Suspend_AFE_CONN4               , MASK_ALL);

    Afe_Set_Reg(AFE_I2S_CON1            , Suspend_reg.Suspend_AFE_I2S_CON1            , MASK_ALL);
    Afe_Set_Reg(AFE_I2S_CON2            , Suspend_reg.Suspend_AFE_I2S_CON2            , MASK_ALL);
    Afe_Set_Reg(AFE_I2S_CON3            , Suspend_reg.Suspend_AFE_I2S_CON3            , MASK_ALL);

    Afe_Set_Reg(AFE_DL1_BASE            , Suspend_reg.Suspend_AFE_DL1_BASE            , MASK_ALL);
    /* Afe_Set_Reg( AFE_DL1_CUR             ,Suspend_reg.Suspend_AFE_DL1_CUR             ,MASK_ALL); */
    Afe_Set_Reg(AFE_DL1_END             , Suspend_reg.Suspend_AFE_DL1_END             , MASK_ALL);
    Afe_Set_Reg(AFE_DL2_BASE            , Suspend_reg.Suspend_AFE_DL2_BASE            , MASK_ALL);
    /* Afe_Set_Reg( AFE_DL2_CUR             ,Suspend_reg.Suspend_AFE_DL2_CUR             ,MASK_ALL); */
    Afe_Set_Reg(AFE_DL2_END             , Suspend_reg.Suspend_AFE_DL2_END             , MASK_ALL);
    Afe_Set_Reg(AFE_AWB_BASE            , Suspend_reg.Suspend_AFE_AWB_BASE            , MASK_ALL);
    Afe_Set_Reg(AFE_AWB_END             , Suspend_reg.Suspend_AFE_AWB_END             , MASK_ALL);
    Afe_Set_Reg(AFE_AWB_CUR             , Suspend_reg.Suspend_AFE_AWB_CUR             , MASK_ALL);
    Afe_Set_Reg(AFE_VUL_BASE            , Suspend_reg.Suspend_AFE_VUL_BASE            , MASK_ALL);
    Afe_Set_Reg(AFE_VUL_END             , Suspend_reg.Suspend_AFE_VUL_END             , MASK_ALL);
    Afe_Set_Reg(AFE_VUL_CUR             , Suspend_reg.Suspend_AFE_VUL_CUR             , MASK_ALL);

    Afe_Set_Reg(AFE_MEMIF_MON0          , Suspend_reg.Suspend_AFE_MEMIF_MON0          , MASK_ALL);
    Afe_Set_Reg(AFE_MEMIF_MON1          , Suspend_reg.Suspend_AFE_MEMIF_MON1          , MASK_ALL);
    Afe_Set_Reg(AFE_MEMIF_MON2          , Suspend_reg.Suspend_AFE_MEMIF_MON2          , MASK_ALL);
    Afe_Set_Reg(AFE_MEMIF_MON4          , Suspend_reg.Suspend_AFE_MEMIF_MON4          , MASK_ALL);

    Afe_Set_Reg(AFE_ADDA_DL_SRC2_CON0   , Suspend_reg.Suspend_AFE_ADDA_DL_SRC2_CON0   , MASK_ALL);
    Afe_Set_Reg(AFE_ADDA_DL_SRC2_CON1   , Suspend_reg.Suspend_AFE_ADDA_DL_SRC2_CON1   , MASK_ALL);
    Afe_Set_Reg(AFE_ADDA_UL_SRC_CON0    , Suspend_reg.Suspend_AFE_ADDA_UL_SRC_CON0    , MASK_ALL);
    Afe_Set_Reg(AFE_ADDA_UL_SRC_CON1    , Suspend_reg.Suspend_AFE_ADDA_UL_SRC_CON1    , MASK_ALL);
    Afe_Set_Reg(AFE_ADDA_TOP_CON0       , Suspend_reg.Suspend_AFE_ADDA_TOP_CON0       , MASK_ALL);
    Afe_Set_Reg(AFE_ADDA_UL_DL_CON0     , Suspend_reg.Suspend_AFE_ADDA_UL_DL_CON0     , MASK_ALL);
    Afe_Set_Reg(AFE_ADDA_SRC_DEBUG      , Suspend_reg.Suspend_AFE_ADDA_SRC_DEBUG      , MASK_ALL);
    Afe_Set_Reg(AFE_ADDA_SRC_DEBUG_MON0 , Suspend_reg.Suspend_AFE_ADDA_SRC_DEBUG_MON0 , MASK_ALL);
    Afe_Set_Reg(AFE_ADDA_SRC_DEBUG_MON1 , Suspend_reg.Suspend_AFE_ADDA_SRC_DEBUG_MON1 , MASK_ALL);
    Afe_Set_Reg(AFE_ADDA_NEWIF_CFG0     , Suspend_reg.Suspend_AFE_ADDA_NEWIF_CFG0     , MASK_ALL);
    Afe_Set_Reg(AFE_ADDA_NEWIF_CFG1     , Suspend_reg.Suspend_AFE_ADDA_NEWIF_CFG1     , MASK_ALL);

    Afe_Set_Reg(AFE_SIDETONE_DEBUG      , Suspend_reg.Suspend_AFE_SIDETONE_DEBUG      , MASK_ALL);
    Afe_Set_Reg(AFE_SIDETONE_MON        , Suspend_reg.Suspend_AFE_SIDETONE_MON        , MASK_ALL);
    Afe_Set_Reg(AFE_SIDETONE_CON0       , Suspend_reg.Suspend_AFE_SIDETONE_CON0       , MASK_ALL);
    Afe_Set_Reg(AFE_SIDETONE_COEFF      , Suspend_reg.Suspend_AFE_SIDETONE_COEFF      , MASK_ALL);
    Afe_Set_Reg(AFE_SIDETONE_CON1       , Suspend_reg.Suspend_AFE_SIDETONE_CON1       , MASK_ALL);
    Afe_Set_Reg(AFE_SIDETONE_GAIN       , Suspend_reg.Suspend_AFE_SIDETONE_GAIN       , MASK_ALL);
    Afe_Set_Reg(AFE_SGEN_CON0           , Suspend_reg.Suspend_AFE_SGEN_CON0           , MASK_ALL);

    Afe_Set_Reg(AFE_TOP_CON0            , Suspend_reg.Suspend_AFE_TOP_CON0            , MASK_ALL);

    Afe_Set_Reg(AFE_ADDA_PREDIS_CON0    , Suspend_reg.Suspend_AFE_ADDA_PREDIS_CON0    , MASK_ALL);
    Afe_Set_Reg(AFE_ADDA_PREDIS_CON1    , Suspend_reg.Suspend_AFE_ADDA_PREDIS_CON1    , MASK_ALL);
    Afe_Set_Reg(AFE_MOD_PCM_BASE        , Suspend_reg.Suspend_AFE_MOD_PCM_BASE        , MASK_ALL);
    Afe_Set_Reg(AFE_MOD_PCM_END         , Suspend_reg.Suspend_AFE_MOD_PCM_END         , MASK_ALL);
    Afe_Set_Reg(AFE_MOD_PCM_CUR         , Suspend_reg.Suspend_AFE_MOD_PCM_CUR         , MASK_ALL);

    Afe_Set_Reg(AFE_IRQ_CON             , Suspend_reg.Suspend_AFE_IRQ_CON             , MASK_ALL);
    Afe_Set_Reg(AFE_IRQ_STATUS          , Suspend_reg.Suspend_AFE_IRQ_STATUS          , MASK_ALL);
    Afe_Set_Reg(AFE_IRQ_CLR             , Suspend_reg.Suspend_AFE_IRQ_CLR             , MASK_ALL);
    Afe_Set_Reg(AFE_IRQ_CNT1            , Suspend_reg.Suspend_AFE_IRQ_CNT1            , MASK_ALL);
    Afe_Set_Reg(AFE_IRQ_CNT2            , Suspend_reg.Suspend_AFE_IRQ_CNT2            , MASK_ALL);
    Afe_Set_Reg(AFE_IRQ_MON2            , Suspend_reg.Suspend_AFE_IRQ_MON2            , MASK_ALL);
    Afe_Set_Reg(AFE_IRQ1_CNT_MON        , Suspend_reg.Suspend_AFE_IRQ1_CNT_MON        , MASK_ALL);
    Afe_Set_Reg(AFE_IRQ2_CNT_MON        , Suspend_reg.Suspend_AFE_IRQ2_CNT_MON        , MASK_ALL);
    Afe_Set_Reg(AFE_IRQ1_EN_CNT_MON     , Suspend_reg.Suspend_AFE_IRQ1_EN_CNT_MON     , MASK_ALL);
    Afe_Set_Reg(AFE_MEMIF_MAXLEN        , Suspend_reg.Suspend_AFE_MEMIF_MAXLEN        , MASK_ALL);
    Afe_Set_Reg(AFE_MEMIF_PBUF_SIZE     , Suspend_reg.Suspend_AFE_MEMIF_PBUF_SIZE     , MASK_ALL);

    Afe_Set_Reg(AFE_GAIN1_CON0          , Suspend_reg.Suspend_AFE_GAIN1_CON0          , MASK_ALL);
    Afe_Set_Reg(AFE_GAIN1_CON1          , Suspend_reg.Suspend_AFE_GAIN1_CON1          , MASK_ALL);
    Afe_Set_Reg(AFE_GAIN1_CON2          , Suspend_reg.Suspend_AFE_GAIN1_CON2          , MASK_ALL);
    Afe_Set_Reg(AFE_GAIN1_CON3          , Suspend_reg.Suspend_AFE_GAIN1_CON3          , MASK_ALL);
    Afe_Set_Reg(AFE_GAIN1_CONN          , Suspend_reg.Suspend_AFE_GAIN1_CONN          , MASK_ALL);
    Afe_Set_Reg(AFE_GAIN1_CUR           , Suspend_reg.Suspend_AFE_GAIN1_CUR           , MASK_ALL);
    Afe_Set_Reg(AFE_GAIN2_CON0          , Suspend_reg.Suspend_AFE_GAIN2_CON0          , MASK_ALL);
    Afe_Set_Reg(AFE_GAIN2_CON1          , Suspend_reg.Suspend_AFE_GAIN2_CON1          , MASK_ALL);
    Afe_Set_Reg(AFE_GAIN2_CON2          , Suspend_reg.Suspend_AFE_GAIN2_CON2          , MASK_ALL);
    Afe_Set_Reg(AFE_GAIN2_CON3          , Suspend_reg.Suspend_AFE_GAIN2_CON3          , MASK_ALL);
    Afe_Set_Reg(AFE_GAIN2_CONN          , Suspend_reg.Suspend_AFE_GAIN2_CONN          , MASK_ALL);
    Afe_Set_Reg(AFE_GAIN2_CONN2         , Suspend_reg.Suspend_AFE_GAIN2_CONN2         , MASK_ALL);
    Afe_Set_Reg(AFE_GAIN2_CUR           , Suspend_reg.Suspend_AFE_GAIN2_CUR           , MASK_ALL);

    Afe_Set_Reg(AFE_ASRC_CON0           , Suspend_reg.Suspend_AFE_ASRC_CON0           , MASK_ALL);
    Afe_Set_Reg(AFE_ASRC_CON1           , Suspend_reg.Suspend_AFE_ASRC_CON1           , MASK_ALL);
    Afe_Set_Reg(AFE_ASRC_CON2           , Suspend_reg.Suspend_AFE_ASRC_CON2           , MASK_ALL);
    Afe_Set_Reg(AFE_ASRC_CON3           , Suspend_reg.Suspend_AFE_ASRC_CON3           , MASK_ALL);
    Afe_Set_Reg(AFE_ASRC_CON4           , Suspend_reg.Suspend_AFE_ASRC_CON4           , MASK_ALL);
    Afe_Set_Reg(AFE_ASRC_CON5           , Suspend_reg.Suspend_AFE_ASRC_CON5           , MASK_ALL);
    Afe_Set_Reg(AFE_ASRC_CON6           , Suspend_reg.Suspend_AFE_ASRC_CON6           , MASK_ALL);
    Afe_Set_Reg(AFE_ASRC_CON7           , Suspend_reg.Suspend_AFE_ASRC_CON7           , MASK_ALL);
    Afe_Set_Reg(AFE_ASRC_CON8           , Suspend_reg.Suspend_AFE_ASRC_CON8           , MASK_ALL);
    Afe_Set_Reg(AFE_ASRC_CON9           , Suspend_reg.Suspend_AFE_ASRC_CON9           , MASK_ALL);
    Afe_Set_Reg(AFE_ASRC_CON10          , Suspend_reg.Suspend_AFE_ASRC_CON10          , MASK_ALL);
    Afe_Set_Reg(AFE_ASRC_CON11          , Suspend_reg.Suspend_AFE_ASRC_CON11          , MASK_ALL);

    Afe_Set_Reg(PCM2_INTF_CON           , Suspend_reg.Suspend_PCM2_INTF_CON           , MASK_ALL);
    Afe_Set_Reg(AFE_ASRC_CON13          , Suspend_reg.Suspend_AFE_ASRC_CON13          , MASK_ALL);
    Afe_Set_Reg(AFE_ASRC_CON14          , Suspend_reg.Suspend_AFE_ASRC_CON14          , MASK_ALL);
    Afe_Set_Reg(AFE_ASRC_CON15          , Suspend_reg.Suspend_AFE_ASRC_CON15          , MASK_ALL);
    Afe_Set_Reg(AFE_ASRC_CON16          , Suspend_reg.Suspend_AFE_ASRC_CON16          , MASK_ALL);
    Afe_Set_Reg(AFE_ASRC_CON17          , Suspend_reg.Suspend_AFE_ASRC_CON17          , MASK_ALL);
    Afe_Set_Reg(AFE_ASRC_CON18          , Suspend_reg.Suspend_AFE_ASRC_CON18          , MASK_ALL);
    Afe_Set_Reg(AFE_ASRC_CON19          , Suspend_reg.Suspend_AFE_ASRC_CON19          , MASK_ALL);
    Afe_Set_Reg(AFE_ASRC_CON20          , Suspend_reg.Suspend_AFE_ASRC_CON20          , MASK_ALL);
    Afe_Set_Reg(AFE_ASRC_CON21          , Suspend_reg.Suspend_AFE_ASRC_CON21          , MASK_ALL);

    AudDrv_Clk_Off();
}


static int AudDrv_remove(struct platform_device *dev)
{
    pr_notice("+AudDrv_remove\n");
    AudDrv_Clk_Off();  /* disable afe clock */
    pr_notice("-AudDrv_remove\n");
    return 0;
}


static void AudDrv_shutdown(struct platform_device *dev)
{
    pr_debug("+AudDrv_shutdown\n");
    Speaker_DeInit();
    pr_notice("-AudDrv_shutdown\n");
}

static int AudDrv_suspend(struct platform_device *dev, pm_message_t state)
/* only one suspend mode */
{
    /* if now in phone call state, not suspend!! */
    bool b_modem1_speech_on;
    /* bool b_modem2_speech_on; */
    AudDrv_Clk_On();/* should enable clk for access reg */
    b_modem1_speech_on = (bool)(Afe_Get_Reg(PCM2_INTF_CON) & 0x1);
    /* b_modem2_speech_on = (bool)(Afe_Get_Reg(PCM_INTF_CON1) & 0x1); */
    AudDrv_Clk_Off();

    if (b_modem1_speech_on == true/* || b_modem2_speech_on == true*/)
    {
	pr_debug("AudDrv_suspend: b_modem1_speech_on(%d), return", b_modem1_speech_on);
	return 0;
    }

    pr_debug("AudDrv_suspend AudDrvSuspendStatus = %d bSpeechFlag = %d\n",AudDrvSuspendStatus,SPH_Ctrl_State.bSpeechFlag);

    if (AudDrvSuspendStatus == false)
    {
	AudDrv_Store_reg_AFE();
	AudDrv_Suspend_Clk_Off();  /* turn off asm afe clock */
	AudioWayEnable();
	AudDrvSuspendStatus = true;/* set suspend mode to true , do suspend... */
    }

    return 0;
}

void CheckPowerState(void)
{
    /*    uint32 Reg_clksq_en = Ana_Get_Reg (TOP_CKCON1);
	Reg_clksq_en = (Reg_clksq_en>>4)&0x1;
	if(Reg_clksq_en ==0)
	{
	    pr_debug("CheckPowerState Reg_clksq_en = 0x%x\n",Ana_Get_Reg (TOP_CKCON1));
	} */
}

static int AudDrv_resume(struct platform_device *dev) /* wake up */
{
   /* PRINTK_AUDDRV("+AudDrv_resume AudDrvSuspendStatus= %d\n",AudDrvSuspendStatus); */
    pr_debug(KERN_EMERG "AudDrv_resume");

    if (AudDrvSuspendStatus == true)
    {
	AFE_BLOCK_T *Afe_Block;
	AudioWayDisable();
	AudDrv_Suspend_Clk_On();
	AudDrv_Recover_reg_AFE();
	Afe_Block = &(AFE_dL1_Control_context.rBlock);
	AudDrv_Clk_On();
	pr_debug("+AudDrv resume memset zero for sysram");
	memset(Afe_Block->pucVirtBufAddr, 0, Afe_Block->u4BufferSize);
	pr_debug("+AudDrv resume memset zero for sysram");
	AudDrv_Clk_Off();
	AudDrvSuspendStatus = false;
	/* PRINTK_AUDDRV("In AudResume"); */
    }
    /* PRINTK_AUDDRV("-AudDrv_resume\n"); */
    return 0;
}

#ifdef CONFIG_PM

static int AudDrv_pm_ops_suspend(struct device *device)
{
    struct platform_device *pdev = to_platform_device(device);
    return AudDrv_suspend(pdev, PMSG_SUSPEND);
}

static int AudDrv_pm_ops_resume(struct device *device)
{
    struct platform_device *pdev = to_platform_device(device);
    return AudDrv_resume(pdev);
}
#else
#define AudDrv_pm_ops_suspend NULL
#define AudDrv_pm_ops_resume NULL

#endif


/*****************************************************************************
 * FILE OPERATION FUNCTION
 *  AudDrv_open / AudDrv_release
 *
 * DESCRIPTION
 *
 *
 *****************************************************************************
 */
static int AudDrv_open(struct inode *inode, struct file *fp)
{
    pr_notice("AudDrv_open do nothing inode:%p, file:%p\n", inode, fp);
    return 0;
}

static int AudDrv_release(struct inode *inode, struct file *fp)
{
    pr_debug("AudDrv_release inode:%p, file:%p\n", inode, fp);

    if (!(fp->f_mode & FMODE_WRITE || fp->f_mode & FMODE_READ))
    {
	return -ENODEV;
    }
    return 0;
}

void Auddrv_Free_Dma_Memory(AFE_MEM_CONTROL_T *pAFE_MEM)
{
    AFE_BLOCK_T *pblock = NULL;
    if (pAFE_MEM == NULL)
    {
	pr_debug("Auddrv_Free_Dma_Memory pAFE_MEM = NULL");
    }

    pblock =  &(pAFE_MEM->rBlock);
    if ((pblock->pucVirtBufAddr != NULL) && (pblock->pucPhysBufAddr != 0))
    {
	pr_debug("dma_free_coherent pucVirtBufAddr = %p pucPhysBufAddr = %x", pblock->pucVirtBufAddr, pblock->pucPhysBufAddr);
	dma_free_coherent(0, pblock->u4BufferSize, pblock->pucVirtBufAddr, pblock->pucPhysBufAddr);
    }
    else
    {
	pr_debug("cannot dma_free_coherent pucVirtBufAddr = %p pucPhysBufAddr = %x", pblock->pucVirtBufAddr, pblock->pucPhysBufAddr);
    }
}

/*****************************************************************************
 * FUNCTION
 *  AudDrv_Force_Free_DL1_Buffer / AudDrv_Free_DL1_Buffer
 *
 * DESCRIPTION
 *  allocate DL1 Buffer
 *
 ******************************************************************************/


int AudDrv_Force_Free_DL1_Buffer(void)
{
    AFE_BLOCK_T *pblock = &AFE_dL1_Control_context.rBlock;
    pr_debug("+AudDrv_Force_Free_DL1_Buffer\n");
    if (pblock->pucVirtBufAddr != NULL)
    {

	/* todo:: here need to free sram by sram manager */
#ifdef AUDIO_MEMORY_SRAM
#else
	if ((pblock->pucVirtBufAddr != NULL) && (pblock->pucPhysBufAddr != 0))
	{
	    pr_debug("AudDrv_Force_Free_DL1_Buffer pucVirtBufAddr = %p pucPhysBufAddr = %x", pblock->pucVirtBufAddr, pblock->pucPhysBufAddr);
	    dma_free_coherent(0, pblock->u4BufferSize, pblock->pucVirtBufAddr, pblock->pucPhysBufAddr);
	}
	else
	{
	    pr_debug("cannot AudDrv_Force_Free_DL1_Buffer pucVirtBufAddr = %p pucPhysBufAddr = %x", pblock->pucVirtBufAddr, pblock->pucPhysBufAddr);
	}
#endif
	/* and then clear all DL1 inforamtion */
	memset((void *)&AFE_dL1_Control_context, 0, sizeof(AFE_dL1_Control_context));
	AFE_dL1_Control_context.MemIfNum = MEM_DL1;
    }
    pr_debug("-AudDrv_Free_DL1_Buffer\n");
    return 0;
}

int AudDrv_Force_Free_Buffer(int mem_type)
{
    AFE_MEM_CONTROL_T *pAFE_MEM = NULL;
    AFE_BLOCK_T *pblock = NULL;
    pr_debug("+ AudDrv_Force_Free_Buffer mem_type = %d\n", mem_type);
    switch (mem_type)
    {
	case MEM_DL1:
	{
#if defined(DL1_MEM_FIXED_IN_SRAM)
	    pr_err("MEM_DL1 should use SRAM\n");
	    return -1;
#else
	    pAFE_MEM = &AFE_dL1_Control_context;
	    pblock =  &(pAFE_MEM->rBlock);
	    if ((Aud_Int_Mem_Flag & (1 << MEM_DL1)) == 0) /* Not use SRAM as memory */
	    {
		Auddrv_Free_Dma_Memory(pAFE_MEM);
	    }
	    memset((void *)&AFE_dL1_Control_context, 0, sizeof(AFE_MEM_CONTROL_T));
	    AFE_dL1_Control_context.MemIfNum = MEM_DL1;
	    break;
#endif

	}
	case MEM_DL2:
	{
	    pAFE_MEM = &AFE_dL2_Control_context;
	    pblock =  &(pAFE_MEM->rBlock);
#if !defined(DL1_MEM_FIXED_IN_SRAM)
	    if ((Aud_Int_Mem_Flag & (1 << MEM_DL2)) == 0) /* Not use SRAM as memory */
#endif
		Auddrv_Free_Dma_Memory(pAFE_MEM);
	    memset((void *)&AFE_dL2_Control_context, 0, sizeof(AFE_MEM_CONTROL_T));
	    AFE_dL2_Control_context.MemIfNum = MEM_DL2;
	    break;
	}
	case MEM_AWB:
	{
	    pAFE_MEM = &AWB_Control_context;
	    pblock =  &(pAFE_MEM->rBlock);
#if !defined(DL1_MEM_FIXED_IN_SRAM)
	    if ((Aud_Int_Mem_Flag & (1 << MEM_AWB)) == 0) /* Not use SRAM as memory */
#endif
		Auddrv_Free_Dma_Memory(pAFE_MEM);
	    memset((void *)&AWB_Control_context, 0, sizeof(AFE_MEM_CONTROL_T));
	    AWB_Control_context.MemIfNum = MEM_AWB;
	    break;
	}
	case MEM_VUL:
	{
	    pAFE_MEM = &VUL_Control_context;
	    pblock =  &(pAFE_MEM->rBlock);
#if !defined(DL1_MEM_FIXED_IN_SRAM)
	    if ((Aud_Int_Mem_Flag & (1 << MEM_VUL)) == 0) /* Not use SRAM as memory */
#endif
		Auddrv_Free_Dma_Memory(pAFE_MEM);
	    memset((void *)&VUL_Control_context, 0, sizeof(AFE_MEM_CONTROL_T));
	    VUL_Control_context.MemIfNum = MEM_VUL;
	    break;
	}
	case MEM_MOD_DAI:
	{
	    pAFE_MEM = &MODDAI_Control_context;
	    pblock =  &(pAFE_MEM->rBlock);
#if !defined(DL1_MEM_FIXED_IN_SRAM)
	    if ((Aud_Int_Mem_Flag & (1 << MEM_MOD_DAI)) == 0) /* Not use SRAM as memory */
#endif
		Auddrv_Free_Dma_Memory(pAFE_MEM);
	    memset((void *)&MODDAI_Control_context, 0, sizeof(AFE_MEM_CONTROL_T));
	    MODDAI_Control_context.MemIfNum = MEM_MOD_DAI;
	    break;
	}
	default:
	    pr_err("NO MEM_IF MATCH\n");
	    return -1;
    }
    pr_debug("-AudDrv_Force_Free_Buffer\n");
    return 0;
}



#if defined(DL1_MEM_FIXED_IN_SRAM)
/*****************************************************************************
 * FUNCTION
 *  AudDrv_Allocate_DL1_Buffer / AudDrv_Free_DL1_Buffer
 *
 * DESCRIPTION
 *  allocate DL1 Buffer
 *
 ******************************************************************************/
int AudDrv_Allocate_DL1_Buffer(struct file *fp, kal_uint32 Afe_Buf_Length)
{
#ifdef AUDIO_MEMORY_SRAM
    kal_uint32 u4PhyAddr = 0;
#endif
    AFE_BLOCK_T *pblock = &AFE_dL1_Control_context.rBlock;
    pblock->u4BufferSize = Afe_Buf_Length;

    pr_debug("AudDrv_Allocate_DL1_Buffer length fp = %p = %d\n", fp, Afe_Buf_Length);
    if (Afe_Buf_Length > AUDDRV_DL1_MAX_BUFFER_LENGTH)
    {
	return -1;
    }

    /* allocate memory */
    if (pblock->pucPhysBufAddr == 0)
    {
#ifdef AUDIO_MEMORY_SRAM
	/* todo , there should be a sram manager to allocate memory for low power.powervr_device */
	u4PhyAddr = AFE_INTERNAL_SRAM_PHY_BASE;
	pblock->pucPhysBufAddr = u4PhyAddr;

#ifdef AUDIO_MEM_IOREMAP
	pr_debug("AudDrv_Allocate_DL1_Buffer length AUDIO_MEM_IOREMAP = %p = %d\n", fp, Afe_Buf_Length);
	pblock->pucVirtBufAddr = (kal_uint8 *)AFE_SRAM_ADDRESS;
#else
	pblock->pucVirtBufAddr = AFE_INTERNAL_SRAM_VIR_BASE;
#endif
	Afe_Set_Reg(AFE_MEMIF_MAXLEN, 0, 0x01); /*  */
#else
	pr_debug("AudDrv_Allocate_DL1_Buffer use dram");
	pblock->pucVirtBufAddr = dma_alloc_coherent(0, pblock->u4BufferSize, &pblock->pucPhysBufAddr, GFP_KERNEL);
	Afe_Set_Reg(AFE_MEMIF_MAXLEN, 1, 0x01); /*  */
#endif
    }
    pr_debug("AudDrv_Allocate_DL1_Buffer pucVirtBufAddr = %p\n", pblock->pucVirtBufAddr);

    /* check 32 bytes align */
    if ((pblock->pucPhysBufAddr & 0x1f) != 0)
    {
	pr_debug("[Auddrv] AudDrv_Allocate_DL1_Buffer is not aligned (0x%x)\n", pblock->pucPhysBufAddr);
    }

    pblock->u4SampleNumMask = 0x001f;  /* 32 byte align */
    pblock->u4WriteIdx     = 0;
    pblock->u4DMAReadIdx    = 0;
    pblock->u4DataRemained  = 0;
    pblock->u4fsyncflag     = false;
    pblock->uResetFlag      = true;

    /* set sram address top hardware */
    Afe_Set_Reg(AFE_DL1_BASE , pblock->pucPhysBufAddr , 0xffffffff);
    Afe_Set_Reg(AFE_DL1_END  , pblock->pucPhysBufAddr + (Afe_Buf_Length - 1) , 0xffffffff);

    return 0;
}

int AudDrv_Free_DL1_Buffer(struct file *fp)
{
    AFE_BLOCK_T *pblock = &AFE_dL1_Control_context.rBlock;
    pr_debug("+AudDrv_Free_DL1_Buffer fp = %p", fp);
    if (pblock->pucVirtBufAddr != NULL)
    {
	/* todo:: here need to free sram by sram manager */

	/* and then clear all DL1 inforamtion */
	memset((void *)&AFE_dL1_Control_context, 0, sizeof(AFE_dL1_Control_context));
	AFE_dL1_Control_context.MemIfNum = MEM_DL1;
    }
    pr_debug("-AudDrv_Free_DL1_Buffer");
    return 0;
}
#endif

/*****************************************************************************
 * FUNCTION
 *  AudDrv_Allocate_Buffer / AudDrv_Free_Buffer
 *
 * DESCRIPTION
 *  allocate  Buffer with dram
 * ******************************************************************************/

int AudDrv_Allocate_Buffer(struct file *fp, kal_uint32 Afe_Buf_Length , int mem_type)
{
    AFE_MEM_CONTROL_T *pAFE_MEM = NULL;
    AFE_BLOCK_T *pblock = NULL;
    pr_debug("AudDrv_Allocate_Buffer fp = %p length = %d mem_type = %d\n", fp, Afe_Buf_Length, mem_type);
    switch (mem_type)
    {
	case MEM_DL1:
	{
#if defined(DL1_MEM_FIXED_IN_SRAM)
	    pr_err("MEM_DL1 should use SRAM\n");
	    return -1;
#else
	    pAFE_MEM = &AFE_dL1_Control_context;
	    break;
#endif

	}
	case MEM_DL2:
	{
	    pAFE_MEM = &AFE_dL2_Control_context;
	    break;
	}
	case MEM_AWB:
	{
	    pAFE_MEM = &AWB_Control_context;
	    break;
	}
	case MEM_VUL:
	{
	    pAFE_MEM = &VUL_Control_context;
	    break;
	}
	case MEM_MOD_DAI:
	{
	    pAFE_MEM = &MODDAI_Control_context;
	    break;
	}
	default:
	    pr_err("NO MEM_IF MATCH\n");
	    return -1;
    }

    /* allocate memory for current MEMIF */
    pblock =  &(pAFE_MEM->rBlock);
    pblock->u4BufferSize = Afe_Buf_Length;
    if ((pblock->pucVirtBufAddr == NULL) && (pblock->pucPhysBufAddr == 0))
    {
	pblock->pucVirtBufAddr = dma_alloc_coherent(0, pblock->u4BufferSize, &pblock->pucPhysBufAddr, GFP_KERNEL);
	if ((0 == pblock->pucPhysBufAddr) || (NULL == pblock->pucVirtBufAddr))
	{
	    pr_debug("AudDrv_Allocate_Buffer dma_alloc_coherent fail\n");
	    return -1;
	}
	/* fix me , is here need to check audio clock? */
	memset((void *)pblock->pucVirtBufAddr, 0, pblock->u4BufferSize);
	pblock->u4SampleNumMask = 0x001f;  /* 32 byte align */
	pblock->u4WriteIdx    = 0;
	pblock->u4DMAReadIdx    = 0;
	pblock->u4DataRemained  = 0;
	pblock->u4fsyncflag     = false;
	pblock->uResetFlag      = true;
#if !defined(DL1_MEM_FIXED_IN_SRAM)
	pblock->pucPhysBufAddrBackup = pblock->pucPhysBufAddr;
	pblock->pucVirtBufAddrBackup = pblock->pucVirtBufAddr;
#endif
	pr_debug("pblock->pucVirtBufAddr = %p pblock->pucPhysBufAddr = 0x%x\n mem_type = %d\n" ,
		      pblock->pucVirtBufAddr, pblock->pucPhysBufAddr, mem_type);
    }

    /* do set physical address to hardware */
    switch (mem_type)
    {
	case MEM_DL1:
	{
#if defined(DL1_MEM_FIXED_IN_SRAM)
	    pr_err("MEM_DL1 should use SRAM\n");
	    return -1;
#else
	    Afe_Set_Reg(AFE_DL1_BASE , pblock->pucPhysBufAddr , 0xffffffff);
	    Afe_Set_Reg(AFE_DL1_END  , pblock->pucPhysBufAddr + (Afe_Buf_Length - 1) , 0xffffffff);
	    break;
#endif

	}
	case MEM_DL2:
	{
	    Afe_Set_Reg(AFE_DL2_BASE , pblock->pucPhysBufAddr , 0xffffffff);
	    Afe_Set_Reg(AFE_DL2_END  , pblock->pucPhysBufAddr + (Afe_Buf_Length - 1) , 0xffffffff);
	    break;
	}
	case MEM_AWB:
	{
	    Afe_Set_Reg(AFE_AWB_BASE , pblock->pucPhysBufAddr , 0xffffffff);
	    Afe_Set_Reg(AFE_AWB_END  , pblock->pucPhysBufAddr + (Afe_Buf_Length - 1) , 0xffffffff);
	    break;
	}
	case MEM_VUL:
	{
	    Afe_Set_Reg(AFE_VUL_BASE , pblock->pucPhysBufAddr , 0xffffffff);
	    Afe_Set_Reg(AFE_VUL_END  , pblock->pucPhysBufAddr + (Afe_Buf_Length - 1) , 0xffffffff);
	    break;
	}
	case MEM_MOD_DAI:
	{
	    Afe_Set_Reg(AFE_MOD_PCM_BASE , pblock->pucPhysBufAddr , 0xffffffff);
	    Afe_Set_Reg(AFE_MOD_PCM_END  , pblock->pucPhysBufAddr + (Afe_Buf_Length - 1) , 0xffffffff);
	    break;
	}
	default:
	    pr_err("NO MEM_IF MATCH\n");
	    return -1;
    }
    return 0;
}

int AudDrv_Free_Buffer(struct file *fp, int mem_type)
{
    AFE_MEM_CONTROL_T *pAFE_MEM = NULL;
    AFE_BLOCK_T *pblock = NULL;
    pr_debug("+ AudDrv_Free_Buffer fp = %p mem_type = %d\n", fp, mem_type);
    switch (mem_type)
    {
	case MEM_DL1:
	{
#if defined(DL1_MEM_FIXED_IN_SRAM)
	    pr_err("MEM_DL1 should use SRAM\n");
	    return -1;
#else
	    pAFE_MEM = &AFE_dL1_Control_context;
	    pblock =  &(pAFE_MEM->rBlock);
	    if ((Aud_Int_Mem_Flag & (1 << MEM_DL1)) == 0) /* Not use SRAM as memory */
	    {
	    Auddrv_Free_Dma_Memory(pAFE_MEM);
	    }
	    memset((void *)&AFE_dL1_Control_context, 0, sizeof(AFE_MEM_CONTROL_T));
	    AFE_dL1_Control_context.MemIfNum = MEM_DL1;
	    break;
#endif

	}
	case MEM_DL2:
	{
	    pAFE_MEM = &AFE_dL2_Control_context;
	    pblock =  &(pAFE_MEM->rBlock);
#if !defined(DL1_MEM_FIXED_IN_SRAM)
	    if ((Aud_Int_Mem_Flag & (1 << MEM_DL2)) == 0) /* Not use SRAM as memory */
#endif
	    Auddrv_Free_Dma_Memory(pAFE_MEM);
	    memset((void *)&AFE_dL2_Control_context, 0, sizeof(AFE_MEM_CONTROL_T));
	    AFE_dL2_Control_context.MemIfNum = MEM_DL2;
	    break;
	}
	case MEM_AWB:
	{
	    pAFE_MEM = &AWB_Control_context;
	    pblock =  &(pAFE_MEM->rBlock);
#if !defined(DL1_MEM_FIXED_IN_SRAM)
	    if ((Aud_Int_Mem_Flag & (1 << MEM_AWB)) == 0) /* Not use SRAM as memory */
#endif
	    Auddrv_Free_Dma_Memory(pAFE_MEM);
	    memset((void *)&AWB_Control_context, 0, sizeof(AFE_MEM_CONTROL_T));
	    AWB_Control_context.MemIfNum = MEM_AWB;
	    break;
	}
	case MEM_VUL:
	{
	    pAFE_MEM = &VUL_Control_context;
	    pblock =  &(pAFE_MEM->rBlock);
#if !defined(DL1_MEM_FIXED_IN_SRAM)
	    if ((Aud_Int_Mem_Flag & (1 << MEM_VUL)) == 0) /* Not use SRAM as memory */
#endif
	    Auddrv_Free_Dma_Memory(pAFE_MEM);
	    memset((void *)&VUL_Control_context, 0, sizeof(AFE_MEM_CONTROL_T));
	    VUL_Control_context.MemIfNum = MEM_VUL;
	    break;
	}
	case MEM_MOD_DAI:
	{
	    pAFE_MEM = &MODDAI_Control_context;
	    pblock =  &(pAFE_MEM->rBlock);
#if !defined(DL1_MEM_FIXED_IN_SRAM)
	    if ((Aud_Int_Mem_Flag & (1 << MEM_MOD_DAI)) == 0) /* Not use SRAM as memory */
#endif
	    Auddrv_Free_Dma_Memory(pAFE_MEM);
	    memset((void *)&MODDAI_Control_context, 0, sizeof(AFE_MEM_CONTROL_T));
	    MODDAI_Control_context.MemIfNum = MEM_MOD_DAI;
	    break;
	}
	default:
	    pr_err("NO MEM_IF MATCH\n");
	    return -1;
    }
    pr_debug("-AudDrv_Free_Buffer\n");
    return 0;
}

void Auddrv_Add_MemIF_Counter(int MEM_Type)
{
    switch (MEM_Type)
    {
	case MEM_DL1:
	    Afe_Mem_Pwr_on++;
	    if (Afe_Mem_Pwr_on == 1)
	    {
		AFE_BLOCK_T *Afe_Block = &(AFE_dL1_Control_context.rBlock);
		pr_debug("+ AudDrv Add_MemIF_Counter memset 0");
		memset(Afe_Block->pucVirtBufAddr, 0, Afe_Block->u4BufferSize);
		pr_debug("- AudDrv Add_MemIF_Counter memset 0");
	    }
	    break;
	case MEM_DL2:
            {
            AFE_BLOCK_T *Afe_Block = &(AFE_dL2_Control_context.rBlock);
            pr_debug("+ AudDrv Add_MemIF_Counter memset 0");
            memset(Afe_Block->pucVirtBufAddr, 0, Afe_Block->u4BufferSize);
            pr_debug("- AudDrv Add_MemIF_Counter memset 0");
            }
	    break;
	case MEM_AWB:
	    break;
	case MEM_VUL:
	    break;
	case MEM_MOD_DAI:
	    break;
	default:
	    pr_debug("Auddrv_Add_MemIF_Conter MEMTYPE = %d", MEM_Type);
	    return;
    }
}

void Auddrv_Release_MemIF_Counter(int MEM_Type)
{
    switch (MEM_Type)
    {
	case MEM_DL1:
	    Afe_Mem_Pwr_on--;
	    if (Afe_Mem_Pwr_on < 0)
	    {
		pr_debug("Auddrv_Release_MemIF_Conter Afe_Mem_Pwr_on <0\n");
		Afe_Mem_Pwr_on = 0;
	    }
	    ResetWriteWaitEvent();
	    break;
	case MEM_DL2:
        ResetWriteWaitEvent();
		pr_debug("DL2 Auddrv_Release_MemIF_Conter\n");
	    break;
	case MEM_AWB:
	    break;
	case MEM_VUL:
	    break;
	case MEM_MOD_DAI:
	    break;
	default:
	    pr_err("Auddrv_Release_MemIF_Conter MEMTYPE = %d", MEM_Type);
	    return;
    }
}

/*****************************************************************************
 * FILE OPERATION FUNCTION
 *  Auddrv_Get_MemIF_Context /Auddrv_Get_MemIF_Context_by_fp
 *
 * DESCRIPTION
 *  when start certaion type of MEMof , call set top use fp as read / write identiti.
 *
 *****************************************************************************
 */

AFE_MEM_CONTROL_T *Auddrv_Get_MemIF_Context(int MEM_Type)
{
    switch (MEM_Type)
    {
	case MEM_DL1:
	    return &AFE_dL1_Control_context;
	case MEM_DL2:
	    return &AFE_dL2_Control_context;
	case MEM_AWB:
	    return &AWB_Control_context;
	case MEM_VUL:
	    return &VUL_Control_context;
	case MEM_MOD_DAI:
	    return &MODDAI_Control_context;
	default:
	    pr_err("Auddrv_Get_MemIF_Context MEMTYPE = %d", MEM_Type);
	    return NULL;
    }
}

AFE_MEM_CONTROL_T *Auddrv_Find_MemIF_Fp(struct file *fp)
{
    /* PRINTK_AUDDRV("+Auddrv_Find_MemIF_Fp  = %p arg = %d\n",fp); */
    if (AFE_dL1_Control_context.flip == fp)
    {
	return &AFE_dL1_Control_context;
    }
    else if (AFE_dL2_Control_context.flip == fp)
    {
	return &AFE_dL2_Control_context;
    }
    else if (AWB_Control_context.flip == fp)
    {
	return &AWB_Control_context;
    }
    else if (VUL_Control_context.flip == fp)
    {
	return &VUL_Control_context;
    }
    else if (MODDAI_Control_context.flip == fp)
    {
	return &MODDAI_Control_context;
    }
    else
    {
	pr_err("+ Auddrv_Find_MemIF_Fp fp = %p\n", fp);
	return NULL;
    }
}

bool Auddrv_CheckRead_MemIF_Fp(int MEM_Type)
{
    switch (MEM_Type)
    {
	case MEM_DL1:
	case MEM_DL2:
	    return false;
	case MEM_AWB:
	case MEM_VUL:
	case MEM_MOD_DAI:
	    return true;
	default:
	    pr_err("Auddrv_CheckRead_MemIF_Fp MEMTYPE = %d", MEM_Type);
	    return false;
    }
}

#if !defined(DL1_MEM_FIXED_IN_SRAM)
int AudDrv_Reassign_Buffer_In_SRAM(struct file *fp, unsigned long arg)
{
#ifdef AUDIO_MEMORY_SRAM
    kal_uint32 u4PhyAddr = 0;
#endif
    AFE_MEM_CONTROL_T *pAfe_MEM_ConTrol;
    AFE_BLOCK_T *pblock;
    int mem_type = (int)arg;
    uint32 AFE_Buffer_Size;
    pAfe_MEM_ConTrol = Auddrv_Get_MemIF_Context(mem_type);
    pblock = &(pAfe_MEM_ConTrol->rBlock);
    AFE_Buffer_Size = pblock->u4BufferSize;
    pr_debug("AudDrv_Reassign_Buffer_In_SRAM type = %d, length %d fp = %p\n", mem_type, AFE_Buffer_Size, fp);
    if (AFE_Buffer_Size > AUDDRV_DL1_MAX_BUFFER_LENGTH)
    {
	return -1;
    }

    /* Reassign memory */
    u4PhyAddr = AFE_INTERNAL_SRAM_PHY_BASE;
    pblock->pucPhysBufAddr = u4PhyAddr;

#ifdef AUDIO_MEM_IOREMAP
    pblock->pucVirtBufAddr = (kal_uint8 *)AFE_SRAM_ADDRESS;
    pr_debug("AudDrv_Reassign_Buffer_In_SRAM AUDIO_MEM_IOREMAP = %p, length = %d, addr 0x%p\n", fp, AFE_Buffer_Size, pblock->pucVirtBufAddr);
#else
    pblock->pucVirtBufAddr = AFE_INTERNAL_SRAM_VIR_BASE;
#endif

    pr_debug("AudDrv_Reassign_Buffer_In_SRAM pucVirtBufAddr = %p\n", pblock->pucVirtBufAddr);

    /* check 32 bytes align */
    if ((pblock->pucPhysBufAddr & 0x1f) != 0)
    {
	pr_err("[Auddrv] AudDrv_Reassign_Buffer_In_SRAM is not aligned (0x%x)\n", pblock->pucPhysBufAddr);
    }

    pblock->u4SampleNumMask = 0x001f;  /* 32 byte align */
    pblock->u4WriteIdx     = 0;
    pblock->u4DMAReadIdx    = 0;
    pblock->u4DataRemained  = 0;
    pblock->u4fsyncflag     = false;
    pblock->uResetFlag      = true;

    /* do set physical address to hardware */
    switch (mem_type)
    {
	case MEM_DL1:
	{
	    Afe_Set_Reg(AFE_DL1_BASE , pblock->pucPhysBufAddr , 0xffffffff);
	    Afe_Set_Reg(AFE_DL1_END  , pblock->pucPhysBufAddr + (AFE_Buffer_Size - 1) , 0xffffffff);
	    Afe_Set_Reg(AFE_MEMIF_MAXLEN, 0x0, 0xf);
	    break;
	}
	case MEM_DL2:
	{
	    Afe_Set_Reg(AFE_DL2_BASE , pblock->pucPhysBufAddr , 0xffffffff);
	    Afe_Set_Reg(AFE_DL2_END  , pblock->pucPhysBufAddr + (AFE_Buffer_Size - 1) , 0xffffffff);
	    Afe_Set_Reg(AFE_MEMIF_MAXLEN, 0x00, 0xf0);
	    break;
	}
	case MEM_AWB:
	{
	    Afe_Set_Reg(AFE_AWB_BASE , pblock->pucPhysBufAddr , 0xffffffff);
	    Afe_Set_Reg(AFE_AWB_END  , pblock->pucPhysBufAddr + (AFE_Buffer_Size - 1) , 0xffffffff);
	    break;
	}
	case MEM_VUL:
	{
	    Afe_Set_Reg(AFE_VUL_BASE , pblock->pucPhysBufAddr , 0xffffffff);
	    Afe_Set_Reg(AFE_VUL_END  , pblock->pucPhysBufAddr + (AFE_Buffer_Size - 1) , 0xffffffff);
	    break;
	}
	case MEM_MOD_DAI:
	{
	    Afe_Set_Reg(AFE_MOD_PCM_BASE , pblock->pucPhysBufAddr , 0xffffffff);
	    Afe_Set_Reg(AFE_MOD_PCM_END  , pblock->pucPhysBufAddr + (AFE_Buffer_Size - 1) , 0xffffffff);
	    break;
	}
	default:
	    pr_err("NO MEM_IF MATCH\n");
	    return -1;
    }
    pr_debug("-AudDrv_Reassign_Buffer_In_SRAM\n");
    return 0;
}

int AudDrv_Reassign_Buffer_In_EMI(struct file *fp, unsigned long arg)
{
    AFE_MEM_CONTROL_T *pAfe_MEM_ConTrol;
    AFE_BLOCK_T *pblock;
    uint32 AFE_Buffer_Size;
    int mem_type = (int)arg;
    pAfe_MEM_ConTrol = Auddrv_Get_MemIF_Context(mem_type);
    pblock = &(pAfe_MEM_ConTrol->rBlock);
    AFE_Buffer_Size = pblock->u4BufferSize;

    if (pblock->pucVirtBufAddrBackup != NULL)
    {
	pblock->pucVirtBufAddr = pblock->pucVirtBufAddrBackup;
	pblock->pucPhysBufAddr = pblock->pucPhysBufAddrBackup;
    }
    pr_debug("AudDrv_Reassign_Buffer_In_EMI type = %d, fp = %p, virtAdddr %p, physAddr %x\n", mem_type, fp, pblock->pucVirtBufAddr, (unsigned int)pblock->pucPhysBufAddr);
    pblock->u4WriteIdx     = 0;
    pblock->u4DMAReadIdx    = 0;
    pblock->u4DataRemained  = 0;
    /* do set physical address to hardware */
    switch (mem_type)
    {
	case MEM_DL1:
	{
	    Afe_Set_Reg(AFE_DL1_BASE , pblock->pucPhysBufAddr , 0xffffffff);
	    Afe_Set_Reg(AFE_DL1_END  , pblock->pucPhysBufAddr + (AFE_Buffer_Size - 1) , 0xffffffff);
	    Afe_Set_Reg(AFE_MEMIF_MAXLEN, 0x1, 0xf);
	    break;
	}
	case MEM_DL2:
	{
	    Afe_Set_Reg(AFE_DL2_BASE , pblock->pucPhysBufAddr , 0xffffffff);
	    Afe_Set_Reg(AFE_DL2_END  , pblock->pucPhysBufAddr + (AFE_Buffer_Size - 1) , 0xffffffff);
	    Afe_Set_Reg(AFE_MEMIF_MAXLEN, 0x10, 0xf0);
	    break;
	}
	case MEM_AWB:
	{
	    Afe_Set_Reg(AFE_AWB_BASE , pblock->pucPhysBufAddr , 0xffffffff);
	    Afe_Set_Reg(AFE_AWB_END  , pblock->pucPhysBufAddr + (AFE_Buffer_Size - 1) , 0xffffffff);
	    break;
	}
	case MEM_VUL:
	{
	    Afe_Set_Reg(AFE_VUL_BASE , pblock->pucPhysBufAddr , 0xffffffff);
	    Afe_Set_Reg(AFE_VUL_END  , pblock->pucPhysBufAddr + (AFE_Buffer_Size - 1) , 0xffffffff);
	    break;
	}
	case MEM_MOD_DAI:
	{
	    Afe_Set_Reg(AFE_MOD_PCM_BASE , pblock->pucPhysBufAddr , 0xffffffff);
	    Afe_Set_Reg(AFE_MOD_PCM_END  , pblock->pucPhysBufAddr + (AFE_Buffer_Size - 1) , 0xffffffff);
	    break;
	}
	default:
	    pr_err("NO MEM_IF MATCH\n");
	    return -1;
    }

    pr_err("-AudDrv_Reassign_Buffer_In_EMI\n");
    return 0;
}
#endif


/*****************************************************************************
 * FILE OPERATION FUNCTION
 *  Auddrv_Set_MemIF_Fp /Auddrv_Release_MemIF_Fp
 *
 * DESCRIPTION
 *  when start certaion type of MEMof , call set top use fp as read / write identiti.
 *
 *****************************************************************************
 */

void Auddrv_Set_MemIF_Fp(struct file *fp, unsigned long arg)
{
    AFE_MEM_CONTROL_T *pAfe_MEM_ConTrol;
    pAfe_MEM_ConTrol = Auddrv_Get_MemIF_Context(arg);
    pr_debug("+Auddrv_Set_MemIF_Fp  = %p arg = %lu\n", fp , arg);
    if (pAfe_MEM_ConTrol == NULL)
    {
	pr_debug("+ pAfe_MEM_ConTrol Error !!! pAfe_MEM_ConTrol = NULL, fp = %p,\n", fp);
	return;
    }
    else if (pAfe_MEM_ConTrol->flip != NULL)
    {
	pr_debug("+ pAfe_MEM_ConTrol flip = %p Error fp = %p\n", pAfe_MEM_ConTrol->flip, fp);
	return;
    }

#if !defined(DL1_MEM_FIXED_IN_SRAM)
    spin_lock(&auddrv_lock);
    if (Aud_Int_Mem_Flag == 0) /* SRAM is not occupied, use SRAM */
    {
	AudDrv_Reassign_Buffer_In_SRAM(fp, arg);
	Aud_Int_Mem_Flag |= (1 << ((int)arg));
    }
    else/* SRAM is occupied */
    {
	/* Use External memeory and disable deep idle so that external memroy can work normally */
	if (Aud_Ext_Mem_Flag == 0)
	{
	    disable_dpidle_by_bit(MT_CG_AUDIO_SW_CG);
	}
	Aud_Ext_Mem_Flag |= (1 << ((int)arg));
    }
    spin_unlock(&auddrv_lock);
#endif
    pAfe_MEM_ConTrol->flip = fp;
    pAfe_MEM_ConTrol->bRunning = true;
    pr_debug("-Auddrv_Set_MemIF_Fp  = %p arg = %lu\n", pAfe_MEM_ConTrol->flip , arg);
}

void Auddrv_Check_Irq(void)
{
    uint32_t u4RegValue = Afe_Get_Reg(AFE_IRQ_STATUS);
    u4RegValue &= 0xf;
    if (u4RegValue & 0x1)
    {
	pr_debug("Auddrv_Check_Irq u4RegValue == %d\n", u4RegValue);
	Afe_Set_Reg(AFE_IRQ_CLR, 1 , 0xff);
    }
    else if (u4RegValue & 0x2)
    {
	pr_debug("Auddrv_Check_Irq u4RegValue == %d\n", u4RegValue);
	Afe_Set_Reg(AFE_IRQ_CLR, 2 , 0xff);
    }
}

void Auddrv_Release_MemIF_Fp(struct file *fp, unsigned long arg)
{
    AFE_MEM_CONTROL_T *pAfe_MEM_ConTrol;
    pAfe_MEM_ConTrol = Auddrv_Get_MemIF_Context(arg);
    pr_debug("+Auddrv_Release_MemIF_Fp  = %p arg = %lu\n", fp , arg);
    if (pAfe_MEM_ConTrol == NULL)
    {
	pr_debug("+ pAfe_MEM_ConTrol Error !!! pAfe_MEM_ConTrol = NULL, fp = %p,\n", fp);
	return;
    }
    else if (pAfe_MEM_ConTrol->flip != fp)
    {
	pr_debug("+ pAfe_MEM_ConTrol flip = %p Error fp = %p\n", pAfe_MEM_ConTrol->flip, fp);
	return;
    }

#if !defined(DL1_MEM_FIXED_IN_SRAM)
    spin_lock(&auddrv_lock);
    if ((Aud_Int_Mem_Flag & (1 << ((int)arg))) != 0) /* This memory type using SRAM, reassign it to EMI */
    {
	AudDrv_Reassign_Buffer_In_EMI(fp, arg);
	Aud_Int_Mem_Flag &= ~(1 << ((int)arg));
    }
    else
    {
	Aud_Ext_Mem_Flag &= ~(1 << ((int)arg));
	if (Aud_Ext_Mem_Flag == 0) /* No memory type using EMI, then enable dpidle of AFE bit. */
	{
	    enable_dpidle_by_bit(MT_CG_AUDIO_SW_CG);
	}
    }
    spin_unlock(&auddrv_lock);
#endif
    pAfe_MEM_ConTrol->flip = NULL;
    pAfe_MEM_ConTrol->bRunning = false;

    pAfe_MEM_ConTrol->rBlock.u4DataRemained = 0;
    pAfe_MEM_ConTrol->rBlock.u4WriteIdx = 0;
    pAfe_MEM_ConTrol->rBlock.u4DMAReadIdx = 0;
    pr_debug("-Auddrv_Release_MemIF_Fp  = %p arg = %lu\n", pAfe_MEM_ConTrol->flip , arg);
}

/*****************************************************************************
 * FILE OPERATION FUNCTION
 *  AudDrv_Reg_Reset
 *
 * DESCRIPTION
 *  when audio driver is not first init and mediaserver died , need to reset audio driver
 *
 *****************************************************************************
 */

void AudDrv_Reg_Reset(void)
{
    unsigned long flags = 0;
    pr_debug("AudDrv_Reg_Reset\n");
    spin_lock_irqsave(&auddrv_lock, flags);
    Afe_Set_Reg(AFE_DAC_CON0 , 0x0 , 0xffffffff);
    Afe_Set_Reg(AFE_DAC_CON1, 0x0, 0xffffffff);
    Afe_Set_Reg(AFE_IRQ_CON, 0x0, 0xffffffff);
    Afe_Set_Reg(AFE_IRQ_CLR, 0x3f, 0xffffffff);
    spin_unlock_irqrestore(&auddrv_lock, flags);
}


/*****************************************************************************
 * FILE OPERATION FUNCTION
 *  AudDrv_Reset
 *
 * DESCRIPTION
 *  when audio driver is not first init and mediaserver died , need to reset audio driver
 *
 *****************************************************************************
 */

void AudDrv_Mem_Reset(void)
{
    int i = 0;
    pr_debug("AudDrv_Mem_Reset\n");
#if defined(DL1_MEM_FIXED_IN_SRAM)
    AudDrv_Force_Free_DL1_Buffer();
    for (i = MEM_DL2; i < NUM_OF_MEM_INTERFACE; i++)
    {
	/* sequence free memory */
	AudDrv_Force_Free_Buffer(i);
    }
#else
    for (i = MEM_DL1; i < NUM_OF_MEM_INTERFACE; i++)
    {
	/* sequence free memory */
	AudDrv_Force_Free_Buffer(i);
    }
#endif
}


/*****************************************************************************
 * FUNCTION
 *  AudDrv_GET_DL1_REMAIN_TIME
 *
 * DESCRIPTION
 *  Get DL1 buffer remained time
 *
 ******************************************************************************/
int AudDrv_GET_DL1_REMAIN_TIME(struct file *fp)
{
    int ret = 0;

    unsigned long flags;
    kal_int32 Afe_consumed_bytes = 0;
    kal_int32 HW_memory_index = 0;
    kal_int32 HW_Cur_ReadIdx = 0;
    kal_int32 RemainSize = 0;
    AFE_BLOCK_T *Afe_Block = &(AFE_dL1_Control_context.rBlock);

    kal_uint32 samplerate = Afe_Get_Reg(AFE_IRQ_CON);
    samplerate = (samplerate >> 4) & 0x0000000f;
    samplerate = AudDrv_SampleRateIndexConvert(samplerate);
    /* PRINTK_AUDDRV("AudDrv_GET_DL1_REMAIN_TIME samplerate=%d\n",samplerate); */

    /* spin lock with interrupt disable */
    spin_lock_irqsave(&auddrv_irqstatus_lock, flags);

    HW_Cur_ReadIdx = Afe_Get_Reg(AFE_DL1_CUR);
    if (HW_Cur_ReadIdx == 0)
    {
	pr_debug("[Auddrv] AudDrv_GET_DL1_REMAIN_TIME HW_Cur_ReadIdx ==0\n");
	HW_Cur_ReadIdx = Afe_Block->pucPhysBufAddr;
    }
    HW_memory_index = (HW_Cur_ReadIdx - Afe_Block->pucPhysBufAddr);
    /*
    PRINTK_AUDDRV("[Auddrv] HW_Cur_ReadIdx=0x%x HW_memory_index = 0x%x Afe_Block->pucPhysBufAddr = 0x%x\n",
	HW_Cur_ReadIdx,HW_memory_index,Afe_Block->pucPhysBufAddr);*/

    /* get hw consume bytes */
    if (HW_memory_index > Afe_Block->u4DMAReadIdx)
    {
	Afe_consumed_bytes = HW_memory_index - Afe_Block->u4DMAReadIdx;
    }
    else if (HW_memory_index == Afe_Block->u4DMAReadIdx)
    {
	Afe_consumed_bytes = 0;
    }
    else
    {
	Afe_consumed_bytes = Afe_Block->u4BufferSize + HW_memory_index - Afe_Block->u4DMAReadIdx;
    }

    if ((Afe_consumed_bytes & 0x07) != 0)
    {
	pr_debug("[Auddrv] GET_DL1_REMAIN_SIZE DMA address is not aligned 8 bytes. Afe_consumed_bytes = [0x%x]\n", Afe_consumed_bytes);
    }
    /*
    PRINTK_AUDDRV("+Auddrv_DL_Interrupt_Handler ReadIdx:%x WriteIdx:%x, DataRemained:%x, Afe_consumed_bytes:%x HW_memory_index = %x\n",
	Afe_Block->u4DMAReadIdx,Afe_Block->u4WriteIdx,Afe_Block->u4DataRemained,Afe_consumed_bytes,HW_memory_index);
	*/

    RemainSize = Afe_Block->u4DataRemained - Afe_consumed_bytes;
    if (RemainSize < 0)
    {
	RemainSize = 0;
	/*
	    PRINTK_AUDDRV("GET_DL1 ReadIdx:%x WriteIdx:%x, DataRemained:%x, Afe_consumed_bytes:%x HW_memory_index = %x, RemainSize=%d\n",
	    Afe_Block->u4DMAReadIdx,Afe_Block->u4WriteIdx,Afe_Block->u4DataRemained,Afe_consumed_bytes,HW_memory_index,RemainSize);
	*/
    }
    spin_unlock_irqrestore(&auddrv_irqstatus_lock, flags);

    /* ret = (((RemainSize * 1000) / 4) / samplerate); */
    ret = RemainSize;

    /* PRINTK_AUDDRV("AudDrv_GET_DL1_REMAIN_TIME samplerate=%d,RemainSize=%d, ret=%d\n",samplerate,RemainSize, ret); */

    return ret;
}


/*****************************************************************************
 * FUNCTION
 *  AudDrv_GET_UL_REMAIN_TIME
 *
 * DESCRIPTION
 *  Get UL buffer remained time
 *
 ******************************************************************************/
int AudDrv_GET_UL_REMAIN_TIME(struct file *fp)
{
    AFE_MEM_CONTROL_T *pAfe_MEM_ConTrol = NULL;
    AFE_BLOCK_T  *Afe_Block = NULL;
    unsigned long flags;
    unsigned long flagsUL;
    kal_uint32 samplerate = 0;
    kal_uint32 HW_Cur_ReadIdx = 0;
    kal_int32 Hw_Get_bytes = 0;
    kal_uint32 HW_Remain_Size = 0;
    int ret = 0;

    /* check which memif need to be read */
    pAfe_MEM_ConTrol = Auddrv_Find_MemIF_Fp(fp);
    Afe_Block = &(pAfe_MEM_ConTrol->rBlock);
    if (pAfe_MEM_ConTrol == NULL)
    {
	pr_err("AudDrv_GET_UL_REMAIN_TIME cannot find MEM control !!!!!!!\n");
	return -1;
    }
    if (!Auddrv_CheckRead_MemIF_Fp(pAfe_MEM_ConTrol->MemIfNum))
    {
	pr_err("AudDrv_GET_UL_REMAIN_TIME cannot find matcg MemIfNum!!!\n");
	return -1;
    }

    if (Afe_Block->u4BufferSize <= 0)
    {
	pr_err("AudDrv_GET_UL_REMAIN_TIME wrong buffer size!!!\n");
	return -1;
    }

    samplerate = Afe_Get_Reg(AFE_DAC_CON1);

    switch (pAfe_MEM_ConTrol->MemIfNum)
    {
	case MEM_VUL:
	    HW_Cur_ReadIdx = Afe_Get_Reg(AFE_VUL_CUR);
	    samplerate = (samplerate >> 16) & 0x0000000f;
	    samplerate = AudDrv_SampleRateIndexConvert(samplerate);
	    break;
#if 0/*  */
	case MEM_DAI:
	    HW_Cur_ReadIdx = Afe_Get_Reg(AFE_DAI_CUR);
	    samplerate = (samplerate >> 20) & 0x00000001;
	    if (samplerate == 0)
	    {
		samplerate = 8000;
	    }
	    else
	    {
		samplerate = 16000;
	    }
	    break;
#endif
	case MEM_AWB:
	    HW_Cur_ReadIdx = Afe_Get_Reg(AFE_AWB_CUR);
	    samplerate = (samplerate >> 12) & 0x0000000f;
	    samplerate = AudDrv_SampleRateIndexConvert(samplerate);
	    break;
	case MEM_MOD_DAI:
	    HW_Cur_ReadIdx = Afe_Get_Reg(AFE_MOD_PCM_CUR);
	    samplerate = (samplerate >> 30) & 0x00000001;
	    if (samplerate == 0)
	    {
		samplerate = 8000;
	    }
	    else
	    {
		samplerate = 16000;
	    }
	    break;
    }

    if (CheckSize(HW_Cur_ReadIdx))
    {
	return -1;
    }
    if (Afe_Block->pucVirtBufAddr  == NULL)
    {
	return -1;
    }

    spin_lock_irqsave(&auddrv_irqstatus_lock, flags);
    spin_lock_irqsave(&auddrv_ULInCtl_lock, flagsUL);
    /* HW already fill in */
    Hw_Get_bytes = (HW_Cur_ReadIdx - Afe_Block->pucPhysBufAddr) - Afe_Block->u4WriteIdx;
    if (Hw_Get_bytes < 0)
    {
	Hw_Get_bytes += Afe_Block->u4BufferSize;
    }

    HW_Remain_Size = Afe_Block->u4DataRemained + Hw_Get_bytes;
    spin_unlock_irqrestore(&auddrv_ULInCtl_lock, flagsUL);
    spin_unlock_irqrestore(&auddrv_irqstatus_lock, flags);

    /* buffer overflow */
    if (HW_Remain_Size > Afe_Block->u4BufferSize)
    {
	HW_Remain_Size = Afe_Block->u4BufferSize;
    }

    ret = (((HW_Remain_Size * 1000) / 4) / samplerate);
    /* PRINTK_AUDDRV("AudDrv_GET_UL_REMAIN_TIME HW_Remain_Size=%d, samplerate=%d, ms=%d",HW_Remain_Size,samplerate,ret); */

    return ret;

}

/*****************************************************************************
 * FUNCTION
 *  AudDrv_AUDIO_REMAINING
 *
 * DESCRIPTION
 *  Get DL buffer remaining size and time stamp
 *
 ******************************************************************************/
void AudDrv_AUDIO_REMAINING(struct file *fp, Data_Remaining *time)
{
	do_posix_clock_monotonic_gettime(&(time->time));
	time->bytes_remaining = AudDrv_GET_DL1_REMAIN_TIME(fp);
}

/*****************************************************************************
 * FILE OPERATION FUNCTION
 *  AudDrv_ioctl
 *
 * DESCRIPTION
 *  IOCTL Msg handle
 *
 *****************************************************************************
 */

static long AudDrv_ioctl(struct file *fp, unsigned int cmd, unsigned long arg)
{
    int  ret = 0;
    Register_Control Reg_Data;
    AMP_Control AMPParam;
    pr_debug("AudDrv_ioctl cmd = %u arg = %lu\n", cmd, arg);
    switch (cmd)
    {
	case SET_AUDSYS_REG:
	{
	    if (copy_from_user((void *)(&Reg_Data), (const void __user *)(arg), sizeof(Reg_Data)))
	    {
		return -EFAULT;
	    }
	    AudDrv_Clk_On();
	    spin_lock(&auddrv_lock);
	    Afe_Set_Reg(Reg_Data.offset, Reg_Data.value, Reg_Data.mask);
	    spin_unlock(&auddrv_lock);
	    AudDrv_Clk_Off();
	    break;
	}
	case GET_AUDSYS_REG:
	{
	    if (copy_from_user((void *)(&Reg_Data), (const void __user *)(arg), sizeof(Reg_Data)))
	    {
		return -EFAULT;
	    }
	    AudDrv_Clk_On();
	    spin_lock(&auddrv_lock);
	    Reg_Data.value = Afe_Get_Reg(Reg_Data.offset);
	    spin_unlock(&auddrv_lock);
	    AudDrv_Clk_Off();
	    if (copy_to_user((void __user *)(arg), (void *)(&Reg_Data), sizeof(Reg_Data)))
	    {
		return -EFAULT;
	    }
	    break;
	}
	case AUDDRV_GPIO_IOCTL:
	{
#if 0   /* Debug only , Won't run @ run time */
	    mt_set_gpio_mode(GPIO136, GPIO_MODE_00);
	    mt_set_gpio_dir(GPIO136, GPIO_DIR_OUT); /* output */
	    mt_set_gpio_out(GPIO136, arg ? GPIO_OUT_ONE : GPIO_OUT_ZERO);
#endif
	    break;
	}
	case SET_ANAAFE_REG:
	{
	    if (copy_from_user((void *)(&Reg_Data), (const void __user *)(arg), sizeof(Reg_Data)))
	    {
		return -EFAULT;
	    }
	    AudDrv_ANA_Clk_On();
	    spin_lock(&auddrv_lock);
	    Ana_Set_Reg(Reg_Data.offset, Reg_Data.value, Reg_Data.mask);
	    spin_unlock(&auddrv_lock);
	    AudDrv_ANA_Clk_Off();
	    break;
	}
	case GET_ANAAFE_REG:
	{
	    if (copy_from_user((void *)(&Reg_Data), (const void __user *)(arg), sizeof(Reg_Data)))
	    {
		return -EFAULT;
	    }
	    AudDrv_ANA_Clk_On();
	    spin_lock(&auddrv_lock);
	    Reg_Data.value = Ana_Get_Reg(Reg_Data.offset);
	    spin_unlock(&auddrv_lock);
	    AudDrv_ANA_Clk_Off();
	    if (copy_to_user((void __user *)(arg), (void *)(&Reg_Data), sizeof(Reg_Data)))
	    {
		return -EFAULT;
	    }
	    break;
	}
	case ALLOCATE_MEMIF_DL1:
	{
	    AudDrv_Clk_On();
	    /* spin_lock(&auddrv_lock);	user space provide protection, Allocation may sleep, and then preemptive */
#if defined(DL1_MEM_FIXED_IN_SRAM)
	    ret = AudDrv_Allocate_DL1_Buffer(fp, arg);
#else
	    ret = AudDrv_Allocate_Buffer(fp, arg, MEM_DL1);
#endif
	    /* spin_unlock(&auddrv_lock);	user space provide protection, Allocation may sleep, and then preemptive */
	    AudDrv_Clk_Off();
	    break;
	}
	case FREE_MEMIF_DL1:
	{
	    AudDrv_Clk_On();
	    /* spin_lock(&auddrv_lock); user space provide protection, Allocation may sleep, and then preemptive */
#if defined(DL1_MEM_FIXED_IN_SRAM)
	    ret = AudDrv_Free_DL1_Buffer(fp);
#else
	    AudDrv_Free_Buffer(fp, MEM_DL1);
#endif
	    /* spin_unlock(&auddrv_lock);	user space provide protection, Allocation may sleep, and then preemptive */
	    AudDrv_Clk_Off();
	    break;
	}
	case ALLOCATE_MEMIF_DL2:
	{
	    AudDrv_Clk_On();
	    ret =  AudDrv_Allocate_Buffer(fp, arg, MEM_DL2);
	    AudDrv_Clk_Off();
	    break;
	}
	case FREE_MEMIF_DL2:
	{
	    AudDrv_Clk_On();
	    ret =  AudDrv_Free_Buffer(fp, MEM_DL2);
	    AudDrv_Clk_Off();
	    break;
	}
	case ALLOCATE_MEMIF_AWB:
	{
	    AudDrv_Clk_On();
	    ret =  AudDrv_Allocate_Buffer(fp, arg, MEM_AWB);
	    AudDrv_Clk_Off();
	    break;
	}
	case FREE_MEMIF_AWB:
	{
	    AudDrv_Clk_On();
	    ret =  AudDrv_Free_Buffer(fp, MEM_AWB);
	    AudDrv_Clk_Off();
	    break;
	}
	case ALLOCATE_MEMIF_ADC:
	{
	    AudDrv_Clk_On();
	    ret =  AudDrv_Allocate_Buffer(fp, arg, MEM_VUL);
	    AudDrv_Clk_Off();
	    break;
	}
	case FREE_MEMIF_ADC:
	{
	    AudDrv_Clk_On();
	    ret =  AudDrv_Free_Buffer(fp, MEM_VUL);
	    AudDrv_Clk_Off();
	    break;
	}
	case ALLOCATE_MEMIF_MODDAI:
	{
	    AudDrv_Clk_On();
	    ret =  AudDrv_Allocate_Buffer(fp, arg, MEM_MOD_DAI);
	    AudDrv_Clk_Off();
	    break;
	}
	case FREE_MEMIF_MODDAI:
	{
	    AudDrv_Clk_On();
	    ret =  AudDrv_Free_Buffer(fp, MEM_MOD_DAI);
	    AudDrv_Clk_Off();
	    break;
	}
	case START_MEMIF_TYPE:
	{
#if defined(CONFIG_MTK_DEEP_IDLE) && defined(DL1_MEM_FIXED_IN_SRAM)
	    int MEM_Type = arg;
	    if (MEM_AWB == MEM_Type || MEM_VUL == MEM_Type ||
		MEM_DAI == MEM_Type || MEM_MOD_DAI == MEM_Type)
	    {
		disable_dpidle_by_bit(MT_CG_AUDIO_SW_CG);
		/* PRINTK_AUDDRV("%s disable_dpidle_by_bit\n",__FUNCTION__); */
	    }
#endif
	    Auddrv_Set_MemIF_Fp(fp, arg);
	    Auddrv_Add_MemIF_Counter(arg);
	    CheckPowerState();
	    break;
	}
	case STANDBY_MEMIF_TYPE:
	{
#if defined(CONFIG_MTK_DEEP_IDLE) && defined(DL1_MEM_FIXED_IN_SRAM)
	    int MEM_Type = arg;
	    if (MEM_AWB == MEM_Type || MEM_VUL == MEM_Type ||
		MEM_DAI == MEM_Type || MEM_MOD_DAI == MEM_Type)
	    {
		enable_dpidle_by_bit(MT_CG_AUDIO_SW_CG);
		/* PRINTK_AUDDRV("%s enable_dpidle_by_bit\n",__FUNCTION__); */
	    }
#endif
	    Auddrv_Check_Irq();
	    Auddrv_Release_MemIF_Fp(fp, arg);
	    Auddrv_Release_MemIF_Counter(arg);
	    ClearInterruptTiming();
	    break;
	}
	case AUD_RESTART:
	{
	    pr_debug("AudDrv AUD_RESTART cmd = %x\n", cmd);
	    /* driver firest start , do notthing */
	    if (Auddrv_First_bootup == true)
	    {
		Auddrv_First_bootup = false;
	    }
	    else
	    {
		unsigned long flags = 0;
		pr_debug("AudDrv +Before AudDrv_Clk_On_DisableISR Clk lock\n");
		spin_lock_irqsave(&auddrv_clk_isr_lock, flags);
		pr_debug("AudDrv +Force DisableISR\n");
		AudDrv_Clk_On_DisableISR();
		pr_debug("AudDrv -Force DisableISR\n");
		spin_unlock_irqrestore(&auddrv_clk_isr_lock, flags);
		pr_debug("AudDrv -After AudDrv_Clk_On_DisableISR Clk lock\n");

		spin_lock(&auddrv_lock);
		if (Aud_AFE_Clk_cntr > 0)
		{
		    unsigned long flags_ = 0;
		    pr_debug("AudDrv +Before Force Disable Clk lock\n");
		    spin_lock_irqsave(&auddrv_clk_isr_lock, flags_);
		    pr_debug("AudDrv +Force Disable Clk\n");
		    AudDrv_Clk_Off();
		    pr_debug("AudDrv -Force Disable Clk\n");
		    spin_unlock_irqrestore(&auddrv_clk_isr_lock, flags_);
		    pr_debug("AudDrv -After Force Disable Clk lock\n");
		    Aud_AFE_Clk_cntr = 0;
		    Aud_Core_Clk_cntr = 0;
		    Afe_Mem_Pwr_on = 0;
		}
		if (Aud_I2S_Clk_cntr > 0)
		{
		    AudDrv_I2S_Clk_Off();
		    Aud_I2S_Clk_cntr = 0;
		}
		Aud_HDMI_Clk_cntr = 0;
		Aud_LineIn_Clk_cntr = 0;

		spin_unlock(&auddrv_lock);
		mutex_lock(&AnaClk_mutex);
		AudDrv_ANA_Clk_Off();
		mutex_unlock(&AnaClk_mutex);
		Aud_ANA_Clk_cntr = 0;
		Aud_ADC_Clk_cntr = 0;

		AudDrv_Clk_On();
		AudDrv_Reg_Reset();
		AudDrv_Mem_Reset();
		AudDrv_Clk_Off();

#if !defined(DL1_MEM_FIXED_IN_SRAM)
		Aud_Int_Mem_Flag   = 0;
		Aud_Ext_Mem_Flag   = 0;
#endif
	    }
	    break;
	}
	case AUD_SET_LINE_IN_CLOCK:
	{
	    pr_debug("+AudDrv AUD_SET_LINE_IN_CLOCK(%ld), lineIn_clk(%d)\n", arg, Aud_LineIn_Clk_cntr);
	    if (arg == 1)
	    {
		spin_lock(&auddrv_lock);
		AudDrv_Clk_On();
		spin_unlock(&auddrv_lock);
	    }
	    else
	    {
		spin_lock(&auddrv_lock);
		AudDrv_Clk_Off();
		spin_unlock(&auddrv_lock);
	    }
	    pr_debug("-AudDrv AUD_SET_LINE_IN_CLOCK, AFE(%d)\n", Aud_AFE_Clk_cntr);
	    break;
	}
	case AUD_SET_CLOCK:
	{
	    unsigned long flags = 0;
	    pr_notice("+AudDrv AUD_SET_CLOCK(%ld)\n", arg);
	    if (arg == 1)
	    {
		spin_lock(&auddrv_lock);
		spin_lock_irqsave(&auddrv_clk_isr_lock, flags);
		AudDrv_Clk_On();
		spin_unlock_irqrestore(&auddrv_clk_isr_lock, flags);
		spin_unlock(&auddrv_lock);
	    }
	    else
	    {
		spin_lock(&auddrv_lock);
		spin_lock_irqsave(&auddrv_clk_isr_lock, flags);
		AudDrv_Clk_Off();
		spin_unlock_irqrestore(&auddrv_clk_isr_lock, flags);
		spin_unlock(&auddrv_lock);
	    }
	    break;
	}
	case AUD_SET_26MCLOCK:
	{
	    pr_notice("+AudDrv AUD_SET_26MCLOCK(%ld),\n", arg);
	    if (arg == 1)
	    {
		spin_lock(&auddrv_lock);
		AudDrv_Core_Clk_On();
		spin_unlock(&auddrv_lock);
	    }
	    else
	    {
		spin_lock(&auddrv_lock);
		AudDrv_Core_Clk_Off();
		spin_unlock(&auddrv_lock);
	    }
	    break;
	}
	case AUD_SET_ADC_CLOCK:
	{
	    pr_notice("+AudDrv AUD_SET_ADC_CLOCK(%ld),\n", arg);
	    if (arg == 1)
	    {
		spin_lock(&auddrv_lock);
		AudDrv_ADC_Clk_On();
		spin_unlock(&auddrv_lock);
	    }
	    else
	    {
		spin_lock(&auddrv_lock);
		AudDrv_ADC_Clk_Off();
		spin_unlock(&auddrv_lock);
	    }
	    break;
	}
	case AUD_SET_I2S_CLOCK:
	{
	    pr_notice("+AudDrv AUD_SET_I2S_CLOCK(%ld),\n", arg);
	    if (arg == 1)
	    {
		spin_lock(&auddrv_lock);
		AudDrv_I2S_Clk_On();
		spin_unlock(&auddrv_lock);
	    }
	    else
	    {
		spin_lock(&auddrv_lock);
		AudDrv_I2S_Clk_Off();
		spin_unlock(&auddrv_lock);
	    }
	    break;
	}
	case AUD_SET_ANA_CLOCK:
	{
	    pr_notice("+AudDrv AUD_SET_ANA_CLOCK(%ld),\n", arg);
	    if (arg == 1)
	    {
		mutex_lock(&AnaClk_mutex);
		AudDrv_ANA_Clk_On();
		mutex_unlock(&AnaClk_mutex);
	    }
	    else
	    {
		mutex_lock(&AnaClk_mutex);
		AudDrv_ANA_Clk_Off();
		mutex_unlock(&AnaClk_mutex);
	    }
	    break;
	}
#if defined(CONFIG_MTK_COMBO) || defined(CONFIG_MTK_COMBO_MODULE)

	case AUDDRV_RESET_BT_FM_GPIO:
	{
	    pr_notice("!! AudDrv, BT OFF, Analog FM, COMBO_AUDIO_STATE_0\n");
	    /* mt_combo_audio_ctrl((COMBO_AUDIO_STATE)COMBO_AUDIO_STATE_0); */
	    mtk_wcn_cmb_stub_audio_ctrl((CMB_STUB_AIF_X)CMB_STUB_AIF_0);
	    break;
	}
	case AUDDRV_SET_BT_PCM_GPIO:
	{
	    pr_notice("!! AudDrv, BT ON, Analog FM, COMBO_AUDIO_STATE_1\n");
	    /* mt_combo_audio_ctrl((COMBO_AUDIO_STATE)COMBO_AUDIO_STATE_1); */
	    mtk_wcn_cmb_stub_audio_ctrl((CMB_STUB_AIF_X)CMB_STUB_AIF_1);
	    break;
	}
	case AUDDRV_SET_FM_I2S_GPIO:
	{
	    pr_notice("!! AudDrv, BT OFF, Digital FM, COMBO_AUDIO_STATE_2\n");
	    /* mt_combo_audio_ctrl((COMBO_AUDIO_STATE)COMBO_AUDIO_STATE_2); */
	    mtk_wcn_cmb_stub_audio_ctrl((CMB_STUB_AIF_X)CMB_STUB_AIF_2);
	    break;
	}
	case AUDDRV_SET_BT_FM_GPIO:
	{
	    pr_notice("!! AudDrv, BT ON, Digital FM, COMBO_AUDIO_STATE_3\n");
	    /* mt_combo_audio_ctrl((COMBO_AUDIO_STATE)COMBO_AUDIO_STATE_3); */
	    mtk_wcn_cmb_stub_audio_ctrl((CMB_STUB_AIF_X)CMB_STUB_AIF_3);
	    break;
	}
	case AUDDRV_RESET_FMCHIP_MERGEIF:
	{
	    break;
	}
        case AUDDRV_SET_BT_I2S_GPIO:
        {
            pr_notice("!! AudDrv, BT ON, use I2S, COMBO_AUDIO_STATE_4 \n");
            mtk_wcn_cmb_stub_audio_ctrl((CMB_STUB_AIF_X)CMB_STUB_AIF_4);
            break;
        }
#else
	case AUDDRV_RESET_BT_FM_GPIO:
	{
	    pr_debug("!! NoCombo, COMBO_AUDIO_STATE_0\n");
	    break;
	}
	case AUDDRV_SET_BT_PCM_GPIO:
	{
	    pr_debug("!! NoCombo, COMBO_AUDIO_STATE_1\n");
	    break;
	}
	case AUDDRV_SET_FM_I2S_GPIO:
	{
	    PRINTK_AUDDRV("!! NoCombo, COMBO_AUDIO_STATE_2\n");
	    break;
	}
#endif
	case AUDDRV_ENABLE_ATV_I2S_GPIO:
	{
#if defined(MTK_MT5192) || defined(MTK_MT5193)
	    pr_notice("AUDDRV_ENABLE_ATV_I2S_GPIO\n");
	    cust_matv_gpio_on();
#endif
	    break;
	}
	case AUDDRV_DISABLE_ATV_I2S_GPIO:
	{
#if defined(MTK_MT5192) || defined(MTK_MT5193)
	    pr_notice("AUDDRV_DISABLE_ATV_I2S_GPIO\n");
	    cust_matv_gpio_off();
#endif
	    break;
	}
	case AUD_SET_HDMI_CLOCK:
	{
	    pr_notice("+AudDrv AUD_SET_ANA_CLOCK(%ld),\n", arg);
	    if (arg == 1)
	    {
		mutex_lock(&AnaClk_mutex);
		AudDrv_HDMI_Clk_On();
		mutex_unlock(&AnaClk_mutex);

	    }
	    else
	    {
		mutex_lock(&AnaClk_mutex);
		AudDrv_HDMI_Clk_Off();
		mutex_unlock(&AnaClk_mutex);
	    }
	    break;
	}
		case GET_EAMP_PARAMETER:
		{
		  pr_debug("AudDrv GET_EAMP_PARAMETER\n");
		  mutex_lock(&gamp_mutex);
		  if (copy_from_user((void *)(&AMPParam), (const void __user *)(arg), sizeof(AMPParam))) {
				mutex_unlock(&gamp_mutex);
				pr_err("Failed to copy from user");
				return -EFAULT;
		  }
		  pr_debug("command=%lu,param1=%lu,param2=%lu\n", AMPParam.command, AMPParam.param1, AMPParam.param2);
		  switch (AMPParam.command)
		  {
			  case AUD_AMP_GET_REGISTER:
			  {
				  ret = Audio_eamp_command(EAMP_GETREGISTER_VALUE, AMPParam.param1, 1);
				  break;
			  }
			  case AUD_AMP_GET_CTRP_NUM:
			  {
				  ret = Audio_eamp_command(EAMP_GET_CTRP_NUM, AMPParam.param1, 1);
				  break;
			  }
			  case AUD_AMP_GET_CTRP_BITS:
			  {
				  ret = Audio_eamp_command(EAMP_GET_CTRP_BITS, AMPParam.param1, 1);
				  break;
			  }
			  case AUD_AMP_GET_CTRP_TABLE:
			  {
				  Audio_eamp_command(EAMP_GET_CTRP_TABLE, (unsigned long)&AMPParam, 1);
				  break;
			  }
			  case AUD_AMP_GET_AMPGAIN:
			  {
				  ret = Audio_eamp_command(EAMP_GETAMP_GAIN, 0, 1);
				  break;
			  }
			  default:
				  break;
		  }
		  mutex_unlock(&gamp_mutex);
		  return ret;
		  break;
		}
		case SET_EAMP_PARAMETER:
		{
		  pr_debug("AudDrv SET_EAMP_PARAMETER\n");
		  mutex_lock(&gamp_mutex);
		  if (copy_from_user((void *)(&AMPParam), (const void __user *)(arg), sizeof(AMPParam))) {
				mutex_unlock(&gamp_mutex);
				pr_err("Failed to copy from user");
				return -EFAULT;
		  }
		  pr_debug("command=%lu,param1=%lu,param2=%lu\n", AMPParam.command, AMPParam.param1, AMPParam.param2);
		  switch (AMPParam.command)
		  {
			  case AUD_AMP_SET_AMPGAIN:
			  {
				  Audio_eamp_command(EAMP_SETAMP_GAIN, AMPParam.param1, 1);
				  break;
			  }
			  case AUD_AMP_SET_REGISTER:
			  {
				  Audio_eamp_command(EAMP_SETREGISTER_VALUE, (unsigned long)&AMPParam, 1);
				  break;
			  }
			  case AUD_AMP_SET_MODE:
			  {
				  Audio_eamp_command(EAMP_SETMODE, AMPParam.param1, 1);
				  break;
			  }
			  default:
				  break;
		  }
		  mutex_unlock(&gamp_mutex);
		  break;
		}

	case SET_SPEAKER_VOL:
	    pr_debug("AudDrv SET_SPEAKER_VOL level:%lu\n", arg);
	    Sound_Speaker_SetVolLevel((int)arg);
	    break;
	case SET_SPEAKER_ON:
	    pr_debug("AudDrv SET_SPEAKER_ON arg:%lu\n", arg);
	    mutex_lock(&gamp_mutex);
	    Sound_Speaker_Turnon((int)arg);
	    AuddrvSpkStatus = true;
	    mutex_unlock(&gamp_mutex);
	    break;
	case SET_SPEAKER_OFF:
	    pr_debug("AudDrv SET_SPEAKER_OFF arg:%lu\n", arg);
	    mutex_lock(&gamp_mutex);
	    Sound_Speaker_Turnoff((int)arg);
	    AuddrvSpkStatus = false;
	    mutex_unlock(&gamp_mutex);
	    break;
	case SET_HEADSET_STATE:
	    mutex_lock(&gamp_mutex);
	    pr_debug("!! AudDrv SET_HEADSET_STATE arg:%lu\n", arg);
	    if (arg)
	    {
		Sound_Headset_Turnon();
	    }
	    else
	    {
		Sound_Headset_Turnoff();
	    }
	    mutex_unlock(&gamp_mutex);
	    break;
	case SET_HEADPHONE_ON:
	    pr_debug("AudDrv SET_HEADPHONE_ON arg:%lu\n", arg);
	    mutex_lock(&gamp_mutex);
	    Audio_eamp_command(EAMP_HEADPHONE_OPEN, arg, 1);
	    mutex_unlock(&gamp_mutex);
	    break;
	case SET_HEADPHONE_OFF:
	    pr_debug("AudDrv SET_HEADPHONE_OFF arg:%lu\n", arg);
	    mutex_lock(&gamp_mutex);
	    Audio_eamp_command(EAMP_HEADPHONE_CLOSE, arg, 1);
	    mutex_unlock(&gamp_mutex);
	    break;
	case SET_EARPIECE_ON:
	    pr_debug("AudDrv SET_EARPIECE_ON arg:%lu\n", arg);
	    mutex_lock(&gamp_mutex);
	    Audio_eamp_command(EAMP_EARPIECE_OPEN, arg, 1);
	    mutex_unlock(&gamp_mutex);
	    break;
	case SET_EARPIECE_OFF:
	    pr_notice("AudDrv SET_EARPIECE_OFF arg:%lu\n", arg);
	    mutex_lock(&gamp_mutex);
	    Audio_eamp_command(EAMP_EARPIECE_CLOSE, arg, 1);
	    mutex_unlock(&gamp_mutex);
	    break;
	case SET_AUDIO_STATE:
	{
	    SPH_Control SPH_Ctrl_State_Temp;
	    pr_debug("AudDrv SET_AUDIO_STATE\n");
	    if (copy_from_user((void *)(&SPH_Ctrl_State_Temp), (const void __user *)(arg), sizeof(SPH_Control)))
	    {
		return -EFAULT;
	    }
	    /* Need to use a temp struct since it is dangerous to enter copy_from_user() when spin_lock is held. Because copy_from_user() may sleep. */
	    spin_lock_bh(&auddrv_SphCtlState_lock);
	    memcpy((void *)&SPH_Ctrl_State, (void *)&SPH_Ctrl_State_Temp, sizeof(SPH_Control));
	    spin_unlock_bh(&auddrv_SphCtlState_lock);
	    dmb();
	    pr_notice("SET_AUDIO_STATE bBgsFlag:%d,bRecordFlag:%d,bSpeechFlag:%d,bTtyFlag:%d,bVT:%d,bAudio:%d\n",
			SPH_Ctrl_State.bBgsFlag,
			SPH_Ctrl_State.bRecordFlag,
			SPH_Ctrl_State.bSpeechFlag,
			SPH_Ctrl_State.bTtyFlag,
			SPH_Ctrl_State.bVT,
			SPH_Ctrl_State.bAudioPlay);
	    break;
	}
	case GET_AUDIO_STATE:
	{
	    pr_notice("AudDrv GET_AUDIO_STATE\n");
	    if (copy_to_user((void __user *)arg, (void *)&SPH_Ctrl_State, sizeof(SPH_Control)))
	    {
		return -EFAULT;
	    }
	    pr_notice("GET_AUDIO_STATE bBgsFlag:%d,bRecordFlag:%d,bSpeechFlag:%d,bTtyFlag:%d,bVT:%d,bAudio:%d\n",
			SPH_Ctrl_State.bBgsFlag,
			SPH_Ctrl_State.bRecordFlag,
			SPH_Ctrl_State.bSpeechFlag,
			SPH_Ctrl_State.bTtyFlag,
			SPH_Ctrl_State.bVT,
			SPH_Ctrl_State.bAudioPlay);
	    break;
	}
	case GET_PMIC_VERSION:
	{
	    ret = 0;
	    break;
	}
	case AUDDRV_LOG_PRINT:
	{
	    pr_debug("AudDrv AUDDRV_LOG_PRINT\n");
	    pr_debug("Afe_Mem_Pwr_on =0x%x\n", Afe_Mem_Pwr_on);
	    pr_debug("Aud_AFE_Clk_cntr = 0x%x\n", Aud_AFE_Clk_cntr);
	    pr_debug("Aud_ANA_Clk_cntr = 0x%x\n", Aud_ANA_Clk_cntr);
	    pr_debug("Aud_HDMI_Clk_cntr = 0x%x\n", Aud_HDMI_Clk_cntr);
	    pr_debug("Aud_I2S_Clk_cntr = 0x%x\n", Aud_I2S_Clk_cntr);
	    pr_debug("AuddrvSpkStatus = 0x%x\n", AuddrvSpkStatus);
	    Ana_Log_Print();
	    Afe_Log_Print();
	    break;
	}
	case AUDDRV_GET_AUXADC_CHANNEL_VALUE:
	{
	    int adc_return = 0;
	    adc_return = PMIC_IMM_GetOneChannelValue(arg, 1, 0);
	    pr_notice("+AudDrv_GetAuxAdc arg %lu, adc_return 0x%x", arg, adc_return);
	    ret = adc_return;
	    break;
	}
	case AUDDRV_AEE_IOCTL:
	{
	    pr_notice("AudDrv AUDDRV_AEE_IOCTL  arg = %lu", arg);
	    AuddrvAeeEnable = arg;
	    break;
	}

	case AUDDRV_GET_DL1_REMAINDATA_TIME:
	{
	    /* pr_notice("AudDrv AUDDRV_GET_DL1_REMAINDATA_TIME "); */
	    ret = AudDrv_GET_DL1_REMAIN_TIME(fp);
	    break;
	}
	case AUDDRV_GET_UL_REMAINDATA_TIME:
	{
	    /* pr_notice("AudDrv AUDDRV_GET_UL_REMAINDATA_TIME "); */
	    ret = AudDrv_GET_UL_REMAIN_TIME(fp);
	    break;
	}
	    case AUDDRV_AUDIO_REMAINING:
		{
			Data_Remaining data;
			AudDrv_AUDIO_REMAINING(fp, &data);
			if (copy_to_user((void __user *)(arg), (void *)(&data), sizeof(Data_Remaining)))
				return -EFAULT;

			break;
		}
	default:
	{
	    pr_debug("AudDrv Fail IOCTL command no such ioctl cmd = %x\n", cmd);
	    ret = -1;
	    break;
	}
    }
    return ret;
}

/*****************************************************************************
 * FILE OPERATION FUNCTION
 *  AudDrv_write
 *
 * DESCRIPTION
 *  User space Write data to (kernel)HW buffer
 *
 * PARAMETERS
 *  fp      [in]
 *  data    [in] data pointer
 *  count   [in] number of bytes to be written
 *  offset  [in] no use
 *
 * RETURNS
 *  None
 *
 *****************************************************************************
 */
static ssize_t AudDrv_write(struct file *fp, const char __user *data, size_t count, loff_t *offset)
{
    AFE_MEM_CONTROL_T *pAfe_MEM_ConTrol = NULL;
    AFE_BLOCK_T  *Afe_Block = NULL;
    int written_size = count , ret = 0, copy_size = 0, Afe_WriteIdx_tmp;
    unsigned long flags;
    char *data_w_ptr = (char *)data;

    pr_debug("+AudDrv_writeAudDrv_write = %p count = %d\n",fp ,count);

    /* check which memif nned to be write */
    pAfe_MEM_ConTrol = Auddrv_Find_MemIF_Fp(fp);
    Afe_Block = &(pAfe_MEM_ConTrol->rBlock);

    if ((pAfe_MEM_ConTrol == NULL) || (Afe_Block == NULL))
    {
	pr_debug("AudDrv_writeAudDrv_write g fbut find no MEM control block");
	msleep(60);
	return written_size;
    }
    /* handle for buffer management */
    /*
    PRINTK_AUDDRV("AudDrv_write WriteIdx=%x, ReadIdx=%x, DataRemained=%x\n",
		Afe_Block->u4WriteIdx, Afe_Block->u4DMAReadIdx,Afe_Block->u4DataRemained);*/


    if (Afe_Block->u4BufferSize == 0)
    {
	pr_debug("AudDrv_write: u4BufferSize=0 Error");
	msleep(AFE_INT_TIMEOUT);
	return -1;
    }

    AudDrv_getDLInterval();

    while (count)
    {
	spin_lock_irqsave(&auddrv_DLCtl_lock, flags);
	copy_size = Afe_Block->u4BufferSize - Afe_Block->u4DataRemained;  /* free space of the buffer */
	spin_unlock_irqrestore(&auddrv_DLCtl_lock, flags);
	if (count <= copy_size)
	{
	    if (copy_size < 0)
	    {
		copy_size = 0;
		msleep(DL1_Interrupt_Interval >> 1);
	    }
	    else
	{
	    copy_size = count;
	    }
	    pr_debug("AudDrv_write copy_size:%x\n", copy_size);  // (free  space of buffer)
	}

	if (copy_size != 0)
	{
	    spin_lock_irqsave(&auddrv_DLCtl_lock, flags);
	    Afe_WriteIdx_tmp = Afe_Block->u4WriteIdx;
	    spin_unlock_irqrestore(&auddrv_DLCtl_lock, flags);
	   if (Afe_WriteIdx_tmp > Afe_Block->u4BufferSize) {
	       pr_debug("[AudDrv][Warn]mcmcpy Afe_Block->pucVirtBufAddr=%p, Afe_WriteIdx= %x data_w_ptr = %p copy_size = %x\n",
                   Afe_Block->pucVirtBufAddr, Afe_WriteIdx_tmp, data_w_ptr, copy_size);
	       Afe_WriteIdx_tmp = 0;
	   }
	    if (Afe_WriteIdx_tmp + copy_size < Afe_Block->u4BufferSize) /* copy once */
	    {
		if (!access_ok(VERIFY_READ, data_w_ptr, copy_size))
		{
		    pr_debug("AudDrv_write 0ptr invalid data_w_ptr=%x, size=%d", (kal_uint32)data_w_ptr, copy_size);
		    pr_debug("AudDrv_write u4BufferSize=%d, u4DataRemained=%d", Afe_Block->u4BufferSize, Afe_Block->u4DataRemained);
		}
		else
		{
		    //pr_debug("mcmcpy Afe_Block->pucVirtBufAddr+Afe_WriteIdx= %x data_w_ptr = %p copy_size = %zu\n",
			//Afe_Block->pucVirtBufAddr+Afe_WriteIdx_tmp,data_w_ptr,copy_size);
		    if (copy_from_user((Afe_Block->pucVirtBufAddr + Afe_WriteIdx_tmp), data_w_ptr, copy_size))
		    {
			pr_err("AudDrv_write Fail copy from user\n");
			return -1;
		    }
		}

		spin_lock_irqsave(&auddrv_DLCtl_lock, flags);
		Afe_Block->u4DataRemained += copy_size;
		Afe_Block->u4WriteIdx = Afe_WriteIdx_tmp + copy_size;
		Afe_Block->u4WriteIdx %= Afe_Block->u4BufferSize;
		spin_unlock_irqrestore(&auddrv_DLCtl_lock, flags);
		data_w_ptr += copy_size;
		count -= copy_size;
		pr_debug("AudDrv_write finish1, copy_size:%x, WriteIdx:%x, ReadIdx=%x, DataRemained:%x, count=%d \r\n",
		    copy_size,Afe_Block->u4WriteIdx,Afe_Block->u4DMAReadIdx,Afe_Block->u4DataRemained,count);
	    }
	    else  /* copy twice */
	    {
		kal_uint32 size_1 = 0, size_2 = 0;
		size_1 = Afe_Block->u4BufferSize - Afe_WriteIdx_tmp;
		size_2 = copy_size - size_1;
		if (!access_ok(VERIFY_READ, data_w_ptr, size_1))
		{
		    pr_debug("AudDrv_write 1ptr invalid data_w_ptr=%x, size_1=%d", (kal_uint32)data_w_ptr, size_1);
		    pr_debug("AudDrv_write u4BufferSize=%d, u4DataRemained=%d", Afe_Block->u4BufferSize, Afe_Block->u4DataRemained);
		}
		else
		{
		    //pr_debug("mcmcpy Afe_Block->pucVirtBufAddr+Afe_WriteIdx= %x data_w_ptr = %p size_1 = %zu\n",
			//Afe_Block->pucVirtBufAddr+Afe_WriteIdx_tmp,data_w_ptr,size_1);
		    if ((copy_from_user((Afe_Block->pucVirtBufAddr + Afe_WriteIdx_tmp), data_w_ptr , size_1)))
		    {
			pr_err("AudDrv_write Fail 1 copy from user");
			return -1;
		    }
		}
		spin_lock_irqsave(&auddrv_DLCtl_lock, flags);
		Afe_Block->u4DataRemained += size_1;
		Afe_Block->u4WriteIdx = Afe_WriteIdx_tmp + size_1;
		Afe_Block->u4WriteIdx %= Afe_Block->u4BufferSize;
		Afe_WriteIdx_tmp = Afe_Block->u4WriteIdx;
		spin_unlock_irqrestore(&auddrv_DLCtl_lock, flags);

		if (!access_ok(VERIFY_READ, data_w_ptr + size_1, size_2))
		{
		    pr_debug("AudDrv_write 2ptr invalid data_w_ptr=%x, size_1=%d, size_2=%d", (kal_uint32)data_w_ptr, size_1, size_2);
		    pr_debug("AudDrv_write u4BufferSize=%d, u4DataRemained=%d", Afe_Block->u4BufferSize, Afe_Block->u4DataRemained);
		}
		else
		{
		    //pr_debug("mcmcpy Afe_Block->pucVirtBufAddr+Afe_WriteIdx= %x data_w_ptr+size_1 = %p size_2 = %zu\n",
			//Afe_Block->pucVirtBufAddr+Afe_WriteIdx_tmp,data_w_ptr+size_1,size_2);
		    if ((copy_from_user((Afe_Block->pucVirtBufAddr + Afe_WriteIdx_tmp), (data_w_ptr + size_1), size_2)))
		    {
			pr_err("AudDrv_write Fail 2  copy from user");
			return -1;
		    }
		}
		spin_lock_irqsave(&auddrv_DLCtl_lock, flags);
		\
		Afe_Block->u4DataRemained += size_2;
		Afe_Block->u4WriteIdx = Afe_WriteIdx_tmp + size_2;
		Afe_Block->u4WriteIdx %= Afe_Block->u4BufferSize;
		spin_unlock_irqrestore(&auddrv_DLCtl_lock, flags);
		count -= copy_size;
		data_w_ptr += copy_size;
		pr_debug("AudDrv_write finish2, copy size:%x, WriteIdx:%x,ReadIdx=%x DataRemained:%x \r\n",
		    copy_size,Afe_Block->u4WriteIdx,Afe_Block->u4DMAReadIdx,Afe_Block->u4DataRemained );

	    }
	}
	else
	{
	    pr_debug("AudDrv_write copy_size =0, copy size:%x, WriteIdx:%x,ReadIdx=%x DataRemained:%x \r\n",
		copy_size,Afe_Block->u4WriteIdx,Afe_Block->u4DMAReadIdx,Afe_Block->u4DataRemained );
	}

	if (count != 0)
	{
	    unsigned long long t1, t2;
	    pr_debug("AudDrv_write wait for interrupt count=%x\n",count);
	    t1 = sched_clock(); /* in ns (10^9) */
	    switch (pAfe_MEM_ConTrol->MemIfNum)
	    {
		case MEM_DL1:
		{
		    DL1_wait_queue_flag = 0;
		    ret = wait_event_interruptible_timeout(DL1_Wait_Queue, DL1_wait_queue_flag, (DL1_Interrupt_Interval * 2 / 10));
		    break;
		}
		case MEM_DL2:
		{
		    DL2_wait_queue_flag = 0;
		    ret = wait_event_interruptible_timeout(DL2_Wait_Queue, DL2_wait_queue_flag, (DL1_Interrupt_Interval * 2 / 10));
		    break;
		}
	    }
	    t2 = sched_clock(); /* in ns (10^9) */
	    /* PRINTK_AUDDRV("auddrv write t2 = %llu t1 = %llu t2-t1 = %llu WriteArrayIndex = %d\n",t2,t1,t2-t1,WriteArrayIndex); */
	    t2 = t2 - t1; /* in ns (10^9) */
	    SaveWriteWaitEvent((unsigned int)t2);
	    CheckWriteWaitEvent();
	    if ((ret <= 0) && (t2  > DL1_Interrupt_Interval * 1000000))
	    {
		pr_debug("AudDrv_write timeout, [Warning]blocked by others.(%llu)ns,(%d)jiffies written_size = %d\n",
			      t2, (DL1_Interrupt_Interval * 2 / 10), written_size);
		pr_debug("auddrv write t2 = %llu t1 = %llu t2-t1 = %llu WriteArrayIndex = %d\n", t2, t1, t2 - t1, WriteArrayIndex);
		/* handle for wait interrupt timoout */
		spin_lock_irqsave(&auddrv_irqstatus_lock, flags);
		AudIrqReset = true;
		spin_unlock_irqrestore(&auddrv_irqstatus_lock, flags);
		return written_size;
	    }
	}
	/* here need to wait for interrupt handler */
    }
    /* PRINTK_AUDDRV("AudDrv_write written_size = %d\n",written_size); */
    return written_size;
}


/*****************************************************************************
 * FILE OPERATION FUNCTION
 *  AudDrv_Read
 *
 * DESCRIPTION
 *  User space read  data from  (kernel)HW buffer
 *
 * PARAMETERS
 *  fp      [in]
 *  data    [in] data pointer
 *  count   [in] number of bytes to be read
 *  offset  [in] no use
 *
 * RETURNS
 *  None
 *
 ******************************************************************************/


ssize_t AudDrv_MEMIF_Read(struct file *fp,  char __user *data, size_t count, loff_t *offset)
{
    AFE_MEM_CONTROL_T *pAfe_MEM_ConTrol = NULL;
    AFE_BLOCK_T  *Afe_Block = NULL;
    char *Read_Data_Ptr = (char *)data;
    ssize_t ret , DMA_Read_Ptr = 0 , read_size = 0, read_count = 0;
    unsigned long flags;

    /* check which memif need to be read */
    pAfe_MEM_ConTrol = Auddrv_Find_MemIF_Fp(fp);
    Afe_Block = &(pAfe_MEM_ConTrol->rBlock);
    if (pAfe_MEM_ConTrol == NULL)
    {
	pr_debug("cannot find MEM control !!!!!!!\n");
	msleep(50);
	return -1;
    }
    if (!Auddrv_CheckRead_MemIF_Fp(pAfe_MEM_ConTrol->MemIfNum))
    {
	pr_debug("cannot find matcg MemIfNum!!!\n");
	msleep(50);
	return -1;
    }

    if (Afe_Block->u4BufferSize <= 0)
    {
	msleep(50);
	return -1;
    }
    /* handle for buffer management */
   pr_debug("AudDrv_MEMIF_Read WriteIdx=%x, ReadIdx=%x, DataRemained=%x\n",
	Afe_Block->u4WriteIdx, Afe_Block->u4DMAReadIdx,Afe_Block->u4DataRemained);
    while (count)
    {
	if (CheckNullPointer((void *)Afe_Block->pucVirtBufAddr))
	{
	    break;
	}

	spin_lock_irqsave(&auddrv_ULInCtl_lock, flags);
	if (Afe_Block->u4DataRemained >  Afe_Block->u4BufferSize)
	{
	    pr_debug("AudDrv_MEMIF_Read u4DataRemained=%x > u4BufferSize=%x" , Afe_Block->u4DataRemained, Afe_Block->u4BufferSize);
	    Afe_Block->u4DataRemained = 0;
            Afe_Block->u4WriteIdx = Afe_Block->u4WriteIdx%Afe_Block->u4BufferSize;
	    Afe_Block->u4DMAReadIdx   = Afe_Block->u4WriteIdx;
	}
	if (count >  Afe_Block->u4DataRemained)
	{
	    read_size = Afe_Block->u4DataRemained;
	}
	else
	{
	    read_size = count;
	}
	DMA_Read_Ptr = Afe_Block->u4DMAReadIdx;
	spin_unlock_irqrestore(&auddrv_ULInCtl_lock, flags);

	pr_debug("AudDrv_MEMIF_Read finish0, read_count:%x, read_size:%x, u4DataRemained:%x, u4DMAReadIdx:0x%x, u4WriteIdx:%x \r\n",
	    read_count,read_size,Afe_Block->u4DataRemained,Afe_Block->u4DMAReadIdx,Afe_Block->u4WriteIdx);

	if (DMA_Read_Ptr + read_size < Afe_Block->u4BufferSize)
	{
#ifndef SOUND_FAKE_READ
	    if (DMA_Read_Ptr != Afe_Block->u4DMAReadIdx)
	    {
		pr_debug("AudDrv_MEMIF_Read 1, read_size:%x, DataRemained:%x, DMA_Read_Ptr:0x%x, DMAReadIdx:%x \r\n",
			      read_size, Afe_Block->u4DataRemained, DMA_Read_Ptr, Afe_Block->u4DMAReadIdx);
	    }

            if (DMA_Read_Ptr < 0 || DMA_Read_Ptr > Afe_Block->u4BufferSize)
	    {
		pr_debug("AudDrv_MEMIF_Read E1, read_size:%x, DataRemained:%x, DMA_Read_Ptr:0x%x, DMAReadIdx:%x pucVirtBufAddr : %x \r\n",
			      read_size, Afe_Block->u4DataRemained, DMA_Read_Ptr, Afe_Block->u4DMAReadIdx, (unsigned int)Afe_Block->pucVirtBufAddr);
	    }

	    if (copy_to_user((void __user *)Read_Data_Ptr, (Afe_Block->pucVirtBufAddr + DMA_Read_Ptr), read_size))
	    {
		pr_debug("AudDrv_MEMIF_Read Fail 1 copy to user Read_Data_Ptr:%p, pucVirtBufAddr:%p, u4DMAReadIdx:0x%x, DMA_Read_Ptr:0x%x, read_size:%x",
			      Read_Data_Ptr, Afe_Block->pucVirtBufAddr, Afe_Block->u4DMAReadIdx, DMA_Read_Ptr, read_size);
		if (read_count == 0)
		{
		    return -1;
		}
		else
		{
		    return read_count;
		}
	    }
#else
	    copy_to_user_fake(Read_Data_Ptr , read_size);
#endif
	    read_count += read_size;
	    spin_lock_irqsave(&auddrv_ULInCtl_lock, flags);
	    Afe_Block->u4DataRemained -= read_size;
	    Afe_Block->u4DMAReadIdx += read_size;
	    Afe_Block->u4DMAReadIdx %= Afe_Block->u4BufferSize;
	    DMA_Read_Ptr = Afe_Block->u4DMAReadIdx;
	    spin_unlock_irqrestore(&auddrv_ULInCtl_lock, flags);

	    Read_Data_Ptr += read_size;
	    count -= read_size;
	    pr_debug("AudDrv_MEMIF_Read finish1, copy size:%x, u4DMAReadIdx:0x%x, u4WriteIdx:%x, u4DataRemained:%x \r\n",
	    read_size,Afe_Block->u4DMAReadIdx,Afe_Block->u4WriteIdx,Afe_Block->u4DataRemained );
	}

	else
	{
	    uint32 size_1 = Afe_Block->u4BufferSize - DMA_Read_Ptr;
	    uint32 size_2 = read_size - size_1;
#ifndef SOUND_FAKE_READ
	    if (DMA_Read_Ptr != Afe_Block->u4DMAReadIdx)
	    {
		pr_debug("AudDrv_MEMIF_Read 2, read_size1:%x, DataRemained:%x, DMA_Read_Ptr:0x%x, DMAReadIdx:%x \r\n",
		     size_1,Afe_Block->u4DataRemained,DMA_Read_Ptr,Afe_Block->u4DMAReadIdx);
	    }

            if (DMA_Read_Ptr < 0 || DMA_Read_Ptr > Afe_Block->u4BufferSize)
	    {
		pr_debug("AudDrv_MEMIF_Read E2, read_size:%x, DataRemained:%x, DMA_Read_Ptr:0x%x, DMAReadIdx:%x pucVirtBufAddr : %x \r\n",
			      read_size, Afe_Block->u4DataRemained, DMA_Read_Ptr, Afe_Block->u4DMAReadIdx, (unsigned int)Afe_Block->pucVirtBufAddr);
	    }

	    if (copy_to_user((void __user *)Read_Data_Ptr, (Afe_Block->pucVirtBufAddr + DMA_Read_Ptr), size_1))
	    {
		pr_debug("AudDrv_MEMIF_Read Fail 2 copy to user Read_Data_Ptr:%p, pucVirtBufAddr:%p, u4DMAReadIdx:0x%x, DMA_Read_Ptr:0x%x, read_size:%x",
		     Read_Data_Ptr,Afe_Block->pucVirtBufAddr, Afe_Block->u4DMAReadIdx, DMA_Read_Ptr, read_size);
		if (read_count == 0)
		{
		    return -1;
		}
		else
		{
		    return read_count;
		}
	    }
#else
	    copy_to_user_fake(Read_Data_Ptr, size_1);
#endif
	    read_count += size_1;
	    spin_lock_irqsave(&auddrv_ULInCtl_lock, flags);
	    Afe_Block->u4DataRemained -= size_1;
	    Afe_Block->u4DMAReadIdx += size_1;
	    Afe_Block->u4DMAReadIdx %= Afe_Block->u4BufferSize;
	    DMA_Read_Ptr = Afe_Block->u4DMAReadIdx;
	    spin_unlock_irqrestore(&auddrv_ULInCtl_lock, flags);
	    pr_debug("AudDrv_MEMIF_Read finish2, copy size_1:%x, u4DMAReadIdx:0x%x, u4WriteIdx:0x%x, u4DataRemained:%x \r\n",
		size_1,Afe_Block->u4DMAReadIdx,Afe_Block->u4WriteIdx,Afe_Block->u4DataRemained );
#ifndef SOUND_FAKE_READ
	    if (DMA_Read_Ptr != Afe_Block->u4DMAReadIdx)
	    {
		/*
		xlog_printk(ANDROID_LOG_INFO, "Sound","AudDrv_AWB_Read 3, read_size2:%x, DataRemained:%x, DMA_Read_Ptr:0x%x, DMAReadIdx:%x \r\n",
		    size_2,Afe_Block->u4DataRemained,DMA_Read_Ptr,Afe_Block->u4DMAReadIdx);*/
	    }
            if (DMA_Read_Ptr < 0 || DMA_Read_Ptr > Afe_Block->u4BufferSize)
	    {
		pr_debug("AudDrv_MEMIF_Read E3, read_size:%x, DataRemained:%x, DMA_Read_Ptr:0x%x, DMAReadIdx:%x pucVirtBufAddr : %x \r\n",
			      read_size, Afe_Block->u4DataRemained, DMA_Read_Ptr, Afe_Block->u4DMAReadIdx, (unsigned int)Afe_Block->pucVirtBufAddr);
	    }

	    if (copy_to_user((void __user *)(Read_Data_Ptr + size_1), (Afe_Block->pucVirtBufAddr + DMA_Read_Ptr), size_2))
	    {
		/*
		xlog_printk(ANDROID_LOG_ERROR, "Sound","AudDrv_MEMIF_Read Fail 3 copy to user Read_Data_Ptr:%p, pucVirtBufAddr:%p, u4DMAReadIdx:0x%x, DMA_Read_Ptr:0x%x, read_size:%x",
		    Read_Data_Ptr, Afe_Block->pucVirtBufAddr, Afe_Block->u4DMAReadIdx, DMA_Read_Ptr, read_size);*/
		if (read_count == 0)
		{
		    return -1;
		}
		else
		{
		    return read_count;
		}
	    }
#else
	    copy_to_user_fake((Read_Data_Ptr + size_1), size_2);
#endif
	    read_count += size_2;
	    spin_lock_irqsave(&auddrv_ULInCtl_lock, flags);
	    Afe_Block->u4DataRemained -= size_2;
	    Afe_Block->u4DMAReadIdx += size_2;
	    DMA_Read_Ptr = Afe_Block->u4DMAReadIdx;
	    spin_unlock_irqrestore(&auddrv_ULInCtl_lock, flags);

	    count -= read_size;
	    Read_Data_Ptr += read_size;
	 pr_debug("AudDrv_MEMIF_Read finish3, copy size_2:%x, u4DMAReadIdx:0x%x, u4WriteIdx:0x%x u4DataRemained:%x \r\n",
		size_2,Afe_Block->u4DMAReadIdx,Afe_Block->u4WriteIdx,Afe_Block->u4DataRemained );
	}

	if (count != 0)
	{
	   pr_debug("AudDrv_MEMIF_Read wait for interrupt signal pAfe_MEM_ConTrol->MemIfNum = %d\n",pAfe_MEM_ConTrol->MemIfNum);
	    switch (pAfe_MEM_ConTrol->MemIfNum)
	    {
		case MEM_VUL:
		{
		    VUL_wait_queue_flag = 0;
		    ret = wait_event_interruptible_timeout(VUL_Wait_Queue, VUL_wait_queue_flag, AFE_UL_TIMEOUT);
		    break;
		}
		case MEM_AWB:
		{
		    AWB_wait_queue_flag = 0;
		    ret = wait_event_interruptible_timeout(AWB_Wait_Queue, AWB_wait_queue_flag, AFE_UL_TIMEOUT);
		    break;
		}
		case MEM_MOD_DAI:
		{
		    MODDAI_wait_queue_flag = 0;
		    ret = wait_event_interruptible_timeout(MODDAI_Wait_Queue, MODDAI_wait_queue_flag, AFE_UL_TIMEOUT);
		    break;
		}
		default:
		    pr_debug("cannot find matcg MemIfNum!!!\n");
		    msleep(200);
		    return -1;
	    }

	    if (ret <= 0)
	    {
		pr_err("AudDrv_Read wait_event_interruptible_timeout, No Audio Interrupt! ret  ret = %dn", ret);
		return read_count;
	    }
	}
    }
    return read_count;
}


/*****************************************************************************
 * FILE OPERATION FUNCTION
 *  AudDrv_read
 *
 * DESCRIPTION
 *  User space Read data from (kernel)HW buffer
 *
 * PARAMETERS
 *  fp      [in]
 *  data    [in] data pointer
 *  count   [in] number of bytes to be written
 *  offset  [in] no use
 *
 * RETURNS
 *  None
 *
 *****************************************************************************
 */
static ssize_t AudDrv_read(struct file *fp,  char __user *data, size_t count, loff_t *offset)
{
    uint32 read_count = 0;
    read_count = AudDrv_MEMIF_Read(fp, data,  count, offset);
    return read_count;
}

/*****************************************************************************
 * FUNCTION
 *  AudDrv_flush
 *
 * DESCRIPTION
 *
 *
 * PARAMETERS
 *  fp   [in]
 *  flip [in]
 *  mode [in]
 *
 * RETURNS
 *  None
 *
 *****************************************************************************
 */

static int AudDrv_flush(struct file *flip, fl_owner_t id)
{
    pr_debug("+AudDrv_flush\n");
    Auddrv_Flush_counter++;
    pr_debug("-AudDrv_flush\n");
    return 0;
}

/*****************************************************************************
 * STRUCT
 *  VM Operations
 *
 *****************************************************************************
 */
void AudDrv_vma_open(struct vm_area_struct *vma)
{
    pr_debug("AudDrv_vma_open virt:%lx, phys:%lx, length:%lx\n", vma->vm_start, vma->vm_pgoff << PAGE_SHIFT, vma->vm_end - vma->vm_start);
}

void AudDrv_vma_close(struct vm_area_struct *vma)
{
    pr_debug("AudDrv_vma_close virt");
}

/*
static struct vm_operations_struct AudDrv_remap_vm_ops =
{
   .open  = AudDrv_vma_open,
   .close = AudDrv_vma_close
};
*/

/*****************************************************************************
 * FUNCTION
 *  AudDrv_remap_mmap
 *
 * DESCRIPTION
 *   mmap hardware memory to userspace
 *
 * PARAMETERS
 *  flip   [in]
 *  vma [in]
 *
 *
 * RETURNS
 *  status
 *
 *****************************************************************************
 */

static int AudDrv_remap_mmap(struct file *flip, struct vm_area_struct *vma)
{
    pr_debug("AudDrv_remap_mmap\n");
    /*
    vma->vm_pgoff =( AFE_dl_Control->rBlock.pucPhysBufAddr)>>PAGE_SHIFT;
    if(remap_pfn_range(vma , vma->vm_start , vma->vm_pgoff ,
	vma->vm_end - vma->vm_start , vma->vm_page_prot) < 0)
    {
	xlog_printk(ANDROID_LOG_ERROR, "Sound","AudDrv_remap_mmap remap_pfn_range Fail\n");
	return -EIO;
    }
    vma->vm_ops = &AudDrv_remap_vm_ops;
    AudDrv_vma_open(vma);
    */
    return -1;
}

/*****************************************************************************
 * FUNCTION
 *  AudDrv_fasync
 *
 * DESCRIPTION
 *  Notify the message to user space
 *
 * PARAMETERS
 *  fp   [in]
 *  flip [in]
 *  mode [in]
 *
 * RETURNS
 *  None
 *
 *****************************************************************************
 */
static int AudDrv_fasync(int fd, struct file *flip, int mode)
{
    pr_debug("AudDrv_fasync\n");
    return fasync_helper(fd, flip, mode, &AudDrv_async);
}


/**************************************************************************
 * STRUCT
 *  File Operations and misc device
 *
 **************************************************************************/

static struct file_operations AudDrv_fops =
{
    .owner   = THIS_MODULE,
    .open    = AudDrv_open,
    .release = AudDrv_release,
    .unlocked_ioctl   = AudDrv_ioctl,
    .write   = AudDrv_write,
    .read    = AudDrv_read,
    .flush   = AudDrv_flush,
    .fasync  = AudDrv_fasync,
    .mmap    = AudDrv_remap_mmap
};

static struct miscdevice AudDrv_audio_device =
{
    .minor = MISC_DYNAMIC_MINOR,
    .name = "eac",
    .fops = &AudDrv_fops,
};

struct dev_pm_ops Auddrv_pm_ops =
{
    .suspend = AudDrv_pm_ops_suspend,
    .resume = AudDrv_pm_ops_resume,
    .freeze = AudDrv_pm_ops_suspend,
    .thaw = AudDrv_pm_ops_resume,
    .poweroff = NULL,
    .restore = AudDrv_pm_ops_resume,
    .restore_noirq = NULL,
};

/***************************************************************************
 * FUNCTION
 *  AudDrv_mod_init / AudDrv_mod_exit
 *
 * DESCRIPTION
 *  Module init and de-init (only be called when system boot up)
 *
 **************************************************************************/

static struct platform_driver AudDrv_driver =
{
    .probe    = AudDrv_probe,
    .remove   = AudDrv_remove,
    .shutdown = AudDrv_shutdown,
    .suspend  = AudDrv_suspend,
    .resume   = AudDrv_resume,
    .driver   = {
#ifdef CONFIG_PM
	.pm     = &Auddrv_pm_ops,
#endif
	.name = auddrv_name,
    },
};

struct platform_device AudDrv_device =
{
    .name  = auddrv_name,
    .id    = 0,
    .dev   = {
	.dma_mask = &AudDrv_dmamask,
	.coherent_dma_mask =  0xffffffffUL
    }
};

static int AudDrv_mod_init(void)
{
    int ret = 0;
    pr_debug("+AudDrv_mod_init\n");

    /* register speaker driver */
    Speaker_Register();


    /* Register platform DRIVER */
    ret = platform_driver_register(&AudDrv_driver);
    if (ret)
    {
	pr_err("AudDrv Fail:%d - Register DRIVER\n", ret);
	return ret;
    }

    /* register MISC device */
    if ((ret = misc_register(&AudDrv_audio_device)))
    {
	pr_err("AudDrv_probe misc_register Fail:%d\n", ret);
	return ret;
    }

    /* register cat /proc/audio */
    proc_create("audio",
			   0,
			   NULL,
			   &auddrv_proc_fops);


    wake_lock_init(&Audio_wake_lock, WAKE_LOCK_SUSPEND, "Audio_WakeLock");
    wake_lock_init(&Audio_record_wake_lock, WAKE_LOCK_SUSPEND, "Audio_Record_WakeLock");

    pr_debug("AudDrv_mod_init: Init Audio WakeLock\n");

    return 0;
}

static void  AudDrv_mod_exit(void)
{
    pr_debug("+AudDrv_mod_exit\n");

    /*
    remove_proc_entry("audio", NULL);
    platform_driver_unregister(&AudDrv_driver);
    */

    pr_debug("-AudDrv_mod_exit\n");
}


EXPORT_SYMBOL(GetHeadPhoneState);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(AUDDRV_NAME);
MODULE_AUTHOR(AUDDRV_AUTHOR);

module_init(AudDrv_mod_init);
module_exit(AudDrv_mod_exit);
