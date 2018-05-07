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

#ifndef __DPI_DRV_H__
#define __DPI_DRV_H__

#include "disp_hal.h"
#include "disp_intr.h"

#ifdef __cplusplus
extern "C" {
#endif

/* --------------------------------------------------------------------------- */

#define DPI_CHECK_RET(expr) \
	do { \
		DPI_STATUS ret = (expr); \
		ASSERT(DPI_STATUS_OK == ret); \
	} while (0)

/* --------------------------------------------------------------------------- */

typedef enum {
	DPI_STATUS_OK = 0,

	DPI_STATUS_ERROR,
} DPI_STATUS;

typedef enum {
	DPI_FB_FORMAT_RGB565 = 0,
	DPI_FB_FORMAT_RGB888 = 1,
	DPI_FB_FORMAT_XRGB888 = 1,
	DPI_FB_FORMAT_RGBX888 = 1,
	DPI_FB_FORMAT_NUM,
} DPI_FB_FORMAT;

typedef enum {
	DPI_RGB_ORDER_RGB = 0,
	DPI_RGB_ORDER_BGR = 1,
} DPI_RGB_ORDER;

typedef enum {
	DPI_FB_0 = 0,
	DPI_FB_1 = 1,
	DPI_FB_2 = 2,
	DPI_FB_NUM,
} DPI_FB_ID;

typedef enum {
	DPI_POLARITY_RISING = 0,
	DPI_POLARITY_FALLING = 1
} DPI_POLARITY;

/* --------------------------------------------------------------------------- */

DPI_STATUS DPI_Init(BOOL isDpiPoweredOn);
DPI_STATUS DPI_Deinit(void);

DPI_STATUS DPI_Init_PLL(LCM_PARAMS *lcm_params);
DPI_STATUS DPI_Set_DrivingCurrent(LCM_PARAMS *lcm_params);

DPI_STATUS DPI_PowerOn(void);
DPI_STATUS DPI_PowerOff(void);

DPI_STATUS DPI_MIPI_PowerOn(void);
DPI_STATUS DPI_MIPI_PowerOff(void);

DPI_STATUS DPI_EnableClk(void);
DPI_STATUS DPI_DisableClk(void);

DPI_STATUS DPI_BackupRegisters(void);
DPI_STATUS DPI_RestoreRegisters(void);
DPI_STATUS DPI_StartTransfer(bool isMutexLocked);
DPI_STATUS DPI_ConfigVsync(DPI_POLARITY polarity, UINT32 pulseWidth, UINT32 backPorch, UINT32 frontPorch);
DPI_STATUS DPI_ConfigHsync(DPI_POLARITY polarity, UINT32 pulseWidth, UINT32 backPorch, UINT32 frontPorch);

DPI_STATUS DPI_OutputSetting(LCM_DPI_PARAMS *pConfig);
DPI_STATUS DPI_FBSetSize(UINT32 width, UINT32 height);

/* Debug */
DPI_STATUS DPI_DumpRegisters(void);

/* FM De-sense */
DPI_STATUS DPI_FMDesense_Query(void);
DPI_STATUS DPI_FM_Desense(unsigned long freq);
DPI_STATUS DPI_Get_Default_CLK(unsigned int *clk);
DPI_STATUS DPI_Get_Current_CLK(unsigned int *clk);
DPI_STATUS DPI_Change_CLK(unsigned int clk);
DPI_STATUS DPI_Reset_CLK(void);

void DPI_mipi_switch(bool on, LCM_PARAMS *lcm_params);
void DPI_DisableIrq(void);
void DPI_EnableIrq(void);

DPI_STATUS DPI_EnableInterrupt(DISP_INTERRUPT_EVENTS eventID);
DPI_STATUS DPI_SetInterruptCallback(void (*pCB) (DISP_INTERRUPT_EVENTS eventID));
DPI_STATUS DPI_WaitVSYNC(void);
void DPI_InitVSYNC(unsigned int vsync_interval);

unsigned int DPI_Check_LCM(void);

/* --------------------------------------------------------------------------- */

#ifdef __cplusplus
}
#endif
#endif				/* __DPI_DRV_H__ */
