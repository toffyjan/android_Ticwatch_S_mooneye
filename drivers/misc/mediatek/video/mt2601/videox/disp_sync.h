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

#ifndef __DISP_SYNC_H__
#define __DISP_SYNC_H__

#include <mt-plat/aee.h>

#define DISP_INVALID_FENCE_FD        -1
#define DISP_INVALID_ION_FD          -1
#define DISP_INVALID_FENCE_INDEX     -1

typedef unsigned int UINT;
typedef unsigned char BOOL;
/* #define NULL                       0x0 */

#ifndef ASSERT
#define ASSERT(expr, fmt, arg...) \
	do { \
		if (!(expr)) \
			aee_kernel_warning("[DISP DRV] ASSERT FAILED(%s:%d):"fmt, __func__, __LINE__, ##arg); \
	} while (0)
#endif

#define AEE_WARNING(tag, fmt, arg...) \
	do { \
		aee_kernel_warning_api(__FILE__, __LINE__, DB_OPT_MMPROFILE_BUFFER, tag, fmt, ##arg); \
	} while (0)

#define XLOG_INFO(fmt, arg...) \
	do { \
		if (log_on) \
			pr_warn("[DISP/SYNC]"fmt, ##arg); \
	} while (0)
#define XLOG_DBG(fmt, arg...) \
	do { \
		if (log_on) \
			pr_warn("[DISP/SYNC]"fmt, ##arg); \
	} while (0)
#define XLOG_WARN(fmt, arg...) \
	do { \
		pr_warn("[DISP/SYNC] "fmt, ##arg); \
	} while (0)
#define XLOG_ERR(fmt, arg...) \
	do { \
		pr_err("[DISP/SYNC] "fmt, ##arg); \
	} while (0)

/* ============================================================================= */
/* forward declarations of external structures */
/* NOTICE: this is the INPUT PARAMETERS directly from its CLIENT */
/* =========================== */
struct ion_client;
struct ion_handle;
struct disp_buffer_info_t;
struct fb_overlay_buffer;

/* ============================================================================= */
/* structure declarations */
/* =========================== */
typedef enum {
	SYNC_STATUS_OK = 0,

	SYNC_STATUS_NOT_IMPLEMENTED,
	SYNC_STATUS_ALREADY_SET,
	SYNC_STATUS_INVALID_PARAM,
	SYNC_STATUS_ERROR,
} SYNC_STATUS;

/* ============================================================================= */
/* function declarations */
/* =========================== */
SYNC_STATUS disp_sync_ion_alloc_buffer(struct ion_client *client, int ion_fd, UINT *mva, struct ion_handle **hnd);
void disp_sync_ion_cache_flush(struct ion_client *client, struct ion_handle *handle);

/**
 * Release all buffer mva of whose fence is signaled.
 * @layer:		specify which timeline (OVL layer)
 */
void disp_sync_release_buffer(UINT session, UINT layer);

/**
 * Update timeline index, all fences behind this will be released later.
 * @layer:		specify which timeline
 * @cur_idx:		fence index to be signaled
 */
int disp_sync_inc_timeline(UINT session, UINT layer, UINT cur_idx);

/**
 * Signal all fences of this timeline to its index
 * @layer:		specify which timeline
 * @idx:		all behind this will be signaled
 */
void disp_sync_signal_fence(UINT session, UINT layer);

/**
 * Do some pre-processing:
 * 1. create fence object
 * 2. create ion mva
 * @buf:		input/output
 * return SYNC_STATUS_OK if success, SYNC_STATUS_ERROR if failed
 */
SYNC_STATUS disp_sync_prepare_buffer(struct disp_buffer_info_t *buf);

/**
 * Deprecated
 * Do some pre-processing:
 * 1. create fence object
 * 2. create ion mva
 * @buf:		input/output
 * return SYNC_STATUS_OK if success, SYNC_STATUS_ERROR if failed
 */
SYNC_STATUS disp_sync_prepare_buffer_deprecated(UINT session, struct fb_overlay_buffer *buf);

/**
 * Query buffer mva of this buffer
 * @layer:		which timeline
 * @idx:		which buffer, input/output
 */
UINT disp_sync_query_buffer_mva(UINT session, UINT layer, UINT idx);

/**
 * Get last Fence index which is signaled
 * @layer:		which timeline
 */
UINT disp_sync_get_last_signaled(UINT session, UINT layer);

/**
 * Release all resource(buffer,fence) on this timeline
 * @layer:		specify which timeline
 */
void disp_sync_release(UINT session, UINT layer);
SYNC_STATUS disp_sync_init(UINT session);
SYNC_STATUS disp_sync_deinit(UINT session);
SYNC_STATUS disp_sync_cleanup(void);

int disp_sync_present_fence_inc(UINT session);
int disp_sync_prepare_present_fence(unsigned long arg);
void disp_sync_set_present_fence(unsigned int present_fence);

#endif				/* __DISP_SYNC_H__ */
