/*
 * Atmel maXTouch Touchscreen driver
 *
 * Copyright (C) 2010 Samsung Electronics Co.Ltd
 * Author: Joonyoung Shim <jy0922.shim@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef __LINUX_ATMEL_MXT_MOBVOI_H
#define __LINUX_ATMEL_MXT_MOBVOI_H

#include <linux/types.h>
#include <linux/input.h>
#include <linux/earlysuspend.h>

#define CONFIG_FB_PM
//#define CONFIG_MXT_T61_HR

#define AT_GPIO_CTP_RST_PIN GPIO139

/* Debug function */
#define ATMEL_MAIN_TAG "<mxt main>"
#define MXT_LOG(fmt, args...)            printk(KERN_DEBUG  ATMEL_MAIN_TAG" %d : "fmt, __LINE__, ##args)
#define MXT_MXT_LOG(fmt, args...)

#define TPD_I2C_BUS					0
#define TPD_I2C_ADDR				0x4a    	/* 7 bits, without  r/w bit */

#define CONFIG_MXT_I2C_DMA
#define CONFIG_MXT_I2C_EXTFLAG
#define I2C_ACCESS_NO_REG   			(1 << 4)  	/* no reg address, directly access i2c reg */
#define I2C_ACCESS_NO_CACHE   			(1 << 5)  	/* no dma cache need */
#define I2C_ACCESS_R_REG_FIXED  		(1 << 0)   	/* don't mov reg address if read len is too long */
#define GTP_DMA_MAX_TRANSACTION_LENGTH  	255   		/* for DMA mode */
#define MXT_T72_NOISE_SUPPRESSION_NOISELVCHG    (1 << 4)

/* Feature list */
#define MAX1_WAKEUP_GESTURE_ENABLE
#define LARGE_TOUCH_SUSPEND

#define ESD_MUTUALCAP_REF_LIMIT_LOW 			5000

#define ESD_SELFCAP_DELTA_LIMIT_HIGH			5000
#define ESD_SELFCAP_DELTA_LIMIT_LOW			-2000

#define ESD_SELFCAP_REF_LIMIT_HIGH			2800   /* TBD */
#define ESD_SELFCAP_REF_LIMIT_LOW			-3500  /* TBD */

#define ESD_CHECK_MUTUALCAP_REF_ID			0x0
#define ESD_CHECK_MUTUALCAP_DELTA_ID			0x1
#define ESD_CHECK_SELFCAP_REF_ID			0x2
#define ESD_CHECK_SELFCAP_DELTA_ID			0x3

#define BACK_COVER_GLASS			0x00
#define BACK_COVER_SAPPHIRE			0x11
#define BACK_COVER_PLASTIC			0x31
#define BACK_COVER_CERAMIC			0x32

#define VENDOR_KOTL					0x31
#define VENDOR_LENS					0x32

/* Configuration file */
#define MXT_CFG_MAGIC				"OBP_RAW V1"

/* Registers */
#define MXT_OBJECT_START			0x07

#define MXT_OBJECT_SIZE				6
#define MXT_INFO_CHECKSUM_SIZE			3
#define MXT_MAX_BLOCK_WRITE			255
#define MXT_MAX_BLOCK_READ			255

/* Object types */
#define MXT_DEBUG_DIAGNOSTIC_T37		37
#define MXT_GEN_MESSAGE_T5			5
#define MXT_GEN_COMMAND_T6			6
#define MXT_GEN_POWER_T7			7
#define MXT_GEN_ACQUIRE_T8			8
#define MXT_GEN_DATASOURCE_T53			53
#define MXT_TOUCH_MULTI_T9			9
#define MXT_TOUCH_KEYARRAY_T15			15
#define MXT_TOUCH_PROXIMITY_T23			23
#define MXT_TOUCH_PROXKEY_T52			52
#define MXT_PROCI_GRIPFACE_T20			20
#define MXT_PROCG_NOISE_T22			22
#define MXT_PROCI_ONETOUCH_T24			24
#define MXT_PROCI_TWOTOUCH_T27			27
#define MXT_PROCI_GRIP_T40			40
#define MXT_PROCI_PALM_T41			41
#define MXT_PROCI_TOUCHSUPPRESSION_T42		42
#define MXT_PROCI_STYLUS_T47			47
#define MXT_PROCG_NOISESUPPRESSION_T48		48
#define MXT_SPT_COMMSCONFIG_T18			18
#define MXT_USER_DATA_T38			38
#define MXT_SPT_GPIOPWM_T19			19
#define MXT_TOUCH_SEQUENCE_PROCESSOR_T24	24
#define MXT_SPT_SELFTEST_T25			25
#define MXT_SPT_CTECONFIG_T28			28
#define MXT_SPT_USERDATA_T38			38
#define MXT_SPT_DIGITIZER_T43			43
#define MXT_SPT_MESSAGECOUNT_T44		44
#define MXT_SPT_CTECONFIG_T46			46
#define MXT_PROCI_ACTIVE_STYLUS_T63		63
#define MXT_OBJ_NOISE_T72               	72
#define	MXT_DYNAMIC_T70               	70
#define MXT_UNLOCK_GESTURE_T81			81
#define MXT_TOUCH_SEQUENCE_PROCESSOR_T93	93
#define MXT_TOUCH_MULTITOUCHSCREEN_T100		100
#define MXT_TOUCH_SELF_CAPACITANCE_CONFIG_T111	111
#define MXT_TOUCH_MULTITOUCHSCREEN_T61		61
#define MXT_SYMBOL_GESTURE_PROCESSOR_T115	115

/* MXT_GEN_MESSAGE_T5 object */
#define MXT_RPTID_NOMSG				0xff

/* MXT_GEN_COMMAND_T6 field */
#define MXT_COMMAND_RESET			0
#define MXT_COMMAND_BACKUPNV			1
#define MXT_COMMAND_CALIBRATE			2
#define MXT_COMMAND_REPORTALL			3
#define MXT_COMMAND_DIAGNOSTIC			5

#define MXT_T6_CMD_PAGE_UP			0x01
#define MXT_T6_CMD_PAGE_DOWN			0x02
#define MXT_T6_CMD_DELTAS			0x10
#define MXT_T6_CMD_REFS				0x11
#define MXT_T6_CMD_SELFCAP_DELTAS		0xF7
#define MXT_T6_CMD_SELFCAP_REFS			0xF8
#define MXT_T6_CMD_DEVICE_ID			0x80
#define MXT_T6_CMD_TOUCH_THRESH			0xF4


/* Define for T6 status byte */
#define MXT_T6_STATUS_RESET			(1 << 7)
#define MXT_T6_STATUS_OFL			(1 << 6)
#define MXT_T6_STATUS_SIGERR			(1 << 5)
#define MXT_T6_STATUS_CAL			(1 << 4)
#define MXT_T6_STATUS_CFGERR			(1 << 3)
#define MXT_T6_STATUS_COMSERR			(1 << 2)

#define THRESHOLD_MAX 27500
#define THRESHOLD_MIN 19000

#define MXT_POWER_CFG_FULL_RUN			0
#define MXT_POWER_CFG_FULL_DEEPSLEEP		1
#define MXT_POWER_CFG_PARTIAL_DEEPSLEEP		2
#define MXT_POWER_CFG_RESTORE			3

/* MXT_TOUCH_MULTI_T9 field */
#define MXT_T9_ORIENT				9
#define MXT_T9_RANGE				18

/* MXT_TOUCH_MULTI_T9 status */
#define MXT_T9_UNGRIP				(1 << 0)
#define MXT_T9_SUPPRESS				(1 << 1)
#define MXT_T9_AMP				(1 << 2)
#define MXT_T9_VECTOR				(1 << 3)
#define MXT_T9_MOVE				(1 << 4)
#define MXT_T9_RELEASE				(1 << 5)
#define MXT_T9_PRESS				(1 << 6)
#define MXT_T9_DETECT				(1 << 7)

/* MXT_TOUCH_MULTI_T9 orient */
#define MXT_T9_ORIENT_SWITCH			(1 << 0)

/* MXT_SPT_COMMSCONFIG_T18 */
#define MXT_COMMS_CTRL				0
#define MXT_COMMS_CMD				1
#define MXT_COMMS_RETRIGEN      		(1 << 6)

/* Define for MXT_GEN_COMMAND_T6 */
#define MXT_BOOT_VALUE				0xa5
#define MXT_RESET_VALUE				0x01
#define MXT_BACKUP_VALUE			0x55
#define MXT_MUTUAL_REFERENCE_MODE		0x11

/* MXT_DEBUG_DIAGNOSTIC_T37 */
#define MXT_DIAG_PAGE_UP			0x01
#define MXT_DIAG_MUTUAL_DELTA			0x10
#define MXT_DIAG_MUTUAL_REF			0x11
#define MXT_DIAG_SELF_DELTA			0xF7
#define MXT_DIAG_SELF_REF			0xF8
#define MXT_DIAG_PAGE_SIZE			0x80
#define MXT_DIAG_TOTAL_SIZE			0x438
#define MXT_DIAG_SELF_SIZE			0x6C
#define MXT_DIAG_REV_ID				21
#define MXT_LOCKDOWN_OFFSET			4

/* Define for MXT_PROCI_TOUCHSUPPRESSION_T42 */
#define MXT_T42_MSG_TCHSUP			(1 << 0)

/* T47 Stylus */
#define MXT_TOUCH_MAJOR_T47_STYLUS		1

/* T63 Stylus */
#define MXT_T63_STYLUS_PRESS			(1 << 0)
#define MXT_T63_STYLUS_RELEASE			(1 << 1)
#define MXT_T63_STYLUS_MOVE			(1 << 2)
#define MXT_T63_STYLUS_SUPPRESS			(1 << 3)

#define MXT_T63_STYLUS_DETECT			(1 << 4)
#define MXT_T63_STYLUS_TIP			(1 << 5)
#define MXT_T63_STYLUS_ERASER			(1 << 6)
#define MXT_T63_STYLUS_BARREL			(1 << 7)

#define MXT_T63_STYLUS_PRESSURE_MASK		0x3F

/* T100 Multiple Touch Touchscreen */
#define MXT_T100_CTRL				0
#define MXT_T100_CFG1				1
#define MXT_T100_TCHAUX				3
#define MXT_T100_XRANGE				13
#define MXT_T100_YRANGE				24

#define MXT_T100_CFG_SWITCHXY			(1 << 5)

#define MXT_T100_TCHAUX_VECT			(1 << 0)
#define MXT_T100_TCHAUX_AMPL			(1 << 1)
#define MXT_T100_TCHAUX_AREA			(1 << 2)

#define MXT_T100_DETECT				(1 << 7)
#define MXT_T100_TYPE_MASK			0x70
#define MXT_T100_EVENT_MASK			0xF
#define MXT_T100_TYPE_STYLUS			0x20
#define MXT_T100_TYPE_LARGE_TOUCH		0x6

/* Delay times */
#define MXT_BACKUP_TIME				50	/* msec */
//#define MXT_RESET_TIME				200	/* msec */
#define MXT_RESET_TIME				500	/* msec */
#define MXT_RESET_TIMEOUT			3000	/* msec */
#define MXT_CRC_TIMEOUT				1000	/* msec */
#define MXT_FW_RESET_TIME			3000	/* msec */
//#define MXT_FW_CHG_TIMEOUT			300	/* msec */
#define MXT_FW_CHG_TIMEOUT			300	/* msec */
#define MXT_WAKEUP_TIME				25	/* msec */
#define MXT_REGULATOR_DELAY			5	/* msec */
#define MXT_CHG_DELAY	        		100	/* msec */
#define MXT_POWERON_DELAY			150	/* msec */
#define MXT_CALIBRATE_DELAY			100	/* msec */

/* Command to unlock bootloader */
#define MXT_UNLOCK_CMD_MSB			0xaa
#define MXT_UNLOCK_CMD_LSB			0xdc

/* Bootloader mode status */
#define MXT_WAITING_BOOTLOAD_CMD		0xc0	/* valid 7 6 bit only */
#define MXT_WAITING_FRAME_DATA			0x80	/* valid 7 6 bit only */
#define MXT_FRAME_CRC_CHECK			0x02
#define MXT_FRAME_CRC_FAIL			0x03
#define MXT_FRAME_CRC_PASS			0x04
#define MXT_APP_CRC_FAIL			0x40	/* valid 7 8 bit only */
#define MXT_BOOT_STATUS_MASK			0x3f
#define MXT_BOOT_EXTENDED_ID			(1 << 5)
#define MXT_BOOT_ID_MASK			0x1f

/* Touchscreen absolute values */
#define MXT_MAX_AREA				0xff

#define MXT_PIXELS_PER_MM			20

#define DEBUG_MSG_MAX				200

/* UPLOAD FW */
#define MXT_FW_NAME_SIZE			17
#define MXT_CFG_OFFSET_TYPE			1
#define MXT_CFG_OFFSET_INSTANCE			3
#define MXT_CFG_OFFSET_SIZE			5
#define MXT_T100_DISABLE_MASK			0xfd
#define MXT_T100_ENABLE_MASK			0x2
#define MXT_T81_ENABLE_MASK			0x1
#define MXT_T81_DISABLE_MASK			0xfe
#define MXT_T38_INFO_SIZE			10

#ifdef MAX1_WAKEUP_GESTURE_ENABLE
/* GESTURE */
#define MXT_GESTURE_UP				0x56
#define MXT_GESTURE_DOWN			0x55
#define MXT_GESTURE_LEFT			0x57
#define MXT_GESTURE_RIGHT			0x58
#define MXT_GESTURE_DOUBLE_CLICK_MASK		0x02
#define MXT_GESTURE_T24_TAP			0x3
#define MXT_GESTURE_T24_DOUBLE_TAP		0x4

#define RIGHT    				1
#define LEFT     				2
#define DOWN     				4
#define UP       				8
#define DOUBLE   				3
#define T24_TAP					13
#define T24_DOUBLE_TAP			14
#endif

#define HEART_BEAT_INTVAL 	1000		/* timer in chip ms */
#define T61_MAX_INSTANCE_NUM	2
#define MXT_PDS_INFO_T68        68
#define HW_LILY_LOTUS		1
#define HW_MERCURY			0
#define MAX_KEYS_SUPPORTED_IN_DRIVER 6

#define CUST_EINTF_TRIGGER_RISING     0x00000001
#define CUST_EINTF_TRIGGER_FALLING    0x00000002
#define CUST_EINTF_TRIGGER_HIGH       0x00000004
#define CUST_EINTF_TRIGGER_LOW        0x00000008
#define RAW_DATA_SIZE 				(TX_NUM * RX_NUM)

#define ADD_MINUS_PERSENT 			30
#define TX_NUM 					12
#define RX_NUM 					12
#define PAGE 					128

/* MXT_PDS_INFO_T68 */
#define MXT_LOCKDOWN_SIZE			8

#define procify(propname) (&proc_##propname)

#define __PROC_FILE(_name, _mode, _read, _write) {\
	.name = __stringify(_name),\
	.fop = {.read =_read, .write = _write},\
	.mode = _mode,\
}

#define PROC_FILE(_name, _mode, _read, _write) struct proc_file proc_##_name = __PROC_FILE(_name, _mode, _read, _write)

/* Noise level test parameter */
#define NOISE_LEVEL_LIMIT   20
#define NOISE_LEVEL_TIMEOUT 10000

#define DBG_LEVEL				0
/* To store the config info in DTS file */
struct mxt_config_info{
	u8 type;
	u8 family_id;
	u8 variant_id;
	u8 version;
	u8 build;
	u8 config_year;
	u8 config_month;
	u8 config_date;
	u8 *config;
	int config_length;
};

/* The platform data for the Atmel maXTouch touchscreen driver */
struct mxt_platform_data {
	unsigned long irqflags;
	u8 t19_num_keys;
	const unsigned int *t19_keymap;
	const u8 *num_keys;  //len is NUM_KEY_TYPE
	const unsigned int (*keymap)[MAX_KEYS_SUPPORTED_IN_DRIVER];
	int t15_num_keys;
	const unsigned int *t15_keymap;
	unsigned long gpio_reset;
	unsigned long gpio_vdd;
	const char *cfg_kotl_s;
	const char *cfg_kotl_g;
	const char *cfg_lens_s;
	const char *cfg_lens_g;
	const char *lily_lotus_ktol_g;
	const char *fw_version;
	const char *input_name;
	struct mxt_config_info info;
};
enum {
	T15_T97_KEY = 0,
	T19_KEY,
	T24_KEY,
	T42_KEY,
	T61_KEY,
	T81_KEY,
	T92_KEY,
	T93_KEY,
	T99_KEY,
	T115_KEY,
	T116_KEY,
	NUM_KEY_TYPE
};
const static unsigned int mxt_keymap[] = {KEY_APPSELECT, KEY_HOMEPAGE, KEY_BACK};
const static unsigned int mxts_keys[NUM_KEY_TYPE][MAX_KEYS_SUPPORTED_IN_DRIVER] = {
	//T15_T97_KEY
	{KEY_BACK , KEY_HOMEPAGE, KEY_APPSELECT, KEY_WAKEUP, KEY_POWER},
	//T19_KEY,
	{KEY_POWER},
	//T24_KEY,
	{KEY_F8},
	//T42_KEY,
	{KEY_F8},
	//T61_KEY,
	{KEY_F9},
	//T81_KEY,
	{KEY_F8,KEY_F9},
	//T92_KEY,
	{KEY_F8,KEY_F9,KEY_F10,KEY_F11},
	//T93_KEY,
	{KEY_F9},
	//T99_KEY,
	{KEY_F10},
	//T115_KEY,
	{KEY_F11},
	//T116_KEY,
	{KEY_F12}
};

const static u8 mxts_num_keys[NUM_KEY_TYPE] = {
	//T15_T97_KEY
	5,
	//T19_KEY,
	0,
	//T24_KEY,
	1,
	//T42_KEY,
	1,
	//T61_KEY,
	1,
	//T81_KEY,
	2,
	//T92_KEY,
	4,
	//T93_KEY,
	1,
	//T99_KEY,
	1,
	//T115_KEY,
	1,
	//T116_KEY,
	1,
};

struct t7_config {
	u8 idle;
	u8 active;
} __packed;

struct t9_range {
	u16 x;
	u16 y;
} __packed;


enum mxt_fw_update{
	IS_UPDATINGE,
	HAVE_UPDATED,
};

struct proc_file{
	struct file_operations fop;
	unsigned int mode;
	char *name;
};

struct mxt_info {
	u8 family_id;
	u8 variant_id;
	u8 version;
	u8 build;
	u8 matrix_xsize;
	u8 matrix_ysize;
	u8 object_num;
};

struct t81_configuration{
	u8 ctrl;
	u8 distlsbyte;
	u8 distmsbyte;
	u8 startxmin;
	u8 startymin;
	u8 startxsize;
	u8 startysize;
	u8 endxmin;
	u8 endymin;
	u8 endxsize;
	u8 endysize;
	u8 movlimmin;
	u8 movlimmax;
	u8 movhyst;
	u8 maxarea;
	u8 maxnumtch;
	u8 angle;
	u8 unused;
};

struct mxt_object {
	u8 type;
	u16 start_address;
	u8 size_minus_one;
	u8 instances_minus_one;
	u8 num_report_ids;
} __packed;

/* For touch panel compatebility */
enum hw_pattern_type{
	old_pattern,
	new_pattern
};

struct mxt_cfg_version{
	u8 year;
	u8 month;
	u8 date;
};

/* To store the cfg version in the hardware */
struct mxt_cfg_info{
	enum hw_pattern_type type;
	u16 fw_version;
	struct mxt_cfg_version cfg_version;
};

/* Each client has this additional data */
struct mxt_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	char phys[64];		/* device physical location */
	struct mxt_platform_data *pdata;
	enum hw_pattern_type pattern_type;
	struct mxt_cfg_info config_info;
	struct work_struct mxt_work;
	u8 *t38_config;
	struct mxt_object *object_table;
	struct mxt_info *info;
	void *raw_info_block;
	unsigned int irq;
	unsigned int max_x;
	unsigned int max_y;
	bool in_bootloader;
	u16 mem_size;
	u8 t100_aux_ampl;
	u8 t100_aux_area;
	u8 t100_aux_vect;
	struct bin_attribute mem_access_attr;
	bool debug_enabled;
	bool debug_v2_enabled;
	u8 *debug_msg_data;
	u16 debug_msg_count;
	struct bin_attribute debug_msg_attr;
	struct mutex debug_msg_lock;
	struct mutex esd_timer_lock;
	u8 max_reportid;
	u32 config_crc;
	u32 info_crc;
#if defined(CONFIG_MXT_I2C_DMA)
	unsigned short bootloader_addr;
	void *i2c_dma_va;
	dma_addr_t i2c_dma_pa;
#else
	u8 bootloader_addr;
#endif
	struct t7_config t7_cfg;
	u8 *msg_buf;
	u8 t6_status;
	bool update_input;
	u8 last_message_count;
	u8 num_touchids;
	u8 num_stylusids;
	unsigned long t15_keystatus;
	bool use_retrigen_workaround;
	bool use_regulator;
	struct regulator *reg_vdd;
	struct regulator *reg_avdd;
	struct regulator *reg_vdd_io;
	struct regulator *reg_vcc_i2c;
	char *fw_name;
	char *cfg_name;

	/* Cached T8 configuration */
	struct t81_configuration t81_cfg;

	/* Cached parameters from object table */
	u16 T5_address;
	u8 T5_msg_size;
	u8 T6_reportid;
	u16 T6_address;
	u16 T7_address;
	u16 T7_size;
	u8 T9_reportid_min;
	u8 T9_reportid_max;
	u8 T15_reportid_min;
	u8 T15_reportid_max;
	u16 T18_address;
	u8 T19_reportid;
	u8 T24_reportid;
	u16 T38_address;
	u8 T38_size;
	u8 T42_reportid_min;
	u8 T42_reportid_max;
	u16 T44_address;
	u8 T48_reportid;
	u8 T63_reportid_min;
	u8 T63_reportid_max;
	u8 T61_reportid_min;
	u8 T61_reportid_max;
	u16 T70_address;
	u8 T70_reportid_min;
	u8 T70_reportid_max;
	u8 T72_reportid;
	u16 T81_address;
	u8 T81_size;
	u8 T81_reportid_min;
	u8 T81_reportid_max;
	u16 T100_address;
	u16 T61_address;
	u16 T24_address;
	u8 T100_reportid_min;
	u8 T100_reportid_max;

	/* for fw update in bootloader */
	struct completion bl_completion;

	/* for reset handling */
	struct completion reset_completion;

	/* for config update handling */
	struct completion crc_completion;

	/* Indicates whether device is in suspend */
	bool suspended;

	/* Indicates whether device is updating configuration */
	bool updating_config;

	/* Indicates whether device need to enter ambient mode  */
	bool is_ambient_mode;

#if defined(CONFIG_FB_PM)
	struct notifier_block fb_notif;
	struct work_struct fb_notify_work;
#endif
	struct notifier_block ambient_notif;
	u16 raw_data_16[RAW_DATA_SIZE];
	u16 raw_data_avg;

	/* Protect access to the T37 object buffer */
	struct mutex T37_buf_mutex;
	u8 *T37_buf;
	size_t T37_buf_size;
	struct early_suspend early_suspend;

#ifdef MAX1_WAKEUP_GESTURE_ENABLE
	u16 T93_address;
	u8 T93_reportid;
	u16 T115_address;
	u8 T115_reportid;
	bool enable_wakeup_gesture;
	bool double_click_wake_enable;
	bool down_to_up_wake_enable;
	bool up_to_down_wake_enable;
	bool right_to_left_wake_enable;
	bool left_to_right_wake_enable;
	bool T24_tap_wake_enable;
	bool T24_double_tap_wake_enable;
	int detected_gesture;
#endif
	bool staying;
	bool update_force;
	struct mutex bus_access_mutex;

	u8 lockdown_info[MXT_LOCKDOWN_SIZE];
	/* Byte0: TP/ST Maker  	  */
	/* Byte1: Disaplay Maker  */
	/* Byte2: HW Version 	  */
	/* Byte3: Back Cover  	  */
	/* Byte4: Bootloalder version */
	/* Byte5: FW Version   	  */
	/* Byte6: Reserved	  */
	/* Byte7: Reserved	  */
};

static struct mxt_data *mxt_i2c_data;


/* T61 Object */
struct t61_config {
        u8 ctrl;
        u8 cmd;
        u8 mode;
        u16 period;
}__packed;
#endif
