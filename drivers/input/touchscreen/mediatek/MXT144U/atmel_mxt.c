/*
 * Atmel maXTouch Touchscreen driver
 *
 * Copyright (C) 2010 Samsung Electronics Co.Ltd
 * Copyright (C) 2011-2014 Atmel Corporation
 * Copyright (C) 2012 Google, Inc.
 *
 * Author: Joonyoung Shim <jy0922.shim@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/dma-mapping.h>
#include <linux/i2c.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <cust_eint.h>
#include <mach/mt_pm_ldo.h>
#include <linux/kthread.h>
#include <mach/mt_pm_ldo.h>
#include <cust_eint.h>
#include <mach/mt_gpio.h>
#include <mach/mt_reg_base.h>
#include <mach/mt_typedefs.h>
#include <mach/eint.h>
#include <ext_wd_drv.h>
#include <linux/watchdog.h>
#include <mach/mt_pm_ldo.h>
#include <mach/mt_wdt.h>
#include <mach/wd_api.h>
#include <pmic_drv.h>
#include <linux/stringify.h>
#include <asm/uaccess.h>
#include <mach/mt_boot.h>
#include <linux/regulator/consumer.h>
#include <atmel_mxt_tp.h>
#include <linux/fb.h>
#include "tpd.h"
#include "cust_gpio_usage.h"
#include "tpd_custom_mxt1144u.h"

#include <linux/wakelock.h>

#if defined(CONFIG_FB_PM)
#include <linux/notifier.h>
#include <linux/fb.h>
#endif

//#define AMBIENT_IDLE_MODE

enum {
	DISP_POWER_MODE_OFF = 0,
	DISP_POWER_MODE_DOZE = 1,
	DISP_POWER_MODE_NORMAL = 2,
	DISP_POWER_MODE_DOZE_SUSPEND = 3
};

static u64 heart_beat_time_stamp;
static unsigned long palm_time_stamp = 0;
static bool need_power_reboot = false;
static int wakeup_event_mask = 0;
struct wake_lock touch_irq_lock;

/* Variable Declaration */

static u8 *gpDMABuf_va = NULL;
static dma_addr_t gpDMABuf_pa = 0;

extern char* mtkfb_find_lcm_driver(void);
static int tpd_flag = 0;
static int noise_data_report_enable = 0;
static int noise_data_result = 0;

static DECLARE_WAIT_QUEUE_HEAD(waiter);
static struct task_struct *thread = NULL;
static unsigned int g_Multouch;
static struct timer_list ESD_timer;
static bool ESD_timer_running = false;
extern int ambient_mode_notifier_register(struct notifier_block *nb);

/* tmp use for debug */
bool tmp_check_mt_ref = false;
int esd_seflcap_ref_limit_high = ESD_SELFCAP_REF_LIMIT_HIGH;
int esd_seflcap_ref_limit_low = ESD_SELFCAP_REF_LIMIT_LOW;
int back_cover = 0;

#ifdef CONFIG_MXT_T61_HR
/* Init t61_config value */
static int mxt_t61_init(struct mxt_data *data,struct t61_config *cfg)
{
        cfg->ctrl = 0x3;
        cfg->cmd  = 0x1;
        cfg->mode = 0x1;
        cfg->period = HEART_BEAT_INTVAL;
}

static int mxt_t61_deinit(struct mxt_data *data,struct t61_config *cfg)
{
        cfg->ctrl = 0x0;
        cfg->cmd  = 0x0;
        cfg->mode = 0x0;
        cfg->period = 0x0;
}
#endif

/* Function Declaration */

static int mxt_update_fw_and_cfg(void *args);
static void mxt_tpd_suspend(struct early_suspend *h);
static void mxt_tpd_resume(struct early_suspend *h);
static int mxt_suspend(struct device *h);
static int mxt_resume(struct device *h);
#ifdef CONFIG_MXT_T61_HR
static int mxt_set_obj_t61(struct mxt_data *data,struct t61_config *cfg, int instance);
#endif
static void mxt_timer_work(struct work_struct *work);
static int __mxt_write_reg_ext(struct i2c_client *client, u16 addr, u16 reg, u16 len, const void *val, unsigned long flag);
static int __mxt_read_reg_ext(struct i2c_client *client, u16 addr, u16 reg, u16 len, void *val, unsigned long flag);
static int mxt_update_fw(struct mxt_data * data);
static int mxt_parse_cfg_and_load(struct mxt_data *data, struct mxt_config_info *info, bool force);
static int mxt_configure_objects(struct mxt_data *data, const struct firmware *cfg);
static struct mxt_object * mxt_get_object(struct mxt_data *data, u8 type);
static int mxt_register_input_device(struct mxt_data *data);

#if defined(CONFIG_FB_PM)
static void fb_notify_resume_work(struct work_struct *work);
static int fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data);
#endif

static int mxt_process_messages_until_invalid(struct mxt_data *data);



static size_t mxt_obj_size(const struct mxt_object *obj)
{
	return obj->size_minus_one + 1;
}

static size_t mxt_obj_instances(const struct mxt_object *obj)
{
	return obj->instances_minus_one + 1;
}

static bool mxt_object_readable(unsigned int type)
{
	switch (type) {
		case MXT_GEN_COMMAND_T6:
		case MXT_GEN_POWER_T7:
		case MXT_GEN_ACQUIRE_T8:
		case MXT_GEN_DATASOURCE_T53:
		case MXT_TOUCH_MULTI_T9:
		case MXT_TOUCH_KEYARRAY_T15:
		case MXT_TOUCH_PROXIMITY_T23:
		case MXT_TOUCH_PROXKEY_T52:
		case MXT_PROCI_GRIPFACE_T20:
		case MXT_PROCG_NOISE_T22:
		case MXT_PROCI_ONETOUCH_T24:
		case MXT_PROCI_TWOTOUCH_T27:
		case MXT_PROCI_GRIP_T40:
		case MXT_PROCI_PALM_T41:
		case MXT_PROCI_TOUCHSUPPRESSION_T42:
		case MXT_PROCI_STYLUS_T47:
		case MXT_PROCG_NOISESUPPRESSION_T48:
		case MXT_SPT_COMMSCONFIG_T18:
		case MXT_SPT_GPIOPWM_T19:
		case MXT_SPT_SELFTEST_T25:
		case MXT_TOUCH_MULTITOUCHSCREEN_T100:
		case MXT_TOUCH_MULTITOUCHSCREEN_T61:
		case MXT_SPT_CTECONFIG_T28:
		case MXT_SPT_USERDATA_T38:
		case MXT_SPT_DIGITIZER_T43:
		case MXT_SPT_CTECONFIG_T46:
		case MXT_TOUCH_SELF_CAPACITANCE_CONFIG_T111:
		case MXT_OBJ_NOISE_T72:
			return true;
		default:
			return false;
	}
}
static inline void mxt_reinit_completion(struct completion *x)
{
	init_completion(x);
}

static void mxt_dump_message(struct mxt_data *data, u8 *message)
{
	MXT_LOG("mxt MSG: %*ph\n", data->T5_msg_size, message);
}

static void mxt_debug_msg_enable(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;

	if (data->debug_v2_enabled)
		return;

	mutex_lock(&data->debug_msg_lock);

	data->debug_msg_data = kcalloc(DEBUG_MSG_MAX,
			data->T5_msg_size, GFP_KERNEL);
	if (!data->debug_msg_data) {
		dev_err(&data->client->dev, "Failed to allocate buffer\n");
		return;
	}

	data->debug_v2_enabled = true;
	mutex_unlock(&data->debug_msg_lock);

	dev_info(dev, "Enabled message output\n");
}

static int ambient_notifier_callback(struct notifier_block *self,
		unsigned long action, void *data)
{
	struct mxt_data *mxt = container_of(self, struct mxt_data, ambient_notif);

	if (action == DISP_POWER_MODE_DOZE || action == DISP_POWER_MODE_DOZE_SUSPEND)
		mxt->is_ambient_mode = true;

	MXT_LOG("mxt: change is_ambient_mode to %d\n", mxt->is_ambient_mode);
	return 0;
}

static void power_reboot(struct mxt_data *data){
	MXT_LOG("mxt doing power reboot ...\n");
	mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
	mt_set_gpio_out(GPIO61, GPIO_OUT_ZERO);
	mt_set_gpio_out(GPIO62, GPIO_OUT_ZERO);
	msleep(5);
	mt_set_gpio_out(GPIO61, GPIO_OUT_ONE);
	msleep(50);

	mt_set_gpio_out(AT_GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
	msleep(10);
	//hwPowerOn(MT6323_POWER_LDO_VGP1, VOL_3300, "TP");
	mt_set_gpio_out(GPIO62, GPIO_OUT_ONE);
	msleep(50);
	mt_set_gpio_out(AT_GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
	msleep(100);
	mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);

#ifdef CONFIG_MXT_T61_HR
	mxt_set_obj_t61(data, &t61, 0);
#endif

	return;
}

static void mxt_proc_t61_messages(struct mxt_data *data, u8 *msg)
{
	int timer_cnt_value;

	heart_beat_time_stamp = jiffies;

	timer_cnt_value = msg[2]|(msg[3]<<8) ;

	MXT_LOG("mxt t61 message ...\n");

	if ((ESD_timer_running == false) && !data->suspended) {
		ESD_timer_running = true;
	}

	if (timer_cnt_value < (HEART_BEAT_INTVAL-100) || timer_cnt_value > (HEART_BEAT_INTVAL+100)) { /* reboot the chip */
		MXT_LOG("mxt T61 status 0x%x 0x%x, count value:%x\n",
			msg[0],
			msg[1],
			timer_cnt_value);

		need_power_reboot = true;
	}
}

static void mxt_debug_msg_disable(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;

	if (!data->debug_v2_enabled)
		return;

	dev_info(dev, "disabling message output\n");
	data->debug_v2_enabled = false;

	mutex_lock(&data->debug_msg_lock);
	kfree(data->debug_msg_data);
	data->debug_msg_data = NULL;
	data->debug_msg_count = 0;
	mutex_unlock(&data->debug_msg_lock);
	dev_info(dev, "Disabled message output\n");
}

static void mxt_debug_msg_add(struct mxt_data *data, u8 *msg)
{
	struct device *dev = &data->client->dev;

	mutex_lock(&data->debug_msg_lock);

	if (!data->debug_msg_data) {
		dev_err(dev, "No buffer!\n");
		return;
	}

	if (data->debug_msg_count < DEBUG_MSG_MAX) {
		memcpy(data->debug_msg_data +
				data->debug_msg_count * data->T5_msg_size,
				msg,
				data->T5_msg_size);
		data->debug_msg_count++;
	} else {
		MXT_LOG("Discarding %u messages\n", data->debug_msg_count);
		data->debug_msg_count = 0;
	}

	mutex_unlock(&data->debug_msg_lock);

	sysfs_notify(&data->client->dev.kobj, NULL, "debug_notify");
}

static ssize_t mxt_debug_msg_write(struct file *filp, struct kobject *kobj,
		struct bin_attribute *bin_attr, char *buf, loff_t off,
		size_t count)
{
	return -EIO;
}

static ssize_t mxt_debug_msg_read(struct file *filp, struct kobject *kobj,
		struct bin_attribute *bin_attr, char *buf, loff_t off, size_t bytes)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct mxt_data *data = dev_get_drvdata(dev);
	int count;
	size_t bytes_read;

	if (!data->debug_msg_data) {
		dev_err(dev, "No buffer!\n");
		return 0;
	}

	count = bytes / data->T5_msg_size;

	if (count > DEBUG_MSG_MAX)
		count = DEBUG_MSG_MAX;

	mutex_lock(&data->debug_msg_lock);

	if (count > data->debug_msg_count)
		count = data->debug_msg_count;

	bytes_read = count * data->T5_msg_size;

	memcpy(buf, data->debug_msg_data, bytes_read);
	data->debug_msg_count = 0;

	mutex_unlock(&data->debug_msg_lock);

	return bytes_read;
}

static int mxt_debug_msg_init(struct mxt_data *data)
{
	sysfs_bin_attr_init(&data->debug_msg_attr);
	data->debug_msg_attr.attr.name = "debug_msg";
	data->debug_msg_attr.attr.mode = 0666;
	data->debug_msg_attr.read = mxt_debug_msg_read;
	data->debug_msg_attr.write = mxt_debug_msg_write;
	data->debug_msg_attr.size = data->T5_msg_size * DEBUG_MSG_MAX;

	if (sysfs_create_bin_file(&data->client->dev.kobj,
				&data->debug_msg_attr) < 0) {
		dev_err(&data->client->dev, "Failed to create %s\n",
				data->debug_msg_attr.attr.name);
		return -EINVAL;
	}

	return 0;
}

static void mxt_debug_msg_remove(struct mxt_data *data)
{
	if (data->debug_msg_attr.attr.name)
		sysfs_remove_bin_file(&data->client->dev.kobj,
				&data->debug_msg_attr);
}

static inline unsigned long test_flag_8bit(unsigned long mask, const volatile unsigned char *addr)
{
	return ((*addr) & mask) != 0;
}

static inline unsigned long test_flag(unsigned long mask, const volatile unsigned long *addr)
{
	return ((*addr) & mask) != 0;
}

static int mxt_wait_for_completion(struct mxt_data *data,
		struct completion *comp,
		unsigned int timeout_ms)
{
	struct device *dev = &data->client->dev;
	unsigned long timeout = msecs_to_jiffies(timeout_ms);
	long ret;

	ret = wait_for_completion_interruptible_timeout(comp, timeout);
	if (ret < 0) {
		return ret;
	} else if (ret == 0) {
		dev_err(dev, "Wait for completion timed out.\n");
		return -ETIMEDOUT;
	}
	return 0;
}

static int mxt_bootloader_read(struct mxt_data *data, u8 *val, unsigned int count)
{
	struct i2c_client *client = data->client;

	return __mxt_read_reg_ext(client, data->bootloader_addr, 0, count, val, I2C_ACCESS_NO_REG);
}
static int mxt_bootloader_write(struct mxt_data *data, const u8 * const val, unsigned int count)
{
	struct i2c_client *client = data->client;

	return __mxt_write_reg_ext(client, data->bootloader_addr, 0, count, val, I2C_ACCESS_NO_REG);
}

static int mxt_lookup_bootloader_address(struct mxt_data *data, bool retry)
{
	u8 appmode = data->client->addr;
	u8 bootloader;
	u8 family_id = data->info ? data->info->family_id : 0;

	switch (appmode) {
		case 0x4a:
		case 0x4b:
			/* Chips after 1664S use different scheme */
			if (retry || family_id >= 0xa2) {
				bootloader = appmode - 0x24;
				break;
			}
			/* Fall through for normal case */
		case 0x4c:
		case 0x4d:
		case 0x5a:
		case 0x5b:
			bootloader = appmode - 0x26;
			break;
		default:
			dev_err(&data->client->dev,
					"Appmode i2c address 0x%02x not found\n",
					appmode);
			return -EINVAL;
	}

	data->bootloader_addr = bootloader;
#if defined(CONFIG_MXT_I2C_DMA)
	data->bootloader_addr |= I2C_RS_FLAG | I2C_ENEXT_FLAG | I2C_DMA_FLAG;
#endif
	dev_info(&data->client->dev, "mxt: bootloader i2c address 0x%02x \n", bootloader);
	return 0;
}

static int mxt_probe_bootloader(struct mxt_data *data, bool retry)
{
	struct device *dev = &data->client->dev;
	int ret;
	u8 val;
	bool crc_failure;

	ret = mxt_lookup_bootloader_address(data, retry);
	if (ret)
		return ret;

	ret = mxt_bootloader_read(data, &val, 1);
	if (ret)
		return ret;

	/* Check app crc fail mode */
	crc_failure = (val & ~MXT_BOOT_STATUS_MASK) == MXT_APP_CRC_FAIL;

	dev_err(dev, "Detected bootloader, status:%02X%s\n",
			val, crc_failure ? ", APP_CRC_FAIL" : "");

	return 0;
}

static u8 mxt_get_bootloader_version(struct mxt_data *data, u8 val)
{
	struct device *dev = &data->client->dev;
	u8 buf[3];

	if (val & MXT_BOOT_EXTENDED_ID) {
		if (mxt_bootloader_read(data, &buf[0], 3) != 0) {
			dev_err(dev, "%s: i2c failure\n", __func__);
			return val;
		}

		dev_err(dev, "Bootloader ID:%d Version:%d\n", buf[1], buf[2]);

		return buf[0];
	} else {
		dev_err(dev, "Bootloader ID:%d\n", val & MXT_BOOT_ID_MASK);

		return val;
	}
}

static int mxt_check_bootloader(struct mxt_data *data, unsigned int state,
		bool wait)
{
	struct device *dev = &data->client->dev;
	u8 val;
	int ret;

recheck:
	if (wait) {
		/*
		 * In application update mode, the interrupt
		 * line signals state transitions. We must wait for the
		 * CHG assertion before reading the status byte.
		 * Once the status byte has been read, the line is deasserted.
		 */
		mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
		ret = mxt_wait_for_completion(data, &data->bl_completion,
				MXT_FW_CHG_TIMEOUT);
		if (ret) {
			/*
			 * TODO: handle -ERESTARTSYS better by terminating
			 * fw update process before returning to userspace
			 * by writing length 0x000 to device (iff we are in
			 * WAITING_FRAME_DATA state).
			 */
			dev_err(dev, "mxg: Update wait error %d\n", ret);
		}
	}
	ret = mxt_bootloader_read(data, &val, 1);
	if (ret)
		return ret;

	if (state == MXT_WAITING_BOOTLOAD_CMD)
		val = mxt_get_bootloader_version(data, val);

	switch (state) {
		case MXT_WAITING_BOOTLOAD_CMD:
		case MXT_WAITING_FRAME_DATA:
		case MXT_APP_CRC_FAIL:
			val &= ~MXT_BOOT_STATUS_MASK;
			break;
		case MXT_FRAME_CRC_PASS:
			if (val == MXT_FRAME_CRC_CHECK) {
				goto recheck;
			} else if (val == MXT_FRAME_CRC_FAIL) {
				dev_err(dev, "Bootloader CRC fail\n");
				return -EINVAL;
			}
			break;
		default:
			return -EINVAL;
	}

	if (val != state) {
		dev_err(dev, "Invalid bootloader state %02X != %02X\n",
				val, state);
		return -EINVAL;
	}

	return 0;
}

static int mxt_send_bootloader_cmd(struct mxt_data *data, bool unlock)
{
	int ret;
	u8 buf[2];

	if (unlock) {
		buf[0] = MXT_UNLOCK_CMD_LSB;
		buf[1] = MXT_UNLOCK_CMD_MSB;
	} else {
		buf[0] = 0x01;
		buf[1] = 0x01;
	}

	ret = mxt_bootloader_write(data, buf, 2);
	if (ret)
		return ret;

	return 0;
}

static int __mxt_cache_read(struct i2c_client *client,u16 addr,
		u16 reg, u16 len, void *val, u8 *r_cache, u8 *r_cache_pa, u8 *w_cache, u8 *w_cache_pa, unsigned long flag)
{
	struct i2c_msg *msgs;
	int num;

	struct i2c_msg xfer[2];
	char buf[2];
	u16 transferred;
	int retry = 3;
	int ret;

	if (test_flag(I2C_ACCESS_NO_CACHE,&flag)) {
		w_cache = w_cache_pa = buf;
		r_cache = r_cache_pa = val;
	}

	if (test_flag(I2C_ACCESS_NO_REG,&flag)) {
		msgs = &xfer[1];
		num = 1;
	}else{
		w_cache[0] = reg & 0xff;
		w_cache[1] = (reg >> 8) & 0xff;

		msgs = &xfer[0];
		num = ARRAY_SIZE(xfer);
		/* Write register */
		xfer[0].addr = addr;
		xfer[0].flags = 0;
		xfer[0].len = 2;
		xfer[0].buf = w_cache_pa;
	}

	/* Read data */
	xfer[1].addr = addr;
	xfer[1].flags = I2C_M_RD;
	xfer[1].buf = r_cache_pa;

#if defined(CONFIG_MXT_I2C_EXTFLAG)
	xfer[1].ext_flag = xfer[0].ext_flag = client->addr & 0xff00;
	xfer[1].timing = xfer[0].timing = 400;
	dev_dbg(&client->dev, "%s: i2c transfer(r)  (addr %x extflag %x) reg %d len %d\n",
			__func__, client->addr, xfer[0].ext_flag, reg, len);
#endif

	transferred = 0;
	while(transferred < len) {
		if (!test_flag(I2C_ACCESS_NO_REG | I2C_ACCESS_R_REG_FIXED,&flag)) {
			w_cache[0] = (reg +  transferred) & 0xff;
			w_cache[1] = ((reg + transferred) >> 8) & 0xff;
		}

		if (test_flag(I2C_ACCESS_NO_CACHE,&flag))
			xfer[1].buf = r_cache_pa + transferred;
		xfer[1].len = len - transferred;
		if (xfer[1].len > MXT_MAX_BLOCK_READ)
			xfer[1].len = MXT_MAX_BLOCK_READ;
retry_read:
		ret = i2c_transfer(client->adapter, msgs, num);
		if (ret != num) {
			if (retry) {
				dev_dbg(&client->dev, "%s: i2c transfer(r) retry, reg %d\n", __func__, reg);
				msleep(MXT_WAKEUP_TIME);
				retry--;
				goto retry_read;
			} else {
				dev_err(&client->dev, "%s: i2c transfer(r) failed (%d) reg %d len %d transferred %d\n",
						__func__, ret, reg, len, transferred);
				mt_set_gpio_out(AT_GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
				MXT_LOG("mxt: i2c error, do reset.\n");
				msleep(80);
				mt_set_gpio_out(AT_GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
				msleep(20);
				return -EIO;
			}
		}
		if (!test_flag(I2C_ACCESS_NO_CACHE,&flag))
			memcpy(val + transferred, r_cache, xfer[1].len);
		transferred += xfer[1].len;

#if (DBG_LEVEL > 1)
		dev_dbg(&client->dev, "[mxt] i2c transfer(r) reg %d len %d current %d transferred %d\n",
				reg, len, xfer[1].len, transferred);
		print_hex_dump(KERN_DEBUG, "[mxt] r:", DUMP_PREFIX_NONE, 16, 1,
				test_flag(I2C_ACCESS_NO_CACHE,&flag) ? xfer[1].buf : r_cache, xfer[1].len, false);
#endif
	}
	return 0;
}

static int __mxt_cache_write(struct i2c_client *client,u16 addr,
		u16 reg, u16 len, const void *val, u8 *w_cache, u8 *w_cache_pa, unsigned long flag)
{
	struct i2c_msg xfer;
	void *buf = NULL;
	u16 transferred,extend;
	int retry = 3;
	int ret;

	if (test_flag(I2C_ACCESS_NO_REG,&flag)) {
		extend = 0;
		if (test_flag(I2C_ACCESS_NO_CACHE,&flag))
			w_cache = w_cache_pa = (u8 *)val;
	}else {
		extend = 2;
		if (test_flag(I2C_ACCESS_NO_CACHE,&flag)) {
			buf = kmalloc( len + extend, GFP_KERNEL);
			if (!buf)
				return -ENOMEM;
			w_cache = w_cache_pa = buf;
		}

		w_cache[0] = reg & 0xff;
		w_cache[1] = (reg >> 8) & 0xff;
	}

	/* Write register */
	xfer.addr = addr;
	xfer.flags = 0;
	xfer.buf = w_cache_pa;

#if defined(CONFIG_MXT_I2C_EXTFLAG)
	xfer.ext_flag = client->addr & 0xff00;
	xfer.timing = 400;
	dev_dbg(&client->dev, "%s: i2c transfer(w) (addr %x extflag %x) reg %d len %d\n",
			__func__, client->addr , xfer.ext_flag, reg, len);
#endif

	transferred = 0;
	while(transferred < len) {
		xfer.len = len - transferred+ extend;
		if (xfer.len> MXT_MAX_BLOCK_WRITE)
			xfer.len = MXT_MAX_BLOCK_WRITE;

		if (test_flag(I2C_ACCESS_NO_CACHE,&flag) &&
				test_flag(I2C_ACCESS_NO_REG,&flag))
			xfer.buf = w_cache_pa + transferred;
		else
			memcpy(w_cache + extend, val + transferred, xfer.len - extend);

		if (extend) {
			w_cache[0] = (reg +  transferred) & 0xff;
			w_cache[1] = ((reg + transferred) >> 8) & 0xff;
		}

retry_write:
		ret = i2c_transfer(client->adapter, &xfer, 1);
		if (ret != 1) {
			if (retry) {
				dev_dbg(&client->dev, "%s: i2c transfer(w) retry, reg %d\n", __func__, reg);
				msleep(MXT_WAKEUP_TIME);
				retry--;
				goto retry_write;
			} else {
				dev_err(&client->dev, "%s: i2c transfer(w) failed (%d) reg %d len %d transferred %d\n",
						__func__, ret, reg, len, transferred);
				mt_set_gpio_out(AT_GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
				MXT_LOG("mxt: i2c error, do reset.\n");
				msleep(80);
				mt_set_gpio_out(AT_GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
				msleep(20);
				if (buf)
					kfree(buf);
				return -EIO;
			}
		}

		transferred += xfer.len -extend;

#if (DBG_LEVEL > 1)
		dev_dbg(&client->dev, "[mxt] i2c transfer(w) reg %d len %d current %d transferred %d\n",
				reg, len, xfer.len -extend, transferred);
		print_hex_dump(KERN_DEBUG, "[mxt] w:", DUMP_PREFIX_NONE, 16, 1,
				test_flag(I2C_ACCESS_NO_CACHE,&flag) ? xfer.buf : w_cache, xfer.len, false);
#endif
	}

	if (buf)
		kfree(buf);
	return 0;
}

static int __mxt_read_reg_ext(struct i2c_client *client, u16 addr, u16 reg, u16 len, void *val, unsigned long flag)
{
	struct mxt_data *data = i2c_get_clientdata(client);
	u8 *r_cache,*r_cache_pa,*w_cache,*w_cache_pa;
	int ret;

#if defined(CONFIG_MXT_I2C_DMA)
	r_cache = data->i2c_dma_va;
	r_cache_pa = (void *)data->i2c_dma_pa;

	w_cache = data->i2c_dma_va + PAGE_SIZE;
	w_cache_pa = (void *)data->i2c_dma_pa + PAGE_SIZE;
#else
	r_cache_pa = r_cache = NULL;
	w_cache_pa = w_cache = NULL;

	flag |= I2C_ACCESS_NO_CACHE;
#endif

	mutex_lock(&data->bus_access_mutex);
	ret = __mxt_cache_read(client, addr, reg, len, val, r_cache, r_cache_pa, w_cache, w_cache_pa, flag);
	mutex_unlock(&data->bus_access_mutex);

	return ret;
}


static int __mxt_write_reg_ext(struct i2c_client *client, u16 addr, u16 reg, u16 len,
		const void *val, unsigned long flag)
{
	struct mxt_data *data = i2c_get_clientdata(client);

	u8 *w_cache,*w_cache_pa;
	int ret;

#if defined(CONFIG_MXT_I2C_DMA)
	w_cache = data->i2c_dma_va + PAGE_SIZE;
	w_cache_pa = (void *)data->i2c_dma_pa + PAGE_SIZE;

#else
	w_cache_pa = w_cache = NULL;

	flag |= I2C_ACCESS_NO_CACHE;
#endif
	mutex_lock(&data->bus_access_mutex);
	ret = __mxt_cache_write(client, addr, reg, len, val, w_cache, w_cache_pa, flag);
	mutex_unlock(&data->bus_access_mutex);

	return ret;
}

static int __mxt_read_reg(struct i2c_client *client, u16 reg, u16 len, void *val)
{
	return __mxt_read_reg_ext(client, client->addr, reg, len, val, 0);
}

static int __mxt_write_reg(struct i2c_client *client, u16 reg, u16 len, const void *val)
{
	return __mxt_write_reg_ext(client, client->addr, reg, len, val, 0);
}

static inline void mxt_config_ctrl_set(struct mxt_data *mxt, u16 addr, u8 mask)
{
	int error, ctrl;

	error = __mxt_read_reg(mxt->client, addr, 1, &ctrl);
	if (error)
		dev_err(&mxt->client->dev, "%s:%d: i2c sent failed\n",
				__func__, __LINE__);
	ctrl |= mask;
	error = __mxt_write_reg(mxt->client, addr, 1, &ctrl);
	if (error)
		dev_err(&mxt->client->dev, "%s:%d: i2c sent failed\n",
				__func__, __LINE__);
}

static inline void mxt_config_ctrl_clear(struct mxt_data *mxt, u16 addr,
		u8 mask)
{
	int error, ctrl;

	error = __mxt_read_reg(mxt->client, addr, 1, &ctrl);
	if (error)
		dev_err(&mxt->client->dev, "%s:%d: i2c sent failed\n",
				__func__, __LINE__);
	ctrl &= ~mask;
	error = __mxt_write_reg(mxt->client, addr, 1, &ctrl);
	if (error)
		dev_err(&mxt->client->dev, "%s:%d: i2c sent failed\n",
				__func__, __LINE__);
}

static inline void mxt_mask_byte(struct mxt_data *mxt, u16 addr, u8 offset, u8 mask)
{
	int error, value;

	error = __mxt_read_reg(mxt->client, addr + offset, 1, &value);
	if (error)
		dev_err(&mxt->client->dev, "%s:%d: i2c sent failed\n",
				__func__, __LINE__);
	value &= ~mask;
	error = __mxt_write_reg(mxt->client, addr + offset, 1, &value);
	if (error)
		dev_err(&mxt->client->dev, "%s:%d: i2c sent failed\n",
				__func__, __LINE__);
}

static inline void mxt_unmask_byte(struct mxt_data *mxt, u16 addr, u8 offset, u8 mask)
{
	int error, value;

	error = __mxt_read_reg(mxt->client, addr + offset, 1, &value);
	if (error)
		dev_err(&mxt->client->dev, "%s:%d: i2c sent failed\n",
				__func__, __LINE__);
	value |= mask;
	error = __mxt_write_reg(mxt->client, addr + offset, 1, &value);
	if (error)
		dev_err(&mxt->client->dev, "%s:%d: i2c sent failed\n",
				__func__, __LINE__);
}

static int mxt_read_reg(struct i2c_client *client, u16 reg, u16 len, void *val)
{
	return __mxt_read_reg(client, reg, len, val);
}

static int mxt_write_reg(struct i2c_client *client, u16 reg, u8 val)
{
	return __mxt_write_reg(client, reg, 1, &val);
}


static int mxt_read_object(struct mxt_data *data, u8 type, u8 offset, u8 *val)
{
	struct mxt_object *object;
	u16 reg;

	object = mxt_get_object(data, type);
	if (!object)
		return -EINVAL;

	reg = object->start_address;
	if (data->debug_enabled)
		dev_info(&data->client->dev, "read from object %d, reg 0x%02x, val 0x%x\n",
				(int)type, reg + offset, *val);
	return mxt_read_reg(data->client, reg + offset, 1, val);
}

static int mxt_write_object(struct mxt_data *data, u8 type, u8 offset, u8 val)
{
	struct mxt_object *object;
	u16 reg;
	int ret;

	object = mxt_get_object(data, type);
	if (!object)
		return -EINVAL;


	reg = object->start_address;
	if (data->debug_enabled)
		dev_info(&data->client->dev, "write to object %d, reg 0x%02x, val 0x%x\n",
				(int)type, reg + offset, val);
	ret = mxt_write_reg(data->client, reg + offset, val);

	return ret;
}

static int mxt_read_lockdown_info(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;
	struct mxt_object *object;
	int ret, i = 0;
	u8 val;
	u16 reg;

	ret = mxt_write_object(data, MXT_GEN_COMMAND_T6,
				MXT_COMMAND_DIAGNOSTIC, 0x81);
	if (ret) {
		dev_err(dev, "mxt: Failed to send lockdown info read command!\n");
		return ret;
	}

	while (i < 100) {
		ret = mxt_read_object(data, MXT_GEN_COMMAND_T6,
					MXT_COMMAND_DIAGNOSTIC, &val);
		if (ret) {
			dev_err(dev, "mxt: Failed to read diagnostic!\n");
			return ret;
		}

		if (val == 0)
			break;

		i++;
		msleep(10);
	}

	object = mxt_get_object(data, MXT_DEBUG_DIAGNOSTIC_T37);
	if (!object)
		return -EINVAL;

	reg = object->start_address;
	ret = mxt_read_reg(data->client, reg + MXT_LOCKDOWN_OFFSET,
			MXT_LOCKDOWN_SIZE, data->lockdown_info);
	if (ret)
		dev_err(dev, "mxt: Failed to read lockdown info!\n");

	return 0;
}

static int mxt_write_lockdown_info(struct mxt_data *data, unsigned char* buf, int length)
{
	struct device *dev = &data->client->dev;
	struct mxt_object *object;
	unsigned char T68_setdata[71];
	int i, ret = 0;
	u16 reg;

	object = mxt_get_object(data, MXT_PDS_INFO_T68);
	if (!object)
		return -EINVAL;

	reg = object->start_address;

	/* Prepare T68 data buf */
	memset(T68_setdata, 0, sizeof(T68_setdata));
	T68_setdata[0] = 0x03;
	T68_setdata[3] = 0x05;
	T68_setdata[4] = 0x00;
	T68_setdata[5] = 0x08;
	for (i = 0; i < length; i++)
		T68_setdata[6 + i] = buf[i];
	T68_setdata[70] = 0x01;

	ret = __mxt_write_reg(data->client, reg, 71, T68_setdata);
	if (ret) {
		dev_err(dev, "Failed to write T68 start frame\n");
		return ret;
	}

	T68_setdata[5] = 0x00;
	T68_setdata[70] = 0x03;
	ret = __mxt_write_reg(data->client, reg, 71, T68_setdata);
	if (ret) {
		dev_err(dev, "Failed to write T68 end frame\n");
		return ret;
	}

	return ret;
}


#ifdef CONFIG_MXT_T61_HR
/* Write t61_config value into relevant instance */
static int mxt_set_obj_t61(struct mxt_data *data,struct t61_config *cfg, int instance)
{
        struct device *dev = &data->client->dev;
        u16 reg;

        dev_err(dev, "mxt set t61 (0x%x 0x%x 0x%x) period %d\n",
                        cfg->ctrl,
                        cfg->cmd,
                        cfg->mode,
                        cfg->period);

        reg = data->T61_address + instance * sizeof(*cfg);
        __mxt_write_reg(data->client, reg ,sizeof(struct t61_config), cfg);

        return 0;
}
#endif

static struct mxt_object * mxt_get_object(struct mxt_data *data, u8 type)
{
	struct mxt_object *object;
	int i;

	for (i = 0; i < data->info->object_num; i++) {
		object = data->object_table + i;
		if (object->type == type)
			return object;
	}

	dev_warn(&data->client->dev, "Invalid object type T%u\n", type);
	return NULL;
}

static void mxt_proc_t6_messages(struct mxt_data *data, u8 *msg)
{
	u8 status = msg[1];
	u32 crc = msg[2] | (msg[3] << 8) | (msg[4] << 16);

	complete(&data->crc_completion);

	if (crc != data->config_crc) {
		data->config_crc = crc;
		MXT_LOG("T6 Config Checksum: 0x%06X\n", crc);
	}

	/* Detect reset */
	if (status & MXT_T6_STATUS_RESET)
	{
		if(data->staying != IS_UPDATINGE && data->input_dev != NULL) {
			input_report_key(data->input_dev, BTN_TOUCH, 0);
			input_sync(data->input_dev);
		}
		complete(&data->reset_completion);
		//frank
#ifdef CONFIG_MXT_T61_HR
		mxt_t61_init(data, &t61);
		mxt_set_obj_t61(data, &t61, 0);
#endif
	}

	MXT_LOG("mxt: process t6 message: 0x%x\n", status);

	/* Output debug if status has changed */
	if (status != data->t6_status)
		MXT_LOG("T6 Status 0x%02X%s%s%s%s%s%s%s\n",
				status,
				status == 0 ? " OK" : "",
				status & MXT_T6_STATUS_RESET ? " RESET" : "",
				status & MXT_T6_STATUS_OFL ? " OFL" : "",
				status & MXT_T6_STATUS_SIGERR ? " SIGERR" : "",
				status & MXT_T6_STATUS_CAL ? " CAL" : "",
				status & MXT_T6_STATUS_CFGERR ? " CFGERR" : "",
				status & MXT_T6_STATUS_COMSERR ? " COMSERR" : "");

	/* Save current status */
	data->t6_status = status;

}

static void mxt_input_button(struct mxt_data *data, u8 *message)
{
	struct input_dev *input = data->input_dev;
	const struct mxt_platform_data *pdata = data->pdata;
	bool button;
	int i;

	/* Active-low switch */
	for (i = 0; i < pdata->t19_num_keys; i++) {
		if (pdata->t19_keymap[i] == KEY_RESERVED)
			continue;
		button = !(message[1] & (1 << i));
		input_report_key(input, pdata->t19_keymap[i], button);
	}
}

static void mxt_input_sync(struct mxt_data *data)
{
	input_mt_report_pointer_emulation(data->input_dev,
			data->pdata->t19_num_keys);
	input_sync(data->input_dev);
}

static void mxt_proc_t9_message(struct mxt_data *data, u8 *message)
{
	struct input_dev *input_dev = data->input_dev;
	int id;
	u8 status;
	int x;
	int y;
	int area;
	int amplitude;
	u8 vector;
	int tool;

	id = message[0] - data->T9_reportid_min;
	status = message[1];
	x = (message[2] << 4) | ((message[4] >> 4) & 0xf);
	y = (message[3] << 4) | ((message[4] & 0xf));

	/* Handle 10/12 bit switching */
	if (data->max_x < 1024)
		x >>= 2;
	if (data->max_y < 1024)
		y >>= 2;

	area = message[5];

	amplitude = message[6];
	vector = message[7];

	MXT_LOG( "[%u] %c%c%c%c%c%c%c%c x: %5u y: %5u area: %3u amp: %3u vector: %02X\n", id,
			(status & MXT_T9_DETECT) ? 'D' : '.',
			(status & MXT_T9_PRESS) ? 'P' : '.',
			(status & MXT_T9_RELEASE) ? 'R' : '.',
			(status & MXT_T9_MOVE) ? 'M' : '.',
			(status & MXT_T9_VECTOR) ? 'V' : '.',
			(status & MXT_T9_AMP) ? 'A' : '.',
			(status & MXT_T9_SUPPRESS) ? 'S' : '.',
			(status & MXT_T9_UNGRIP) ? 'U' : '.',
			x, y, area, amplitude, vector);

	input_mt_slot(input_dev, id);

	if (status & MXT_T9_DETECT) {
		/*
		 * Multiple bits may be set if the host is slow to read
		 * the status messages, indicating all the events that
		 * have happened.
		 */
		if (status & MXT_T9_RELEASE) {
			input_mt_report_slot_state(input_dev,
					MT_TOOL_FINGER, 0);
			mxt_input_sync(data);
		}

		/* A size of zero indicates touch is from a linked T47 Stylus */
		if (area == 0) {
			area = MXT_TOUCH_MAJOR_T47_STYLUS;
			tool = MT_TOOL_PEN;
		} else {
			tool = MT_TOOL_FINGER;
		}

		/* Touch active */
		input_mt_report_slot_state(input_dev, tool, 1);
		input_report_abs(input_dev, ABS_MT_POSITION_X, x);
		input_report_abs(input_dev, ABS_MT_POSITION_Y, y);
		input_report_abs(input_dev, ABS_MT_PRESSURE, amplitude);
		input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, area);
		input_report_abs(input_dev, ABS_MT_ORIENTATION, vector);
	} else {
		/* Touch no longer active, close out slot */
		input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, 0);
	}

	data->update_input = true;
}

static void mxt_proc_t81_message(struct mxt_data *data, u8 *message)
{
	int id ;
	u8 status;
	int xdelta, ydelta;
	struct input_dev *dev = data->input_dev;

	/* we don't enable this function now 141119 */
	return;

	if (message == NULL)
		return;
	id = message[0];
	status = message[1];
	xdelta = (message[3]<< 8) | message[2];
	ydelta = (message[5]<< 8) | message[4];

	if (xdelta && ydelta) {
		input_report_key(dev, KEY_POWER, 1);
		input_sync(dev);
		input_report_key(dev, KEY_POWER, 0);
		input_sync(dev);
		dev_info(&data->client->dev, "Report a power key event\n");
	}

	dev_info(&data->client->dev, "%s: id: 0x%x, status: 0x%x, xdelta: 0x%x, ydelta: 0x%x\n",
			__func__, id, status, xdelta, ydelta);

	return;
}
static void mxt_proc_t100_message(struct mxt_data *data, u8 *message)
{
	struct input_dev *input_dev = data->input_dev;
	static char pointer[10] = {0};
	int cnt = 0;
	int id;
	int i;
	u8 status;
	int x;
	int y;
	int tool;

	if(data->staying == IS_UPDATINGE)
	{
		MXT_LOG("Updating fw or cfg!\n");
		return;
	}

	id = message[0] - data->T100_reportid_min - 2;

	/* ignore SCRSTATUS events */
	if (id < 0)
		return;
	else
		pointer[id] = 1;

	status = message[1];

	x = (message[3] << 8) | message[2];
	y = (message[5] << 8) | message[4];

	if (data->lockdown_info[2] == HW_LILY_LOTUS && (y < 30 && x < 30)) {
		input_report_key(input_dev, BTN_TOUCH, 0);
		input_sync(input_dev);
		return;
	}

	input_mt_slot(input_dev, id);


	if (status & MXT_T100_DETECT) {
		#if !defined(AMBIENT_IDLE_MODE)
		if (data->is_ambient_mode == true)
			return;
		#endif
		mt_eint_mask(2);    //mask side touch eint
		if ((status & MXT_T100_TYPE_MASK) == MXT_T100_TYPE_STYLUS)
			tool = MT_TOOL_PEN;
		else
			tool = MT_TOOL_FINGER;

		/* Touch active */
		input_mt_report_slot_state(input_dev, tool, 1);
		input_report_abs(input_dev, ABS_MT_POSITION_X, y);
		input_report_abs(input_dev, ABS_MT_POSITION_Y, x);

		if (data->t100_aux_ampl)
			input_report_abs(input_dev, ABS_MT_PRESSURE,
					message[data->t100_aux_ampl]);

		if (data->t100_aux_area) {
			if (tool == MT_TOOL_PEN)
				input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR,
						MXT_TOUCH_MAJOR_T47_STYLUS);
			else
				input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR,
						message[data->t100_aux_area]);
		}

		if (data->t100_aux_vect)
			input_report_abs(input_dev, ABS_MT_ORIENTATION,
					message[data->t100_aux_vect]);
	} else {
		pointer[id] = 0;
		MXT_LOG("T100 status = %d\n", (int)status);

		/* Touch no longer active, close out slot */
		/* If SUP EVENT, don't report BTN_TOUCH UP */
		if((status & MXT_T100_EVENT_MASK) != 0x3) {
			input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, 0);
		}
#if !defined(AMBIENT_IDLE_MODE)
		if (data->is_ambient_mode == true && status == 21 && wakeup_event_mask == 0 ) {
			++wakeup_event_mask;
			input_mt_report_slot_state(input_dev, 0, 1);
			input_report_abs(input_dev, ABS_MT_POSITION_X, 50);
			input_report_abs(input_dev, ABS_MT_POSITION_Y, 50);
			input_sync(input_dev);
			MXT_LOG("T100 send position info to WAKEUP system\n");
		}
#endif
		mt_eint_unmask(2);
	}
	for(i = 0; i < 10; i++)
	{
		cnt += pointer[i];
	}
	g_Multouch = cnt;

	data->update_input = true;

}

static void mxt_proc_t15_messages(struct mxt_data *data, u8 *msg)
{
	struct input_dev *input_dev = data->input_dev;
	int key;
	bool curr_state, new_state;
	bool sync = false;
	u8 num_keys;  //len is NUM_KEY_TYPE
	const unsigned int *keymap;

	unsigned long keystates = le32_to_cpu(msg[2]);
	if(keystates == 3)
	{
		MXT_LOG("mxt invalid t15 message!!, keystates: %lu\n", keystates);
		return;
	}

	num_keys = data->pdata->num_keys[T15_T97_KEY];
	keymap = data->pdata->keymap[T15_T97_KEY];
	for (key = 0; key < 3; key++) {
		curr_state = test_bit(key, &data->t15_keystatus);
		new_state = test_bit(key, &keystates);

		MXT_LOG("mxt t15 key: %d, curr_state: %d, new_state: %d\n", key, curr_state, new_state);
		if (!curr_state && new_state) {
			__set_bit(key, &data->t15_keystatus);

			//__set_bit(key, input_dev->keybit);
			MXT_LOG("mxt keymap: %u\n", keymap[key]);
			input_event(input_dev, EV_KEY, keymap[key], 1);
			MXT_LOG("mxt input_event\n");
			sync = true;
		} else if (curr_state && !new_state) {
			__clear_bit(key, &data->t15_keystatus);

			//__clear_bit(key, input_dev->keybit);
			MXT_LOG("mxt data->t15_keystatus: %lu\n", data->t15_keystatus);
			input_event(input_dev, EV_KEY, keymap[key], 0);
			MXT_LOG("mxt input_event\n");
			sync = true;
		}
	}

	if (sync)
		input_sync(input_dev);
}

static void mxt_proc_t42_messages(struct mxt_data *data, u8 *msg)
{
	struct device *dev = &data->client->dev;
	u8 status = msg[1];

	if (status & MXT_T42_MSG_TCHSUP)
		dev_info(dev, "T42 suppress\n");
	else
		dev_info(dev, "T42 normal\n");
}

static int mxt_proc_t48_messages(struct mxt_data *data, u8 *msg)
{
	u8 status, state;

	status = msg[1];
	state  = msg[4];

	MXT_LOG("T48 state %d status %02X %s%s%s%s%s\n", state, status,
			status & 0x01 ? "FREQCHG " : "",
			status & 0x02 ? "APXCHG " : "",
			status & 0x04 ? "ALGOERR " : "",
			status & 0x10 ? "STATCHG " : "",
			status & 0x20 ? "NLVLCHG " : "");

	return 0;
}

static void mxt_proc_t63_messages(struct mxt_data *data, u8 *msg)
{
	struct device *dev = &data->client->dev;
	struct input_dev *input_dev = data->input_dev;
	u8 id;
	u16 x, y;
	u8 pressure;

	/* stylus slots come after touch slots */
	id = data->num_touchids + (msg[0] - data->T63_reportid_min);

	if (id < 0 || id > (data->num_touchids + data->num_stylusids)) {
		dev_err(dev, "invalid stylus id %d, max slot is %d\n",
				id, data->num_stylusids);
		return;
	}

	x = msg[3] | (msg[4] << 8);
	y = msg[5] | (msg[6] << 8);
	pressure = msg[7] & MXT_T63_STYLUS_PRESSURE_MASK;

	printk(KERN_DEBUG
			"[%d] %c%c%c%c x: %d y: %d pressure: %d stylus:%c%c%c%c\n",
			id,
			msg[1] & MXT_T63_STYLUS_SUPPRESS ? 'S' : '.',
			msg[1] & MXT_T63_STYLUS_MOVE     ? 'M' : '.',
			msg[1] & MXT_T63_STYLUS_RELEASE  ? 'R' : '.',
			msg[1] & MXT_T63_STYLUS_PRESS    ? 'P' : '.',
			x, y, pressure,
			msg[2] & MXT_T63_STYLUS_BARREL   ? 'B' : '.',
			msg[2] & MXT_T63_STYLUS_ERASER   ? 'E' : '.',
			msg[2] & MXT_T63_STYLUS_TIP      ? 'T' : '.',
			msg[2] & MXT_T63_STYLUS_DETECT   ? 'D' : '.');

	input_mt_slot(input_dev, id);

	if (msg[2] & MXT_T63_STYLUS_DETECT) {
		input_mt_report_slot_state(input_dev, MT_TOOL_PEN, 1);
		input_report_abs(input_dev, ABS_MT_POSITION_X, x);
		input_report_abs(input_dev, ABS_MT_POSITION_Y, y);
		input_report_abs(input_dev, ABS_MT_PRESSURE, pressure);
	} else {
		input_mt_report_slot_state(input_dev, MT_TOOL_PEN, 0);
	}

	input_report_key(input_dev, BTN_STYLUS,
			(msg[2] & MXT_T63_STYLUS_ERASER));
	input_report_key(input_dev, BTN_STYLUS2,
			(msg[2] & MXT_T63_STYLUS_BARREL));

	mxt_input_sync(data);
}

static void mxt_proc_t70_messages(struct mxt_data *data, u8 *msg)
{
	struct input_dev *input_dev = data->input_dev;

	/* Bit0: Handled;
  	   Bit1: Eventedge */
	if ((msg[1] & 0x1) && (msg[1] & 0x2)) {
		MXT_LOG("mxt t70 large touch suppressed report power key, status = 0x%x\n", msg[1]);
		palm_time_stamp = jiffies;
		if (data->suspended) {
			MXT_LOG("large touch detected after suspended\n");
			return;
		}
		input_report_key(input_dev, KEY_SLEEP, 1);
		input_sync(input_dev);
		input_report_key(input_dev, KEY_SLEEP, 0);
		input_sync(input_dev);
	}
}

static void mxt_proc_t72_messages(struct mxt_data *data, u8 *msg)
{
	if (msg[1] & MXT_T72_NOISE_SUPPRESSION_NOISELVCHG) {
		MXT_LOG("mxt: T72 noise change, state = %d, peak = %d, level = %d\n", msg[2] & 0x7, msg[4], msg[5]);
		if(noise_data_report_enable && (msg[5] > NOISE_LEVEL_LIMIT)) {
			noise_data_result++;
		}
	}

}

#ifdef MAX1_WAKEUP_GESTURE_ENABLE
/* Add wake up processing for mXT144U */
static void mxt_proc_t24_messages(struct mxt_data *data, u8 *msg)
{
	int gesture_detected = 0;
	struct input_dev *dev = data->input_dev;

	MXT_LOG("mxt t24 Event Type: 0x%x is_ambient_mode=%d\n", msg[1]&0xF, data->is_ambient_mode);

	//In ambient mode, only wakeup screen via T100 message.
	if (msg[0] != data->T24_reportid || data->is_ambient_mode)
		return;

	switch (msg[1]&0xF) {
		case MXT_GESTURE_T24_TAP:
			if (data->T24_tap_wake_enable) {
				data->detected_gesture = T24_TAP;
				gesture_detected = 1;
			}
			break;
		case MXT_GESTURE_T24_DOUBLE_TAP:
			if (data->T24_double_tap_wake_enable) {
				data->detected_gesture = T24_DOUBLE_TAP;
				gesture_detected = 1;
			}
			break;

		default:
			break;
	}

	if (gesture_detected) {
		//filter "report palm, then touch" condition when black screen(tap-wake enable).
		if (jiffies - palm_time_stamp > msecs_to_jiffies(1600)) {
			input_mt_report_slot_state(dev, 0, 1);
			input_report_abs(dev, ABS_MT_POSITION_X, 50);
			input_report_abs(dev, ABS_MT_POSITION_Y, 50);
			input_sync(dev);
			MXT_LOG("T24 send position info to WAKEUP system\n");
		}
	}
}

static void mxt_proc_t93_messages(struct mxt_data *data, u8 *msg)
{
	struct input_dev *dev = data->input_dev;

	if ((msg[0] == data->T93_reportid) &&
			(msg[1] & MXT_GESTURE_DOUBLE_CLICK_MASK)) {
		data->detected_gesture = DOUBLE;
		input_report_key(dev, KEY_POWER, 1);
		input_sync(dev);
		input_report_key(dev, KEY_POWER, 0);
		input_sync(dev);
	}
}

static void mxt_proc_t115_messages(struct mxt_data *data, u8 *msg)
{
	int gesture_detected = 0;
	struct input_dev *dev = data->input_dev;
	if (msg[0] != data->T115_reportid)
		return;

	switch (msg[1]) {
		case MXT_GESTURE_DOWN:
			if (data->up_to_down_wake_enable) {
				data->detected_gesture = DOWN;
				gesture_detected = 1;
			}
			break;
		case MXT_GESTURE_UP:
			if (data->down_to_up_wake_enable) {
				data->detected_gesture = UP;
				gesture_detected = 1;
			}
			break;
		case MXT_GESTURE_LEFT:
			if (data->right_to_left_wake_enable) {
				data->detected_gesture = LEFT;
				gesture_detected = 1;
			}
			break;
		case MXT_GESTURE_RIGHT:
			if (data->left_to_right_wake_enable) {
				data->detected_gesture = RIGHT;
				gesture_detected = 1;
			}
			break;
		default:
			break;
	}

	if (gesture_detected) {
		input_report_key(dev, KEY_POWER, 1);
		input_sync(dev);
		input_report_key(dev, KEY_POWER, 0);
		input_sync(dev);
	}
}
#endif

static int mxt_proc_message(struct mxt_data *data, u8 *message)
{
	u8 report_id = message[0];
	bool dump = data->debug_enabled;

	if (report_id == MXT_RPTID_NOMSG)
		return 0;

	if (report_id == data->T6_reportid) {
		mxt_proc_t6_messages(data, message);
	} else if (report_id >= data->T42_reportid_min
			&& report_id <= data->T42_reportid_max) {
		mxt_proc_t42_messages(data, message);
	} else if (report_id == data->T48_reportid) {
		mxt_proc_t48_messages(data, message);
	} else if (report_id >= data->T9_reportid_min
			&& report_id <= data->T9_reportid_max) {
		mxt_proc_t9_message(data, message);
	} else if (report_id == data->T81_reportid_min) {
		mxt_proc_t81_message(data, message);
	} else if (report_id >= data->T100_reportid_min
			&& report_id <= data->T100_reportid_max) {
		mxt_proc_t100_message(data, message);
	} else if (report_id == data->T19_reportid) {
		mxt_input_button(data, message);
		data->update_input = true;
	} else if (report_id >= data->T63_reportid_min && report_id <= data->T63_reportid_max) {
		mxt_proc_t63_messages(data, message);
	} else if (report_id >= data->T70_reportid_min
			&& report_id <= data->T70_reportid_max) {
		mxt_proc_t70_messages(data, message);
	} else if (report_id == data->T72_reportid){
		mxt_proc_t72_messages(data, message);
	} else if (report_id >= data->T15_reportid_min && report_id <= data->T15_reportid_max) {
		mxt_dump_message(data, message);
		mxt_proc_t15_messages(data, message);
#ifdef MAX1_WAKEUP_GESTURE_ENABLE
	} else if ((report_id == data->T115_reportid) && data->suspended) {
		mxt_proc_t115_messages(data, message);
	} else if (report_id == data->T93_reportid && data->suspended && data->double_click_wake_enable) {
		mxt_proc_t93_messages(data, message);
	} else if (report_id == data->T24_reportid && data->suspended && (data->T24_tap_wake_enable || data->T24_double_tap_wake_enable)) {
		mxt_proc_t24_messages(data, message);
#endif
	} else if (report_id >= data->T61_reportid_min && report_id <= data->T61_reportid_max) {
		mxt_proc_t61_messages(data, message);
	} else if (!data->input_dev|| (data->suspended && !data->enable_wakeup_gesture)) {
		/*
		 * do not report events if input device is not
		 * yet registered or returning from suspend
		 */
		mxt_dump_message(data, message);
	}else {
		dump = true;
		//MXT_LOG("mxt: not valid reportid 0x%x\n", report_id);
	}

	if (dump) {
		//mxt_dump_message(data, message);
	}
	if (data->debug_v2_enabled)
		mxt_debug_msg_add(data, message);

	return 1;
}

static int mxt_read_and_process_messages(struct mxt_data *data, u8 count)
{
	struct device *dev = &data->client->dev;
	int ret;
	int i;
	u8 num_valid = 0;

	/* Safety check for msg_buf */
	if (count > data->max_reportid)
		return -EINVAL;

	/* Process remaining messages if necessary */
	ret = __mxt_read_reg(data->client, data->T5_address,
			data->T5_msg_size * count, data->msg_buf);
	if (ret) {
		dev_err(dev, "Failed to read %u messages (%d)\n", count, ret);
		return ret;
	}

	for (i = 0;  i < count; i++) {
		ret = mxt_proc_message(data,
				data->msg_buf + data->T5_msg_size * i);

		if (ret == 1)
			num_valid++;
	}

	/* return number of messages read */
	return num_valid;
}

static irqreturn_t mxt_process_messages_t44(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;
	int ret;
	u8 count, num_left;

	/* Read T44 and T5 together */
	ret = __mxt_read_reg(data->client, data->T44_address,
			data->T5_msg_size + 1, data->msg_buf);
	if (ret) {
		dev_err(dev, "mxt: Failed to read T44 and T5 (%d)\n", ret);
		return IRQ_NONE;
	}

	count = data->msg_buf[0];

	if (count == 0) {
		/* MXT_LOG("Interrupt triggered but zero messages\n"); */
		return IRQ_NONE;
	} else if (count > data->max_reportid) {
		MXT_LOG("T44 count %d exceeded max report id\n", count);
		count = data->max_reportid;
	}

	/* Process first message */
	ret = mxt_proc_message(data, data->msg_buf + 1);
	if (ret < 0) {
		MXT_LOG("mxt:Unexpected invalid message\n");
		return IRQ_NONE;
	}

	num_left = count - 1;

	/* Process remaining messages if necessary */
	if (num_left) {
		//MXT_LOG("mxt_process_messages_t44 going to handle left 0x%x messages\n", num_left);
		ret = mxt_read_and_process_messages(data, num_left);
		if (ret < 0){
			MXT_LOG("mxt_process_messages_t44 handle left messages error\n");
			goto end;
		}
		else if (ret != num_left)
			MXT_LOG("mxt: Unexpected invalid message in left messages\n");
	}

end:
	if (data->update_input) {
		mxt_input_sync(data);
		data->update_input = false;
	}

	return IRQ_HANDLED;
}

static int mxt_process_messages_until_invalid(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;
	int count, read;
	u8 tries = 2;

	count = data->max_reportid;

	/* Read messages until we force an invalid */
	do {
		read = mxt_read_and_process_messages(data, count);
		if (read < count)
			return 0;
	} while (--tries);

	if (data->update_input) {
		mxt_input_sync(data);
		data->update_input = false;
	}

	dev_err(dev, "CHG pin isn't cleared\n");
	return -EBUSY;
}

static irqreturn_t mxt_process_messages(struct mxt_data *data)
{
	u8 count = data->last_message_count;
	int total_handled = 0;
	int num_handled = 0;
	if (count < 1 || count > data->max_reportid)
		count = 1;

	/* include final invalid message */
	total_handled = mxt_read_and_process_messages(data, count + 1);
	if (total_handled < 0)
		return IRQ_NONE;
	/* if there were invalid messages, then we are done */
	else if (total_handled <= count)
		goto update_count;

	/* keep reading two msgs until one is invalid or reportid limit */
	do {
		num_handled = mxt_read_and_process_messages(data, 2);
		if (num_handled < 0)
			return IRQ_NONE;

		total_handled += num_handled;

		if (num_handled < 2)
			break;
	} while (total_handled < data->num_touchids);

update_count:
	data->last_message_count = total_handled;

	if (data->update_input) {
		mxt_input_sync(data);
		data->update_input = false;
	}

	return IRQ_HANDLED;
}


static void mxt_interrupt(void)
{
	mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);

	wake_lock(&touch_irq_lock);

	tpd_flag=1;

	wake_up_interruptible(&waiter);

	return;
}

static int touch_event_handler(void *pdata)
{
	struct mxt_data *data = pdata;
	//struct sched_param param = { .sched_priority = RTPM_PRIO_TPD };
	do{
		set_current_state(TASK_INTERRUPTIBLE);

		wait_event_interruptible(waiter, tpd_flag != 0);

		tpd_flag = 0;

		set_current_state(TASK_RUNNING);

		if (data->in_bootloader) {
			/* bootloader state transition completion */
			//MXT_LOG("<--- bl_completion ");
			complete(&data->bl_completion);
			//MXT_LOG("bl\n");
			//mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
		}

		if (!data->object_table) {
			//mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
		}
		if(!data->in_bootloader)
		{
			if (data->T44_address) {
				mxt_process_messages_t44(data);
			} else {
				mxt_process_messages(data);
			}
			//mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
		}
		wake_unlock(&touch_irq_lock);
		mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
	}while(!kthread_should_stop());
	return 0;
}

static int mxt_t6_command(struct mxt_data *data, u16 cmd_offset, u8 value, bool wait)
{
	u16 reg;
	u8 command_register;
	int timeout_counter = 0;
	int ret;

	reg = data->T6_address + cmd_offset;

	ret = mxt_write_reg(data->client, reg, value);
	if (ret) {
		dev_err(&data->client->dev, "%s: Reg writing failed!\n",
				__func__);
		return ret;
	}

	if (!wait)
		return 0;

	do {
		msleep(20);
		ret = __mxt_read_reg(data->client, reg, 1, &command_register);
		if (ret)
			return ret;
	} while (command_register != 0 && timeout_counter++ <= 100);

	if (timeout_counter > 100) {
		dev_err(&data->client->dev, "%s: Command failed!\n", __func__);
		return -EIO;
	}

	return 0;
}

static int mxt_soft_reset(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;
	int ret = 0;

	dev_info(dev, "Resetting chip\n");

	MXT_LOG("mxt: resetting chip\n");

	mxt_reinit_completion(&data->reset_completion);

	ret = mxt_t6_command(data, MXT_COMMAND_RESET, MXT_RESET_VALUE, false);
	if (ret)
		return ret;

	ret = mxt_wait_for_completion(data, &data->reset_completion,
			MXT_RESET_TIMEOUT);
	if (ret)
		return ret;

	return 0;
}

static void mxt_update_crc(struct mxt_data *data, u8 cmd, u8 value)
{
	/*
	 * On failure, CRC is set to 0 and config will always be
	 * downloaded.
	 */
	data->config_crc = 0;
	mxt_reinit_completion(&data->crc_completion);

	mxt_t6_command(data, cmd, value, true);

	/*
	 * Wait for crc message. On failure, CRC is set to 0 and config will
	 * always be downloaded.
	 */
	mxt_wait_for_completion(data, &data->crc_completion, MXT_CRC_TIMEOUT);
}

static void mxt_calc_crc24(u32 *crc, u8 firstbyte, u8 secondbyte)
{
	static const unsigned int crcpoly = 0x80001B;
	u32 result;
	u32 data_word;

	data_word = (secondbyte << 8) | firstbyte;
	result = ((*crc << 1) ^ data_word);

	if (result & 0x1000000)
		result ^= crcpoly;

	*crc = result;
}

static u32 mxt_calculate_crc(u8 *base, off_t start_off, off_t end_off)
{
	u32 crc = 0;
	u8 *ptr = base + start_off;
	u8 *last_val = base + end_off - 1;

	if (end_off < start_off)
		return -EINVAL;

	while (ptr < last_val) {
		mxt_calc_crc24(&crc, *ptr, *(ptr + 1));
		ptr += 2;
	}

	/* if len is odd, fill the last byte with 0 */
	if (ptr == last_val)
		mxt_calc_crc24(&crc, *ptr, 0);

	/* Mask to 24-bit */
	crc &= 0x00FFFFFF;

	return crc;
}

static int mxt_check_retrigen(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	int error;
	int val;

	//if (irq_get_trigger_type(6) & IRQF_TRIGGER_LOW)
	if(1)
		return 0;

	if (data->T18_address) {
		error = __mxt_read_reg(client,
				data->T18_address + MXT_COMMS_CTRL,
				1, &val);
		if (error)
			return error;

		if (val & MXT_COMMS_RETRIGEN)
			return 0;
	}

	dev_warn(&client->dev, "Enabling RETRIGEN workaround\n");
	data->use_retrigen_workaround = true;
	return 0;
}

static int mxt_init_t7_power_cfg(struct mxt_data *data);

/*
 * mxt_update_cfg - download configuration to chip
 *
 * Atmel Raw Config File Format
 *
 * The first four lines of the raw config file contain:
 *  1) Version
 *  2) Chip ID Information (first 7 bytes of device memory)
 *  3) Chip Information Block 24-bit CRC Checksum
 *  4) Chip Configuration 24-bit CRC Checksum
 *
 * The rest of the file consists of one line per object instance:
 *   <TYPE> <INSTANCE> <SIZE> <CONTENTS>
 *
 *   <TYPE> - 2-byte object type as hex
 *   <INSTANCE> - 2-byte object instance number as hex
 *   <SIZE> - 2-byte object size as hex
 *   <CONTENTS> - array of <SIZE> 1-byte hex values
 */
static int mxt_update_cfg(struct mxt_data *data, const struct firmware *cfg)
{
	struct device *dev = &data->client->dev;
	struct mxt_info cfg_info;
	struct mxt_object *object;
	int ret;
	int offset;
	int data_pos;
	int byte_offset;
	int i;
	int cfg_start_ofs;
	u32 info_crc, config_crc, calculated_crc;
	u8 *config_mem;
	size_t config_mem_size;
	unsigned int type, instance, size;
	u8 val;
	u16 reg;

	mxt_update_crc(data, MXT_COMMAND_REPORTALL, 1);

	if (strncmp(cfg->data, MXT_CFG_MAGIC, strlen(MXT_CFG_MAGIC))) {
		dev_err(dev, "Unrecognised config file\n");
		ret = -EINVAL;
		goto release;
	}

	data_pos = strlen(MXT_CFG_MAGIC);

	/* Load information block and check */
	for (i = 0; i < sizeof(struct mxt_info); i++) {
		ret = sscanf(cfg->data + data_pos, "%hhx%n",
				(unsigned char *)&cfg_info + i,
				&offset);
		if (ret != 1) {
			dev_err(dev, "Bad format\n");
			ret = -EINVAL;
			goto release;
		}

		data_pos += offset;
	}

	if (cfg_info.family_id != data->info->family_id) {
		dev_err(dev, "Family ID mismatch!\n");
		ret = -EINVAL;
		goto release;
	}

	if (cfg_info.variant_id != data->info->variant_id) {
		dev_err(dev, "Variant ID mismatch!\n");
		ret = -EINVAL;
		goto release;
	}

	/* Read CRCs */
	ret = sscanf(cfg->data + data_pos, "%x%n", &info_crc, &offset);
	if (ret != 1) {
		dev_err(dev, "Bad format: failed to parse Info CRC\n");
		ret = -EINVAL;
		goto release;
	}
	data_pos += offset;

	ret = sscanf(cfg->data + data_pos, "%x%n", &config_crc, &offset);
	if (ret != 1) {
		dev_err(dev, "Bad format: failed to parse Config CRC\n");
		ret = -EINVAL;
		goto release;
	}
	data_pos += offset;

	/*
	 * The Info Block CRC is calculated over mxt_info and the object
	 * table. If it does not match then we are trying to load the
	 * configuration from a different chip or firmware version, so
	 * the configuration CRC is invalid anyway.
	 */
	if (info_crc == data->info_crc) {
		if (config_crc == 0 || data->config_crc == 0) {
			dev_info(dev, "CRC zero, attempting to apply config\n");
		} else if (config_crc == data->config_crc) {
			dev_info(dev, "Config CRC 0x%06X: OK\n",
					data->config_crc);
			ret = 0;
			goto release;
		} else {
			dev_info(dev, "Config CRC 0x%06X: does not match file 0x%06X\n",
					data->config_crc, config_crc);
		}
	} else {
		dev_warn(dev,
				"Warning: Info CRC error - device=0x%06X file=0x%06X\n",
				data->info_crc, info_crc);
	}

	/* Malloc memory to store configuration */
	cfg_start_ofs = MXT_OBJECT_START +
		data->info->object_num * sizeof(struct mxt_object) +
		MXT_INFO_CHECKSUM_SIZE;
	config_mem_size = data->mem_size - cfg_start_ofs;
	config_mem = kzalloc(config_mem_size, GFP_KERNEL);
	if (!config_mem) {
		dev_err(dev, "Failed to allocate memory\n");
		ret = -ENOMEM;
		goto release;
	}

	while (data_pos < cfg->size) {
		/* Read type, instance, length */
		ret = sscanf(cfg->data + data_pos, "%x %x %x%n",
				&type, &instance, &size, &offset);
		if (ret == 0) {
			/* EOF */
			break;
		} else if (ret != 3) {
			dev_err(dev, "Bad format: failed to parse object\n");
			ret = -EINVAL;
			goto release_mem;
		}
		data_pos += offset;

		object = mxt_get_object(data, type);
		if (!object) {
			/* Skip object */
			for (i = 0; i < size; i++) {
				ret = sscanf(cfg->data + data_pos, "%hhx%n",
						&val,
						&offset);
				data_pos += offset;
			}
			continue;
		}

		if (size > mxt_obj_size(object)) {
			/*
			 * Either we are in fallback mode due to wrong
			 * config or config from a later fw version,
			 * or the file is corrupt or hand-edited.
			 */
			dev_warn(dev, "Discarding %zu byte(s) in T%u\n",
					size - mxt_obj_size(object), type);
		} else if (mxt_obj_size(object) > size) {
			/*
			 * If firmware is upgraded, new bytes may be added to
			 * end of objects. It is generally forward compatible
			 * to zero these bytes - previous behaviour will be
			 * retained. However this does invalidate the CRC and
			 * will force fallback mode until the configuration is
			 * updated. We warn here but do nothing else - the
			 * malloc has zeroed the entire configuration.
			 */
			dev_warn(dev, "Zeroing %zu byte(s) in T%d\n",
					mxt_obj_size(object) - size, type);
		}

		if (instance >= mxt_obj_instances(object)) {
			dev_err(dev, "Object instances exceeded!\n");
			ret = -EINVAL;
			goto release_mem;
		}

		reg = object->start_address + mxt_obj_size(object) * instance;

		for (i = 0; i < size; i++) {
			ret = sscanf(cfg->data + data_pos, "%hhx%n",
					&val,
					&offset);
			if (ret != 1) {
				dev_err(dev, "Bad format in T%d\n", type);
				ret = -EINVAL;
				goto release_mem;
			}
			data_pos += offset;

			if (i > mxt_obj_size(object))
				continue;

			byte_offset = reg + i - cfg_start_ofs;

			if ((byte_offset >= 0)
					&& (byte_offset <= config_mem_size)) {
				*(config_mem + byte_offset) = val;
			} else {
				dev_err(dev, "Bad object: reg:%d, T%d, ofs=%d\n",
						reg, object->type, byte_offset);
				ret = -EINVAL;
				goto release_mem;
			}
		}

		if (type == MXT_USER_DATA_T38 && data->t38_config) {
			memcpy(config_mem + reg - cfg_start_ofs + MXT_T38_INFO_SIZE, data->t38_config + MXT_T38_INFO_SIZE,
					data->T38_size - MXT_T38_INFO_SIZE);
		}
	}

	/* Calculate crc of the received configs (not the raw config file) */
	if (data->T7_address < cfg_start_ofs) {
		dev_err(dev, "Bad T7 address, T7addr = %x, config offset %x\n",
				data->T7_address, cfg_start_ofs);
		ret = 0;
		goto release_mem;
	}

	calculated_crc = mxt_calculate_crc(config_mem,
			data->T7_address - cfg_start_ofs,
			config_mem_size);

	if (config_crc > 0 && (config_crc != calculated_crc))
		dev_warn(dev, "Config CRC error, calculated=%06X, file=%06X\n",
				calculated_crc, config_crc);

	/* Write configuration as blocks */
	byte_offset = 0;
	while (byte_offset < config_mem_size) {
		size = config_mem_size - byte_offset;

		if (size > MXT_MAX_BLOCK_WRITE)
			size = MXT_MAX_BLOCK_WRITE;

		ret = __mxt_write_reg(data->client,
				cfg_start_ofs + byte_offset,
				size, config_mem + byte_offset);
		if (ret != 0) {
			dev_err(dev, "Config write error, ret=%d\n", ret);
			goto release_mem;
		}

		byte_offset += size;
	}

	mxt_update_crc(data, MXT_COMMAND_BACKUPNV, MXT_BACKUP_VALUE);

	ret = mxt_check_retrigen(data);
	if (ret)
		goto release_mem;

	ret = mxt_soft_reset(data);
	if (ret)
		goto release_mem;

	dev_info(dev, "Config successfully updated\n");

	/* T7 config may have changed */
	mxt_init_t7_power_cfg(data);

release_mem:
	kfree(config_mem);
release:
	release_firmware(cfg);
	return ret;
}

static int mxt_set_t7_power_cfg(struct mxt_data *data, u8 state, bool backup)
{
	int error;
	struct t7_config *new_config;
	struct t7_config full_deepsleep = { .active = 0, .idle = 0};
	struct t7_config partial_deepsleep = { .active = 48, .idle = 48 };
	struct t7_config full_active_mode = { .active = 255, .idle = 255 };

	if(backup){
		error = __mxt_read_reg(data->client, data->T7_address,
				sizeof(data->t7_cfg), &data->t7_cfg);
		if (error)
			return error;
	}

	if (state == MXT_POWER_CFG_FULL_DEEPSLEEP)
		new_config = &full_deepsleep;
	else if(state == MXT_POWER_CFG_PARTIAL_DEEPSLEEP)
		new_config = &partial_deepsleep;
	else if(state == MXT_POWER_CFG_FULL_RUN)
		new_config = &full_active_mode;
	else if(state == MXT_POWER_CFG_RESTORE)
		new_config = &data->t7_cfg;
	else
		new_config = &full_active_mode;

	error = __mxt_write_reg(data->client, data->T7_address,
			sizeof(data->t7_cfg), new_config);
	if (error)
		return error;

	MXT_LOG("mxt: Set T7 ACTV:%d IDLE:%d\n",
			new_config->active, new_config->idle);

	return 0;
}

static int mxt_init_t7_power_cfg(struct mxt_data *data)
{
	int error;
	bool retry = false;

recheck:
	error = __mxt_read_reg(data->client, data->T7_address,
			sizeof(data->t7_cfg), &data->t7_cfg);
	if (error)
		return error;

	if (data->t7_cfg.active == 0 || data->t7_cfg.idle == 0) {
		if (!retry) {
			MXT_LOG("T7 cfg zero, resetting\n");
			mxt_soft_reset(data);
			retry = true;
			goto recheck;
		} else {
			MXT_LOG("T7 cfg zero after reset, overriding\n");
			data->t7_cfg.active = 20;
			data->t7_cfg.idle = 100;
			return mxt_set_t7_power_cfg(data, MXT_POWER_CFG_FULL_RUN, false);
		}
	}

	MXT_LOG("Initialized power cfg: ACTV %d, IDLE %d\n",
			data->t7_cfg.active, data->t7_cfg.idle);
	return 0;
}

static int mxt_acquire_irq(struct mxt_data *data)
{
	int error;

	mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);

	if (data->use_retrigen_workaround) {
		error = mxt_process_messages_until_invalid(data);
		if (error)
			return error;
	}

	return 0;
}

static void mxt_free_input_device(struct mxt_data *data)
{
	if (data->input_dev) {
		input_unregister_device(data->input_dev);
		data->input_dev = NULL;
	}
}

static void mxt_free_object_table(struct mxt_data *data)
{
	mxt_debug_msg_remove(data);
	mxt_free_input_device(data);

	data->object_table = NULL;
	data->info = NULL;

	kfree(data->raw_info_block);
	data->raw_info_block = NULL;

	kfree(data->msg_buf);
	data->msg_buf = NULL;

	/* free the t38 configuration mem */
	if (data->t38_config) {
		kfree(data->t38_config);
		data->t38_config = NULL;
	}

	data->T5_address = 0;
	data->T5_msg_size = 0;
	data->T6_reportid = 0;
	data->T7_address = 0;
	data->T9_reportid_min = 0;
	data->T9_reportid_max = 0;
	data->T15_reportid_min = 0;
	data->T15_reportid_max = 0;
	data->T18_address = 0;
	data->T19_reportid = 0;
	data->T24_reportid = 0;
	data->T42_reportid_min = 0;
	data->T42_reportid_max = 0;
	data->T44_address = 0;
	data->T48_reportid = 0;
	data->T63_reportid_min = 0;
	data->T61_reportid_min = 0;
	data->T70_reportid_min = 0;
	data->T70_reportid_max = 0;
	data->T72_reportid = 0;
	data->T63_reportid_max = 0;
	data->T61_reportid_max = 0;
	data->T100_reportid_min = 0;
	data->T100_reportid_max = 0;
	data->max_reportid = 0;
}

static int mxt_parse_object_table(struct mxt_data *data,
		struct mxt_object *object_table)
{
	struct i2c_client *client = data->client;
	int i;
	u8 reportid;
	u16 end_address;

	/* Valid Report IDs start counting from 1 */
	reportid = 1;
	data->mem_size = 0;
	for (i = 0; i < data->info->object_num; i++) {
		struct mxt_object *object = object_table + i;
		u8 min_id, max_id;

		le16_to_cpus(&object->start_address);

		if (object->num_report_ids) {
			min_id = reportid;
			reportid += object->num_report_ids *
				mxt_obj_instances(object);
			max_id = reportid - 1;
		} else {
			min_id = 0;
			max_id = 0;
		}

		MXT_LOG( "mxt: T%u Start:%u Size:%zu Instances:%zu Report IDs:%u-%u\n",
				object->type, object->start_address,
				mxt_obj_size(object), mxt_obj_instances(object),
				min_id, max_id);

		switch (object->type) {
			case MXT_GEN_MESSAGE_T5:
				if (data->info->family_id == 0x80) {
					/*
					 * On mXT224 read and discard unused CRC byte
					 * otherwise DMA reads are misaligned
					 */
					data->T5_msg_size = mxt_obj_size(object);
				} else {
					/* CRC not enabled, so skip last byte */
					data->T5_msg_size = mxt_obj_size(object) - 1;
				}
				data->T5_address = object->start_address;
			case MXT_GEN_COMMAND_T6:
				data->T6_reportid = min_id;
				data->T6_address = object->start_address;
				break;
			case MXT_GEN_POWER_T7:
				data->T7_address = object->start_address;
				data->T7_size = object->size_minus_one + 1;
				break;
			case MXT_TOUCH_MULTI_T9:
				/* Only handle messages from first T9 instance */
				data->T9_reportid_min = min_id;
				data->T9_reportid_max = min_id +
					object->num_report_ids - 1;
				data->num_touchids = object->num_report_ids;
				break;
			case MXT_TOUCH_KEYARRAY_T15:
				data->T15_reportid_min = min_id;
				data->T15_reportid_max = max_id;
				break;
			case MXT_SPT_COMMSCONFIG_T18:
				data->T18_address = object->start_address;
				break;
			case MXT_USER_DATA_T38:
				data->T38_address = object->start_address;
				data->T38_size = object->size_minus_one + 1;
				break;
			case MXT_PROCI_TOUCHSUPPRESSION_T42:
				data->T42_reportid_min = min_id;
				data->T42_reportid_max = max_id;
				break;
			case MXT_SPT_MESSAGECOUNT_T44:
				data->T44_address = object->start_address;
				break;
			case MXT_SPT_GPIOPWM_T19:
				data->T19_reportid = min_id;
				break;
			case MXT_PROCG_NOISESUPPRESSION_T48:
				data->T48_reportid = min_id;
				break;
			case MXT_PROCI_ACTIVE_STYLUS_T63:
				/* Only handle messages from first T63 instance */
				data->T63_reportid_min = min_id;
				data->T63_reportid_max = min_id;
				data->num_stylusids = 1;
			case MXT_DYNAMIC_T70:
				data->T70_address = object->start_address;
				data->T70_reportid_min = min_id;
				data->T70_reportid_max = max_id;
				break;
			case MXT_OBJ_NOISE_T72:
				data->T72_reportid = min_id;
				break;
			case MXT_UNLOCK_GESTURE_T81:
				data->T81_address = object->start_address;
				data->T81_size = object->size_minus_one + 1;
				data->T81_reportid_min = min_id;
				data->T81_reportid_max = min_id;
				break;
			case MXT_TOUCH_MULTITOUCHSCREEN_T100:
				data->T100_address = object->start_address;
				data->T100_reportid_min = min_id;
				data->T100_reportid_max = max_id;
				/* first two report IDs reserved */
				data->num_touchids = object->num_report_ids - 2;
				break;
			case MXT_TOUCH_MULTITOUCHSCREEN_T61:
				data->T61_address = object->start_address;
				data->T61_reportid_min = min_id;
				data->T61_reportid_max = max_id;
				break;
#ifdef MAX1_WAKEUP_GESTURE_ENABLE
			case MXT_TOUCH_SEQUENCE_PROCESSOR_T24:
				data->T24_address = object->start_address;
				data->T24_reportid = min_id;
				break;
			case MXT_TOUCH_SEQUENCE_PROCESSOR_T93:
				data->T93_address = object->start_address;
				data->T93_reportid = min_id;
				break;
			case MXT_SYMBOL_GESTURE_PROCESSOR_T115:
				data->T115_address = object->start_address;
				data->T115_reportid = min_id;
				break;
#endif
		}

		end_address = object->start_address
			+ mxt_obj_size(object) * mxt_obj_instances(object) - 1;

		if (end_address >= data->mem_size)
			data->mem_size = end_address + 1;
	}

	/* Store maximum reportid */
	data->max_reportid = reportid;

	/* If T44 exists, T5 position has to be directly after */
	if (data->T44_address && (data->T5_address != data->T44_address + 1)) {
		dev_err(&client->dev, "Invalid T44 position\n");
		return -EINVAL;
	}

	data->msg_buf = kcalloc(data->max_reportid,
			data->T5_msg_size, GFP_KERNEL);
	if (!data->msg_buf) {
		dev_err(&client->dev, "Failed to allocate message buffer\n");
		return -ENOMEM;
	}

	return 0;
}

static int mxt_read_t38_object(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	struct device *dev = &client->dev;
	struct mxt_cfg_info *config_info = &data->config_info;
	size_t t38_size = data->T38_size;
	u8 *cfg_buf;
	u8 *info;
	int error = 0;

	if (data->t38_config) {
		kfree(data->t38_config);
		data->t38_config = NULL;
	}

	cfg_buf = kzalloc(t38_size, GFP_KERNEL);
	if (!cfg_buf) {
		dev_err(dev, "%s: Do not have enough memory\n", __func__);
		return -ENOMEM;
	}

	error = __mxt_read_reg(client, data->T38_address, t38_size, cfg_buf);
	if (error) {
		dev_err(dev, "%s Failed to read t38 object\n", __func__);
		goto err_free_mem;
	}


	/* store the config info */
	info = (u8 *)cfg_buf;
	config_info->type = info[0];
	config_info->fw_version = info[2] << 8 | info[1];
	config_info->cfg_version.year = info[3];
	config_info->cfg_version.month = info[4];
	config_info->cfg_version.date = info[5];
	data->t38_config = info;

	MXT_LOG("%s: T38 address: 0x%x\n"
			"data: 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x\n",
			__func__, data->T38_address, info[0], info[1],
			info[2], info[3], info[4], info[5]);

	/* store the pattern type info */
	data->pattern_type = info[0];

	return 0;
err_free_mem:
	kfree(data->t38_config);
	data->t38_config = NULL;
	return error;
}

static int mxt_read_info_block(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	int error;
	size_t size, byte_offset, read_size;
	void *id_buf, *buf;
	uint8_t num_objects;
	u32 calculated_crc;
	u8 *crc_ptr;

	/* If info block already allocated, free it */
	if (data->raw_info_block != NULL)
		mxt_free_object_table(data);

	/* Read 7-byte ID information block starting at address 0 */
	size = sizeof(struct mxt_info);
	id_buf = kzalloc(size, GFP_KERNEL);
	if (!id_buf) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}
	error = __mxt_read_reg(client, 0, size, id_buf);
	if (error) {
		kfree(id_buf);
		return error;
	}

	/* Resize buffer to give space for rest of info block */
	num_objects = ((struct mxt_info *)id_buf)->object_num;
	size += (num_objects * sizeof(struct mxt_object))
		+ MXT_INFO_CHECKSUM_SIZE;

	buf = kmalloc(size, GFP_KERNEL);
	if (!buf) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		error = -ENOMEM;
		goto err_free_mem;
	}

	memcpy(buf, id_buf, MXT_OBJECT_START);
	kfree(id_buf);

	/* Read rest of info block */
	byte_offset = MXT_OBJECT_START;
	while (byte_offset < size) {
		if (size - byte_offset > MXT_MAX_BLOCK_READ)
			read_size = MXT_MAX_BLOCK_READ;
		else
			read_size = size - byte_offset;

		error = __mxt_read_reg(client, byte_offset, read_size,
				buf + byte_offset);
		if (error)
			goto err_free_mem;

		byte_offset += read_size;
	}

	/* Extract & calculate checksum */
	crc_ptr = buf + size - MXT_INFO_CHECKSUM_SIZE;
	data->info_crc = crc_ptr[0] | (crc_ptr[1] << 8) | (crc_ptr[2] << 16);

	calculated_crc = mxt_calculate_crc(buf, 0,
			size - MXT_INFO_CHECKSUM_SIZE);

	/*
	 * CRC mismatch can be caused by data corruption due to I2C comms
	 * issue or else device is not using Object Based Protocol (eg i2c-hid)
	 */
	if ((data->info_crc == 0) || (data->info_crc != calculated_crc)) {
		dev_err(&client->dev,
				"Info Block CRC error calculated=0x%06X read=0x%06X\n",
				calculated_crc, data->info_crc);
		error = -EIO;
		goto err_free_mem;
	}

	data->raw_info_block = buf;
	data->info = (struct mxt_info *)buf;

	MXT_LOG("Family: %u Variant: %u Firmware V%u.%u.%02X Objects: %u\n",
			data->info->family_id, data->info->variant_id,
			data->info->version >> 4, data->info->version & 0xf,
			data->info->build, data->info->object_num);

	/* Parse object table information */
	error = mxt_parse_object_table(data, buf + MXT_OBJECT_START);
	if (error) {
		dev_err(&client->dev, "Error %d parsing object table\n", error);
		mxt_free_object_table(data);
		return error;
	}

	data->object_table = (struct mxt_object *)(buf + MXT_OBJECT_START);

	error = mxt_read_t38_object(data);
	if (error) {
		dev_err(&client->dev, "%s: Failed to read t38 object\n",__func__);
		return error;
	}

	return 0;

err_free_mem:
	kfree(buf);
	return error;
}

static void mxt_regulator_enable(struct mxt_data *data)
{
	int error;
	struct device *dev = &data->client->dev;

	gpio_direction_output(data->pdata->gpio_reset, 0);

	error = regulator_enable(data->reg_avdd);
	if (error) {
		dev_err(dev, "regulator_enable failed  reg_avdd\n");
		return;
	}

	msleep(MXT_REGULATOR_DELAY);
	gpio_direction_output(data->pdata->gpio_reset, 1);
	msleep(MXT_CHG_DELAY);

retry_wait:
	mxt_reinit_completion(&data->bl_completion);
	data->in_bootloader = true;
	error = mxt_wait_for_completion(data, &data->bl_completion,
			MXT_POWERON_DELAY);
	if (error == -EINTR)
		goto retry_wait;
	data->in_bootloader = false;
}


static int mxt_read_t9_resolution(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	int error;
	struct t9_range range;
	unsigned char orient;
	struct mxt_object *object;

	object = mxt_get_object(data, MXT_TOUCH_MULTI_T9);
	if (!object)
		return -EINVAL;

	error = __mxt_read_reg(client,
			object->start_address + MXT_T9_RANGE,
			sizeof(range), &range);
	if (error)
		return error;

	le16_to_cpus(&range.x);
	le16_to_cpus(&range.y);

	error =  __mxt_read_reg(client,
			object->start_address + MXT_T9_ORIENT,
			1, &orient);
	if (error)
		return error;

	/* Handle default values */
	if (range.x == 0)
		range.x = 1023;

	if (range.y == 0)
		range.y = 1023;

	if (orient & MXT_T9_ORIENT_SWITCH) {
		data->max_x = range.y;
		data->max_y = range.x;
	} else {
		data->max_x = range.x;
		data->max_y = range.y;
	}

	printk(KERN_DEBUG
			"Touchscreen size X%uY%u\n", data->max_x, data->max_y);

	return 0;
}

static void mxt_start(struct mxt_data *data);
static void mxt_stop(struct mxt_data *data);
static int mxt_input_open(struct input_dev *dev);
static void mxt_input_close(struct input_dev *dev);

static int mxt_initialize_t9_input_device(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;
	const struct mxt_platform_data *pdata = data->pdata;
	struct input_dev *input_dev;
	int error;
	unsigned int num_mt_slots;
	unsigned int mt_flags = 0;
	int i;

	error = mxt_read_t9_resolution(data);
	if (error)
		dev_warn(dev, "Failed to initialize T9 resolution\n");

	input_dev = input_allocate_device();
	if (!input_dev) {
		dev_err(dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}

	input_dev->name = "Atmel maXTouch Touchscreen";
	input_dev->phys = data->phys;
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = dev;
	input_dev->open = mxt_input_open;
	input_dev->close = mxt_input_close;

	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);

	if (pdata->t19_num_keys) {
		__set_bit(INPUT_PROP_BUTTONPAD, input_dev->propbit);

		for (i = 0; i < pdata->t19_num_keys; i++)
			if (pdata->t19_keymap[i] != KEY_RESERVED)
				input_set_capability(input_dev, EV_KEY,
						pdata->t19_keymap[i]);

		mt_flags |= INPUT_MT_POINTER;

		input_abs_set_res(input_dev, ABS_X, MXT_PIXELS_PER_MM);
		input_abs_set_res(input_dev, ABS_Y, MXT_PIXELS_PER_MM);
		input_abs_set_res(input_dev, ABS_MT_POSITION_X,
				MXT_PIXELS_PER_MM);
		input_abs_set_res(input_dev, ABS_MT_POSITION_Y,
				MXT_PIXELS_PER_MM);

		input_dev->name = "Atmel maXTouch Touchpad";
	} else {
		mt_flags |= INPUT_MT_DIRECT;
	}

	/* For single touch */
	input_set_abs_params(input_dev, ABS_X,
			0, data->max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_Y,
			0, data->max_y, 0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE,
			0, 255, 0, 0);

	/* For multi touch */
	num_mt_slots = data->num_touchids + data->num_stylusids;
	error = input_mt_init_slots(input_dev, num_mt_slots, mt_flags);
	if (error) {
		dev_err(dev, "Error %d initialising slots\n", error);
		goto err_free_mem;
	}

	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR,
			0, MXT_MAX_AREA, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X,
			0, data->max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y,
			0, data->max_y, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE,
			0, 255, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_ORIENTATION,
			0, 255, 0, 0);

	/* For T63 active stylus */
	if (data->T63_reportid_min) {
		input_set_capability(input_dev, EV_KEY, BTN_STYLUS);
		input_set_capability(input_dev, EV_KEY, BTN_STYLUS2);
		input_set_abs_params(input_dev, ABS_MT_TOOL_TYPE,
				0, MT_TOOL_MAX, 0, 0);
	}

	/* For T15 key array */
	if (data->T15_reportid_min) {
		data->t15_keystatus = 0;
		for (i = 0; i < data->pdata->t15_num_keys; i++)
			input_set_capability(input_dev, EV_KEY,
					data->pdata->t15_keymap[i]);
	}

	input_set_drvdata(input_dev, data);

	error = input_register_device(input_dev);
	if (error) {
		dev_err(dev, "Error %d registering input device\n", error);
		goto err_free_mem;
	}

	data->input_dev = input_dev;

	return 0;

err_free_mem:
	input_free_device(input_dev);
	return error;
}

static int mxt_read_t100_config(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	int error;
	struct mxt_object *object;
	u16 range_x, range_y;
	u8 cfg, tchaux;
	u8 aux;

	object = mxt_get_object(data, MXT_TOUCH_MULTITOUCHSCREEN_T100);
	if (!object)
		return -EINVAL;

	error = __mxt_read_reg(client,
			object->start_address + MXT_T100_XRANGE,
			sizeof(range_x), &range_x);
	if (error)
		return error;

	le16_to_cpus(&range_x);

	error = __mxt_read_reg(client,
			object->start_address + MXT_T100_YRANGE,
			sizeof(range_y), &range_y);
	if (error)
		return error;

	le16_to_cpus(&range_y);

	error =  __mxt_read_reg(client,
			object->start_address + MXT_T100_CFG1,
			1, &cfg);
	if (error)
		return error;

	error =  __mxt_read_reg(client,
			object->start_address + MXT_T100_TCHAUX,
			1, &tchaux);
	if (error)
		return error;

	/* Handle default values */
	if (range_x == 0)
		range_x = 1023;

	if (range_y == 0)
		range_y = 1023;

	if (cfg & MXT_T100_CFG_SWITCHXY) {
		data->max_x = range_x;
		data->max_y = range_y;
	} else {
		data->max_x = range_y;
		data->max_y = range_x;
	}

	/* allocate aux bytes */
	aux = 6;

	if (tchaux & MXT_T100_TCHAUX_VECT)
		data->t100_aux_vect = aux++;

	if (tchaux & MXT_T100_TCHAUX_AMPL)
		data->t100_aux_ampl = aux++;

	if (tchaux & MXT_T100_TCHAUX_AREA)
		data->t100_aux_area = aux++;

	dev_info(&client->dev,
			"T100 Touchscreen size X%uY%u\n", data->max_x, data->max_y);

	return 0;
}

static int mxt_initialize_t100_input_device(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;
	struct input_dev *input_dev;
	int error;
	int key;
	char num_keys;  //len is NUM_KEY_TYPE
	const unsigned int *keymap;

	error = mxt_read_t100_config(data);
	if (error)
		dev_err(dev, "Failed to initialize T00 resolution\n");

	input_dev = input_allocate_device();
	if (!data || !input_dev) {
		dev_err(dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}

	if (data->pdata->input_name){
		input_dev->name = data->pdata->input_name;
	}
	else{
		input_dev->name = "atmel_mxt_T100";
	}

	mutex_init(&input_dev->mutex);
	input_dev->phys = data->phys;
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &data->client->dev;
	input_dev->open = mxt_input_open;
	input_dev->close = mxt_input_close;

	set_bit(EV_ABS, input_dev->evbit);
	input_set_capability(input_dev, EV_KEY, KEY_POWER);
	input_set_capability(input_dev, EV_KEY, KEY_SLEEP);
	input_set_capability(input_dev, EV_KEY, KEY_WAKEUP);

	__set_bit(BTN_TOUCH, input_dev->keybit);
	__set_bit(BTN_TOOL_FINGER, input_dev->keybit);
#if 1
	num_keys = data->pdata->num_keys[T15_T97_KEY];
	keymap = data->pdata->keymap[T15_T97_KEY];
	for (key = 0; key < num_keys; key++) {
		//MXT_LOG("mxt T15 key press: %u\n", key);

		input_set_capability(input_dev, EV_KEY, keymap[key]);

		//MXT_LOG("mxt keymap[%d]: %d\n", key, keymap[key]);
	}
#endif
	input_set_capability(input_dev, EV_KEY, KEY_POWER);

	/* For single touch */
	input_set_abs_params(input_dev, ABS_X,
			0, data->max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_Y,
			0, data->max_y, 0, 0);

	if (data->t100_aux_ampl)
		input_set_abs_params(input_dev, ABS_PRESSURE,
				0, 255, 0, 0);

	/* For multi touch */
	error = input_mt_init_slots(input_dev, data->num_touchids,
			INPUT_MT_DIRECT);
	if (error) {
		dev_err(dev, "Error %d initialising slots\n", error);
		goto err_free_mem;
	}

	//input_set_abs_params(input_dev, ABS_MT_TOOL_TYPE, 0, MT_TOOL_MAX, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X,
			0, data->max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y,
			0, data->max_y, 0, 0);

	if (data->t100_aux_area)
		input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR,
				0, MXT_MAX_AREA, 0, 0);

	if (data->t100_aux_ampl)
		input_set_abs_params(input_dev, ABS_MT_PRESSURE,
				0, 255, 0, 0);

	if (data->t100_aux_vect)
		input_set_abs_params(input_dev, ABS_MT_ORIENTATION,
				0, 255, 0, 0);

	input_set_drvdata(input_dev, data);

	error = input_register_device(input_dev);
	if (error) {
		dev_err(dev, "Error %d registering input device\n", error);
		goto err_free_mem;
	}

	data->input_dev = input_dev;

	return 0;

err_free_mem:
	input_free_device(input_dev);
	return error;
}

static int strtobyte(const char *data, u8 *value)
{
	char str[3];

	str[0] = data[0];
	str[1] = data[1];
	str[2] = '\0';

	return kstrtou8(str,16, value);
}

static size_t mxt_convert_text_to_binary(u8 *buffer, size_t len)
{
	int ret;
	int i;
	int j = 0;

	for (i = 0; i < len; i+=2) {
		ret = strtobyte(&buffer[i], &buffer[j]);
		if (ret)
			return -EINVAL;
		j++;
	}

	return (size_t)j;
}

static int mxt_initialize(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	int error;
	bool alt_bootloader_addr = false;
	bool retry = false;

	mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
retry_info:
	error = mxt_read_info_block(data);
	if (error == 0) {
		mxt_read_lockdown_info(data);
		back_cover = (int)data->lockdown_info[3];
		MXT_LOG("atmel mxt back_cover = %d\n", back_cover);
	} else {
retry_bootloader:
		error = mxt_probe_bootloader(data, alt_bootloader_addr);
		if (error) {
			if (alt_bootloader_addr) {
				/* Chip is not in appmode or bootloader mode */
				return error;
			}

			dev_info(&client->dev, "Trying alternate bootloader address\n");
			alt_bootloader_addr = true;
			goto retry_bootloader;
		} else {
			if (retry) {
				dev_err(&client->dev, "mxt: Could not recover from bootloader mode, try to flash a firmware image to TP anyway\n");
				/*
				 * We can reflash from this state, so do not
				 * abort init
				 */
				data->in_bootloader = true;

				return -EINVAL;
			}

			/* Attempt to exit bootloader into app mode */
			mxt_send_bootloader_cmd(data, false);
			msleep(MXT_FW_RESET_TIME);
			retry = true;
			goto retry_info;
		}
	}

	error = mxt_check_retrigen(data);
	if (error)
		goto err_free_object_table;

	error = mxt_register_input_device(data);
	if (error)
		dev_err(&data->client->dev, "Failed to register input device\n");

	error = mxt_acquire_irq(data);
	if (error)
		goto err_free_object_table;

	error = mxt_debug_msg_init(data);
	if (error)
		goto err_free_object_table;

	return 0;

err_free_object_table:
	mxt_free_object_table(data);
	return error;
}

static int mxt_register_input_device(struct mxt_data *data)
{
	int ret = 0;
	struct device *dev = &data->client->dev;

	if (!data->T9_reportid_min && !data->T100_reportid_min) {
		dev_err(dev, "%s, invalid parameters\n", __func__);
		return -EINVAL;
	}
	mxt_free_input_device(data);
	if (data->T9_reportid_min) {
		ret = mxt_initialize_t9_input_device(data);
		if (ret) {
			dev_err(dev, "Failed to register t9 input device\n");
			return ret;
		}
	} else if (data->T100_reportid_min) {
		ret = mxt_initialize_t100_input_device(data);
		if (ret) {
			dev_err(dev, "Failed to register t100 input device\n");
			return ret;
		}
	}else
		dev_err(dev, "Failed to find T9 or T100 object\n");

	return ret;
}

static int mxt_configure_objects(struct mxt_data *data,
		const struct firmware *cfg)
{
	struct device *dev = &data->client->dev;
	int error;

	error = mxt_init_t7_power_cfg(data);
	if (error) {
		dev_err(dev, "Failed to initialize power cfg\n");
		goto err_free_object_table;
	}

	if (cfg) {
		error = mxt_update_cfg(data, cfg);
		if (error)
			dev_warn(dev, "Error %d updating config\n", error);
	}

	error = mxt_register_input_device(data);
	if (error) {
		dev_err(dev, "Failed to register input device\n");
		return error;
	}

	return 0;

err_free_object_table:
	mxt_free_object_table(data);
	return error;
}

/* Configuration crc check sum is returned as hex xxxxxx */
static ssize_t mxt_config_csum_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	return scnprintf(buf, PAGE_SIZE, "%06x\n", data->config_crc);
}

/* Firmware Version is returned as Major.Minor.Build */
static ssize_t mxt_fw_version_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	return scnprintf(buf, PAGE_SIZE, "%02X.%02X\n",
			data->info->version, data->info->build);
}
static ssize_t mxt_fw_version_check_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	const char *fw_name = data->pdata->fw_version;
	struct mxt_info *info = data->info;
	struct mxt_config_info *config_info = &data->pdata->info;
	struct mxt_cfg_version *cfg_version = &data->config_info.cfg_version;
	int cnt = 0;
	unsigned int version, build;

	while(data->staying == IS_UPDATINGE)
	{
		msleep(1000);
	}
	if(sscanf(fw_name, "%2x%2x", &version, &build) != 2)
	{
		return scnprintf(buf, PAGE_SIZE, "Error\n");
	}

	if (info->version != version || info->build != build) {
		cnt += snprintf(buf + cnt, PAGE_SIZE - cnt, "Image FW: %X.%X, IC FW: %X.%X\n", version, build,
				info->version, info->build);
	}

	if(cfg_version->year != config_info->config_year ||
			cfg_version->month != config_info->config_month ||
			cfg_version->date != config_info->config_date)
	{
		cnt += snprintf(buf + cnt, PAGE_SIZE - cnt, "IC Config: %d.%d.%d, Image Config: %d.%d.%d",
				cfg_version->year, cfg_version->month, cfg_version->date,
				config_info->config_year, config_info->config_month, config_info->config_date);
	}
	if(cnt == 0)
		cnt += snprintf(buf + cnt, PAGE_SIZE - cnt, "Pass\n");
	return cnt;
}
/* Firmware Config Version is returned as YearMonthDay */
static ssize_t mxt_cfg_version_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	struct mxt_cfg_version *cfg_version = &data->config_info.cfg_version;
	return scnprintf(buf, PAGE_SIZE, "%02d%02d%02d\n",
			cfg_version->year,
			cfg_version->month,
			cfg_version->date);
}

/* Hardware Version is returned as FamilyID.VariantID */
static ssize_t mxt_hw_version_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	return scnprintf(buf, PAGE_SIZE, "%u.%u\n",
			data->info->family_id, data->info->variant_id);
}

static ssize_t mxt_show_instance(char *buf, int count,
		struct mxt_object *object, int instance,
		const u8 *val)
{
	int i;

	if (mxt_obj_instances(object) > 1)
		count += scnprintf(buf + count, PAGE_SIZE - count,
				"Instance %u\n", instance);

	for (i = 0; i < mxt_obj_size(object); i++)
		count += scnprintf(buf + count, PAGE_SIZE - count,
				"\t[%2u]: %02x (%d)\n", i, val[i], val[i]);
	count += scnprintf(buf + count, PAGE_SIZE - count, "\n");

	return count;
}

static ssize_t mxt_object_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	struct mxt_object *object;
	int count = 0;
	int i, j;
	int error;
	u8 *obuf;

	/* Pre-allocate buffer large enough to hold max sized object. */
	obuf = kmalloc(256, GFP_KERNEL);
	if (!obuf)
		return -ENOMEM;

	error = 0;
	for (i = 0; i < data->info->object_num; i++) {
		object = data->object_table + i;

		if (!mxt_object_readable(object->type))
			continue;

		count += scnprintf(buf + count, PAGE_SIZE - count,
				"T%u:\n", object->type);

		for (j = 0; j < mxt_obj_instances(object); j++) {
			u16 size = mxt_obj_size(object);
			u16 addr = object->start_address + j * size;

			error = __mxt_read_reg(data->client, addr, size, obuf);
			if (error)
				goto done;

			count = mxt_show_instance(buf, count, object, j, obuf);
		}
	}

done:
	kfree(obuf);
	return error ?: count;
}

static int mxt_check_firmware_format(struct device *dev,
		const struct firmware *fw)
{
	unsigned int pos = 0;
	char c;

	while (pos < fw->size) {
		c = *(fw->data + pos);

		if (c < '0' || (c > '9' && c < 'A') || c > 'F')
			return 0;

		pos++;
	}

	/*
	 * To convert file try:
	 * xxd -r -p mXTXXX__APP_VX-X-XX.enc > maxtouch.fw
	 */
	dev_err(dev, "Firmware file isn't in binary format\n");

	return -EINVAL;
}

static void mtk_kick_wdt(void)
{

	mtk_wdt_restart(WD_TYPE_NORMAL);
	mtk_wdt_restart(WD_TYPE_NOLOCK);

}

static int mxt_load_fw(struct device *dev)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	const struct firmware *fw = NULL;
	unsigned int frame_size;
	unsigned int pos = 0;
	unsigned int retry = 0;
	unsigned int frame = 0;
	int ret;
	size_t len = 0;
	u8 *buffer  = NULL;
	ret = request_firmware(&fw, data->fw_name, dev);
	if (ret) {
		dev_err(dev, "Unable to open firmware %s\n", data->fw_name);
		return ret;
	}
	buffer = kmalloc(fw->size, GFP_KERNEL);
	if (!buffer) {
		dev_err(dev, "unable to allocate the memory for fw buffer\n");
		return -ENOMEM;
	}
	memcpy(buffer, fw->data, fw->size);
	len = fw->size;

	/* Check for incorrect enc file */
	ret = mxt_check_firmware_format(dev, fw);
	if (ret) {
		dev_info(dev, "converted to binary image\n");
		len = mxt_convert_text_to_binary(buffer, len);
		if (len < 0)
			goto release_firmware;
	}

	if (data->suspended) {
		if (data->use_regulator)
			mxt_regulator_enable(data);
		mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
		data->suspended = false;
	}

	if (!data->in_bootloader) {
		/* Change to the bootloader mode */
		data->in_bootloader = true;

		ret = mxt_t6_command(data, MXT_COMMAND_RESET,
				MXT_BOOT_VALUE, false);
		if (ret)
			goto release_firmware;

		msleep(MXT_RESET_TIME);

		/* Do not need to scan since we know family ID */
		ret = mxt_lookup_bootloader_address(data, 0);
		if (ret)
			goto release_firmware;
	} else {
		mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
	}

	mxt_free_object_table(data);
	mxt_reinit_completion(&data->bl_completion);

	ret = mxt_check_bootloader(data, MXT_WAITING_BOOTLOAD_CMD, false);
	if (ret) {
		/* Bootloader may still be unlocked from previous attempt */
		ret = mxt_check_bootloader(data, MXT_WAITING_FRAME_DATA, false);
		if (ret)
			goto mt_eint_mask;
	} else {
		MXT_LOG("Unlocking bootloader\n");

		msleep(100);

		/* Unlock bootloader */
		ret = mxt_send_bootloader_cmd(data, true);
		msleep(MXT_RESET_TIME);
		if (ret)
			goto mt_eint_mask;
	}

	while (pos < len) {

		ret = mxt_check_bootloader(data, MXT_WAITING_FRAME_DATA, true);
		if (ret)
			goto mt_eint_mask;

		frame_size = ((*(buffer + pos) << 8) | *(buffer + pos + 1));

		/* Take account of CRC bytes */
		frame_size += 2;

		/* Write one frame to device */
		ret = mxt_bootloader_write(data, buffer + pos, frame_size);
		if (ret)
			goto mt_eint_mask;

		ret = mxt_check_bootloader(data, MXT_FRAME_CRC_PASS, true);
		if (ret) {
			retry++;
			MXT_LOG("mxt retry: %d\n", retry);
			/* Back off by 20ms per retry */
			msleep(retry * 20);

			if (retry > 20) {
				dev_err(dev, "Retry count exceeded\n");
				goto mt_eint_mask;
			}
		} else {
			retry = 0;
			pos += frame_size;
			frame++;
		}

		if (frame % 5 == 0)
		{
			mtk_kick_wdt();
			MXT_LOG("mxt: sent %d frames, %d/%zd bytes\n", frame, pos, fw->size);
		}
	}

	mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);

	/* Wait for flash. */
	ret = mxt_wait_for_completion(data, &data->bl_completion,
			MXT_FW_RESET_TIME);
	if (ret)
		goto mt_eint_mask;

	MXT_LOG("mxt: sent %d frames, %d bytes\n", frame, pos);
	MXT_LOG("mxt: update fw successfully\n");

	/*
	 * Wait for device to reset. Some bootloader versions do not assert
	 * the CHG line after bootloading has finished, so ignore potential
	 * errors.
	 */
	// mxt_wait_for_completion(data, &data->bl_completion, MXT_FW_RESET_TIME);

	data->in_bootloader = false;

mt_eint_mask:
	mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
release_firmware:
	release_firmware(fw);
	if (buffer)
		kfree(buffer);
	return ret;
}

static int mxt_update_file_name(struct device *dev, char **file_name,
		const char *buf, size_t count)
{
	char *file_name_tmp;

	/* Simple sanity check */
	if (count > 64) {
		dev_warn(dev, "File name too long\n");
		return -EINVAL;
	}

	file_name_tmp = krealloc(*file_name, count + 1, GFP_KERNEL);
	if (!file_name_tmp) {
		dev_warn(dev, "no memory\n");
		return -ENOMEM;
	}

	*file_name = file_name_tmp;
	memcpy(*file_name, buf, count);

	/* Echo into the sysfs entry may append newline at the end of buf */
	if (buf[count - 1] == '\n')
		(*file_name)[count - 1] = '\0';
	else
		(*file_name)[count] = '\0';

	return 0;
}


static ssize_t mxt_update_fw_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	int error;

	mutex_lock(&data->esd_timer_lock);
	MXT_LOG("%s %d lock ESD_timer_running:%d\n", __func__, __LINE__, ESD_timer_running);
	if(ESD_timer_running){
#ifdef CONFIG_MXT_T61_HR
		mxt_t61_deinit(data, &t61);
		MXT_LOG("mxt: ctrl:%d cmd: %d, mode:%d, period:%d\n",t61.ctrl, t61.cmd, t61.mode, t61.period);
		mxt_set_obj_t61(data, &t61, 0);
#endif
		del_timer_sync(&ESD_timer);
		ESD_timer_running = false;
	}
	MXT_LOG("%s %d unlock ESD_timer_running:%d\n", __func__, __LINE__, ESD_timer_running);
	mutex_unlock(&data->esd_timer_lock);
	MXT_LOG("%s %d\n", __func__, __LINE__);

	error = mxt_update_file_name(dev, &data->fw_name, buf, count);
	if (error)
		return error;

	error = mxt_load_fw(dev);
	if (error) {
		dev_err(dev, "The firmware update failed(%d)\n", error);
		count = error;
	} else {
		dev_info(dev, "The firmware update succeeded\n");

		data->suspended = false;

		msleep(200);

		error = mxt_initialize(data);
		if (error)
			return error;
	}

	return count;
}

static int __mxt_update_fw(struct device *dev,
		const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	int error;

	error = mxt_update_file_name(dev, &data->fw_name, buf, count);
	if (error)
		return error;

	error = mxt_load_fw(dev);
	if (error) {
		dev_err(dev, "The firmware update failed(%d)\n", error);
		return error;
	} else {
		dev_info(dev, "The firmware update succeeded\n");

		data->suspended = false;

		msleep(200);

		error = mxt_initialize(data);
		if (error)
			return error;
	}

	return 0;
}


static ssize_t mxt_update_cfg_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	const struct firmware *cfg;
	int ret;

	if (data->in_bootloader) {
		dev_err(dev, "Not in appmode\n");
		return -EINVAL;
	}

	ret = mxt_update_file_name(dev, &data->cfg_name, buf, count);
	if (ret)
		return ret;

	ret = request_firmware(&cfg, data->cfg_name, dev);
	if (ret < 0) {
		dev_err(dev, "Failure to request config file %s\n",
				data->cfg_name);
		ret = -ENOENT;
		goto out;
	}

	data->updating_config = true;

	if (data->suspended) {
		if (data->use_regulator) {
			mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
			mxt_regulator_enable(data);
		} else {
			mxt_set_t7_power_cfg(data, MXT_POWER_CFG_FULL_RUN, false);
			mxt_acquire_irq(data);
		}

		data->suspended = false;
	}

	ret = mxt_configure_objects(data, cfg);
	if (ret)
		goto out;

	ret = count;
out:
	data->updating_config = false;
	return ret;
}


static ssize_t mxt_update_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	int ret;
	data->update_force = true;
	ret = mxt_update_fw_and_cfg(data);
	if (ret) {
		return snprintf(buf, PAGE_SIZE, "Update Failed\n");
	}
	else{
		return snprintf(buf, PAGE_SIZE, "Update Pass\n");
	}
}


static ssize_t mxt_reset_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int ret = 1;
	unsigned int reset;
	struct mxt_data *data = dev_get_drvdata(dev);

	if (sscanf(buf, "%u", &reset) != 1)
		return -EINVAL;

	if (reset != 1)
		return -EINVAL;

	msleep(MXT_RESET_TIME);
	/* Recalibrate to avoid touch panel chaos */
	mxt_t6_command(data, MXT_COMMAND_RESET, 1, true);

	dev_err(dev, "%s: after reset\n", __func__);

	return ret;
}

static ssize_t mxt_debug_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	char c;

	c = data->debug_enabled ? '1' : '0';
	return scnprintf(buf, PAGE_SIZE, "%c\n", c);
}

static ssize_t mxt_debug_notify_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "0\n");
}

static ssize_t mxt_debug_v2_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	int i;

	if (sscanf(buf, "%u", &i) == 1 && i < 2) {
		if (i == 1)
			mxt_debug_msg_enable(data);
		else
			mxt_debug_msg_disable(data);

		return count;
	} else {
		MXT_LOG("debug_enabled write error\n");
		return -EINVAL;
	}
}

static ssize_t mxt_debug_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	int i;

	if (sscanf(buf, "%u", &i) == 1 && i < 2) {
		data->debug_enabled = (i == 1);

		MXT_LOG("%s\n", i ? "debug enabled" : "debug disabled");
		return count;
	} else {
		MXT_LOG("debug_enabled write error\n");
		return -EINVAL;
	}
}

static ssize_t mxt_sys_suspend_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	struct mxt_object *t81_obj= mxt_get_object(data,MXT_UNLOCK_GESTURE_T81);
	int error = 0;

	if (!t81_obj) {
		dev_err(&client->dev, "There is no t81 object\n");
		return 0;
	}

	error = __mxt_read_reg(client,t81_obj->start_address,
			t81_obj->size_minus_one + 1,&data->t81_cfg);
	if (error)
		dev_err(&client->dev, "%s: i2c_sent  failed\n",__func__);

	return scnprintf(buf, PAGE_SIZE,"0x%x\n", data->t81_cfg.ctrl);
}

static ssize_t mxt_sys_suspend_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	struct mxt_object *t81_obj = mxt_get_object(data, MXT_UNLOCK_GESTURE_T81);
	struct mxt_object *t100_obj = mxt_get_object(data, MXT_TOUCH_MULTITOUCHSCREEN_T100);
	u8 ctrl = 0, mask1 = 0xfe, mask2 = 0x1;
	u8 mask3 = 0xfd, mask4 = 0x2;
	u8 error = 0, value = 0;
	u8 *cmd = NULL;

	if (!t81_obj) {
		dev_err(&client->dev, "There is no object\n");
		return 0;
	}

	cmd = kmalloc(count + 1, GFP_KERNEL);
	if (cmd == NULL)
		goto release_error;

	memcpy(cmd, buf, count);

	if (cmd[count-1] == '\n')
		cmd[count-1] = '\0';
	else
		cmd[count] = '\0';

	if (!strcmp(cmd, "true")) {
		error = __mxt_read_reg(client,t100_obj->start_address, 1, &ctrl);
		if (error)
			dev_err(&client->dev, "%s: i2c sent failed\n", __func__);

		value = ctrl & mask3;

		dev_info(dev, "%s: t100 configuration write: 0x%x\n",__func__, value);

		error = __mxt_write_reg(client,t100_obj->start_address, 1,&value);
		if (error)
			dev_err(&client->dev, "%s: i2c sent failed\n", __func__);

		error = __mxt_read_reg(client, t81_obj->start_address, 1, &ctrl);
		if (error)
			dev_err(&client->dev, "%s: i2c sent failed\n", __func__);
		value = ctrl | mask2;
		dev_info(&client->dev, "%s: t81_configuration write:0x%x\n",__func__,value);

		error = __mxt_write_reg(client,t81_obj->start_address, 1,&value);
		if (error)
			dev_err(&client->dev, "%s: i2c sent failed\n", __func__);
	} else if (!strcmp(cmd, "false")) {
		error = __mxt_read_reg(client,t81_obj->start_address,1, &ctrl);
		if (error)
			dev_err(&client->dev, "%s: i2c sent failed\n", __func__);

		value = ctrl & mask1;
		dev_info(&client->dev, "%s: else t81 configuration write 0x%x\n",__func__,value);

		error = __mxt_write_reg(client,t81_obj->start_address, 1,&value);
		if (error)
			dev_err(&client->dev, "%s: i2c sent failed\n", __func__);

		error = __mxt_read_reg(client,t100_obj->start_address, 1, &ctrl);
		if (error)
			dev_err(&client->dev, "%s: i2c sent failed\n", __func__);
		value = ctrl | mask4;
		dev_err(&client->dev, "%s: else t100 configuration write: 0x%x\n",__func__, value);

		error = __mxt_write_reg(client,t100_obj->start_address, 1,&value);
		if (error)
			dev_err(&client->dev, "%s: i2c sent failed\n", __func__);

	}
release_error:
	return count;
}

static int mxt_check_mem_access_params(struct mxt_data *data, loff_t off,
		size_t *count)
{
	if (off >= data->mem_size)
		return -EIO;

	if (off + *count > data->mem_size)
		*count = data->mem_size - off;

	if (*count > MXT_MAX_BLOCK_WRITE)
		*count = MXT_MAX_BLOCK_WRITE;

	return 0;
}

static ssize_t mxt_mem_access_read(struct file *filp, struct kobject *kobj,
		struct bin_attribute *bin_attr, char *buf, loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct mxt_data *data = dev_get_drvdata(dev);
	int ret = 0;

	ret = mxt_check_mem_access_params(data, off, &count);
	if (ret < 0)
		return ret;

	if (count > 0)
		ret = __mxt_read_reg(data->client, off, count, buf);

	return ret == 0 ? count : ret;
}

static ssize_t mxt_mem_access_write(struct file *filp, struct kobject *kobj,
		struct bin_attribute *bin_attr, char *buf, loff_t off,
		size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct mxt_data *data = dev_get_drvdata(dev);
	int ret = 0;

	ret = mxt_check_mem_access_params(data, off, &count);
	if (ret < 0)
		return ret;

	if (count > 0)
		ret = __mxt_write_reg(data->client, off, count, buf);

	return ret == 0 ? count : ret;
}


/* Help function for performing a T6 diagnostic command */
static int mxt_T6_diag_cmd(struct mxt_data *data, struct mxt_object *T6,
		u8 cmd)
{
	int ret;
	u16 addr = T6->start_address + MXT_COMMAND_DIAGNOSTIC;

	ret = mxt_write_reg(data->client, addr, cmd);
	if (ret)
		return ret;

	/* Poll T6.diag until it returns 0x00, which indicates command has completed. */
	while (cmd != 0) {
		ret = __mxt_read_reg(data->client, addr, 1, &cmd);
		if (ret)
			return ret;
	}
	return 0;
}

static int read_raw_data(struct mxt_data *data, u8 mode)
{
	struct mxt_object *T6, *T37;
	u8 *obuf;
	ssize_t ret = 0;
	size_t i,j;
	u8 lsb, msb;
	size_t T37_buf_size, num_pages;
	size_t pos;

	if (!data || !data->object_table)
		return -ENODEV;

	T6 = mxt_get_object(data, MXT_GEN_COMMAND_T6);
	T37 = mxt_get_object(data, MXT_DEBUG_DIAGNOSTIC_T37);
	if (!T6 || mxt_obj_size(T6) < 6 || !T37 || mxt_obj_size(T37) < 3) {
		dev_err(&data->client->dev, "Invalid T6 or T37 object\n");
		return -ENODEV;
	}

	/* Something has gone wrong if T37_buf is already allocated */
	if (data->T37_buf)
	{
		kfree(data->T37_buf);
		data->T37_buf = NULL;
		data->T37_buf_size = 0;
	}

	if(mode == MXT_T6_CMD_SELFCAP_DELTAS)
	{
		T37_buf_size = (data->info->matrix_xsize + data->info->matrix_ysize) * sizeof(__le16);
	}else
	{
		T37_buf_size = data->info->matrix_xsize * data->info->matrix_ysize * sizeof(__le16);
	}
	data->T37_buf_size = T37_buf_size;
	data->T37_buf = kmalloc(data->T37_buf_size, GFP_KERNEL);
	if (!data->T37_buf)
		return -ENOMEM;

	/* Temporary buffer used to fetch one T37 page */
	obuf = kmalloc(mxt_obj_size(T37), GFP_KERNEL);
	if (!obuf)
		return -ENOMEM;

	// disable_irq(data->irq);
	mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);

	//frank add for 144U
	if((mode == MXT_T6_CMD_SELFCAP_DELTAS) || (mode == MXT_T6_CMD_SELFCAP_REFS)){
		num_pages = 1;
	} else{
		num_pages = DIV_ROUND_UP(T37_buf_size, mxt_obj_size(T37) - 2);
	}

	pos = 0;
	for (i = 0; i < num_pages; i++) {
		u8 cmd;
		size_t chunk_len;

		/* For first page, send mode as cmd, otherwise PageUp */
		cmd = (i == 0) ? mode : MXT_T6_CMD_PAGE_UP;
		ret = mxt_T6_diag_cmd(data, T6, cmd);
		if (ret)
			goto err_free_T37_buf;

		ret = __mxt_read_reg(data->client, T37->start_address,
				mxt_obj_size(T37), obuf);
		if (ret)
			goto err_free_T37_buf;

		/* Verify first two bytes are current mode and page # */
		if (obuf[0] != mode) {
			dev_err(&data->client->dev,
					"Unexpected mode (%u != %u)\n", obuf[0], mode);
			ret = -EIO;
			goto err_free_T37_buf;
		}

		if (obuf[1] != i) {
			dev_err(&data->client->dev,
					"Unexpected page (%u != %zu)\n", obuf[1], i);
			ret = -EIO;
			goto err_free_T37_buf;
		}

		/*
		 * Copy the data portion of the page, or however many bytes are
		 * left, whichever is less.
		 */
		chunk_len = min(mxt_obj_size(T37) - 2, T37_buf_size - pos);
		memcpy(&data->T37_buf[pos], &obuf[2], chunk_len);
		pos += chunk_len;
	}

	if((mode == MXT_T6_CMD_SELFCAP_DELTAS) || (mode == MXT_T6_CMD_SELFCAP_REFS)) {
		i = 0;
		for(j = 0; j < (data->info->matrix_xsize + data->info->matrix_ysize) * 2; j += 2) {
			lsb = data->T37_buf[j] & 0xff;
			msb = data->T37_buf[j+1] & 0xff;
			data->raw_data_16[i] = lsb | (msb << 8);
			i++;
		}
	}else if((mode == MXT_T6_CMD_DELTAS) || (mode == MXT_T6_CMD_REFS)) {
		i = 0;
		for(j = 0; j < data->info->matrix_xsize * data->info->matrix_ysize * 2; j += 2) {
			lsb = data->T37_buf[j] & 0xff;
			msb = data->T37_buf[j+1] & 0xff;
			data->raw_data_16[i] = lsb | (msb << 8);
			i++;
		}
	}else {
		i = 0;
		for(j = 0; j < data->info->matrix_xsize * data->info->matrix_ysize * 2; j += 2) {
			lsb = data->T37_buf[j] & 0xff;
			msb = data->T37_buf[j+1] & 0xff;
			data->raw_data_16[i] = lsb | (msb << 8);
			i++;
		}
	}

	goto out;

err_free_T37_buf:
	kfree(data->T37_buf);
	data->T37_buf = NULL;
	data->T37_buf_size = 0;
out:
	kfree(obuf);
	return ret;
}


static ssize_t mxt_short_circuit_test_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	//do not need this
	return snprintf(buf, PAGE_SIZE, "Pass\n");
}

static ssize_t mxt_open_circuit_test_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	short *report_data_16, limit_b, limit_l;

	read_raw_data(data, MXT_T6_CMD_REFS);
	report_data_16 = data->raw_data_16;
	limit_b = data->raw_data_avg + 3500;
	limit_l = data->raw_data_avg - 3500;

	return snprintf(buf, PAGE_SIZE, "Pass\n");
}

static ssize_t mxt_selfcap_delta_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	int ii;
	int cnt = 0;
	int count = 0;

	short *report_data_16;

	read_raw_data(data, MXT_T6_CMD_SELFCAP_DELTAS);
	report_data_16 = data->raw_data_16;

	for (ii = 0; ii < (TX_NUM + RX_NUM); ii++) {
		cnt = snprintf(buf, PAGE_SIZE - count, "%-4d, ", *report_data_16);
		report_data_16++;
		buf += cnt;
		count += cnt;

		cnt = snprintf(buf, PAGE_SIZE - count, "\n");
		buf += cnt;
		count += cnt;
	}

	cnt = snprintf(buf, PAGE_SIZE - count, "tx = %d\nrx = %d\n", TX_NUM, RX_NUM);
	buf += cnt;
	count += cnt;

	return count;
}

static ssize_t mxt_selfcap_ref_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	int ii;
	int cnt = 0;
	int count = 0;

	short *report_data_16;

	read_raw_data(data, MXT_T6_CMD_SELFCAP_REFS);
	report_data_16 = data->raw_data_16;

	for (ii = 0; ii < (TX_NUM + RX_NUM); ii++) {
		cnt = snprintf(buf, PAGE_SIZE - count, "%-4d, ", *report_data_16);
		report_data_16++;
		buf += cnt;
		count += cnt;

		cnt = snprintf(buf, PAGE_SIZE - count, "\n");
		buf += cnt;
		count += cnt;
	}

	cnt = snprintf(buf, PAGE_SIZE - count, "tx = %d\nrx = %d\n", TX_NUM, RX_NUM);
	buf += cnt;
	count += cnt;

	return count;
}

static ssize_t mxt_reference_data_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	int ii;
	unsigned int jj;
	int cnt = 0;
	int count = 0;

	unsigned int sum = 0;
	short max = 0;
	short min = 0;
	u16 *report_data_16;

	struct mxt_info *info = data->info;
	struct mxt_cfg_version *cfg_version = &data->config_info.cfg_version;

	read_raw_data(data, MXT_T6_CMD_REFS);
	report_data_16 = data->raw_data_16;

	for (ii = 0; ii < TX_NUM; ii++) {
		for (jj = 0; jj < RX_NUM; jj++) {
			cnt = snprintf(buf, PAGE_SIZE - count, "%-4d, ",
					*report_data_16);

			sum += *report_data_16;

			if (max < *report_data_16)
				max = *report_data_16;

			if (ii == 0 && jj == 0)
				min = *report_data_16;
			else if (*report_data_16 < min)
				min = *report_data_16;

			report_data_16++;
			buf += cnt;
			count += cnt;
		}
		cnt = snprintf(buf, PAGE_SIZE - count, "\n");
		buf += cnt;
		count += cnt;
	}
	cnt = snprintf(buf, PAGE_SIZE - count, "\n");
	buf += cnt;
	count += cnt;

	cnt = snprintf(buf, PAGE_SIZE - count, "tx = %d\nrx = %d\n",
			TX_NUM, RX_NUM);
	buf += cnt;
	count += cnt;

	cnt = snprintf(buf, PAGE_SIZE - count, "FW: %X.%X\t, Config: %d.%d.%d\n", info->version, info->build,
			cfg_version->year, cfg_version->month, cfg_version->date);
	buf += cnt;
	count += cnt;

	data->raw_data_avg = sum/RAW_DATA_SIZE;
	cnt = snprintf(buf, PAGE_SIZE - count,
			"max = %d, min = %d, average = %d\n",
			max, min, data->raw_data_avg);
	buf += cnt;
	count += cnt;

	return count;
}

static ssize_t mxt_delta_data_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	int ii;
	unsigned int jj;
	int cnt = 0;
	int count = 0;

	unsigned int sum = 0;
	short max = 0;
	short min = 0;
	short *report_data_16;

	struct mxt_info *info = data->info;
	struct mxt_cfg_version *cfg_version = &data->config_info.cfg_version;

	read_raw_data(data, MXT_T6_CMD_DELTAS);
	report_data_16 = data->raw_data_16;

	for (ii = 0; ii < TX_NUM; ii++) {
		for (jj = 0; jj < RX_NUM; jj++) {
			cnt = snprintf(buf, PAGE_SIZE - count, "%-4d, ",
					*report_data_16);

			sum += *report_data_16;

			if (max < *report_data_16)
				max = *report_data_16;

			if (ii == 0 && jj == 0)
				min = *report_data_16;
			else if (*report_data_16 < min)
				min = *report_data_16;

			report_data_16++;
			buf += cnt;
			count += cnt;
		}
		cnt = snprintf(buf, PAGE_SIZE - count, "\n");
		buf += cnt;
		count += cnt;
	}
	cnt = snprintf(buf, PAGE_SIZE - count, "\n");
	buf += cnt;
	count += cnt;

	cnt = snprintf(buf, PAGE_SIZE - count, "tx = %d\nrx = %d\n",
			TX_NUM, RX_NUM);
	buf += cnt;
	count += cnt;

	cnt = snprintf(buf, PAGE_SIZE - count, "FW: %X.%X\t, Config: %d.%d.%d\n", info->version, info->build,
			cfg_version->year, cfg_version->month, cfg_version->date);
	buf += cnt;
	count += cnt;

	data->raw_data_avg = sum/RAW_DATA_SIZE;
	cnt = snprintf(buf, PAGE_SIZE - count,
			"max = %d, min = %d, average = %d\n",
			max, min, data->raw_data_avg);
	buf += cnt;
	count += cnt;

	return count;
}

static ssize_t mxt_noise_data_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	noise_data_report_enable = 1;
	noise_data_result = 0;

	msleep(NOISE_LEVEL_TIMEOUT);

	noise_data_report_enable = 0;

	if(noise_data_result){
		return snprintf(buf, PAGE_SIZE, "Fail\n");
	} else{
		return snprintf(buf, PAGE_SIZE, "Pass\n");
	}
}

#ifdef MAX1_WAKEUP_GESTURE_ENABLE
static ssize_t mxt_double_click_wake_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "double_click_wake_enable:%u ; enable_wakeup_gesture:%u\n",
			data->double_click_wake_enable, data->enable_wakeup_gesture);
}

static ssize_t mxt_double_click_wake_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int input;
	struct mxt_data *data = dev_get_drvdata(dev);

	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	data->double_click_wake_enable = input;

	if (data->double_click_wake_enable)
		data->enable_wakeup_gesture = true;
	else
		data->enable_wakeup_gesture = false;

	if(data->enable_wakeup_gesture == true){
		enable_irq_wake(data->client->irq);
	} else {
		disable_irq_wake(data->client->irq);
	}

	return count;
}

static ssize_t mxt_down_to_up_wake_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n",
			data->down_to_up_wake_enable);
}

static ssize_t mxt_down_to_up_wake_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int input;
	struct mxt_data *data = dev_get_drvdata(dev);

	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	data->down_to_up_wake_enable = input;

	if (data->down_to_up_wake_enable)
		data->enable_wakeup_gesture = true;
	else
		data->enable_wakeup_gesture = false;

	if(data->enable_wakeup_gesture == true)
		enable_irq_wake(data->client->irq);
	else
		disable_irq_wake(data->client->irq);

	return count;
}

static ssize_t mxt_up_to_down_wake_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n",
			data->up_to_down_wake_enable);
}

static ssize_t mxt_up_to_down_wake_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int input;
	struct mxt_data *data = dev_get_drvdata(dev);

	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	data->up_to_down_wake_enable = input;

	if (data->up_to_down_wake_enable)
		data->enable_wakeup_gesture = true;
	else
		data->enable_wakeup_gesture = false;

	if(data->enable_wakeup_gesture == true)
		enable_irq_wake(data->client->irq);
	else
		disable_irq_wake(data->client->irq);

	return count;
}

static ssize_t mxt_right_to_left_wake_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n",
			data->right_to_left_wake_enable);
}

static ssize_t mxt_right_to_left_wake_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int input;
	struct mxt_data *data = dev_get_drvdata(dev);

	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	data->right_to_left_wake_enable = input;

	if (data->right_to_left_wake_enable)
		data->enable_wakeup_gesture = true;
	else
		data->enable_wakeup_gesture = false;

	if(data->enable_wakeup_gesture == true)
		enable_irq_wake(data->client->irq);
	else
		disable_irq_wake(data->client->irq);

	return count;
}

static ssize_t mxt_left_to_right_wake_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n",
			data->left_to_right_wake_enable);
}

static ssize_t mxt_left_to_right_wake_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int input;
	struct mxt_data *data = dev_get_drvdata(dev);

	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	data->left_to_right_wake_enable = input;

	if (data->left_to_right_wake_enable)
		data->enable_wakeup_gesture = true;
	else
		data->enable_wakeup_gesture = false;

	if(data->enable_wakeup_gesture == true)
		enable_irq_wake(data->client->irq);
	else
		disable_irq_wake(data->client->irq);

	return count;
}

static ssize_t mxt_detected_gesture_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n",
			data->detected_gesture);
}
#endif

static ssize_t touch_debug_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int cmd, arg;
	struct mxt_data *data = dev_get_drvdata(dev);
	struct input_dev *input_dev = data->input_dev;
	if(sscanf(buf,"%d%d", &cmd, &arg) < 0)
		return -EINVAL;
	switch(cmd)
	{
		case 0:
			input_report_key(input_dev, KEY_APPSELECT, 1);
			input_sync(input_dev);
			input_report_key(input_dev, KEY_APPSELECT, 0);
			input_sync(input_dev);
			break;
		case 1:
			input_report_key(input_dev, KEY_HOMEPAGE, 1);
			input_sync(input_dev);
			input_report_key(input_dev, KEY_HOMEPAGE, 0);
			input_sync(input_dev);
			break;
		case 2:
			input_report_key(input_dev, KEY_BACK, 1);
			input_sync(input_dev);
			input_report_key(input_dev, KEY_BACK, 0);
			input_sync(input_dev);
			break;
		case 3:
			mxt_suspend(dev);
			break;
		case 4:
			mxt_resume(dev);
			break;
		case 5:
			input_report_key(input_dev, KEY_POWER, 1);
			input_sync(input_dev);
			input_report_key(input_dev, KEY_POWER, 0);
			input_sync(input_dev);
			break;
		case 6:
			data->enable_wakeup_gesture = true;
			break;
		case 7:
			tmp_check_mt_ref = true;
			MXT_LOG("mxt: tmp_check_mt_ref: %d\n", tmp_check_mt_ref);
			break;
		case 8:
			tmp_check_mt_ref = false;
			MXT_LOG("mxt: tmp_check_mt_ref: %d\n", tmp_check_mt_ref);
			break;
		default:
			break;
	}
	return count;
}
static ssize_t mulTouchtest_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int count = sprintf(buf,"%d\n", g_Multouch);
	return count;
}

static ssize_t mxt_ambient_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	int i = 0;

	if (sscanf(buf, "%u", &i) == 1 && i < 2) {
		MXT_LOG("power hal set ambient= %d\n", i);
		if (i == 0) {    //disable
			data->is_ambient_mode = false;
		} else if (i == 1) {
			data->is_ambient_mode = true;
		}
		return count;
	}
	return -EINVAL;
}

static ssize_t mxt_ambient_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	int count = sprintf(buf,"is_ambient_mode=%d\n", data->is_ambient_mode);
	return count;
}

static ssize_t mxt_suspend_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	int count = sprintf(buf,"suspend_show:%d %d %d\n", data->enable_wakeup_gesture, data->T24_tap_wake_enable, data->T24_double_tap_wake_enable);
	return count;
}
static ssize_t mxt_suspend_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	int i = 0;

	if (sscanf(buf, "%u", &i) == 1 && i < 3) {
		MXT_LOG("suspend value %d\n", i);
		if (i == 1) {    //disable
			data->enable_wakeup_gesture = false;
			data->T24_tap_wake_enable = false;
			data->T24_double_tap_wake_enable = false;
			disable_irq_wake(data->client->irq);
		}else if (i == 0) {    //single tap
			data->enable_wakeup_gesture = true;
			data->T24_tap_wake_enable = true;
			data->T24_double_tap_wake_enable = true;
			enable_irq_wake(data->client->irq);
		}else if (i == 2) {    //double tap
			data->enable_wakeup_gesture = true;
			data->T24_tap_wake_enable = false;
			data->T24_double_tap_wake_enable = true;
			enable_irq_wake(data->client->irq);
		}
		return count;
	} else {
		MXT_LOG("staying enabled write error\n");
		return -EINVAL;
	}
}

static ssize_t mxt_range_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int count = sprintf(buf, "mxt: sef cap ref range: [%d , %d] \n", esd_seflcap_ref_limit_low, esd_seflcap_ref_limit_high);

	return count;
}

static ssize_t mxt_range_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	if (sscanf(buf, "%d %d", &esd_seflcap_ref_limit_low, &esd_seflcap_ref_limit_high) == 2){
		MXT_LOG("mxt: sef cap ref range: [%d , %d] \n", esd_seflcap_ref_limit_low, esd_seflcap_ref_limit_high);
		return count;
	}else{
		MXT_LOG("Wrong format of range information\n");
		return -EINVAL;
	}
}


static ssize_t mxt_lockdown_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int count;
	struct mxt_data *data = dev_get_drvdata(dev);

	mxt_read_lockdown_info(data);

	count = sprintf(buf,"mxt: lock_down_info(0x): %x %x %x %x %x %x %x %x\n",
				(unsigned int)data->lockdown_info[0],
				(unsigned int)data->lockdown_info[1],
				(unsigned int)data->lockdown_info[2],
				(unsigned int)data->lockdown_info[3],
				(unsigned int)data->lockdown_info[4],
				(unsigned int)data->lockdown_info[5],
				(unsigned int)data->lockdown_info[6],
				(unsigned int)data->lockdown_info[7]);
	return count;
}

static ssize_t mxt_lockdown_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);

	if (sscanf(buf, "%x %x %x %x %x %x %x %x",
				(unsigned int *)&data->lockdown_info[0],
				(unsigned int *)&data->lockdown_info[1],
				(unsigned int *)&data->lockdown_info[2],
				(unsigned int *)&data->lockdown_info[3],
				(unsigned int *)&data->lockdown_info[4],
				(unsigned int *)&data->lockdown_info[5],
				(unsigned int *)&data->lockdown_info[6],
				(unsigned int *)&data->lockdown_info[7]) == MXT_LOCKDOWN_SIZE) {

		MXT_LOG("mxt: prepare to write lock_down_info: 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n",
				(unsigned int)data->lockdown_info[0],
				(unsigned int)data->lockdown_info[1],
				(unsigned int)data->lockdown_info[2],
				(unsigned int)data->lockdown_info[3],
				(unsigned int)data->lockdown_info[4],
				(unsigned int)data->lockdown_info[5],
				(unsigned int)data->lockdown_info[6],
				(unsigned int)data->lockdown_info[7]);

		mxt_write_lockdown_info(data, data->lockdown_info, MXT_LOCKDOWN_SIZE);

		return count;

	} else {
		MXT_LOG("Wrong format of lock down information\n");
		return -EINVAL;
	}
}
static int mxt_mulTouchtest_proc(struct file *file, char __user *page, size_t count, loff_t *ppos)
{
	char *ptr = page;
	struct mxt_data *data = mxt_i2c_data;

	if (*ppos)
	{
		return 0;
	}
	ptr += mulTouchtest_show(&data->client->dev, NULL, ptr);

	*ppos += ptr - page;
	return (ptr - page);
}
static int mxt_raw_cap_data_proc(struct file *file, char __user *page, size_t count, loff_t *ppos)
{
	char *ptr = page;
	struct mxt_data *data = mxt_i2c_data;

	if (*ppos)  // CMD call again
	{
		return 0;
	}
	ptr += mxt_reference_data_show(&data->client->dev, NULL, ptr);

	*ppos += ptr - page;
	return (ptr - page);
}

static int mxt_selfcap_delta_proc(struct file *file, char __user *page, size_t count, loff_t *ppos)
{
	char *ptr = page;
	struct mxt_data *data = mxt_i2c_data;

	if (*ppos)  // CMD call again
	{
		return 0;
	}
	ptr += mxt_selfcap_delta_show(&data->client->dev, NULL, ptr);

	*ppos += ptr - page;
	return (ptr - page);
}


static int mxt_open_circuit_test_proc(struct file *file, char __user *page, size_t count, loff_t *ppos)
{
	char *ptr = page;
	struct mxt_data *data = mxt_i2c_data;

	if (*ppos)  // CMD call again
	{
		return 0;
	}
	ptr += mxt_open_circuit_test_show(&data->client->dev, NULL, ptr);

	*ppos += ptr - page;
	return (ptr - page);
}

static int mxt_short_circuit_test_proc(struct file *file, char __user *page, size_t count, loff_t *ppos)
{
	char *ptr = page;
	struct mxt_data *data = mxt_i2c_data;

	if (*ppos)  // CMD call again
	{
		return 0;
	}
	ptr += mxt_short_circuit_test_show(&data->client->dev, NULL, ptr);

	*ppos += ptr - page;
	return (ptr - page);
}

static int mxt_fw_version_check_proc(struct file *file, char __user *page, size_t count, loff_t *ppos)
{
	char *ptr = page;
	struct mxt_data *data = mxt_i2c_data;

	if (*ppos)  // CMD call again
	{
		return 0;
	}
	ptr += mxt_fw_version_check_show(&data->client->dev, NULL, ptr);

	*ppos += ptr - page;
	return (ptr - page);
}

static PROC_FILE(mxt_fw_version_check, S_IRUGO | S_IWUGO, mxt_fw_version_check_proc, NULL);
static PROC_FILE(open_circuit_test, S_IRUGO | S_IWUGO, mxt_open_circuit_test_proc, NULL);
static PROC_FILE(short_circuit_test, S_IRUGO | S_IWUGO, mxt_short_circuit_test_proc, NULL);
static PROC_FILE(mulTouchtest, S_IRUGO | S_IWUGO, mxt_mulTouchtest_proc, NULL);
static PROC_FILE(reference, S_IRUGO | S_IWUGO, mxt_raw_cap_data_proc, NULL);
static PROC_FILE(selfcap_delta, S_IRUGO | S_IWUGO, mxt_selfcap_delta_proc, NULL);

static struct proc_file *sproc_file[] = {
	procify(mxt_fw_version_check),
	procify(open_circuit_test),
	procify(short_circuit_test),
	procify(mulTouchtest),
	procify(reference),
	procify(selfcap_delta),
};
static void mxt_reset_slots(struct mxt_data *data)
{
	struct input_dev *input_dev = data->input_dev;
	unsigned int num_mt_slots;
	int id;
	bool curr_state;
	const unsigned int *keymap;
	if (!input_dev)
		return;
	keymap = data->pdata->keymap[T15_T97_KEY];
	num_mt_slots = data->num_touchids + data->num_stylusids;

	for (id = 0; id < num_mt_slots; id++) {
		input_mt_slot(input_dev, id);
		input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, 0);
	}
	mxt_input_sync(data);
	for (id = 0; id < 3; id++) {
		curr_state = test_bit(id, &data->t15_keystatus);
		if (curr_state){
			input_event(input_dev, EV_KEY, keymap[id], 0);
			input_sync(input_dev);
			__clear_bit(id, &data->t15_keystatus);
		}
	}
}

static void ESD_timer_function(unsigned long data){
	struct mxt_data *mxt_data_p = (void *)data;

	MXT_LOG("mxt ESD_timer_function run %s\n", __func__);
	mod_timer(&ESD_timer, jiffies +  HZ * 2); /* check the time stamp every 2 heart beat */

	schedule_work(&mxt_data_p->mxt_work);
}

static void ESD_timer_init(struct mxt_data *data){
	MXT_LOG("mxt ESD_timer_init\n");
	init_timer(&ESD_timer);
	ESD_timer.function = ESD_timer_function;
	ESD_timer.data = (unsigned long)data;
	ESD_timer.expires = jiffies + 2 * HZ;

	add_timer(&ESD_timer);
	ESD_timer_running = true;
}

static void mxt_start(struct mxt_data *data)
{
	if (!data->suspended || data->in_bootloader || data->updating_config) {
		return;
	}

	MXT_LOG("mxt: in mxt_start\n");
	if (data->is_ambient_mode && data->enable_wakeup_gesture) {
		data->suspended = false;
		mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
		MXT_LOG("mxt: ambient mode mxt_start\n");
		mxt_soft_reset(data);
		return;
	}

#ifdef MAX1_WAKEUP_GESTURE_ENABLE
	if (data->enable_wakeup_gesture) {
		mxt_set_t7_power_cfg(data, MXT_POWER_CFG_RESTORE, false);
		mxt_config_ctrl_set(data, data->T100_address, 0x02);
		mxt_config_ctrl_set(data, data->T70_address, 0x02);
		mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
		data->suspended = false;
		MXT_LOG("mxt: in mxt_start enable wake gesture=1\n");
		mxt_soft_reset(data);
		return;
	}
#endif

	/* restore the setting of t7 */
	mxt_set_t7_power_cfg(data, MXT_POWER_CFG_RESTORE, false);
	mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
	data->suspended = false;
	mxt_soft_reset(data);
	return;
}


static void mxt_stop(struct mxt_data *data)
{
	input_report_key(data->input_dev, BTN_TOUCH, 0);
	input_sync(data->input_dev);
	MXT_LOG("start in mxt_stop\n");

	if (data->suspended || data->in_bootloader || data->updating_config || (data->is_ambient_mode && data->enable_wakeup_gesture)) {
		if (data->is_ambient_mode && data->enable_wakeup_gesture)
			data->suspended = true;

		mxt_reset_slots(data);
		return;
	}

	MXT_LOG("mxt: in mxt_stop, is_ambient_mode=%d\n", data->is_ambient_mode);
#ifdef MAX1_WAKEUP_GESTURE_ENABLE
	if (data->enable_wakeup_gesture) {
		mxt_set_t7_power_cfg(data, MXT_POWER_CFG_PARTIAL_DEEPSLEEP, true);
		mxt_config_ctrl_clear(data, data->T100_address, 0x02);
		//mxt_config_ctrl_clear(data, data->T70_address, 0x02);   //enable palm detection in back screen(tap-wake enable).

		if(!data->T24_tap_wake_enable)
			//clear T24 tap
			mxt_mask_byte(data, data->T24_address, 0x2, 0x04);
		else
			mxt_unmask_byte(data, data->T24_address, 0x2, 0x04);

		if(!data->T24_double_tap_wake_enable)
			//clear T24 double tap
			mxt_mask_byte(data, data->T24_address, 0x2, 0x08);
		else
			mxt_unmask_byte(data, data->T24_address, 0x2, 0x08);

		data->suspended = true;
		MXT_LOG("mxt: in mxt_stop enable_wakeup_gesture=1\n");
		mxt_reset_slots(data);
		return;
	}
#endif

	mxt_set_t7_power_cfg(data, MXT_POWER_CFG_FULL_DEEPSLEEP, true);
	mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
	mxt_reset_slots(data);
	data->suspended = true;

	return;

}

static int mxt_input_open(struct input_dev *dev)
{
	//	mxt_start(data);

	return 0;
}

static void mxt_input_close(struct input_dev *dev)
{
	struct mxt_data *data = input_get_drvdata(dev);
	struct device *device = &data->client->dev;

	dev_err(device, "%s\n", __func__);
	//mxt_stop(data);
}

static void mxt_release_config_mem(struct mxt_config_info *info)
{
	if (info->config)
		kfree(info->config);

	return;
}

/*
 * Atmel Raw Config File Format
 *
 * The first four lines of the raw config file contain:
 *  1) Version
 *  2) Chip ID Information (first 7 bytes of device memory)
 *  3) Chip Information Block 24-bit CRC Checksum
 *  4) Chip Configuration 24-bit CRC Checksum
 *
 * The rest of the file consists of one line per object instance:
 *   <TYPE> <INSTANCE> <SIZE> <CONTENTS>
 *
 *  <TYPE> - 2-byte object type as hex
 *  <INSTANCE> - 2-byte object instance number as hex
 *  <SIZE> - 2-byte object size as hex
 *  <CONTENTS> - array of <SIZE> 1-byte hex values
 */
static int mxt_cfg_check_hdr_crc(struct mxt_data *data, char **config, u32 *file_crc_p)
{
	struct i2c_client *client = data->client;
	struct device *dev = &client->dev;
	char *token;
	int ret = 0;
	u32 crc;

	/* Process the first four lines of the file*/
	/* 1) Version */
	token = strsep(config, "\n");
	if (!token) {
		dev_err(dev, "Invalid config file: no version info\n");
		return -EINVAL;
	}
	dev_info(dev, "mxt Config File: Version = %s\n", token ?: "<null>");

	/* 2) Chip ID */
	token = strsep(config, "\n");

	/* 3) Info Block CRC */
	token = strsep(config, "\n");

	dev_info(dev, "mxt Config File: Info Block CRC = %s\n", token);

	/* 4) Config CRC */
	token = strsep(config, "\n");
	if (!token) {
		dev_err(dev, "mxt Invalid config file: No Config CRC\n");
		return -EINVAL;
	}
	ret = sscanf(token, "%x", &crc);
	dev_info(dev, "mxt Config File: Config CRC = %06x\n", crc);
	if (ret != 1) {
		dev_err(dev, "Invalid config file: Bad Config CRC\n");
		return -EINVAL;
	}

	*file_crc_p = crc;

	return 0;
}

static int mxt_parse_cfg_and_load(struct mxt_data *data, struct mxt_config_info *info, bool force)
{
	int rc, ret = 0;
	bool is_need_to_update_cfg = false;
	struct mxt_info *infor = data->info;
	u8 *config_mem = NULL;
	struct device *dev = &data->client->dev;

	const struct firmware *cfg;
	char *cfg_copy = NULL;

	const char *local_cfg_name = data->pdata->cfg_kotl_g;
	u32 file_crc = 0x0;

	/* TBD Add one name judgement here */

	//read pts information here and judge cfg_name value

	/* Byte0: TP/ST Maker  	  */
	/* Byte1: Disaplay Maker  */
	/* Byte2: HW Version 	  */
	/* Byte3: Back Cover  	  */
	/* Byte4: Bootloalder version */
	/* Byte5: FW Version   	  */
	/* Byte6: Reserved	  */
	/* Byte7: Reserved	  */

	if (infor->version == 0x05 && (infor->build & 0xF0) == 0xE0) {		/* For A1070 fw only support manual update*/
		MXT_LOG("mxt: The A1070 version, only support manual update cfg\n");
		return false;
	}

	mxt_read_lockdown_info(data);

	MXT_LOG("mxt lockdown info: %x %x %x %x %x %x %x %x\n", (unsigned int)data->lockdown_info[0], (unsigned int)data->lockdown_info[1], (unsigned int)data->lockdown_info[2], (unsigned int)data->lockdown_info[3], 
			(unsigned int)data->lockdown_info[4], (unsigned int)data->lockdown_info[5], (unsigned int)data->lockdown_info[6], (unsigned int)data->lockdown_info[7]);
	if((data->lockdown_info[0] | data->lockdown_info[1] | data->lockdown_info[2] | data->lockdown_info[3] |
			data->lockdown_info[4] | data->lockdown_info[5] | data->lockdown_info[6] | data->lockdown_info[7]) ==  0x0) {
		MXT_LOG("mxt: lockdown information blank, quit update cfg directly\n");
		return 0;
	}

	if (data->lockdown_info[0] == VENDOR_KOTL) {
		if (data->lockdown_info[2] == HW_LILY_LOTUS) // for lily & lotus
			local_cfg_name = data->pdata->lily_lotus_ktol_g;
		else {
			if (data->lockdown_info[3] == BACK_COVER_GLASS)
				local_cfg_name = data->pdata->cfg_kotl_g;
			else if (data->lockdown_info[3] == BACK_COVER_SAPPHIRE)
				local_cfg_name = data->pdata->cfg_kotl_s;
		}
	} else if (data->lockdown_info[0] == VENDOR_LENS) {
		if (data->lockdown_info[3] == BACK_COVER_GLASS)
			local_cfg_name = data->pdata->cfg_lens_g;
		else if (data->lockdown_info[3] == BACK_COVER_SAPPHIRE)
			local_cfg_name = data->pdata->cfg_lens_s;
	}else
		local_cfg_name = data->pdata->cfg_kotl_g;

	MXT_LOG("mxt: open config file %s\n", local_cfg_name);

	ret = request_firmware(&cfg, local_cfg_name, dev);
	if (ret < 0) {
		dev_err(dev, "mxt: Unable to open config file %s\n", local_cfg_name);
		ret = -ENOENT;
		return ret;
	}

	/* Make a mutable, '\0'-terminated copy of the config file */
	cfg_copy = kmalloc(cfg->size + 1, GFP_KERNEL);
	if (!cfg_copy) {
		ret = -ENOMEM;
		return ret;
	}
	memcpy(cfg_copy, cfg->data, cfg->size);
	cfg_copy[cfg->size] = '\0';

	/* Verify config file header and get crc value */
	ret = mxt_cfg_check_hdr_crc(data, &cfg_copy, &file_crc);
	if (ret) {
		dev_err(dev, "Error verifying config header (%d)\n", ret);
		kfree(cfg_copy);
	}

	if(file_crc != data->config_crc)
	{
		MXT_LOG("mxt file_crc: 0x%x, data->config_crc: 0x%x\n", file_crc, data->config_crc);
		MXT_LOG("%s, need to update config\n", __func__);
		is_need_to_update_cfg = true;
	}

	if (force) {
		is_need_to_update_cfg = true;
	}

	if(!is_need_to_update_cfg)
	{
		MXT_LOG("%s, mxt: the same config crc 0x%x, no need to update\n", __func__, file_crc);
		return 0;
	}

	data->updating_config = true;

	mxt_free_input_device(data);

	if (data->suspended) {
		if (data->use_regulator) {
			mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
			mxt_regulator_enable(data);
		} else {
			mxt_set_t7_power_cfg(data, MXT_POWER_CFG_FULL_RUN, false);
			mxt_acquire_irq(data);
		}

		data->suspended = false;
	}

	ret = mxt_configure_objects(data, cfg);
	if (ret)
	{
		dev_err(dev, "%s, mxt: update config failed\n",__func__);
	}

	rc = mxt_read_t38_object(data);
	if (rc) {
		dev_err(dev, "%s: Failed to read t38 object\n",__func__);
		return rc;
	}
	rc = mxt_register_input_device(data);
	if (rc) {
		dev_err(dev, "Failed to register input device\n");
		return rc;
	}
	MXT_LOG("mxt: update config successfully\n");

	mxt_release_config_mem(info);
	if (config_mem)
		kfree(config_mem);
	data->updating_config = false;
	return 0;
}

static bool is_need_to_update_fw(struct mxt_data *data)
{
	struct mxt_info *info = data->info;
	unsigned int version, build;
	int rc;
	const char *fw_name = data->pdata->fw_version;
	struct device *dev = &data->client->dev;

	rc = sscanf(fw_name, "%2x%2x", &version, &build);
	MXT_LOG("mxt: chip->version: 0x%x, chip->build: 0x%x, filename version: 0x%x, filename build: 0x%x\n",
			info->version, info->build, version, build);
	if (rc != 2) {
		dev_err(dev, "Can't get the fw version from fw file\n");
		return false;
	}

	if(data->update_force)
	{
		MXT_LOG("mxt: force update fw and cfg!\n");
		return true;
	}

	if (info->version == version && info->build == build) {
		MXT_LOG("mxt: The same fw version\n");
		return false;
	}

	if (info->version == 0x05 && (info->build & 0xF0) == 0xE0) {		/* For A1070 fw only support manual update*/
		MXT_LOG("mxt: The A1070 version, only support manual update\n");
		return false;
	}

	if((info->version != version) ||
			((info->version == version) && (info->build != build)))
	{
		MXT_LOG("mxt: need to update fw\n");
		return true;
	}

	return false;
}

static int mxt_update_fw(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	struct device *dev = &client->dev;
	const char *fw_version = data->pdata->fw_version;
	char fw_name[MXT_FW_NAME_SIZE];
	int ret;

	// Get the fw name from parsing dts file
	//ret = snprintf(fw_name, MXT_FW_NAME_SIZE, "mXT640T_V%4s.fw", fw_version);
	ret = snprintf(fw_name, MXT_FW_NAME_SIZE, "%4s", fw_version);
	if (ret != MXT_FW_NAME_SIZE) {
		//dev_err(dev, "TP: %s, Failed to get fw name from DTS\n", __func__);
		//return ret;
	}

	MXT_LOG("mxt: %s: fw_name: %s, ret: %d\n", __func__, fw_name, ret);

	ret = __mxt_update_fw(&client->dev, fw_name, strlen(fw_name));
	if (ret) {
		dev_err(dev, "unable to update firmware\n");
		return ret;
	}

	return 0;
}

static int mxt_update_fw_and_cfg(void *args)
{
	int error;
	struct mxt_data *data = (struct mxt_data *)args;
	struct device *dev = &data->client->dev;
	char boot_mode = get_boot_mode();
	if (boot_mode == META_BOOT) {
		data->staying = HAVE_UPDATED;
		dev_dbg(dev, "%s: Don't update fw in META_BOOT\n", __func__);
		return 0;
	}

	data->staying = IS_UPDATINGE;
	if (is_need_to_update_fw(data)) {
		error = mxt_update_fw(data);
		if (error) {
			dev_err(dev, "mxt: %s: Failed to update fw\n", __func__);
			goto update_fail;
		}

		error = mxt_parse_cfg_and_load(data,&data->pdata->info, true);
		if (error) {
			dev_err(dev, "mxt: Failed to update config, after firmware updating\n");
			goto update_fail;
		}
	} else {
		error = mxt_parse_cfg_and_load(data, &data->pdata->info, false);
		if (error) {
			dev_err(dev, "mxt: Failed to update config\n");
			goto update_fail;
		}
	}

	data->staying = HAVE_UPDATED;

	msleep(100);

	mutex_lock(&data->esd_timer_lock);
	MXT_LOG("%s %d lock ESD_timer_running:%d\n", __func__, __LINE__, ESD_timer_running);
#ifdef CONFIG_MXT_T61_HR
	mxt_set_obj_t61(data, &t61, 0);
#endif
	if(ESD_timer_running == false)
		ESD_timer_init(data);
	MXT_LOG("%s %d unlock ESD_timer_running:%d\n", __func__, __LINE__, ESD_timer_running);
	mutex_unlock(&data->esd_timer_lock);
	return 0;

update_fail:
	data->staying = HAVE_UPDATED;
	msleep(100);

	mutex_lock(&data->esd_timer_lock);
	MXT_LOG("%s %d lock ESD_timer_running:%d\n", __func__, __LINE__, ESD_timer_running);
#ifdef CONFIG_MXT_T61_HR
	mxt_set_obj_t61(data, &t61, 0);
#endif
	if(ESD_timer_running == false)
		ESD_timer_init(data);
	MXT_LOG("%s %d unlock ESD_timer_running:%d\n", __func__, __LINE__, ESD_timer_running);
	mutex_unlock(&data->esd_timer_lock);
	return -EIO;
}


static DEVICE_ATTR(touch_debug, S_IWUSR|S_IWGRP, NULL, touch_debug_store);
static DEVICE_ATTR(fw_version, S_IRUSR|S_IRGRP, mxt_fw_version_show, NULL);
static DEVICE_ATTR(cfg_version, S_IRUSR|S_IRGRP, mxt_cfg_version_show, NULL);
static DEVICE_ATTR(hw_version, S_IRUSR|S_IRGRP, mxt_hw_version_show, NULL);
static DEVICE_ATTR(object, S_IRUSR|S_IRGRP, mxt_object_show, NULL);
static DEVICE_ATTR(update_fw, S_IWUSR|S_IWGRP, NULL, mxt_update_fw_store);
static DEVICE_ATTR(update_cfg, S_IWUSR|S_IWGRP, NULL, mxt_update_cfg_store);
static DEVICE_ATTR(mxt_update, S_IRUSR|S_IRGRP, mxt_update_show, NULL);
static DEVICE_ATTR(reset, S_IWUSR|S_IWGRP, NULL, mxt_reset_store);
static DEVICE_ATTR(debug_v2_enable, S_IWUSR|S_IWGRP, NULL,
		mxt_debug_v2_enable_store);
static DEVICE_ATTR(debug_notify, S_IRUSR|S_IRGRP, mxt_debug_notify_show, NULL);
static DEVICE_ATTR(debug_enable, S_IRUSR|S_IRGRP|S_IWUSR|S_IWGRP, mxt_debug_enable_show,
		mxt_debug_enable_store);
static DEVICE_ATTR(config_csum, S_IRUSR|S_IRGRP, mxt_config_csum_show, NULL);
static DEVICE_ATTR(sys_suspend, S_IRUSR|S_IRGRP|S_IWUSR|S_IWGRP, mxt_sys_suspend_show, mxt_sys_suspend_store);
static DEVICE_ATTR(open_circuit_test, S_IRUSR|S_IRGRP, mxt_open_circuit_test_show, NULL);
static DEVICE_ATTR(reference, S_IRUSR|S_IRGRP, mxt_reference_data_show, NULL);
static DEVICE_ATTR(delta, S_IRUSR|S_IRGRP, mxt_delta_data_show, NULL);
static DEVICE_ATTR(selfcap_delta, S_IRUSR|S_IRGRP, mxt_selfcap_delta_show, NULL);
static DEVICE_ATTR(selfcap_ref, S_IRUSR|S_IRGRP, mxt_selfcap_ref_show, NULL);

static DEVICE_ATTR(noise_data_test, S_IRUSR|S_IRGRP, mxt_noise_data_show, NULL);
static DEVICE_ATTR(ambient, S_IRUSR|S_IRGRP|S_IWUSR|S_IWGRP, mxt_ambient_show, mxt_ambient_store);

#ifdef MAX1_WAKEUP_GESTURE_ENABLE
static DEVICE_ATTR(double_click_wake_enable, S_IRUSR|S_IRGRP|S_IWUSR|S_IWGRP, mxt_double_click_wake_enable_show, mxt_double_click_wake_enable_store);
static DEVICE_ATTR(down_to_up_wake_enable, S_IRUSR|S_IRGRP|S_IWUSR|S_IWGRP, mxt_down_to_up_wake_enable_show, mxt_down_to_up_wake_enable_store);
static DEVICE_ATTR(up_to_down_wake_enable, S_IRUSR|S_IRGRP|S_IWUSR|S_IWGRP, mxt_up_to_down_wake_enable_show, mxt_up_to_down_wake_enable_store);
static DEVICE_ATTR(right_to_left_wake_enable, S_IRUSR|S_IRGRP|S_IWUSR|S_IWGRP, mxt_right_to_left_wake_enable_show, mxt_right_to_left_wake_enable_store);
static DEVICE_ATTR(left_to_right_wake_enable, S_IRUSR|S_IRGRP|S_IWUSR|S_IWGRP, mxt_left_to_right_wake_enable_show, mxt_left_to_right_wake_enable_store);
static DEVICE_ATTR(detected_gesture, S_IRUSR|S_IRGRP, mxt_detected_gesture_show, NULL);
static DEVICE_ATTR(suspend, S_IRUSR|S_IRGRP|S_IWUSR|S_IWGRP, mxt_suspend_show, mxt_suspend_store);
#endif

static DEVICE_ATTR(pts, S_IRUSR|S_IRGRP|S_IWUSR|S_IWGRP, mxt_lockdown_show, mxt_lockdown_store);

static DEVICE_ATTR(range, S_IRUSR|S_IRGRP|S_IWUSR|S_IWGRP, mxt_range_show, mxt_range_store);

static struct attribute *mxt_attrs[] = {
	&dev_attr_fw_version.attr,
	&dev_attr_touch_debug.attr,
	&dev_attr_cfg_version.attr,
	&dev_attr_hw_version.attr,
	&dev_attr_object.attr,
	&dev_attr_update_fw.attr,
	&dev_attr_update_cfg.attr,
	&dev_attr_reset.attr,
	&dev_attr_debug_enable.attr,
	&dev_attr_debug_v2_enable.attr,
	&dev_attr_debug_notify.attr,
	&dev_attr_config_csum.attr,
	&dev_attr_sys_suspend.attr,
	&dev_attr_open_circuit_test.attr,
	&dev_attr_reference.attr,
	&dev_attr_delta.attr,
	&dev_attr_selfcap_delta.attr,
	&dev_attr_selfcap_ref.attr,
	&dev_attr_noise_data_test.attr,
	&dev_attr_mxt_update.attr,
#ifdef MAX1_WAKEUP_GESTURE_ENABLE
	&dev_attr_double_click_wake_enable.attr,
	&dev_attr_down_to_up_wake_enable.attr,
	&dev_attr_up_to_down_wake_enable.attr,
	&dev_attr_right_to_left_wake_enable.attr,
	&dev_attr_left_to_right_wake_enable.attr,
	&dev_attr_detected_gesture.attr,
	&dev_attr_suspend.attr,
#endif
	&dev_attr_pts.attr,
	&dev_attr_range.attr,
	&dev_attr_ambient.attr,
	NULL
};

static const struct attribute_group mxt_attr_group = {
	.attrs = mxt_attrs,
};


static int mxt_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct mxt_data *data;
	unsigned char attr_count;
	struct proc_dir_entry *mxt_config_proc = NULL;
	int error;
	int retval;
	data = kzalloc(sizeof(struct mxt_data), GFP_KERNEL);
	if (!data) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}
	mxt_i2c_data = data;

	wake_lock_init(&touch_irq_lock, WAKE_LOCK_SUSPEND, "touch irq wakelock");

	mt_set_gpio_mode(GPIO61, 0);
	mt_set_gpio_mode(GPIO62, 0);
	mt_set_gpio_dir(GPIO61, GPIO_DIR_OUT);
	mt_set_gpio_dir(GPIO62, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO61, GPIO_OUT_ZERO);
	mt_set_gpio_out(GPIO62, GPIO_OUT_ZERO);
	msleep(10);
	mt_set_gpio_out(GPIO61, GPIO_OUT_ONE);
	mt_set_gpio_out(GPIO62, GPIO_OUT_ONE);
	msleep(50);

	mutex_init(&data->bus_access_mutex);
	mutex_init(&data->esd_timer_lock);

	snprintf(data->phys, sizeof(data->phys), "i2c-%u-%04x/input0",
			client->adapter->nr, client->addr);

	data->client = client;
	data->pdata = &mxt_platform_data;
	if(!data->pdata)
	{
		MXT_LOG("\n");
	}
	i2c_set_clientdata(client, data);

#if defined(CONFIG_MXT_I2C_DMA)
	client->addr |= I2C_RS_FLAG | I2C_ENEXT_FLAG | I2C_DMA_FLAG;
	client->dev.coherent_dma_mask = DMA_BIT_MASK(32);
	data->i2c_dma_va = (u8 *)dma_alloc_coherent(&client->dev, PAGE_SIZE * 2, &data->i2c_dma_pa, GFP_KERNEL);
	if (!data->i2c_dma_va)	{
		error = -ENOMEM;
		dev_err(&client->dev, "Allocate DMA I2C Buffer failed!\n");
		goto err_free_mem;
	}
#endif
#ifdef CONFIG_OF
	if (!data->pdata && client->dev.of_node)
	{
		data->pdata = mxt_parse_dt(client);
	}
#endif

	if (!data->pdata) {
		data->pdata = devm_kzalloc(&client->dev, sizeof(*data->pdata), GFP_KERNEL);
		if (!data->pdata) {
			dev_err(&client->dev, "Failed to allocate pdata\n");
			error = -ENOMEM;
			goto err_free_mem;
		}

		/* Set default parameters */
		data->pdata->irqflags = IRQF_TRIGGER_LOW;
	}

	data->staying = HAVE_UPDATED;

	init_completion(&data->bl_completion);
	init_completion(&data->reset_completion);
	init_completion(&data->crc_completion);
	mutex_init(&data->debug_msg_lock);

	thread = kthread_run(touch_event_handler, data, "atmel-mxt-tp");
	if ( IS_ERR(thread) ) {
		retval = PTR_ERR(thread);
		pr_err(" %s: failed to create kernel thread: %d\n",__func__, retval);
	}

	/* power supply */
	/* set reset output 0 	*/
	/* power up vdd/avdd	*/
	/* msleep(50);		*/
	/* set reset output 1 	*/
	/* msleep(200);		*/
	mt_set_gpio_mode(AT_GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	mt_set_gpio_dir(AT_GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(AT_GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
	msleep(10);
	//hwPowerOn(MT6323_POWER_LDO_VGP1, VOL_3300, "TP");
	msleep(50);
	mt_set_gpio_out(AT_GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
	msleep(200);

	mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
	mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_DISABLE);

	error = mxt_initialize(data);
	if (error) {
		dev_err(&client->dev, "Failed to initialize device\n");
		goto err_free_object;
	}

	mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINTF_TRIGGER_LOW, mxt_interrupt, 0);

	/* Determine whether to update fw and config */

	thread = kthread_run(mxt_update_fw_and_cfg, (void *)data, "atmel_fw_update_tp");
	data = dev_get_drvdata(&client->dev);

	if (IS_ERR(thread))
	{
		retval = PTR_ERR(thread);
		pr_err(" %s: failed to create kernel thread: %d\n",__func__, retval);
		return -1;
	}

	error = sysfs_create_group(&client->dev.kobj, &mxt_attr_group);
	if (error) {
		dev_err(&client->dev, "Failure %d creating sysfs group\n",
				error);
		goto err_free_object;
	}

	for(attr_count = 0; attr_count < ARRAY_SIZE(sproc_file); attr_count++)
	{
		mxt_config_proc = proc_create(sproc_file[attr_count]->name,
				sproc_file[attr_count]->mode, NULL, &(sproc_file[attr_count]->fop));
		if(!mxt_config_proc)
		{
			dev_err(&client->dev,
					"%s: Failed to create procfs entry\n",
					__func__);
		}
	}
	sysfs_bin_attr_init(&data->mem_access_attr);
	data->mem_access_attr.attr.name = "mem_access";
	data->mem_access_attr.attr.mode = S_IRUGO | S_IWUSR;
	data->mem_access_attr.read = mxt_mem_access_read;
	data->mem_access_attr.write = mxt_mem_access_write;
	data->mem_access_attr.size = data->mem_size;

	if (sysfs_create_bin_file(&client->dev.kobj,
				&data->mem_access_attr) < 0) {
		dev_err(&client->dev, "Failed to create %s\n",
				data->mem_access_attr.attr.name);
		goto err_remove_sysfs_group;
	}

	INIT_WORK(&data->mxt_work, mxt_timer_work);
#if defined(CONFIG_FB_PM)
	INIT_WORK(&data->fb_notify_work, fb_notify_resume_work);
	data->fb_notif.notifier_call = fb_notifier_callback;
	error = fb_register_client(&data->fb_notif);
	if (error) {
		dev_err(&client->dev,
				"Unable to register fb_notifier: %d\n", error);
		goto err_remove_mem_access_attr;
	}
#endif

	data->ambient_notif.notifier_call = ambient_notifier_callback;
	error = ambient_mode_notifier_register(&data->ambient_notif);
	if (error) {
		dev_err(&client->dev,
				"Unable to register ambient_notifier: %d\n", error);
		goto err_remove_mem_access_attr;
	}

#if !defined(CONFIG_FB_PM)
	data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 2;
	data->early_suspend.suspend = mxt_tpd_suspend;
	data->early_suspend.resume = mxt_tpd_resume;
	register_early_suspend(&data->early_suspend);
#endif
	tpd_load_status = 1;

#ifdef CONFIG_MXT_T61_HR
	mxt_t61_init(data, &t61);
#endif

	data->double_click_wake_enable = true;
	data->enable_wakeup_gesture = true;
	data->T24_tap_wake_enable = true;
	data->T24_double_tap_wake_enable = true;

#if defined(CONFIG_FB)
	if ((get_boot_mode() == KERNEL_POWER_OFF_CHARGING_BOOT) || (get_boot_mode() == LOW_POWER_OFF_CHARGING_BOOT))
		mxt_tpd_suspend(NULL);
#endif

	printk(KERN_DEBUG "mxt touch probe successful.\n");
	return 0;

#if defined(CONFIG_FB_PM)
err_remove_mem_access_attr:
	if(!gpDMABuf_va){
		dma_free_coherent(&(client->dev), GTP_DMA_MAX_TRANSACTION_LENGTH, gpDMABuf_va, gpDMABuf_pa);
		gpDMABuf_va = NULL;
		gpDMABuf_pa = 0;
		return -1;
	}
	sysfs_remove_bin_file(&client->dev.kobj, &data->mem_access_attr);
#endif
err_remove_sysfs_group:
	sysfs_remove_group(&client->dev.kobj, &mxt_attr_group);

err_free_object:
	mxt_free_object_table(data);

	free_irq(client->irq, data);

err_free_mem:
#if defined(CONFIG_MXT_I2C_DMA)
	dma_free_coherent(&client->dev, PAGE_SIZE * 2, data->i2c_dma_va, data->i2c_dma_pa);
#endif
	kfree(data);
	return error;
}

static int mxt_remove(struct i2c_client *client)
{
	struct mxt_data *data = i2c_get_clientdata(client);
	tpd_load_status = 0;
	if (data->mem_access_attr.attr.name)
		sysfs_remove_bin_file(&client->dev.kobj,
				&data->mem_access_attr);
	//ambient_mode_notifier_unregister(&data->ambient_notif);
	sysfs_remove_group(&client->dev.kobj, &mxt_attr_group);
#if defined(CONFIG_MXT_I2C_DMA)
	dma_free_coherent(&client->dev, PAGE_SIZE * 2, data->i2c_dma_va, data->i2c_dma_pa);
#endif
	free_irq(client->irq, data);
	regulator_put(data->reg_avdd);
	regulator_put(data->reg_vdd);
	mxt_free_object_table(data);
	kfree(data);

	return 0;
}

static bool is_need_reboot(struct mxt_data *data, int id)
{
	short *report_data_16;
	int i, max_value = 0, min_value = 0, num = 0;
	bool check_result = false;

	if(id == ESD_CHECK_SELFCAP_DELTA_ID)		/* Check the selfcap deltas value*/
	{
		read_raw_data(data, MXT_T6_CMD_SELFCAP_DELTAS);
		report_data_16 = data->raw_data_16;

		for (i = 0; i < (TX_NUM + RX_NUM); i++) {
			//MXT_LOG("raw_data[%d] = %d,\n", i, (short)(mxt_data_p->raw_data_16[i]));
			//MXT_LOG("report_data[%d] = %d,\n", i, *report_data_16);
			max_value =  (*report_data_16 > max_value ) ? *report_data_16 : max_value;
			min_value =  (*report_data_16 < min_value ) ? *report_data_16 : min_value;
			report_data_16++;
		}

		MXT_LOG("mxt id: %d, max_value: %d, min_value: %d\n", id, max_value, min_value);

		if((max_value > ESD_SELFCAP_DELTA_LIMIT_HIGH) || (min_value < ESD_SELFCAP_DELTA_LIMIT_LOW))
		{
			MXT_LOG("mxt going to reboot id: %d  max_value: %d  min_value: %d\n", id, max_value, min_value);
			check_result = true;
		}
	}else if(id == ESD_CHECK_SELFCAP_REF_ID)	/* Check the selfcap ref value */
	{
		read_raw_data(data, MXT_T6_CMD_SELFCAP_REFS);
		report_data_16 = data->raw_data_16;

		for (i = 0; i < (TX_NUM + RX_NUM); i++) {
			if ((*report_data_16 > esd_seflcap_ref_limit_high) || (( *report_data_16 > -32760 ) && (*report_data_16 < esd_seflcap_ref_limit_low)))
				++num;
			report_data_16++;
		}

		MXT_LOG("mxt id: %d, bad selfcap number: %d\n", id, num);

		if (num > 2)
		{
			MXT_LOG("mxt going to reboot id: %d\n", id);
			check_result = true;
		} else {
			check_result = false;
		}
	}else if (id == ESD_CHECK_MUTUALCAP_REF_ID)	/* check the mutual ref value */
	{
		read_raw_data(data, MXT_T6_CMD_REFS);
		report_data_16 = data->raw_data_16;

		for (i = 0; i < (TX_NUM * RX_NUM); i++) {
			max_value =  (*report_data_16 > max_value ) ? *report_data_16 : max_value;
			min_value =  (*report_data_16 < min_value ) ? *report_data_16 : min_value;
			report_data_16++;
		}

		MXT_LOG("mxt id: %d, max_value: %d, min_value: %d\n", id, max_value, min_value);

		if(max_value < ESD_MUTUALCAP_REF_LIMIT_LOW)
		{
			MXT_LOG("mxt going to reboot id: %d  max_value: %d  min_value: %d\n", id, max_value, min_value);
			check_result = true;
		}


	}else 			/* Invalid ID*/
	{
			MXT_LOG("mxt invalid id: %d\n", id);
			check_result = false;
	}
	return check_result;
}

static void mxt_timer_work(struct work_struct *work)
{
	bool need_soft_reboot = false;
	struct mxt_data *mxt_data_p = container_of(work, struct mxt_data, mxt_work);

#ifdef CONFIG_MXT_T61_HR
	heart_beat_time_store[0] = heart_beat_time_store[1];
	heart_beat_time_store[1] = heart_beat_time_store[2];
	heart_beat_time_store[2] = heart_beat_time_stamp;

	if((heart_beat_time_store[0] == heart_beat_time_store[1]) && (heart_beat_time_store[1] == heart_beat_time_store[2])){
		MXT_LOG("mxt heart beaten set power_reboot. %lld\n", (long long)heart_beat_time_stamp);
		need_power_reboot = true;
	}
#endif

	need_power_reboot = is_need_reboot(mxt_data_p, ESD_CHECK_SELFCAP_DELTA_ID);

	if(need_power_reboot)
	{
		power_reboot(mxt_data_p);
		need_power_reboot = false;
		return;
	}

	if(tmp_check_mt_ref){
		need_soft_reboot = is_need_reboot(mxt_data_p, ESD_CHECK_SELFCAP_REF_ID) \
					|| is_need_reboot(mxt_data_p, ESD_CHECK_MUTUALCAP_REF_ID);
	}else {
		need_soft_reboot = is_need_reboot(mxt_data_p, ESD_CHECK_SELFCAP_REF_ID);
	}

	if(need_soft_reboot)
	{
		mxt_soft_reset(mxt_data_p);
	}

}

#ifdef MAX1_WAKEUP_GESTURE_ENABLE
static void mxt_set_gesture(struct mxt_data *mxt, bool enable)
{
	if (enable) {
		/* Enable gesture, double click, event reporting */
		mxt_config_ctrl_set(mxt, mxt->T93_address, 0x02);

		/* Enable gesture, up, down, left, right, event reporting */
		mxt_config_ctrl_set(mxt, mxt->T115_address, 0x02);
	} else {
		/* Disable gesture, double click, event reporting */
		mxt_config_ctrl_clear(mxt, mxt->T93_address, 0x02);

		/* Disable gesture, up, down, left, right, event reporting */
		mxt_config_ctrl_clear(mxt, mxt->T115_address, 0x02);
	}
}
#endif

#if defined(CONFIG_FB_PM)
static void fb_notify_resume_work(struct work_struct *work)
{
	struct mxt_data *mxt =
		container_of(work,
				struct mxt_data, fb_notify_work);
	mxt_resume(&(mxt->input_dev->dev));

	/* Enable general touch event reporting */
	mxt_config_ctrl_set(mxt, mxt->T100_address, 0x02);
#ifdef MAX1_WAKEUP_GESTURE_ENABLE
	if (mxt->enable_wakeup_gesture)
		mxt_set_gesture(mxt, false);
#endif
}

static int fb_notifier_callback(struct notifier_block *self,
		unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct mxt_data *mxt = container_of(self, struct mxt_data, fb_notif);

	if (evdata && evdata->data && event == FB_EVENT_BLANK &&
			mxt && mxt->client) {
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK) {
			wakeup_event_mask = 0;
			mxt->is_ambient_mode = false;
			mxt_tpd_resume(NULL);
			MXT_LOG("is_ambient_mode change to 0\n");
			return 0;
#ifdef MAX1_WAKEUP_GESTURE_ENABLE
			mxt->suspended = 0;
#endif

			printk(KERN_DEBUG
					"TP: %s(), FB_BLANK_UNBLANK\n",
					__func__);

			schedule_work(&mxt->fb_notify_work);

		} else if (*blank == FB_BLANK_POWERDOWN) {
			MXT_LOG("start mxt_tpd_suspend\n");
			mxt_tpd_suspend(NULL);
			return 0;
			if (flush_work(&mxt->fb_notify_work))
				pr_warn("%s: waited resume worker finished\n",
						__func__);

			printk(KERN_DEBUG
					"TP: %s(), FB_BLANK_POWERDOWN\n",
					__func__);

			/* Disable general touch event reporting */
			mxt_config_ctrl_clear(mxt, mxt->T100_address, 0x02);

#ifdef MAX1_WAKEUP_GESTURE_ENABLE
			mxt->suspended = 1;
			if (mxt->enable_wakeup_gesture)
				mxt_set_gesture(mxt, true);
#endif
			mxt_reset_slots(mxt);

			mxt_suspend(&(mxt->input_dev->dev));
		}
	}
	return 0;
}
#endif

#ifdef CONFIG_PM_SLEEP
static int mxt_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mxt_data *data = i2c_get_clientdata(client);
	struct input_dev *input_dev = data->input_dev;

		mutex_lock(&data->esd_timer_lock);
		MXT_LOG("%s %d lock ESD_timer_running:%d\n", __func__, __LINE__, ESD_timer_running);
		if(ESD_timer_running){
#ifdef CONFIG_MXT_T61_HR
			mxt_t61_deinit(data, &t61);
			MXT_LOG("mxt: ctrl:%d cmd: %d, mode:%d, period:%d\n",t61.ctrl, t61.cmd, t61.mode, t61.period);
			mxt_set_obj_t61(data, &t61, 0);
#endif
			del_timer_sync(&ESD_timer);
			MXT_LOG("mxt: del ESD timer\n");
			ESD_timer_running = false;
		}
		MXT_LOG("%s %d unlock ESD_timer_running:%d\n", __func__, __LINE__, ESD_timer_running);
		mutex_unlock(&data->esd_timer_lock);
	MXT_LOG("after mxt_suspend del_timer\n");

	if (input_dev->users)
	{
		mxt_stop(data);
	}

	return 0;
}

static int mxt_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mxt_data *data = i2c_get_clientdata(client);
	struct input_dev *input_dev = data->input_dev;

	//mutex_lock(&input_dev->mutex);

	mxt_reset_slots(data);
	if (input_dev->users)
	{
		mxt_start(data);
	}

	mutex_lock(&data->esd_timer_lock);
	MXT_LOG("%s %d lock ESD_timer_running:%d\n", __func__, __LINE__, ESD_timer_running);
	if(ESD_timer_running == false){
#ifdef CONFIG_MXT_T61_HR
		mxt_t61_init(data, &t61);
		mxt_set_obj_t61(data, &t61, 0);
#endif
		ESD_timer_init(data);
	}
	MXT_LOG("%s %d unlock ESD_timer_running:%d\n", __func__, __LINE__, ESD_timer_running);
	mutex_unlock(&data->esd_timer_lock);

	MXT_LOG("end of mxt_resume\n");
	//mutex_unlock(&input_dev->mutex);
	return 0;
}

static void mxt_tpd_suspend(struct early_suspend *h)
{
	struct mxt_data *data = mxt_i2c_data;

	MXT_LOG("mxt: mxt_tpd_suspend\n");

	if(data->staying == IS_UPDATINGE)
	{
		MXT_LOG("%s staying!!\n", __func__);
		return;
	}

	if(data)
	{
		mxt_suspend(&(data->client->dev));
	}
	else
	{
		MXT_LOG("mxt: mxt_data is NULL\n");
	}
}

static void mxt_tpd_resume(struct early_suspend *h)
{
	struct mxt_data *data = mxt_i2c_data;

	MXT_LOG("mxt: mxt_tpd_resume\n");

	if(data->staying == IS_UPDATINGE)
	{
		MXT_LOG("%s staying!!\n", __func__);
		return;
	}
	if(data)
		mxt_resume(&data->client->dev);
}

#endif

static const struct i2c_device_id mxt_id[] = {
	{ "atmel_mxt_ts", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mxt_id);

static struct i2c_driver tpd_i2c_driver = {
	.driver = {
		.name	= "atmel_mxt_ts",
		.owner	= THIS_MODULE,
	},
	.probe		= mxt_probe,
	.remove		= mxt_remove,
	.id_table	= mxt_id,
};

static int tpd_local_init(void)
{
	/*MXT_LOG("Atmel I2C Touchscreen Driver (Built %s @ %s)\n", __DATE__, __TIME__);*/

	if(i2c_add_driver(&tpd_i2c_driver)!=0)
	{
		MXT_LOG("tangjie Error unable to add i2c driver.\n");
		return -1;
	}
#ifdef TPD_HAVE_BUTTON
	tpd_button_setting(TPD_KEY_COUNT, tpd_keys_local, tpd_keys_dim_local);// initialize tpd button data
#endif
	if(tpd_load_status == 0) {  // disable auto load touch driver for linux3.0 porting
		MXT_LOG("mxt atmel add error touch panel driver.\n");
		i2c_del_driver(&tpd_i2c_driver);
		return -1;
	}
	return 0;
}

static struct tpd_driver_t atmel_mxt_driver = {
	.tpd_device_name = "atmel_mxt_ts",
	.tpd_local_init = tpd_local_init,
#ifdef TPD_HAVE_BUTTON
	.tpd_have_button = 1,
#else
	.tpd_have_button = 0,
#endif
};

static int __init atmel_mxt_init(void)
{
	if(i2c_add_driver(&tpd_i2c_driver)!=0)
	{
		MXT_LOG("Error unable to add i2c driver.\n");
		return -1;
	}
	return 0;
}

static void __exit atmel_mxt_exit(void)
{
	tpd_driver_remove(&atmel_mxt_driver);
	return;
}

module_init(atmel_mxt_init);
module_exit(atmel_mxt_exit);

/* Module information */
MODULE_AUTHOR("Joonyoung Shim <jy0922.shim@samsung.com>");
MODULE_DESCRIPTION("Atmel maXTouch Touchscreen driver");

