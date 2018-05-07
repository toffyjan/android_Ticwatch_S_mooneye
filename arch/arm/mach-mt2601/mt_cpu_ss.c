/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/spinlock.h>
#include <linux/kthread.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/jiffies.h>
#include <linux/seq_file.h>
#include <linux/printk.h>

#include <asm/system.h>
#include <asm/uaccess.h>

#include <mt-plat/sync_write.h>
/*#include <mt-plat/sync_write.h>*/
#include "mach/mt_typedefs.h"
#include "mach/mt_cpufreq.h"

static struct hrtimer mt_cpu_ss_timer;
struct task_struct *mt_cpu_ss_thread = NULL;
static DECLARE_WAIT_QUEUE_HEAD(mt_cpu_ss_timer_waiter);

static int mt_cpu_ss_period_s;
static int mt_cpu_ss_period_ns = 100;

static int mt_cpu_ss_timer_flag;

static bool mt_cpu_ss_debug_mode;
static bool mt_cpu_ss_period_mode;

enum hrtimer_restart mt_cpu_ss_timer_func(struct hrtimer *timer)
{
	if (mt_cpu_ss_debug_mode)
		pr_info("[%s]: enter timer function\n", __func__);

	mt_cpu_ss_timer_flag = 1;
	wake_up_interruptible(&mt_cpu_ss_timer_waiter);

	return HRTIMER_NORESTART;
}

int mt_cpu_ss_thread_handler(void *unused)
{
	kal_uint32 flag = 0;

	do {
		ktime_t ktime = ktime_set(mt_cpu_ss_period_s, mt_cpu_ss_period_ns);

		wait_event_interruptible(mt_cpu_ss_timer_waiter, mt_cpu_ss_timer_flag != 0);
		mt_cpu_ss_timer_flag = 0;

		if (!flag) {
			mt_reg_sync_writel((DRV_Reg32(TOP_CKMUXSEL) & 0x0ff3), TOP_CKMUXSEL);
			flag = 1;
		} else {
			mt_reg_sync_writel((DRV_Reg32(TOP_CKMUXSEL) | 0x0004), TOP_CKMUXSEL);
			flag = 0;
		}

		if (mt_cpu_ss_debug_mode)
			pr_info("[%s]: TOP_CKMUXSEL = 0x%x\n", __func__, DRV_Reg32(TOP_CKMUXSEL));

		hrtimer_start(&mt_cpu_ss_timer, ktime, HRTIMER_MODE_REL);

	} while (!kthread_should_stop());

	return 0;
}

static int cpu_ss_mode_read(struct seq_file *m, void *v)
{
	if ((DRV_Reg32(TOP_CKMUXSEL) & 0x000C) == 0)
		seq_puts(m, "CPU clock source is CLKSQ\n");
	else
		seq_puts(m, "CPU clock source is ARMPLL\n");

	return 0;
}

static ssize_t cpu_ss_mode_write(struct file *file, const char __user *buffer, size_t count,
				 loff_t *data)
{
	int len = 0, mode = 0;
	char desc[32];

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len)) {
		return 0;
	}
	desc[len] = '\0';

	if (sscanf(desc, "%d", &mode) == 1) {
		if (mode) {
			pr_info("[%s]: config cpu speed switch mode = ARMPLL\n", __func__);
			mt_reg_sync_writel((DRV_Reg32(TOP_CKMUXSEL) | 0x0004), TOP_CKMUXSEL);
		} else {
			pr_info("[%s]: config cpu speed switch mode = CLKSQ\n", __func__);
			mt_reg_sync_writel((DRV_Reg32(TOP_CKMUXSEL) & 0x0ff3), TOP_CKMUXSEL);
		}

		return count;
	} else {
		pr_info("[%s]: bad argument!! should be \"1\" or \"0\"\n", __func__);
	}

	return -EINVAL;
}

static int cpu_ss_period_read(struct seq_file *m, void *v)
{
	seq_printf(m, "%d (s) %d (ns)\n", mt_cpu_ss_period_s, mt_cpu_ss_period_ns);

	return 0;
}

static ssize_t cpu_ss_period_write(struct file *file, const char __user *buffer, size_t count,
				   loff_t *data)
{
	int len = 0, s = 0, ns = 0;
	char desc[32];

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len)) {
		return 0;
	}
	desc[len] = '\0';

	if (sscanf(desc, "%d %d", &s, &ns) == 2) {
		pr_info("[%s]: set cpu speed switch period = %d (s), %d (ns)\n", __func__, s, ns);
		mt_cpu_ss_period_s = s;
		mt_cpu_ss_period_ns = ns;
		return count;
	} else {
		pr_info("[%s]: bad argument!! should be \"[s]\" or \"[ns]\"\n", __func__);
	}

	return -EINVAL;
}

static int cpu_ss_period_mode_read(struct seq_file *m, void *v)
{
	if (mt_cpu_ss_period_mode)
		seq_puts(m, "enable");
	else
		seq_puts(m, "disable");

	return 0;
}

static ssize_t cpu_ss_period_mode_write(struct file *file, const char __user *buffer, size_t count,
					loff_t *data)
{
	int len = 0;
	char mode[20], desc[32];
	ktime_t ktime = ktime_set(mt_cpu_ss_period_s, mt_cpu_ss_period_ns);

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len)) {
		return 0;
	}
	desc[len] = '\0';

	if (sscanf(desc, "%s", mode) == 1) {
		if (!strcmp(mode, "enable")) {
			pr_info("[%s]: enable cpu speed switch period mode\n", __func__);
			mt_cpu_ss_period_mode = true;

			mt_cpu_ss_thread =
			    kthread_run(mt_cpu_ss_thread_handler, 0, "cpu speed switch");
			if (IS_ERR(mt_cpu_ss_thread)) {
				pr_info("[%s]: failed to create cpu speed switch thread\n",
				       __func__);
			}

			hrtimer_start(&mt_cpu_ss_timer, ktime, HRTIMER_MODE_REL);
			return count;
		} else if (!strcmp(mode, "disable")) {
			pr_info("[%s]: disable cpu speed switch period mode\n", __func__);
			mt_cpu_ss_period_mode = false;

			kthread_stop(mt_cpu_ss_thread);

			mt_reg_sync_writel((DRV_Reg32(TOP_CKMUXSEL) | 0x0004), TOP_CKMUXSEL);

			hrtimer_cancel(&mt_cpu_ss_timer);
			return count;
		} else {
			pr_info("[%s]: bad argument!! should be \"enable\" or \"disable\"\n",
			       __func__);
		}
	} else {
		pr_info("[%s]: bad argument!! should be \"enable\" or \"disable\"\n", __func__);
	}

	return -EINVAL;
}

static ssize_t cpu_ss_debug_mode_write(struct file *file, const char __user *buffer, size_t count,
				       loff_t *data)
{
	int len = 0;
	char mode[20], desc[32];

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len)) {
		return 0;
	}
	desc[len] = '\0';

	if (sscanf(desc, "%s", mode) == 1) {
		if (!strcmp(mode, "enable")) {
			pr_info("[%s]: enable cpu speed switch debug mode\n", __func__);
			mt_cpu_ss_debug_mode = true;
			return count;
		} else if (!strcmp(mode, "disable")) {
			pr_info("[%s]: disable cpu speed switch debug mode\n", __func__);
			mt_cpu_ss_debug_mode = false;
			return count;
		} else {
			pr_info("[%s]: bad argument!! should be \"enable\" or \"disable\"\n",
			       __func__);
		}
	} else {
		pr_info("[%s]: bad argument!! should be \"enable\" or \"disable\"\n", __func__);
	}

	return -EINVAL;
}

static int cpu_ss_debug_mode_read(struct seq_file *m, void *v)
{
	if (mt_cpu_ss_debug_mode)
		seq_puts(m, "enable");
	else
		seq_puts(m, "disable");

	return 0;
}

#if 0
static int cpu_ss_mode_read(char *buf, char **start, off_t off, int count, int *eof, void *data)
{
	int len = 0;
	char *p = buf;

	if ((DRV_Reg32(TOP_CKMUXSEL) & 0x000C) == 0)
		p += sprintf(p, "CPU clock source is CLKSQ\n");
	else
		p += sprintf(p, "CPU clock source is ARMPLL\n");

	*start = buf + off;

	len = p - buf;
	if (len > off)
		len -= off;
	else
		len = 0;

	return len < count ? len : count;
}

static ssize_t cpu_ss_mode_write(struct file *file, const char *buffer, unsigned long count,
				 void *data)
{
	int len = 0, mode = 0;
	char desc[32];

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len)) {
		return 0;
	}
	desc[len] = '\0';

	if (sscanf(desc, "%d", &mode) == 1) {
		if (mode) {
			pr_info("[%s]: config cpu speed switch mode = ARMPLL\n", __func__);
			mt_reg_sync_writel((DRV_Reg32(TOP_CKMUXSEL) | 0x0004), TOP_CKMUXSEL);
		} else {
			pr_info("[%s]: config cpu speed switch mode = CLKSQ\n", __func__);
			mt_reg_sync_writel((DRV_Reg32(TOP_CKMUXSEL) & 0x0ff3), TOP_CKMUXSEL);
		}

		return count;
	} else {
		pr_info("[%s]: bad argument!! should be \"1\" or \"0\"\n", __func__);
	}

	return -EINVAL;
}

static int cpu_ss_period_read(char *buf, char **start, off_t off, int count, int *eof, void *data)
{
	int len = 0;
	char *p = buf;

	p += sprintf(p, "%d (s) %d (ns)\n", mt_cpu_ss_period_s, mt_cpu_ss_period_ns);

	*start = buf + off;

	len = p - buf;
	if (len > off)
		len -= off;
	else
		len = 0;

	return len < count ? len : count;
}

static ssize_t cpu_ss_period_write(struct file *file, const char *buffer, unsigned long count,
				   void *data)
{
	int len = 0, s = 0, ns = 0;
	char desc[32];

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len)) {
		return 0;
	}
	desc[len] = '\0';

	if (sscanf(desc, "%d %d", &s, &ns) == 2) {
		pr_info("[%s]: set cpu speed switch period = %d (s), %d (ns)\n", __func__, s, ns);
		mt_cpu_ss_period_s = s;
		mt_cpu_ss_period_ns = ns;
		return count;
	} else {
		pr_info("[%s]: bad argument!! should be \"[s]\" or \"[ns]\"\n", __func__);
	}

	return -EINVAL;
}

static int cpu_ss_period_mode_read(char *buf, char **start, off_t off, int count, int *eof,
				   void *data)
{
	int len = 0;
	char *p = buf;

	if (mt_cpu_ss_period_mode)
		p += sprintf(p, "enable");
	else
		p += sprintf(p, "disable");

	*start = buf + off;

	len = p - buf;
	if (len > off)
		len -= off;
	else
		len = 0;

	return len < count ? len : count;
}

static ssize_t cpu_ss_period_mode_write(struct file *file, const char *buffer, unsigned long count,
					void *data)
{
	int len = 0;
	char mode[20], desc[32];
	ktime_t ktime = ktime_set(mt_cpu_ss_period_s, mt_cpu_ss_period_ns);

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len)) {
		return 0;
	}
	desc[len] = '\0';

	if (sscanf(desc, "%s", mode) == 1) {
		if (!strcmp(mode, "enable")) {
			pr_info("[%s]: enable cpu speed switch period mode\n", __func__);
			mt_cpu_ss_period_mode = true;

			mt_cpu_ss_thread =
			    kthread_run(mt_cpu_ss_thread_handler, 0, "cpu speed switch");
			if (IS_ERR(mt_cpu_ss_thread)) {
				pr_info("[%s]: failed to create cpu speed switch thread\n",
				       __func__);
			}

			hrtimer_start(&mt_cpu_ss_timer, ktime, HRTIMER_MODE_REL);
			return count;
		} else if (!strcmp(mode, "disable")) {
			pr_info("[%s]: disable cpu speed switch period mode\n", __func__);
			mt_cpu_ss_period_mode = false;

			kthread_stop(mt_cpu_ss_thread);

			mt_reg_sync_writel((DRV_Reg32(TOP_CKMUXSEL) | 0x0004), TOP_CKMUXSEL);

			hrtimer_cancel(&mt_cpu_ss_timer);
			return count;
		} else {
			pr_info("[%s]: bad argument!! should be \"enable\" or \"disable\"\n",
			       __func__);
		}
	} else {
		pr_info("[%s]: bad argument!! should be \"enable\" or \"disable\"\n", __func__);
	}

	return -EINVAL;
}

static int cpu_ss_debug_mode_read(char *buf, char **start, off_t off, int count, int *eof,
				  void *data)
{
	int len = 0;
	char *p = buf;

	if (mt_cpu_ss_debug_mode)
		p += sprintf(p, "enable");
	else
		p += sprintf(p, "disable");

	*start = buf + off;

	len = p - buf;
	if (len > off)
		len -= off;
	else
		len = 0;

	return len < count ? len : count;
}

static ssize_t cpu_ss_debug_mode_write(struct file *file, const char *buffer, unsigned long count,
				       void *data)
{
	int len = 0;
	char mode[20], desc[32];

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len)) {
		return 0;
	}
	desc[len] = '\0';

	if (sscanf(desc, "%s", mode) == 1) {
		if (!strcmp(mode, "enable")) {
			pr_info("[%s]: enable cpu speed switch debug mode\n", __func__);
			mt_cpu_ss_debug_mode = true;
			return count;
		} else if (!strcmp(mode, "disable")) {
			pr_info("[%s]: disable cpu speed switch debug mode\n", __func__);
			mt_cpu_ss_debug_mode = false;
			return count;
		} else {
			pr_info("[%s]: bad argument!! should be \"enable\" or \"disable\"\n",
			       __func__);
		}
	} else {
		pr_info("[%s]: bad argument!! should be \"enable\" or \"disable\"\n", __func__);
	}

	return -EINVAL;
}
#endif

static int cpu_ss_mode_open(struct inode *inode, struct file *file)
{
	return single_open(file, cpu_ss_mode_read, NULL);
}

static const struct file_operations cpu_ss_mode_fops = {
	.owner = THIS_MODULE,
	.open = cpu_ss_mode_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.write = cpu_ss_mode_write,
	.release = single_release,
};

static int cpu_ss_period_open(struct inode *inode, struct file *file)
{
	return single_open(file, cpu_ss_period_read, NULL);
}

static const struct file_operations cpu_ss_period_fops = {
	.owner = THIS_MODULE,
	.open = cpu_ss_period_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.write = cpu_ss_period_write,
	.release = single_release,
};

static int cpu_ss_period_mode_open(struct inode *inode, struct file *file)
{
	return single_open(file, cpu_ss_period_mode_read, NULL);
}

static const struct file_operations cpu_ss_period_mode_fops = {
	.owner = THIS_MODULE,
	.open = cpu_ss_period_mode_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.write = cpu_ss_period_mode_write,
	.release = single_release,
};

static int cpu_ss_debug_mode_open(struct inode *inode, struct file *file)
{
	return single_open(file, cpu_ss_debug_mode_read, NULL);
}

static const struct file_operations cpu_ss_debug_mode_fops = {
	.owner = THIS_MODULE,
	.open = cpu_ss_debug_mode_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.write = cpu_ss_debug_mode_write,
	.release = single_release,
};

/*********************************
* cpu speed stress initialization
**********************************/
static int __init mt_cpu_ss_init(void)
{
	struct proc_dir_entry *mt_entry = NULL;
	struct proc_dir_entry *mt_cpu_ss_dir = NULL;

	hrtimer_init(&mt_cpu_ss_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	mt_cpu_ss_timer.function = mt_cpu_ss_timer_func;

	mt_cpu_ss_dir = proc_mkdir("cpu_ss", NULL);
	if (!mt_cpu_ss_dir) {
		pr_err("[%s]: mkdir /proc/cpu_ss failed\n", __func__);
	} else {
		mt_entry =
		    proc_create("cpu_ss_debug_mode", S_IRUGO | S_IWUSR | S_IWGRP, mt_cpu_ss_dir,
				&cpu_ss_debug_mode_fops);
		mt_entry =
		    proc_create("cpu_ss_period_mode", S_IRUGO | S_IWUSR | S_IWGRP, mt_cpu_ss_dir,
				&cpu_ss_period_mode_fops);
		mt_entry =
		    proc_create("cpu_ss_period", S_IRUGO | S_IWUSR | S_IWGRP, mt_cpu_ss_dir,
				&cpu_ss_period_fops);
		mt_entry =
		    proc_create("cpu_ss_mode", S_IRUGO | S_IWUSR | S_IWGRP, mt_cpu_ss_dir,
				&cpu_ss_mode_fops);

#if 0
		mt_entry =
		    create_proc_entry("cpu_ss_debug_mode", S_IRUGO | S_IWUSR | S_IWGRP,
				      mt_cpu_ss_dir);
		if (mt_entry) {
			mt_entry->read_proc = cpu_ss_debug_mode_read;
			mt_entry->write_proc = cpu_ss_debug_mode_write;
		}

		mt_entry =
		    create_proc_entry("cpu_ss_period_mode", S_IRUGO | S_IWUSR | S_IWGRP,
				      mt_cpu_ss_dir);
		if (mt_entry) {
			mt_entry->read_proc = cpu_ss_period_mode_read;
			mt_entry->write_proc = cpu_ss_period_mode_write;
		}

		mt_entry =
		    create_proc_entry("cpu_ss_period", S_IRUGO | S_IWUSR | S_IWGRP, mt_cpu_ss_dir);
		if (mt_entry) {
			mt_entry->read_proc = cpu_ss_period_read;
			mt_entry->write_proc = cpu_ss_period_write;
		}

		mt_entry =
		    create_proc_entry("cpu_ss_mode", S_IRUGO | S_IWUSR | S_IWGRP, mt_cpu_ss_dir);
		if (mt_entry) {
			mt_entry->read_proc = cpu_ss_mode_read;
			mt_entry->write_proc = cpu_ss_mode_write;
		}
#endif
	}

	return 0;
}

static void __exit mt_cpu_ss_exit(void)
{

}
module_init(mt_cpu_ss_init);
module_exit(mt_cpu_ss_exit);

MODULE_DESCRIPTION("MediaTek CPU Speed Stress driver");
MODULE_LICENSE("GPL");

