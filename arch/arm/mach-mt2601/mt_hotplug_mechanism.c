/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

/*********************************
* include
**********************************/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/cpu.h>
#include <linux/proc_fs.h>
#include <linux/notifier.h>
#include <linux/fb.h>
#include <asm/uaccess.h>
#include <mach/hotplug.h>
#include <mt-plat/sync_write.h>
#include <linux/seq_file.h>


/*********************************
* macro
**********************************/

/*********************************
* glabal variable
**********************************/
static int g_enable;
/* FB notifier */
static int mt_hotplug_mechanism_fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data);
static struct notifier_block mt_hotplug_mechanism_fb_notifier = {
	.notifier_call = mt_hotplug_mechanism_fb_notifier_callback,
};
static int g_test0;
static int g_test1;

/*********************************
* extern function
**********************************/
#ifdef CONFIG_CPU_FREQ_GOV_HOTPLUG
extern void hp_set_dynamic_cpu_hotplug_enable(int enable);
#endif

#ifdef CONFIG_CPU_FREQ_GOV_BALANCE
extern struct mutex bl_onoff_mutex;
#endif

/*********************************
* early suspend callback function
**********************************/
static void mt_hotplug_mechanism_early_suspend(void)
{
	HOTPLUG_INFO("[%s] g_enable=%d\n", __func__, g_enable);

	if (g_enable) {
		int i = 0;

    #ifdef CONFIG_CPU_FREQ_GOV_BALANCE
		mutex_lock(&bl_onoff_mutex);
    #endif
    #ifdef CONFIG_CPU_FREQ_GOV_HOTPLUG
		hp_set_dynamic_cpu_hotplug_enable(0);
    #endif //#ifdef CONFIG_CPU_FREQ_GOV_HOTPLUG

		for (i = (num_possible_cpus() - 1); i > 0; i--) {
			if (cpu_online(i))
				cpu_down(i);
		}

    #ifdef CONFIG_CPU_FREQ_GOV_BALANCE
		mutex_unlock(&bl_onoff_mutex);
    #endif
	}
}

/*******************************
* late resume callback function
********************************/
static void mt_hotplug_mechanism_late_resume(void)
{
	HOTPLUG_INFO("[%s] g_enable=%d\n", __func__, g_enable);

	if (g_enable) {
    #ifdef CONFIG_CPU_FREQ_GOV_HOTPLUG
		hp_set_dynamic_cpu_hotplug_enable(1);
    #endif //#ifdef CONFIG_CPU_FREQ_GOV_HOTPLUG
	}

	return;
}

/*********************************
* FB notifier function
**********************************/
static int mt_hotplug_mechanism_fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int blank;

	/* skip if it's not a blank event */
	if (event != FB_EVENT_BLANK)
		return 0;

	blank = *(int *)evdata->data;

	HOTPLUG_INFO("blank=%d, event=%lu\n", blank, event);

	switch (blank) {
	/* LCM ON */
	case FB_BLANK_UNBLANK:
		mt_hotplug_mechanism_late_resume();
		break;
	/* LCM OFF */
	case FB_BLANK_POWERDOWN:
		mt_hotplug_mechanism_early_suspend();
		break;
	default:
		break;
	}

	return 0;
}



#if 1
static int mt_hotplug_mechanism_read_test0(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", g_test0);

	HOTPLUG_INFO("mt_hotplug_mechanism_read_test0, hotplug_cpu_count: %d\n",
		     atomic_read(&hotplug_cpu_count));
	on_each_cpu((smp_call_func_t) dump_stack, NULL, 1);

	mt_reg_sync_writel(8, 0xf0200080);
	printk(KERN_EMERG "CPU%u, debug event: 0x%08x, debug monitor: 0x%08x\n", 0,
	       *(volatile u32 *)(0xf0200080), *(volatile u32 *)(0xf0200084));
	mt_reg_sync_writel(9, 0xf0200080);
	printk(KERN_EMERG "CPU%u, debug event: 0x%08x, debug monitor: 0x%08x\n", 1,
	       *(volatile u32 *)(0xf0200080), *(volatile u32 *)(0xf0200084));

	return 0;
}

static int mt_hotplug_mechanism_write_test0(struct file *file, const char __user *buffer,
					    size_t count, loff_t *pos)
{
	int len = 0, test0 = 0;
	char desc[32];

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len)) {
		return 0;
	}
	desc[len] = '\0';

	if (sscanf(desc, "%d", &test0) == 1) {
		g_test0 = test0;
		return count;
	} else {
		HOTPLUG_INFO("mt_hotplug_mechanism_write_test0, bad argument\n");
	}

	return -EINVAL;
}

static int mt_hotplug_mechanism_open_test0(struct inode *inode, struct file *file)
{
	return single_open(file, mt_hotplug_mechanism_read_test0, NULL);
}

static const struct file_operations mt_hotplug_test0_fops = {
	.owner = THIS_MODULE,
	.open = mt_hotplug_mechanism_open_test0,
	.read = seq_read,
	.write = mt_hotplug_mechanism_write_test0,
};

#else
/**************************************************************
* mt hotplug mechanism control interface for procfs test0
***************************************************************/
static int mt_hotplug_mechanism_read_test0(char *buf, char **start, off_t off, int count, int *eof,
					   void *data)
{
	char *p = buf;

	p += sprintf(p, "%d\n", g_test0);
	*eof = 1;

	HOTPLUG_INFO("mt_hotplug_mechanism_read_test0, hotplug_cpu_count: %d\n",
		     atomic_read(&hotplug_cpu_count));
	on_each_cpu((smp_call_func_t) dump_stack, NULL, 1);

	mt_reg_sync_writel(8, 0xf0200080);
	printk(KERN_EMERG "CPU%u, debug event: 0x%08x, debug monitor: 0x%08x\n", 0,
	       *(volatile u32 *)(0xf0200080), *(volatile u32 *)(0xf0200084));
	mt_reg_sync_writel(9, 0xf0200080);
	printk(KERN_EMERG "CPU%u, debug event: 0x%08x, debug monitor: 0x%08x\n", 1,
	       *(volatile u32 *)(0xf0200080), *(volatile u32 *)(0xf0200084));

	return p - buf;
}

static int mt_hotplug_mechanism_write_test0(struct file *file, const char *buffer,
					    unsigned long count, void *data)
{
	int len = 0, test0 = 0;
	char desc[32];

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len)) {
		return 0;
	}
	desc[len] = '\0';

	if (sscanf(desc, "%d", &test0) == 1) {
		g_test0 = test0;
		return count;
	} else {
		HOTPLUG_INFO("mt_hotplug_mechanism_write_test0, bad argument\n");
	}

	return -EINVAL;
}
#endif

#if 1
static int mt_hotplug_mechanism_read_test1(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", g_test1);

	return 0;
}

static int mt_hotplug_mechanism_write_test1(struct file *file, const char __user *buffer,
					    size_t count, loff_t *pos)
{
	int len = 0, test1 = 0;
	char desc[32];

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len)) {
		return 0;
	}
	desc[len] = '\0';

	if (sscanf(desc, "%d", &test1) == 1) {
		g_test1 = test1;
		return count;
	} else {
		HOTPLUG_INFO("mt_hotplug_mechanism_write_test1, bad argument\n");
	}

	return -EINVAL;
}

static int mt_hotplug_mechanism_open_test1(struct inode *inode, struct file *file)
{
	return single_open(file, mt_hotplug_mechanism_read_test1, NULL);
}

static const struct file_operations mt_hotplug_test1_fops = {
	.owner = THIS_MODULE,
	.open = mt_hotplug_mechanism_open_test1,
	.read = seq_read,
	.write = mt_hotplug_mechanism_write_test1,
};

#else
/**************************************************************
* mt hotplug mechanism control interface for procfs test1
***************************************************************/
static int mt_hotplug_mechanism_read_test1(char *buf, char **start, off_t off, int count, int *eof,
					   void *data)
{
	char *p = buf;

	p += sprintf(p, "%d\n", g_test1);
	*eof = 1;

	return p - buf;
}

static int mt_hotplug_mechanism_write_test1(struct file *file, const char *buffer,
					    unsigned long count, void *data)
{
	int len = 0, test1 = 0;
	char desc[32];

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len)) {
		return 0;
	}
	desc[len] = '\0';

	if (sscanf(desc, "%d", &test1) == 1) {
		g_test1 = test1;
		return count;
	} else {
		HOTPLUG_INFO("mt_hotplug_mechanism_write_test1, bad argument\n");
	}

	return -EINVAL;
}
#endif


/*******************************
* kernel module init function
********************************/
static int __init mt_hotplug_mechanism_init(void)
{
	struct proc_dir_entry *entry = NULL;
	struct proc_dir_entry *mt_hotplug_test_dir = NULL;

	HOTPLUG_INFO("mt_hotplug_mechanism_init");

	mt_hotplug_test_dir = proc_mkdir("mt_hotplug_test", NULL);
	if (!mt_hotplug_test_dir) {
		HOTPLUG_INFO("mkdir /proc/mt_hotplug_test failed");
	} else {
		entry =
		    proc_create("test0", S_IRUGO | S_IWUSR, mt_hotplug_test_dir,
				&mt_hotplug_test0_fops);
		entry =
		    proc_create("test1", S_IRUGO | S_IWUSR, mt_hotplug_test_dir,
				&mt_hotplug_test1_fops);

#if 0
		entry = create_proc_entry("test0", S_IRUGO | S_IWUSR, mt_hotplug_test_dir);
		if (entry) {
			entry->read_proc = mt_hotplug_mechanism_read_test0;
			entry->write_proc = mt_hotplug_mechanism_write_test0;
		}
		entry = create_proc_entry("test1", S_IRUGO | S_IWUSR, mt_hotplug_test_dir);
		if (entry) {
			entry->read_proc = mt_hotplug_mechanism_read_test1;
			entry->write_proc = mt_hotplug_mechanism_write_test1;
		}
#endif
	}

	if (fb_register_client(&mt_hotplug_mechanism_fb_notifier)) {
		HOTPLUG_INFO("register FB client failed!\n");
	}

	return 0;
}
module_init(mt_hotplug_mechanism_init);



/*******************************
* kernel module exit function
********************************/
static void __exit mt_hotplug_mechanism_exit(void)
{
	HOTPLUG_INFO("mt_hotplug_mechanism_exit");
}
module_exit(mt_hotplug_mechanism_exit);



/**************************************************************
* mt hotplug mechanism control interface for thermal protect
***************************************************************/
void mt_hotplug_mechanism_thermal_protect(int limited_cpus)
{
	HOTPLUG_INFO("mt_hotplug_mechanism_thermal_protect\n");

}
EXPORT_SYMBOL(mt_hotplug_mechanism_thermal_protect);

MODULE_DESCRIPTION("MediaTek CPU Hotplug Mechanism");
MODULE_LICENSE("GPL");
