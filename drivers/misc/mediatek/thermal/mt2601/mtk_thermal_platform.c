/*
* Copyright (C) 2014 MediaTek Inc.
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
#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt


#include <asm/uaccess.h>
#include <asm/system.h>

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/dmi.h>
#include <linux/acpi.h>
#include <linux/thermal.h>
#include <linux/platform_device.h>
#include <mt-plat/aee.h>
#include <linux/printk.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/err.h>
#include <linux/syscalls.h>
#include <linux/time.h>
#include <linux/string.h>
#include <linux/mutex.h>
#include <linux/bug.h>
#include <linux/workqueue.h>

#include <mt-plat/mtk_thermal_platform.h>
#include <mt-plat/mtk_mdm_monitor.h>
#include <mach/mt_thermal.h>
#include <mt-plat/aee.h>
#include <linux/cpufreq.h>

/* ************************************ */
/* Definition */
/* ************************************ */

/* Number of CPU CORE */
#define NUMBER_OF_CORE (2)

extern unsigned int mt_gpufreq_cur_load(void);
extern unsigned int mt_gpufreq_cur_freq(void);
extern unsigned long wmt_wifi_tx_throughput(void);
extern int force_get_tbat(void);

#if defined(CONFIG_MTK_SMART_BATTERY)
/* global variable from battery driver... */
extern kal_bool gFG_Is_Charging;
#endif


/* ************************************ */
/* Global Variable */
/* ************************************ */
static bool enable_ThermalMonitorXlog;

static DEFINE_MUTEX(MTM_SYSINFO_LOCK);

/* ************************************ */
/* Macro */
/* ************************************ */
#define THRML_LOG(fmt, args...) \
	do { \
		if (unlikely(enable_ThermalMonitorXlog)) { \
			pr_debug("THERMAL/PLATFORM " fmt, ##args); \
		} \
	} while (0)


#define THRML_ERROR_LOG(fmt, args...) \
	do { \
		pr_debug("THERMAL/PLATFORM " fmt, ##args); \
	} while (0)


/* ************************************ */
/* Define */
/* ************************************ */

/* ********************************************* */
/* For get_sys_cpu_usage_info_ex() */
/* ********************************************* */

#define CPU_USAGE_CURRENT_FIELD (0)
#define CPU_USAGE_SAVE_FIELD    (1)
#define CPU_USAGE_FRAME_FIELD   (2)

struct cpu_index_st
{
	unsigned long  u[3];
	unsigned long  s[3];
	unsigned long  n[3];
	unsigned long  i[3];
	unsigned long  w[3];
	unsigned long  q[3];
	unsigned long  sq[3];
	unsigned long  tot_frme;
	unsigned long  tz;
	int  usage;
	int  freq;
};

struct gpu_index_st
{
	int  usage;
	int  freq;
};

static struct cpu_index_st cpu_index_list[4];   /* /< 4-Core is maximum */

#define NO_CPU_CORES (2)
static int cpufreqs[NO_CPU_CORES];
static int cpuloadings[NO_CPU_CORES];

#define SEEK_BUFF(x, c)  while (*x != c)x++; \
							x++;

#define TRIMz_ex(tz, x)   ((tz = (unsigned long long)(x)) < 0 ? 0 : tz)


#include <linux/kernel_stat.h>
#include <linux/cpumask.h>
#include <asm/cputime.h>
#include <linux/sched.h>
#include <linux/tick.h>
#include <linux/time.h>

#ifdef arch_idle_time

static cputime64_t get_idle_time(int cpu)
{
	cputime64_t idle;

	idle = kcpustat_cpu(cpu).cpustat[CPUTIME_IDLE];
	if (cpu_online(cpu) && !nr_iowait_cpu(cpu))
		idle += arch_idle_time(cpu);
	return idle;
}

static cputime64_t get_iowait_time(int cpu)
{
	cputime64_t iowait;

	iowait = kcpustat_cpu(cpu).cpustat[CPUTIME_IOWAIT];
	if (cpu_online(cpu) && nr_iowait_cpu(cpu))
		iowait += arch_idle_time(cpu);
	return iowait;
}

#else

static u64 get_idle_time(int cpu)
{
	u64 idle, idle_time = -1ULL;

	if (cpu_online(cpu))
		idle_time = get_cpu_idle_time_us(cpu, NULL);

	if (idle_time == -1ULL)
		/* !NO_HZ or cpu offline so we can rely on cpustat.idle */
		idle = kcpustat_cpu(cpu).cpustat[CPUTIME_IDLE];
	else
		idle = usecs_to_cputime64(idle_time);

	return idle;
}

static u64 get_iowait_time(int cpu)
{
	u64 iowait, iowait_time = -1ULL;

	if (cpu_online(cpu))
		iowait_time = get_cpu_iowait_time_us(cpu, NULL);

	if (iowait_time == -1ULL)
		/* !NO_HZ or cpu offline so we can rely on cpustat.iowait */
		iowait = kcpustat_cpu(cpu).cpustat[CPUTIME_IOWAIT];
	else
		iowait = usecs_to_cputime64(iowait_time);

	return iowait;
}

#endif

static int get_sys_cpu_usage_info_ex(void)
{
	int nCoreIndex = 0, i;

	for (i = 0; i < NO_CPU_CORES; i++)
		cpuloadings[i] = 0;

	for_each_online_cpu(nCoreIndex) {

		/* Get CPU Info */
		cpu_index_list[nCoreIndex].u[CPU_USAGE_CURRENT_FIELD] = kcpustat_cpu(nCoreIndex).cpustat[CPUTIME_USER];
		cpu_index_list[nCoreIndex].n[CPU_USAGE_CURRENT_FIELD] = kcpustat_cpu(nCoreIndex).cpustat[CPUTIME_NICE];
		cpu_index_list[nCoreIndex].s[CPU_USAGE_CURRENT_FIELD] = kcpustat_cpu(nCoreIndex).cpustat[CPUTIME_SYSTEM];
		cpu_index_list[nCoreIndex].i[CPU_USAGE_CURRENT_FIELD] = get_idle_time(nCoreIndex);
		cpu_index_list[nCoreIndex].w[CPU_USAGE_CURRENT_FIELD] = get_iowait_time(nCoreIndex);
		cpu_index_list[nCoreIndex].q[CPU_USAGE_CURRENT_FIELD] = kcpustat_cpu(nCoreIndex).cpustat[CPUTIME_IRQ];
		cpu_index_list[nCoreIndex].sq[CPU_USAGE_CURRENT_FIELD] = kcpustat_cpu(nCoreIndex).cpustat[CPUTIME_SOFTIRQ];

		/* Frame */
		cpu_index_list[nCoreIndex].u[CPU_USAGE_FRAME_FIELD] = cpu_index_list[nCoreIndex].u[CPU_USAGE_CURRENT_FIELD] -
																cpu_index_list[nCoreIndex].u[CPU_USAGE_SAVE_FIELD];
		cpu_index_list[nCoreIndex].n[CPU_USAGE_FRAME_FIELD] = cpu_index_list[nCoreIndex].n[CPU_USAGE_CURRENT_FIELD] -
																cpu_index_list[nCoreIndex].n[CPU_USAGE_SAVE_FIELD];
		cpu_index_list[nCoreIndex].s[CPU_USAGE_FRAME_FIELD] = cpu_index_list[nCoreIndex].s[CPU_USAGE_CURRENT_FIELD] -
																cpu_index_list[nCoreIndex].s[CPU_USAGE_SAVE_FIELD];
		cpu_index_list[nCoreIndex].i[CPU_USAGE_FRAME_FIELD] = TRIMz_ex(cpu_index_list[nCoreIndex].tz,
																(cpu_index_list[nCoreIndex].i[CPU_USAGE_CURRENT_FIELD] -
																cpu_index_list[nCoreIndex].i[CPU_USAGE_SAVE_FIELD]));
		cpu_index_list[nCoreIndex].w[CPU_USAGE_FRAME_FIELD] = cpu_index_list[nCoreIndex].w[CPU_USAGE_CURRENT_FIELD] -
																cpu_index_list[nCoreIndex].w[CPU_USAGE_SAVE_FIELD];
		cpu_index_list[nCoreIndex].q[CPU_USAGE_FRAME_FIELD] = cpu_index_list[nCoreIndex].q[CPU_USAGE_CURRENT_FIELD] -
																cpu_index_list[nCoreIndex].q[CPU_USAGE_SAVE_FIELD];
		cpu_index_list[nCoreIndex].sq[CPU_USAGE_FRAME_FIELD] = cpu_index_list[nCoreIndex].sq[CPU_USAGE_CURRENT_FIELD] -
																cpu_index_list[nCoreIndex].sq[CPU_USAGE_SAVE_FIELD];

		/* Total Frame */
		cpu_index_list[nCoreIndex].tot_frme = cpu_index_list[nCoreIndex].u[CPU_USAGE_FRAME_FIELD] +
											  cpu_index_list[nCoreIndex].n[CPU_USAGE_FRAME_FIELD] +
											  cpu_index_list[nCoreIndex].s[CPU_USAGE_FRAME_FIELD] +
											  cpu_index_list[nCoreIndex].i[CPU_USAGE_FRAME_FIELD] +
											  cpu_index_list[nCoreIndex].w[CPU_USAGE_FRAME_FIELD] +
											  cpu_index_list[nCoreIndex].q[CPU_USAGE_FRAME_FIELD] +
											  cpu_index_list[nCoreIndex].sq[CPU_USAGE_FRAME_FIELD];

		/* CPU Usage */
		if (cpu_index_list[nCoreIndex].tot_frme > 0) {
			cpuloadings[nCoreIndex] = (100-(((int)cpu_index_list[nCoreIndex].i[CPU_USAGE_FRAME_FIELD]*100)/(int)cpu_index_list[nCoreIndex].tot_frme));
		} else {
			/* CPU unplug case */
			cpuloadings[nCoreIndex] = 0;
		}

		cpu_index_list[nCoreIndex].u[CPU_USAGE_SAVE_FIELD]  = cpu_index_list[nCoreIndex].u[CPU_USAGE_CURRENT_FIELD];
		cpu_index_list[nCoreIndex].n[CPU_USAGE_SAVE_FIELD]  = cpu_index_list[nCoreIndex].n[CPU_USAGE_CURRENT_FIELD];
		cpu_index_list[nCoreIndex].s[CPU_USAGE_SAVE_FIELD]  = cpu_index_list[nCoreIndex].s[CPU_USAGE_CURRENT_FIELD];
		cpu_index_list[nCoreIndex].i[CPU_USAGE_SAVE_FIELD]  = cpu_index_list[nCoreIndex].i[CPU_USAGE_CURRENT_FIELD];
		cpu_index_list[nCoreIndex].w[CPU_USAGE_SAVE_FIELD]  = cpu_index_list[nCoreIndex].w[CPU_USAGE_CURRENT_FIELD];
		cpu_index_list[nCoreIndex].q[CPU_USAGE_SAVE_FIELD]  = cpu_index_list[nCoreIndex].q[CPU_USAGE_CURRENT_FIELD];
		cpu_index_list[nCoreIndex].sq[CPU_USAGE_SAVE_FIELD] = cpu_index_list[nCoreIndex].sq[CPU_USAGE_CURRENT_FIELD];

		THRML_LOG("CPU%d Frame:%lu USAGE:%d\n", nCoreIndex,
			  cpu_index_list[nCoreIndex].tot_frme, cpuloadings[nCoreIndex]);

		for (i = 0; i < 3; i++) {
			THRML_LOG
				("Index %d [u:%lu] [n:%lu] [s:%lu] [i:%lu] [w:%lu] [q:%lu] [sq:%lu]\n",
				 i, cpu_index_list[nCoreIndex].u[i], cpu_index_list[nCoreIndex].n[i],
				 cpu_index_list[nCoreIndex].s[i], cpu_index_list[nCoreIndex].i[i],
				 cpu_index_list[nCoreIndex].w[i], cpu_index_list[nCoreIndex].q[i],
					  cpu_index_list[nCoreIndex].sq[i]);

		}
	}

	return 0;

}


static int get_sys_all_cpu_freq_info(void)
{
	int i;

	for (i=0 ; i<NO_CPU_CORES ; i++)
		cpufreqs[i] = cpufreq_quick_get(i)/1000; // MHz

	return 0;
}



/* Init */
static int __init mtk_thermal_platform_init(void)
{
	int err = 0;

	return err;
}

/* Exit */
static void __exit mtk_thermal_platform_exit(void)
{

}

int mtk_thermal_get_cpu_info(
	int *nocores,
	int **cpufreq,
	int **cpuloading)
{
	/* ****************** */
	/* CPU Usage */
	/* ****************** */
	mutex_lock(&MTM_SYSINFO_LOCK);

	/* Read CPU Usage Information */
	get_sys_cpu_usage_info_ex();

	get_sys_all_cpu_freq_info();

	mutex_unlock(&MTM_SYSINFO_LOCK);

	if (nocores)
		*nocores = NO_CPU_CORES;

	if (cpufreq)
		*cpufreq = cpufreqs;

	if (cpuloading)
		*cpuloading = cpuloadings;

	return 0;
}

#define NO_GPU_CORES (1)
static int gpufreqs[NO_GPU_CORES];
static int gpuloadings[NO_GPU_CORES];

int mtk_thermal_get_gpu_info(
	int *nocores,
	int **gpufreq,
	int **gpuloading)
{
	/* ****************** */
	/* GPU Index */
	/* ****************** */
	if (nocores)
		*nocores = NO_GPU_CORES;

	if (gpufreq)
	{
		gpufreqs[0] = (int) mt_gpufreq_cur_freq()/1000; /* the return value is KHz */
		*gpufreq = gpufreqs;
	}

	if (gpuloading)
	{
		gpuloadings[0] = (int) mt_gpufreq_cur_load();
		*gpuloading = gpuloadings;
	}

	return 0;
}
extern int read_tbat_value(void);
extern int battery_meter_get_battery_current(void);

int mtk_thermal_get_batt_info(
	int *batt_voltage,
	int *batt_current,
	int *batt_temp)
{
	/* ****************** */
	/* Battery */
	/* ****************** */
		/* Read Battery Information */
	if (batt_current)
	{
		/* from 72, all platforms use mt-battery */
		*batt_current = battery_meter_get_battery_current();
		/* the return value is 0.1mA */
		if (*batt_current%10 < 5)
			*batt_current /= 10;
		else
			*batt_current = 1+(*batt_current/10);


		#if defined(CONFIG_MTK_SMART_BATTERY)
		if (KAL_TRUE == gFG_Is_Charging)
		{
			*batt_current *= -1;
		}
		#endif
	}

	if (batt_voltage)
		*batt_voltage = 0;

	if (batt_temp)
		*batt_temp = 0;

	return 0;
}

#define NO_EXTRA_THERMAL_ATTR (7)
static char *extra_attr_names[NO_EXTRA_THERMAL_ATTR] = {0};
static int extra_attr_values[NO_EXTRA_THERMAL_ATTR] = {0};
static char *extra_attr_units[NO_EXTRA_THERMAL_ATTR] = {0};

int mtk_thermal_get_extra_info(
	int *no_extra_attr,
	char ***attr_names,
	int **attr_values,
	char ***attr_units)
{
	int i = 0;

	if (no_extra_attr)
		*no_extra_attr = NO_EXTRA_THERMAL_ATTR;

	/* ****************** */
	/* Wifi Index */
	/* ****************** */
	/* Get Wi-Fi Tx throughput */
	extra_attr_names[i] = "WiFi_TP";
#ifdef CONFIG_MTK_COMBO_CHIP
	extra_attr_values[i] = wmt_wifi_tx_throughput();
#else
	extra_attr_values[i] = 0;
#endif
	extra_attr_units[i] = "Kbps";

	if (attr_names)
		*attr_names = extra_attr_names;

	if (attr_values)
		*attr_values = extra_attr_values;

	if (attr_units)
		*attr_units = extra_attr_units;

	return 0;
}


int mtk_thermal_force_get_batt_temp(
	void)
{
	int ret = 0;

	ret = force_get_tbat();
	return ret;
}

static unsigned int _thermal_scen;

unsigned int mtk_thermal_set_user_scenarios(
	unsigned int mask)
{
	if ((mask & MTK_THERMAL_SCEN_CALL)) /* only one scen is handled now... */
	{
		set_taklking_flag(true); /* make mtk_ts_cpu.c aware of call scenario */
		_thermal_scen |= (unsigned int) MTK_THERMAL_SCEN_CALL;
	}
	return _thermal_scen;
}

unsigned int mtk_thermal_clear_user_scenarios(
	unsigned int mask)
{
	if ((mask & MTK_THERMAL_SCEN_CALL)) /* only one scen is handled now... */
	{
		set_taklking_flag(false); /* make mtk_ts_cpu.c aware of call scenario */
		_thermal_scen &= ~((unsigned int) MTK_THERMAL_SCEN_CALL);
	}
	return _thermal_scen;
}

/* ********************************************* */
/* Export Interface */
/* ********************************************* */

EXPORT_SYMBOL(mtk_thermal_get_cpu_info);
EXPORT_SYMBOL(mtk_thermal_get_gpu_info);
EXPORT_SYMBOL(mtk_thermal_get_batt_info);
EXPORT_SYMBOL(mtk_thermal_get_extra_info);
EXPORT_SYMBOL(mtk_thermal_force_get_batt_temp);
EXPORT_SYMBOL(mtk_thermal_set_user_scenarios);
EXPORT_SYMBOL(mtk_thermal_clear_user_scenarios);
module_init(mtk_thermal_platform_init);
module_exit(mtk_thermal_platform_exit);
