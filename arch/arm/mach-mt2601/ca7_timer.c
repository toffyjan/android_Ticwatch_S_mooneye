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

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/smp.h>
#include <linux/jiffies.h>
#include <linux/clockchips.h>
#include <linux/interrupt.h>
#include <linux/cpu.h>
#include <mach/mt_irq.h>
#include <mach/mt_gpt.h>

#define read_cntfrq(cntfrq) \
do {    \
    __asm__ __volatile__(   \
    "MRC p15, 0, %0, c14, c0, 0\n"  \
 : "=r"(cntfrq)   \
    :   \
 : "memory"); \
} while (0)

#define change_cntfrq(cntfrq) \
do {    \
    __asm__ __volatile__(   \
    "MCR p15, 0, %0, c14, c0, 0\n"  \
    :   \
 : "r"(cntfrq));  \
} while (0)

#define read_cntkctl(cntkctl)   \
do {    \
    __asm__ __volatile__(   \
    "MRC p15, 0, %0, c14, c1, 0\n"  \
 : "=r"(cntkctl)  \
    :   \
 : "memory"); \
} while (0)

#define read_cntpct(cntpct_lo, cntpct_hi)   \
do {    \
    __asm__ __volatile__(   \
    "MRRC p15, 0, %0, %1, c14\n"    \
 : "=r"(cntpct_lo), "=r"(cntpct_hi)   \
    :   \
 : "memory"); \
} while (0)

#define read_cntvct(cntvct_lo, cntvct_hi)   \
do {    \
    __asm__ __volatile__(   \
    "MRRC p15, 1, %0, %1, c14\n"    \
 : "=r"(cntvct_lo), "=r"(cntvct_hi)   \
    :   \
 : "memory"); \
} while (0)

#define read_cntp_ctl(cntp_ctl)   \
do {    \
    __asm__ __volatile__(   \
    "MRC p15, 0, %0, c14, c2, 1\n"  \
 : "=r"(cntp_ctl) \
    :   \
 : "memory"); \
} while (0)

#define write_cntp_ctl(cntp_ctl)  \
do {    \
    __asm__ __volatile__(   \
    "MCR p15, 0, %0, c14, c2, 1\n"  \
    :   \
 : "r"(cntp_ctl)); \
} while (0)


#define read_cntp_cval(cntp_cval_lo, cntp_cval_hi) \
do {    \
    __asm__ __volatile__(   \
    "MRRC p15, 2, %0, %1, c14\n"    \
 : "=r"(cntp_cval_lo), "=r"(cntp_cval_hi) \
    :   \
 : "memory"); \
} while (0)

#define write_cntp_cval(cntp_cval_lo, cntp_cval_hi) \
do {    \
    __asm__ __volatile__(   \
    "MCRR p15, 2, %0, %1, c14\n"    \
    :   \
 : "r"(cntp_cval_lo), "r"(cntp_cval_hi));    \
} while (0)

#define read_cntp_tval(cntp_tval) \
do {    \
    __asm__ __volatile__(   \
    "MRC p15, 0, %0, c14, c2, 0"    \
 : "=r"(cntp_tval)    \
    :   \
 : "memory"); \
} while (0)

#define write_cntp_tval(cntp_tval) \
do {    \
    __asm__ __volatile__(   \
    "MCR p15, 0, %0, c14, c2, 0\n"    \
    :   \
 : "r"(cntp_tval));    \
} while (0)


#define read_cntv_ctl(cntv_ctl)   \
do {    \
    __asm__ __volatile__(   \
    "MRC p15, 0, %0, c14, c3, 1\n"  \
 : "=r"(cntv_ctl) \
    :   \
 : "memory"); \
} while (0)

#define read_cntv_cval(cntv_cval_lo, cntv_cval_hi) \
do {    \
    __asm__ __volatile__(   \
    "MRRC p15, 3, %0, %1, c14\n"    \
 : "=r"(cntv_cval_lo), "=r"(cntv_cval_hi) \
    :   \
 : "memory"); \
} while (0)

#define read_cntv_tval(cntv_tval) \
do {    \
    __asm__ __volatile__(   \
    "MRC p15, 0, %0, c14, c3, 0"    \
 : "=r"(cntv_tval)    \
    :   \
 : "memory"); \
} while (0)


#define CNTP_CTL_ENABLE     (1 << 0)
#define CNTP_CTL_IMASK      (1 << 1)
#define CNTP_CTL_ISTATUS    (1 << 2)

#define MT_LOCAL_TIMER_DEBUG
static void save_localtimer_info(unsigned long evt, int ext);

unsigned long generic_timer_rate;

static struct clock_event_device __percpu *timer_evt;
static int timer_ppi;

static int generic_timer_shutdown(struct clock_event_device *clk)
{
	unsigned int cntp_ctl;
	
	read_cntp_ctl(cntp_ctl);
	cntp_ctl &= ~CNTP_CTL_ENABLE;
	write_cntp_ctl(cntp_ctl);
	return 0;
}

static int generic_timer_set_next_event(unsigned long evt, struct clock_event_device *unused)
{
	write_cntp_tval(evt);
	write_cntp_ctl(CNTP_CTL_ENABLE);

	save_localtimer_info(evt, 0);

	return 0;
}

int localtimer_set_next_event(unsigned long evt)
{
	generic_timer_set_next_event(evt, NULL);

	save_localtimer_info(evt, 1);

	return 0;
}

unsigned long localtimer_get_counter(void)
{
	unsigned long evt;
	read_cntp_tval(evt);

	return evt;
}


/*
 * generic_timer_ack: checks for a local timer interrupt.
 *
 * If a local timer interrupt has occurred, acknowledge and return 1.
 * Otherwise, return 0.
 */
static int generic_timer_ack(void)
{
	unsigned int cntp_ctl;
	read_cntp_ctl(cntp_ctl);

	if (cntp_ctl & CNTP_CTL_ISTATUS) {
		write_cntp_ctl(CNTP_CTL_IMASK);
		return 1;
	}

	pr_warning("Generic Timer CNTP_CTL = 0x%x\n", cntp_ctl);
	return 0;
}

static void generic_timer_stop(struct clock_event_device *clk)
{
	disable_percpu_irq(clk->irq);
	clk->set_state_shutdown(clk);
}

static void generic_timer_calibrate_rate(void)
{
	unsigned long count;
	u64 waitjiffies;

	/*
	 * If this is the first time round, we need to work out how fast
	 * the timer ticks
	 */
	if (generic_timer_rate == 0) {
		printk("Calibrating local timer... ");

		/* Wait for a tick to start */
		waitjiffies = get_jiffies_64() + 1;

		while (get_jiffies_64() < waitjiffies)
			udelay(10);

		/* OK, now the tick has started, let's get the timer going */
		waitjiffies += 5;

		/* enable, no interrupt or reload */
		write_cntp_ctl(CNTP_CTL_ENABLE | CNTP_CTL_IMASK);

		/* maximum value */
		write_cntp_tval(0xFFFFFFFFU);

		while (get_jiffies_64() < waitjiffies)
			udelay(10);

		read_cntp_tval(count);
		generic_timer_rate = (0xFFFFFFFFU - count) * (HZ / 5);

		printk("%lu.%02luMHz.\n", generic_timer_rate / 1000000,
		       (generic_timer_rate / 10000) % 100);
	}
}

static irqreturn_t timer_handler(int irq, void *dev_id)
{
	struct clock_event_device *evt = (struct clock_event_device *)dev_id;
/* #ifdef CONFIG_MT_SCHED_MONITOR */
#if 0
	/* add timer event tracer for wdt debug */
	__raw_get_cpu_var(local_timer_ts) = sched_clock();
	if (generic_timer_ack()) {
		evt->event_handler(evt);
		__raw_get_cpu_var(local_timer_te) = sched_clock();
		return IRQ_HANDLED;
	}
	__raw_get_cpu_var(local_timer_te) = sched_clock();
	return IRQ_NONE;
#else
	if (generic_timer_ack()) {
		evt->event_handler(evt);
		return IRQ_HANDLED;
	}
	return IRQ_NONE;
#endif
}


/*
 * Setup the local clock events for a CPU.
 */
static int generic_timer_setup(struct clock_event_device *clk)
{
	pr_info("[ca7_timer]%s entry\n", __func__);
	generic_timer_calibrate_rate();

	write_cntp_ctl(0x0);

	clk->name = "generic_timer";
	clk->features = CLOCK_EVT_FEAT_ONESHOT;
	clk->rating = 350;
	clk->cpumask = cpumask_of(smp_processor_id());
	clk->set_state_shutdown = generic_timer_shutdown;
	clk->set_next_event = generic_timer_set_next_event;
	clk->irq = timer_ppi;

	clk->set_state_shutdown(clk);
	clockevents_config_and_register(clk, generic_timer_rate, 0xf, 0x7fffffff);

	enable_percpu_irq(clk->irq, 0);

	return 0;
}

/*
static struct local_timer_ops generic_timer_ops __cpuinitdata = {
	.setup = generic_timer_setup,
	.stop = generic_timer_stop,
};*/

static int arch_timer_cpu_notify(struct notifier_block *self,
					   unsigned long action, void *hcpu)
{
	/*
	 * Grab cpu pointer in each case to avoid spurious
	 * preemptible warnings
	 */
	switch (action & ~CPU_TASKS_FROZEN) {
	case CPU_STARTING:
		generic_timer_setup(this_cpu_ptr(timer_evt));
		break;
	case CPU_DYING:
		generic_timer_stop(this_cpu_ptr(timer_evt));
		break;
	}

	return NOTIFY_OK;
}

static struct notifier_block arch_timer_cpu_nb = {
	.notifier_call = arch_timer_cpu_notify,
};

int __init generic_timer_register(void)
{
	int err;

	if (timer_evt)
		return -EBUSY;

	timer_ppi = GIC_PPI_PRIVATE_TIMER;

	timer_evt = alloc_percpu(struct clock_event_device);

	if (!timer_evt) {
		err = -ENOMEM;
		goto out_exit;
	}

	err = request_percpu_irq(timer_ppi, timer_handler, "timer", timer_evt);
	if (err) {
		pr_err("generic timer: can't register interrupt %d (%d)\n", timer_ppi, err);
		goto out_free;
	}

	err = register_cpu_notifier(&arch_timer_cpu_nb);
	if (err)
		goto out_irq;

	/* Immediately configure the timer on the boot CPU */
	generic_timer_setup(this_cpu_ptr(timer_evt));

	return 0;

out_irq:
	free_percpu_irq(timer_ppi, timer_evt);
out_free:
	free_percpu(timer_evt);
	timer_evt = NULL;
out_exit:
	return err;
}

#ifdef MT_LOCAL_TIMER_DEBUG
#include <linux/sched.h>

struct localtimer_info {
	unsigned long evt;
	unsigned int ctrl;
	int ext;
	unsigned long long timestamp;
};

static struct localtimer_info save_data[NR_CPUS];

static void save_localtimer_info(unsigned long evt, int ext)
{
	int cpu;
	unsigned int ctrl;

	cpu = smp_processor_id();
	read_cntp_ctl(ctrl);

	save_data[cpu].evt = evt;
	save_data[cpu].ctrl = ctrl;
	save_data[cpu].ext = ext;
	save_data[cpu].timestamp = sched_clock();
}

int dump_localtimer_register(char *buffer, int size)
{
	int i;
	int len = 0;
#define LOCAL_LEN   256
	char fmt[LOCAL_LEN];

	unsigned int cntp_ctl;
	unsigned int cntp_tval;
	unsigned int cntp_cval_lo, cntp_cval_hi;
	unsigned int cntpct_lo, cntpct_hi;

	if (!buffer || size <= 1) {
		return 0;
	}

	len += snprintf(fmt + len, LOCAL_LEN - len, "[localtimer]cpu evt ctl ext time\n");

	for (i = 0; i < nr_cpu_ids; i++) {
		len += snprintf(fmt + len, LOCAL_LEN - len, "%d %lx %x %d %llx\n",
				i, save_data[i].evt, save_data[i].ctrl,
				save_data[i].ext, save_data[i].timestamp);
	}

	read_cntp_ctl(cntp_ctl);
	read_cntp_cval(cntp_cval_lo, cntp_cval_hi);
	read_cntp_tval(cntp_tval);
	read_cntpct(cntpct_lo, cntpct_hi);

	len += snprintf(fmt + len, LOCAL_LEN - len, "cpu ctl tval cval pct\n");
	len += snprintf(fmt + len, LOCAL_LEN - len,
			"%d %x %x (%x,%x) (%x,%x)\n",
			smp_processor_id(), cntp_ctl, cntp_tval,
			cntp_cval_lo, cntp_cval_hi, cntpct_lo, cntpct_hi);

	len = min(len, size - 1);
	memcpy(buffer, fmt, len);
	*(buffer + len) = '\0';

	return len;
}
#else

static inline void save_localtimer_info(unsigned long evt, int ext)
{
	return;
}

int dump_localtimer_register(char *buffer, int size)
{
	return 0;
}

#endif


#if 0

#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/proc_fs.h>
#include <linux/kthread.h>
#include <linux/err.h>

#include <asm/uaccess.h>
#include <mach/mt_gpt.h>

static int cpuid[NR_CPUS] = { 0 };

static struct completion notify[NR_CPUS];
static struct completion ack;
static unsigned int opcode;
static unsigned int op1;
static unsigned int op2;

static DEFINE_MUTEX(opcode_lock);
static DEFINE_SPINLOCK(cpu_lock);


void dump_timer_regs(void)
{
	unsigned int cntpct_lo = 0xFFFFFFFF;
	unsigned int cntpct_hi = 0xFFFFFFFF;
	unsigned int cntp_ctl = 0xFFFFFFFF;
	unsigned int cntp_cval_lo = 0xFFFFFFFF;
	unsigned int cntp_cval_hi = 0xFFFFFFFF;
	unsigned int cntp_tval = 0xFFFFFFFF;

	read_cntpct(cntpct_lo, cntpct_hi);
	read_cntp_ctl(cntp_ctl);
	read_cntp_cval(cntp_cval_lo, cntp_cval_hi);
	read_cntp_tval(cntp_tval);

	printk("[ca7_timer]2. cntpct_lo = 0x%08x, cntpct_hi = 0x%08x\n", cntpct_lo, cntpct_hi);
	printk("[ca7_timer]4. cntp_ctl = 0x%x\n", cntp_ctl);
	printk("[ca7_timer]5. cntp_cval_lo = 0x%08x, cntp_cval_hi = 0x%08x\n", cntp_cval_lo, cntp_cval_hi);
	printk("[ca7_timer]6. cntp_tval = 0x%08x\n", cntp_tval);
}


static int test_ack(void)
{
	unsigned int cntp_ctl;
	read_cntp_ctl(cntp_ctl);

	printk("test_ack: CNTP_CTL = 0x%x\n", cntp_ctl);

	if (cntp_ctl & CNTP_CTL_ISTATUS) {
		write_cntp_ctl(CNTP_CTL_IMASK);
		return 1;
	}

	return 0;
}

static irqreturn_t test_handler(int irq, void *dev_id)
{
	if (test_ack()) {
		/* unsigned int pos = gpt_get_cnt(GPT2); */
		/* printk("[generic_timer:%s]entry, pos=%lu\n", __func__, pos); */
		return IRQ_HANDLED;
	}
	return IRQ_NONE;
}


static void test_operation(void)
{
	if (opcode == 0) {
		dump_timer_regs();
	} else if (opcode == 1) {
		write_cntp_tval(op1);
		write_cntp_ctl(op2);
	}
}


static int local_timer_test(void *data)
{
	int cpu = *(int *)data;

	printk("[%s]: thread for cpu%d start\n", __func__, cpu);
	enable_percpu_irq(GIC_PPI_PRIVATE_TIMER, 0);

	while (1) {
		wait_for_completion(&notify[cpu]);
		test_operation();
		complete(&ack);
	}

	printk("[%s]: thread for cpu%d stop\n", __func__, cpu);
	return 0;
}

void local_timer_test_init(void)
{
	int err = 0;
	int i = 0;
	unsigned char name[10] = { '\0' };
	struct task_struct *thread[nr_cpu_ids];
	static struct clock_event_device *evt;

	err = request_gpt(GPT6, GPT_FREE_RUN, GPT_CLK_SRC_SYS, GPT_CLK_DIV_1, 0, NULL, 0);
	if (err) {
		pr_err("fail to request gpt, err=%d\n", err);
	}

	err = request_percpu_irq(GIC_PPI_PRIVATE_TIMER, test_handler, "timer", &evt);
	if (err) {
		pr_err("can't register interrupt %d, err=%d\n", GIC_PPI_PRIVATE_TIMER, err);
	}

	init_completion(&ack);
	for (i = 0; i < nr_cpu_ids; i++) {
		cpuid[i] = i;
		init_completion(&notify[i]);
		sprintf(name, "timer-%d", i);
		thread[i] = kthread_create(local_timer_test, &cpuid[i], name);
		if (IS_ERR(thread[i])) {
			err = PTR_ERR(thread[i]);
			thread[i] = NULL;
			pr_err("[%s]: kthread_create %s fail(%d)\n", __func__, name, err);
			return;
		}
		kthread_bind(thread[i], i);
		wake_up_process(thread[i]);
	}
}


static int local_timer_test_read(char *page, char **start, off_t off,
				 int count, int *eof, void *data)
{
	char *p = page;
	int len = 0;

	p += sprintf(p, "********** ca7 timer debug help *********\n");
	p += sprintf(p, "echo opcode cpumask > /proc/lttest\n");
	p += sprintf(p, "opcode:\n");
	p += sprintf(p, "0: dump register\n");
	p += sprintf(p, "1: count down\n");
	p += sprintf(p, "2: count up\n");

	*start = page + off;

	len = p - page;
	if (len > off)
		len -= off;
	else
		len = 0;

	*eof = 1;

	return len < count ? len : count;
}

static int local_timer_test_write(struct file *file, const char *buffer,
				  unsigned long count, void *data)
{
	char desc[32];
	int len = 0;

	unsigned int i = 0;
	unsigned int cpu = 0;

	unsigned int mask = 0;

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len)) {
		return 0;
	}
	desc[len] = '\0';

	spin_lock(&cpu_lock);
	cpu = smp_processor_id();
	spin_unlock(&cpu_lock);
	printk("[%s]trigger test on cpu%u\n", __func__, cpu);

	mutex_lock(&opcode_lock);

	opcode = 0;
	op1 = 0;
	op2 = 0;

	sscanf(desc, "%u %x %x %x", &opcode, &mask, &op1, &op2);
	printk("opcode=%u, mask=%x, op1=%x, op2=%x\n", opcode, mask, op1, op2);

	for (i = 0; i < nr_cpu_ids; i++) {
		if (mask & (0x1 << i)) {
			complete(&notify[i]);
			wait_for_completion(&ack);
		}
	}

	mutex_unlock(&opcode_lock);

	return count;
}


static int __init local_timer_test_mod_init(void)
{
	struct proc_dir_entry *entry = NULL;

	entry = create_proc_entry("lttest", S_IRUGO | S_IWUSR, NULL);
	if (entry) {
		entry->read_proc = local_timer_test_read;
		entry->write_proc = local_timer_test_write;
	}

	local_timer_test_init();

	return 0;
}
module_init(local_timer_test_mod_init);
#endif
