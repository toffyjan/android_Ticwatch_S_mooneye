#include <mach/mt_timer.h>

extern struct mt_clock mt6572_gpt;
extern int generic_timer_register(void);
extern struct mt_clock mt_gpt;
extern unsigned long generic_timer_rate;
#define SYS_CLK_RATE        (13000000)

struct mt_clock *mt_clocks[] = {
	&mt_gpt,
};

void __init mt_timer_init(void)
{
	int i;
	struct mt_clock *clock;
	int err;

	for (i = 0; i < ARRAY_SIZE(mt_clocks); i++) {
		clock = mt_clocks[i];

		clock->init_func();

		if (clock->clocksource.name) {
			//err = clocksource_register(&(clock->clocksource));
			err = clocksource_register_hz(&(clock->clocksource), SYS_CLK_RATE);
			if (err) {
				pr_err("mt_timer_init: clocksource_register failed for %s\n",
				       clock->clocksource.name);
			}
		}

		err = setup_irq(clock->irq.irq, &(clock->irq));
		if (err) {
			pr_err("mt_timer_init: setup_irq failed for %s\n", clock->irq.name);
		}

		if (clock->clockevent.name)
			clockevents_register_device(&(clock->clockevent));
	}

	/* Assign value here to prevent dead loop in generic_timer_calibrate_rate() */
	generic_timer_rate = SYS_CLK_RATE;
	err = generic_timer_register();
	if (err) {
		pr_err("generic_timer_register failed, err=%d\n", err);
	}
}
