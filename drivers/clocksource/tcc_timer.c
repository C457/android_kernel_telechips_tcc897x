/*
 *  linux/drivers/clocksource/tcc_timer.c
 *
 *  Copyright (C) 2014 Telechips Inc.
 *  Copyright (C) 2011 ARM Ltd.
 *  All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/smp.h>
#include <linux/cpu.h>
#include <linux/clockchips.h>
#include <linux/interrupt.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/sched_clock.h>
#include <asm/cputype.h>

#define TC32EN		0x000	// 32-bit Counter Enable / Pre-scale Value
#define TC32CMP0	0x008	// 32-bit Counter Match Value 0
#define TC32CMP1	0x00C	// 32-bit Counter Match Value 1
#define TC32MCNT	0x014	// 32-bit Counter Current Value (main counter)
#define TC32IRQ		0x018	// 32-bit Counter Interrupt Control

#define TC32IRQ_EN0	(1<<16)
#define TC32IRQ_EN1	(1<<17)
#define TC32IRQ_CLR	(1<<31)

#define timer_writel(value, reg) __raw_writel(value, tcc_timer_base + (reg))
#define timer_readl(reg) __raw_readl(tcc_timer_base + (reg))

#ifndef CONFIG_LOCAL_TIMERS
static cpumask_var_t tick_broadcast_mask;
#endif

static u32 tcc_timer_rate;
static struct clock_event_device __percpu *tcc_timer_evt;

/* set up by the platform code */
static void __iomem *tcc_timer_base;
static int tcc_timer_ppi;

static irqreturn_t tcc_timer_handler(int irq, void *dev_id)
{
	struct clock_event_device *evt = this_cpu_ptr(dev_id);

	if (timer_readl(TC32IRQ)&(1<<8)) {
		timer_writel((timer_readl(TC32IRQ)&0xFFFF0000) & ~(1<<16), TC32IRQ);
		timer_writel((timer_readl(TC32IRQ)&0xFFFF0000) | (1<<8) | 1, TC32IRQ);
		evt->event_handler(evt);
	}

#ifndef CONFIG_LOCAL_TIMERS
	tick_broadcast(tick_broadcast_mask);
#endif

	return IRQ_HANDLED;
}


static void tcc_timer_set_mode(enum clock_event_mode mode,
				struct clock_event_device *clk)
{
	switch (mode) {
	case CLOCK_EVT_MODE_UNUSED:
	case CLOCK_EVT_MODE_SHUTDOWN:
		timer_writel((timer_readl(TC32IRQ)&0xFFFF0000) & ~(1<<16), TC32IRQ);
		break;
	default:
		break;
	}
}

static int tcc_timer_set_next_event(unsigned long evt,
			struct clock_event_device *unused)
{
	volatile unsigned long cnt;

	/* clear irq */
	timer_writel((timer_readl(TC32IRQ)&0xFFFF0000)|(1<<8)|1, TC32IRQ);

	/* set compare values*/
	cnt = timer_readl(TC32MCNT);
	cnt = (evt > (0xFFFFFFFF - cnt)) ? (evt - (0xFFFFFFFF-cnt)) : (cnt + evt);
	timer_writel(cnt, TC32CMP0);

	/* set irq */
	timer_writel((timer_readl(TC32IRQ)&0xFFFF0000)|(1<<16), TC32IRQ);

	return 0;
}

static int __cpuinit tcc_timer_setup(struct clock_event_device *clk)
{
	/*
	 * The following is done once per CPU the first time .setup() is
	 * called.
	 */
	clk->features = CLOCK_EVT_FEAT_ONESHOT/* | CLOCK_EVT_FEAT_C3STOP*/;
	clk->name = "tcc_timer";
	clk->rating = 200;
	clk->irq = tcc_timer_ppi;
	clk->set_mode = tcc_timer_set_mode;
	clk->set_next_event = tcc_timer_set_next_event;
	clk->cpumask = cpumask_of(smp_processor_id());
	clk->set_mode(CLOCK_EVT_MODE_SHUTDOWN, NULL);
	clockevents_config_and_register(clk, tcc_timer_rate,
					0xf, 0x7fffffff);
	return 0;
}

#ifndef CONFIG_LOCAL_TIMERS
static int __cpuinit tcc_timer_cpu_notify(struct notifier_block *self,
					  unsigned long action, void *hcpu)
{
	unsigned int cpu = (unsigned int) hcpu;

	if ((action & ~CPU_TASKS_FROZEN) == CPU_ONLINE)
		cpumask_set_cpu(cpu, tick_broadcast_mask);
	return NOTIFY_OK;
}

static struct notifier_block tcc_timer_cpu_nb __cpuinitdata = {
	.notifier_call = tcc_timer_cpu_notify,
};
#endif

static cycle_t tcc_counter_read(struct clocksource *cs) {
	return (cycle_t)timer_readl(TC32MCNT);
}

static cycle_t tcc_counter_read_cc(const struct cyclecounter *cc)
{
	return (cycle_t)timer_readl(TC32MCNT);
}

static struct clocksource clocksource_counter = {
	.name		= "tcc_sys_counter",
	.rating		= 200,
	.read		= tcc_counter_read,
	.mask		= CLOCKSOURCE_MASK(32),
	.flags		= CLOCK_SOURCE_IS_CONTINUOUS,
};

static struct cyclecounter cyclecounter = {
	.read	= tcc_counter_read_cc,
	.mask	= CLOCKSOURCE_MASK(32),
};

static struct timecounter timecounter;

static u32 notrace tcc_update_sched_clock(void)
{
	return timer_readl(TC32MCNT);
}

static int __init tcc_timer_register(void)
{
	int err;

	tcc_timer_evt = alloc_percpu(struct clock_event_device);
	if (!tcc_timer_evt) {
		err = -ENOMEM;
		goto out;
	}

	clocksource_register_hz(&clocksource_counter, tcc_timer_rate);
	cyclecounter.mult = clocksource_counter.mult;
	cyclecounter.shift = clocksource_counter.shift;
	timecounter_init(&timecounter, &cyclecounter,
			 (u64)timer_readl(TC32MCNT));

	sched_clock_register(tcc_update_sched_clock, 32, tcc_timer_rate);
	pr_info("Initialize the clocksource device.... rate[%u]\n", tcc_timer_rate);

//	err = request_percpu_irq(tcc_timer_ppi, tcc_timer_handler, "tcc_timer", tcc_timer_evt);
	err = request_irq(tcc_timer_ppi, tcc_timer_handler, 0, "tcc_timer", tcc_timer_evt);
	if (err) {
		pr_err("tcc_timer: can't register interrupt %d (%d)\n",
			tcc_timer_ppi, err);
		goto out_free;
	}

#ifndef CONFIG_LOCAL_TIMERS
	err = register_cpu_notifier(&tcc_timer_cpu_nb);
	if (err)
		goto out_free_irq;
#endif

	/* Immediately configure the timer on the boot CPU */
	if (tcc_timer_setup(this_cpu_ptr(tcc_timer_evt)))
		goto out_free_irq;

	return 0;

out_free_irq:
//	free_percpu_irq(tcc_timer_ppi, tcc_timer_evt);
	free_irq(tcc_timer_ppi, tcc_timer_evt);
out_free:
	free_percpu(tcc_timer_evt);
out:
	return err;
}

static void __init tcc_timer_init(struct device_node *np)
{
	struct clk *clk;

	tcc_timer_base = of_iomap(np, 0);
	if (!tcc_timer_base) {
		pr_warn("tcc_timer: failed to get register\n");
		return;
	}

	tcc_timer_ppi = irq_of_parse_and_map(np, 0);
	if (!tcc_timer_ppi) {
		pr_warn("tcc_timer: failed to get interrupt\n");
		return;
	}

	clk = of_clk_get(np, 0);
	if (IS_ERR(clk)) {
		pr_err("tcc_timer: clock not found %d\n", (int) PTR_ERR(clk));
		return;
	}

	if (of_property_read_u32(np, "clock-frequency", &tcc_timer_rate))
		tcc_timer_rate = 12000000;

	if (clk_prepare_enable(clk)) {
		pr_err("tcc_timer: clock failed to prepare+enable\n");
		clk_put(clk);
		return;
	}

#ifndef CONFIG_LOCAL_TIMERS
	cpumask_empty(tick_broadcast_mask);
#endif

	clk_set_rate(clk, tcc_timer_rate);
	tcc_timer_rate = clk_get_rate(clk);

	/* Initialize the timer */
	timer_writel(0, TC32EN);			/* Timer Disable, Prescale is 0 */
	timer_writel((1<<24), TC32EN);			/* Timer Enable */
	timer_writel(TC32IRQ_CLR|0x1F1F, TC32IRQ);	/* IRQ clear */

	tcc_timer_register();
}

CLOCKSOURCE_OF_DECLARE(tcc_timer, "telechips,timer32", tcc_timer_init);
