/*
 * linux/drivers/rtc/tcc_rtc.c
 *
 * Author: <linux@telechips.com>
 * Created: Dec 7, 2011
 * Description: RTC driver for Telechips TCC Series
 *
 * Copyright (C) 2011 Telechips
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see the file COPYING, or write
 * to the Free Software Foundation, Inc.,
 * 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */


#include <linux/module.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/rtc.h>
#include <linux/bcd.h>
#include <linux/clk.h>
#include <linux/log2.h>

//#include <mach/bsp.h>
//#include <mach/hardware.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/delay.h>
#include <asm/mach/time.h>
#include "tcc/tca_alarm.h"

#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <mach/daudio.h>

#define DRV_NAME "tcc-rtc"

//#define RTC_PMWKUP_TEST	// This option will set alarm 0sec ~ 59sec. Check it is working correctly.

#ifdef RTC_PMWKUP_TEST
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/delay.h>
struct task_struct *alaram_test_thread;
rtctime pTime_test;
static atomic_t irq_flag = ATOMIC_INIT(0);
#endif

#if 1
#define RTCCON      0x00
#define INTCON      0x04
#define RTCALM      0x08
#define ALMSEC      0x0C
#define ALMMIN      0x10
#define ALMHOUR     0x14
#define ALMDATE     0x18
#define ALMDAY      0x1C
#define ALMMON      0x20
#define ALMYEAR     0x24
#define BCDSEC      0x28
#define BCDMIN      0x2C
#define BCDHOUR     0x30
#define BCDDATE     0x34
#define BCDDAY      0x38
#define BCDMON      0x3C
#define BCDYEAR     0x40
#define RTCIM       0x44
#define RTCPEND     0x48
#define RTCSTR		0x4C
#endif

#define rtc_readl	__raw_readl
#define rtc_writel	__raw_writel
#define rtc_reg(x) (*(unsigned int *)(tcc_rtc->regs + x))


struct tcc_rtc_data {
	void __iomem	*regs;
	struct clk	*fclk;
	struct clk	*hclk;
	int		irq;
	struct rtc_device *rtc_dev;
};

static int g_aie_enabled = 0;

/* IRQ Handlers */
static irqreturn_t tcc_rtc_alarmirq(int irq, void *class_dev)
{

	struct tcc_rtc_data *tcc_rtc = class_dev;

	printk("[tcc_rtc_alarmirq][%u]", irq);
	//local_irq_disable();

	BITSET(rtc_reg(RTCCON), Hw1);			// RTC Register write enabled
	BITSET(rtc_reg(INTCON), Hw0);			// Interrupt Block Write Enable


	BITCLR(rtc_reg(RTCIM), 0xFFFFFFFF);		// Clear Interrupt Setting values.
	BITSET(rtc_reg(RTCIM), Hw2);			// Change Operation Mode from PowerDown Mode to Normal Operation Mode.
	BITCLR(rtc_reg(RTCPEND), 0xFFFFFFFF);	// PEND bit Clear - Clear RTC Wake-Up pin.
	BITSET(rtc_reg(RTCSTR), Hw6|Hw7);		// RTC Alarm, wake-up interrupt pending clear
	printk("RTCIM[%#x] RTCPEND[%#x] RTCSTR[%#x]\n", rtc_readl(tcc_rtc->regs + RTCIM), rtc_readl(tcc_rtc->regs + RTCPEND), rtc_readl(tcc_rtc->regs + RTCSTR));


	BITCLR(rtc_reg(RTCCON), Hw1);			// RTC Register write Disable
	BITCLR(rtc_reg(INTCON), Hw0);			// Interrupt Block Write Disable

	//local_irq_enable();

	rtc_update_irq(tcc_rtc->rtc_dev, 1, RTC_AF | RTC_IRQF);

#ifdef RTC_PMWKUP_TEST
	printk("\x1b[1;33mRTC TEST : %s __________________ \x1b[0m\n", __func__);
	atomic_set(&irq_flag, 0);
#endif

	return IRQ_HANDLED;
}

/* Update control registers */
static void tcc_rtc_setaie(struct device *dev, int to)
{
	struct tcc_rtc_data *tcc_rtc = dev_get_drvdata(dev);
	unsigned int tmp;

	printk("%s: aie=%d\n", __func__, to);

#if !defined(CONFIG_TCC_CODESONAR_BLOCKED)
	if (!tcc_rtc)
		return -ENOMEM;
#endif
	rtc_writel( rtc_readl(tcc_rtc->regs + RTCCON) | Hw1, tcc_rtc->regs + RTCCON);
	rtc_writel( rtc_readl(tcc_rtc->regs + INTCON) | Hw0, tcc_rtc->regs + INTCON);

	tmp = rtc_readl(tcc_rtc->regs + RTCALM) & ~Hw7;

	if (to)
		tmp |= Hw7;

	rtc_writel(tmp, tcc_rtc->regs + RTCALM);

	//rtc_writel( rtc_readl(tcc_rtc->regs + INTCON) & ~Hw0, tcc_rtc->regs + INTCON);
	rtc_writel( rtc_readl(tcc_rtc->regs + RTCCON) & ~Hw1, tcc_rtc->regs + RTCCON);
}

int tcc_rtc_alarm_irq_enable(struct device *dev, unsigned int enabled)
{
	g_aie_enabled = enabled;
	tcc_rtc_setaie(dev,enabled);

	return 0;
}

/* Time read/write */
static int tcc_rtc_gettime(struct device *dev, struct rtc_time *rtc_tm)
{
	struct tcc_rtc_data *tcc_rtc = dev_get_drvdata(dev);
	rtctime pTime;

	local_irq_disable();
	tca_rtc_gettime((unsigned int)tcc_rtc->regs, &pTime);

	rtc_tm->tm_sec  = pTime.wSecond;
	rtc_tm->tm_min  = pTime.wMinute;
	rtc_tm->tm_hour = pTime.wHour;
	rtc_tm->tm_mday = pTime.wDay;
	rtc_tm->tm_wday = pTime.wDayOfWeek;
	rtc_tm->tm_mon  = pTime.wMonth - 1;
	rtc_tm->tm_year = pTime.wYear - 1900;

	printk("read time %04d/%02d/%02d %02d:%02d:%02d\n",
			pTime.wYear, pTime.wMonth, pTime.wDay,
			pTime.wHour, pTime.wMinute, pTime.wSecond);

	local_irq_enable();

	return 0;
}

static int tcc_rtc_settime(struct device *dev, struct rtc_time *tm)
{
	struct tcc_rtc_data *tcc_rtc = dev_get_drvdata(dev);
	rtctime pTime;

	local_irq_disable();

	pTime.wSecond       = tm->tm_sec;
	pTime.wMinute       = tm->tm_min;
	pTime.wHour         = tm->tm_hour;
	pTime.wDay          = tm->tm_mday;
	pTime.wDayOfWeek    = tm->tm_wday;
	pTime.wMonth        = tm->tm_mon + 1;
	pTime.wYear         = tm->tm_year + 1900;

	printk("set time %02d/%02d/%02d %02d:%02d:%02d\n",
			pTime.wYear, pTime.wMonth, pTime.wDay,
			pTime.wHour, pTime.wMinute,  pTime.wSecond);


	tca_rtc_settime((unsigned int)tcc_rtc->regs, &pTime);

	local_irq_enable();

	return 0;
}

static int tcc_rtc_getalarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct tcc_rtc_data *tcc_rtc = dev_get_drvdata(dev);
	struct rtc_time *alm_tm = &alrm->time;
	unsigned int alm_en, alm_pnd;
	rtctime pTime;
	printk("%s\n", __func__);

	local_irq_disable();

	rtc_writel( rtc_readl(tcc_rtc->regs + RTCCON) | Hw1, tcc_rtc->regs + RTCCON);
	rtc_writel( rtc_readl(tcc_rtc->regs + INTCON) | Hw0, tcc_rtc->regs + INTCON);

	alm_en = rtc_readl(tcc_rtc->regs + RTCALM);
	alm_pnd = rtc_readl(tcc_rtc->regs + RTCPEND);

	alrm->enabled = (alm_en & Hw7) ? 1 : 0;
	alrm->pending = (alm_pnd & Hw0) ? 1 : 0;

	printk(" alrm->enabled = %d, alm_en = %d\n", alrm->enabled, alm_en);

	rtc_writel( rtc_readl(tcc_rtc->regs + INTCON) & ~Hw0, tcc_rtc->regs + INTCON);
	rtc_writel( rtc_readl(tcc_rtc->regs + RTCCON) & ~Hw1, tcc_rtc->regs + RTCCON);

	tca_alarm_gettime((unsigned int)tcc_rtc->regs, &pTime);

	alm_tm->tm_sec  = pTime.wSecond;
	alm_tm->tm_min  = pTime.wMinute;
	alm_tm->tm_hour = pTime.wHour;
	alm_tm->tm_mday = pTime.wDay ;
	alm_tm->tm_mon  = pTime.wMonth - 1;
	alm_tm->tm_year = pTime.wYear - 1900;

	printk("read alarm %02x %02x/%02x/%02x %02x:%02x:%02x\n",
			alm_en,
			pTime.wYear, pTime.wMonth, pTime.wDay,
			pTime.wHour, pTime.wMinute, pTime.wSecond);

	local_irq_enable();

	return 0;
}

static struct rtc_time rtctime_to_rtc_time(rtctime pTime)
{
	struct rtc_time tm;

	tm.tm_sec	= pTime.wSecond;
	tm.tm_min	= pTime.wMinute;
	tm.tm_hour	= pTime.wHour;
	tm.tm_mday	= pTime.wDay;
	tm.tm_wday	= pTime.wDayOfWeek;
	tm.tm_mon	= pTime.wMonth - 1;
	tm.tm_year	= pTime.wYear - 1900;

	return tm;
}

static rtctime rtc_time_to_rtctime(struct rtc_time tm)
{
	rtctime pTime;

	pTime.wSecond		= tm.tm_sec;
	pTime.wMinute		= tm.tm_min;
	pTime.wHour			= tm.tm_hour;
	pTime.wDay			= tm.tm_mday;
	pTime.wDayOfWeek	= tm.tm_wday;
	pTime.wMonth		= tm.tm_mon + 1;
	pTime.wYear			= tm.tm_year + 1900;

	return pTime;
}

static int check_time(rtctime now_time, rtctime new_time)
{
	struct rtc_time now_tm, new_tm;
	ktime_t now_kt, new_kt;

	now_tm = rtctime_to_rtc_time(now_time);
	new_tm = rtctime_to_rtc_time(new_time);

	now_kt = rtc_tm_to_ktime(now_tm);
	new_kt = rtc_tm_to_ktime(new_tm);

	if (now_kt.tv64 == new_kt.tv64) {
		return 0;			// now_time and new_time is same.
	}

#if 1
	/*		if 1sec is different, but this function regard it same.		*/
	if( (now_kt.tv64 + NSEC_PER_SEC) == new_kt.tv64) {
		return 0;			// now_time + 1sec and new_time is same.
	}

	if( (now_kt.tv64 - NSEC_PER_SEC) == new_kt.tv64) {
		return 0;			// now_time - 1sec and new_time is same.
	}
#endif

	return -1;
}

static int tcc_rtc_setalarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct tcc_rtc_data *tcc_rtc = dev_get_drvdata(dev);
	rtctime pTime;
	rtctime now_time, new_time;

	struct rtc_time *tm = &alrm->time;

	int ret = 0;

	printk("%s\n", __func__);

	alrm->enabled = 1;

	pTime.wSecond   = tm->tm_sec;
	pTime.wMinute   = tm->tm_min;
	pTime.wHour     = tm->tm_hour;
	pTime.wDay      = tm->tm_mday;
	pTime.wMonth    = tm->tm_mon + 1;
	pTime.wYear     = tm->tm_year + 1900;

	printk("set alarm %02d/%02d/%02d %02d:%02d:%02d \n",
			pTime.wYear, pTime.wMonth, pTime.wDay,
			pTime.wHour, pTime.wMinute, pTime.wSecond);

	local_irq_disable();

	if (g_aie_enabled) {
		tcc_rtc_setaie(dev,0);
	}	
	
#if 1	// Original Alarm Set Code.
	tca_alarm_settime((unsigned int)tcc_rtc->regs, &pTime);


	BITSET(rtc_reg(RTCCON), Hw1);	// Interrupt Block Write Enable bit - Enable
	BITSET(rtc_reg(INTCON), Hw0);	// RTC write enable bit - Enable

	BITSET(rtc_reg(RTCALM), Hw7);   //  Alarm Global Set & Enable
	enable_irq_wake(tcc_rtc->irq);

	BITCLR(rtc_reg(INTCON), Hw0);	// RTC write enable bit - Disable
	BITCLR(rtc_reg(RTCCON), Hw1);	// Interrupt Block Write Enable bit - Disable

	if (g_aie_enabled) {
		tcc_rtc_setaie(dev,1);
	}
	
#else	// Changed Alarm Set Code to avoid changed RTC Time problem.

#define STABLE_WAIT	0x10000000
#define SET_ALARM_LOOP	1000
	int iLoop = SET_ALARM_LOOP;


	BITSET(rtc_reg(RTCCON), Hw1);			// RTC Register write enabled
	BITSET(rtc_reg(INTCON), Hw0);			// Interrupt Block Write Enable

	BITSET(rtc_reg(RTCCON), Hw0);			// RTC Start bit - Halt

	tca_rtc_gettime((unsigned int)tcc_rtc->regs, &now_time);

	while(iLoop--) {
		tca_alarm_settime((unsigned int)tcc_rtc->regs, &pTime);
	
		BITSET(rtc_reg(RTCCON), Hw1);	// Interrupt Block Write Enable bit - Enable
		BITSET(rtc_reg(INTCON), Hw0);	// RTC write enable bit - Enable

		BITSET(rtc_reg(RTCALM), Hw7);	//  Alarm Global Set & Enable
		enable_irq_wake(tcc_rtc->irq);

		BITCLR(rtc_reg(INTCON), Hw0);	// RTC write enable bit - Disable
		BITCLR(rtc_reg(RTCCON), Hw1);	// Interrupt Block Write Enable bit - Disable

		{ unsigned int wait_stable = STABLE_WAIT; while(wait_stable-- > 0) { asm("nop"); } }

		tca_rtc_gettime((unsigned int)tcc_rtc->regs, &new_time);

	
		if( check_time(now_time, new_time) ) {
			/*		initialize RTC Alarm Registers		*/
			BITSET(rtc_reg(RTCCON), Hw1);			// RTC Register write enabled
			BITSET(rtc_reg(INTCON), Hw0);			// Interrupt Block Write Enable

			BITCLR(rtc_reg(RTCIM), 0xFFFFFFFF);		// Clear Interrupt Setting values.
			BITSET(rtc_reg(RTCIM), Hw2);			// Change Operation Mode from PowerDown Mode to Normal Operation Mode.
			BITCLR(rtc_reg(RTCPEND), 0xFFFFFFFF);	// PEND bit Clear - Clear RTC Wake-Up pin.
			BITSET(rtc_reg(RTCSTR), Hw6|Hw7);		// RTC Alarm, wake-up interrupt pending clear
			BITCLR(rtc_reg(RTCSTR), Hw3|Hw2|Hw1|Hw0);	// Clear Alarm Wake-Up Count value.	

			BITCLR(rtc_reg(RTCCON), Hw1);			// RTC Register write Disable
			BITCLR(rtc_reg(INTCON), Hw0);			// Interrupt Block Write Disable

			/*		Set time to correct time			*/
			tca_rtc_settime((unsigned int)tcc_rtc->regs, &now_time);
		} else
			break;
	}
	printk("set alarm - : %d\n", SET_ALARM_LOOP - iLoop);


	BITSET(rtc_reg(RTCCON), Hw1);	// Interrupt Block Write Enable bit - Enable
	BITSET(rtc_reg(INTCON), Hw0);	// RTC write enable bit - Enable

	BITCLR(rtc_reg(RTCCON), Hw0);			// RTC Start bit - run

	BITCLR(rtc_reg(INTCON), Hw0);	// RTC write enable bit - Disable
	BITCLR(rtc_reg(RTCCON), Hw1);	// Interrupt Block Write Enable bit - Disable


	if( iLoop <= 0 ) 
	{
		BITSET(rtc_reg(RTCCON), Hw1);	// Interrupt Block Write Enable bit - Enable
		BITSET(rtc_reg(INTCON), Hw0);	// RTC write enable bit - Enable


		BITCLR(rtc_reg(RTCALM), Hw7);	//  Alarm Global Disable
		disable_irq_wake(tcc_rtc->irq);
		
		BITCLR(rtc_reg(RTCCON), Hw1);	// RTC Register write Disable
		BITCLR(rtc_reg(INTCON), Hw0);	// Interrupt Block Write Disable

		ret = -EIO;
	}
#endif	// Changed Alarm Set Code to avoid changed RTC Time problem.

	local_irq_enable();

	return ret;
}

static int tcc_rtc_proc(struct device *dev, struct seq_file *seq)
{
	return 0;
}

static int tcc_rtc_ioctl(struct device *dev,
		unsigned int cmd, unsigned long arg)
{
	int ret = -ENOIOCTLCMD;

printk("%s %d\n",__func__,__LINE__);
	switch (cmd) {
		case RTC_AIE_OFF:
			tcc_rtc_setaie(dev, 0);
			ret = 0;
			break;
		case RTC_AIE_ON:
			tcc_rtc_setaie(dev, 1);
			ret = 0;
			break;
		case RTC_PIE_OFF:
			break;
		case RTC_PIE_ON:
			break;
		case RTC_IRQP_READ:
			break;
		case RTC_IRQP_SET:
			break;
		case RTC_UIE_ON:
			break;
		case RTC_UIE_OFF:
			break;
			ret = -EINVAL;
	}

	return ret;
}

static const struct rtc_class_ops tcc_rtcops = {
	.read_time	= tcc_rtc_gettime,
	.set_time	= tcc_rtc_settime,
	.read_alarm	= tcc_rtc_getalarm,
	.set_alarm	= tcc_rtc_setalarm,
	.alarm_irq_enable = tcc_rtc_alarm_irq_enable,
	.proc		= tcc_rtc_proc,
	.ioctl		= tcc_rtc_ioctl,
};

static int tcc_rtc_remove(struct platform_device *pdev)
{
	struct tcc_rtc_data *tcc_rtc = platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);
	rtc_device_unregister(tcc_rtc->rtc_dev);
	tcc_rtc_setaie(&pdev->dev, 0);
	free_irq(tcc_rtc->irq, &pdev->dev);

	if (tcc_rtc->fclk) {
		clk_disable_unprepare(tcc_rtc->fclk);
		clk_put(tcc_rtc->fclk);
		tcc_rtc->fclk = NULL;
	}
	if (tcc_rtc->hclk) {
		clk_disable_unprepare(tcc_rtc->hclk);
		clk_put(tcc_rtc->hclk);
		tcc_rtc->hclk = NULL;
	}

	return 0;
}

#ifdef RTC_PMWKUP_TEST
static int cmp_rtc_gettime(struct device *dev)
{
	rtctime pTime;
	struct rtc_time now_time;

	tcc_rtc_gettime(dev, &now_time);

	pTime = rtc_time_to_rtctime(now_time);

	printk("\x1b[1;33mRTC TEST : now irq time %04d/%02d/%02d %02d:%02d:%02d\x1b[0m\n",
			pTime.wYear, pTime.wMonth, pTime.wDay,
			pTime.wHour, pTime.wMinute, pTime.wSecond);

	if (check_time(pTime, pTime_test)) {
		printk("\x1b[1;31mRTC TEST : _______________________ alarm un-matched time.\x1b[0m\n");
	}

	return 0;
}
static int get_next_sec(int set_sec)
{
	int out;

#if 0
	out = set_sec +1;
#else
	int interval = 10;

	out = set_sec + interval;
	if (out == 60) {
		out = 1;
	} else if (out > 60) {
		out = out % 10;
		out++;

		if (out >= 10) {
			return 60;
		}
	}
#endif

	return out;
}

static int tcc_alarm_test(void *arg)
{
	struct device *dev = (struct device*)arg;

	struct tcc_rtc_data *tcc_rtc = dev_get_drvdata(dev);
	struct rtc_time rtc_tm, now_time;
	struct rtc_time *tm = &rtc_tm;
	rtctime pTime;

	int ret = 0;
	int set_sec = 0;

	printk("\x1b[1;35mRTC TEST :  __________________ %s START\x1b[0m\n", __func__);
	msleep(30000);

	do {
#if 0
		/*	If irq is not freed irq & initialized irq again, sometimes irq is not work.	*/
		devm_free_irq(dev, tcc_rtc->irq, tcc_rtc);
		msleep(2000);
		ret = devm_request_irq(dev, tcc_rtc->irq, tcc_rtc_alarmirq, 0, DRV_NAME, tcc_rtc);
		if (ret) {
			printk(KERN_ERR "\x1b[1;31mRTC TEST : RTC timer interrupt IRQ%d already claimed\x1b[0m\n", tcc_rtc->irq);
		}
#endif

		if (set_sec >= 60)
			break;

		printk("\x1b[1;35mRTC TEST : Set Alarm - sec[%d]\x1b[0m\n", set_sec);
		tcc_rtc_gettime(dev, tm);

		if ( tm->tm_sec >= (set_sec -2) )   // -2 is delay time to set alarm.
		{
			tm->tm_min += 1;
		}
		tm->tm_sec = set_sec;

		if( tm->tm_min >= 60)
		{
			tm->tm_min %= 60;
			tm->tm_hour++;
		}

		if( tm->tm_hour >= 24)
		{
			tm->tm_hour %= 24;
			tm->tm_mday++;
		}

		struct rtc_wkalrm rtc_al;
		rtc_al.time = *tm;
		tcc_rtc_setalarm(dev, &rtc_al);
		atomic_set(&irq_flag, 1);

		pTime.wSecond   = tm->tm_sec;
		pTime.wMinute   = tm->tm_min;
		pTime.wHour     = tm->tm_hour;
		pTime.wDay      = tm->tm_mday;
		pTime.wMonth    = tm->tm_mon + 1;
		pTime.wYear     = tm->tm_year + 1900;
	
		printk("\x1b[1;33mRTC TEST : set alarm %02d/%02d/%02d %02d:%02d:%02d\x1b[0m\n",
				pTime.wYear, pTime.wMonth, pTime.wDay,
				pTime.wHour, pTime.wMinute, pTime.wSecond);

		memcpy(&pTime_test, &pTime, sizeof(rtctime));

		set_sec = get_next_sec(set_sec);

		while(atomic_read(&irq_flag)) { msleep(400); };	// Wait until interrupt is occured.

		cmp_rtc_gettime(dev);
	} while(1);

	printk("\x1b[1;34mRTC TEST : Alarm Test is finished.\x1b[0m\n");

	return 0;
}
#endif	// RTC_PMWKUP_TEST

static const struct of_device_id tcc_rtc_of_match[];

static int tcc_rtc_probe(struct platform_device *pdev)
{
	struct tcc_rtc_data *tcc_rtc;
	int ret;
	unsigned int ulINTCON;
	int iLoop = 1000;

	tcc_rtc = devm_kzalloc(&pdev->dev, sizeof(struct tcc_rtc_data), GFP_KERNEL);
	if (!tcc_rtc) {
		dev_err(&pdev->dev, "failed to allocate memory\n");
		return -ENOMEM;
	}
	platform_set_drvdata(pdev, tcc_rtc);

	tcc_rtc->regs = of_iomap(pdev->dev.of_node, 0);
	if (tcc_rtc->regs == NULL) {
		printk("failed RTC of_iomap()\n");
		ret = -ENOMEM;
		goto err_get_dt_property;
	}

	tcc_rtc->irq = platform_get_irq(pdev, 0);
	if (tcc_rtc->irq <= 0) {
		dev_err(&pdev->dev, "no irq for alarm\n");
		ret = -ENOENT;
		goto err_get_dt_property;
	}

	tcc_rtc->hclk = of_clk_get(pdev->dev.of_node, 1);
	if (IS_ERR(tcc_rtc->hclk)) {
		dev_err(&pdev->dev, "failed to get hclk\n");
		ret = -ENXIO;
		goto err_get_hclk;
	}
	clk_prepare_enable(tcc_rtc->hclk);

	tcc_rtc->fclk = of_clk_get(pdev->dev.of_node, 0);
	if (IS_ERR(tcc_rtc->fclk)) {
		dev_err(&pdev->dev, "failed to get fclk\n");
		ret = -ENXIO;
		goto err_get_fclk;
	}
	clk_prepare_enable(tcc_rtc->fclk);


	BITSET(rtc_reg(RTCCON), Hw1);			// RTC Write Enable
	BITSET(rtc_reg(INTCON), Hw0);			// Interrupt Write Enable	
	//printk("FIRST INTCON[0x%x]RTCCON[0x%x]", rtc_readl(tcc_rtc->regs + INTCON), rtc_readl(tcc_rtc->regs + RTCCON));
	

	BITSET(rtc_reg(RTCCON), Hw0);			// RTC Start bit - Halt
	BITCLR(rtc_reg(INTCON), Hw15);			// Disable Protection.

	BITCLR(rtc_reg(INTCON), Hw13|Hw12);		// 32.768kHz XTAL @ 1.8V	- XTAL I/O Driver Strength Select
	BITCLR(rtc_reg(INTCON), Hw10|Hw9|Hw8);	// 32.768kHz XTAL - Divider Output Select	- FSEL

	BITSET(rtc_reg(INTCON), Hw15);			// Enable Protection.


	//Start : Disable the RTC Alarm - 120125, hjbae
	// Disable - Wake Up Interrupt Output(Hw7), Alarm Interrupt Output(Hw6)
	BITCLR(rtc_reg(RTCCON), Hw7|Hw6);		// Disable RTC Wake-Up, Alarm Interrupt.
	
	// Disable - Alarm Control
	BITCLR(rtc_reg(RTCALM), 0xFFFFFFFF);	// Clear RTCALM reg - Disable Alarm
	
	// Power down mode, Active HIGH, Disable alarm interrupt
	BITSET(rtc_reg(RTCIM), Hw2);			// ActiveHigh
	BITCLR(rtc_reg(RTCIM), Hw3|Hw1|Hw0);	// PowerDown Mode, Disable Interrupt.

	// printk("CUR INTCON[0x%x]\n", rtc_readl(tcc_rtc->regs + INTCON));

	BITCLR(rtc_reg(RTCCON), Hw0);			// RTC Start bit - Run
	
	BITCLR(rtc_reg(RTCCON), Hw1);			// RTC Write Disable
	BITCLR(rtc_reg(INTCON), Hw0);			// Interrupt Write Disable


	// Check Valid Time
	if(tca_rtc_checkvalidtime((unsigned int)tcc_rtc->regs))
	{
		/*              Invalied Time           */
		rtctime pTime;

		// temp init;
		pTime.wDay      = DAUDIO_RTC_DAY;
		pTime.wMonth    = DAUDIO_RTC_MONTH;
		pTime.wDayOfWeek= DAUDIO_RTC_WEEK;
		pTime.wHour     = DAUDIO_RTC_HOUR;
		pTime.wMinute   = DAUDIO_RTC_MINUTE;
		pTime.wSecond   = DAUDIO_RTC_SECOND;
		pTime.wYear     = DAUDIO_RTC_YEAR;	// year 2038 problem.

		tca_rtc_settime((unsigned int)tcc_rtc->regs, &pTime);

		printk("RTC Invalied Time, Set Time %04d/%02d/%02d %02d:%02d:%02d\n",
				pTime.wYear, pTime.wMonth, pTime.wDay,
				pTime.wHour, pTime.wMinute, pTime.wSecond);
	}

	printk("tcc_rtc: alarm irq %d\n", tcc_rtc->irq);

	if (tcc_rtc->irq < 0) {
		dev_err(&pdev->dev, "no irq for alarm\n");
		return -ENOENT;
	}

	// From kernel 3.4 version, 
	// the following function has to be used to wake device from suspend mode.
	device_init_wakeup(&pdev->dev, true);

	/* register RTC and exit */
	tcc_rtc->rtc_dev = rtc_device_register(pdev->name, &pdev->dev, &tcc_rtcops, THIS_MODULE);
	if (IS_ERR(tcc_rtc->rtc_dev)) {
		dev_err(&pdev->dev, "cannot attach rtc\n");
		ret = PTR_ERR(tcc_rtc->rtc_dev);
		goto err_nortc;
	}

	//Use threaded IRQ, remove IRQ enable in interrupt handler - 120126, hjbae
	//    if(request_irq(tcc_rtc->irq, tcc_rtc_alarmirq, IRQF_DISABLED, DRV_NAME, rtc)) {
	ret = devm_request_irq(&pdev->dev, tcc_rtc->irq, tcc_rtc_alarmirq, 0, DRV_NAME, tcc_rtc);
	if (ret) {
		dev_err(&pdev->dev, "%s: RTC timer interrupt IRQ%d already claimed\n",
			pdev->name, tcc_rtc->irq);
		goto err_request_irq;
	}

#ifdef RTC_PMWKUP_TEST
	alaram_test_thread = (struct task_struct*)kthread_run(tcc_alarm_test, 
						(void*)&pdev->dev, "alarm_test");
#endif

	return 0;

err_request_irq:
	rtc_device_unregister(tcc_rtc->rtc_dev);

err_nortc:
	clk_disable_unprepare(tcc_rtc->fclk);
	clk_put(tcc_rtc->fclk);

err_get_fclk:
	clk_disable_unprepare(tcc_rtc->hclk);
	clk_put(tcc_rtc->hclk);	

err_get_hclk:
	
err_get_dt_property:
	devm_kfree(&pdev->dev, tcc_rtc);

	return ret;
}


#ifdef CONFIG_PM

/* RTC Power management control */
static int tcc_rtc_suspend(struct device *dev)
{
	struct tcc_rtc_data *tcc_rtc = dev_get_drvdata(dev);
	
//	if (device_may_wakeup(dev)) {
//		enable_irq_wake(tcc_rtc->irq);
//	}

	BITSET(rtc_reg(RTCCON), Hw1);
	BITSET(rtc_reg(INTCON), Hw0);
	BITSET(rtc_reg(RTCIM), Hw3);	/* set PWDN - Power Down Mode */
	BITCLR(rtc_reg(INTCON), Hw0);
	BITCLR(rtc_reg(RTCCON), Hw1);

	return 0;
}

static int tcc_rtc_resume(struct device *dev)
{
	struct tcc_rtc_data *tcc_rtc = dev_get_drvdata(dev);

//	if (device_may_wakeup(dev)) {
//		disable_irq_wake(tcc_rtc->irq);
//	}

	BITSET(rtc_reg(RTCCON), Hw1);
	BITSET(rtc_reg(INTCON), Hw0);
	BITCLR(rtc_reg(RTCIM), Hw3);	/* set PWDN - Normal Operation */
	BITCLR(rtc_reg(INTCON), Hw0);
	BITCLR(rtc_reg(RTCCON), Hw1);

	return 0;
}

static int tcc_rtc_restore(struct device *dev)
{
	struct tcc_rtc_data *tcc_rtc = dev_get_drvdata(dev);

	// Check Valid Time
	if(tca_rtc_checkvalidtime((unsigned int)tcc_rtc->regs))
	{
		/*              Invalied Time           */
		rtctime pTime;

		// temp init;
		pTime.wDay      = DAUDIO_RTC_DAY;
		pTime.wMonth    = DAUDIO_RTC_MONTH;
		pTime.wDayOfWeek= DAUDIO_RTC_WEEK;
		pTime.wHour     = DAUDIO_RTC_HOUR;
		pTime.wMinute   = DAUDIO_RTC_MINUTE;
		pTime.wSecond   = DAUDIO_RTC_SECOND;
		pTime.wYear     = DAUDIO_RTC_YEAR;	// year 2038 problem.

		tca_rtc_settime((unsigned int)tcc_rtc->regs, &pTime);

		printk("RTC Invalied Time, Set Time %04d/%02d/%02d %02d:%02d:%02d\n",
				pTime.wYear, pTime.wMonth, pTime.wDay,
				pTime.wHour, pTime.wMinute, pTime.wSecond);
	}

//	if (device_may_wakeup(dev)) {
//		disable_irq_wake(tcc_rtc->irq);
//	}
	
	BITSET(rtc_reg(RTCCON), Hw1);
	BITSET(rtc_reg(INTCON), Hw0);
	BITCLR(rtc_reg(RTCIM), Hw3);	/* set PWDN - Normal Operation */
	BITCLR(rtc_reg(INTCON), Hw0);
	BITCLR(rtc_reg(RTCCON), Hw1);
	
	return 0;
}

#else
#define tcc_rtc_suspend NULL
#define tcc_rtc_resume  NULL
#endif

/*
static SIMPLE_DEV_PM_OPS(tcc_rtc_pm_ops, tcc_rtc_suspend,
			 tcc_rtc_resume);
*/			 
#ifdef CONFIG_PM
static const struct dev_pm_ops tcc_rtc_pm_ops = {
//	.runtime_suspend      = tcc_rtc_runtime_suspend,
//	.runtime_resume       = tcc_rtc_runtime_resume,
	.suspend    = tcc_rtc_suspend,
	.resume = tcc_rtc_resume,
	.freeze = tcc_rtc_suspend,
	.thaw = tcc_rtc_resume,
	.restore = tcc_rtc_restore,
};
#endif

static const struct of_device_id tcc_rtc_of_match[] = {
	{ .compatible = "telechips,rtc", },
	{},
};

static struct platform_driver tcc_rtc_driver = {
	.probe		= tcc_rtc_probe,
	.remove		= tcc_rtc_remove,
#ifndef CONFIG_PM
	.suspend	= tcc_rtc_suspend,
	.resume		= tcc_rtc_resume,
#endif
	.driver		= {
		.name	= DRV_NAME,
		.owner	= THIS_MODULE,
		.pm	= &tcc_rtc_pm_ops,
		.of_match_table = of_match_ptr(tcc_rtc_of_match),
	},
};

module_platform_driver(tcc_rtc_driver);

MODULE_AUTHOR("linux <linux@telechips.com>");
MODULE_DESCRIPTION("Telechips RTC Driver");
MODULE_LICENSE("GPL");
