/*
 *  drivers/switch/switch_gpio.c
 *
 * Copyright (C) 2008 Google, Inc.
 * Author: Mike Lockwood <lockwood@android.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/switch.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>

#include <asm/io.h>
#include <mach/bsp.h>
#include <mach/irqs.h>
#include <asm/mach-types.h>

#include <linux/kthread.h>	/* thread */
#include <linux/delay.h>	/* msleep_interruptible */

#define TCC_BIT_HEADSET				(1 << 0)
#define TCC_BIT_HEADSET_NO_MIC		(1 << 1)
#define TCC_BIT_USB_HEADSET_ANLG	(1 << 2)
#define TCC_BIT_USB_HEADSET_DGTL	(1 << 3)
#define TCC_BIT_HDMI_AUDIO			(1 << 4)

#define TCC_BIT_FM_HEADSET			(1 << 5)
#define TCC_BIT_FM_SPEAKER			(1 << 6)

#define EARJACK_DETECT_GPIO_NUM		0x00000800

#undef tcc_dbg
#if 1
#define tcc_dbg(fmt,arg...)
#else
#define tcc_dbg(fmt,arg...)			printk(fmt,##arg)
#endif

struct tcc_gpio_switch_data {
	struct switch_dev sdev;
	unsigned gpio;
	const char *name_on;
	const char *name_off;
	const char *state_on;
	const char *state_off;
	unsigned int state_val;
	int irq;
	struct work_struct work;
	struct task_struct *poll_task;
};

static int gJack_disconnect_count = 0;
static unsigned tcc_hpdet_port;

static void tcc_gpio_switch_work(struct work_struct *work)
{
	struct tcc_gpio_switch_data	*data = container_of(work, struct tcc_gpio_switch_data, work);
	switch_set_state(&data->sdev, data->state_val);
}

static ssize_t switch_tcc_gpio_print_state(struct switch_dev *sdev, char *buf)
{
	struct tcc_gpio_switch_data	*switch_data = container_of(sdev, struct tcc_gpio_switch_data, sdev);
	return sprintf(buf, "%d\n", switch_data->state_val);
}

static int tcc_gpio_thread(void* _switch_data)
{
	struct tcc_gpio_switch_data *switch_data = (struct tcc_gpio_switch_data *)_switch_data;
	unsigned int state = 0;
	unsigned int state_old = 0;

	tcc_dbg("%s() in...\n\n", __func__);


	/* GPIO setting for HeadPhone dectection */
	tcc_gpio_config(tcc_hpdet_port, GPIO_FN(0));
	gpio_request(tcc_hpdet_port, "HP_DETECTION");
	gpio_direction_input(tcc_hpdet_port);
	while(!kthread_should_stop()) {
		/* polling time is 100ms */
		msleep_interruptible(100);
		state = gpio_get_value(tcc_hpdet_port);

		if(!state && state_old) {
			gJack_disconnect_count++;
			if(gJack_disconnect_count < 7)	state = state_old;
		}
		else
			gJack_disconnect_count = 0;

		if(state != state_old) {
			state_old = state;
			if (state)	switch_data->state_val = TCC_BIT_HEADSET_NO_MIC;
			else		switch_data->state_val = 0;
			schedule_work(&switch_data->work);
		}
	}
	tcc_dbg("%s() out...\n\n", __func__);
	return 0;
}

static int tcc_gpio_switch_probe(struct platform_device *pdev)
{
	struct tcc_gpio_switch_data *switch_data;
	int ret = 0;

	tcc_dbg("gpio_switch_probe()...in \n\n");

	switch_data = kzalloc(sizeof(struct tcc_gpio_switch_data), GFP_KERNEL);
	if (!switch_data)
		return -ENOMEM;

	switch_data->sdev.name = "h2w";
	switch_data->state_val = 0;
	switch_data->name_on = "h2w_name_on";
	switch_data->name_off = "h2w_name_off";
	switch_data->state_on = "2";
	switch_data->state_off = "1";
	switch_data->sdev.print_state = switch_tcc_gpio_print_state;

	ret = switch_dev_register(&switch_data->sdev);
	if (ret < 0)
		goto err_switch_dev_register;

	INIT_WORK(&switch_data->work, tcc_gpio_switch_work);

//	switch_data->irq = INT_EI3;	 // External Interrupt3
	switch_data->poll_task = (struct task_struct *)(-EINVAL);
	switch_data->poll_task = kthread_create(tcc_gpio_thread, switch_data, "tcc-gpio-poll-thread");
	if (IS_ERR(switch_data->poll_task)) {
		printk("\ntcc-gpio-poll-thread create error: %p\n", switch_data->poll_task);
		kfree(switch_data);
		return (-EINVAL);
	}
	wake_up_process(switch_data->poll_task);

	/* Perform initial detection */
	tcc_gpio_switch_work(&switch_data->work);

	tcc_dbg("gpio_switch_probe()...out \n\n");
	return 0;

err_switch_dev_register:
	kfree(switch_data);

	return ret;
}

static int tcc_gpio_switch_remove(struct platform_device *pdev)
{
	struct tcc_gpio_switch_data *switch_data = platform_get_drvdata(pdev);

	cancel_work_sync(&switch_data->work);
	gpio_free(switch_data->gpio);
	switch_dev_unregister(&switch_data->sdev);
	kfree(switch_data);

	return 0;
}

static struct platform_driver tcc_gpio_switch_driver = {
	.probe		= tcc_gpio_switch_probe,
	.remove		= tcc_gpio_switch_remove,
	.driver		= {
		.name	= "switch-gpio-earjack-detect",
		.owner	= THIS_MODULE,
	},
};

static int tcc_gpio_switch_set_ports(void)
{
	tcc_hpdet_port = 0xFFFFFFFF;

#if defined(CONFIG_ARCH_TCC92XX)
	if(machine_is_m57te() || machine_is_m801())
		tcc_hpdet_port = TCC_GPA(12);
#elif defined(CONFIG_ARCH_TCC88XX)
	if(machine_is_m801_88()|| machine_is_m803())
		tcc_hpdet_port = TCC_GPD(10);
#elif defined(CONFIG_ARCH_TCC892X)
	if(machine_is_m805_892x()) {
		if (system_rev == 0x2000 || system_rev == 0x2001)
			tcc_hpdet_port = TCC_GPE(5);
		else	// 0x2002, 0x2003, 0x2004, 0x2005, 0x2006, 0x2007, 0x2008
			tcc_hpdet_port = TCC_GPE(16);
	}
#elif defined(CONFIG_MACH_M805_893X)
	if(machine_is_m805_893x()) {
		#if defined(CONFIG_CHIP_TCC8935S) || defined(CONFIG_CHIP_TCC8933S) || defined(CONFIG_CHIP_TCC8937S)
		tcc_hpdet_port = TCC_GPE(16);
		#else
		tcc_hpdet_port = TCC_GPE(12);	// tcc892x : gpio_e(16)
		#endif
	}
#endif

	if (tcc_hpdet_port == 0xFFFFFFFF)
		return -1;

	return 0;
}

static int __init tcc_gpio_switch_init(void)
{
#if defined(CONFIG_ARCH_TCC92XX)
	if( !(machine_is_m57te() || machine_is_m801()))
		return 0;
#elif defined(CONFIG_ARCH_TCC88XX)
	if( !(machine_is_m801_88() || machine_is_m803()))
		return 0;
#elif defined(CONFIG_ARCH_TCC892X)
	if( !(machine_is_m805_892x() || machine_is_m805_893x()))
		return 0;
#endif

	if (tcc_gpio_switch_set_ports())
		return 0;

	tcc_dbg("\n%s()...\n\n", __func__);
	return platform_driver_register(&tcc_gpio_switch_driver);
}

static void __exit tcc_gpio_switch_exit(void)
{
	platform_driver_unregister(&tcc_gpio_switch_driver);
}

module_init(tcc_gpio_switch_init);
module_exit(tcc_gpio_switch_exit);

MODULE_AUTHOR("Mike Lockwood <lockwood@android.com>");
MODULE_DESCRIPTION("GPIO Switch driver");
MODULE_LICENSE("GPL");
