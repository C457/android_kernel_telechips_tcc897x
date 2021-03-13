/*
 *  drivers/switch/switch_gpio_composite.c
 *
 * Copyright (C) 2008 Google, Inc.
 * Author: Mike Lockwood <lockwood@android.com>
 *         Android ce team of telechips.
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
#include <linux/workqueue.h>
#include <linux/gpio.h>

#include <asm/io.h>
#include <mach/bsp.h>
#include <mach/irqs.h>
#include <asm/mach-types.h>

#include <linux/kthread.h>  /* thread */
#include <linux/delay.h>    /* msleep_interruptible */
#include <mach/tcc_board_composite.h>


/*---------------------------------------------------------------------------
 * Debug Option
 *---------------------------------------------------------------------------*/
#if defined(tcc_dbg)
#undef tcc_dbg(fmt,arg...)  
#endif

#if 1
#define tcc_dbg(fmt,arg...)  
#else
#define tcc_dbg(fmt,arg...)      printk(fmt,##arg)
#endif
/*---------------------------------------------------------------------------*/


void composite_send_hpd_evnet(void  *pswitch_data, unsigned int composite_state)
{
	struct composite_gpio_switch_data *switch_data =(struct composite_gpio_switch_data *)pswitch_data;
	
	tcc_dbg("%s composite state set:%d  before: %d  \n", __func__, composite_state, switch_data->state_val);
	
	if(switch_data->state_val != composite_state)
	{
		if (composite_state)  
			switch_data->state_val = TCC_COMPOSITE_ON;
		else        
			switch_data->state_val = TCC_COMPOSITE_OFF;

		schedule_work(&switch_data->work);
	}
}


static void composite_gpio_switch_work(struct work_struct *work)
{
	struct composite_gpio_switch_data	*data =
		container_of(work, struct composite_gpio_switch_data, work);
	
	tcc_dbg("%s %d \n", __func__, data->state_val);
	switch_set_state(&data->sdev, data->state_val);
}


static ssize_t switch_composite_gpio_print_state(struct switch_dev *sdev, char *buf)
{
	struct composite_gpio_switch_data	*switch_data =
		container_of(sdev, struct composite_gpio_switch_data, sdev);

    return sprintf(buf, "%d\n", switch_data->state_val);
}



static int composite_gpio_switch_probe(struct platform_device *pdev)
{
	struct composite_gpio_switch_data *switch_data;
	int ret = 0;

    tcc_dbg("composite_gpio_switch_probe()...in \n\n");

	switch_data = kzalloc(sizeof(struct composite_gpio_switch_data), GFP_KERNEL);
	if (!switch_data)
		return -ENOMEM;

	switch_data->sdev.name = "composite";
	switch_data->state_val = 0;
	switch_data->name_on = "composite_name_on";
	switch_data->name_off = "composite_name_off";
	switch_data->state_on = "2";
	switch_data->state_off = "1";
	switch_data->sdev.print_state = switch_composite_gpio_print_state;
	switch_data->send_composite_event = composite_send_hpd_evnet;
	pdev->dev.platform_data = switch_data;
    ret = switch_dev_register(&switch_data->sdev);
	if (ret < 0)
		goto err_switch_dev_register;

	INIT_WORK(&switch_data->work, composite_gpio_switch_work);

	/* Perform initial detection */
	composite_gpio_switch_work(&switch_data->work);

	tcc_dbg("composite_gpio_switch_probe()...out \n\n");

	return 0;

err_switch_dev_register:
	printk("Error composite_gpio_switch_probe\n");

	kfree(switch_data);

	return ret;
}

static int composite_gpio_switch_remove(struct platform_device *pdev)
{
	struct composite_gpio_switch_data *switch_data = platform_get_drvdata(pdev);

	tcc_dbg("%s %d \n", __func__, switch_data->state_val);

	cancel_work_sync(&switch_data->work);
	switch_dev_unregister(&switch_data->sdev);
	kfree(switch_data);

	return 0;
}

static struct platform_driver composite_gpio_switch_driver = {
	.probe		= composite_gpio_switch_probe,
	.remove		= composite_gpio_switch_remove,
	.driver		= {
		.name	= "switch-gpio-composite-detect",
		.owner	= THIS_MODULE,
	},
};

static int __init composite_gpio_switch_init(void)
{

    tcc_dbg("\n%s()...\n\n", __func__);
	return platform_driver_register(&composite_gpio_switch_driver);
}

static void __exit composite_gpio_switch_exit(void)
{
	platform_driver_unregister(&composite_gpio_switch_driver);
}

module_init(composite_gpio_switch_init);
module_exit(composite_gpio_switch_exit);

MODULE_AUTHOR("Android ce team <android_ce@telechips.com>");
MODULE_DESCRIPTION("GPIO Switch driver for COMPOSITE");
MODULE_LICENSE("GPL");


