/*
 *  drivers/switch/switch_gpio_hdmi.c
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
#include <mach/tcc_board_hdmi.h>


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


void hdmi_send_hpd_evnet(void  *pswitch_data, unsigned int hpd_state)
{
	struct hdmi_gpio_switch_data *switch_data =(struct hdmi_gpio_switch_data *)pswitch_data;
	
	tcc_dbg("%s hpd state set:%d  before: %d  \n", __func__, hpd_state, switch_data->state_val);
	
	if(switch_data->state_val != hpd_state)
	{
		if (hpd_state)  
			switch_data->state_val = TCC_HDMI_ON;
		else        
			switch_data->state_val = TCC_HDMI_OFF;

		schedule_work(&switch_data->work);
	}
}


static void hdmi_gpio_switch_work(struct work_struct *work)
{
	struct hdmi_gpio_switch_data	*data =
		container_of(work, struct hdmi_gpio_switch_data, work);
	
	tcc_dbg("%s %d \n", __func__, data->state_val);
	switch_set_state(&data->sdev, data->state_val);
}


static ssize_t switch_hdmi_gpio_print_state(struct switch_dev *sdev, char *buf)
{
	struct hdmi_gpio_switch_data	*switch_data =
		container_of(sdev, struct hdmi_gpio_switch_data, sdev);

    return sprintf(buf, "%d\n", switch_data->state_val);
}



static int hdmi_gpio_switch_probe(struct platform_device *pdev)
{
	struct hdmi_gpio_switch_data *switch_data;
	int ret = 0;

    tcc_dbg("hdmi_gpio_switch_probe()...in \n\n");

	switch_data = kzalloc(sizeof(struct hdmi_gpio_switch_data), GFP_KERNEL);
	if (!switch_data)
		return -ENOMEM;

	switch_data->sdev.name = "hdmi";
	switch_data->state_val = 0;
	switch_data->name_on = "hdmi_name_on";
	switch_data->name_off = "hdmi_name_off";
	switch_data->state_on = "2";
	switch_data->state_off = "1";
	switch_data->sdev.print_state = switch_hdmi_gpio_print_state;
	switch_data->send_hdp_event = hdmi_send_hpd_evnet;
	pdev->dev.platform_data = switch_data;
    ret = switch_dev_register(&switch_data->sdev);
	if (ret < 0)
		goto err_switch_dev_register;

	INIT_WORK(&switch_data->work, hdmi_gpio_switch_work);

	/* Perform initial detection */
	hdmi_gpio_switch_work(&switch_data->work);

	tcc_dbg("hdmi_gpio_switch_probe()...out \n\n");

	return 0;

err_switch_dev_register:
	printk("Error hdmi_gpio_switch_probe\n");

	kfree(switch_data);

	return ret;
}

static int hdmi_gpio_switch_remove(struct platform_device *pdev)
{
	struct hdmi_gpio_switch_data *switch_data = platform_get_drvdata(pdev);

	tcc_dbg("%s %d \n", __func__, switch_data->state_val);

	cancel_work_sync(&switch_data->work);
	switch_dev_unregister(&switch_data->sdev);
	kfree(switch_data);

	return 0;
}

static struct platform_driver hdmi_gpio_switch_driver = {
	.probe		= hdmi_gpio_switch_probe,
	.remove		= hdmi_gpio_switch_remove,
	.driver		= {
		.name	= "switch-gpio-hdmi-detect",
		.owner	= THIS_MODULE,
	},
};

static int __init hdmi_gpio_switch_init(void)
{

    tcc_dbg("\n%s()...\n\n", __func__);
	return platform_driver_register(&hdmi_gpio_switch_driver);
}

static void __exit hdmi_gpio_switch_exit(void)
{
	platform_driver_unregister(&hdmi_gpio_switch_driver);
}

module_init(hdmi_gpio_switch_init);
module_exit(hdmi_gpio_switch_exit);

MODULE_AUTHOR("Android ce team <android_ce@telechips.com>");
MODULE_DESCRIPTION("GPIO Switch driver for HDMI");
MODULE_LICENSE("GPL");


