/*
 * hdmi_1920x1080.c -- support for Hdmi 1920x1080 LCD
 *
 * Copyright (C) 2009, 2010 Telechips, Inc.
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/mutex.h>
#include <asm/mach-types.h>
#include <linux/module.h>

#ifdef CONFIG_ARCH_TCC897X
#include <mach/tcc_fb.h>
#include <mach/gpio.h>
#include <mach/tca_lcdc.h>
#include <mach/TCC_LCD_Interface.h>
#else
#include <video/tcc/gpio.h>
#include <video/tcc/tcc_fb.h>
#include <video/tcc/tca_lcdc.h>
#include <video/tcc/TCC_LCD_Interface.h>
#endif

static struct mutex panel_lock;

static int hdmi1920x1080_panel_init(struct lcd_panel *panel, struct tcc_dp_device *fb_pdata)
{
	return 0;
}

static int hdmi1920x1080_set_power(struct lcd_panel *panel, int on, struct tcc_dp_device *fb_pdata)
{
	return 0;
}

static struct lcd_panel hdmi1920x1080_panel = {
	.name		= "HDMI1920x1080",
	.manufacturer	= "Telechips",
	.id		= PANEL_ID_HDMI,
	.xres		= 1920,
	.yres		= 1080,
	.width		= 103,
	.height		= 62,
	.bpp		= 32,
	.clk_freq	= 24500000,
	.clk_div	= 2,
	.bus_width	= 24,
	.lpw		= 2,
	.lpc		= 1920,
	.lswc		= 12,
	.lewc		= 7,
	.vdb		= 0,
	.vdf		= 0,
	.fpw1		= 0,
	.flc1		= 1080,
	.fswc1		= 6,
	.fewc1		= 4,
	.fpw2		= 0,
	.flc2		= 1080,
	.fswc2		= 6,
	.fewc2		= 4,
	.sync_invert	= IV_INVERT | IH_INVERT,
	.init		= hdmi1920x1080_panel_init,
	.set_power	= hdmi1920x1080_set_power,
};

static int hdmi1920x1080_probe(struct platform_device *pdev)
{
	printk("%s\n", __func__);
	
	mutex_init(&panel_lock);

	hdmi1920x1080_panel.dev = &pdev->dev;

	tccfb_register_panel(&hdmi1920x1080_panel);

	return 0;
}

static int hdmi1920x1080_remove(struct platform_device *pdev)
{
	return 0;
}

#ifdef CONFIG_OF
static struct of_device_id hdmi1920x1080_of_match[] = {
	{ .compatible = "telechips,hdmi1920x1080" },
	{}
};
MODULE_DEVICE_TABLE(of, hdmi1920x1080_of_match);
#endif

static struct platform_driver hdmi1920x1080_driver = {
	.probe	= hdmi1920x1080_probe,
	.remove	= hdmi1920x1080_remove,
	.driver	= {
		.name	= "hdmi1920x1080_lcd",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(hdmi1920x1080_of_match),
	},
};

static __init int hdmi1920x1080_init(void)
{
	return platform_driver_register(&hdmi1920x1080_driver);
}

static __exit void hdmi1920x1080_exit(void)
{
	platform_driver_unregister(&hdmi1920x1080_driver);
}

subsys_initcall(hdmi1920x1080_init);
module_exit(hdmi1920x1080_exit);

MODULE_DESCRIPTION("HDMI 1920x1080 LCD driver");
MODULE_LICENSE("GPL");
