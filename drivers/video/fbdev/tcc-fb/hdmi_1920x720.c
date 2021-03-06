/*
 * hdmi_1920x720.c -- support for Hdmi 1920x720 LCD
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
#include <mach/gpio.h>
#include <mach/tccfb.h>
#include <mach/tcc_fb.h>
#include <mach/tca_lcdc.h>
#include <mach/TCC_LCD_Interface.h>
#else
#include <video/tcc/tcc_fb.h>
#include <video/tcc/tca_lcdc.h>
#include <video/tcc/TCC_LCD_Interface.h>
#endif

#include <linux/clk.h>
static struct mutex panel_lock;
static struct clk *hdmi_link_clk = NULL;
static struct clk *hdmi_peri_clk = NULL;
static struct clk *hdmi_ddi_clk = NULL;
static struct clk  *hdmi_isoip_clk = NULL;
static struct clk  *hdmi_lcdc0_clk = NULL;
static struct clk  *hdmi_lcdc1_clk = NULL;

static int hdmi1920x720_panel_init(struct lcd_panel *panel, struct tcc_dp_device *fb_pdata)
{
	if(fb_pdata->DispNum)
		clk_prepare_enable(hdmi_lcdc1_clk);
	else
		clk_prepare_enable(hdmi_lcdc0_clk);
	
	clk_prepare_enable(hdmi_peri_clk);
	clk_prepare_enable(hdmi_ddi_clk);
	clk_prepare_enable(hdmi_isoip_clk);
	clk_prepare_enable(hdmi_link_clk);

        #if defined(CONFIG_TCC_DISPLAY_HDMI_LVDS)
        pr_info("%s lcdc:%d DispOrder:%d \n", __func__, fb_pdata->ddc_info.blk_num, fb_pdata->DispOrder);
        fb_pdata->FbPowerState = true;
        fb_pdata->FbUpdateType = FB_RDMA_UPDATE;
        fb_pdata->DispDeviceType = TCC_OUTPUT_HDMI;
        #endif
	return 0;
}

static int hdmi1920x720_set_power(struct lcd_panel *panel, int on, struct tcc_dp_device *fb_pdata)
{
	return 0;
}

static struct lcd_panel hdmi1920x720_panel = {
	.name		= "HDMI1920x720",
	.manufacturer	= "Telechips",
	.id		= PANEL_ID_HDMI,
	.xres		= 1920,
	.yres		= 720,
	.width		= 177,//103, //177
	.height		= 100,//62, //100
	.bpp		= 32,
#ifdef CONFIG_DAUDIO	
	.clk_freq = 95000000,
#else	
	.clk_freq	= 97340000,
#endif	
	.clk_div	= 2,
	.bus_width	= 24,
	.lpw		= 20,
	.lpc		= 1920,
	.lswc		= 70,
	.lewc		= 70,
	.vdb		= 0,
	.vdf		= 0,
	.fpw1		= 4,
	.flc1		= 720,
	.fswc1		= 18,
	.fewc1		= 18,
	.fpw2		= 4,
	.flc2		= 720,
	.fswc2		= 18,
	.fewc2		= 18,
	.sync_invert	= IV_INVERT | IH_INVERT,
	.init		= hdmi1920x720_panel_init,
	.set_power	= hdmi1920x720_set_power,
};

static int hdmi1920x720_probe(struct platform_device *pdev)
{
	printk("%s\n", __func__);
	
	hdmi_lcdc0_clk = of_clk_get_by_name(pdev->dev.of_node, "lcdc0-clk");
	hdmi_lcdc1_clk = of_clk_get_by_name(pdev->dev.of_node, "lcdc1-clk");
	hdmi_peri_clk = of_clk_get_by_name(pdev->dev.of_node, "hdmi-pclk");
	hdmi_ddi_clk = of_clk_get_by_name(pdev->dev.of_node, "hdmi-hclk");
	hdmi_isoip_clk = of_clk_get_by_name(pdev->dev.of_node, "hdmi-phy");
	hdmi_link_clk = of_clk_get_by_name(pdev->dev.of_node, "hdmi-lclk");
	
	mutex_init(&panel_lock);

	hdmi1920x720_panel.dev = &pdev->dev;

	tccfb_register_panel(&hdmi1920x720_panel);

	return 0;
}

static int hdmi1920x720_remove(struct platform_device *pdev)
{
	return 0;
}

#ifdef CONFIG_OF
static struct of_device_id hdmi1920x720_of_match[] = {
	{ .compatible = "telechips,hdmi1920x720" },
	{}
};
MODULE_DEVICE_TABLE(of, hdmi1920x720_of_match);
#endif

static struct platform_driver hdmi1920x720_driver = {
	.probe	= hdmi1920x720_probe,
	.remove	= hdmi1920x720_remove,
	.driver	= {
		.name	= "hdmi1920x720_lcd",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(hdmi1920x720_of_match),
	},
};

static __init int hdmi1920x720_init(void)
{
	return platform_driver_register(&hdmi1920x720_driver);
}

static __exit void hdmi1920x720_exit(void)
{
	platform_driver_unregister(&hdmi1920x720_driver);
}

subsys_initcall(hdmi1920x720_init);
module_exit(hdmi1920x720_exit);

MODULE_DESCRIPTION("HDMI 1920x720 LCD driver");
MODULE_LICENSE("GPL");
