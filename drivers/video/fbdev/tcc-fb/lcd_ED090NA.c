/*
 * lcd_ED090NA.c -- support for ED090NA LVDS Panel
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
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/module.h>

#include <asm/mach-types.h>
#include <asm/io.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#endif
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_dma.h>

#ifdef CONFIG_ARCH_TCC897X
#include <mach/bsp.h>
#include <mach/hardware.h>
#include <mach/tccfb.h>
#include <mach/tcc_fb.h>
#include <mach/gpio.h>
#include <mach/tca_lcdc.h>
#include <mach/TCC_LCD_Interface.h>
#include <mach/vioc_outcfg.h>
#include <mach/vioc_disp.h>
#else
#include <video/tcc/gpio.h>
#include <video/tcc/tccfb.h>
#include <video/tcc/tcc_fb.h>
#include <video/tcc/tca_lcdc.h>
#include <video/tcc/TCC_LCD_Interface.h>
#include <video/tcc/vioc_outcfg.h>
#include <video/tcc/vioc_disp.h>
#endif

static struct mutex panel_lock;
static char lcd_pwr_state;
static struct clk *lvds_clk;

unsigned int ed090na_lvds_stby;
unsigned int ed090na_lvds_power;

static struct lcd_gpio_data lvds_ed090na;

#define     LVDS_VCO_45MHz        45000000
#define     LVDS_VCO_60MHz        60000000

static int ed090na_panel_init(struct lcd_panel *panel, struct tcc_dp_device *fb_pdata)
{
	pr_info("%s lcdc:%d DispOrder:%d \n", __func__, fb_pdata->ddc_info.blk_num, fb_pdata->DispOrder);

	fb_pdata->FbPowerState = true;
	fb_pdata->FbUpdateType = FB_RDMA_UPDATE;

	return 0;
}

static int ed090na_set_power(struct lcd_panel *panel, int on, struct tcc_dp_device *fb_pdata)
{
	PDDICONFIG	pDDICfg = VIOC_DDICONFIG_GetAddress();
	unsigned int P, M, S, VSEL, TC;

	printk("%s : %d\n", __func__, on);
	if(!pDDICfg)
		return -ENODEV;

	mutex_lock(&panel_lock);

	 fb_pdata->FbPowerState = panel->state = on;

	if (on) {
		gpio_set_value_cansleep(ed090na_lvds_power, 1);
		gpio_set_value_cansleep(lvds_ed090na.power_on, 1);
		udelay(20);
		gpio_set_value_cansleep(lvds_ed090na.reset, 1);
		gpio_set_value_cansleep(ed090na_lvds_stby, 1);
		gpio_set_value_cansleep(lvds_ed090na.display_on, 1);
		mdelay(20);
		
		// LVDS power on
		clk_prepare_enable(lvds_clk);

		lcdc_initialize(panel, fb_pdata);

		// LVDS reset	
		pDDICfg->LVDS_CTRL.bREG.RST =1;
		pDDICfg->LVDS_CTRL.bREG.RST =0;		

		BITCSET(pDDICfg->LVDS_TXO_SEL0.nREG, 0xFFFFFFFF, 0x15141312);
		BITCSET(pDDICfg->LVDS_TXO_SEL1.nREG, 0xFFFFFFFF, 0x0B0A1716);
		BITCSET(pDDICfg->LVDS_TXO_SEL2.nREG, 0xFFFFFFFF, 0x0F0E0D0C);
		BITCSET(pDDICfg->LVDS_TXO_SEL3.nREG, 0xFFFFFFFF, 0x05040302);
		BITCSET(pDDICfg->LVDS_TXO_SEL4.nREG, 0xFFFFFFFF, 0x1A190706);
		BITCSET(pDDICfg->LVDS_TXO_SEL5.nREG, 0xFFFFFFFF, 0x1F1E1F18);
		BITCSET(pDDICfg->LVDS_TXO_SEL6.nREG, 0xFFFFFFFF, 0x1F1E1F1E);
		BITCSET(pDDICfg->LVDS_TXO_SEL7.nREG, 0xFFFFFFFF, 0x1F1E1F1E);
		BITCSET(pDDICfg->LVDS_TXO_SEL8.nREG, 0xFFFFFFFF, 0x001E1F1E);

		if (panel->clk_freq >= LVDS_VCO_45MHz && panel->clk_freq < LVDS_VCO_60MHz) {
			#if defined(CONFIG_ARCH_TCC897X)
			M = 7; P = 7; S = 0; VSEL = 0; TC = 4;
			#elif defined(CONFIG_ARCH_TCC896X)
			//M = 10; P = 10; S = 1; VSEL = 0; TC = 4;
			M = 10; P = 10; S = 2; VSEL = 0; TC = 4;
			#endif
		} else {
			#if defined(CONFIG_ARCH_TCC897X)
			M = 10; P = 10; S = 0; VSEL = 0; TC = 4;
			#elif defined(CONFIG_ARCH_TCC896X)
			//M = 14; P = 14; S = 1; VSEL = 1; TC = 4;
			M = 10; P = 10; S = 1; VSEL = 0; TC = 4;
			#endif
		}

		BITCSET(pDDICfg->LVDS_CTRL.nREG, 0x00FFFFF0, (VSEL<<4)|(S<<5)|(M<<8)|(P<<15)|(TC<<21)); //LCDC1

		pDDICfg->LVDS_MISC1.bREG.LC = 0;
		pDDICfg->LVDS_MISC1.bREG.CC = 0;
		pDDICfg->LVDS_MISC1.bREG.CMS = 0;
		pDDICfg->LVDS_MISC1.bREG.VOC = 1;


		// LVDS Select LCDC 1
		if(fb_pdata->ddc_info.blk_num)
			BITCSET(pDDICfg->LVDS_CTRL.nREG, 0x3 << 30 , 0x1 << 30);
		else
			BITCSET(pDDICfg->LVDS_CTRL.nREG, 0x3 << 30 , 0x0 << 30);
		
	    	pDDICfg->LVDS_CTRL.bREG.RST = 1;	//  reset
	  	pDDICfg->LVDS_CTRL.bREG.EN = 1;   // enable

		msleep(100);

	}
	else 
	{

		pDDICfg->LVDS_CTRL.bREG.RST = 1;		// reset
		pDDICfg->LVDS_CTRL.bREG.EN = 0;		// reset


		clk_disable_unprepare(lvds_clk);
		gpio_set_value_cansleep(lvds_ed090na.display_on, 0);
		gpio_set_value_cansleep(lvds_ed090na.reset, 0);
		gpio_set_value_cansleep(ed090na_lvds_stby, 0);
		gpio_set_value_cansleep(lvds_ed090na.power_on, 0);
		gpio_set_value_cansleep(ed090na_lvds_power, 0);

	}
	mutex_unlock(&panel_lock);

	return 0;
}

struct lcd_panel ed090na_panel = {
	.name		= "ED090NA",
	.manufacturer	= "innolux",
	.id		= PANEL_ID_ED090NA,
	.xres		= 1280,
	.yres		= 800,
	.width		= 235,
	.height		= 163,
	.bpp		= 24,
	.clk_freq	= 68000000,
	.clk_div	= 2,
	.bus_width	= 24,
	
	.lpw		= 10,
	.lpc		= 1280,
	.lswc	= 74,
	.lewc	= 74,
	.vdb		= 0,
	.vdf		= 0,

	.fpw1		= 2,
	.flc1		= 800,
	.fswc1		= 10,
	.fewc1		= 25,
	
	.fpw2		= 2,
	.flc2		= 800,
	.fswc2		= 10,
	.fewc2		= 25,
	.sync_invert	= IV_INVERT | IH_INVERT,
	.init		= ed090na_panel_init,
	.set_power	= ed090na_set_power,
};

static void ed090na_parse_dt(struct device_node *np)
{
	if(np){
		lvds_ed090na.power_on= of_get_named_gpio(np, "power-on-gpios", 0);

		if(!gpio_is_valid(lvds_ed090na.power_on))
		{
			printk("%s: err to get power_on gpios: ret:%x\n", __func__, lvds_ed090na.power_on);
			lvds_ed090na.power_on = -1;
		} else {
			gpio_request(lvds_ed090na.power_on, "lcd_on");
			gpio_direction_output(lvds_ed090na.power_on, 1);
		}

		lvds_ed090na.display_on= of_get_named_gpio(np, "display-on-gpios", 0);

		if(!gpio_is_valid(lvds_ed090na.display_on))
		{
			printk("%s: err to get (lvds_fld0800.display_on) gpios: ret:%x\n", __func__, lvds_ed090na.display_on);
			lvds_ed090na.display_on = -1;
		} else {
			gpio_request(lvds_ed090na.display_on, "lvds_display");
			gpio_direction_output(lvds_ed090na.display_on, 1);
		}

		lvds_ed090na.reset= of_get_named_gpio(np, "reset-gpios", 0);

		if(!gpio_is_valid(lvds_ed090na.reset))
		{
			printk("%s: err to get reset gpios: ret:%x\n", __func__, lvds_ed090na.reset);
			lvds_ed090na.reset = -1;
		} else {
			gpio_request(lvds_ed090na.reset, "lcd_reset");
			gpio_direction_output(lvds_ed090na.reset, 1);
		}

		ed090na_lvds_stby = of_get_named_gpio(np, "lvds-stby-gpios", 0);

		if(!gpio_is_valid(ed090na_lvds_stby))
		{
			printk("%s: err to get lvds_stby gpios: ret:%x\n", __func__, ed090na_lvds_stby);
			ed090na_lvds_stby = -1;
		} else {
			gpio_request(ed090na_lvds_stby, "lcd_stbyb");
			gpio_direction_output(ed090na_lvds_stby, 1);
		}

		ed090na_lvds_power= of_get_named_gpio(np, "lvds-power-gpios", 0);

		if(!gpio_is_valid(ed090na_lvds_power))
		{
			printk("%s: err to get lvds_power gpios: ret:%x\n", __func__, ed090na_lvds_power);
			ed090na_lvds_power = -1;
		} else {
			gpio_request(ed090na_lvds_power, "lvds_power");
			gpio_direction_output(ed090na_lvds_power, 1);
		}

	}
}

static int ed090na_probe(struct platform_device *pdev)
{
	struct device_node *np;

	printk("%s : %s\n", __func__,  pdev->name);
	printk(" ###### %s ###### \n",__func__);

	mutex_init(&panel_lock);

	ed090na_parse_dt(pdev->dev.of_node);

	ed090na_panel.state = 1;
	ed090na_panel.dev = &pdev->dev;

       np = of_parse_phandle(pdev->dev.of_node, "lvds0", 0);
       lvds_clk = of_clk_get(np, 0);

       if(IS_ERR(lvds_clk)){
               printk(" ED090NA : failed to get lvds clock \n");
               lvds_clk = NULL;
               return -ENODEV;
       }

       if(lvds_clk)
               clk_prepare_enable(lvds_clk);

	tccfb_register_panel(&ed090na_panel);
	return 0;
}

static int ed090na_remove(struct platform_device *pdev)
{
	return 0;
}

#ifdef CONFIG_OF
static struct of_device_id ed090na_of_match[] = {
       { .compatible = "telechips,lvds-ed090na" },
       {}
};
MODULE_DEVICE_TABLE(of, ed090na_of_match);
#endif

static struct platform_driver ed090na_lcd = {
	.probe	= ed090na_probe,
	.remove	= ed090na_remove,
	.driver	= {
		.name	= "ed090na_lcd",
		.owner	= THIS_MODULE,
#ifdef CONFIG_OF
               .of_match_table = of_match_ptr(ed090na_of_match),
#endif
	},
};

static __init int ed090na_init(void)
{
	printk("~ %s ~ \n", __func__);
	return platform_driver_register(&ed090na_lcd);
}
static __exit void ed090na_exit(void)
{
	platform_driver_unregister(&ed090na_lcd);
}

module_init(ed090na_init);
module_exit(ed090na_exit);

MODULE_DESCRIPTION("ed090na LCD driver");
MODULE_LICENSE("GPL");
