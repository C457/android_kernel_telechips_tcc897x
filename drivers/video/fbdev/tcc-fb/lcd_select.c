/*
 * lcd_FLD0800.c -- support for FLD0800 LVDS Panel
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

#include <mach/daudio_info.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/mutex.h>
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

#include <mach/daudio.h>

static struct mutex panel_lock;
static struct clk *hdmi_link_clk = NULL;
static struct clk *hdmi_peri_clk = NULL;
static struct clk *hdmi_ddi_clk = NULL;
static struct clk  *hdmi_isoip_clk = NULL;
static struct clk  *hdmi_lcdc0_clk = NULL;
static struct clk  *hdmi_lcdc1_clk = NULL;

static char lcd_pwr_state;
static struct clk *lvds_clk;

unsigned int lvds_stby;
unsigned int lvds_power;

static struct lcd_gpio_data lvds_fld0800;

#define     LVDS_VCO_45MHz        45000000
#define     LVDS_VCO_60MHz        60000000

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

static int fld0800_panel_init(struct lcd_panel *panel, struct tcc_dp_device *fb_pdata)
{
	pr_info("%s lcdc:%d DispOrder:%d \n", __func__, fb_pdata->ddc_info.blk_num, fb_pdata->DispOrder);

	fb_pdata->FbPowerState = true;
	fb_pdata->FbUpdateType = FB_RDMA_UPDATE;

	return 0;
}

static int fld0800_set_power(struct lcd_panel *panel, int on, struct tcc_dp_device *fb_pdata)
{

	PDDICONFIG	pDDICfg = VIOC_DDICONFIG_GetAddress();
	unsigned int P, M, S, VSEL, TC;

	printk("%s : %d\n", __func__, on);
	if(!pDDICfg) 
		return -ENODEV;

	mutex_lock(&panel_lock);
	
	 fb_pdata->FbPowerState = panel->state = on;

	if (on) {
		if(gpio_is_valid(lvds_power))
			gpio_set_value_cansleep(lvds_power, 1);

		if(gpio_is_valid(lvds_fld0800.power_on))
			gpio_set_value_cansleep(lvds_fld0800.power_on, 1);

		udelay(20);

		if(gpio_is_valid(lvds_fld0800.reset))			
			gpio_set_value_cansleep(lvds_fld0800.reset, 1);

		if(gpio_is_valid(lvds_stby))
			gpio_set_value_cansleep(lvds_stby, 1);

		if(gpio_is_valid(lvds_fld0800.display_on))		
			gpio_set_value_cansleep(lvds_fld0800.display_on, 1);

		mdelay(20);

		// LVDS power on
	       if(lvds_clk)
			clk_prepare_enable(lvds_clk);	

		lcdc_initialize(panel, fb_pdata);

		//LVDS 6bit setting for internal dithering option !!!
		//tcc_lcdc_dithering_setting(fb_pdata);

		// LVDS reset	
		pDDICfg->LVDS_CTRL.bREG.RST =1;
		pDDICfg->LVDS_CTRL.bREG.RST =0;	
		
		BITCSET(pDDICfg->LVDS_TXO_SEL0.nREG, 0xFFFFFFFF, 0x13121110);
		BITCSET(pDDICfg->LVDS_TXO_SEL1.nREG, 0xFFFFFFFF, 0x09081514);
		BITCSET(pDDICfg->LVDS_TXO_SEL2.nREG, 0xFFFFFFFF, 0x0D0C0B0A);
		BITCSET(pDDICfg->LVDS_TXO_SEL3.nREG, 0xFFFFFFFF, 0x03020100);
		BITCSET(pDDICfg->LVDS_TXO_SEL4.nREG, 0xFFFFFFFF, 0x1A190504);
		BITCSET(pDDICfg->LVDS_TXO_SEL5.nREG, 0xFFFFFFFF, 0x0E171618);
		BITCSET(pDDICfg->LVDS_TXO_SEL6.nREG, 0xFFFFFFFF, 0x1F07060F);
		BITCSET(pDDICfg->LVDS_TXO_SEL7.nREG, 0xFFFFFFFF, 0x1F1E1F1E);
		BITCSET(pDDICfg->LVDS_TXO_SEL8.nREG, 0xFFFFFFFF, 0x001E1F1E);

		if( panel->clk_freq >= LVDS_VCO_45MHz && panel->clk_freq < LVDS_VCO_60MHz) {
			#if defined (VIOC_TCC8930)
				   M = 7; P = 7; S = 0; VSEL = 0; TC = 4;
			#else //defined (VIOC_TCC8960)
//				   M = 10; P = 10; S = 1; VSEL = 0; TC = 4;
				   M = 10; P = 10; S = 2; VSEL = 0; TC = 4;
			#endif//
		}
		else {
			#if defined (VIOC_TCC8930)
				M = 10; P = 10; S = 0; VSEL = 0; TC = 4;		
			#else //defined (VIOC_TCC8960)
//				M = 14; P = 14; S = 1; VSEL = 1; TC = 4;		
				M = 10; P = 10; S = 1; VSEL = 0; TC = 4;		
			#endif//		
		}
		
		BITCSET(pDDICfg->LVDS_CTRL.nREG, 0x00FFFFF0, (VSEL<<4)|(S<<5)|(M<<8)|(P<<15)|(TC<<21)); //LCDC1

//		#if defined (VIOC_TCC8960)
		pDDICfg->LVDS_MISC1.bREG.LC = 0;
		pDDICfg->LVDS_MISC1.bREG.CC = 0;
		pDDICfg->LVDS_MISC1.bREG.CMS = 0;
		pDDICfg->LVDS_MISC1.bREG.VOC = 1;
//		#endif//

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

	       if(lvds_clk)		
			clk_disable_unprepare(lvds_clk);	

		if(gpio_is_valid(lvds_fld0800.display_on))
			gpio_set_value_cansleep(lvds_fld0800.display_on, 0);

		if(gpio_is_valid(lvds_fld0800.reset))
			gpio_set_value_cansleep(lvds_fld0800.reset, 0);	//NC

		if(gpio_is_valid(lvds_stby))
			gpio_set_value_cansleep(lvds_stby, 0);

		if(gpio_is_valid(lvds_fld0800.power_on))
			gpio_set_value_cansleep(lvds_fld0800.power_on, 0);

		if(gpio_is_valid(lvds_power))
			gpio_set_value_cansleep(lvds_power, 0);
	}
	mutex_unlock(&panel_lock);
	return 0;
}



static struct lcd_panel Truly_lvds1280x720_8 = {
	.name		= "LVDS1280x720_8_0",
	.manufacturer	= "Truly",
	.id		= PANEL_ID_FLD0800,
	.xres		= 1280,
	.yres		= 720,
	.width		= 60,
	.height		= 72,
	.bpp		= 24,

	.clk_freq	= 63700000,
	.clk_div	= 2,
	.bus_width	= 24,
	
	.lpw		= 2,
	.lpc		= 1280,
	.lswc		= 12,
	.lewc		= 44,

	.vdb		= 0,
	.vdf		= 0,

	.fpw1		= 2,
	.flc1		= 720,
	.fswc1		= 1,
	.fewc1		= 67,
	
	.fpw2		= 2,
	.flc2		= 720,
	.fswc2		= 1,
	.fewc2		= 67,

	.sync_invert	= IV_INVERT | IH_INVERT,
	.init		= fld0800_panel_init,
	.set_power	= fld0800_set_power,
};

static struct lcd_panel BOE_hdmi1280x720_8 = {
        .name           = "HDMI1280x720_8_0",
        .manufacturer   = "BOE",
        .id             = PANEL_ID_HDMI,
        .xres           = 1280,
        .yres           = 720,
        .width          = 60,
        .height         = 72,
        .bpp            = 24,

        .clk_freq       = 64000000,
        .clk_div        = 2,
        .bus_width      = 24,

        .lpw            = 32,
        .lpc            = 1280,
        .lswc           = 80,
        .lewc           = 48,

        .vdb            = 0,
        .vdf            = 0,

        .fpw1           = 5,
        .flc1           = 720,
        .fswc1          = 13,
        .fewc1          = 3,

        .fpw2           = 5,
        .flc2           = 720,
        .fswc2          = 13,
        .fewc2          = 3,

        .sync_invert    = IV_INVERT | IH_INVERT,
        .init           = hdmi1920x720_panel_init,
        .set_power      = hdmi1920x720_set_power,
};

static struct lcd_panel BOE_lvds1280x720_8 = {
        .name           = "LVDS1280x720_8_0",
        .manufacturer   = "BOE",
        .id             = PANEL_ID_FLD0800,
        .xres           = 1280,
        .yres           = 720,
        .width          = 176,
        .height         = 99,
        .bpp            = 24,

        .clk_freq       = 64200000,
        .clk_div        = 2,
        .bus_width      = 24,

        .lpw            = 31,
        .lpc            = 1279,
        .lswc           = 79,
        .lewc           = 47,

        .vdb            = 0,
        .vdf            = 0,

        .fpw1           = 4,
        .flc1           = 719,
        .fswc1          = 12,
        .fewc1          = 4,

        .fpw2           = 4,
        .flc2           = 719,
        .fswc2          = 12,
        .fewc2          = 4,

	.sync_invert    = IV_INVERT | IH_INVERT,
        .init           = fld0800_panel_init,
        .set_power      = fld0800_set_power,
};

static struct lcd_panel LGD_hdmi1920x720_10_25 = {
	.name		= "HDMI1920x720_10_25",
	.manufacturer	= "LGDisplay",
	.id		= PANEL_ID_HDMI,
	.xres		= 1920,
	.yres		= 720,
	.width		= 177,
	.height		= 100,
	.bpp		= 32,
	.clk_freq 	= 89600000,
	.clk_div	= 2,
	.bus_width	= 24,
	.lpw		= 40,
	.lpc		= 1920,
	.lswc		= 44,
	.lewc		= 44,
	.vdb		= 0,
	.vdf		= 0,
	.fpw1		= 2,
	.flc1		= 720,
	.fswc1		= 3,
	.fewc1		= 4,
	.fpw2		= 2,
	.flc2		= 720,
	.fswc2		= 3,
	.fewc2		= 4,
	.sync_invert	= IV_INVERT | IH_INVERT,
	.init		= hdmi1920x720_panel_init,
	.set_power	= hdmi1920x720_set_power,
};

static struct lcd_panel LGD_hdmi1920x720_12_3 = {
        .name           = "HDMI1920x720_12_3",
        .manufacturer   = "LGDisplay",
        .id             = PANEL_ID_HDMI,
        .xres           = 1920,
        .yres           = 720,
        .width          = 177,
        .height         = 100,
        .bpp            = 32,
        .clk_freq       = 99700000,
        .clk_div        = 2,
        .bus_width      = 24,
        .lpw            = 32,
        .lpc            = 1920,
        .lswc           = 160,
        .lewc           = 50,
        .vdb            = 0,
        .vdf            = 0,
        .fpw1           = 5,
        .flc1           = 720,
        .fswc1          = 28,
        .fewc1          = 15,
        .fpw2           = 5,
        .flc2           = 720,
        .fswc2          = 28,
        .fewc2          = 15,
        .sync_invert    = IV_INVERT | IH_INVERT,
        .init           = hdmi1920x720_panel_init,
        .set_power      = hdmi1920x720_set_power,
};

static int hdmi_probe(struct platform_device *pdev)
{

	unsigned char lcd_ver = daudio_lcd_version();

	printk("%s\n", __func__);
	
	hdmi_lcdc0_clk = of_clk_get_by_name(pdev->dev.of_node, "lcdc0-clk");
	hdmi_lcdc1_clk = of_clk_get_by_name(pdev->dev.of_node, "lcdc1-clk");
	hdmi_peri_clk = of_clk_get_by_name(pdev->dev.of_node, "hdmi-pclk");
	hdmi_ddi_clk = of_clk_get_by_name(pdev->dev.of_node, "hdmi-hclk");
	hdmi_isoip_clk = of_clk_get_by_name(pdev->dev.of_node, "hdmi-phy");
	hdmi_link_clk = of_clk_get_by_name(pdev->dev.of_node, "hdmi-lclk");
	
	mutex_init(&panel_lock);

	if(gpio_get_value(TCC_GPB(24))) // OE
                switch(lcd_ver)
                {
                        case DAUDIOKK_LCD_OI_10_25_1920_720_INCELL_Si_LG: //0
			case DAUDIOKK_LCD_OI_10_25_1920_720_INCELL_Si_2_LG: //1
                        case DAUDIOKK_LCD_OD_10_25_1920_720_INCELL_Si_LG: //3
                        case DAUDIOKK_LCD_OI_10_25_1920_720_INCELL_LTPS_LG: //7
			case DAUDIOKK_LCD_OD_10_25_1920_720_INCELL_LTPS_LG: //5
                                LGD_hdmi1920x720_10_25.dev = &pdev->dev;
			        tccfb_register_panel(&LGD_hdmi1920x720_10_25);
                                break;
			case DAUDIOKK_LCD_OD_12_30_1920_720_INCELL_Si_LG: //4
                                LGD_hdmi1920x720_12_3.dev = &pdev->dev;
                                tccfb_register_panel(&LGD_hdmi1920x720_12_3);
                                break;
                        case DAUDIOKK_LCD_OD_08_00_1280_720_OGS_Si_BOE: //6 HDMI SERDES
                               	BOE_hdmi1280x720_8.dev = &pdev->dev;
                                tccfb_register_panel(&BOE_hdmi1280x720_8);
                                break;
                        case DAUDIOKK_LCD_OI_DISCONNECTED: //10
                                LGD_hdmi1920x720_10_25.dev = &pdev->dev;
                                tccfb_register_panel(&LGD_hdmi1920x720_10_25);
                                break;
			case DAUDIOKK_LCD_OI_08_00_1280_720_OGS_Si_BOE: //8 LVDS
                        default:
                                printk("ADC value is wrong number : %d\n",lcd_ver);
                                break;
                }
        else                            // PIO
                switch(lcd_ver)
                {
                        case DAUDIOKK_LCD_PI_10_25_1920_720_PIO_AUO: //5
                                LGD_hdmi1920x720_10_25.dev = &pdev->dev;
                                tccfb_register_panel(&LGD_hdmi1920x720_10_25);
                                break;
                        case DAUDIOKK_LCD_PI_08_00_800_400_PIO_TRULY: //6
                        case DAUDIOKK_LCD_PI_DISCONNECTED: //10 Temporary use DAUDIOKK_LCD_PI_08_00_800_400_PIO_TRULY
                        default:
                                printk("ADC value is wrong number : %d\n",lcd_ver);
                                break;
                }
	return 0;
}

static int hdmi_remove(struct platform_device *pdev)
{
	return 0;
}

static void lvds_parse_dt(struct device_node *np)
{
	if(np){
		lvds_fld0800.power_on= of_get_named_gpio(np, "power-on-gpios", 0);

		if(!gpio_is_valid(lvds_fld0800.power_on))
		{
			printk("%s: err to get power_on gpios: ret:%x\n", __func__, lvds_fld0800.power_on);
			lvds_fld0800.power_on = -1;
		} else {
			gpio_request(lvds_fld0800.power_on, "lcd_on");
			gpio_direction_output(lvds_fld0800.power_on, 1);
		}

		lvds_fld0800.display_on= of_get_named_gpio(np, "display-on-gpios", 0);

		if(!gpio_is_valid(lvds_fld0800.display_on))
		{
			printk("%s: err to get (lvds_fld0800.display_on) gpios: ret:%x\n", __func__, lvds_fld0800.display_on);
			lvds_fld0800.display_on = -1;
		} else {
			gpio_request(lvds_fld0800.display_on, "lvds_display");
			gpio_direction_output(lvds_fld0800.display_on, 1);
		}

		lvds_fld0800.reset= of_get_named_gpio(np, "reset-gpios", 0);

		if(!gpio_is_valid(lvds_fld0800.reset))
		{
			printk("%s: err to get reset gpios: ret:%x\n", __func__, lvds_fld0800.reset);
			lvds_fld0800.reset = -1;
		} else {
			gpio_request(lvds_fld0800.reset, "lcd_reset");
			gpio_direction_output(lvds_fld0800.reset, 1);
		}

		lvds_stby = of_get_named_gpio(np, "lvds-stby-gpios", 0);

		if(!gpio_is_valid(lvds_stby))
		{
			printk("%s: err to get lvds_stby gpios: ret:%x\n", __func__, lvds_stby);
			lvds_stby = -1;
		} else {
			gpio_request(lvds_stby, "lcd_stbyb");
			gpio_direction_output(lvds_stby, 1);
		}

		lvds_power= of_get_named_gpio(np, "lvds-power-gpios", 0);

		if(!gpio_is_valid(lvds_power))
		{
			printk("%s: err to get lvds_power gpios: ret:%x\n", __func__, lvds_power);
			lvds_power = -1;
		} else {
			gpio_request(lvds_power, "lvds_power");
			gpio_direction_output(lvds_power, 1);
		}

	}
}

static int lvds_probe(struct platform_device *pdev)
{
	struct device_node *np;

	unsigned char lcd_ver = daudio_lcd_version();
	
	printk("%s : %s\n", __func__,  pdev->name);
    	printk(" ###### %s ###### \n",__func__);
		
	mutex_init(&panel_lock);
	lcd_pwr_state = 1;

	lvds_parse_dt(pdev->dev.of_node);

	np = of_parse_phandle(pdev->dev.of_node, "lvds0", 0);
	lvds_clk = of_clk_get(np, 0);

	if(IS_ERR(lvds_clk)){
		printk(" FLD0800 : failed to get lvds clock \n");
		lvds_clk = NULL;
		return -ENODEV;
	}

	if(lvds_clk)
        	clk_prepare_enable(lvds_clk);
	
	if(gpio_get_value(TCC_GPB(24))) // OE
                switch(lcd_ver)
                {
			case DAUDIOKK_LCD_OI_08_00_1280_720_OGS_Si_BOE: //8 LVDS
				BOE_lvds1280x720_8.state = 1;
                                BOE_lvds1280x720_8.dev = &pdev->dev;
                                tccfb_register_panel(&BOE_lvds1280x720_8);
                                break;			
                        case DAUDIOKK_LCD_OI_10_25_1920_720_INCELL_Si_LG: //0
			case DAUDIOKK_LCD_OI_10_25_1920_720_INCELL_Si_2_LG: //1
                        case DAUDIOKK_LCD_OD_10_25_1920_720_INCELL_Si_LG: //3
                        case DAUDIOKK_LCD_OI_10_25_1920_720_INCELL_LTPS_LG: //7
			case DAUDIOKK_LCD_OD_10_25_1920_720_INCELL_LTPS_LG: //5
                        case DAUDIOKK_LCD_OD_12_30_1920_720_INCELL_Si_LG: //4
                        case DAUDIOKK_LCD_OD_08_00_1280_720_OGS_Si_BOE: //6 HDMI SERDES
                        case DAUDIOKK_LCD_OI_DISCONNECTED: //10
                        default:
                                printk("ADC value is wrong number : %d\n",lcd_ver);
                                break;
                }
        else                            // PIO
                switch(lcd_ver)
                {
			case DAUDIOKK_LCD_PI_08_00_800_400_PIO_TRULY: //6
				Truly_lvds1280x720_8.state = 1;
			        Truly_lvds1280x720_8.dev = &pdev->dev;
			        tccfb_register_panel(&Truly_lvds1280x720_8);
				break;
                        case DAUDIOKK_LCD_PI_DISCONNECTED: //10 Temporary use DAUDIOKK_LCD_PI_08_00_800_400_PIO_TRULY
				Truly_lvds1280x720_8.state = 1;
                                Truly_lvds1280x720_8.dev = &pdev->dev;
                                tccfb_register_panel(&Truly_lvds1280x720_8);
				break;
                        case DAUDIOKK_LCD_PI_10_25_1920_720_PIO_AUO: //5
                        default:
                                printk("ADC value is wrong number : %d\n",lcd_ver);
                                break;
                }
	return 0;
}

static int lvds_remove(struct platform_device *pdev)
{
	return 0;
}



static struct of_device_id hdmi_of_match[] = {
	{ .compatible = "telechips,hdmi1920x720" },
	{}
};
MODULE_DEVICE_TABLE(of, hdmi_of_match);


static struct platform_driver hdmi_driver = {
	.probe	= hdmi_probe,
	.remove	= hdmi_remove,
	.driver	= {
		.name	= "hdmi_lcd",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(hdmi_of_match),
	},
};



static struct of_device_id lvds_of_match[] = {
       { .compatible = "telechips,lvds-fld0800" },
       {}
};
MODULE_DEVICE_TABLE(of, lvds_of_match);


static struct platform_driver lvds_lcd = {
	.probe	= lvds_probe,
	.remove	= lvds_remove,
	.driver	= {
		.name	= "lvds_lcd",
		.owner	= THIS_MODULE,
               .of_match_table = of_match_ptr(lvds_of_match),
	},
};

static __init int lcd_select_init(void)
{
	printk("~ %s ~ \n", __func__);

	unsigned char lcd_ver = daudio_lcd_version();

	if(gpio_get_value(TCC_GPB(24))) // OE
                switch(lcd_ver)
                {
                        case DAUDIOKK_LCD_OI_10_25_1920_720_INCELL_Si_LG: //0
			case DAUDIOKK_LCD_OI_10_25_1920_720_INCELL_Si_2_LG: //1
                        case DAUDIOKK_LCD_OD_10_25_1920_720_INCELL_Si_LG: //3
                        case DAUDIOKK_LCD_OD_12_30_1920_720_INCELL_Si_LG: //4
                        case DAUDIOKK_LCD_OI_10_25_1920_720_INCELL_LTPS_LG: //7
			case DAUDIOKK_LCD_OD_10_25_1920_720_INCELL_LTPS_LG: //5
                                platform_driver_register(&hdmi_driver);
                                break;
                        case DAUDIOKK_LCD_OD_08_00_1280_720_OGS_Si_BOE: //6 HDMI SERDES
				platform_driver_register(&hdmi_driver);
                                break;
			case DAUDIOKK_LCD_OI_08_00_1280_720_OGS_Si_BOE: //8 LVDS
				platform_driver_register(&lvds_lcd);
                                break;
                        case DAUDIOKK_LCD_OI_DISCONNECTED: //10
                                platform_driver_register(&hdmi_driver);
                                break;
                        default:
                                printk("ADC value is wrong number : %d\n",lcd_ver);
                                break;
                }
        else                            // PIO
                switch(lcd_ver)
                {
                        case DAUDIOKK_LCD_PI_10_25_1920_720_PIO_AUO: //5
                                platform_driver_register(&hdmi_driver);
                                break;
                        case DAUDIOKK_LCD_PI_08_00_800_400_PIO_TRULY: //6
                                platform_driver_register(&lvds_lcd);
                                break;
                        case DAUDIOKK_LCD_PI_DISCONNECTED: //10 Temporary use DAUDIOKK_LCD_PI_08_00_800_400_PIO_TRULY
                                platform_driver_register(&lvds_lcd);
                                break;
                        default:
                                printk("ADC value is wrong number : %d\n",lcd_ver);
                                break;
                }
	return 0;	
}
static __exit void lcd_select_exit(void)
{
	
	printk("~ %s ~ \n", __func__);

        unsigned char lcd_ver = daudio_lcd_version();
	
	if(gpio_get_value(TCC_GPB(24)))	// OE
	        switch(lcd_ver)
	        {
	                case DAUDIOKK_LCD_OI_10_25_1920_720_INCELL_Si_LG: //0
			case DAUDIOKK_LCD_OI_10_25_1920_720_INCELL_Si_2_LG: //1
	                case DAUDIOKK_LCD_OD_10_25_1920_720_INCELL_Si_LG: //3
	                case DAUDIOKK_LCD_OD_12_30_1920_720_INCELL_Si_LG: //4
	                case DAUDIOKK_LCD_OI_10_25_1920_720_INCELL_LTPS_LG: //7
			case DAUDIOKK_LCD_OD_10_25_1920_720_INCELL_LTPS_LG: //5
	                        platform_driver_unregister(&hdmi_driver);
	                        break;
			case DAUDIOKK_LCD_OD_08_00_1280_720_OGS_Si_BOE: //6 HDMI SERDES
				platform_driver_unregister(&hdmi_driver);
	                        break;
			case DAUDIOKK_LCD_OI_08_00_1280_720_OGS_Si_BOE: //8 LVDS
				platform_driver_unregister(&lvds_lcd);
                                break;
	                case DAUDIOKK_LCD_OI_DISCONNECTED: //10
	                        platform_driver_unregister(&hdmi_driver);
	                        break;
	                default:
	                        printk("ADC value is wrong number : %d\n",lcd_ver);
	                        break;
	        }
	else				// PIO
		switch(lcd_ver)
                {
                        case DAUDIOKK_LCD_PI_10_25_1920_720_PIO_AUO: //5
                                platform_driver_unregister(&hdmi_driver);
                                break;
                        case DAUDIOKK_LCD_PI_08_00_800_400_PIO_TRULY: //6
                                platform_driver_unregister(&lvds_lcd);
                                break;
                        case DAUDIOKK_LCD_PI_DISCONNECTED: //10 Temporary use DAUDIOKK_LCD_PI_08_00_800_400_PIO_TRULY
                                platform_driver_unregister(&lvds_lcd);
                                break;
                        default:
                                printk("ADC value is wrong number : %d\n",lcd_ver);
                                break;
                }

}

module_init(lcd_select_init);
module_exit(lcd_select_exit);

MODULE_DESCRIPTION("LCD driver");
MODULE_LICENSE("GPL");
