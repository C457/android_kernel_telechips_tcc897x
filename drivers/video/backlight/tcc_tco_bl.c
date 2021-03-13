/****************************************************************************
 * drivers/video/backlight/tcc_tco_bl.c
 * Copyright (C) 2014 Telechips Inc.
 *
 * This program is free software; you can redistribute it and/or modify it under the terms
 * of the GNU General Public License as published by the Free Software Foundation;
 * either version 2 of the License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE. See the GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc., 59 Temple Place,
 * Suite 330, Boston, MA 02111-1307 USA
****************************************************************************/
#include <linux/platform_device.h>

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/irq.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/leds.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>

#ifdef CONFIG_ARCH_TCC897X
#include <mach/gpio.h>
#else
#include <video/tcc/gpio.h>
#endif

#define tco_readl			__raw_readl
#define tco_writel		__raw_writel

#define TCC_TCFG	0x0
#define TCC_TCNT	0x4
#define TCC_TREF	0x8
#define TCC_TMREF	0xC

#define PWM_ADJUST_RANGE		85		//persent
#define PWM_ADJUST_MIN		(100 - PWM_ADJUST_RANGE)
#define PWM_DIVID				(LED_FULL + ((LED_FULL * PWM_ADJUST_MIN) /100 ))


static void __iomem	*tco_base_reg = NULL;
static unsigned long	clk_rate;
struct tcc_tco_data{
	struct clk		*tco_clk;
	unsigned int 		ntco;		//tco number
	unsigned	k;
	unsigned	mref;
	unsigned	ref;
};

struct tcc_tco_bl_data {
	struct device		*dev;
	struct tcc_tco_data tco;
	int				bl_gpio;
	int				bl_gpio_func;
	unsigned int		duty;
	unsigned int		period;
};

//get duty range \
// p : period v : value
#define GET_DUTY(p, v)		((v + ((LED_FULL * PWM_ADJUST_MIN) /100 )) * (p /PWM_DIVID))

int tcc_tco_enable(struct tcc_tco_data *tco)
{
	void __iomem *reg;
	if (!tco)
		return 0;

	reg = tco_base_reg+(tco->ntco*0x10);
	tco_writel(tco_readl(reg+TCC_TCFG)|(1<<2)|(1<<0), reg+TCC_TCFG);	/* IEN, EN */
	return 0;
}

int tcc_tco_disable(struct tcc_tco_data *tco)
{
	void __iomem *reg;
	if (!tco)
		return 0;
	
	reg = tco_base_reg+(tco->ntco*0x10);
	tco_writel(tco_readl(reg+TCC_TCFG) & ~((1<<2)|(1<<0)), reg+TCC_TCFG);	/* IEN, EN */
	tco_writel(0x0, reg+TCC_TCNT);
	return 0;
}

int tcc_tco_config(struct tcc_tco_data *tco)
{
	void __iomem *reg;
	if (!tco)
		return 0;
	reg = tco_base_reg+(tco->ntco*0x10);		
	tco_writel(tco->mref, reg +TCC_TMREF); 	//high count
	tco_writel(tco->ref, reg+TCC_TREF);		//max count
	return 0;
}

int tco_pwm_init(struct tcc_tco_bl_data  *bl)
{
	int		k;
	unsigned int 		rate, max_ref, tclk, req_hz;
	struct device_node *tco_np;
	void __iomem *reg;

	tco_np = of_find_compatible_node(NULL, NULL, "telechips,timer");

	if(tco_np == NULL) {
		pr_err("cann't find vioc_config \n");
		return -1;
	}
	else{
		tco_base_reg = of_iomap(tco_np, 0);
	}

	if (of_property_read_u32(tco_np, "clock-frequency", &rate)) {
		printk("%s: Can't read clock-frequency\n", __func__);
		rate = 12000000;
	}
	bl->tco.tco_clk = of_clk_get(tco_np, 0);
	if (IS_ERR(bl->tco.tco_clk)) {
		pr_warn("Unable to get timer clock.\n");
		BUG();
	} else {
		clk_set_rate(bl->tco.tco_clk, rate);
		printk("%s: clk_rate: %lu\n", __func__, clk_get_rate(bl->tco.tco_clk));
		clk_prepare_enable(bl->tco.tco_clk);
	}

	/* find divide factor */
	clk_rate = clk_get_rate(bl->tco.tco_clk);

	req_hz = ((1000 * 1000)/bl->period) * PWM_DIVID;

	if (bl->tco.ntco < 4)
		max_ref = 0xFFFF;	/* 16bit counter */
	else
		max_ref = 0xFFFFF;	/* 20bit counter */

	/* find divide factor */
	for (k=0 ; k <= 6 ; k++) {
		int div, cnt, max_cnt;
		if (k<5)
			max_cnt = k+1;
		else
			max_cnt = 2*k;
		div = 1;
		for (cnt=0 ; cnt<max_cnt ; cnt++)
			div *= 2;

		tclk = clk_rate/div;

		if ((tclk>req_hz)  && (tclk/req_hz <= max_ref))
			break;
		else if(tclk < req_hz)
			break;
	}

	bl->tco.k = k;
	bl->tco.ref = (tclk / req_hz) * PWM_DIVID;

	reg = tco_base_reg + (bl->tco.ntco * 0x10);
	tco_writel((1<<8)|(bl->tco.k<<4)|(1<<2), reg+TCC_TCFG); //pwm en, 

	return 0;
}



static void tcc_brightness_set(struct led_classdev *led_cdev, enum led_brightness value)
{
	struct device *dev = led_cdev->dev->parent;
	struct tcc_tco_bl_data *tccbl = dev_get_drvdata(dev);

	if (value >= LED_FULL)
		tccbl->duty = LED_FULL;
	else
		tccbl->duty = value;

	tccbl->tco.mref = tccbl->tco.ref - (((LED_FULL -  tccbl->duty)* tccbl->tco.ref) /PWM_DIVID);

	tcc_tco_config(&tccbl->tco);
	
	if(value)
	{
		tcc_tco_enable(&tccbl->tco);
		tcc_gpio_config(tccbl->bl_gpio,  GPIO_FN(tccbl->bl_gpio_func));
	}
	else
	{	
		gpio_direction_output(tccbl->bl_gpio, 0);
		tcc_gpio_config(tccbl->bl_gpio,  GPIO_FN(0));
		tcc_tco_disable(&tccbl->tco);
	}
}

static struct led_classdev tcc_backlight_led = {
	.name		= "lcd-backlight",
	.brightness	= 102,
	.brightness_set	= tcc_brightness_set,
};

static int tcc_tco_backlight_probe(struct platform_device *pdev)
{
	struct tcc_tco_bl_data *tccbl;
	int ret = 0;

	tccbl = devm_kzalloc(&pdev->dev, sizeof(*tccbl), GFP_KERNEL);

	if (!tccbl) {
		dev_err(&pdev->dev, "no memory for state\n");
		return -ENOMEM;
	}

	tccbl->bl_gpio = of_get_named_gpio(pdev->dev.of_node, "bl-gpios", 0);
	of_property_read_u32_index(pdev->dev.of_node, "tco", 0, &tccbl->tco.ntco);
	of_property_read_u32_index(pdev->dev.of_node, "tco", 1, &tccbl->period);

	 if (gpio_is_valid(tccbl->bl_gpio))
		gpio_request(tccbl->bl_gpio, "lcd_backlight");
	 
	if (!gpio_is_valid(tccbl->bl_gpio)) {
		ret = -ENOMEM;
		goto err_tco;
	}

	of_property_read_u32(pdev->dev.of_node, "bl-gpio-func", &tccbl->bl_gpio_func);
	tcc_gpio_config(tccbl->bl_gpio,  GPIO_FN(tccbl->bl_gpio_func));

	of_property_read_u32(pdev->dev.of_node, "default-brightness", &tccbl->duty);
	tcc_backlight_led.brightness = tccbl->duty;



	ret = tco_pwm_init(tccbl);
	
	tccbl->tco.mref = tccbl->tco.ref - (((LED_FULL -  tccbl->duty)* tccbl->tco.ref) /PWM_DIVID);
	
	tcc_tco_config(&tccbl->tco);
	tcc_tco_enable(&tccbl->tco);

	tccbl->dev = &pdev->dev;
	ret = led_classdev_register(&pdev->dev, &tcc_backlight_led);
	if (ret) {
		dev_err(&pdev->dev, "led_classdev_register failed!\n");
		goto err_tco;
	}

	platform_set_drvdata(pdev, tccbl);
	return 0;


err_tco:
	devm_kfree(&pdev->dev, tccbl);
	return ret;
}

static int tcc_backlight_remove(struct platform_device *pdev)
{
	struct tcc_tco_bl_data *tccbl = platform_get_drvdata(pdev);

	led_classdev_unregister(&tcc_backlight_led);

	tcc_tco_disable(&tccbl->tco);
	devm_kfree(tccbl->dev, tccbl);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int tcc_backlight_suspend(struct device *dev)
{
	return 0;
}

static int tcc_backlight_resume(struct device *dev)
{
	return 0;
}

static int tcc_backlight_restore(struct device *dev)
{
	struct platform_device *tccbl_device = container_of(dev, struct platform_device, dev);
	struct tcc_tco_bl_data *tccbl = platform_get_drvdata(tccbl_device);
	
	tcc_tco_config(&tccbl->tco);
	return 0;
}
#endif

const struct dev_pm_ops tcc_tco_backlight_pm_ops = {
	.suspend		= tcc_backlight_suspend,
	.resume			= tcc_backlight_resume,
	.freeze			= tcc_backlight_suspend,
	.thaw			= tcc_backlight_resume,
	.restore		= tcc_backlight_restore,
};

static struct of_device_id tcc_tco_backlight_of_match[] = {
	{ .compatible = "telechips,tco_backlight" },
	{ }
};

MODULE_DEVICE_TABLE(of, tcc_tco_backlight_of_match);

static struct platform_driver tcc_backlight_driver = {
	.driver		= {
		.name		= "tcc-tco-backlight",
		.owner		= THIS_MODULE,
		.pm		= &tcc_tco_backlight_pm_ops,
		.of_match_table	= of_match_ptr(tcc_tco_backlight_of_match),
	},
	.probe		= tcc_tco_backlight_probe,
	.remove		= tcc_backlight_remove,
};

module_platform_driver(tcc_backlight_driver);

MODULE_DESCRIPTION("Telechips Backlight Driver");
MODULE_LICENSE("GPL");

