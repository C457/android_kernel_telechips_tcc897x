/****************************************************************************
 * tcc_bl.c
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/err.h>
#include <linux/pwm.h>
#include <linux/of_gpio.h>

#include <linux/pinctrl/consumer.h>

#ifdef CONFIG_ARCH_TCC897X
#include <mach/gpio.h>
#else
#include <video/tcc/gpio.h>
#endif

struct tcc_bl_data {
	struct device		*dev;
	struct pwm_device	*pwm;
	int			bl_gpio;
	int			bl_gpio_func;
	unsigned int		duty;
	unsigned int		period;
};
static void tcc_brightness_set(struct led_classdev *led_cdev, enum led_brightness value)
{
	struct device *dev = led_cdev->dev->parent;
	struct tcc_bl_data *tccbl = dev_get_drvdata(dev);


	if (tccbl->pwm) {

		if (value >= LED_FULL)
			tccbl->duty = tccbl->period;
		else
			tccbl->duty = value*(tccbl->period/LED_FULL);

		pwm_config(tccbl->pwm, tccbl->duty, tccbl->period);
		if(value)
		{
			pwm_enable(tccbl->pwm);
			pinctrl_get_select(dev, "active");
		}
		else {	
			pinctrl_get_select(dev, "idle");
			pwm_disable(tccbl->pwm);
		}

	}
	else if (gpio_is_valid(tccbl->bl_gpio)) {
		if (value)
			gpio_direction_output(tccbl->bl_gpio, 1);
		else

			gpio_direction_output(tccbl->bl_gpio, 0);
	}
}

static struct led_classdev tcc_backlight_led = {
	.name		= "lcd-backlight",
	.brightness	= 102,
	.brightness_set	= tcc_brightness_set,
};

static int tcc_backlight_probe(struct platform_device *pdev)
{
	struct tcc_bl_data *tccbl;
	unsigned int init_value = 0;
	int ret = 0;

	tccbl = devm_kzalloc(&pdev->dev, sizeof(*tccbl), GFP_KERNEL);
	
	if (!tccbl) {
		dev_err(&pdev->dev, "no memory for state\n");
		return -ENOMEM;
	}
	tccbl->pwm = devm_pwm_get(&pdev->dev, NULL);

	if (IS_ERR(tccbl->pwm)) {
		tccbl->pwm = NULL;
	}

	of_property_read_u32(pdev->dev.of_node, "default-brightness", &init_value);
	tcc_backlight_led.brightness = init_value;

	if (tccbl->pwm) {
		tccbl->period = pwm_get_period(tccbl->pwm);
		tccbl->duty = init_value * (tccbl->period/LED_FULL);

		pwm_config(tccbl->pwm, tccbl->duty, tccbl->period);
		if (pwm_enable(tccbl->pwm))
			goto err_probe;

		pinctrl_get_select(&pdev->dev, "active");

	}
	else {

		tccbl->bl_gpio = of_get_named_gpio(pdev->dev.of_node, "bl-gpios", 0);
		 if (gpio_is_valid(tccbl->bl_gpio))
			gpio_request(tccbl->bl_gpio, "lcd_backlight");
		 
		if (!gpio_is_valid(tccbl->bl_gpio)) {
			ret = -ENOMEM;
			goto err_probe;
		}

		if(tcc_backlight_led.brightness)
			gpio_direction_output(tccbl->bl_gpio, 1);
		else
			gpio_direction_output(tccbl->bl_gpio, 0);
	}

	tccbl->dev = &pdev->dev;
	ret = led_classdev_register(&pdev->dev, &tcc_backlight_led);
	if (ret) {
		dev_err(&pdev->dev, "led_classdev_register failed!\n");
		goto err_reg_led;
	}

	platform_set_drvdata(pdev, tccbl);
	return 0;

err_reg_led:
	devm_pwm_put(&pdev->dev, tccbl->pwm);
	
err_probe:
	devm_kfree(&pdev->dev, tccbl);
	return ret;
}

static int tcc_backlight_remove(struct platform_device *pdev)
{
	struct tcc_bl_data *tccbl = platform_get_drvdata(pdev);

	led_classdev_unregister(&tcc_backlight_led);
	pwm_config(tccbl->pwm, 0, tccbl->period);
	pwm_disable(tccbl->pwm);
	devm_pwm_put(tccbl->dev, tccbl->pwm);
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
	struct tcc_bl_data *tccbl = platform_get_drvdata(tccbl_device);
	struct led_classdev *led_cdev = dev_get_drvdata(dev);

	tcc_brightness_set(led_cdev, led_cdev->brightness);

	return 0;
}
#endif

/*
static SIMPLE_DEV_PM_OPS(tcc_backlight_pm_ops, tcc_backlight_suspend,
			 tcc_backlight_resume);
*/
const struct dev_pm_ops tcc_backlight_pm_ops = {
	.suspend		= tcc_backlight_suspend,
	.resume			= tcc_backlight_resume,
	.freeze			= tcc_backlight_suspend,
	.thaw			= tcc_backlight_resume,
	.restore			= tcc_backlight_restore,
};

static struct of_device_id tcc_backlight_of_match[] = {
	{ .compatible = "telechips,backlight" },
	{ }
};

MODULE_DEVICE_TABLE(of, tcc_backlight_of_match);

static struct platform_driver tcc_backlight_driver = {
	.driver		= {
		.name		= "tcc-backlight",
		.owner		= THIS_MODULE,
		.pm		= &tcc_backlight_pm_ops,
		.of_match_table	= of_match_ptr(tcc_backlight_of_match),
	},
	.probe		= tcc_backlight_probe,
	.remove		= tcc_backlight_remove,
};

module_platform_driver(tcc_backlight_driver);

MODULE_DESCRIPTION("Telechips Backlight Driver");
MODULE_LICENSE("GPL");

