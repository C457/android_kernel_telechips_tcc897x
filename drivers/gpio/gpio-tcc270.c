/*********************************************************************************************
   * drivers/mfd/tcc270-gpio.c
   *
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
**********************************************************************************************/


#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/of.h>

#include <linux/mfd/tcc270/tcc270.h>
#include <linux/mfd/tcc270/tcc270-gpio.h>

struct tcc270_gpio_info {
	struct i2c_client *i2c;
	unsigned gpio_base;
	unsigned irq_base;
	int ngpio;
	struct gpio_chip gpio_chip;
};

static inline int find_tcc270_gpioreg(unsigned off, int *gpio_reg)
{
	int ret = 0;
	switch (off)
	{
		case 0:
		case 1:
		case 2:
			*gpio_reg = TCC270_REG_GPIO0 + off;
			break;
		default:
			ret = -EINVAL;
	}
	return ret;
}

static int tcc270_gpio_direction_input(struct gpio_chip *chip, unsigned offset)
{
	struct tcc270_gpio_info *gi = dev_get_drvdata(chip->dev);
	int gpio_reg = 0;
	int ret = 0;
	
	ret = find_tcc270_gpioreg(offset , &gpio_reg);
	if (ret < 0)
	{
		dev_err(chip->dev, "not a valid gpio index\n");
		return ret;
	}

	ret = tcc270_clr_bits(gi->i2c, gpio_reg, TCC270_GPIO_DIRMASK);
	if (ret<0)
	{
		dev_err(chip->dev, "set gpio input fail\n");
		return ret;
	}

	return 0;
}

static int tcc270_gpio_direction_output(struct gpio_chip *chip, unsigned offset, int value)
{
	struct tcc270_gpio_info *gi = dev_get_drvdata(chip->dev);
	int gpio_reg = 0;
	int ret = 0;
	
	ret = find_tcc270_gpioreg(offset, &gpio_reg);
	if (ret < 0)
	{
		dev_err(chip->dev, "not a valid gpio index\n");
		return ret;
	}
	
	ret = tcc270_clr_bits(gi->i2c, gpio_reg, TCC270_GPIO_DIRSHIFT);
	if (ret<0)
	{
		dev_err(chip->dev, "clr gpio direction fail\n");
		return ret;
	}

	ret = tcc270_set_bits(gi->i2c, gpio_reg, TCC270_GPIO_OUTPUT<<TCC270_GPIO_DIRSHIFT);
	if (ret<0)
	{
		dev_err(chip->dev, "set gpio output dir fail\n");
		return ret;
	}

	if (value)
		ret = tcc270_set_bits(gi->i2c, gpio_reg, TCC270_GPIO_OVALUEMASK);
	else
		ret = tcc270_clr_bits(gi->i2c, gpio_reg, TCC270_GPIO_OVALUEMASK);

	if (ret<0)
	{
		dev_err(chip->dev, "set gpio output value fail\n");
		return ret;
	}

	return 0;
}

static int tcc270_gpio_get_value(struct gpio_chip *chip, unsigned offset)
{
	struct tcc270_gpio_info *gi = dev_get_drvdata(chip->dev);
	int gpio_reg = 0;
	int ret = 0;
	
	ret = find_tcc270_gpioreg(offset, &gpio_reg);
	if (ret < 0)
	{
		dev_err(chip->dev, "not a valid gpio index\n");
		return ret;
	}
	
	ret = tcc270_reg_read(gi->i2c, gpio_reg);
	if (ret<0)
	{
		dev_err(chip->dev, "read gpio register fail\n");
		return ret;
	}

	return (ret&TCC270_GPIO_IVALUEMASK)?1:0;
}

static void tcc270_gpio_set_value(struct gpio_chip *chip, unsigned offset, int value)
{
	struct tcc270_gpio_info *gi = dev_get_drvdata(chip->dev);
	int gpio_reg = 0;
	int ret = 0;
	
	ret = find_tcc270_gpioreg(offset, &gpio_reg);
	if (ret < 0)
	{
		dev_err(chip->dev, "not a valid gpio index\n");
		return;
	}

	if (value)
		ret = tcc270_set_bits(gi->i2c, gpio_reg, TCC270_GPIO_OVALUEMASK);
	else
		ret = tcc270_clr_bits(gi->i2c, gpio_reg, TCC270_GPIO_OVALUEMASK);

	if (ret<0)
	{
		dev_err(chip->dev, "read gpio register fail\n");
	}
}

static int tcc270_parse_dt(struct tcc270_gpio_info *gi, struct device *dev)
{
	#ifdef CONFIG_OF
	struct device_node *np = dev->of_node;
	of_property_read_u32(np, "tcc,ngpio", &gi->ngpio);
	#endif /* #ifdef CONFIG_OF */
	return 0;
}

static int tcc270_parse_pdata(struct tcc270_gpio_info *gi, struct device *dev)
{
	struct tcc270_gpio_data *gpio_pdata = dev->platform_data;
	gi->ngpio = gpio_pdata->ngpio;
	return 0;
}

static int tcc270_gpio_probe(struct platform_device *pdev)
{
	struct tcc270_chip *chip = dev_get_drvdata(pdev->dev.parent);
	struct tcc270_platform_data *pdata = (pdev->dev.parent)->platform_data;
	struct tcc270_gpio_info *gi;
	bool use_dt = pdev->dev.of_node;
	int rc = 0;

	gi = devm_kzalloc(&pdev->dev, sizeof(*gi), GFP_KERNEL);
	if (!gi)
		return -ENOMEM;


	gi->i2c = chip->i2c;
	if (use_dt)
		tcc270_parse_dt(gi, &pdev->dev);
	else
	{
		if (!pdata)
		{
			rc = -EINVAL;
			goto out_dev;
		}
		pdev->dev.platform_data = pdata->gpio_pdata;
		tcc270_parse_pdata(gi, &pdev->dev);
	}

	gi->gpio_chip.direction_input  = tcc270_gpio_direction_input;
	gi->gpio_chip.direction_output = tcc270_gpio_direction_output;
	gi->gpio_chip.get = tcc270_gpio_get_value;
	gi->gpio_chip.set = tcc270_gpio_set_value;
	gi->gpio_chip.can_sleep = 0;

	gi->gpio_chip.base = -1;
	gi->gpio_chip.ngpio = gi->ngpio;
	gi->gpio_chip.label = pdev->name;
	gi->gpio_chip.dev = &pdev->dev;
	gi->gpio_chip.owner = THIS_MODULE;

	rc = gpiochip_add(&gi->gpio_chip);
	if (rc)
		goto out_dev;
		
	platform_set_drvdata(pdev, gi);
	dev_info(&pdev->dev, "driver successfully loaded\n");
	return rc;
out_dev:
	devm_kfree(&pdev->dev, gi);
	return rc;
}

static int tcc270_gpio_remove(struct platform_device *pdev)
{
	int rc = 0;
	struct tcc270_gpio_info *gi = platform_get_drvdata(pdev);
	rc = gpiochip_remove(&gi->gpio_chip);
	dev_info(&pdev->dev, "\n");
	return 0;
}

static struct of_device_id tcc270_match_table[] = {
	{ .compatible = "tcc,tcc270-gpio",},
	{},
};
static struct platform_driver tcc270_gpio_driver = 
{
	.driver = {
		.name = TCC270_DEV_NAME "-gpio",
		.owner = THIS_MODULE,
		.of_match_table = tcc270_match_table,
	},
	.probe = tcc270_gpio_probe,
	.remove = tcc270_gpio_remove,
};

static int __init tcc270_gpio_init(void)
{
	return platform_driver_register(&tcc270_gpio_driver);
}
subsys_initcall(tcc270_gpio_init);

static void __exit tcc270_gpio_exit(void)
{
	platform_driver_unregister(&tcc270_gpio_driver);
}
module_exit(tcc270_gpio_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Telechips <android_ce@telechips.com>");
MODULE_DESCRIPTION("GPIO driver for TCC270");
MODULE_ALIAS("platform:" TCC270_DEV_NAME "-gpio");
MODULE_VERSION(TCC270_DRV_VER);
