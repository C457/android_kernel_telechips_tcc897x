/* da9062-onkey.c - ONKEY device driver for DA9062
 * Copyright (C) 2015  Dialog Semiconductor Ltd.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Library General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Library General Public License for more details.
 */

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/regmap.h>
#include <linux/of.h>
#include <linux/mfd/da9062/core.h>
#include <linux/mfd/da9062/registers.h>

struct da9062_onkey {
	struct da9062 *hw;
	struct delayed_work work;
	struct	input_dev *input;
	int irq;
	bool key_power;
};

static void da9062_poll_on(struct work_struct *work)
{
	struct da9062_onkey *onkey = container_of(work, struct da9062_onkey,
						  work.work);
	unsigned int val;
	int fault_log = 0;
	bool poll = true;
	int ret;

	/* poll to see when the pin is released */
	ret = regmap_read(onkey->hw->regmap, DA9062AA_STATUS_A, &val);
	if (ret < 0) {
		dev_err(&onkey->input->dev,
			"Failed to read ON status: %d\n", ret);
		goto err_poll;
	}

	if (!(val & DA9062AA_NONKEY_MASK)) {
		ret = regmap_update_bits(onkey->hw->regmap,
					 DA9062AA_CONTROL_B,
					 DA9062AA_NONKEY_LOCK_MASK, 0);
		if (ret < 0) {
			dev_err(&onkey->input->dev,
				"Failed to reset the Key Delay %d\n", ret);
			goto err_poll;
		}

		/* unmask the onkey interrupt again */
		ret = regmap_update_bits(onkey->hw->regmap,
					 DA9062AA_IRQ_MASK_A,
					 DA9062AA_NONKEY_MASK, 0);
		if (ret < 0) {
			dev_err(&onkey->input->dev,
				"Failed to unmask the onkey IRQ: %d\n", ret);
			goto err_poll;
		}

		input_report_key(onkey->input, KEY_POWER, 0);
		input_sync(onkey->input);

		poll = false;
	}

	/* if the fault log KEY_RESET is detected,
	 * then clear it and shutdown DA9062 via I2C
	 */
	ret = regmap_read(onkey->hw->regmap, DA9062AA_FAULT_LOG, &fault_log);
	if (ret < 0)
		dev_warn(&onkey->input->dev, "Cannot read FAULT_LOG\n");

	if( fault_log & DA9062AA_KEY_RESET_MASK ) {
		ret = regmap_write(onkey->hw->regmap,
				   DA9062AA_FAULT_LOG,
				   DA9062AA_KEY_RESET_MASK);
		if (ret < 0)
			dev_warn(&onkey->input->dev,
				 "Cannot reset KEY_RESET fault log\n");
		else {
			/* at this point we do any S/W housekeeping
			 * and then send shutdown command
			 */
			printk( KERN_EMERG "Sending SHUTDOWN to DA9062 ...\n");
			ret = regmap_write(onkey->hw->regmap,
					   DA9062AA_CONTROL_F,
					   DA9062AA_SHUTDOWN_MASK);
			if (ret < 0)
				dev_err(&onkey->input->dev,
					"Cannot SHUTDOWN DA9062\n");
		}
	}

err_poll:
	if (poll)
		schedule_delayed_work(&onkey->work, 50);
}

/*static irqreturn_t da9062_onkey_irq_handler(int irq, void *data)
{
	struct da9062_onkey *onkey = data;
	unsigned int val;
	int ret;

	ret = regmap_read(onkey->hw->regmap, DA9062AA_STATUS_A, &val);
	if (onkey->key_power && (ret >= 0) && (val & DA9062AA_NONKEY_MASK)) {
		ret = regmap_update_bits(onkey->hw->regmap,
					 DA9062AA_IRQ_MASK_A,
					 DA9062AA_NONKEY_MASK, 1);
		if (ret < 0)
			dev_err(&onkey->input->dev,
				"Failed to mask the onkey IRQ: %d\n", ret);

		input_report_key(onkey->input, KEY_POWER, 1);
		input_sync(onkey->input);
		schedule_delayed_work(&onkey->work, 0);
		dev_notice(&onkey->input->dev, "KEY_POWER pressed.\n");
	} else {
		input_report_key(onkey->input, KEY_SLEEP, 1);
		input_sync(onkey->input);
		input_report_key(onkey->input, KEY_SLEEP, 0);
		input_sync(onkey->input);
		dev_notice(&onkey->input->dev, "KEY_SLEEP pressed.\n");
	}

	return IRQ_HANDLED;
}*/

static int da9062_onkey_probe(struct platform_device *pdev)
{
	struct da9062 *chip = dev_get_drvdata(pdev->dev.parent);
	struct da9062_onkey *onkey;
	bool kp_tmp = true;
	int ret = 0;

		kp_tmp = of_property_read_bool((&pdev->dev)->of_node,
					       "dlg,disable-key-power");
		kp_tmp = !kp_tmp;


	onkey = devm_kzalloc(&pdev->dev, sizeof(struct da9062_onkey),
			     GFP_KERNEL);
	if (!onkey) {
		dev_err(&pdev->dev, "Failed to allocate memory.\n");
		ret = -ENOMEM;
		goto err;
	}

	INIT_DELAYED_WORK(&onkey->work, da9062_poll_on);

	onkey->input = devm_input_allocate_device(&pdev->dev);
	if (!onkey->input) {
		dev_err(&pdev->dev, "Failed to allocated input device.\n");
		ret = -ENOMEM;
		goto err;
	}

	/*ret = platform_get_irq_byname(pdev, "ONKEY");
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to get platform IRQ.\n");
		goto err;
	}
    ret = regmap_irq_get_virq(chip->regmap_irq, ret);
	onkey->irq = ret;

	ret = request_threaded_irq(onkey->irq, NULL,
				   da9062_onkey_irq_handler,
				   IRQF_TRIGGER_LOW | IRQF_ONESHOT,
				   "ONKEY", onkey);
	if (ret) {
		dev_err(&pdev->dev,
			"Failed to request input device IRQ.\n");
		goto err;
	}*/

	onkey->hw = chip;
	onkey->key_power = kp_tmp;
	onkey->input->evbit[0] = BIT_MASK(EV_KEY);
	onkey->input->name = "da9062-onkey";
	onkey->input->phys = "da9062-onkey/input0";
	onkey->input->dev.parent = &pdev->dev;

	if (onkey->key_power)
		input_set_capability(onkey->input, EV_KEY, KEY_POWER);
	input_set_capability(onkey->input, EV_KEY, KEY_SLEEP);

	ret = input_register_device(onkey->input);
	if (ret) {
		dev_err(&pdev->dev,
			"Failed to register input device.\n");
		goto err_irq;
	}

	platform_set_drvdata(pdev, onkey);
	return 0;

err_irq:
	// free_irq(onkey->irq, onkey);
	cancel_delayed_work_sync(&onkey->work);
err:
	return ret;
}

static int da9062_onkey_remove(struct platform_device *pdev)
{
	struct	da9062_onkey *onkey = platform_get_drvdata(pdev);
	// free_irq(onkey->irq, onkey);
	cancel_delayed_work_sync(&onkey->work);
	input_unregister_device(onkey->input);
	return 0;
}

static struct platform_driver da9062_onkey_driver = {
	.probe	= da9062_onkey_probe,
	.remove	= da9062_onkey_remove,
	.driver	= {
		.name	= "da9062-onkey",
		.owner	= THIS_MODULE,
	},
};

module_platform_driver(da9062_onkey_driver);

MODULE_AUTHOR("S Twiss <stwiss.opensource@diasemi.com>");
MODULE_DESCRIPTION("ONKEY device driver for Dialog DA9062");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform: da9062-onkey");
