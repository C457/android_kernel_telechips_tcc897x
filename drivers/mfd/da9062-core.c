/* da9062-core.c - CORE device driver for DA9062
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/mfd/core.h>
#include <linux/regmap.h>

#include <linux/mfd/da9062/core.h>
#include <linux/mfd/da9062/registers.h>

#include <linux/proc_fs.h>
#include <linux/kthread.h>
#include <linux/uaccess.h>


static struct resource da9062_regulators_resources[] = {
	{
		.name	= "LDO_LIM",
		.start	= DA9062_IRQ_LDO_LIM,
		.end	= DA9062_IRQ_LDO_LIM,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "VDD_WARN",
		.start	= DA9062_IRQ_VDD_WARN,
		.end	= DA9062_IRQ_VDD_WARN,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct resource da9062_thermal_resources[] = {
	{
		.name	= "THERMAL",
		.start	= DA9062_IRQ_TEMP,
		.end	= DA9062_IRQ_TEMP,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct resource da9062_wdt_resources[] = {
	{
		.name	= "WDG_WARN",
		.start	= DA9062_IRQ_WDG_WARN,
		.end	= DA9062_IRQ_WDG_WARN,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct resource da9062_rtc_resources[] = {
	{
		.name	= "ALARM",
		.start	= DA9062_IRQ_ALARM,
		.end	= DA9062_IRQ_ALARM,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "TICK",
		.start	= DA9062_IRQ_TICK,
		.end	= DA9062_IRQ_TICK,
		.flags	= IORESOURCE_IRQ,
	}
};

static struct resource da9062_onkey_resources[] = {
	{
		.name	= "ONKEY",
		.start	= DA9062_IRQ_ONKEY,
		.end	= DA9062_IRQ_ONKEY,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct mfd_cell da9062_devs[] = {
	{
		.name		= "da9062-regulators",
		.num_resources	= ARRAY_SIZE(da9062_regulators_resources),
		.resources	= da9062_regulators_resources,
	},
	{
		.name		= "da9062-watchdog",
		.num_resources	= ARRAY_SIZE(da9062_wdt_resources),
		.resources	= da9062_wdt_resources,
		.of_compatible  = "dlg,da9062-wdt",
	},
	{
		.name		= "da9062-onkey",
		.num_resources	= ARRAY_SIZE(da9062_onkey_resources),
		.resources	= da9062_onkey_resources,
		.of_compatible  = "dlg,da9062-onkey",
	},
	{
		.name		= "da9062-thermal",
		.num_resources	= ARRAY_SIZE(da9062_thermal_resources),
		.resources	= da9062_thermal_resources,
		.of_compatible  = "dlg,da9062-thermal",
	},
	{
		.name		= "da9062-rtc",
		.num_resources	= ARRAY_SIZE(da9062_rtc_resources),
		.resources	= da9062_rtc_resources,
		.of_compatible  = "dlg,da9062-rtc",
	},
};

static struct da9062 *da9062_pm_power_off;

static int da9062_clear_fault_log(struct da9062 *chip)
{
	int ret = 0;
	int fault_log = 0;

	ret = regmap_read(chip->regmap, DA9062AA_FAULT_LOG, &fault_log );
	if (ret < 0) {
		dev_err(chip->dev, "Cannot read FAULT_LOG.\n");
		return -EIO;
	}
	else {
		if (fault_log) {
			if( fault_log & DA9062AA_TWD_ERROR_MASK )
				dev_notice(chip->dev, "Fault log entry detected: TWD_ERROR\n");
			if( fault_log & DA9062AA_POR_MASK )
				dev_notice(chip->dev, "Fault log entry detected: POR\n");
			if( fault_log & DA9062AA_VDD_FAULT_MASK )
				dev_notice(chip->dev, "Fault log entry detected: VDD_FAULT\n");
			if( fault_log & DA9062AA_VDD_START_MASK )
				dev_notice(chip->dev, "Fault log entry detected: VDD_START\n");
			if( fault_log & DA9062AA_TEMP_CRIT_MASK )
				dev_notice(chip->dev, "Fault log entry detected: TEMP_CRIT\n");
			if( fault_log & DA9062AA_KEY_RESET_MASK )
				dev_notice(chip->dev, "Fault log entry detected: KEY_RESET\n");
			if( fault_log & DA9062AA_NSHUTDOWN_MASK )
				dev_notice(chip->dev, "Fault log entry detected: NSHUTDOWN\n");
			if( fault_log & DA9062AA_WAIT_SHUT_MASK )
				dev_notice(chip->dev, "Fault log entry detected: WAIT_SHUT\n");
		}

		ret = regmap_write(chip->regmap, DA9062AA_FAULT_LOG,
				   fault_log );
		if (ret < 0)
			dev_err(chip->dev, "Cannot reset FAULT_LOG values %d\n", ret);
	}

	return ret;
}

void da9062_power_off(void)
{
    int ret;
    unsigned int power_f = 0;

    if(da9062_pm_power_off)
    {
        ret = regmap_write(da9062_pm_power_off->regmap, DA9062AA_LDO1_CONT, 0x05);
        if (ret < 0)
            dev_err(da9062_pm_power_off->dev, "Cannot write ldo1 power_conf values %d\n", ret);

        power_f = 0x00;

        ret = regmap_write(da9062_pm_power_off->regmap, DA9062AA_CONTROL_A, power_f);
        if (ret < 0)
            dev_err(da9062_pm_power_off->dev, "Cannot write cont_a power_conf values %d\n", ret);
    }
}
EXPORT_SYMBOL(da9062_power_off);

int da9062_device_init(struct da9062 *chip, unsigned int irq)
{
	int device_id, variant_id, variant_mrc;
	int ret;

    da9062_pm_power_off = chip;

	ret = da9062_clear_fault_log(chip);
	if (ret < 0)
		dev_err(chip->dev, "Cannot clear fault log\n");

	chip->irq_base = -1;
	chip->chip_irq = irq;

	ret = regmap_read(chip->regmap, DA9062AA_DEVICE_ID, &device_id);
	if (ret < 0) {
		dev_err(chip->dev, "Cannot read chip ID.\n");
		return -EIO;
	}
	if (device_id != DA9062_PMIC_DEVICE_ID) {
		dev_err(chip->dev, "Invalid device ID: 0x%02x\n", device_id);
		return -ENODEV;
	}

	ret = regmap_read(chip->regmap, DA9062AA_VARIANT_ID, &variant_id);
	if (ret < 0) {
		dev_err(chip->dev, "Cannot read chip variant id.\n");
		return -EIO;
	}

	dev_info(chip->dev,
		 "Device detected (device-ID: 0x%02X, var-ID: 0x%02X)\n",
		 device_id, variant_id);

	variant_mrc = (variant_id & DA9062AA_MRC_MASK) >> DA9062AA_MRC_SHIFT;

	if (variant_mrc < DA9062_PMIC_VARIANT_MRC_AA) {
		dev_err(chip->dev,
			"Cannot support variant MRC: 0x%02X\n", variant_mrc);
		return -ENODEV;
	}

	chip->device_id = device_id;
	chip->variant_mrc = variant_mrc;

	/*ret = da9062_irq_init(chip);
	if (ret) {
		dev_err(chip->dev, "Cannot initialize interrupts.\n");
		return ret;
	}

	chip->irq_base = regmap_irq_chip_get_base(chip->regmap_irq);*/

	ret = mfd_add_devices(chip->dev, -1, da9062_devs,
			      ARRAY_SIZE(da9062_devs), NULL, 0,
			      NULL);
	if (ret) {
		dev_err(chip->dev, "Cannot add MFD cells\n");
		// da9062_irq_exit(chip);
		return ret;
	}

	return ret;
}

void da9062_device_exit(struct da9062 *chip)
{
	mfd_remove_devices(chip->dev);
	// da9062_irq_exit(chip);
}

MODULE_DESCRIPTION("CORE device driver for Dialog DA9062");
MODULE_AUTHOR("S Twiss <stwiss.opensource@diasemi.com>");
MODULE_LICENSE("GPL v2");
