/* da9062-i2c.c - I2C device driver for DA9062
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
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/mfd/core.h>
#include <linux/mfd/da9062/core.h>
#include <linux/mfd/da9062/registers.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/regulator/of_regulator.h>

static const struct regmap_range da9062_aa_readable_ranges[] = {
	{
		.range_min = DA9062AA_PAGE_CON,
		.range_max = DA9062AA_STATUS_B,
	}, {
		.range_min = DA9062AA_STATUS_D,
		.range_max = DA9062AA_EVENT_C,
	}, {
		.range_min = DA9062AA_IRQ_MASK_A,
		.range_max = DA9062AA_IRQ_MASK_C,
	}, {
		.range_min = DA9062AA_CONTROL_A,
		.range_max = DA9062AA_GPIO_4,
	}, {
		.range_min = DA9062AA_GPIO_WKUP_MODE,
		.range_max = DA9062AA_BUCK4_CONT,
	}, {
		.range_min = DA9062AA_BUCK3_CONT,
		.range_max = DA9062AA_BUCK3_CONT,
	}, {
		.range_min = DA9062AA_LDO1_CONT,
		.range_max = DA9062AA_LDO4_CONT,
	}, {
		.range_min = DA9062AA_DVC_1,
		.range_max = DA9062AA_DVC_1,
	}, {
		.range_min = DA9062AA_COUNT_S,
		.range_max = DA9062AA_SECOND_D,
	}, {
		.range_min = DA9062AA_SEQ,
		.range_max = DA9062AA_ID_4_3,
	}, {
		.range_min = DA9062AA_ID_12_11,
		.range_max = DA9062AA_ID_16_15,
	}, {
		.range_min = DA9062AA_ID_22_21,
		.range_max = DA9062AA_ID_32_31,
	}, {
		.range_min = DA9062AA_SEQ_A,
		.range_max = DA9062AA_BUCK3_CFG,
	}, {
		.range_min = DA9062AA_VBUCK2_A,
		.range_max = DA9062AA_VBUCK4_A,
	}, {
		.range_min = DA9062AA_VBUCK3_A,
		.range_max = DA9062AA_VBUCK3_A,
	}, {
		.range_min = DA9062AA_VLDO1_A,
		.range_max = DA9062AA_VLDO4_A,
	}, {
		.range_min = DA9062AA_VBUCK2_B,
		.range_max = DA9062AA_VBUCK4_B,
	}, {
		.range_min = DA9062AA_VBUCK3_B,
		.range_max = DA9062AA_VBUCK3_B,
	}, {
		.range_min = DA9062AA_VLDO1_B,
		.range_max = DA9062AA_VLDO4_B,
	}, {
		.range_min = DA9062AA_BBAT_CONT,
		.range_max = DA9062AA_BBAT_CONT,
	}, {
		.range_min = DA9062AA_INTERFACE,
		.range_max = DA9062AA_CONFIG_E,
	}, {
		.range_min = DA9062AA_CONFIG_G,
		.range_max = DA9062AA_CONFIG_K,
	}, {
		.range_min = DA9062AA_CONFIG_M,
		.range_max = DA9062AA_CONFIG_M,
	}, {
		.range_min = DA9062AA_TRIM_CLDR,
		.range_max = DA9062AA_GP_ID_19,
	}, {
		.range_min = DA9062AA_DEVICE_ID,
		.range_max = DA9062AA_CONFIG_ID,
	},
};

static const struct regmap_range da9062_aa_writeable_ranges[] = {
	{
		.range_min = DA9062AA_PAGE_CON,
		.range_max = DA9062AA_PAGE_CON,
	}, {
		.range_min = DA9062AA_FAULT_LOG,
		.range_max = DA9062AA_EVENT_C,
	}, {
		.range_min = DA9062AA_IRQ_MASK_A,
		.range_max = DA9062AA_IRQ_MASK_C,
	}, {
		.range_min = DA9062AA_CONTROL_A,
		.range_max = DA9062AA_GPIO_4,
	}, {
		.range_min = DA9062AA_GPIO_WKUP_MODE,
		.range_max = DA9062AA_BUCK4_CONT,
	}, {
		.range_min = DA9062AA_BUCK3_CONT,
		.range_max = DA9062AA_BUCK3_CONT,
	}, {
		.range_min = DA9062AA_LDO1_CONT,
		.range_max = DA9062AA_LDO4_CONT,
	}, {
		.range_min = DA9062AA_DVC_1,
		.range_max = DA9062AA_DVC_1,
	}, {
		.range_min = DA9062AA_COUNT_S,
		.range_max = DA9062AA_ALARM_Y,
	}, {
		.range_min = DA9062AA_SEQ,
		.range_max = DA9062AA_ID_4_3,
	}, {
		.range_min = DA9062AA_ID_12_11,
		.range_max = DA9062AA_ID_16_15,
	}, {
		.range_min = DA9062AA_ID_22_21,
		.range_max = DA9062AA_ID_32_31,
	}, {
		.range_min = DA9062AA_SEQ_A,
		.range_max = DA9062AA_BUCK3_CFG,
	}, {
		.range_min = DA9062AA_VBUCK2_A,
		.range_max = DA9062AA_VBUCK4_A,
	}, {
		.range_min = DA9062AA_VBUCK3_A,
		.range_max = DA9062AA_VBUCK3_A,
	}, {
		.range_min = DA9062AA_VLDO1_A,
		.range_max = DA9062AA_VLDO4_A,
	}, {
		.range_min = DA9062AA_VBUCK2_B,
		.range_max = DA9062AA_VBUCK4_B,
	}, {
		.range_min = DA9062AA_VBUCK3_B,
		.range_max = DA9062AA_VBUCK3_B,
	}, {
		.range_min = DA9062AA_VLDO1_B,
		.range_max = DA9062AA_VLDO4_B,
	}, {
		.range_min = DA9062AA_BBAT_CONT,
		.range_max = DA9062AA_BBAT_CONT,
	}, {
		.range_min = DA9062AA_GP_ID_0,
		.range_max = DA9062AA_GP_ID_19,
	},
};

static const struct regmap_range da9062_aa_volatile_ranges[] = {
	{
		.range_min = DA9062AA_PAGE_CON,
		.range_max = DA9062AA_STATUS_B,
	}, {
		.range_min = DA9062AA_STATUS_D,
		.range_max = DA9062AA_EVENT_C,
	}, {
		.range_min = DA9062AA_CONTROL_F,
		.range_max = DA9062AA_CONTROL_F,
	}, {
		.range_min = DA9062AA_COUNT_S,
		.range_max = DA9062AA_SECOND_D,
	},
};

static const struct regmap_access_table da9062_aa_readable_table = {
	.yes_ranges = da9062_aa_readable_ranges,
	.n_yes_ranges = ARRAY_SIZE(da9062_aa_readable_ranges),
};

static const struct regmap_access_table da9062_aa_writeable_table = {
	.yes_ranges = da9062_aa_writeable_ranges,
	.n_yes_ranges = ARRAY_SIZE(da9062_aa_writeable_ranges),
};

static const struct regmap_access_table da9062_aa_volatile_table = {
	.yes_ranges = da9062_aa_volatile_ranges,
	.n_yes_ranges = ARRAY_SIZE(da9062_aa_volatile_ranges),
};

static const struct regmap_range_cfg da9062_range_cfg[] = {
	{
		.range_min = DA9062AA_PAGE_CON,
		.range_max = DA9062AA_CONFIG_ID,
		.selector_reg = DA9062AA_PAGE_CON,
		.selector_mask = 1 << DA9062_I2C_PAGE_SEL_SHIFT,
		.selector_shift = DA9062_I2C_PAGE_SEL_SHIFT,
		.window_start = 0,
		.window_len = 256,
	}
};

static struct regmap_config da9062_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.ranges = da9062_range_cfg,
	.num_ranges = ARRAY_SIZE(da9062_range_cfg),
	.max_register = DA9062AA_CONFIG_ID,
	.cache_type = REGCACHE_RBTREE,
};

static const struct of_device_id da9062_dt_ids[] = {
        { .compatible = "dlg,da9062", },
        { }
};
MODULE_DEVICE_TABLE(of, da9062_dt_ids);

static int da9062_i2c_probe(struct i2c_client *i2c,
	const struct i2c_device_id *id)
{
	struct da9062 *chip;
    struct device_node *np;
	int ret, irq_port;

    np = i2c->dev.of_node;

    irq_port = of_get_named_gpio(np, "int-gpios", 0);
    if (gpio_is_valid(irq_port))
        i2c->irq = gpio_to_irq(irq_port);

	chip = devm_kzalloc(&i2c->dev, sizeof(struct da9062), GFP_KERNEL);
	if (chip == NULL)
		return -ENOMEM;

	i2c_set_clientdata(i2c, chip);
	chip->dev = &i2c->dev;
	chip->chip_irq = i2c->irq;

	da9062_regmap_config.rd_table = &da9062_aa_readable_table;
	da9062_regmap_config.wr_table = &da9062_aa_writeable_table;
	da9062_regmap_config.volatile_table = &da9062_aa_volatile_table;

	chip->regmap = devm_regmap_init_i2c(i2c, &da9062_regmap_config);
	if (IS_ERR(chip->regmap)) {
		ret = PTR_ERR(chip->regmap);
		dev_err(chip->dev, "Failed to allocate register map: %d\n",
			ret);
		return ret;
	}

	return da9062_device_init(chip, i2c->irq);
}

static int da9062_i2c_remove(struct i2c_client *i2c)
{
	struct da9062 *chip = i2c_get_clientdata(i2c);

	da9062_device_exit(chip);

	return 0;
}

static const struct i2c_device_id da9062_i2c_id[] = {
	{"da9062", DA9062_PMIC_DEVICE_ID},
	{},
};
MODULE_DEVICE_TABLE(i2c, da9062_i2c_id);

static struct i2c_driver da9062_i2c_driver = {
	.driver = {
		.name = "da9062",
		.owner = THIS_MODULE,
                .of_match_table = of_match_ptr(da9062_dt_ids),
	},
	.probe    = da9062_i2c_probe,
	.remove   = da9062_i2c_remove,
	.id_table = da9062_i2c_id,
};

module_i2c_driver(da9062_i2c_driver);

MODULE_DESCRIPTION("I2C device driver for Dialog DA9062");
MODULE_AUTHOR("S Twiss <stwiss.opensource@diasemi.com>");
MODULE_LICENSE("GPL v2");
