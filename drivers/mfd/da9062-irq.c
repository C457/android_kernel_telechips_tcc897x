/* da9062-irq.c - IRQ device driver for DA9062
 * Copyright (C) 2012  Dialog Semiconductor Ltd.
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
#include <linux/irq.h>
#include <linux/mfd/core.h>
#include <linux/interrupt.h>
#include <linux/regmap.h>
#include <linux/mfd/da9062/core.h>

#define	DA9062_REG_EVENT_A_OFFSET	0
#define	DA9062_REG_EVENT_B_OFFSET	1
#define	DA9062_REG_EVENT_C_OFFSET	2

static struct regmap_irq da9062_irqs[] = {
	/* EVENT A */
	[DA9062_IRQ_ONKEY] = {
		.reg_offset = DA9062_REG_EVENT_A_OFFSET,
		.mask = DA9062AA_M_NONKEY_MASK,
	},
	[DA9062_IRQ_ALARM] = {
		.reg_offset = DA9062_REG_EVENT_A_OFFSET,
		.mask = DA9062AA_M_ALARM_MASK,
	},
	[DA9062_IRQ_TICK] = {
		.reg_offset = DA9062_REG_EVENT_A_OFFSET,
		.mask = DA9062AA_M_TICK_MASK,
	},
	[DA9062_IRQ_WDG_WARN] = {
		.reg_offset = DA9062_REG_EVENT_A_OFFSET,
		.mask = DA9062AA_M_WDG_WARN_MASK,
	},
	[DA9062_IRQ_SEQ_RDY] = {
		.reg_offset = DA9062_REG_EVENT_A_OFFSET,
		.mask = DA9062AA_M_SEQ_RDY_MASK,
	},
	/* EVENT B */
	[DA9062_IRQ_TEMP] = {
		.reg_offset = DA9062_REG_EVENT_B_OFFSET,
		.mask = DA9062AA_M_TEMP_MASK,
	},
	[DA9062_IRQ_LDO_LIM] = {
		.reg_offset = DA9062_REG_EVENT_B_OFFSET,
		.mask = DA9062AA_M_LDO_LIM_MASK,
	},
	[DA9062_IRQ_DVC_RDY] = {
		.reg_offset = DA9062_REG_EVENT_B_OFFSET,
		.mask = DA9062AA_M_DVC_RDY_MASK,
	},
	[DA9062_IRQ_VDD_WARN] = {
		.reg_offset = DA9062_REG_EVENT_B_OFFSET,
		.mask = DA9062AA_M_VDD_WARN_MASK,
	},
	/* EVENT C */
	[DA9062_IRQ_GPI0] = {
		.reg_offset = DA9062_REG_EVENT_C_OFFSET,
		.mask = DA9062AA_M_GPI0_MASK,
	},
	[DA9062_IRQ_GPI1] = {
		.reg_offset = DA9062_REG_EVENT_C_OFFSET,
		.mask = DA9062AA_M_GPI1_MASK,
	},
	[DA9062_IRQ_GPI2] = {
		.reg_offset = DA9062_REG_EVENT_C_OFFSET,
		.mask = DA9062AA_M_GPI2_MASK,
	},
	[DA9062_IRQ_GPI3] = {
		.reg_offset = DA9062_REG_EVENT_C_OFFSET,
		.mask = DA9062AA_M_GPI3_MASK,
	},
	[DA9062_IRQ_GPI4] = {
		.reg_offset = DA9062_REG_EVENT_C_OFFSET,
		.mask = DA9062AA_M_GPI4_MASK,
	},
};

static struct regmap_irq_chip da9062_irq_chip = {
	/* IRQ */
	.name = "da9062-irq",
	.irqs = da9062_irqs,
	.num_irqs = DA9062_NUM_IRQ,
	/* STATUS and EVENT */
	.num_regs = 3,
	.status_base = DA9062AA_EVENT_A,
	.mask_base = DA9062AA_IRQ_MASK_A,
	.ack_base = DA9062AA_EVENT_A,
};

int da9062_irq_init(struct da9062 *chip)
{
	int ret;

	if (!chip->chip_irq) {
		dev_err(chip->dev, "No IRQ configured\n");
		return -EINVAL;
	}

	ret = regmap_add_irq_chip(chip->regmap, chip->chip_irq,
			IRQF_TRIGGER_LOW | IRQF_ONESHOT | IRQF_SHARED,
			chip->irq_base, &da9062_irq_chip,
			&chip->regmap_irq);
	if (ret) {
		dev_err(chip->dev, "Failed to request IRQ %d: %d\n",
			chip->chip_irq, ret);
		return ret;
	}

	return 0;
}

void da9062_irq_exit(struct da9062 *chip)
{
	regmap_del_irq_chip(chip->chip_irq, chip->regmap_irq);
}

MODULE_DESCRIPTION("IRQ device driver for Dialog DA9062");
MODULE_AUTHOR("S Twiss <stwiss.opensource@diasemi.com>");
MODULE_LICENSE("GPL v2");
