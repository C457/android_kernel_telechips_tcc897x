/* core.h - CORE H for DA9062
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

#ifndef __MFD_DA9062_CORE_H__
#define __MFD_DA9062_CORE_H__

#include <linux/interrupt.h>
#include <linux/mfd/da9062/registers.h>

/* Interrupts */
enum da9062_irqs {
	/* IRQ A */
	DA9062_IRQ_ONKEY,
	DA9062_IRQ_ALARM,
	DA9062_IRQ_TICK,
	DA9062_IRQ_WDG_WARN,
	DA9062_IRQ_SEQ_RDY,
	/* IRQ B*/
	DA9062_IRQ_TEMP,
	DA9062_IRQ_LDO_LIM,
	DA9062_IRQ_DVC_RDY,
	DA9062_IRQ_VDD_WARN,
	/* IRQ C */
	DA9062_IRQ_GPI0,
	DA9062_IRQ_GPI1,
	DA9062_IRQ_GPI2,
	DA9062_IRQ_GPI3,
	DA9062_IRQ_GPI4,

	DA9062_IRQ_LAST,
};

#define DA9062_NUM_IRQ DA9062_IRQ_LAST

struct da9062 {
	struct device *dev;
	unsigned char device_id;
	unsigned char variant_mrc;
	struct regmap *regmap;
	int chip_irq;
	unsigned int irq_base;
	struct regmap_irq_chip_data *regmap_irq;
};

int da9062_device_init(struct da9062*, unsigned int);
int da9062_irq_init(struct da9062*);

void da9062_device_exit(struct da9062*);
void da9062_irq_exit(struct da9062*);

#endif /* __MFD_DA9062_CORE_H__ */
