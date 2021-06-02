/* da9062-regulator.c - REGULATOR device driver for DA9062
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
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include <linux/mfd/da9062/core.h>
#include <linux/mfd/da9062/registers.h>


/* Regulator IDs */
enum {
	DA9062_ID_BUCK1,
	DA9062_ID_BUCK2,
	DA9062_ID_BUCK3,
	DA9062_ID_BUCK4,
	DA9062_ID_LDO1,
	DA9062_ID_LDO2,
	DA9062_ID_LDO3,
	DA9062_ID_LDO4,
};

/* Compatibility API -- this matches the API used in future kernel
 * and makes forward rebases easier. This API can then be devolved
 * back into the kernel's core functions
 */
struct da9062_reg_field {
	unsigned int reg;
	unsigned int lsb;
	unsigned int msb;
	unsigned int id_size;
	unsigned int id_offset;
};

#define DA9062_REG_FIELD(_reg, _lsb, _msb) {	\
				.reg = _reg,	\
				.lsb = _lsb,	\
				.msb = _msb,	\
				}

struct da9062_regmap_field {
	struct regmap *regmap;
	unsigned int mask;
	unsigned int shift;
	unsigned int reg;
};

static void da9062_regmap_field_init(struct da9062_regmap_field *rm_field,
	struct regmap *regmap, struct da9062_reg_field reg_field)
{
	int field_bits = reg_field.msb - reg_field.lsb + 1;
	rm_field->regmap = regmap;
	rm_field->reg = reg_field.reg;
	rm_field->shift = reg_field.lsb;
	rm_field->mask = ((BIT(field_bits) - 1) << reg_field.lsb);
}

struct da9062_regmap_field *devm_da9062_regmap_field_alloc(struct device *dev,
		struct regmap *regmap, struct da9062_reg_field reg_field)
{
	struct da9062_regmap_field *rm_field = devm_kzalloc(dev,
					sizeof(*rm_field), GFP_KERNEL);
	if (!rm_field)
		return ERR_PTR(-ENOMEM);

	da9062_regmap_field_init(rm_field, regmap, reg_field);

	return rm_field;

}

void devm_da9062_regmap_field_free(struct device *dev,
	struct da9062_regmap_field *field)
{
	devm_kfree(dev, field);
}

int da9062_regmap_field_write(struct da9062_regmap_field *field,
			      unsigned int val)
{
	return regmap_update_bits(field->regmap, field->reg,
				field->mask, val << field->shift);
}

int da9062_regmap_field_read(struct da9062_regmap_field *field,
			     unsigned int *val)
{
	int ret;
	unsigned int reg_val;
	
	ret = regmap_read(field->regmap, field->reg, &reg_val);
	if (ret != 0)
		return ret;
	reg_val &= field->mask;
	reg_val >>= field->shift;
	*val = reg_val;

	return ret;
}
/* End of compatibility API */


/* Regulator capabilities and registers description */
struct da9062_regulator_info {
	struct regulator_desc desc;
	/* Current limiting */
	unsigned int n_current_limits;
	const int *current_limits;
	/* Main register fields */
	struct da9062_reg_field mode;
	struct da9062_reg_field suspend;
	struct da9062_reg_field sleep;
	struct da9062_reg_field suspend_sleep;
	unsigned int suspend_vsel_reg;
	struct da9062_reg_field ilimit;
	/* Event detection bit */
	struct da9062_reg_field oc_event;
};

/* Single regulator settings */
struct da9062_regulator {
	struct regulator_desc			desc;
	struct regulator_dev			*rdev;
	struct da9062				*hw;
	const struct da9062_regulator_info	*info;

	struct da9062_regmap_field		*mode;
	struct da9062_regmap_field		*suspend;
	struct da9062_regmap_field		*sleep;
	struct da9062_regmap_field		*suspend_sleep;
	struct da9062_regmap_field		*ilimit;
};

/* Encapsulates all information for the regulators driver */
struct da9062_regulators {
	int					irq_ldo_lim;
	int					irq_vdd_warn;
	unsigned				n_regulators;
	/* Array size to be defined during init. Keep at end. */
	struct da9062_regulator			regulator[0];
};

/* BUCK modes */
enum {
	BUCK_MODE_MANUAL,	/* 0 */
	BUCK_MODE_SLEEP,	/* 1 */
	BUCK_MODE_SYNC,		/* 2 */
	BUCK_MODE_AUTO		/* 3 */
};

/* Regulator operations */

/* Current limits array (in uA) BUCK1 and BUCK3.
   Entry indexes corresponds to register values. */
static const int da9062_buck_a_limits[] = {
	 500000,  600000,  700000,  800000,  900000, 1000000, 1100000, 1200000,
	1300000, 1400000, 1500000, 1600000, 1700000, 1800000, 1900000, 2000000
};

/* Current limits array (in uA) for BUCK2.
   Entry indexes corresponds to register values. */
static const int da9062_buck_b_limits[] = {
	1500000, 1600000, 1700000, 1800000, 1900000, 2000000, 2100000, 2200000,
	2300000, 2400000, 2500000, 2600000, 2700000, 2800000, 2900000, 3000000
};

static int da9062_set_current_limit(struct regulator_dev *rdev,
				    int min_ua, int max_ua)
{
	struct da9062_regulator *regl = rdev_get_drvdata(rdev);
	const struct da9062_regulator_info *rinfo = regl->info;
	int n, tval;

	for (n = 0; n < rinfo->n_current_limits; n++) {
		tval = rinfo->current_limits[n];
		if (tval >= min_ua && tval <= max_ua)
			return da9062_regmap_field_write(regl->ilimit, n);
	}

	return -EINVAL;
}

static int da9062_get_current_limit(struct regulator_dev *rdev)
{
	struct da9062_regulator *regl = rdev_get_drvdata(rdev);
	const struct da9062_regulator_info *rinfo = regl->info;
#if !defined(CONFIG_TCC_CODESONAR_BLOCKED)
	unsigned int sel = 0;
#else
	unsigned int sel;
#endif
	int ret;

	ret = da9062_regmap_field_read(regl->ilimit, &sel);
	if (ret < 0)
		return ret;

	if (sel >= rinfo->n_current_limits)
		sel = rinfo->n_current_limits - 1;

	return rinfo->current_limits[sel];
}

static int da9062_buck_set_mode(struct regulator_dev *rdev, unsigned mode)
{
	struct da9062_regulator *regl = rdev_get_drvdata(rdev);
	unsigned val;

	switch (mode) {
	case REGULATOR_MODE_FAST:
		val = BUCK_MODE_SYNC;
		break;
	case REGULATOR_MODE_NORMAL:
		val = BUCK_MODE_AUTO;
		break;
	case REGULATOR_MODE_STANDBY:
		val = BUCK_MODE_SLEEP;
		break;
	default:
		return -EINVAL;
	}

	return da9062_regmap_field_write(regl->mode, val);
}

/*
 * Bucks use single mode register field for normal operation
 * and suspend state.
 * There are 3 modes to map to: FAST, NORMAL, and STANDBY.
 */

static unsigned da9062_buck_get_mode(struct regulator_dev *rdev)
{
	struct da9062_regulator *regl = rdev_get_drvdata(rdev);
	struct da9062_regmap_field *field;
#if !defined(CONFIG_TCC_CODESONAR_BLOCKED)
	unsigned int val = 0, mode = 0;
#else
	unsigned int val, mode = 0;
#endif
	int ret;

	ret = da9062_regmap_field_read(regl->mode, &val);
#if !defined(CONFIG_TCC_CODESONAR_BLOCKED)
	if (ret < 0)
		return 0;
#else
	if (ret < 0)
		return ret;
#endif

	switch (val) {
	default:
	case BUCK_MODE_MANUAL:
		mode = REGULATOR_MODE_FAST | REGULATOR_MODE_STANDBY;
		/* Sleep flag bit decides the mode */
		break;
	case BUCK_MODE_SLEEP:
		return REGULATOR_MODE_STANDBY;
	case BUCK_MODE_SYNC:
		return REGULATOR_MODE_FAST;
	case BUCK_MODE_AUTO:
		return REGULATOR_MODE_NORMAL;
	}

	/* Detect current regulator state */
	ret = da9062_regmap_field_read(regl->suspend, &val);
	if (ret < 0)
		return 0;

	/* Read regulator mode from proper register, depending on state */
	if (val)
		field = regl->suspend_sleep;
	else
		field = regl->sleep;

	ret = da9062_regmap_field_read(field, &val);
	if (ret < 0)
		return 0;

	if (val)
		mode &= REGULATOR_MODE_STANDBY;
	else
		mode &= REGULATOR_MODE_NORMAL | REGULATOR_MODE_FAST;

	return mode;
}

/*
 * LDOs use sleep flags - one for normal and one for suspend state.
 * There are 2 modes to map to: NORMAL and STANDBY (sleep) for each state.
 */

static int da9062_ldo_set_mode(struct regulator_dev *rdev, unsigned mode)
{
	struct da9062_regulator *regl = rdev_get_drvdata(rdev);
	unsigned val;

	switch (mode) {
	case REGULATOR_MODE_NORMAL:
		val = 0;
		break;
	case REGULATOR_MODE_STANDBY:
		val = 1;
		break;
	default:
		return -EINVAL;
	}

	return da9062_regmap_field_write(regl->sleep, val);
}

static unsigned da9062_ldo_get_mode(struct regulator_dev *rdev)
{
	struct da9062_regulator *regl = rdev_get_drvdata(rdev);
	struct da9062_regmap_field *field;
#if !defined(CONFIG_TCC_CODESONAR_BLOCKED)
	int ret, val = 0;
#else
	int ret, val;
#endif

	/* Detect current regulator state */
	ret = da9062_regmap_field_read(regl->suspend, &val);
	if (ret < 0)
		return 0;

	/* Read regulator mode from proper register, depending on state */
	if (val)
		field = regl->suspend_sleep;
	else
		field = regl->sleep;

	ret = da9062_regmap_field_read(field, &val);
	if (ret < 0)
		return 0;

	if (val)
		return REGULATOR_MODE_STANDBY;
	else
		return REGULATOR_MODE_NORMAL;
}

static int da9062_buck_get_status(struct regulator_dev *rdev)
{
	int ret = regulator_is_enabled_regmap(rdev);

	if (ret == 0) {
		ret = REGULATOR_STATUS_OFF;
	} else if (ret > 0) {
		ret = da9062_buck_get_mode(rdev);
		if (ret > 0)
			ret = regulator_mode_to_status(ret);
		else if (ret == 0)
			ret = -EIO;
	}

	return ret;
}

static int da9062_ldo_get_status(struct regulator_dev *rdev)
{
	int ret = regulator_is_enabled_regmap(rdev);

	if (ret == 0) {
		ret = REGULATOR_STATUS_OFF;
	} else if (ret > 0) {
		ret = da9062_ldo_get_mode(rdev);
		if (ret > 0)
			ret = regulator_mode_to_status(ret);
		else if (ret == 0)
			ret = -EIO;
	}

	return ret;
}

static int da9062_set_suspend_voltage(struct regulator_dev *rdev, int uv)
{
	struct da9062_regulator *regl = rdev_get_drvdata(rdev);
	const struct da9062_regulator_info *rinfo = regl->info;
	int ret, sel;

	sel = regulator_map_voltage_linear(rdev, uv, uv);
	if (sel < 0)
		return sel;

	sel <<= ffs(rdev->desc->vsel_mask) - 1;

	ret = regmap_update_bits(regl->hw->regmap, rinfo->suspend_vsel_reg,
				 rdev->desc->vsel_mask, sel);

	return ret;
}

static int da9062_suspend_enable(struct regulator_dev *rdev)
{
	struct da9062_regulator *regl = rdev_get_drvdata(rdev);

	return da9062_regmap_field_write(regl->suspend, 1);
}

static int da9062_suspend_disable(struct regulator_dev *rdev)
{
	struct da9062_regulator *regl = rdev_get_drvdata(rdev);

	return da9062_regmap_field_write(regl->suspend, 0);
}

static int da9062_buck_set_suspend_mode(struct regulator_dev *rdev,
					unsigned mode)
{
	struct da9062_regulator *regl = rdev_get_drvdata(rdev);
	int val;

	switch (mode) {
	case REGULATOR_MODE_FAST:
		val = BUCK_MODE_SYNC;
		break;
	case REGULATOR_MODE_NORMAL:
		val = BUCK_MODE_AUTO;
		break;
	case REGULATOR_MODE_STANDBY:
		val = BUCK_MODE_SLEEP;
		break;
	default:
		return -EINVAL;
	}

	return da9062_regmap_field_write(regl->mode, val);
}

static int da9062_ldo_set_suspend_mode(struct regulator_dev *rdev,
						unsigned mode)
{
	struct da9062_regulator *regl = rdev_get_drvdata(rdev);
	unsigned val;

	switch (mode) {
	case REGULATOR_MODE_NORMAL:
		val = 0;
		break;
	case REGULATOR_MODE_STANDBY:
		val = 1;
		break;
	default:
		return -EINVAL;
	}

	return da9062_regmap_field_write(regl->suspend_sleep, val);
}

static struct regulator_ops da9062_buck_ops = {
	.enable			= regulator_enable_regmap,
	.disable		= regulator_disable_regmap,
	.is_enabled		= regulator_is_enabled_regmap,
	.get_voltage_sel	= regulator_get_voltage_sel_regmap,
	.set_voltage_sel	= regulator_set_voltage_sel_regmap,
	.list_voltage		= regulator_list_voltage_linear,
	.set_current_limit	= da9062_set_current_limit,
	.get_current_limit	= da9062_get_current_limit,
	.set_mode		= da9062_buck_set_mode,
	.get_mode		= da9062_buck_get_mode,
	.get_status		= da9062_buck_get_status,
	.set_suspend_voltage	= da9062_set_suspend_voltage,
	.set_suspend_enable	= da9062_suspend_enable,
	.set_suspend_disable	= da9062_suspend_disable,
	.set_suspend_mode	= da9062_buck_set_suspend_mode,
};

static struct regulator_ops da9062_ldo_ops = {
	.enable			= regulator_enable_regmap,
	.disable		= regulator_disable_regmap,
	.is_enabled		= regulator_is_enabled_regmap,
	.get_voltage_sel	= regulator_get_voltage_sel_regmap,
	.set_voltage_sel	= regulator_set_voltage_sel_regmap,
	.list_voltage		= regulator_list_voltage_linear,
	.set_mode		= da9062_ldo_set_mode,
	.get_mode		= da9062_ldo_get_mode,
	.get_status		= da9062_ldo_get_status,
	.set_suspend_voltage	= da9062_set_suspend_voltage,
	.set_suspend_enable	= da9062_suspend_enable,
	.set_suspend_disable	= da9062_suspend_disable,
	.set_suspend_mode	= da9062_ldo_set_suspend_mode,
};

/* Regulator information */
static const struct da9062_regulator_info local_regulator_info[] = {
	{
		.desc.id = DA9062_ID_BUCK1,
		.desc.name = "DA9062 BUCK1",
		.desc.ops = &da9062_buck_ops,
		.desc.min_uV = (300) * 1000,
		.desc.uV_step = (10) * 1000,
		.desc.n_voltages = ((1570) - (300))/(10) + 1,
		.current_limits = da9062_buck_a_limits,
		.n_current_limits = ARRAY_SIZE(da9062_buck_a_limits),
		.desc.enable_reg = DA9062AA_BUCK1_CONT,
		.desc.enable_mask = DA9062AA_BUCK1_EN_MASK,
		.desc.vsel_reg = DA9062AA_VBUCK1_A,
		.desc.vsel_mask = DA9062AA_VBUCK1_A_MASK,
		.desc.linear_min_sel = 0,
		.sleep = DA9062_REG_FIELD(DA9062AA_VBUCK1_A,
			__builtin_ffs((int)DA9062AA_BUCK1_SL_A_MASK) - 1,
			sizeof(unsigned int) * 8 -
			__builtin_clz((DA9062AA_BUCK1_SL_A_MASK)) - 1),
		.suspend_sleep = DA9062_REG_FIELD(DA9062AA_VBUCK1_B,
			__builtin_ffs((int)DA9062AA_BUCK1_SL_B_MASK) - 1,
			sizeof(unsigned int) * 8 -
			__builtin_clz((DA9062AA_BUCK1_SL_B_MASK)) - 1),
		.suspend_vsel_reg = DA9062AA_VBUCK1_B,
		.mode = DA9062_REG_FIELD(DA9062AA_BUCK1_CFG,
			__builtin_ffs((int)DA9062AA_BUCK1_MODE_MASK) - 1,
			sizeof(unsigned int) * 8 -
			__builtin_clz((DA9062AA_BUCK1_MODE_MASK)) - 1),
		.suspend = DA9062_REG_FIELD(DA9062AA_DVC_1,
			__builtin_ffs((int)DA9062AA_VBUCK1_SEL_MASK) - 1,
			sizeof(unsigned int) * 8 -
			__builtin_clz((DA9062AA_VBUCK1_SEL_MASK)) - 1),
		.ilimit = DA9062_REG_FIELD(DA9062AA_BUCK_ILIM_C,
			__builtin_ffs((int)DA9062AA_BUCK1_ILIM_MASK) - 1,
			sizeof(unsigned int) * 8 -
			__builtin_clz((DA9062AA_BUCK1_ILIM_MASK)) - 1),
	},
	{
		.desc.id = DA9062_ID_BUCK2,
		.desc.name = "DA9062 BUCK2",
		.desc.ops = &da9062_buck_ops,
		.desc.min_uV = (300) * 1000,
		.desc.uV_step = (10) * 1000,
		.desc.n_voltages = ((1570) - (300))/(10) + 1,
		.current_limits = da9062_buck_a_limits,
		.n_current_limits = ARRAY_SIZE(da9062_buck_a_limits),
		.desc.enable_reg = DA9062AA_BUCK2_CONT,
		.desc.enable_mask = DA9062AA_BUCK2_EN_MASK,
		.desc.vsel_reg = DA9062AA_VBUCK2_A,
		.desc.vsel_mask = DA9062AA_VBUCK2_A_MASK,
		.desc.linear_min_sel = 0,
		.sleep = DA9062_REG_FIELD(DA9062AA_VBUCK2_A,
			__builtin_ffs((int)DA9062AA_BUCK2_SL_A_MASK) - 1,
			sizeof(unsigned int) * 8 -
			__builtin_clz((DA9062AA_BUCK2_SL_A_MASK)) - 1),
		.suspend_sleep = DA9062_REG_FIELD(DA9062AA_VBUCK2_B,
			__builtin_ffs((int)DA9062AA_BUCK2_SL_B_MASK) - 1,
			sizeof(unsigned int) * 8 -
			__builtin_clz((DA9062AA_BUCK2_SL_B_MASK)) - 1),
		.suspend_vsel_reg = DA9062AA_VBUCK2_B,
		.mode = DA9062_REG_FIELD(DA9062AA_BUCK2_CFG,
			__builtin_ffs((int)DA9062AA_BUCK2_MODE_MASK) - 1,
			sizeof(unsigned int) * 8 -
			__builtin_clz((DA9062AA_BUCK2_MODE_MASK)) - 1),
		.suspend = DA9062_REG_FIELD(DA9062AA_DVC_1,
			__builtin_ffs((int)DA9062AA_VBUCK2_SEL_MASK) - 1,
			sizeof(unsigned int) * 8 -
			__builtin_clz((DA9062AA_VBUCK2_SEL_MASK)) - 1),
		.ilimit = DA9062_REG_FIELD(DA9062AA_BUCK_ILIM_C,
			__builtin_ffs((int)DA9062AA_BUCK2_ILIM_MASK) - 1,
			sizeof(unsigned int) * 8 -
			__builtin_clz((DA9062AA_BUCK2_ILIM_MASK)) - 1),
	},
	{
		.desc.id = DA9062_ID_BUCK3,
		.desc.name = "DA9062 BUCK3",
		.desc.ops = &da9062_buck_ops,
		.desc.min_uV = (800) * 1000,
		.desc.uV_step = (20) * 1000,
		.desc.n_voltages = ((3340) - (800))/(20) + 1,
		.current_limits = da9062_buck_b_limits,
		.n_current_limits = ARRAY_SIZE(da9062_buck_b_limits),
		.desc.enable_reg = DA9062AA_BUCK3_CONT,
		.desc.enable_mask = DA9062AA_BUCK3_EN_MASK,
		.desc.vsel_reg = DA9062AA_VBUCK3_A,
		.desc.vsel_mask = DA9062AA_VBUCK3_A_MASK,
		.desc.linear_min_sel = 0,
		.sleep = DA9062_REG_FIELD(DA9062AA_VBUCK3_A,
			__builtin_ffs((int)DA9062AA_BUCK3_SL_A_MASK) - 1,
			sizeof(unsigned int) * 8 -
			__builtin_clz((DA9062AA_BUCK3_SL_A_MASK)) - 1),
		.suspend_sleep = DA9062_REG_FIELD(DA9062AA_VBUCK3_B,
			__builtin_ffs((int)DA9062AA_BUCK3_SL_B_MASK) - 1,
			sizeof(unsigned int) * 8 -
			__builtin_clz((DA9062AA_BUCK3_SL_B_MASK)) - 1),
		.suspend_vsel_reg = DA9062AA_VBUCK3_B,
		.mode = DA9062_REG_FIELD(DA9062AA_BUCK3_CFG,
			__builtin_ffs((int)DA9062AA_BUCK3_MODE_MASK) - 1,
			sizeof(unsigned int) * 8 -
			__builtin_clz((DA9062AA_BUCK3_MODE_MASK)) - 1),
		.suspend = DA9062_REG_FIELD(DA9062AA_DVC_1,
			__builtin_ffs((int)DA9062AA_VBUCK3_SEL_MASK) - 1,
			sizeof(unsigned int) * 8 -
			__builtin_clz((DA9062AA_VBUCK3_SEL_MASK)) - 1),
		.ilimit = DA9062_REG_FIELD(DA9062AA_BUCK_ILIM_A,
			__builtin_ffs((int)DA9062AA_BUCK3_ILIM_MASK) - 1,
			sizeof(unsigned int) * 8 -
			__builtin_clz((DA9062AA_BUCK3_ILIM_MASK)) - 1),
	},
	{
		.desc.id = DA9062_ID_BUCK4,
		.desc.name = "DA9062 BUCK4",
		.desc.ops = &da9062_buck_ops,
		.desc.min_uV = (530) * 1000,
		.desc.uV_step = (10) * 1000,
		.desc.n_voltages = ((1800) - (530))/(10) + 1,
		.current_limits = da9062_buck_a_limits,
		.n_current_limits = ARRAY_SIZE(da9062_buck_a_limits),
		.desc.enable_reg = DA9062AA_BUCK4_CONT,
		.desc.enable_mask = DA9062AA_BUCK4_EN_MASK,
		.desc.vsel_reg = DA9062AA_VBUCK4_A,
		.desc.vsel_mask = DA9062AA_VBUCK4_A_MASK,
		.desc.linear_min_sel = 0,
		.sleep = DA9062_REG_FIELD(DA9062AA_VBUCK4_A,
			__builtin_ffs((int)DA9062AA_BUCK4_SL_A_MASK) - 1,
			sizeof(unsigned int) * 8 -
			__builtin_clz((DA9062AA_BUCK4_SL_A_MASK)) - 1),
		.suspend_sleep = DA9062_REG_FIELD(DA9062AA_VBUCK4_B,
			__builtin_ffs((int)DA9062AA_BUCK4_SL_B_MASK) - 1,
			sizeof(unsigned int) * 8 -
			__builtin_clz((DA9062AA_BUCK4_SL_B_MASK)) - 1),
		.suspend_vsel_reg = DA9062AA_VBUCK4_B,
		.mode = DA9062_REG_FIELD(DA9062AA_BUCK4_CFG,
			__builtin_ffs((int)DA9062AA_BUCK4_MODE_MASK) - 1,
			sizeof(unsigned int) * 8 -
			__builtin_clz((DA9062AA_BUCK4_MODE_MASK)) - 1),
		.suspend = DA9062_REG_FIELD(DA9062AA_DVC_1,
			__builtin_ffs((int)DA9062AA_VBUCK4_SEL_MASK) - 1,
			sizeof(unsigned int) * 8 -
			__builtin_clz((DA9062AA_VBUCK4_SEL_MASK)) - 1),
		.ilimit = DA9062_REG_FIELD(DA9062AA_BUCK_ILIM_B,
			__builtin_ffs((int)DA9062AA_BUCK4_ILIM_MASK) - 1,
			sizeof(unsigned int) * 8 -
			__builtin_clz((DA9062AA_BUCK4_ILIM_MASK)) - 1),
	},
	{
		.desc.id = DA9062_ID_LDO1,
		.desc.name = "DA9062 LDO1",
		.desc.ops = &da9062_ldo_ops,
		.desc.min_uV = (900) * 1000,
		.desc.uV_step = (50) * 1000,
		.desc.n_voltages = ((3600) - (900))/(50) + 1,
		.desc.enable_reg = DA9062AA_LDO1_CONT,
		.desc.enable_mask = DA9062AA_LDO1_EN_MASK,
		.desc.vsel_reg = DA9062AA_VLDO1_A,
		.desc.vsel_mask = DA9062AA_VLDO1_A_MASK,
		.desc.linear_min_sel = 0,
		.sleep = DA9062_REG_FIELD(DA9062AA_VLDO1_A,
			__builtin_ffs((int)DA9062AA_LDO1_SL_A_MASK) - 1,
			sizeof(unsigned int) * 8 -
			__builtin_clz((DA9062AA_LDO1_SL_A_MASK)) - 1),
		.suspend_sleep = DA9062_REG_FIELD(DA9062AA_VLDO1_B,
			__builtin_ffs((int)DA9062AA_LDO1_SL_B_MASK) - 1,
			sizeof(unsigned int) * 8 -
			__builtin_clz((DA9062AA_LDO1_SL_B_MASK)) - 1),
		.suspend_vsel_reg = DA9062AA_VLDO1_B,
		.suspend = DA9062_REG_FIELD(DA9062AA_DVC_1,
			__builtin_ffs((int)DA9062AA_VLDO1_SEL_MASK) - 1,
			sizeof(unsigned int) * 8 -
			__builtin_clz((DA9062AA_VLDO1_SEL_MASK)) - 1),
		.oc_event = DA9062_REG_FIELD(DA9062AA_STATUS_D,
			__builtin_ffs((int)DA9062AA_LDO1_ILIM_MASK) - 1,
			sizeof(unsigned int) * 8 -
			__builtin_clz((DA9062AA_LDO1_ILIM_MASK)) - 1),
	},
	{
		.desc.id = DA9062_ID_LDO2,
		.desc.name = "DA9062 LDO2",
		.desc.ops = &da9062_ldo_ops,
		.desc.min_uV = (900) * 1000,
		.desc.uV_step = (50) * 1000,
		.desc.n_voltages = ((3600) - (600))/(50) + 1,
		.desc.enable_reg = DA9062AA_LDO2_CONT,
		.desc.enable_mask = DA9062AA_LDO2_EN_MASK,
		.desc.vsel_reg = DA9062AA_VLDO2_A,
		.desc.vsel_mask = DA9062AA_VLDO2_A_MASK,
		.desc.linear_min_sel = 0,
		.sleep = DA9062_REG_FIELD(DA9062AA_VLDO2_A,
			__builtin_ffs((int)DA9062AA_LDO2_SL_A_MASK) - 1,
			sizeof(unsigned int) * 8 -
			__builtin_clz((DA9062AA_LDO2_SL_A_MASK)) - 1),
		.suspend_sleep = DA9062_REG_FIELD(DA9062AA_VLDO2_B,
			__builtin_ffs((int)DA9062AA_LDO2_SL_B_MASK) - 1,
			sizeof(unsigned int) * 8 -
			__builtin_clz((DA9062AA_LDO2_SL_B_MASK)) - 1),
		.suspend_vsel_reg = DA9062AA_VLDO2_B,
		.suspend = DA9062_REG_FIELD(DA9062AA_DVC_1,
			__builtin_ffs((int)DA9062AA_VLDO2_SEL_MASK) - 1,
			sizeof(unsigned int) * 8 -
			__builtin_clz((DA9062AA_VLDO2_SEL_MASK)) - 1),
		.oc_event = DA9062_REG_FIELD(DA9062AA_STATUS_D,
			__builtin_ffs((int)DA9062AA_LDO2_ILIM_MASK) - 1,
			sizeof(unsigned int) * 8 -
			__builtin_clz((DA9062AA_LDO2_ILIM_MASK)) - 1),
	},
	{
		.desc.id = DA9062_ID_LDO3,
		.desc.name = "DA9062 LDO3",
		.desc.ops = &da9062_ldo_ops,
		.desc.min_uV = (900) * 1000,
		.desc.uV_step = (50) * 1000,
		.desc.n_voltages = ((3600) - (900))/(50) + 1,
		.desc.enable_reg = DA9062AA_LDO3_CONT,
		.desc.enable_mask = DA9062AA_LDO3_EN_MASK,
		.desc.vsel_reg = DA9062AA_VLDO3_A,
		.desc.vsel_mask = DA9062AA_VLDO3_A_MASK,
		.desc.linear_min_sel = 0,
		.sleep = DA9062_REG_FIELD(DA9062AA_VLDO3_A,
			__builtin_ffs((int)DA9062AA_LDO3_SL_A_MASK) - 1,
			sizeof(unsigned int) * 8 -
			__builtin_clz((DA9062AA_LDO3_SL_A_MASK)) - 1),
		.suspend_sleep = DA9062_REG_FIELD(DA9062AA_VLDO3_B,
			__builtin_ffs((int)DA9062AA_LDO3_SL_B_MASK) - 1,
			sizeof(unsigned int) * 8 -
			__builtin_clz((DA9062AA_LDO3_SL_B_MASK)) - 1),
		.suspend_vsel_reg = DA9062AA_VLDO3_B,
		.suspend = DA9062_REG_FIELD(DA9062AA_DVC_1,
			__builtin_ffs((int)DA9062AA_VLDO3_SEL_MASK) - 1,
			sizeof(unsigned int) * 8 -
			__builtin_clz((DA9062AA_VLDO3_SEL_MASK)) - 1),
		.oc_event = DA9062_REG_FIELD(DA9062AA_STATUS_D,
			__builtin_ffs((int)DA9062AA_LDO3_ILIM_MASK) - 1,
			sizeof(unsigned int) * 8 -
			__builtin_clz((DA9062AA_LDO3_ILIM_MASK)) - 1),
	},
	{
		.desc.id = DA9062_ID_LDO4,
		.desc.name = "DA9062 LDO4",
		.desc.ops = &da9062_ldo_ops,
		.desc.min_uV = (900) * 1000,
		.desc.uV_step = (50) * 1000,
		.desc.n_voltages = ((3600) - (900))/(50) + 1,
		.desc.enable_reg = DA9062AA_LDO4_CONT,
		.desc.enable_mask = DA9062AA_LDO4_EN_MASK,
		.desc.vsel_reg = DA9062AA_VLDO4_A,
		.desc.vsel_mask = DA9062AA_VLDO4_A_MASK,
		.desc.linear_min_sel = 0,
		.sleep = DA9062_REG_FIELD(DA9062AA_VLDO4_A,
			__builtin_ffs((int)DA9062AA_LDO4_SL_A_MASK) - 1,
			sizeof(unsigned int) * 8 -
			__builtin_clz((DA9062AA_LDO4_SL_A_MASK)) - 1),
		.suspend_sleep = DA9062_REG_FIELD(DA9062AA_VLDO4_B,
			__builtin_ffs((int)DA9062AA_LDO4_SL_B_MASK) - 1,
			sizeof(unsigned int) * 8 -
			__builtin_clz((DA9062AA_LDO4_SL_B_MASK)) - 1),
		.suspend_vsel_reg = DA9062AA_VLDO4_B,
		.suspend = DA9062_REG_FIELD(DA9062AA_DVC_1,
			__builtin_ffs((int)DA9062AA_VLDO4_SEL_MASK) - 1,
			sizeof(unsigned int) * 8 -
			__builtin_clz((DA9062AA_VLDO4_SEL_MASK)) - 1),
		.oc_event = DA9062_REG_FIELD(DA9062AA_STATUS_D,
			__builtin_ffs((int)DA9062AA_LDO4_ILIM_MASK) - 1,
			sizeof(unsigned int) * 8 -
			__builtin_clz((DA9062AA_LDO4_ILIM_MASK)) - 1),
	},
};

/* Regulator interrupt handlers */
/*static irqreturn_t da9062_ldo_lim_event(int irq, void *data)
{
	struct da9062_regulators *regulators = data;
	struct da9062 *hw = regulators->regulator[0].hw;
	struct da9062_regulator *regl;
	int bits, i , ret;

	ret = regmap_read(hw->regmap, DA9062AA_STATUS_D, &bits);
	if (ret < 0)
		return IRQ_NONE;

	for (i = regulators->n_regulators - 1; i >= 0; i--) {
		regl = &regulators->regulator[i];
		if (regl->info->oc_event.reg != DA9062AA_STATUS_D)
			continue;

		if (BIT(regl->info->oc_event.lsb) & bits)
			regulator_notifier_call_chain(regl->rdev,
					REGULATOR_EVENT_OVER_CURRENT, NULL);
	}

	return IRQ_HANDLED;
}

static irqreturn_t da9062_vdd_warn_event(int irq, void *data)
{
	return IRQ_HANDLED;
}*/
/* Regulators platform data */
struct da9062_regulator_data {
	int				id;
	struct regulator_init_data	*initdata;
};

struct da9062_regulators_pdata {
	unsigned			n_regulators;
	struct da9062_regulator_data	*regulator_data;
};

/*
 * Probing and Initialisation functions
 */
static const struct regulator_init_data *da9062_get_regulator_initdata(
		const struct da9062_regulators_pdata *pdata, int id)
{
	int i;

	for (i = 0; i < pdata->n_regulators; i++) {
		if (id == pdata->regulator_data[i].id)
			return pdata->regulator_data[i].initdata;
	}

	return NULL;
}

#ifdef CONFIG_OF
static struct of_regulator_match da9062_matches[] = {
	[DA9062_ID_BUCK1] = { .name = "buck1" },
	[DA9062_ID_BUCK2] = { .name = "buck2" },
	[DA9062_ID_BUCK3] = { .name = "buck3" },
	[DA9062_ID_BUCK4] = { .name = "buck4" },
	[DA9062_ID_LDO1]  = { .name = "ldo1"  },
	[DA9062_ID_LDO2]  = { .name = "ldo2"  },
	[DA9062_ID_LDO3]  = { .name = "ldo3"  },
	[DA9062_ID_LDO4]  = { .name = "ldo4"  },
};

static struct da9062_regulators_pdata *da9062_parse_regulators_dt(
		struct platform_device *pdev,
		struct of_regulator_match **reg_matches)
{
	struct da9062_regulators_pdata *pdata;
	struct da9062_regulator_data *rdata;
	struct device_node *node;
	int i, n, num;

	node = of_get_child_by_name(pdev->dev.parent->of_node, "regulators");
	if (!node) {
		dev_err(&pdev->dev, "Regulators device node not found\n");
		return ERR_PTR(-ENODEV);
	}

	num = of_regulator_match(&pdev->dev, node, da9062_matches,
				 ARRAY_SIZE(da9062_matches));
	of_node_put(node);
	if (num < 0) {
		dev_err(&pdev->dev, "Failed to match regulators\n");
		return ERR_PTR(-EINVAL);
	}

	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	pdata->regulator_data = devm_kzalloc(&pdev->dev,
					num * sizeof(*pdata->regulator_data),
					GFP_KERNEL);
	if (!pdata->regulator_data)
		return ERR_PTR(-ENOMEM);
	pdata->n_regulators = num;

	n = 0;
	for (i = 0; i < ARRAY_SIZE(da9062_matches); i++) {
		if (!da9062_matches[i].init_data)
			continue;

		rdata = &pdata->regulator_data[n];
		rdata->id = i;
		rdata->initdata = da9062_matches[i].init_data;

		n++;
	};

	*reg_matches = da9062_matches;
	return pdata;
}
#else
static struct da9062_regulators_pdata *da9062_parse_regulators_dt(
		struct platform_device *pdev,
		struct of_regulator_match **reg_matches)
{
	*reg_matches = NULL;
	return ERR_PTR(-ENODEV);
}
#endif

static int da9062_regulator_probe(struct platform_device *pdev)
{
	struct da9062 *chip = dev_get_drvdata(pdev->dev.parent);
	struct of_regulator_match *reg_matches = NULL;
	struct da9062_regulators_pdata *pdata;
	struct da9062_regulators *regulators;
	struct da9062_regulator *regl;
	struct regulator_config config;

	int id, n, n_regulators, ret;
	size_t size;

	pdata = da9062_parse_regulators_dt(pdev, &reg_matches);

	if (IS_ERR(pdata) || pdata->n_regulators == 0) {
		dev_err(&pdev->dev,
			"No regulators defined for the platform\n");
		return PTR_ERR(pdata);
	}

	n_regulators = ARRAY_SIZE(local_regulator_info),

	/* Allocate memory required by usable regulators */
	size = sizeof(struct da9062_regulators) +
		n_regulators * sizeof(struct da9062_regulator);
	regulators = devm_kzalloc(&pdev->dev, size, GFP_KERNEL);
	if (!regulators)
		return -ENOMEM;

	regulators->n_regulators = n_regulators;
	platform_set_drvdata(pdev, regulators);

	/* Register all regulators declared in platform information */
	n = 0;
	id = 0;
	while (n < regulators->n_regulators) {
		/* Initialise regulator structure */
		regl = &regulators->regulator[n];
		regl->hw = chip;
		regl->info = &local_regulator_info[id];
		regl->desc = regl->info->desc;
		regl->desc.type = REGULATOR_VOLTAGE;
		regl->desc.owner = THIS_MODULE;

		if (regl->info->mode.reg)
			regl->mode = devm_da9062_regmap_field_alloc(
					&pdev->dev,
					chip->regmap,
					regl->info->mode);


		if (regl->info->suspend.reg)
			regl->suspend = devm_da9062_regmap_field_alloc(
					&pdev->dev,
					chip->regmap,
					regl->info->suspend);
		if (regl->info->sleep.reg)
			regl->sleep = devm_da9062_regmap_field_alloc(
					&pdev->dev,
					chip->regmap,
					regl->info->sleep);
		if (regl->info->suspend_sleep.reg)
			regl->suspend_sleep = devm_da9062_regmap_field_alloc(
					&pdev->dev,
					chip->regmap,
					regl->info->suspend_sleep);
		if (regl->info->ilimit.reg)
			regl->ilimit = devm_da9062_regmap_field_alloc(
					&pdev->dev,
					chip->regmap,
					regl->info->ilimit);

		/* Register regulator */
		memset(&config, 0, sizeof(config));
		config.dev = &pdev->dev;
		config.init_data = da9062_get_regulator_initdata(pdata, id);
		config.driver_data = regl;
		if (reg_matches)
			config.of_node = reg_matches[id].of_node;
		config.regmap = chip->regmap;
		regl->rdev = regulator_register(&regl->desc,
						     &config);
		if (IS_ERR(regl->rdev)) {
			dev_err(&pdev->dev,
				"Failed to register %s regulator\n",
				regl->desc.name);
			ret = PTR_ERR(regl->rdev);
			goto err;
		}
		id++;
		n++;
	}

	/* LDOs overcurrent event support */
	/*irq = platform_get_irq_byname(pdev, "LDO_LIM");
	if (irq < 0) {
		dev_err(&pdev->dev, "Failed to get IRQ.\n");
		ret = irq;
		goto err;
	}
    printk("IRQ: %d\n", irq);
    irq = regmap_irq_get_virq(chip->regmap_irq, irq);
    printk("IRQ: %d\n", irq);
	regulators->irq_ldo_lim = irq;

	ret = request_threaded_irq(irq,
				   NULL, da9062_ldo_lim_event,
				   IRQF_TRIGGER_LOW | IRQF_ONESHOT,
				   "LDO_LIM", regulators);
	if (ret) {
		dev_err(&pdev->dev,
				"Failed to request LDO_LIM IRQ.\n");
		regulators->irq_ldo_lim = -ENXIO;
	}*/

	/* VDD WARN event support */
	/*irq = platform_get_irq_byname(pdev, "VDD_WARN");
	if (irq < 0) {
		dev_err(&pdev->dev, "Failed to get IRQ.\n");
		ret = irq;
		goto err;
	}
    printk("IRQ: %d\n", irq);
    irq = regmap_irq_get_virq(chip->regmap_irq, irq);
    printk("IRQ: %d\n", irq);
	regulators->irq_vdd_warn = irq;

	ret = request_threaded_irq(irq,
				   NULL, da9062_vdd_warn_event,
				   IRQF_TRIGGER_LOW | IRQF_ONESHOT,
				   "VDD_WARN", regulators);
	if (ret) {
		dev_err(&pdev->dev,
				"Failed to request VDD_WARN IRQ.\n");
		regulators->irq_vdd_warn = -ENXIO;
	}*/
	dev_info(&pdev->dev, "DA9062 regulator driver loaded\n");
	return 0;

err:
	/* Wind back regulators registeration */
	while (--n >= 0)
		regulator_unregister(regulators->regulator[n].rdev);

	// free_irq(regulators->irq_ldo_lim, regulators);

	return ret;
}

static int da9062_regulator_remove(struct platform_device *pdev)
{
/*	struct da9062_regulators *regulators = platform_get_drvdata(pdev);

	free_irq(regulators->irq_ldo_lim, regulators);
	free_irq(regulators->irq_vdd_warn, regulators);*/

	return 0;
}

static struct platform_driver da9062_regulator_driver = {
	.driver = {
		.name = "da9062-regulators",
		.owner = THIS_MODULE,
	},
	.probe = da9062_regulator_probe,
	.remove = da9062_regulator_remove,
};

static int da9062_regulator_init(void)
{
	return platform_driver_register(&da9062_regulator_driver);
}
subsys_initcall(da9062_regulator_init);

static void da9062_regulator_cleanup(void)
{
	platform_driver_unregister(&da9062_regulator_driver);
}
module_exit(da9062_regulator_cleanup);

/* Module information */
MODULE_AUTHOR("S Twiss <stwiss.opensource@diasemi.com>");
MODULE_DESCRIPTION("REGULATOR device driver for Dialog DA9062");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform: da9062-regulators");
