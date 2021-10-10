/****************************************************************************
 * clk.c
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

#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <linux/clocksource.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/clk/tcc.h>

#if defined(CONFIG_ARCH_TCC897X)
#include "clk-tcc897x.h"
#define ISOIP_TOP	1
#define ISOIP_DDI	0
#elif defined(CONFIG_ARCH_TCC898X)
#include "clk-tcc898x.h"
#define ISOIP_TOP	1
#define ISOIP_DDI	1
#else
#error
#endif

struct tcc_clk {
	struct clk_hw hw;
	int id;
};

static struct tcc_ckc_ops *ckc_ops = NULL;

#define to_tcc_clk(tcc) container_of(tcc, struct tcc_clk, hw)

#ifdef CONFIG_PLATFORM_AVN
/* do not disable PMU_PWDN & BUS clock & PMU IP ISOL */
#define IGNORE_CLK_DISABLE
#endif

static void tcc_clk_register(struct device_node *np, struct tcc_clks_type *clks,
		char *default_name, struct clk_ops *ops)
{
	struct clk_onecell_data *clk_data;
	struct clk_init_data init;
	struct tcc_clk *tcc;
	struct clk *clk;
	unsigned int flags;
	char clk_name[20];
	int cnt, data_cnt, ret;

	if (!clks)
		return;
	if (clks->clks_num <= 0)
		return;

	clk_data = kzalloc(sizeof(struct clk_onecell_data), GFP_KERNEL);
	if (!clk_data)
		panic("could not allocate clock provider context.\n");

	clk_data->clks = kcalloc(clks->clks_num, sizeof(struct clk*), GFP_KERNEL);
	if (!clk_data->clks)
		panic("could not allocate clock lookup table\n");

	clk_data->clk_num = clks->clks_num;

	for (cnt=0, data_cnt=0 ; cnt < clks->clks_num ; cnt++) {
		flags = clks->common_flags;
		clk_data->clks[cnt] = ERR_PTR(-ENOENT);
		memset(&init, 0x0, sizeof(struct clk_init_data));
		tcc = kzalloc(sizeof(struct tcc_clk), GFP_KERNEL);
		if (!tcc) {
#if !defined(CONFIG_TCC_CODESONAR_BLOCKED)
			kfree(clk_data->clks);
			kfree(clk_data);
#else
#endif
			pr_err("could not allocate tcc clk\n");
			return;
		}
		if (data_cnt < clks->data_num) {
			if (clks->data[data_cnt].idx == cnt) {
				init.name = clks->data[data_cnt].name;
				flags |= clks->data[data_cnt].flags;
				init.parent_names = clks->data[data_cnt].parent_name ? &(clks->data[data_cnt].parent_name) : &(clks->parent_name);
				init.num_parents = clks->data[data_cnt].parent_name ? 1 : 0;
				data_cnt++;
				goto skip_clks_default_set;
			}
		}
		init.parent_names = &(clks->parent_name);
		init.num_parents = clks->parent_name ? 1 : 0;
skip_clks_default_set:
		if (init.name == NULL) {
			sprintf(clk_name, "%s_%d", default_name, cnt);
			init.name = clk_name;
		}
		init.flags = init.num_parents ? flags : (flags|CLK_IS_ROOT);
#ifdef CONFIG_ARM64
		init.flags |= CLK_IGNORE_UNUSED;
#endif
		init.ops = ops;
		tcc->hw.init = &init;
		tcc->id = cnt;
		clk = clk_register(NULL, &tcc->hw);
		if (!IS_ERR_OR_NULL(clk)) {
			ret = clk_register_clkdev(clk, init.name, NULL);
			if (ret)
				pr_err("%s: clk register clkdev failed. ret:0x%x\n", init.name, ret);
			clk_prepare(clk);  /* should be check prepare function */

			clk_data->clks[cnt] = clk;
			continue;
		}
		pr_err("%s: clk register failed\n", init.name);
		kfree(tcc);
	}
#if !defined(CONFIG_TCC_CODESONAR_BLOCKED)
	ret = of_clk_add_provider(np, of_clk_src_onecell_get, clk_data);

	if (ret < 0)
		kfree(clk_data);
#else
	of_clk_add_provider(np, of_clk_src_onecell_get, clk_data);
#endif
}

void tcc_ckc_set_ops(struct tcc_ckc_ops *ops)
{
	ckc_ops = ops;
}


/* common function */
static long tcc_round_rate(struct clk_hw *hw, unsigned long rate, unsigned long *best_parent_rate)
{
	return rate;
}


static int tcc_clkctrl_enable(struct clk_hw *hw)
{
	struct tcc_clk *tcc = to_tcc_clk(hw);
	if (ckc_ops->ckc_pmu_pwdn)
		ckc_ops->ckc_pmu_pwdn(tcc->id, false);
	if (ckc_ops->ckc_swreset)
		ckc_ops->ckc_swreset(tcc->id, false);
	if (ckc_ops->ckc_clkctrl_enable)
		ckc_ops->ckc_clkctrl_enable(tcc->id);
	return 0;
}

static void tcc_clkctrl_disable(struct clk_hw *hw)
{
	struct tcc_clk *tcc = to_tcc_clk(hw);
	if (ckc_ops->ckc_clkctrl_disable)
		ckc_ops->ckc_clkctrl_disable(tcc->id);
	if (ckc_ops->ckc_swreset)
		ckc_ops->ckc_swreset(tcc->id, true);
	if (ckc_ops->ckc_pmu_pwdn)
		ckc_ops->ckc_pmu_pwdn(tcc->id, true);
}

static unsigned long tcc_clkctrl_recalc_rate(struct clk_hw *hw, unsigned long parent_rate)
{
	unsigned long rate = 0;
	struct tcc_clk *tcc = to_tcc_clk(hw);
	if (ckc_ops->ckc_clkctrl_get_rate)
		rate = ckc_ops->ckc_clkctrl_get_rate(tcc->id);
	return rate;
}

static int tcc_clkctrl_set_rate(struct clk_hw *hw, unsigned long rate, unsigned long parent_rate)
{
	struct tcc_clk *tcc = to_tcc_clk(hw);
	if (ckc_ops->ckc_clkctrl_set_rate)
		ckc_ops->ckc_clkctrl_set_rate(tcc->id, rate);
	return 0;
}

static int tcc_clkctrl_is_enabled(struct clk_hw *hw)
{
	struct tcc_clk *tcc = to_tcc_clk(hw);
	if (ckc_ops->ckc_is_clkctrl_enabled)
		return ckc_ops->ckc_is_clkctrl_enabled(tcc->id) ? 1 : 0;
	return 0;
}

static struct clk_ops tcc_clkctrl_ops= {
	.enable		= tcc_clkctrl_enable,
#ifndef IGNORE_CLK_DISABLE
	.disable	= tcc_clkctrl_disable,
#endif
	.is_enabled	= tcc_clkctrl_is_enabled,
	.recalc_rate	= tcc_clkctrl_recalc_rate,
	.round_rate	= tcc_round_rate,
	.set_rate	= tcc_clkctrl_set_rate,
};

void __init tcc_fbus_init(struct device_node *np)
{
	BUG_ON(!ckc_ops);
	tcc_clk_register(np, &tcc_fbus_clks, "fbus", &tcc_clkctrl_ops);
}
CLOCKSOURCE_OF_DECLARE(tcc_clk_fbus, "telechips,clk-fbus", tcc_fbus_init);


static int tcc_peri_enable(struct clk_hw *hw)
{
	struct tcc_clk *tcc = to_tcc_clk(hw);
	if (ckc_ops->ckc_peri_enable)
		ckc_ops->ckc_peri_enable(tcc->id);
	return 0;
}

static void tcc_peri_disable(struct clk_hw *hw)
{
	struct tcc_clk *tcc = to_tcc_clk(hw);
	if (ckc_ops->ckc_peri_disable)
		ckc_ops->ckc_peri_disable(tcc->id);
}

static unsigned long tcc_peri_recalc_rate(struct clk_hw *hw, unsigned long parent_rate)
{
	unsigned long rate = 0;
	struct tcc_clk *tcc = to_tcc_clk(hw);
	if (ckc_ops->ckc_peri_get_rate)
		rate = ckc_ops->ckc_peri_get_rate(tcc->id);
	return rate;
}

static int tcc_peri_set_rate(struct clk_hw *hw, unsigned long rate, unsigned long parent_rate)
{
	struct tcc_clk *tcc = to_tcc_clk(hw);
	if (ckc_ops->ckc_peri_set_rate)
		ckc_ops->ckc_peri_set_rate(tcc->id, rate);
	return 0;
}

static int tcc_peri_is_enabled(struct clk_hw *hw)
{
	struct tcc_clk *tcc = to_tcc_clk(hw);
	if (ckc_ops->ckc_is_peri_enabled)
		return ckc_ops->ckc_is_peri_enabled(tcc->id) ? 1 : 0;
	return 0;
}

static struct clk_ops tcc_peri_ops = {
	.enable		= tcc_peri_enable,
	.disable	= tcc_peri_disable,
	.recalc_rate	= tcc_peri_recalc_rate,
	.round_rate	= tcc_round_rate,
	.set_rate	= tcc_peri_set_rate,
	.is_enabled	= tcc_peri_is_enabled,
};

void __init tcc_peri_init(struct device_node *np)
{
	BUG_ON(!ckc_ops);
	tcc_clk_register(np, &tcc_peri_clks, "peri", &tcc_peri_ops);
}
CLOCKSOURCE_OF_DECLARE(tcc_clk_peri, "telechips,clk-peri", tcc_peri_init);

#if (ISOIP_TOP)
static int tcc_isoip_top_enable(struct clk_hw *hw)
{
	struct tcc_clk *tcc = to_tcc_clk(hw);
	if (ckc_ops->ckc_isoip_top_pwdn)
		ckc_ops->ckc_isoip_top_pwdn(tcc->id, false);
	return 0;
}

static void tcc_isoip_top_disable(struct clk_hw *hw)
{
	struct tcc_clk *tcc = to_tcc_clk(hw);
	if (ckc_ops->ckc_isoip_top_pwdn)
		ckc_ops->ckc_isoip_top_pwdn(tcc->id, true);
}

static int tcc_isoip_top_is_enabled(struct clk_hw *hw)
{
	struct tcc_clk *tcc = to_tcc_clk(hw);
	if (ckc_ops->ckc_is_isoip_top_pwdn)
		return ckc_ops->ckc_is_isoip_top_pwdn(tcc->id) ? 0 : 1;
	return 0;
}

static struct clk_ops tcc_isoip_top_ops = {
	.enable		= tcc_isoip_top_enable,
#ifndef IGNORE_CLK_DISABLE
	.disable	= tcc_isoip_top_disable,
#endif
	.is_enabled	= tcc_isoip_top_is_enabled,
};

void __init tcc_isoip_top_init(struct device_node *np)
{
	BUG_ON(!ckc_ops);
	tcc_clk_register(np, &tcc_isoip_top_clks, "isoip", &tcc_isoip_top_ops);
}
CLOCKSOURCE_OF_DECLARE(tcc_clk_isoip_top, "telechips,clk-isoip_top", tcc_isoip_top_init);
#endif

#if (ISOIP_DDI)
static int tcc_isoip_ddi_enable(struct clk_hw *hw)
{
	struct tcc_clk *tcc = to_tcc_clk(hw);
	if (ckc_ops->ckc_isoip_ddi_pwdn)
		ckc_ops->ckc_isoip_ddi_pwdn(tcc->id, false);
	return 0;
}

static void tcc_isoip_ddi_disable(struct clk_hw *hw)
{
	struct tcc_clk *tcc = to_tcc_clk(hw);
	if (ckc_ops->ckc_isoip_ddi_pwdn)
		ckc_ops->ckc_isoip_ddi_pwdn(tcc->id, true);
}

static int tcc_isoip_ddi_is_enabled(struct clk_hw *hw)
{
	struct tcc_clk *tcc = to_tcc_clk(hw);
	if (ckc_ops->ckc_is_isoip_ddi_pwdn)
		return ckc_ops->ckc_is_isoip_ddi_pwdn(tcc->id) ? 0 : 1;
	return 0;
}

static struct clk_ops tcc_isoip_ddi_ops = {
	.enable		= tcc_isoip_ddi_enable,
#ifndef IGNORE_CLK_DISABLE
	.disable	= tcc_isoip_ddi_disable,
#endif
	.is_enabled	= tcc_isoip_ddi_is_enabled,
};

void __init tcc_isoip_ddi_init(struct device_node *np)
{
	BUG_ON(!ckc_ops);
	tcc_clk_register(np, &tcc_isoip_ddi_clks, "isoip_ddi", &tcc_isoip_ddi_ops);
}
CLOCKSOURCE_OF_DECLARE(tcc_clk_isoip_ddi, "telechips,clk-isoip_ddi", tcc_isoip_ddi_init);
#endif

static int tcc_ddibus_enable(struct clk_hw *hw)
{
	struct tcc_clk *tcc = to_tcc_clk(hw);
	if (ckc_ops->ckc_ddibus_pwdn)
		ckc_ops->ckc_ddibus_pwdn(tcc->id, false);
	if (ckc_ops->ckc_ddibus_swreset)
		ckc_ops->ckc_ddibus_swreset(tcc->id, false);
	return 0;
}

static void tcc_ddibus_disable(struct clk_hw *hw)
{
	struct tcc_clk *tcc = to_tcc_clk(hw);
	if (ckc_ops->ckc_ddibus_swreset)
		ckc_ops->ckc_ddibus_swreset(tcc->id, true);
	if (ckc_ops->ckc_ddibus_pwdn)
		ckc_ops->ckc_ddibus_pwdn(tcc->id, true);
}

static int tcc_ddibus_is_enabled(struct clk_hw *hw)
{
	struct tcc_clk *tcc = to_tcc_clk(hw);
	if (ckc_ops->ckc_is_ddibus_pwdn)
		return ckc_ops->ckc_is_ddibus_pwdn(tcc->id) ? 0 : 1;
	return 0;
}

static void tcc_ddibus_reset(struct clk_hw *hw, unsigned reset)
{
	struct tcc_clk *tcc = to_tcc_clk(hw);
	if (ckc_ops->ckc_ddibus_swreset)
		ckc_ops->ckc_ddibus_swreset(tcc->id, (reset)?true:false);
}

static struct clk_ops tcc_ddibus_ops = {
	.enable		= tcc_ddibus_enable,
	.disable	= tcc_ddibus_disable,
	.is_enabled	= tcc_ddibus_is_enabled,
	.reset		= tcc_ddibus_reset,
};

void __init tcc_ddibus_init(struct device_node *np)
{
	BUG_ON(!ckc_ops);
	tcc_clk_register(np, &tcc_ddibus_clks, "ddibus", &tcc_ddibus_ops);
}
CLOCKSOURCE_OF_DECLARE(tcc_clk_ddibus, "telechips,clk-ddibus", tcc_ddibus_init);

static int tcc_iobus_enable(struct clk_hw *hw)
{
	struct tcc_clk *tcc = to_tcc_clk(hw);
	if (ckc_ops->ckc_iobus_pwdn)
		ckc_ops->ckc_iobus_pwdn(tcc->id, false);
	if (ckc_ops->ckc_iobus_swreset)
		ckc_ops->ckc_iobus_swreset(tcc->id, false);
	return 0;
}

static void tcc_iobus_disable(struct clk_hw *hw)
{
	struct tcc_clk *tcc = to_tcc_clk(hw);
	if (ckc_ops->ckc_iobus_swreset)
		ckc_ops->ckc_iobus_swreset(tcc->id, true);
	if (ckc_ops->ckc_iobus_pwdn)
		ckc_ops->ckc_iobus_pwdn(tcc->id, true);
}

static int tcc_iobus_is_enabled(struct clk_hw *hw)
{
	struct tcc_clk *tcc = to_tcc_clk(hw);
	if (ckc_ops->ckc_is_iobus_pwdn)
		return ckc_ops->ckc_is_iobus_pwdn(tcc->id) ? 0 : 1;
	return 0;
}

static void tcc_iobus_reset(struct clk_hw *hw, unsigned reset)
{
	struct tcc_clk *tcc = to_tcc_clk(hw);
	if (ckc_ops->ckc_iobus_swreset)
		ckc_ops->ckc_iobus_swreset(tcc->id, (reset)?true:false);
}

static struct clk_ops tcc_iobus_ops = {
	.enable		= tcc_iobus_enable,
	.disable	= tcc_iobus_disable,
	.is_enabled	= tcc_iobus_is_enabled,
	.reset		= tcc_iobus_reset,
};

void __init tcc_iobus_init(struct device_node *np)
{
	BUG_ON(!ckc_ops);
	tcc_clk_register(np, &tcc_iobus_clks, "iobus", &tcc_iobus_ops);
}
CLOCKSOURCE_OF_DECLARE(tcc_clk_iobus, "telechips,clk-iobus", tcc_iobus_init);


static int tcc_vpubus_enable(struct clk_hw *hw)
{
	struct tcc_clk *tcc = to_tcc_clk(hw);
	if (ckc_ops->ckc_vpubus_pwdn)
		ckc_ops->ckc_vpubus_pwdn(tcc->id, false);
	if (ckc_ops->ckc_vpubus_swreset)
		ckc_ops->ckc_vpubus_swreset(tcc->id, false);
	return 0;
}

static void tcc_vpubus_disable(struct clk_hw *hw)
{
	struct tcc_clk *tcc = to_tcc_clk(hw);
	if (ckc_ops->ckc_vpubus_swreset)
		ckc_ops->ckc_vpubus_swreset(tcc->id, true);
	if (ckc_ops->ckc_vpubus_pwdn)
		ckc_ops->ckc_vpubus_pwdn(tcc->id, true);
}

static int tcc_vpubus_is_enabled(struct clk_hw *hw)
{
	struct tcc_clk *tcc = to_tcc_clk(hw);
	if (ckc_ops->ckc_is_vpubus_pwdn)
		return ckc_ops->ckc_is_vpubus_pwdn(tcc->id) ? 0 : 1;
	return 0;
}

static void tcc_vpubus_reset(struct clk_hw *hw, unsigned reset)
{
	struct tcc_clk *tcc = to_tcc_clk(hw);
	if (ckc_ops->ckc_vpubus_swreset)
		ckc_ops->ckc_vpubus_swreset(tcc->id, (reset)?true:false);
}

static struct clk_ops tcc_vpubus_ops = {
	.enable		= tcc_vpubus_enable,
	.disable	= tcc_vpubus_disable,
	.is_enabled	= tcc_vpubus_is_enabled,
	.reset		= tcc_vpubus_reset,
};

void __init tcc_vpubus_init(struct device_node *np)
{
	BUG_ON(!ckc_ops);
	tcc_clk_register(np, &tcc_vpubus_clks, "vpubus", &tcc_vpubus_ops);
}
CLOCKSOURCE_OF_DECLARE(tcc_clk_vpubus, "telechips,clk-vpubus", tcc_vpubus_init);


static int tcc_hsiobus_enable(struct clk_hw *hw)
{
	struct tcc_clk *tcc = to_tcc_clk(hw);
	if (ckc_ops->ckc_hsiobus_pwdn)
		ckc_ops->ckc_hsiobus_pwdn(tcc->id, false);
	if (ckc_ops->ckc_hsiobus_swreset)
		ckc_ops->ckc_hsiobus_swreset(tcc->id, false);
	return 0;
}

static void tcc_hsiobus_disable(struct clk_hw *hw)
{
	struct tcc_clk *tcc = to_tcc_clk(hw);
	if (ckc_ops->ckc_hsiobus_swreset)
		ckc_ops->ckc_hsiobus_swreset(tcc->id, true);
	if (ckc_ops->ckc_hsiobus_pwdn)
		ckc_ops->ckc_hsiobus_pwdn(tcc->id, true);
}

static int tcc_hsiobus_is_enabled(struct clk_hw *hw)
{
	struct tcc_clk *tcc = to_tcc_clk(hw);
	if (ckc_ops->ckc_is_hsiobus_pwdn)
		return ckc_ops->ckc_is_hsiobus_pwdn(tcc->id) ? 0 : 1;
	return 0;
}

static void tcc_hsiobus_reset(struct clk_hw *hw, unsigned reset)
{
	struct tcc_clk *tcc = to_tcc_clk(hw);
	if (ckc_ops->ckc_hsiobus_swreset)
		ckc_ops->ckc_hsiobus_swreset(tcc->id, (reset)?true:false);
}

static struct clk_ops tcc_hsiobus_ops = {
	.enable		= tcc_hsiobus_enable,
	.disable	= tcc_hsiobus_disable,
	.is_enabled	= tcc_hsiobus_is_enabled,
	.reset		= tcc_hsiobus_reset,
};

void __init tcc_hsiobus_init(struct device_node *np)
{
	BUG_ON(!ckc_ops);
	tcc_clk_register(np, &tcc_hsiobus_clks, "hsiobus", &tcc_hsiobus_ops);
}
CLOCKSOURCE_OF_DECLARE(tcc_clk_hsiobus, "telechips,clk-hsiobus", tcc_hsiobus_init);

