/****************************************************************************
 * arch/arm/plat-tcc/cpufreq.c
 * Copyright (C) 2015 Telechips Inc.
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

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/cpufreq.h>
#include <linux/cpu.h>
#include <linux/pm_opp.h>
#include <linux/of.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/consumer.h>

struct tcc_dvfs_data {
	unsigned long		voltage;
	struct clk		*clk;
	struct regulator	*vdd;
};

struct tcc_cluster_data {
	unsigned cluster_num;
	struct tcc_dvfs_data	*dvfs[0];
};

static struct tcc_cluster_data *cluster_data = NULL;
static bool is_suspended;

static int tcc_verify_speed(struct cpufreq_policy *policy)
{
	struct cpufreq_frequency_table *table = cpufreq_frequency_get_table(policy->cpu);
	if (!table)
		return -ENODEV;

	return cpufreq_frequency_table_verify(policy, table);
}

unsigned int tcc_getspeed(unsigned int cpu)
{
	unsigned cluster = topology_physical_package_id(cpu);

	if (cluster_data->dvfs[cluster]->clk)
		return clk_get_rate(cluster_data->dvfs[cluster]->clk) / 1000;

	return 0;
}

static int tcc_target(struct cpufreq_policy *policy,
			      unsigned int target_freq,
			      unsigned int relation)
{
	struct cpufreq_frequency_table *table = cpufreq_frequency_get_table(policy->cpu);
	unsigned cluster = topology_physical_package_id(policy->cpu);
	static struct device *cpu_dev;
	struct cpufreq_freqs freqs;
	unsigned long volt = 0;
	struct dev_pm_opp *opp = NULL;
	int idx, ret = 0;

	if (is_suspended)
		return -EBUSY ;

	cpu_dev = get_cpu_device(policy->cpu);
	ret = cpufreq_frequency_table_target(policy, table, target_freq,
		relation, &idx);

	if (ret)
		goto _out;

	freqs.old = policy->cur;
	freqs.new = table[idx].frequency;

	/* get target voltage */
	if (cluster_data->dvfs[cluster]->vdd) {
		unsigned long freq;
		rcu_read_lock();
		freq = freqs.new * 1000;
		opp = dev_pm_opp_find_freq_ceil(cpu_dev, &freq);
		if (IS_ERR(opp)) {
			opp = NULL;
			rcu_read_unlock();
			dev_err(cpu_dev, "%s: unable to find OPP for %d\n",
				__func__, freqs.new);
			return -EINVAL;
		}
		volt = dev_pm_opp_get_voltage(opp);
		rcu_read_unlock();
	}

//	pr_info("%s: cpu:%d, target(freq:%d, volt:%ld), old(freq:%d, volt:%ld)\n",
//		__func__, policy->cpu, freqs.new, volt, freqs.old, cluster_data->dvfs[cluster]->voltage);

	if (cluster_data->dvfs[cluster]->vdd && opp) {
		if (volt > cluster_data->dvfs[cluster]->voltage) {
			ret = regulator_set_voltage(cluster_data->dvfs[cluster]->vdd, volt, volt);
			if (ret) {
				pr_warn("%s: cpu:%d, failed to set voltage(%ld). ret=%d\n", __func__, policy->cpu, volt, ret);
				return -EINVAL;
			}
			else
				cluster_data->dvfs[cluster]->voltage = volt;
		}
	}

	cpufreq_freq_transition_begin(policy, &freqs);
	if (cluster_data->dvfs[cluster]->clk) {
		ret = clk_set_rate(cluster_data->dvfs[cluster]->clk, freqs.new * 1000);
		if (ret) {
			pr_err("%s: Failed to set cpu frequency to %d kHz\n", __func__, freqs.new);
			return ret;
		}
	}
	cpufreq_freq_transition_end(policy, &freqs, 0);

	if (cluster_data->dvfs[cluster]->vdd && opp) {
		if (volt < cluster_data->dvfs[cluster]->voltage) {
			if (!regulator_set_voltage(cluster_data->dvfs[cluster]->vdd, volt, volt))
				cluster_data->dvfs[cluster]->voltage = volt;
		}
	}

_out:
	return ret;
}

static void tcc_cpu_add_node(struct device *cpu_dev, int cpu)
{
	struct device_node *np, *parent;
	int nr;

	parent = of_find_node_by_path("/cpus");
	if (parent) {
		nr = 0;
		for_each_child_of_node(parent, np) {
			if (nr == cpu) {
				cpu_dev->of_node = np;
				return;
			}
			nr++;
		}
	}

}

static int tcc_cpu_init(struct cpufreq_policy *policy)
{
	struct cpufreq_frequency_table *freq_table;
	static struct device *cpu_dev;
	int ret = 0;

	cpu_dev = get_cpu_device(policy->cpu);
	if (!cpu_dev) {
		pr_warn("%s: unable to get the cpu device\n", __func__);
		return -EINVAL;
	}

	/* init opp table */
	tcc_cpu_add_node(cpu_dev, policy->cpu);
	ret = of_init_opp_table(cpu_dev);
	if (ret) {
		dev_err(cpu_dev, "%s: cpu%d: failed init opp table[%d]\n", __func__, policy->cpu, ret);
		goto fail;
	}

	/* get opp table */
	ret = dev_pm_opp_init_cpufreq_table(cpu_dev, &freq_table);
	if (ret) {
		dev_err(cpu_dev, "%s: cpu%d: failed creating freq table[%d]\n", __func__, policy->cpu, ret);
		goto fail;
	}

	ret = cpufreq_generic_init(policy, freq_table, 300 * 1000);
	if (ret) {
		dev_err(cpu_dev, "CPU %d invalid freq table\n", policy->cpu);
		goto fail_table;
	}

	return 0;

fail_table:
	dev_pm_opp_free_cpufreq_table(cpu_dev, &freq_table);

fail:
	return ret;
}

static int tcc_cpu_exit(struct cpufreq_policy *policy)
{
	struct device *cpu_dev = get_cpu_device(policy->cpu);
	struct cpufreq_frequency_table *freq_table = cpufreq_frequency_get_table(policy->cpu);

	if (!cpu_dev)
		goto tcc_cpu_exit_finish;

	dev_pm_opp_free_cpufreq_table(cpu_dev, &freq_table);

tcc_cpu_exit_finish:
	return 0;
}

static struct cpufreq_driver tcc_driver = {
	.name	= "tcc",
	.verify	= tcc_verify_speed,
	.target	= tcc_target,
	.get	= tcc_getspeed,
	.init	= tcc_cpu_init,
	.exit	= tcc_cpu_exit,
	.attr	= cpufreq_generic_attr,
};

static int tcc_cpu_pm_notify(struct notifier_block *nb, unsigned long event, void *dummy)
{
	switch (event) {
	case PM_HIBERNATION_PREPARE:
	case PM_SUSPEND_PREPARE:
		pr_info("%s: enter suspend. event:%ld\n", __func__, event);
		is_suspended = true;
		break;

	case PM_POST_HIBERNATION:
	case PM_POST_SUSPEND:
		pr_info("%s: exit suspend. event:%ld\n", __func__, event);
		is_suspended = false;
		break;
	}

	return NOTIFY_OK;
}

static struct notifier_block tcc_cpu_pm_notifier = {
	.notifier_call = tcc_cpu_pm_notify,
};
static int tcc_cpufreq_probe(struct platform_device *pdev)
{
	struct device_node *np, *parent;
	unsigned cluster;
	char buf[15];
	int ret = -ENODEV;

	is_suspended = false;

	parent = of_find_node_by_path("/clusters");
	if (parent) {
		cluster = 0;
		for_each_child_of_node(parent, np)
			cluster++;
	}
	else {
		cluster = 1;
	}

	cluster_data = kzalloc(sizeof(struct tcc_cluster_data), GFP_KERNEL);
	if (!cluster_data)
		return -ENOMEM;

	cluster_data->cluster_num = cluster;

	for (cluster=0 ; cluster < cluster_data->cluster_num ; cluster++) {
		cluster_data->dvfs[cluster] = kzalloc(sizeof(struct tcc_dvfs_data), GFP_KERNEL);
		if (cluster_data->dvfs[cluster] == 0)
			goto fail_malloc_dvfs;

		sprintf(buf, "cpu%1d", cluster);
		cluster_data->dvfs[cluster]->clk = clk_get(NULL, buf);
		if (IS_ERR(cluster_data->dvfs[cluster]->clk))
			cluster_data->dvfs[cluster]->clk = NULL;
		else
			clk_enable(cluster_data->dvfs[cluster]->clk);

		sprintf(buf, "vdd_cpu%1d", cluster);
		/* use regulator_get_optional. for not using dummy regulator */
		cluster_data->dvfs[cluster]->vdd = regulator_get_optional(NULL, buf);

		if (IS_ERR(cluster_data->dvfs[cluster]->vdd)) {
			pr_err("%s: failed to get regulator_get for %s\n", __func__, buf);
			cluster_data->dvfs[cluster]->vdd = NULL;
		}
	}

	ret = cpufreq_register_driver(&tcc_driver);
	if (ret) {
		pr_err("fail to register cpufreq driver. ret:%d\n", ret);
		cluster = cluster_data->cluster_num;
		goto fail_reg_cpufreq_driver;
	}

	register_pm_notifier(&tcc_cpu_pm_notifier);

	pr_info("%s: Telechips Cpufreq. Driver is Initialized\n", __func__);

	return 0;		

fail_reg_cpufreq_driver:
fail_malloc_dvfs:
	if (cluster) {
		do {
			cluster--;
			kfree(cluster_data->dvfs[cluster]);
		} while (cluster);
	}
	kfree(cluster_data);

	return ret;
}

static int tcc_cpufreq_remove(struct platform_device *pdev)
{
	int i, ret;

	unregister_pm_notifier(&tcc_cpu_pm_notifier);
	ret = cpufreq_unregister_driver(&tcc_driver);
	if (ret)
		return ret;

	if (cluster_data) {
		for (i=0 ; i<cluster_data->cluster_num ; i++)
			kfree(cluster_data->dvfs[i]);
		kfree(cluster_data);
	}
	return ret;
}

static struct platform_driver tcc_cpufreq_platdrv = {
	.driver = {
		.name	= "tcc-cpufreq",
	},
	.probe		= tcc_cpufreq_probe,
	.remove		= tcc_cpufreq_remove,
};
module_platform_driver(tcc_cpufreq_platdrv);

MODULE_DESCRIPTION("cpufreq driver for Telechips SoCs");
MODULE_LICENSE("GPL");
