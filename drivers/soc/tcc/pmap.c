/*
 * PMAP driver for Telechips SoCs
 *
 * Copyright (C) 2010-2016 Telechips Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/of_address.h>
#include <linux/of_reserved_mem.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/memblock.h>
#include <uapi/asm/setup.h>
#include <soc/tcc/pmap.h>

#define MAX_PMAPS		64

static int num_pmaps = 0;
static pmap_t pmap_table[MAX_PMAPS];

int pmap_get_info(const char *name, pmap_t *mem)
{
	int i;

	for (i = 0; i < num_pmaps; i++) {
		if (strcmp(name, pmap_table[i].name) == 0) {
			memcpy(mem, &pmap_table[i], sizeof(pmap_t));
			return 1;
		}
	}
	return 0;
}
EXPORT_SYMBOL(pmap_get_info);

static int proc_pmap_show(struct seq_file *m, void *v)
{
	pmap_t *p = pmap_table;
	int i;

	seq_printf(m, "%-10s %-10s %s\n", "base_addr", "size", "name");
	for (i = 0; i < num_pmaps; i++, p++) {
		seq_printf(m, "0x%8.8x 0x%8.8x %s\n",
				p->base, p->size, p->name);
	}
	return 0;
}

static int proc_pmap_open(struct inode *inode, struct file *file)
{
	return single_open(file, proc_pmap_show, PDE_DATA(inode));
}

static const struct file_operations proc_pmap_fops = {
	.open		= proc_pmap_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= seq_release,
};

static void pmap_add(const char *name, unsigned long base, unsigned long size)
{
	if (num_pmaps < MAX_PMAPS) {
		strncpy(pmap_table[num_pmaps].name, name, TCC_PMAP_NAME_LEN);
		pmap_table[num_pmaps].base = base;
		pmap_table[num_pmaps].size = size;
		++num_pmaps;
	}
}

static int tcc_pmap_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct device_node *child;
	struct resource *res;
	unsigned long pmap_base, base;
	u32 pmap_size;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "no mem resource?\n");
		return -ENODEV;
	}
	pmap_base = res->start;
	pmap_size = 0;

	base = pmap_base;
	for_each_child_of_node(np, child) {
		const char *name;
		u32 size;
		int ret;

		if (of_property_read_string(child, "telechips,pmap-name", &name))
			continue;

		ret = of_property_read_u32_index(child, "reg", 0, &size);
		if (ret < 0)
			continue;

		dev_vdbg(&pdev->dev,
			 "pmap: name='%s', base=0x%lx, size=0x%x\n",
			 name, base, size);

		pmap_add(name, base, size);

		base += size;
		pmap_size += size;
	}
	pmap_add("total", pmap_base, pmap_size);

	pr_info("Total reserved memory: base=0x%lx, size=0x%x\n",
		pmap_base, pmap_size);

#ifdef CONFIG_PROC_FS
	if (!proc_create("pmap", S_IRUGO, NULL, &proc_pmap_fops))
		return -ENOMEM;
#endif
	return 0;
}

static int tcc_pmap_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id tcc_pmap_of_id_table[] = {
	{ .compatible = "telechips,pmap", },
	{}
};
MODULE_DEVICE_TABLE(of, tcc_pmap_of_id_table);

static struct platform_driver tcc_pmap_driver = {
	.driver = {
		.name	= "pmap",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(tcc_pmap_of_id_table),
	},
	.probe	= tcc_pmap_probe,
	.remove	= tcc_pmap_remove,
};

static int __init tcc_pmap_init(void)
{
	return platform_driver_register(&tcc_pmap_driver);
}
core_initcall(tcc_pmap_init);

static void tcc_pmap_exit(void)
{
	platform_driver_unregister(&tcc_pmap_driver);
}
module_exit(tcc_pmap_exit);

MODULE_AUTHOR("Albert Kim <kimys@telechips.com>");
MODULE_DESCRIPTION("Telechips PMAP driver");
MODULE_LICENSE("GPL");
