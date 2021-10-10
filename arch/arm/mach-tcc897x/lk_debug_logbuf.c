/*
 *  linux/arch/arm/mach-tcc897x/lk_debug_logbuf.c
 *
 *  Copyright (C) 2019 kibum.lee@mobis.co.kr
 *  Original Copyright (C) 1995  Linus Torvalds
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <mach/lk_debug_logbuf.h>
#include <asm/io.h>

static void __iomem *iomem_lk_log = NULL;
static int  lk_memblock_reserved = 0;

int get_lk_memblock_reserved(void)
{
	return lk_memblock_reserved;
}
EXPORT_SYMBOL(get_lk_memblock_reserved);

void set_lk_memblock_reserved(int value)
{
	lk_memblock_reserved = value;
	return;
}
EXPORT_SYMBOL(set_lk_memblock_reserved);

int lk_log_print_show(void)
{
	struct lk_log* p_lk_log=NULL;
	p_lk_log = (struct lk_log*)iomem_lk_log;
	if (p_lk_log != NULL && p_lk_log->header.cookie == LK_LOG_COOKIE)
	{
		pr_info("=================== lk_log_print_show ===================\n");
		pr_info("%s\n", p_lk_log->data);
		return 0;
	}
	else
		return -EFAULT;
}
EXPORT_SYMBOL(lk_log_print_show);

static int lk_log_proc_show(struct seq_file *m, void *v) {
	struct lk_log* p_lk_log=NULL;
	p_lk_log = (struct lk_log*)iomem_lk_log;
	seq_printf(m, "=================== lk_log proc filesystem ===================\n");
	if (p_lk_log != NULL && p_lk_log->header.cookie == LK_LOG_COOKIE)
		seq_printf(m, "%s\n", p_lk_log->data);
	return 0;
}

static int lk_log_proc_open(struct inode *inode, struct  file *file) {
  return single_open(file, lk_log_proc_show, NULL);
}

static const struct file_operations lk_log_proc_fops = {
  .owner = THIS_MODULE,
  .open = lk_log_proc_open,
  .read = seq_read,
  .llseek = seq_lseek,
  .release = single_release,
};

static int __init lk_log_proc_init(void) {

	struct lk_log* p_lk_log=NULL;

	if (!proc_create("lk_log", S_IRUGO, NULL, &lk_log_proc_fops))
		return -ENOMEM;

	if(lk_memblock_reserved > 0) {
		//iomem_lk_log = ioremap((phys_addr_t)(PHYS_DDR_BASE + ABOOT_FORCE_UART_ADDR_OFFSET), ABOOT_FORCE_UART_SIZE);
		iomem_lk_log = __phys_to_virt((phys_addr_t)(PHYS_DDR_BASE + ABOOT_FORCE_UART_ADDR_OFFSET));

		p_lk_log = (struct lk_log*)iomem_lk_log;
		if (p_lk_log != NULL) {
			printk(KERN_INFO "lk_log cookie=0x%x, written_size=%d, max_size=%d\n", p_lk_log->header.cookie, p_lk_log->header.size_written, p_lk_log->header.max_size);
		}
	}
	return 0;
}

static void __exit lk_log_proc_exit(void) {
  remove_proc_entry("lk_log", NULL);
}

MODULE_LICENSE("GPL");
module_init(lk_log_proc_init);
module_exit(lk_log_proc_exit);

