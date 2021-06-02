//+[TCCQB] QB driver ( qb_com )
/*
 * drivers/char/qb_common.c
 *
 * TCC QUICKBOOT driver
 *
 * Copyright (C) 2009 Telechips, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/mman.h>
#include <linux/init.h>
#include <linux/ptrace.h>
#include <linux/device.h>
#include <linux/highmem.h>
#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/clk.h>
#include <linux/syscalls.h>
#include <linux/version.h>

#include <asm/uaccess.h>
#include <asm/io.h>
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 39))
#include <asm/pgtable.h>
#endif
#include <mach/bsp.h>

#include <asm/mach-types.h>
#include <soc/tcc/pmap.h>
#include <linux/of.h>

#include <linux/qb_manager.h>	// For QuickBoot Definitions. ( Properties... )


static int DEV_MAJOR = 291;
#define DEV_MINOR			1
#define DEV_NAME			"qb_com"

#define QB_UART_LOG			0x901	// For QuickBoot Uart Log.
#define QB_BOOT_TIME		0x902   // For QuickBoot Boot Time.
#define LOG_BUF_SIZE		1024		

#define RED_LOG				"\x1b[1;31m"		
#define BLUE_LOG			"\x1b[1;34m"
#define GREEN_LOG			"\x1b[1;32m"
#define YELLO_LOG			"\x1b[1;33m"
#define CLEAR_LOG			"\x1b[0m"

#define COLOR_LOG			GREEN_LOG



struct mutex qb_io_mutex;
static struct class *qb_com_class;

extern int lk_boot_time;
extern int kernel_boot_time;
extern unsigned long long frameworks_boot_time;

static long qb_com_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	char log_str[LOG_BUF_SIZE];
	int boot_time[3];

	mutex_lock(&qb_io_mutex);
	{
		switch(cmd)
		{
			case QB_UART_LOG:
			{
				if (arg == 0) {
					ret = -EINVAL;
					break;
				}

				copy_from_user(log_str, (void __user *)arg, LOG_BUF_SIZE);
				log_str[LOG_BUF_SIZE -1] = '\0';
				printk(KERN_ERR COLOR_LOG"- %s"CLEAR_LOG"\n", log_str);
				break;
			}
			case QB_BOOT_TIME:
			{
				if (arg == 0) {
					ret = -EINVAL;
					break;
				}

				boot_time[0] = lk_boot_time;
				boot_time[1] = kernel_boot_time;
				frameworks_boot_time = cpu_clock(0) - frameworks_boot_time;
				do_div(frameworks_boot_time, 1000000);
				boot_time[2] = (int)frameworks_boot_time;

				copy_to_user((void __user *)arg, &boot_time, sizeof(boot_time));
				break;
			}
			default:
				printk("Unsupported cmd(0x%x) for qb_com_ioctl. \n", cmd);
				ret = -EFAULT;
				break;
		}
	}

	mutex_unlock(&qb_io_mutex);

	return ret;
}

static int qb_com_open(struct inode *inode, struct file *filp)
{
	return 0;
}
static ssize_t qb_com_read(struct file *filp, char __user *ubuf, size_t count, loff_t *offp)
{
	printk(" %s \n", __func__);
	return 0;
}
static ssize_t qb_com_write(struct file *filp, const char __user *ubuf, size_t count, loff_t *offp)
{
	char *buff;
	int err;

	mutex_lock(&qb_io_mutex);

	buff = kmalloc(count, GFP_KERNEL);

	if (err = copy_from_user(buff, ubuf, count) < 0)
		return err;

#if 1	// To avoid Trash String....
	int idx;
	for (idx = 0; idx < count; idx++) {
		switch (buff[idx]) {
			case '\r' :
			case '\n' :
				buff[idx] = '\0';
				break;
			default :
				break;
		}
	}
#endif

	printk(KERN_ERR COLOR_LOG"- %s"CLEAR_LOG"\n", buff);
	kfree(buff);

	mutex_unlock(&qb_io_mutex);

	return count;
}
static int qb_com_release(struct inode *inode, struct file *filp)
{
	return 0;
}
static struct file_operations qb_com_fops = {
	.owner			= THIS_MODULE,
	.unlocked_ioctl	= qb_com_ioctl,
	.open			= qb_com_open,
	.read			= qb_com_read,
	.write			= qb_com_write,
	.release		= qb_com_release,
};
static struct miscdevice qb_com_miscdev = {
	.minor			= DEV_MINOR,
	.name			= DEV_NAME,
	.fops			= &qb_com_fops,
};


#ifdef CONFIG_PM
static int qb_com_suspend(struct device *dev) 
{
	printk("\x1b[1;31m  _______________________ %s\x1b[0m\n", __func__);
	return 0;
}
static int qb_com_resume(struct device *dev) 
{
	printk("\x1b[1;31m  _______________________ %s\x1b[0m\n", __func__);
	return 0;
}
static int qb_com_freeze(struct device *dev) 
{
	printk("\x1b[1;31m  _______________________ %s\x1b[0m\n", __func__);
	return 0;
}
static int qb_com_thaw(struct device *dev) 
{
	printk("\x1b[1;31m  _______________________ %s\x1b[0m\n", __func__);
	return 0;
}
static int qb_com_restore(struct device *dev) 
{
	printk("\x1b[1;31m  _______________________ %s\x1b[0m\n", __func__);
	return 0;
}

static const struct dev_pm_ops qb_com_pm_ops = {
	.suspend    = qb_com_suspend,
	.resume 	= qb_com_resume,
	.freeze 	= qb_com_freeze,
	.thaw 		= qb_com_thaw,
	.restore 	= qb_com_restore,
};
static const struct of_device_id qb_com_of_match[] = {
	{ .compatible = "telechips,qb_com", },
	{},
};
static struct platform_driver qb_com_driver = {
//	.suspend	= qb_com_suspend,
//	.resume		= qb_com_resume,
	.driver		= {
		.name	= DEV_NAME,
		.owner	= THIS_MODULE,
		.pm		= &qb_com_pm_ops,
		.of_match_table = of_match_ptr(qb_com_of_match),
	},
};
//module_platform_driver(qb_com_driver);

#if 0
/*		To use platform driver qb_com,
 *		Add below to tcc896x.dtsi & tcc897x.dtsi ...	*/

	qb_com {
		compatible = "telechips,qb_com";
	};
#endif

#endif	// CONFIG_PM



static void __exit qb_com_exit(void)
{
	device_destroy(qb_com_class, MKDEV(DEV_MAJOR, DEV_MINOR));
	class_destroy(qb_com_class);
	unregister_chrdev(DEV_MAJOR,DEV_NAME);
}

static int __init qb_com_init(void)
{
	void *ret = NULL;
	int error;

#ifdef CONFIG_PM
	ret = platform_driver_register(&qb_com_driver);
	if (ret)
		printk(KERN_ERR " Failed to register qb_com platform driver. [%d]\n", ret);
#endif

#if 1
	error = register_chrdev(DEV_MAJOR, DEV_NAME, &qb_com_fops);
	if(error < 0) {
		printk("%s: device failed widh %d\n", __func__, DEV_MAJOR);
		return -ENODEV;
	}

	qb_com_class = class_create(THIS_MODULE, DEV_NAME);
	ret = device_create(qb_com_class, NULL, MKDEV(DEV_MAJOR, DEV_MINOR), NULL, DEV_NAME);
/*
	if( (unsigned int)ret < -1 || (unsigned int)ret > -35) 
		printk(KERN_ERR " Failed to create device : %s major[%d] minor[%d] error[%d]\n", DEV_NAME, DEV_MAJOR, DEV_MINOR, (int)ret);
*/
#else
	error = misc_register(&qb_com_miscdev);
	if (ret) {
		printk("Failed to register miscdev on minor=%d\n", DEV_MINOR);
	}
#endif

	mutex_init(&qb_io_mutex);

	return 0;
}

module_init(qb_com_init)
module_exit(qb_com_exit)
//-[TCCQB]
//
