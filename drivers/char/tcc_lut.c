/*
 * linux/drivers/char/tcc_lut.c
 * Author:  <linux@telechips.com>
 * Created: June 10, 2008
 * Description: TCC VIOC h/w block 
 *
 * Copyright (C) 2008-2009 Telechips
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see the file COPYING, or write
 * to the Free Software Foundation, Inc.,
 * 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */ 

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/div64.h>
#include <asm/mach/map.h>
#include <linux/fs.h> 
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_device.h>
#include <linux/of_dma.h>
#include <linux/miscdevice.h>
#include <soc/tcc/pmap.h>
#ifdef CONFIG_ARCH_TCC897X
#include <mach/bsp.h>
#include <mach/vioc_rdma.h>
#include <mach/vioc_wdma.h>
#include <mach/vioc_wmix.h>
#include <mach/vioc_config.h>
#include <mach/vioc_api.h>
#include <mach/vioc_disp.h>
#include <mach/tcc_lut_ioctl.h>
#include <mach/vioc_lut.h>
#include <mach/vioc_global.h>
#include <mach/vioc_blk.h>
#else
#include <video/tcc/vioc_rdma.h>
#include <video/tcc/vioc_wdma.h>
#include <video/tcc/vioc_wmix.h>
#include <video/tcc/vioc_config.h>
#include <video/tcc/vioc_api.h>
#include <video/tcc/vioc_disp.h>
#include <video/tcc/tcc_lut_ioctl.h>
#include <video/tcc/vioc_lut.h>
#include <video/tcc/vioc_global.h>
#include <video/tcc/vioc_blk.h>
#endif

#define TCC_LUT_DEBUG 		1
#define dprintk(msg...) 		if(TCC_LUT_DEBUG) { printk("TCC LUT: " msg); }


struct lut_drv_type {
	unsigned int  			dev_opened;
	struct miscdevice		*misc;
};

static long lut_drv_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	//struct miscdevice	*misc = (struct miscdevice *)filp->private_data;
	//struct scaler_drv_type	*lut_drv_data = dev_get_drvdata(misc->parent);

	switch(cmd) {
		case TCC_LUT_SET:
			{
				struct VIOC_LUT_VALUE_SET *lut_cmd;

				lut_cmd = kmalloc(sizeof(struct VIOC_LUT_VALUE_SET), GFP_KERNEL);
				if (lut_cmd == NULL) {
					pr_err("%s: could not allocate lut_cmd\n", __func__);
					return -ENOMEM;
				}

				if(copy_from_user((void *)lut_cmd, (const void *)arg, sizeof(struct VIOC_LUT_VALUE_SET)))				{
					kfree(lut_cmd);
					return -EFAULT;
				}

				tcc_set_lut_table(TVC_LUT(lut_cmd->lut_number), lut_cmd->Gamma);
				kfree(lut_cmd);
				
			}
			return 0;
			
		case TCC_LUT_PLUG_IN:
			{
				struct VIOC_LUT_PLUG_IN_SET lut_cmd;
				
				if(copy_from_user((void *)&lut_cmd, (const void *)arg, sizeof(lut_cmd)))
					return -EFAULT;
				
				if(!lut_cmd.enable) {
					tcc_set_lut_enable(TVC_LUT(lut_cmd.lut_number), false);
				}
				else 	{
					tcc_set_lut_plugin(TVC_LUT(lut_cmd.lut_number), TVC_RDMA(lut_cmd.lut_plug_in_ch));
					tcc_set_lut_enable(TVC_LUT(lut_cmd.lut_number), true);
				}
			}
			return 0;

		default:
			printk(KERN_ALERT "not supported LUT IOCTL(0x%x). \n", cmd);
			break;			
	}

	return 0;
}



static int lut_drv_open(struct inode *inode, struct file *filp)
{
	struct miscdevice	*misc = (struct miscdevice *)filp->private_data;
	struct lut_drv_type	*lut = dev_get_drvdata(misc->parent);

	int ret = 0;
	
	dprintk("%s():  In -open(%d) \n", __func__, lut->dev_opened);

	lut->dev_opened++;
	return ret;
}



static int lut_drv_release(struct inode *inode, struct file *filp)
{
	struct miscdevice	*misc = (struct miscdevice *)filp->private_data;
	struct lut_drv_type	*lut = dev_get_drvdata(misc->parent);

	dprintk("%s(): open(%d). \n", __func__, lut->dev_opened);

	return 0;
}


static struct file_operations lut_drv_fops = {
	.owner			= THIS_MODULE,
	.unlocked_ioctl		= lut_drv_ioctl,
	.open			= lut_drv_open,
	.release			= lut_drv_release,
};

static int lut_drv_probe(struct platform_device *pdev)
{
	struct lut_drv_type *lut;
	int ret = -ENODEV;

	lut = kzalloc(sizeof(struct lut_drv_type), GFP_KERNEL);
	if (!lut)
		return -ENOMEM;

	lut->misc = kzalloc(sizeof(struct miscdevice), GFP_KERNEL);
	if (lut->misc == NULL)
		goto err_misc_alloc;
	
 	/* register scaler discdevice */
	lut->misc->minor = MISC_DYNAMIC_MINOR;
	lut->misc->fops = &lut_drv_fops;
	lut->misc->name = "tcc_lut";
	lut->misc->parent = &pdev->dev;
	ret = misc_register(lut->misc);
	if (ret)
		goto err_misc_register;
	
	platform_set_drvdata(pdev, lut);

	pr_info("%s: :%s, Driver Initialized\n",__func__, pdev->name);
	return 0;

err_misc_register:
	misc_deregister(lut->misc);
	kfree(lut->misc);
err_misc_alloc:
	kfree(lut);
	printk("%s: %s: err ret:%d \n", __func__, pdev->name, ret);
	return ret;
}

static int lut_drv_remove(struct platform_device *pdev)
{
	struct lut_drv_type *lut = (struct lut_drv_type *)platform_get_drvdata(pdev);

	misc_deregister(lut->misc);
	kfree(lut);
	return 0;
}

static int lut_drv_suspend(struct platform_device *pdev, pm_message_t state)
{
	// TODO:
	return 0;
}

static int lut_drv_resume(struct platform_device *pdev)
{
	//struct lut_drv_type *lut = (struct lut_drv_type *)platform_get_drvdata(pdev);

	return 0;
}


static struct of_device_id lut_of_match[] = {
	{ .compatible = "telechips,vioc_lut" },
	{}
};
MODULE_DEVICE_TABLE(of, lut_of_match);

static struct platform_driver lut_driver = {
	.probe		= lut_drv_probe,
	.remove		= lut_drv_remove,
	.suspend		= lut_drv_suspend,
	.resume		= lut_drv_resume,
	.driver 	= {
		.name	= "tcc_lut",
		.owner	= THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table	= of_match_ptr(lut_of_match),
#endif
	},
};

static int __init lut_drv_init(void)
{
	return platform_driver_register(&lut_driver);
}

static void __exit lut_drv_exit(void)
{
	platform_driver_unregister(&lut_driver);
}

module_init(lut_drv_init);
module_exit(lut_drv_exit);


MODULE_AUTHOR("Telechips.");
MODULE_DESCRIPTION("Telechips look up table Driver");
MODULE_LICENSE("GPL");



