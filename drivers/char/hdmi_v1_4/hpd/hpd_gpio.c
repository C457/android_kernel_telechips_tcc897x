/****************************************************************************
 * FileName    : kernel/drivers/char/hdmi_v1_3/hpd/hpd_gpio.c
 * Description : hdmi hpd driver
 *
 * Copyright (C) 2013 Telechips Inc.
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
 * ****************************************************************************/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/errno.h>
#include <linux/wait.h>
#include <linux/poll.h>

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>

#include <asm/io.h>

#ifdef CONFIG_PM_RUNTIME
#include <linux/pm_runtime.h>
#endif

#ifdef CONFIG_PM
#include <linux/pm.h>
#endif
#include <asm/mach-types.h>
#include <asm/uaccess.h>

#include <mach/tcc_board_hdmi.h>
#include <mach/hdmi_hpd.h>
#include <mach/gpio.h>

#include "../hdmi/regs-hdmi.h"

#define HPD_DEBUG 		0
#define HPD_DEBUG_GPIO 	0
#if HPD_DEBUG
#define DPRINTK(args...)    printk("hpd-gpio:" args)
#else
#define DPRINTK(args...)
#endif

static struct clk *hdmi_hpd_pclk;
static struct clk *hdmi_hpd_hclk;


#define VERSION         "1.2" /* Driver version number */
#define HPD_MINOR       243 /* Major 10, Minor 243, /dev/hpd */

#define HPD_LO          0
#define HPD_HI          1


struct hpd_struct {
	spinlock_t lock;
	wait_queue_head_t waitq;
	atomic_t state;

};

static struct hpd_struct hpd_struct;

static int hpd_open(struct inode *inode, struct file *file);
static int hpd_release(struct inode *inode, struct file *file);
static ssize_t hpd_read(struct file *file, char __user *buffer, size_t count, loff_t *ppos);
static long hpd_ioctl(struct file *file, unsigned int cmd, unsigned long arg);
static int hpd_start(void);
static int hpd_stop(void);

static int last_hpd_state;
static unsigned hpd_port;

static const struct file_operations hpd_fops =
{
    .owner   = THIS_MODULE,
    .open    = hpd_open,
    .release = hpd_release,
    .read    = hpd_read,
    .unlocked_ioctl = hpd_ioctl,
//    .poll    = hpd_poll,
};

static struct miscdevice hpd_misc_device =
{
    HPD_MINOR,
    "hpd",
    &hpd_fops,
};


static struct device *pdev_hpd;

int hpd_open(struct inode *inode, struct file *file)
{
	DPRINTK(KERN_INFO "%s\n", __FUNCTION__);

    return 0;
}

int hpd_release(struct inode *inode, struct file *file)
{

	DPRINTK(KERN_INFO "%s\n", __FUNCTION__);

    return 0;
}

ssize_t hpd_read(struct file *file, char __user *buffer, size_t count, loff_t *ppos)
{
    ssize_t retval = 0;


    spin_lock_irq(&hpd_struct.lock);

	if(gpio_get_value(hpd_port) && (last_hpd_state  == 1))
		atomic_set(&hpd_struct.state, 1);
	else
		atomic_set(&hpd_struct.state, 0);

	//printk(KERN_INFO "%s register :0x%x hpd_port:0x%x\n", __FUNCTION__, atomic_read(&hpd_struct.state), hpd_port);

	retval = put_user(atomic_read(&hpd_struct.state), (unsigned int __user *) buffer);

	atomic_set(&hpd_struct.state, -1);

    spin_unlock_irq(&hpd_struct.lock);

    return retval;
}

/**
 *      tccfb_blank
 *	@blank_mode: the blank mode we want.
 *	@info: frame buffer structure that represents a single frame buffer
 *
 *	Blank the screen if blank_mode != 0, else unblank. Return 0 if
 *	blanking succeeded, != 0 if un-/blanking failed due to e.g. a
 *	video mode which doesn't support it. Implements VESA suspend
 *	and powerdown modes on hardware that supports disabling hsync/vsync:
 *	blank_mode == 2: suspend vsync
 *	blank_mode == 3: suspend hsync
 *	blank_mode == 4: powerdown
 *
 *	Returns negative errno on error, or zero on success.
 *
 */
#ifdef CONFIG_PM_RUNTIME

static int hpd_enable(void)
{		
	printk("%s\n", __func__);

	pm_runtime_get_sync(pdev_hpd);

	return 0;
}

static int hpd_disable(void)
{
	printk("%s\n", __func__);

	pm_runtime_put_sync(pdev_hpd);

	return 0;
}

#endif

static int hpd_blank(int blank_mode)
{
	int ret = 0;

	printk("%s : blank(mode=%d)\n",__func__, blank_mode);

#ifdef CONFIG_PM_RUNTIME
	if( (pdev_hpd->power.usage_count.counter==1) && (blank_mode == 0))
	{
		// usage_count = 1 ( resume ), blank_mode = 0 ( FB_BLANK_UNBLANK ) is stable state when booting
		// don't call runtime_suspend or resume state 
	      //printk("%s ### state = [%d] count =[%d] power_cnt=[%d] \n",__func__,blank_mode, pdev_hpd->power.usage_count, pdev_hpd->power.usage_count.counter);		  
		return 0;
	}

	switch(blank_mode)
	{
		case FB_BLANK_POWERDOWN:
		case FB_BLANK_NORMAL:
			hpd_disable();
			break;
		case FB_BLANK_UNBLANK:
			hpd_enable();
			break;
		case FB_BLANK_HSYNC_SUSPEND:
		case FB_BLANK_VSYNC_SUSPEND:
		default:
			ret = -EINVAL;
	}
#endif

	return ret;
}


int hpd_start(void)
{

	DPRINTK(KERN_INFO "%s\n", __FUNCTION__);

	last_hpd_state  = 1;

    return 0;
}

int hpd_stop(void)
{
	DPRINTK(KERN_INFO "%s\n", __FUNCTION__);

    return 0;
}

long hpd_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    DPRINTK(KERN_INFO "%s\n", __FUNCTION__);

    switch (cmd) {
		case HPD_IOC_START:
			hpd_start();
			break;
		case HPD_IOC_STOP:
			hpd_stop();
			break;
		case HPD_IOC_BLANK:
		{
			unsigned int cmd;

			if (get_user(cmd, (unsigned int __user *) arg))
				return -EFAULT;

			printk(KERN_INFO "HPD: ioctl(HPD_IOC_BLANK :  %d )\n", cmd);

			hpd_blank(cmd);

			break;

		}
        default:
            return -EINVAL;	
	}

	return 0;
}

static int hpd_remove(struct platform_device *pdev)
{
    DPRINTK(KERN_INFO "%s\n", __FUNCTION__);

    misc_deregister(&hpd_misc_device);
	return 0;
}

#ifdef CONFIG_PM_RUNTIME
int hpd_runtime_suspend(struct device *dev)
{
	printk("%s:  \n", __FUNCTION__);

	hpd_stop();

	printk("%s: finish \n", __FUNCTION__);

	return 0;
}

int hpd_runtime_resume(struct device *dev)
{
	printk("%s:  \n", __FUNCTION__);

	hpd_start();

	return 0;
}

#else

#ifdef CONFIG_PM
static int hpd_suspend(struct platform_device *pdev, pm_message_t state)
{
	DPRINTK(KERN_INFO "%s\n", __FUNCTION__);
	
	return 0;
}

static int hpd_resume(struct platform_device *pdev)
{
	DPRINTK(KERN_INFO "%s\n", __FUNCTION__);

	return 0;
}
#else
#define hpd_suspend NULL
#define hpd_resume NULL
#endif
#endif

static int hpd_probe(struct platform_device *pdev)
{
#if defined(CONFIG_ARCH_TCC893X)
	unsigned int reg;
#endif
	int err;

	if (!pdev->dev.of_node) {
		pdev_hpd = pdev->dev.platform_data;;
		if (!pdev_hpd) {
			dev_err(&pdev->dev, "no platform data\n");
			return -EINVAL;
		}
	}

	pdev_hpd = &pdev->dev;

    if (!machine_is_hdmidp())
        return -ENODEV;

    printk(KERN_INFO "HPD Driver ver. %s\n", VERSION);

	if (hdmi_hpd_pclk == NULL && hdmi_hpd_hclk == NULL ) {
	
		hdmi_hpd_hclk = of_clk_get(pdev->dev.of_node, 1);
		hdmi_hpd_pclk = of_clk_get(pdev->dev.of_node, 0);

		if (IS_ERR(hdmi_hpd_hclk)) {
			printk(KERN_WARNING "HDMI: failed to get hpd hclock\n");
			hdmi_hpd_hclk = NULL;
			return -ENODEV;
		}

		if (IS_ERR(hdmi_hpd_pclk)) {
			printk(KERN_WARNING "HDMI: failed to get hpd pclock\n");
			hdmi_hpd_pclk = NULL;
			return -ENODEV;
		}
	}

	if(hdmi_hpd_pclk)
		clk_prepare_enable(hdmi_hpd_pclk);

	if(hdmi_hpd_hclk)
		clk_prepare_enable(hdmi_hpd_hclk);

	hpd_port = of_get_gpio(pdev->dev.of_node,0);
	dev_dbg(&pdev->dev, "hpd gpio: %d\n", hpd_port);
	err = devm_gpio_request_one(&pdev->dev, hpd_port, GPIOF_IN, "hpd");
	if (err)
		dev_err(&pdev->dev, "failed to get hpd gpio (%d)\n", err);

    if (misc_register(&hpd_misc_device))    {
        printk(KERN_WARNING "HPD: Couldn't register device 10, %d.\n", HPD_MINOR);
        return -EBUSY;
    }
    spin_lock_init(&hpd_struct.lock);
	
#if defined(CONFIG_ARCH_TCC893X)
    /* disable HPD interrupts */
    reg = readb(IOMEM(HDMI_SS_INTC_CON));
    reg &= ~(1<<HDMI_IRQ_HPD_PLUG);
    reg &= ~(1<<HDMI_IRQ_HPD_UNPLUG);
    writeb(reg, IOMEM(HDMI_SS_INTC_CON));
    atomic_set(&hpd_struct.state, -1);
#endif

#ifdef CONFIG_PM_RUNTIME
	pm_runtime_set_active(pdev_hpd);	
	pm_runtime_enable(pdev_hpd);  
	pm_runtime_get_noresume(pdev_hpd);  //increase usage_count 
#endif

	clk_disable_unprepare(hdmi_hpd_pclk);
	clk_disable_unprepare(hdmi_hpd_hclk);

    return 0;
}

#if 0
#ifdef CONFIG_OF
static void hpd_parse_dt(struct device_node *np, struct tcc_hdmi_hpd *hpd)
{
}
#else
static void hpd_parse_dt(struct device_node *np, struct tcc_hdmi_hpd *hpd)
{
}
#endif
#endif


#ifdef CONFIG_OF
static struct of_device_id hpd_of_match[] = {
	{ .compatible = "telechips,tcc893x-hdmi-hpd" },
	{ .compatible = "telechips,tcc896x-hdmi-hpd" },
	{ .compatible = "telechips,tcc897x-hdmi-hpd" },
	{}
};
MODULE_DEVICE_TABLE(of, hpd_of_match);
#endif


#if CONFIG_PM
static const struct dev_pm_ops hpd_pm_ops = {
#ifdef CONFIG_PM_RUNTIME
	.runtime_suspend      = hpd_runtime_suspend,
	.runtime_resume       = hpd_runtime_resume,
#else
	.suspend	= hpd_suspend,
	.resume = hpd_resume,
#endif
};
#endif


static struct platform_driver tcc_hdmi_hpd = {
	.probe	= hpd_probe,
	.remove	= hpd_remove,
	.driver	= {
		.name	= "tcc_hdmi_hpd",
		.owner	= THIS_MODULE,
#ifdef CONFIG_PM_RUNTIME
		.pm		= &hpd_pm_ops,
#endif
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(hpd_of_match),
#endif
	},
};

static __init int hpd_init(void)
{
	DPRINTK(KERN_INFO "%s\n", __FUNCTION__);

	return platform_driver_register(&tcc_hdmi_hpd);
}

static __exit void hpd_exit(void)
{
	DPRINTK(KERN_INFO "%s\n", __FUNCTION__);

	platform_driver_unregister(&tcc_hdmi_hpd);
}


module_init(hpd_init);
module_exit(hpd_exit);
MODULE_AUTHOR("Telechips Inc. <linux@telechips.com>");
MODULE_DESCRIPTION("TCCxxx hdmi hpd driver");
MODULE_LICENSE("GPL");

