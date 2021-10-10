/*
 * linux/drivers/char/tcc_otp.c
 *
 * Author:  <linux@telechips.com>
 * Created: March 18, 2010
 * Description: TCC Cipher driver
 *
 * Copyright (C) 20010-2011 Telechips
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
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/wait.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/clk.h>
#include <linux/cpufreq.h>
#ifdef CONFIG_PM
#include <linux/pm.h>
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>

#include <linux/timer.h>
#include <linux/delay.h>

#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/div64.h>
#include <asm/mach/map.h>

#include <mach/io.h>

#include "tcc_otp.h"

/****************************************************************************
  DEFINITiON
 ****************************************************************************/
#define tcc_otp_readl			__raw_readl
#define tcc_otp_writel			__raw_writel

//#define DEBUG_TCC_OTP
#ifdef DEBUG_TCC_OTP
#undef dprintk
#define dprintk(msg...)			printk("tcc_otp: " msg);
#undef eprintk
#define eprintk(msg...)			printk("tcc_otp: err: " msg);
#else
#undef dprintk
#define dprintk(msg...)
#undef eprintk
#define eprintk(msg...)			//printk("tcc_otp: err: " msg);
#endif

//OTP control register offset
#define OFFSET_OTPCTRL			0x0

//OTP control register setting value
#define HwOTP_CTRL_EXTWP		0x40000000
#define HwOTP_CTRL_WP			0x00000008

/****************************************************************************
DEFINITION OF LOCAL VARIABLES
****************************************************************************/
static DEFINE_MUTEX(tcc_otp_mutex);

static void __iomem *tcc_otp_wrapper_base_addr = NULL;
static void __iomem *tcc_otp_user_base_addr = NULL;

static unsigned tcc_otp_user_area_start = 0;
static unsigned tcc_otp_user_area_end = 0;

/****************************************************************************
DEFINITION OF LOCAL FUNCTIONS
****************************************************************************/
int tcc_otp_read(unsigned int addr, unsigned int *buf, unsigned int size)
{
        void __iomem *base = NULL;
        unsigned offset = 0;
        int i = 0;

        if((addr < tcc_otp_user_area_start) || ((addr + size) > tcc_otp_user_area_end)) {
                eprintk("[%s] invalid addr err(%x, %d) \n", __func__, addr, size);
                return -EINVAL;
        }

        if((buf == NULL) || ((size % sizeof(unsigned)) != 0)) {
                eprintk("[%s] invalid param err(%p, %d) \n", __func__, buf, size);
                return -EINVAL;
        }

        offset = addr - tcc_otp_user_area_start;
        base = tcc_otp_user_base_addr + offset;
        for(i=0; i<size; i+=sizeof(unsigned))
        {
                *buf = tcc_otp_readl(base + i);
				buf++;
        }

	return 0;
}

int tcc_otp_write(unsigned int addr, unsigned int *buf, unsigned int size)
{
        void __iomem *base = NULL;
        unsigned offset = 0;
        unsigned data = 0;
        int i = 0;

        if((addr < tcc_otp_user_area_start) || ((addr + size) > tcc_otp_user_area_end)) {
                eprintk("[%s] invalid addr err(%x, %d) \n", __func__, addr, size);
                return -EINVAL;
        }

        if((buf == NULL) || ((size % sizeof(unsigned)) != 0)) {
                eprintk("[%s] invalid buf err(%p, %d) \n", __func__, buf, size);
                return -EINVAL;
        }

        //check external write protection pin
        data = tcc_otp_readl(tcc_otp_wrapper_base_addr + OFFSET_OTPCTRL);
        if(data & HwOTP_CTRL_EXTWP) {
                eprintk("[%s] external write protection enabled(%d)\n", __func__, data);
                return -EPERM;
        }

        //set programmable state
        data &= ~HwOTP_CTRL_WP;
        tcc_otp_writel(data, tcc_otp_wrapper_base_addr + OFFSET_OTPCTRL);

        offset = addr - tcc_otp_user_area_start;
        base = tcc_otp_user_base_addr + offset;
        for(i=0; i<size; i+=sizeof(unsigned))
        {
                tcc_otp_writel(*buf++, base + i);
        }

        //set write protection state
        data |= HwOTP_CTRL_WP;
        tcc_otp_writel(data, tcc_otp_wrapper_base_addr + OFFSET_OTPCTRL);

        return 0;
}

static long tcc_otp_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{

        struct tcc_otp_ioctl_param param;
        long ret = 0;

        dprintk("[%s] cmd=%d\n", __func__, cmd);

        mutex_lock(&tcc_otp_mutex);

        switch(cmd) {
        case TCC_OTP_IOCTL_READ:
                if(copy_from_user((void *)&param, (const void *)arg, sizeof(struct tcc_otp_ioctl_param))) {
                        eprintk("[%s] copy_from_user failed(%d)\n", __func__, cmd);
                        ret = -EFAULT;
                        break;
                }

                ret = tcc_otp_read((unsigned int)param.addr, param.buf, param.size);
                break;
        case TCC_OTP_IOCTL_WRITE:
                if(copy_from_user((void *)&param, (const void *)arg, sizeof(struct tcc_otp_ioctl_param))) {
                        eprintk("[%s] copy_from_user failed(%d)\n", __func__, cmd);
                        ret = -EFAULT;
                        break;
                }

                ret = tcc_otp_write((unsigned int)param.addr, param.buf, param.size);
                break;
        default:
                eprintk("[%s] unknown command(%d)\n", __func__, cmd);
                break;
        }

        mutex_unlock(&tcc_otp_mutex);

        return ret;
}

int tcc_otp_open(struct inode *inode, struct file *filp)
{
        dprintk("%s\n", __func__);
        return 0;
}

int tcc_otp_release(struct inode *inode, struct file *file)
{
        dprintk("%s\n", __func__);
        return 0;
}

static const struct file_operations tcc_otp_fops =
{
        .owner		= THIS_MODULE,
        .open		= tcc_otp_open,
        .unlocked_ioctl	= tcc_otp_ioctl,
        .release	= tcc_otp_release,
};

static struct miscdevice otp_misc_device =
{
        .minor = MISC_DYNAMIC_MINOR,
        .name = DEVICE_NAME,
        .fops = &tcc_otp_fops,
};

static int tcc_otp_probe(struct platform_device *pdev)
{
        struct resource *res;

        res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
        if(!res) {
                printk("[%s] get resource-0 err\n", __func__);
                return -EBUSY;
        }

        tcc_otp_wrapper_base_addr = devm_ioremap_resource(&pdev->dev, res);
        if(IS_ERR(tcc_otp_wrapper_base_addr)) {
                printk("[%s] tcc_otp_wrapper_base_addr err", __func__);
                return PTR_ERR(tcc_otp_wrapper_base_addr);
        }

        dprintk("[%s] tcc otp wrapper base addr = (%08x)\n", __func__, (unsigned int)tcc_otp_wrapper_base_addr);

        res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
        if(!res) {
                printk("[%s] get resource-1 err\n", __func__);
                return -EBUSY;
        }

        tcc_otp_user_area_start = res->start;
        tcc_otp_user_area_end = res->end;

        tcc_otp_user_base_addr = devm_ioremap_resource(&pdev->dev, res);
        if(IS_ERR(tcc_otp_user_base_addr)) {
                printk("[%s] tcc_otp_user_base_addr err", __func__);
                return PTR_ERR(tcc_otp_user_base_addr);
		}

        dprintk("[%s] tcc otp user base addr = (%08x), start = (%08x), end = (%08x)\n", \
                __func__, (unsigned int)tcc_otp_user_base_addr, tcc_otp_user_area_start, tcc_otp_user_area_end);

        if(misc_register(&otp_misc_device)) {
                eprintk("[%s] register device err\n", __func__);
                return -EBUSY;
        }

        return 0;
}

static int tcc_otp_remove(struct platform_device *pdev)
{
        dprintk("%s\n", __func__);
        return 0;
}

#ifdef CONFIG_PM
static int tcc_otp_suspend(struct platform_device *pdev, pm_message_t state)
{
        dprintk("%s\n", __func__);
        return 0;
}

static int tcc_otp_resume(struct platform_device *pdev)
{
        dprintk("%s\n", __func__);
        return 0;
}
#else
#define tcc_otp_suspend NULL
#define tcc_otp_resume NULL
#endif

#ifdef CONFIG_OF
static struct of_device_id otp_of_match[] = {
        { .compatible = "telechips,tcc-otp" },
        { "", "", "", NULL },
};
MODULE_DEVICE_TABLE(of, otp_of_match);
#endif

static struct platform_driver tcc_otp_driver = {
        .driver	= {
                .name	= "tcc_otp",
                .owner 	= THIS_MODULE,
#ifdef CONFIG_OF
                .of_match_table = of_match_ptr(otp_of_match)
#endif
        },
        .probe	= tcc_otp_probe,
        .remove	= tcc_otp_remove,
#ifdef CONFIG_PM
        .suspend= tcc_otp_suspend,
        .resume	= tcc_otp_resume,
#endif
};

static int __init tcc_otp_init(void)
{
        int ret =0;

        dprintk("tcc otp driver init\n");

        ret = platform_driver_register(&tcc_otp_driver);
        if (ret) {
                eprintk("[%s] platform_driver_register err(%d) \n", __func__, ret);
                return 0;
        }

        return ret;
}

static void __exit tcc_otp_exit(void)
{
        dprintk("tcc otp driver exit\n");

	platform_driver_unregister(&tcc_otp_driver);
}

module_init(tcc_otp_init);
module_exit(tcc_otp_exit);

MODULE_AUTHOR("linux <linux@telechips.com>");
MODULE_DESCRIPTION("Telechips TCC OTP driver");
MODULE_LICENSE("GPL");

