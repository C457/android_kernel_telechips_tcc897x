/*
 * vp9_devices.c
 *
 * TCC VP9 DEVICES driver
 *
 * Copyright (C) 2013 Telechips, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/moduleparam.h>
#include <linux/device.h>
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
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/poll.h>
#include <linux/miscdevice.h>
#include <linux/of.h>

#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/div64.h>
#include <asm/mach/map.h>

#ifdef CONFIG_SUPPORT_TCC_VP9

#include "vpu_comm.h"
#include "vpu_devices.h"


extern int vp9mgr_probe(struct platform_device *pdev);
extern int vp9mgr_remove(struct platform_device *pdev);
#if defined(CONFIG_PM)
extern int vp9mgr_suspend(struct platform_device *pdev, pm_message_t state);
extern int vp9mgr_resume(struct platform_device *pdev);
#endif

#if 0 // ZzaU :: device-tree
static struct platform_device vp9mgr_device = {
	.name	= VP9MGR_NAME,
	.dev	= {},
	.id	= 0,
};
#endif

#ifdef CONFIG_OF
static struct of_device_id vp9mgr_of_match[] = {
        { .compatible = "telechips,vp9_dev_mgr" },//VP9MGR_NAME
        {}
};
MODULE_DEVICE_TABLE(of, vp9mgr_of_match);
#endif

static struct platform_driver vp9mgr_driver = {
	.probe          = vp9mgr_probe,
	.remove         = vp9mgr_remove,
#if defined(CONFIG_PM)
	.suspend        = vp9mgr_suspend,
	.resume         = vp9mgr_resume,
#endif
	.driver         = {
	     .name   	= VP9MGR_NAME,
	     .owner  	= THIS_MODULE,
#ifdef CONFIG_OF
	     .of_match_table = of_match_ptr(vp9mgr_of_match),
#endif
	},
};


static void __exit vp9mgr_cleanup(void)
{
	platform_driver_unregister(&vp9mgr_driver);
#if 0 // ZzaU :: device-tree	
	//platform_device_unregister(&vp9mgr_device);
#endif
}

static int vp9mgr_init(void)
{
	printk("============> VP9 Devices drivers initializing!!  Start ------- ");

#if 0 // ZzaU :: device-tree
	platform_device_register(&vp9mgr_device);
#endif
	platform_driver_register(&vp9mgr_driver);

	printk("Done!! \n");
	return 0;
}

module_init(vp9mgr_init);
module_exit(vp9mgr_cleanup);
#endif

MODULE_AUTHOR("Telechips.");
MODULE_DESCRIPTION("TCC vp9 devices driver");
MODULE_LICENSE("GPL");
