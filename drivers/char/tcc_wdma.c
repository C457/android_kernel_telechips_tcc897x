/*
 * drivers/char/tcc_wdma.c
 *
 * Copyright (C) 2004 Texas Instruments, Inc. 
 *
 * Video-for-Linux (Version 2) graphic capture driver for
 * the OMAP H2 and H3 camera controller.
 *
 * Adapted from omap24xx driver written by Andy Lowe (source@mvista.com)
 * Copyright (C) 2003-2004 MontaVista Software, Inc.
 * 
 * This package is free software; you can redistribute it and/or modify 
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation. 
 * 
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR 
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED 
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE. 
 *
 * History:
 *   27/03/05   Vladimir Barinov - Added support for power management
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
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/poll.h>

#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/div64.h>
#include <asm/mach/map.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#endif

#include <soc/tcc/pmap.h>
#ifdef CONFIG_ARCH_TCC897X
#include <mach/bsp.h>
#include <mach/irqs.h>
#include <mach/vioc_ireq.h>
#include <mach/vioc_rdma.h>
#include <mach/vioc_wdma.h>
#include <mach/vioc_wmix.h>
#include <mach/vioc_config.h>
#include <mach/tccfb_ioctrl.h>
#include <mach/tcc_fb.h>
#include <mach/tccfb.h>
#include <mach/tca_display_config.h>
#include <mach/vioc_global.h>
#include <mach/vioc_scaler.h>
#include <mach/vioc_wmix.h>
#include <mach/vioc_api.h>
#include <mach/vioc_disp.h>
#include <mach/of_vioc_sc.h>
#include <mach/of_vioc_wmix.h>
#include <mach/of_vioc_wdma.h>

#else
#include <video/tcc/irqs.h>
#include <video/tcc/vioc_ireq.h>
#include <video/tcc/vioc_rdma.h>
#include <video/tcc/vioc_wdma.h>
#include <video/tcc/vioc_wmix.h>
#include <video/tcc/vioc_config.h>
#include <video/tcc/tccfb_ioctrl.h>
#include <video/tcc/tcc_wdma_ioctrl.h>
#include <video/tcc/tcc_fb.h>
#include <video/tcc/tccfb.h>
#include <video/tcc/tca_display_config.h>
#include <video/tcc/vioc_global.h>
#include <video/tcc/vioc_scaler.h>
#include <video/tcc/vioc_wmix.h>
#include <video/tcc/vioc_api.h>
#include <video/tcc/vioc_disp.h>
#include <video/tcc/of_vioc_sc.h>
#include <video/tcc/of_vioc_wmix.h>
#include <video/tcc/of_vioc_wdma.h>

#endif

#include "tcc_wdma.h"

#define WDMA_DEBUG 		0
#define dprintk(msg...) 	if(WDMA_DEBUG) { printk("WDMA: " msg); }

struct tcc_wdma_dev{
	struct miscdevice	*misc;

	struct tcc_vioc_block 	disp_info;
    struct vioc_rdma_device   	*rdma;
    struct vioc_sc_device   	*sc;
    struct vioc_wdma_device   *wdma;
    struct vioc_wmix_device   	*wmix;			
	// wait for poll  
	wait_queue_head_t 	poll_wq;
	spinlock_t 			poll_lock;
	unsigned int 			poll_count;

	// wait for ioctl command
	wait_queue_head_t 	cmd_wq;
	spinlock_t 			cmd_lock;
	unsigned int 			cmd_count;

	struct mutex 			io_mutex;
	unsigned char 		block_operating;
	unsigned char 		block_waiting;
	unsigned char 		irq_reged;
	unsigned int  			dev_opened;
	
	struct clk *wdma_clk;

	//extend infomation
	unsigned int fb_dd_num;
};


extern void tccxxx_GetAddress(unsigned char format, unsigned int base_Yaddr, unsigned int src_imgx, unsigned int  src_imgy,
									unsigned int start_x, unsigned int start_y, unsigned int* Y, unsigned int* U,unsigned int* V);

extern int range_is_allowed(unsigned long pfn, unsigned long size);
static int tccxxx_wdma_mmap(struct file *file, struct vm_area_struct *vma)
{
	if(range_is_allowed(vma->vm_pgoff, vma->vm_end - vma->vm_start) < 0) {
		printk(KERN_ERR	"wdma: this address is not allowed. \n");
		return -EAGAIN;
	}

	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	if(remap_pfn_range(vma,vma->vm_start, vma->vm_pgoff , vma->vm_end - vma->vm_start, vma->vm_page_prot)) {
		return -EAGAIN;
	}

	vma->vm_ops		=  NULL;
	vma->vm_flags 	|= VM_IO;
	vma->vm_flags 	|= VM_DONTEXPAND | VM_PFNMAP;

	return 0;
}

char tccxxx_wdma_ctrl(unsigned int argp, struct tcc_wdma_dev *pwdma_data)
{
	#define RGBtoYUV			(1)

	struct lcd_panel *panel;
	VIOC_WDMA_IMAGE_INFO_Type ImageCfg;
	unsigned int base_addr = 0, Wmix_Height = 0, Wmix_Width = 0, DDevice = 0;
	int ret = 0;
	int addr_Y = 0, addr_U = 0, addr_V = 0;

      memset((char *)&ImageCfg, 0x00, sizeof(ImageCfg));
	panel = tccfb_get_panel();

	if(copy_from_user( &ImageCfg, (VIOC_WDMA_IMAGE_INFO_Type *)argp, sizeof(ImageCfg))){
		pr_err("### %s error \n",__func__);
		return -EFAULT;
	}

	ImageCfg.Interlaced = 0;
	ImageCfg.ContinuousMode = 0;
	ImageCfg.SyncMode = 0;
	ImageCfg.ImgSizeWidth = panel->xres;
	ImageCfg.ImgSizeHeight = panel->yres;

	ImageCfg.ImgFormat = TCC_LCDC_IMG_FMT_YUV420SP;

	base_addr = (unsigned int )ImageCfg.BaseAddress;
	
	tccxxx_GetAddress(ImageCfg.ImgFormat, (unsigned int)base_addr, ImageCfg.TargetWidth, ImageCfg.TargetHeight,
	 						0, 0, &addr_Y, &addr_U,&addr_V);

	if(ImageCfg.ImgFormat == TCC_LCDC_IMG_FMT_YUV420SP || ImageCfg.ImgFormat == TCC_LCDC_IMG_FMT_RGB888)
	{
		addr_U = GET_ADDR_YUV42X_spU(base_addr, ImageCfg.TargetWidth, ImageCfg.TargetHeight);

		if(ImageCfg.ImgFormat == TCC_LCDC_IMG_FMT_YUV420SP)
			addr_V = GET_ADDR_YUV420_spV(addr_U, ImageCfg.TargetWidth, ImageCfg.TargetHeight);
		else
			addr_V = GET_ADDR_YUV422_spV(addr_U, ImageCfg.TargetWidth, ImageCfg.TargetHeight);
	}
	ImageCfg.BaseAddress1 = addr_U;
	ImageCfg.BaseAddress2 = addr_V;

	
	VIOC_WMIX_GetSize(pwdma_data->wmix->reg, &Wmix_Width, &Wmix_Height);

	DDevice = VIOC_DISP_Get_TurnOnOff(pwdma_data->disp_info.virt_addr);
	if((Wmix_Width ==0) || (Wmix_Height ==0) || (DDevice == 0))
	{
		pwdma_data->block_operating = 0;
		printk("Error tccxxx_wdma_ctrl W:%d H:%d DD-Power:%d \n", Wmix_Width, Wmix_Height, DDevice);
		return 0;
	}

	dprintk("src  w:%d h:%d base:0x%08x  \n",ImageCfg.ImgSizeWidth,ImageCfg.ImgSizeHeight,ImageCfg.BaseAddress);
	dprintk("dest w:%d h:%d  %d  \n",ImageCfg.TargetWidth,ImageCfg.TargetHeight, pwdma_data->fb_dd_num);
 	dprintk("wmixer size  %d %d  : base1:0x%08x  base2:0x%08x  \n",Wmix_Width, Wmix_Height, ImageCfg.BaseAddress1, ImageCfg.BaseAddress2);

	  /* scaler setting */
	pwdma_data->sc->data.bypass = 0;
	pwdma_data->sc->data.dst_width = ImageCfg.TargetWidth;
	pwdma_data->sc->data.dst_height = ImageCfg.TargetHeight;
	pwdma_data->sc->data.out_x = 0;
	pwdma_data->sc->data.out_y = 0;
	pwdma_data->sc->data.out_width = ImageCfg.TargetWidth;
	pwdma_data->sc->data.out_height = ImageCfg.TargetHeight;
	vioc_sc_set_image(pwdma_data->sc, 1);
	  
	spin_lock_irq(&(pwdma_data->cmd_lock));

	VIOC_CONFIG_PlugIn(pwdma_data->sc->id, pwdma_data->sc->path);

	if(pwdma_data->fb_dd_num)
	{
		VIOC_CONFIG_WMIXPath(WMIX10, 1 /* Mixing */);
		VIOC_CONFIG_WMIXPath(WMIX13, 1 /* Mixing */); 
	}
	else
	{
		VIOC_CONFIG_WMIXPath(WMIX00, 1 /* Mixing */);
		VIOC_CONFIG_WMIXPath(WMIX03, 1 /* Mixing */);
	}

	ImageCfg.BaseAddress = base_addr;
	
       VIOC_WDMA_SetWifiDisplayImageConfig(pwdma_data->wdma->reg, &ImageCfg, RGBtoYUV);

	pwdma_data->wdma->data.base0 = ImageCfg.BaseAddress;
	pwdma_data->wdma->data.base1 = ImageCfg.BaseAddress1;
	pwdma_data->wdma->data.base2 = ImageCfg.BaseAddress2;
	pwdma_data->wdma->data.cont = ImageCfg.ContinuousMode;
	pwdma_data->wdma->data.format = ImageCfg.ImgFormat;
	pwdma_data->wdma->data.width = ImageCfg.TargetWidth;
	pwdma_data->wdma->data.height = ImageCfg.TargetHeight;
	pwdma_data->wdma->data.offset = ImageCfg.TargetWidth;

	pwdma_data->wdma->data.mode.mode = VIOC_WDMA_MODE_RGB2YUV;
	pwdma_data->wdma->data.mode.convert = 0;
	pwdma_data->wdma->data.mode.rgb_swap = 0;	
	vioc_wdma_set_image(pwdma_data->wdma, 1);
	

	spin_unlock_irq(&(pwdma_data->cmd_lock));

	ret = wait_event_interruptible_timeout(pwdma_data->poll_wq, pwdma_data->block_operating == 0, msecs_to_jiffies(100));
	if(ret <= 0) {
		pwdma_data->block_operating = 0;
		printk("wdma time out: %d, Line: %d. \n", ret, __LINE__);
	}

	VIOC_CONFIG_PlugOut(pwdma_data->sc->id); 

   	if (copy_to_user( (VIOC_WDMA_IMAGE_INFO_Type *)argp, &ImageCfg, sizeof(ImageCfg))) {
		return -EFAULT;
	}
	
	return ret;
}

static unsigned int tccxxx_wdma_poll(struct file *filp, poll_table *wait)
{
	int ret = 0;

	struct miscdevice	*misc = (struct miscdevice *)filp->private_data;
	struct tcc_wdma_dev *wdma_data_dev = dev_get_drvdata(misc->parent);
	
	if(wdma_data_dev == NULL) 	return -EFAULT;

	poll_wait(filp, &(wdma_data_dev->poll_wq), wait);

	spin_lock_irq(&(wdma_data_dev->poll_lock));

	if(wdma_data_dev->block_operating == 0) 	ret = (POLLIN|POLLRDNORM);

	spin_unlock_irq(&(wdma_data_dev->poll_lock));

	return ret;
}

static irqreturn_t tccxxx_wdma_handler(int irq, void *client_data)
{  	
	//INTERRUPT_DATA_T *wdma_data = client_data;

	struct tcc_wdma_dev *wdma_data_dev = client_data;

	if (!is_vioc_intr_activatied(wdma_data_dev->wdma->intr->id, wdma_data_dev->wdma->intr->bits)) {
		return IRQ_NONE;
	}

	if(wdma_data_dev->block_operating >= 1)
		wdma_data_dev->block_operating = 0;

	wake_up_interruptible(&(wdma_data_dev->poll_wq));

	if(wdma_data_dev->block_waiting)
		wake_up_interruptible(&wdma_data_dev->cmd_wq);

	vioc_intr_clear(wdma_data_dev->wdma->intr->id, VIOC_WDMA_INT_MASK);

	return IRQ_HANDLED;
}

long tccxxx_wdma_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	struct miscdevice	*misc = (struct miscdevice *)file->private_data;
	struct tcc_wdma_dev *wdma_data_dev = dev_get_drvdata(misc->parent);

 	dprintk("wdma:  cmd(0x%x), block_operating(0x%x), block_waiting(0x%x), cmd_count(0x%x), poll_count(0x%x). \n", 	\
 					cmd, wdma_data_dev->block_operating, wdma_data_dev->block_waiting, wdma_data_dev->cmd_count, wdma_data_dev->poll_count);
	
	switch(cmd) {
		case TCC_WDMA_IOCTRL:
			mutex_lock(&wdma_data_dev->io_mutex);

			if(wdma_data_dev->block_operating) {
				wdma_data_dev->block_waiting = 1;
				ret = wait_event_interruptible_timeout(wdma_data_dev->cmd_wq, wdma_data_dev->block_operating == 0, msecs_to_jiffies(200));
				if(ret <= 0) {
					wdma_data_dev->block_operating = 0;
					printk("ret: %d : wdma 0 timed_out block_operation:%d!! cmd_count:%d \n", ret, wdma_data_dev->block_waiting, wdma_data_dev->cmd_count);
				}
				ret = 0;
			}

			//if(ret >= 0) {
				if(wdma_data_dev->block_operating >= 1) {
					printk("wdma driver :: block_operating(%d) - block_waiting(%d) - cmd_count(%d) - poll_count(%d)!!!\n", 	\
								wdma_data_dev->block_operating, wdma_data_dev->block_waiting, wdma_data_dev->cmd_count, wdma_data_dev->poll_count);
				}

				wdma_data_dev->block_waiting = 0;
				wdma_data_dev->block_operating = 1;

				ret = tccxxx_wdma_ctrl(arg, wdma_data_dev);// function call

				if(ret < 0)
					wdma_data_dev->block_operating = 0;
			//}

			mutex_unlock(&wdma_data_dev->io_mutex);
			return 0;

		default:
			printk(KERN_ALERT "not supported WMIXER IOCTL(0x%x). \n", cmd);
			break;			
	}

	return 0;
}
EXPORT_SYMBOL(tccxxx_wdma_ioctl);

int tccxxx_wdma_release(struct inode *inode, struct file *filp)
{
	int ret = 0;
	struct miscdevice	*misc = (struct miscdevice *)filp->private_data;
	struct tcc_wdma_dev *pwdma_data = dev_get_drvdata(misc->parent);
	
	printk("wdma_release IN:  %d'th, block(%d/%d), cmd(%d), irq(%d). \n", pwdma_data->dev_opened, 			\
			pwdma_data->block_operating, pwdma_data->block_waiting, pwdma_data->cmd_count, pwdma_data->irq_reged);

	if(pwdma_data->dev_opened > 0) 	
        pwdma_data->dev_opened--;

	if(pwdma_data->dev_opened == 0) {
		if(pwdma_data->block_operating) {
			ret = wait_event_interruptible_timeout(pwdma_data->cmd_wq, pwdma_data->block_operating == 0, msecs_to_jiffies(200));
		}

		if(ret <= 0) {
 			printk("[%d]: wdma timed_out block_operation: %d, cmd_count: %d. \n", ret, pwdma_data->block_waiting, pwdma_data->cmd_count);
		}

		if(pwdma_data->irq_reged) {
			free_irq(pwdma_data->wdma->irq, pwdma_data);
			pwdma_data->irq_reged = 0;
		}

		pwdma_data->block_operating = pwdma_data->block_waiting = 0;
		pwdma_data->poll_count = pwdma_data->cmd_count = 0;
	}
	
	clk_disable_unprepare(pwdma_data->wdma_clk);

	dprintk("wdma_release OUT:  %d'th. \n", pwdma_data->dev_opened);
	return 0;
}
EXPORT_SYMBOL(tccxxx_wdma_release);



int tccxxx_wdma_open(struct inode *inode, struct file *filp)
{	
	int ret = 0;
	struct miscdevice	*misc = (struct miscdevice *)filp->private_data;
	struct tcc_wdma_dev *pwdma_data = dev_get_drvdata(misc->parent);

	dprintk("wdma_open IN:  %d'th, block(%d/%d), cmd(%d), irq(%d). \n", pwdma_data->dev_opened, 				\
			pwdma_data->block_operating, pwdma_data->block_waiting, pwdma_data->cmd_count, pwdma_data->irq_reged);

	clk_prepare_enable(pwdma_data->wdma_clk);

	if(!pwdma_data->irq_reged) {
		vioc_intr_clear(pwdma_data->wdma->intr->id, pwdma_data->wdma->intr->bits);
		vioc_intr_enable(pwdma_data->wdma->intr->id, pwdma_data->wdma->intr->bits);

		if( request_irq(pwdma_data->wdma->irq, tccxxx_wdma_handler, IRQF_SHARED, "wdma", pwdma_data) < 0){
			pr_err("failed to aquire irq\n");
			return -EFAULT;
		}

		//if(ret) {
		//	clk_disable_unprepare(pwdma_data->wdma_clk);
		//	pr_err("failed to aquire wdma request_irq. \n");
		//	return -EFAULT;
		//}

		pwdma_data->irq_reged = 1;
	}

	pwdma_data->dev_opened++;
	
	dprintk("wdma_open OUT:  %d'th. \n", pwdma_data->dev_opened);
	return ret;	
}
EXPORT_SYMBOL(tccxxx_wdma_open);

static int tcc_wdma_parse_dt(struct platform_device *pdev, struct tcc_wdma_dev *pwdma_data)
{
	int ret = 0;
	unsigned int index;
	struct device_node *vioc_node, *disp_node;
	struct device_node *np;

	vioc_node = of_parse_phandle(pdev->dev.of_node, "wdma-fbdisplay", 0);

	if(vioc_node)
	{
		if(of_property_read_u32(vioc_node, "telechips,fbdisplay_num",&pwdma_data->fb_dd_num)){
			pr_err( "could not find  telechips,fbdisplay_nubmer\n");
			ret = -ENODEV;
		}

		if(pwdma_data->fb_dd_num)
			np = of_find_node_by_name(vioc_node, "fbdisplay1");
		else
			np = of_find_node_by_name(vioc_node, "fbdisplay0");

		if(!np){
			pr_err(" %s could not fine fb node \n",__func__);
			return -ENODEV;
		}

		disp_node = of_parse_phandle(np, "telechips,disp", 0);
		of_property_read_u32_index(np, "telechips,disp", 1, &index);

		if (!disp_node) {
			pr_err( "could not find disp node\n");
			ret = -ENODEV;
		}else {
			pwdma_data->disp_info.virt_addr= of_iomap(disp_node, index);
		}
	}

	return ret;
}


static struct file_operations tcc_wdma_fops = {
	.owner 				= THIS_MODULE,
	.unlocked_ioctl 	= tccxxx_wdma_ioctl,
	.mmap 			= tccxxx_wdma_mmap,
	.open 			= tccxxx_wdma_open,
	.release 			= tccxxx_wdma_release,
	.poll 			= tccxxx_wdma_poll,
};

static char banner[] = KERN_INFO "TCC WDMA Driver Initializing. \n";


static int  tcc_wdma_probe(struct platform_device *pdev)
{
	struct tcc_wdma_dev *wdma_data;
	int ret = -ENOMEM;

	printk(banner);

	wdma_data = kzalloc(sizeof(struct tcc_wdma_dev ), GFP_KERNEL);
	if(!wdma_data)
		return -ENOMEM;

	wdma_data->misc = kzalloc(sizeof(struct miscdevice), GFP_KERNEL);
	if (wdma_data->misc == 0)
		goto err_misc_alloc;
	
	tcc_wdma_parse_dt(pdev, wdma_data);
	
	wdma_data->misc->minor = MISC_DYNAMIC_MINOR;
	wdma_data->misc->fops = &tcc_wdma_fops;
	wdma_data->misc->name = pdev->name;
	wdma_data->misc->parent = &pdev->dev;
	ret = misc_register(wdma_data->misc);
	if (ret)
		goto err_misc_regist;	

	wdma_data->sc = devm_vioc_sc_get(&pdev->dev, wdma_data->fb_dd_num);
	dprintk("sc [id:%d irq:%d path:%d type:%d addr:0x%p] \n", wdma_data->sc->id, wdma_data->sc->irq, wdma_data->sc->path, wdma_data->sc->type, wdma_data->sc->reg);

	wdma_data->wdma = devm_vioc_wdma_get(&pdev->dev, wdma_data->fb_dd_num);
	dprintk("wdma [id:%d irq:%d addr:0x%p] \n", wdma_data->wdma->id, wdma_data->wdma->irq,  wdma_data->wdma->reg);
	
	wdma_data->wmix = devm_vioc_wmix_get(&pdev->dev, wdma_data->fb_dd_num);
	dprintk("wmix [id:%d irq:%d path:%d mixed mod:%d    addr:0x%p] \n", wdma_data->wmix->id, wdma_data->wmix->irq, wdma_data->wmix->path, 
		wdma_data->wmix->mixmode, wdma_data->wmix->reg);

	spin_lock_init(&(wdma_data->poll_lock));
	spin_lock_init(&(wdma_data->cmd_lock));

	mutex_init(&(wdma_data->io_mutex));
	
	init_waitqueue_head(&(wdma_data->poll_wq));
	init_waitqueue_head(&(wdma_data->cmd_wq));

	platform_set_drvdata(pdev, (void *)wdma_data);

	return 0;

err_misc_regist:
	kfree(wdma_data->misc);
err_misc_alloc:
	kfree(wdma_data);

	return ret;
}

static int tcc_wdma_remove(struct platform_device *pdev)
{
	struct tcc_wdma_dev *pwdma_data = (struct tcc_wdma_dev *)platform_get_drvdata(pdev);
	misc_deregister(pwdma_data->misc);
	kfree(pwdma_data->misc);
	kfree(pwdma_data);
	return 0;
}

#ifdef CONFIG_OF
static struct of_device_id tcc_wdma_of_match[] = {
	{.compatible = "telechips,tcc_wdma"},
	{}
};
MODULE_DEVICE_TABLE(of, tcc_wdma_of_match);
#endif

static struct platform_driver tcc_wdma_driver = {
	.probe 		= tcc_wdma_probe,
	.remove		= tcc_wdma_remove,
	.driver		= {
		.name	= "wdma",
		.owner	= THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(tcc_wdma_of_match),
#endif
	},
};



static int __init tcc_wdma_init(void)
{
	printk(KERN_INFO " %s\n", __func__);
	return platform_driver_register(&tcc_wdma_driver);
}

static void __exit tcc_wdma_exit(void)
{
	platform_driver_unregister(&tcc_wdma_driver);
}




MODULE_AUTHOR("Telechips.");
MODULE_DESCRIPTION("TCC WDMA Driver");
MODULE_LICENSE("GPL");


module_init(tcc_wdma_init);
module_exit(tcc_wdma_exit);


