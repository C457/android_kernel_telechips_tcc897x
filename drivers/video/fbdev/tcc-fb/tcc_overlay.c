/*
 * drivers/video/Tcc_overlay.c
 *
 * Copyright (C) 2004 Telechips, Inc. 
 *
 * Video-for-Linux (Version 2) graphic capture driver
 *
 * 
 * This package is free software; you can redistribute it and/or modify 
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation. 
 * 
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR 
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED 
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE. 
 *
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
#include <linux/clk.h>
#include <linux/videodev2.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>

#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/div64.h>
#include <asm/mach/map.h>
#include <linux/poll.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_address.h>
#endif

#include <soc/tcc/pmap.h>
#ifdef CONFIG_ARCH_TCC897X
#include <mach/bsp.h>

#include <mach/tccfb.h>
#include <mach/tcc_fb.h>
#include <mach/tcc_overlay_ioctl.h>
#include <mach/tccfb_ioctrl.h>
#include <mach/tca_display_config.h>

#include <mach/vioc_outcfg.h>
#include <mach/vioc_rdma.h>
#include <mach/of_vioc_rdma.h>
#include <mach/vioc_wdma.h>
#include <mach/of_vioc_wdma.h>
#include <mach/vioc_wmix.h>
#include <mach/of_vioc_wmix.h>
#include <mach/vioc_disp.h>
#include <mach/vioc_global.h>
#else
#include <video/tcc/tccfb.h>
#include <video/tcc/tcc_fb.h>
#include <video/tcc/tcc_overlay_ioctl.h>
#include <video/tcc/tccfb_ioctrl.h>
#include <video/tcc/tca_display_config.h>
#include <video/tcc/of_vioc_rdma.h>
#include <video/tcc/of_vioc_wdma.h>
#include <video/tcc/of_vioc_wmix.h>

#include <video/tcc/vioc_outcfg.h>
#include <video/tcc/vioc_rdma.h>
#include <video/tcc/vioc_wdma.h>
#include <video/tcc/vioc_wmix.h>
#include <video/tcc/vioc_disp.h>
#include <video/tcc/vioc_global.h>
#endif

#if 0
static int debug	   = 1;
#else
static int debug	   = 0;
#endif


#define dprintk(msg...)	if (debug) { printk( "tcc_overlay: " msg); }

#define	VIDEO_CH_NUM		3

struct overlay_drv_type {
	struct miscdevice	*misc;
	struct clk		*clk;

	struct vioc_rdma_device	*rdma;
 	struct vioc_wmix_device	*wmix;

	//extend infomation
	unsigned int fb_dd_num;
	unsigned int open_cnt;
	VIOC_IREQ_CONFIG *pIREQConfig;

	//to back up image  infomation.
	overlay_video_buffer_t overBuffCfg;
};

extern int range_is_allowed(unsigned long pfn, unsigned long size);
static int tccxxx_overlay_mmap(struct file *file, struct vm_area_struct *vma)
{
	if(range_is_allowed(vma->vm_pgoff, vma->vm_end - vma->vm_start) < 0){
		pr_err(KERN_ERR  "overlay: this address is not allowed \n");
		return -EAGAIN;
	}

	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	if(remap_pfn_range(vma,vma->vm_start, vma->vm_pgoff , vma->vm_end - vma->vm_start, vma->vm_page_prot))
		return -EAGAIN;

	vma->vm_ops	= NULL;
	vma->vm_flags 	|= VM_IO;
	vma->vm_flags 	|= VM_DONTEXPAND | VM_DONTDUMP;
	
	return 0;
}

DECLARE_WAIT_QUEUE_HEAD(overlay_wait);

static unsigned int tccxxx_overlay_poll(struct file *file, struct poll_table_struct *wait)
{
	dprintk(" tccxxx_overlay_poll wait[%d][%d]!!!\n", (unsigned)wait, (unsigned)&overlay_wait);
	poll_wait(file, &(overlay_wait), wait);	
	dprintk(" tccxxx_overlay_poll finish[%d][%d]!!!\n", (unsigned)wait, (unsigned)&overlay_wait);	
	return POLLIN;
}
extern int tcc_ctrl_ext_frame(char enable);
static int tccxxx_overlay_display_video_buffer(overlay_video_buffer_t buffer_cfg, struct overlay_drv_type *overlay_drv)
{
	int ret;

	dprintk("%s addr:0x%x  fmt : 0x%x position:%d %d  size: %d %d \n", __func__, buffer_cfg.addr, buffer_cfg.cfg.format, buffer_cfg.cfg.sx, buffer_cfg.cfg.sy, buffer_cfg.cfg.width, buffer_cfg.cfg.height);

	overlay_drv->rdma->data.aen = 0;
	overlay_drv->rdma->data.asel = 0;	
	overlay_drv->rdma->data.format = buffer_cfg.cfg.format;
	overlay_drv->rdma->data.width = buffer_cfg.cfg.width;
	overlay_drv->rdma->data.height = buffer_cfg.cfg.height;
	overlay_drv->rdma->data.offset = buffer_cfg.cfg.width;
	overlay_drv->rdma->data.base0 = buffer_cfg.addr;
	overlay_drv->rdma->data.base1 = GET_ADDR_YUV42X_spU(buffer_cfg.addr,  buffer_cfg.cfg.width, buffer_cfg.cfg.height); 
	overlay_drv->rdma->data.base2 = GET_ADDR_YUV420_spV(overlay_drv->rdma->data.base1,  buffer_cfg.cfg.width, buffer_cfg.cfg.height);

	if(buffer_cfg.cfg.format  >= VIOC_IMG_FMT_COMP)
	{
		overlay_drv->rdma->data.mode.mode = VIOC_RDMA_MODE_YUV2RGB;
		/*
		   * IM367A-279: CARPLAY_IMPROVED_DECODING_PERFORMANCE
		   * CarPlay H.264 Full range issue
		   * Y2RMD[10:9]: Studio Color HDTV(2)
		   */
		overlay_drv->rdma->data.mode.convert = 2;

		if(buffer_cfg.cfg.format == VIOC_IMG_FMT_COMP) {
			VIOC_PlugInOutCheck VIOC_PlugIn;
			ret = VIOC_CONFIG_Device_PlugState(VIOC_FCDEC0,  &VIOC_PlugIn);

			if(ret == VIOC_DEVICE_CONNECTED && !VIOC_PlugIn.enable) {
				VIOC_CONFIG_SWReset(overlay_drv->pIREQConfig, VIOC_CONFIG_FCDEC, 0, VIOC_CONFIG_RESET);
				VIOC_CONFIG_SWReset(overlay_drv->pIREQConfig, VIOC_CONFIG_FCDEC, 0, VIOC_CONFIG_CLEAR);
				VIOC_CONFIG_PlugIn(VIOC_FCDEC0, overlay_drv->rdma->id);
			}
		}
	}
	else
	{
		overlay_drv->rdma->data.mode.mode = VIOC_RDMA_MODE_NONE;
		overlay_drv->rdma->data.mode.convert = 0;
	}
	overlay_drv->rdma->data.mode.rgb_swap = 0;

	vioc_rdma_set_image(overlay_drv->rdma, 1);

	overlay_drv->wmix->pos.layer= VIDEO_CH_NUM;
	overlay_drv->wmix->pos.sx = buffer_cfg.cfg.sx;
	overlay_drv->wmix->pos.sy = buffer_cfg.cfg.sy;
	vioc_wmix_set_position(overlay_drv->wmix, 1);

#ifdef CONFIG_DISPLAY_EXT_FRAME
	tcc_ctrl_ext_frame(0);
#endif//CONFIG_DISPLAY_EXT_FRAME
	return 0;	
}


static long tccxxx_overlay_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct miscdevice	*misc = (struct miscdevice *)file->private_data;
	struct overlay_drv_type *overlay_drv = dev_get_drvdata(misc->parent);

	switch(cmd)
	{
		case OVERLAY_PUSH_VIDEO_BUFFER:
		{
			overlay_video_buffer_t overBuffCfg;

			if(copy_from_user(&overBuffCfg, (overlay_video_buffer_t *)arg, sizeof(overlay_video_buffer_t)))
				return -EFAULT;

			overlay_drv->overBuffCfg = overBuffCfg;

			return tccxxx_overlay_display_video_buffer(overBuffCfg, overlay_drv);
		}

		case OVERLAY_SET_CONFIGURE:
			{
				overlay_config_t overCfg;

				if(copy_from_user(&overCfg, (overlay_config_t *)arg, sizeof(overlay_config_t)))
					return -EFAULT;

				return 0;
			}

		default:
			pr_err(" Unsupported IOCTL(%d)!!!\n", cmd);      
			break;			
	}

	return 0;
}

static int tccxxx_overlay_release(struct inode *inode, struct file *file)
{
	int ret;
	struct miscdevice	*misc = (struct miscdevice *)file->private_data;
	struct overlay_drv_type *overlay_drv = dev_get_drvdata(misc->parent);

	overlay_drv->open_cnt--;

	dprintk(" ===========> tccxxx_overlay_release num:%d \n", overlay_drv->open_cnt);
	if(overlay_drv->open_cnt==0)
	{
		VIOC_PlugInOutCheck VIOC_PlugIn;

		vioc_rdma_set_image(overlay_drv->rdma, 0);
		ret = VIOC_CONFIG_Device_PlugState(VIOC_FCDEC0, &VIOC_PlugIn);

		if (ret == VIOC_DEVICE_CONNECTED &&
			(VIOC_PlugIn.enable && (overlay_drv->rdma->id == VIOC_PlugIn.connect_device)))
		{
			pr_info("tcc overlay drv fcdec plug out  from rdma : %d \n", overlay_drv->rdma->id);

			VIOC_CONFIG_PlugOut(VIOC_FCDEC0);

			VIOC_CONFIG_SWReset(overlay_drv->pIREQConfig, VIOC_CONFIG_FCDEC, 0, VIOC_CONFIG_RESET);
			VIOC_CONFIG_SWReset(overlay_drv->pIREQConfig, VIOC_CONFIG_FCDEC, 0, VIOC_CONFIG_CLEAR);
		}

		clk_disable_unprepare(overlay_drv->clk);

		#ifdef CONFIG_DISPLAY_EXT_FRAME
		tcc_ctrl_ext_frame(0);
		#endif
	}

	return 0;
}

static int tccxxx_overlay_open(struct inode *inode, struct file *file)
{
	struct miscdevice	*misc = (struct miscdevice *)file->private_data;
	struct overlay_drv_type *overlay_drv = dev_get_drvdata(misc->parent);

	overlay_drv->open_cnt++;
	clk_prepare_enable(overlay_drv->clk);

	dprintk(" ===========> tccxxx_overlay_open num:%d \n", overlay_drv->open_cnt);

	return 0;	
}

static const struct file_operations tcc_overlay_fops =
{
	.owner          = THIS_MODULE,
	.poll           = tccxxx_overlay_poll,
	.unlocked_ioctl = tccxxx_overlay_ioctl,
	.mmap           = tccxxx_overlay_mmap,
	.open           = tccxxx_overlay_open,
	.release        = tccxxx_overlay_release,
};


static int tcc_overlay_probe(struct platform_device *pdev)
{	
	struct device_node *config_node;
	struct overlay_drv_type *overlay_drv;
	struct device_node  *vioc_node;
	int ret = -ENODEV;

	overlay_drv = kzalloc(sizeof(struct overlay_drv_type), GFP_KERNEL);

	if(!overlay_drv)
		return -ENOMEM;
		
	overlay_drv->misc = kzalloc(sizeof(struct miscdevice), GFP_KERNEL);

	if(!overlay_drv->misc)
		goto err_overlay_drv_misc;

	vioc_node = of_parse_phandle(pdev->dev.of_node,"fbdisplay-overlay", 0);
	if(vioc_node)
		of_property_read_u32(vioc_node, "telechips,fbdisplay_num", &overlay_drv->fb_dd_num);
	else
		goto err_overlay_drv_init;


	config_node = of_find_compatible_node(NULL, NULL, "telechips,vioc_config");
		
	if (config_node) {
		overlay_drv->pIREQConfig = (PVIOC_IREQ_CONFIG)of_iomap(config_node, 0);
	} 

	overlay_drv->clk = of_clk_get(pdev->dev.of_node, 0);
	if (IS_ERR(overlay_drv->clk))
		overlay_drv->clk = NULL;


 	/* register scaler discdevice */
	overlay_drv->misc->minor = MISC_DYNAMIC_MINOR;
	overlay_drv->misc->fops = &tcc_overlay_fops;
	overlay_drv->misc->name = pdev->name;
	overlay_drv->misc->parent = &pdev->dev;
	ret = misc_register(overlay_drv->misc);
	if (ret)
		goto err_overlay_drv_init;

	overlay_drv->rdma = devm_vioc_rdma_get(&pdev->dev, overlay_drv->fb_dd_num);
	if (IS_ERR(overlay_drv->rdma)) {
		printk("could not find rdma0 node of %s driver. \n", overlay_drv->misc->name);
		overlay_drv->rdma = NULL;
	}

	overlay_drv->wmix = devm_vioc_wmix_get(&pdev->dev, overlay_drv->fb_dd_num);
	if (IS_ERR(overlay_drv->wmix)) {
		printk("could not find wmix node of %s driver. \n", overlay_drv->misc->name);
		overlay_drv->wmix = NULL;
	}
	overlay_drv->open_cnt = 0;

	platform_set_drvdata(pdev, overlay_drv);

	return 0;

err_overlay_drv_init:
	kfree(overlay_drv->misc);

err_overlay_drv_misc:
	kfree(overlay_drv);
	
	return ret;
}

static int tcc_overlay_remove(struct platform_device *pdev)
{
	struct overlay_drv_type *overlay_drv = (struct overlay_drv_type *)platform_get_drvdata(pdev);

	misc_deregister(overlay_drv->misc);

	kfree(overlay_drv->misc);
	kfree(overlay_drv);
	return 0;
}

#ifdef CONFIG_PM
static int tcc_overlay_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct overlay_drv_type *overlay_drv = (struct overlay_drv_type *)platform_get_drvdata(pdev);

	if(overlay_drv->open_cnt != 0)
	{	
		pr_info("tcc_overlay_suspend %d opened\n", overlay_drv->open_cnt);
		clk_disable_unprepare(overlay_drv->clk);
	}
	return 0;
}

static int tcc_overlay_resume(struct platform_device *pdev)
{
	struct overlay_drv_type *overlay_drv = (struct overlay_drv_type *)platform_get_drvdata(pdev);

	if(overlay_drv->open_cnt != 0)
	{	
		pr_info("tcc_overlay_resume %d opened\n", overlay_drv->open_cnt);
		clk_prepare_enable(overlay_drv->clk);
		tccxxx_overlay_display_video_buffer(overlay_drv->overBuffCfg, overlay_drv);
	}
	return 0;
}
#else //CONFIG_PM
#define tcc_overlay_suspend NULL
#define tcc_overlay_resume NULL
#endif //CONFIG_PM

#ifdef CONFIG_OF
static struct of_device_id tcc_overlay_of_match[] = {
       { .compatible = "telechips,tcc_overlay" },
       {}
};
MODULE_DEVICE_TABLE(of, tcc_overlay_of_match);
#endif


static struct platform_driver tcc_overlay_driver = {
	.probe          = tcc_overlay_probe,
	.remove         = tcc_overlay_remove,
	.suspend        = tcc_overlay_suspend,
	.resume         = tcc_overlay_resume,
	.driver         = {
	     .name   = "tcc_overlay",
	     .owner  = THIS_MODULE,
#ifdef CONFIG_OF
	     .of_match_table = of_match_ptr(tcc_overlay_of_match),
#endif
	},
};


static void __exit
tccxxx_overlay_cleanup(void)
{
	platform_driver_unregister(&tcc_overlay_driver);
}

static char banner[] __initdata = KERN_INFO "TCC Overlay driver initializing\n";

static int __init 
tccxxx_overlay_init(void)
{
	printk(banner);
	platform_driver_register(&tcc_overlay_driver);
	return 0;
}


MODULE_AUTHOR("Telechips.");
MODULE_DESCRIPTION("TCC Video for Linux overlay driver");
MODULE_LICENSE("GPL");


module_init(tccxxx_overlay_init);
module_exit(tccxxx_overlay_cleanup);

