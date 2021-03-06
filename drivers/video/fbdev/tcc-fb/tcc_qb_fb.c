/*********************************************************************************
 * 
 * (C) Copyright All Rights Reserved by Telechips Inc.
 *
 * *  This material is confidential and shall remain as such.
 * *  Any unauthorized use, distribution, reproduction is strictly prohibited.
 * *
 * *******************************************************************************
 * *
 * *  File Name   : tcc_qb_fb.c
 * *
 * *  Description : fb display for Quickboot system
 * *
 * *******************************************************************************
 * *
 * *  yyyy/mm/dd     ver            descriptions                Author
 * *	---------	--------   ---------------       -----------------
 * *  2013/08/21     0.1            created                     KCMin
 * *******************************************************************************/

#include <asm/uaccess.h>

#include <linux/module.h>
#include <linux/types.h>
#include <linux/fb.h>
#include <linux/vt_kern.h>
#include <linux/unistd.h>
#include <linux/syscalls.h>
#include <linux/irq.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>


#include <mach/tca_lcdc.h>
#include <mach/timex.h>
#include <mach/vioc_outcfg.h>
#include <mach/vioc_rdma.h>
#include <mach/vioc_wdma.h>
#include <mach/vioc_wmix.h>
#include <mach/vioc_disp.h>
#include <mach/vioc_config.h>

#ifdef CONFIG_DRM_TCC
unsigned int *drm_fb_virt_addr;
EXPORT_SYMBOL(drm_fb_virt_addr);
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#include <linux/fb.h>
#include <mach/tccfb.h>

#if defined(CONFIG_TCC_VIOCMG)
#include <video/tcc/viocmg.h>
#endif

static int debug = 0;
#define dprintk(msg...) if(debug) { printk("VIOC_I :" msg); }

#define fb_width(fb)	((fb)->var.xres)
#define fb_height(fb)	((fb)->var.yres)
#define fb_size(fb)	((fb)->var.xres * (fb)->var.yres * 2)

#define QUICKBOOT_IMG_BUFF_SIZE(fb) ((fb)->var.xres * (fb)->var.yres * 4)

dma_addr_t	QB_Lastframe_dma;	/* physical */
u_char *		QB_Lastframe_cpu;	/* virtual */

#if defined(CONFIG_USING_IMAGE_WHAT_YOU_WANT)
#define     QUICKBOOT_USER_LOGO     "QuickBoot_user.rle"
#endif
#define QUICKBOOT_LOGO         "QuickBoot_logo.rle"
extern void tca_fb_vsync_activate(struct tcc_dp_device *pdata);

#if defined(CONFIG_QUICK_BOOT_PROGRESS_BAR)

typedef struct
{
    int    sx;
    int    sy;
    int    width;
    int    height;
    int    addr;
}progress_bar_info_t;

progress_bar_info_t    bar;
unsigned int img_width ;
unsigned int img_height;
unsigned int QB_BAR_ADDR;
char * bar_addr_start;
#endif

extern unsigned int do_hibernation;
extern unsigned int do_hibernate_boot;

int fb_quickboot_lastframe_display(void)
{
	int err = 0;
	struct fb_info *info = registered_fb[0];
	unsigned int size = QUICKBOOT_IMG_BUFF_SIZE(info);

	printk(KERN_WARNING "~~  %s:  \n",__func__);

	QB_Lastframe_cpu = dma_alloc_writecombine(0, size, &QB_Lastframe_dma, GFP_KERNEL); //virtual 

	if(!QB_Lastframe_cpu)
	{
		printk(KERN_WARNING "%s Can not alloc back_up \n", __func__);
		err = -ENOMEM;
		goto free_memory;
	}

	dprintk(" %s back_up alloc addr = [0x%p] size = [%x] \n",__func__,QB_Lastframe_cpu, size);

	memset(QB_Lastframe_cpu, 0x0, size);
#ifdef CONFIG_DRM_TCC
	memcpy(QB_Lastframe_cpu, drm_fb_virt_addr, size);
#else
	memcpy(QB_Lastframe_cpu, info->screen_base + (info->var.xres * info->var.yoffset * (info->var.bits_per_pixel/8))  , size);
#endif
	return 0;


free_memory:
	dma_free_writecombine(0,size,QB_Lastframe_cpu, QB_Lastframe_dma);
	dprintk(" %s QB fb init failed.\n",__func__);
	return 0;
}
EXPORT_SYMBOL(fb_quickboot_lastframe_display);

void fb_quickboot_lastframe_display_release(void)
{
	struct fb_info *info;
	info = registered_fb[0];

	 dma_free_writecombine(0, info->var.xres * info->var.yres * 4, QB_Lastframe_cpu, QB_Lastframe_dma);
}
EXPORT_SYMBOL(fb_quickboot_lastframe_display_release);

static void memset32(void *_ptr, unsigned short val, unsigned count)
{

	char *ptr = _ptr;
	char b = val & 0x001f;
	char g = (val & 0x07e0)>>5;
	char r = (val & 0xf800)>>11;
	count >>= 1;
	while (count--) {
		*ptr++ = b<<3 | b>>2;
		*ptr++ = g<<2 | g>>4;
		*ptr++ = r<<3 | r>>2;
		*ptr++ = 0xff;
	}

}

#if defined(CONFIG_QUICK_BOOT_PROGRESS_BAR)
int fb_quickboot_progress_bar(int percent)
{
	char * bar_addr;
	char * bar_addr_width;
	char * bar_addr_height;

	static int mem_init = 0;
	struct fb_info *info;
	struct tccfb_info *pfbi;

	info = registered_fb[0];
	pfbi = info->par;

	if(!mem_init)	{    
		pr_info("~~  %s:  \n",__func__);

		img_width = fb_width(info);
		img_height = fb_height(info);

#ifdef INCLUDE_LCD_RESOLUTION_1280_720

		bar.sx = 320;
		bar.sy = 525;
		bar.width = 640;
		bar.height = 19;
#else
#ifdef CONFIG_DAUDIO_KK
#if 1//def CONFIG_LCD_HDMI1920X720
		//wide platform
		bar.sx = (img_width /4);
		bar.sy = (img_height / 4 ) * 3;
		bar.width = (img_width /2);
		bar.height = (img_height/10) - 20;	
#else
		bar.sx = 200;
		bar.sy = 351;
		bar.width = 400;
		bar.height = 13;
	#endif
#else
		bar.sx = (img_width /4);
		bar.sy = (img_height / 4 ) * 3;
		bar.width = (img_width /2);
		bar.height = (img_height/10) - 1;
#endif // TCC Original
#endif // 1280_720

		bar_addr_start = (char *)(info->screen_base) + (info->var.xres * info->var.yoffset * (info->var.bits_per_pixel/8));
		QB_BAR_ADDR = pfbi->map_dma + (info->var.xres * info->var.yoffset * (info->var.bits_per_pixel/8));	

		mem_init = 1;
	}

        dprintk(" @@@ img_width = %d , img_height = %d percent = %d \n", img_width, img_height, percent);
        dprintk(" @@@ (%d, %d) ~ (%d, %d) \n", bar.sx, bar.sy, bar.width, bar.height);

#ifdef CONFIG_DAUDIO_KK
		if( percent < 100 )

#else
        if( percent <= 100 )
#endif
        {
			bar_addr = (bar_addr_start + ((img_width * bar.sy * 4 ) + (bar.sx * 4)) );     
			bar_addr_width = (bar_addr + ((bar.width /100) * 4));
			bar_addr_height= (bar_addr + ( (img_width * bar.height  * 4 ) + ((bar.width / 100) * 4)));
			

			while(bar_addr < bar_addr_height)
			{
				while(bar_addr < bar_addr_width)
				{
					// color setting
					*bar_addr++ = 0x00;
					*bar_addr++ = 0x00;
					*bar_addr++ = 0xFF;
					*bar_addr++ = 0xFF;
				}
				bar_addr += (img_width - (bar.width/100)) * 4;
				bar_addr_width += (img_width)* 4;
			}
//			pfbi->p;
            VIOC_RDMA_SetImageBase(pfbi->pdata.Mdp_data.rdma_info[RDMA_FB].virt_addr, QB_BAR_ADDR, 0 , 0 );
            VIOC_RDMA_SetImageEnable(pfbi->pdata.Mdp_data.rdma_info[RDMA_FB].virt_addr);
            bar.sx += (bar.width / 100) ;
                
            dprintk(" @@@ (%d, %d) ~ (%d, %d) percent = %d\n", bar.sx, bar.sy, bar.width, bar.height, percent);

			if(pfbi->pdata.Sdp_data.FbPowerState)	{
//				tca_fb_activate_var(QB_BAR_ADDR,  &info->var, &pfbi->pdata.Sdp_data);
//				tca_fb_vsync_activate(&pfbi->pdata.Sdp_data);
			}
        }
		else if (percent == 1000 ) {
			/*		Complete progress bar. Change it's color to Green.		*/
			percent = 100;

			img_width = fb_width(info);
			img_height = fb_height(info);

#ifdef CONFIG_DAUDIO_KK
			bar.sx = 200;
			bar.sy = 351;
			bar.width = 400;
			bar.height = 13;
#else
			bar.sx = (img_width /4);
			bar.sy = (img_height / 4 ) * 3;
			bar.width = (img_width /2);
			bar.height = (img_height/10) - 1;
#endif  // TCC_Original
			
			bar_addr_start = (char *)(info->screen_base) + (info->var.xres * info->var.yoffset * (info->var.bits_per_pixel/8));
			QB_BAR_ADDR = pfbi->map_dma + (info->var.xres * info->var.yoffset * (info->var.bits_per_pixel/8));	

			bar_addr = (bar_addr_start + ((img_width * bar.sy * 4 ) + (bar.sx * 4)) );     
			bar_addr_width = (bar_addr + ((bar.width) * 4));
			bar_addr_height= (bar_addr + ( (img_width * bar.height  * 4 ) + ((bar.width / 100) * 4)));
			

			while(bar_addr < bar_addr_height)
			{
				while(bar_addr < bar_addr_width)
				{
				
					// color setting
					*(unsigned long *)bar_addr = 0xFF00FF00;	// A.R.G.B.
					bar_addr += sizeof(unsigned long);
				}
				bar_addr += (img_width - (bar.width)) * 4;
				bar_addr_width += (img_width)* 4;
			}
			VIOC_RDMA_SetImageBase(pfbi->pdata.Mdp_data.rdma_info[RDMA_FB].virt_addr, QB_BAR_ADDR, 0 , 0 );
			VIOC_RDMA_SetImageEnable(pfbi->pdata.Mdp_data.rdma_info[RDMA_FB].virt_addr);
			bar.sx += (bar.width / 100) ;
			
			dprintk("complete. @@@ (%d, %d) ~ (%d, %d) percent = %d\n", bar.sx, bar.sy, bar.width, bar.height, percent);
		}
        
    return 0;
}
EXPORT_SYMBOL(fb_quickboot_progress_bar);
#endif

#if defined(CONFIG_USING_IMAGE_WHAT_YOU_WANT)
int fb_hibernation_user_logo(char *rle_filename)
{
	int fd, err = 0;
	unsigned count;

	unsigned short *data;

	mm_segment_t oldfs;	//setting for sys_open

	oldfs = get_fs();		//setting for sys_open
	set_fs(get_ds());		//setting for sys_open

	printk(KERN_WARNING "~~  %s:  %s\n",__func__, rle_filename);

	fd = sys_open(rle_filename, O_RDONLY, 0);
	if (fd < 0) {
		printk(KERN_WARNING "%s: Can not open %s\n",
				__func__, rle_filename);
		return -ENOENT;
	}
	count = (unsigned)sys_lseek(fd, (off_t)0, 2);
	if (count == 0) {
		sys_close(fd);
		err = -EIO;
		goto err_logo_close_file;
	}
	sys_lseek(fd, (off_t)0, 0);
	data = kmalloc(count, GFP_KERNEL);
	if (!data) {
		printk(KERN_WARNING "%s: Can not alloc data\n", __func__);
		err = -ENOMEM;
		goto err_logo_close_file;
	}

	if ((unsigned)sys_read(fd, (char *)data, count) != count) {
		printk(KERN_WARNING "%s: Can not read file \n",__func__);
		err = -EIO;
		goto err_logo_free_data;
	}

err_logo_free_data:
	kfree(data);

err_logo_close_file:
	sys_close(fd);

	set_fs(oldfs);

	dprintk(KERN_WARNING "~~  %s:  %s  err:%d end \n",__func__, rle_filename, err);

	return err;
}
#endif

int fb_hibernation_logo(char *rle_filename)
{
	struct tccfb_info *fbi;
	struct fb_info *info;
	int fd, err = 0;
	unsigned count, max;

	unsigned short *data, *ptr;
	char *bits;

	unsigned int BaseAddr;

	mm_segment_t oldfs;	//setting for sys_open

	oldfs = get_fs();		//setting for sys_open
	set_fs(get_ds());		//setting for sys_open

	printk(KERN_WARNING "~~  %s:  %s\n",__func__, rle_filename);

	fd = sys_open(rle_filename, O_RDONLY, 0);
	if (fd < 0) {
		printk(KERN_WARNING "%s: Can not open %s\n",
				__func__, rle_filename);
		return -ENOENT;
	}
	count = (unsigned)sys_lseek(fd, (off_t)0, 2);
	if (count == 0) {
		sys_close(fd);
		err = -EIO;
		goto err_logo_close_file;
	}
	sys_lseek(fd, (off_t)0, 0);
	data = kmalloc(count, GFP_KERNEL);
	if (!data) {
		printk(KERN_WARNING "%s: Can not alloc data\n", __func__);
		err = -ENOMEM;
		goto err_logo_close_file;
	}

	if ((unsigned)sys_read(fd, (char *)data, count) != count) {
		printk(KERN_WARNING "%s: Can not read file \n",__func__);
		err = -EIO;
		goto err_logo_free_data;
	}

	info = registered_fb[0];

	if (!info) {
		printk(KERN_WARNING "%s: Can not access framebuffer\n",
				__func__);
		return -ENODEV;
	}

	fbi = info->par;

	max = fb_width(info) * fb_height(info);
	ptr = data;	

	bits = (char *)(info->screen_base) + (info->var.xres * info->var.yoffset * (info->var.bits_per_pixel/8));


	dprintk(KERN_WARNING "%s: registered_fb xres=[%d] yres=[%d] bpp=[%d]\n",__func__,info->var.xres,info->var.yres,info->var.bits_per_pixel );

	while (count > 3) {
		unsigned n = ptr[0];
		if (n > max)
			break;

		memset32( (char *)bits, ptr[1], n << 1);
		bits += info->var.bits_per_pixel/8 * n;
		max -= n;
		ptr += 2;
		count -= 4;
	}

	BaseAddr = fbi->map_dma + (info->var.xres * info->var.yoffset * (info->var.bits_per_pixel/8));	// physical mem address setting for hdmi

	if(fbi->pdata.Sdp_data.FbPowerState)	{
//		tca_fb_activate_var(BaseAddr,  &info->var, &fbi->pdata.Sdp_data);
//		tca_fb_vsync_activate(&fbi->pdata.Sdp_data);
	}

err_logo_free_data:
	kfree(data);

err_logo_close_file:
	sys_close(fd);

	set_fs(oldfs);

	dprintk(KERN_WARNING "~~  %s:  %s  err:%d end \n",__func__, rle_filename, err);

	return err;

}
EXPORT_SYMBOL(fb_hibernation_logo);

#define MB(x)	((x)*1024*1024)

int fb_hibernation_enter(void)
{
	struct fb_info *fbinfo = registered_fb[0];
	struct tccfb_info *info;
	unsigned int FrameSize , i , Addroffset = 0;
	info = fbinfo->par;

	// back up framebuffer 
	FrameSize = info->fb->var.xres * info->fb->var.yres_virtual * info->fb->var.bits_per_pixel/8;

	for(i = 0; i <= (FrameSize / MB(4)); i++)
	{
		unsigned int copy_size = MB(4);
		
		Addroffset = i * MB(4);
		
		if(i == (FrameSize / MB(4)))	{
			copy_size = FrameSize % MB(4);	
		}

		if(copy_size == 0)
			break;
		
		info->backup_map_cpu[i] = kzalloc(copy_size, GFP_KERNEL);

		if(info->backup_map_cpu[i])		{
			memcpy(info->backup_map_cpu[i], info->map_cpu + Addroffset, copy_size);
		}
		else	{
			pr_info("Error fb alloc :%d addr:0x%p  offset:%d \n", i, info->backup_map_cpu[i], Addroffset);
			break;
		}
	}
	printk("%s loop : %d FrameSize:%d \n", __func__, i, FrameSize);	
	
	#if defined(CONFIG_USING_LAST_FRAMEBUFFER)
	    fb_quickboot_lastframe_display();
    	#elif defined(CONFIG_USING_IMAGE_WHAT_YOU_WANT)
	    fb_hibernation_user_logo(QUICKBOOT_USER_LOGO);
	#endif

	#ifdef CONFIG_QUICK_BOOT_LOGO
	fb_hibernation_logo(QUICKBOOT_LOGO);
	#endif

	return 0;
}

int fb_quickboot_display_fb_resume(struct tccfb_info *info)
{
	struct tcc_dp_device *pdp_data = &info->pdata.Mdp_data;

	unsigned int base_addr;
	unsigned int imgch = RDMA_FB;

	base_addr = info->map_dma + (info->fb->var.xres * info->fb->var.yoffset * (info->fb->var.bits_per_pixel/8));

#if defined(CONFIG_USING_LAST_FRAMEBUFFER)
	#if defined(CONFIG_PLATFORM_STB)
	base_addr = (char *)QB_Lastframe_dma;
	#elif defined(CONFIG_PLATFORM_AVN)
	unsigned int size = QUICKBOOT_IMG_BUFF_SIZE(info->fb);
	memcpy(info->fb->screen_base + (info->fb->var.xres * info->fb->var.yoffset * (info->fb->var.bits_per_pixel/8)), QB_Lastframe_cpu, size);
	#endif
#elif defined(CONFIG_USING_IMAGE_WHAT_YOU_WANT)
	if (fb_hibernation_logo(QUICKBOOT_USER_LOGO) >= 0)
#endif
	{
		VIOC_WMIX_SetPosition(pdp_data->wmixer_info.virt_addr, imgch, 0, 0);
		VIOC_RDMA_SetImageFormat(pdp_data->rdma_info[RDMA_FB].virt_addr, TCC_LCDC_IMG_FMT_RGB888);
		VIOC_RDMA_SetImageOffset(pdp_data->rdma_info[RDMA_FB].virt_addr, TCC_LCDC_IMG_FMT_RGB888, info->fb->var.xres); 	//offset
		VIOC_RDMA_SetImageSize(pdp_data->rdma_info[RDMA_FB].virt_addr, info->fb->var.xres, info->fb->var.yres); 		//size
		VIOC_RDMA_SetImageBase(pdp_data->rdma_info[RDMA_FB].virt_addr, base_addr, 0 , 0);
		#if defined(CONFIG_PLATFORM_AVN)
        VIOC_RDMA_SetImageAlphaSelect(pdp_data->rdma_info[RDMA_FB].virt_addr, 1);
        VIOC_RDMA_SetImageAlphaEnable(pdp_data->rdma_info[RDMA_FB].virt_addr, 1);
		#endif
		VIOC_RDMA_SetImageUpdate(pdp_data->rdma_info[RDMA_FB].virt_addr);
	}
	return 0;
}
EXPORT_SYMBOL(fb_quickboot_display_fb_resume);

void fb_quickboot_resume(struct tccfb_info *info)
{
	unsigned int i = 0, FrameSize  = 0;
	struct tcc_dp_device *pdp_data = &info->pdata.Mdp_data;	

	printk("\x1b[1;38m  %s \n", __func__);
	pr_info("%s do_hibernation:%d do_hibernate_boot:%d size:%d  %d  FrameSize:%d\n", __func__,do_hibernation, do_hibernate_boot, 
					info->map_size, sizeof(info->backup_map_cpu)/sizeof(info->backup_map_cpu[0]), FrameSize);
	printk("\x1b[0m");
	
	if(do_hibernation)
	{
		if(do_hibernate_boot){

			FrameSize = info->fb->var.xres * info->fb->var.yres_virtual * info->fb->var.bits_per_pixel/8;
			
			for(i = 0; i <= (FrameSize / MB(4)); i++)
			{

				if(info->backup_map_cpu[i])
				{
					if(i == (FrameSize / MB(4)))
						memcpy(info->map_cpu + (i * MB(4)), info->backup_map_cpu[i], (FrameSize % MB(4)));
					else
						memcpy(info->map_cpu + (i * MB(4)), info->backup_map_cpu[i], MB(4));
					
					kfree(info->backup_map_cpu[i]);
				}
			}
			printk("restore frame buffer size :%d \n", i);
			
			#if defined(CONFIG_TCC_VIOCMG)
			viocmg_set_wmix_ovp(VIOCMG_CALLERID_FB, pdp_data->wmixer_info.blk_num, 8);
			#else
				#if defined(CONFIG_PLATFORM_AVN)
#if defined(CONFIG_TCC_EARLY_VIEW_CM_CONTROL)
			VIOC_WMIX_SetOverlayPriority(pdp_data->wmixer_info.virt_addr, 19);		//Image1 - Image0 - Image2 - Image3
#else
			VIOC_WMIX_SetOverlayPriority(pdp_data->wmixer_info.virt_addr, 24);		//Image0 - Image1 - Image2 - Image3
#endif /* CONFIG_TCC_EARLY_VIEW_CM_CONTROL */
				#elif defined(CONFIG_PLATFORM_STB)
			VIOC_WMIX_SetOverlayPriority(pdp_data->wmixer_info.virt_addr, 8);		//Image2 - Image0 - Image1 - Image3
				#endif
			#endif

			#if defined(CONFIG_DISPLAY_IMAGE_DURING_QUICKBOOT)        
			fb_quickboot_display_fb_resume(info);
			#endif

			VIOC_WMIX_SetUpdate(pdp_data->wmixer_info.virt_addr);	
		}
		tca_fb_vsync_activate(pdp_data);
	}	
}
EXPORT_SYMBOL(fb_quickboot_resume);
