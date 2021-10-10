/****************************************************************************
 * Copyright (C) 2015 Telechips Inc.
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
 ****************************************************************************/

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/errno.h>

#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/clk.h>
#include <linux/poll.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <asm/io.h>

#ifdef CONFIG_ARCH_TCC897X
#include <mach/bsp.h>
#include <mach/vioc_rdma.h>
#include <mach/vioc_wdma.h>
#include <mach/vioc_wmix.h>
#include <mach/vioc_config.h>
#include <mach/vioc_scaler.h>
#include <mach/vioc_api.h>
#include <mach/vioc_global.h>
#include <mach/tcc_wmixer_ioctrl.h>

#include <mach/vioc_intr.h>
#include <mach/of_vioc_rdma.h>
#include <mach/of_vioc_wmix.h>
#include <mach/of_vioc_sc.h>
#include <mach/of_vioc_wdma.h>

#include <mach/vioc_lut.h>
#include <mach/vioc_blk.h>
#include <mach/tcc_lut_ioctl.h>

#else
#include <video/tcc/vioc_rdma.h>
#include <video/tcc/vioc_wdma.h>
#include <video/tcc/vioc_wmix.h>
#include <video/tcc/vioc_config.h>
#include <video/tcc/vioc_scaler.h>
#include <video/tcc/vioc_api.h>
#include <video/tcc/vioc_global.h>
#include <video/tcc/tcc_wmixer_ioctrl.h>

#include <video/tcc/vioc_intr.h>
#include <video/tcc/of_vioc_rdma.h>
#include <video/tcc/of_vioc_wmix.h>
#include <video/tcc/of_vioc_sc.h>
#include <video/tcc/of_vioc_wdma.h>
#endif

#define WMIXER_DEBUG 		0
#define dprintk(msg...) 		if(WMIXER_DEBUG) { printk("WMIXER_DRV: " msg); }

struct wmixer_data {
	// wait for poll  
	wait_queue_head_t	poll_wq;
	spinlock_t		poll_lock;
	unsigned int		poll_count;

	// wait for ioctl command
	wait_queue_head_t	cmd_wq;
	spinlock_t		cmd_lock;
	unsigned int		cmd_count;

	struct mutex		io_mutex;
	unsigned char		block_operating;
	unsigned char		block_waiting;
	unsigned char		irq_reged;
	unsigned int		dev_opened;
};

struct wmixer_drv_type {
	struct vioc_intr_type	*vioc_intr;

	unsigned int		id;
	unsigned int		irq;

	struct miscdevice	*misc;

	struct vioc_rdma_device	*rdma0;
	struct vioc_rdma_device	*rdma1;
	struct vioc_wmix_device	*wmix;
	struct vioc_sc_device	*sc;
	struct vioc_wdma_device	*wdma;

	struct clk		*clk;
	struct wmixer_data	*data;
	WMIXER_INFO_TYPE	*info;
	WMIXER_ALPHASCALERING_INFO_TYPE	alpha_scalering;
	WMIXER_ALPHABLENDING_TYPE	alpha_blending;
	unsigned char		scaler_plug_status;
};

extern void tccxxx_GetAddress(unsigned char format, unsigned int base_Yaddr, unsigned int src_imgx, unsigned int  src_imgy,
		                                unsigned int start_x, unsigned int start_y, unsigned int* Y, unsigned int* U,unsigned int* V);
extern int range_is_allowed(unsigned long pfn, unsigned long size);
static int wmixer_drv_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct miscdevice *misc = (struct miscdevice *)filp->private_data;
	struct wmixer_drv_type *wmixer = dev_get_drvdata(misc->parent);

	if (range_is_allowed(vma->vm_pgoff, vma->vm_end - vma->vm_start) < 0) {
		printk(KERN_ERR	"%s: %s: Address range is not allowed.\n", __func__, wmixer->misc->name);
		return -EAGAIN;
	}

	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	if (remap_pfn_range(vma,vma->vm_start, vma->vm_pgoff , vma->vm_end - vma->vm_start, vma->vm_page_prot)) {
		return -EAGAIN;
	}

	vma->vm_ops		=  NULL;
	vma->vm_flags 	|= VM_IO;
	vma->vm_flags 	|= VM_DONTEXPAND | VM_PFNMAP;

	return 0;
}

static int wmixer_drv_ctrl(struct wmixer_drv_type *wmixer)
{
	WMIXER_INFO_TYPE	*wmix_info = wmixer->info;
#ifndef CONFIG_OF_VIOC
	volatile PVIOC_RDMA	pWMIX_rdma_base = (PVIOC_RDMA)wmixer->rdma0->reg;
	volatile PVIOC_WMIX	pWMIX_wmix_base = (PVIOC_WMIX)wmixer->wmix->reg;
	volatile PVIOC_WDMA	pWMIX_wdma_base = (PVIOC_WDMA)wmixer->wdma->reg;
#endif

	int ret = 0;
	unsigned int pSrcBase0 = 0, pSrcBase1 = 0, pSrcBase2 = 0;
	unsigned int pDstBase0 = 0, pDstBase1 = 0, pDstBase2 = 0;
	unsigned int crop_width;

	dprintk("%s() : name:%s \n", __func__, wmixer->misc->name);
	dprintk("Src  : addr:0x%x 0x%x 0x%x  fmt:%d \n", wmix_info->src_y_addr, wmix_info->src_u_addr, wmix_info->src_v_addr, wmix_info->src_fmt);
	dprintk("Dest : addr:0x%x 0x%x 0x%x  fmt:%d \n", wmix_info->dst_y_addr, wmix_info->dst_u_addr, wmix_info->dst_v_addr, wmix_info->dst_fmt);
	dprintk("Size : SRC_W:%d  SRC_H:%d DST_W:%d DST_H:%d. \n", wmix_info->src_img_width, wmix_info->src_img_height, wmix_info->dst_img_width, wmix_info->dst_img_height);

	if (wmixer->scaler_plug_status == 1) {
		wmixer->scaler_plug_status = 0;
#ifdef CONFIG_OF_VIOC
		vioc_sc_set_path(wmixer->sc, 0);
#else
		VIOC_API_SCALER_SetPlugOut(wmixer->sc->id);
#endif
	}

	pSrcBase0 = wmix_info->src_y_addr;
	pSrcBase1 = wmix_info->src_u_addr;
	pSrcBase2 = wmix_info->src_v_addr;
	if((wmix_info->src_img_width != (wmix_info->src_win_right - wmix_info->src_win_left)) || (wmix_info->src_img_height != (wmix_info->src_win_bottom - wmix_info->src_win_top))) {
		crop_width 				= wmix_info->src_win_right - wmix_info->src_win_left;
		wmix_info->src_win_left 	= (wmix_info->src_win_left >> 3) << 3;
		wmix_info->src_win_right 	= wmix_info->src_win_left + crop_width;
		tccxxx_GetAddress(wmix_info->src_fmt, wmix_info->src_y_addr, wmix_info->src_img_width, wmix_info->src_img_height,
						     wmix_info->src_win_left, wmix_info->src_win_top, &pSrcBase0, &pSrcBase1, &pSrcBase2);
	}

	if((wmix_info->dst_img_width != (wmix_info->dst_win_right - wmix_info->dst_win_left)) || (wmix_info->dst_img_height != (wmix_info->dst_win_bottom - wmix_info->dst_win_top))) {
		crop_width 				= wmix_info->dst_win_right - wmix_info->dst_win_left;
		wmix_info->dst_win_left 	= (wmix_info->dst_win_left >> 3) << 3;
		wmix_info->dst_win_right 	= wmix_info->dst_win_left + crop_width;
		tccxxx_GetAddress(wmix_info->dst_fmt, wmix_info->dst_y_addr, wmix_info->dst_img_width, wmix_info->dst_img_height,
						     wmix_info->dst_win_left, wmix_info->dst_win_top, &pDstBase0, &pDstBase1, &pDstBase2);
	}

	spin_lock_irq(&(wmixer->data->cmd_lock));

#ifdef CONFIG_OF_VIOC
	if (wmixer->wmix) {
		wmixer->wmix->data.width = wmix_info->dst_win_right - wmix_info->dst_win_left;
		wmixer->wmix->data.height = wmix_info->dst_win_bottom - wmix_info->dst_win_top;
		vioc_wmix_set_image(wmixer->wmix, 1);
	}

	if (wmixer->rdma0) {
		wmixer->rdma0->data.format = wmix_info->src_fmt;
		wmixer->rdma0->data.width = wmix_info->src_win_right - wmix_info->src_win_left;
		wmixer->rdma0->data.height = wmix_info->src_win_bottom - wmix_info->src_win_top;
		wmixer->rdma0->data.offset = wmix_info->src_img_width;
		wmixer->rdma0->data.base0 = pSrcBase0;
		wmixer->rdma0->data.base1 = pSrcBase1;
		wmixer->rdma0->data.base2 = pSrcBase2;
		if ((wmix_info->src_fmt > VIOC_IMG_FMT_COMP) && (wmix_info->dst_fmt > VIOC_IMG_FMT_COMP))
		{
			wmixer->rdma0->data.mode.mode = VIOC_RDMA_MODE_YUV2RGB;
			wmixer->rdma0->data.mode.rgb_swap = VIOC_WDMA_SWAP_RGB;
		}
		else if((wmix_info->src_fmt < VIOC_IMG_FMT_COMP) && (wmix_info->dst_fmt < VIOC_IMG_FMT_COMP))
		 {
			wmixer->rdma0->data.mode.mode = VIOC_RDMA_MODE_NONE;
			wmixer->rdma0->data.mode.rgb_swap = VIOC_WDMA_SWAP_RGB;
		}
		else if (wmix_info->src_fmt > VIOC_IMG_FMT_COMP && wmix_info->dst_fmt < VIOC_IMG_FMT_COMP) {
			wmixer->rdma0->data.mode.mode = VIOC_RDMA_MODE_YUV2RGB;
			wmixer->rdma0->data.mode.convert = 1;
			wmixer->rdma0->data.mode.rgb_swap = wmix_info->src_rgb_swap;
		}
		vioc_rdma_set_image(wmixer->rdma0, 1);
	}

	if (wmixer->wdma) {
		wmixer->wdma->data.format = wmix_info->dst_fmt;
		wmixer->wdma->data.width = wmix_info->dst_win_right - wmix_info->dst_win_left;
		wmixer->wdma->data.height = wmix_info->dst_win_bottom - wmix_info->dst_win_top;
		wmixer->wdma->data.offset = wmix_info->dst_img_width;
		wmixer->wdma->data.base0 = wmix_info->dst_y_addr;
		wmixer->wdma->data.base1 = wmix_info->dst_u_addr;
		wmixer->wdma->data.base2 = wmix_info->dst_v_addr;
		wmixer->wdma->data.cont = 0;
		if ((wmix_info->src_fmt > VIOC_IMG_FMT_COMP) && (wmix_info->dst_fmt > VIOC_IMG_FMT_COMP))	{
			wmixer->wdma->data.mode.mode = VIOC_WDMA_MODE_RGB2YUV;
			wmixer->wdma->data.mode.rgb_swap = VIOC_WDMA_SWAP_RGB;
		}
		else if((wmix_info->src_fmt < VIOC_IMG_FMT_COMP) && (wmix_info->dst_fmt < VIOC_IMG_FMT_COMP)) {
			wmixer->wdma->data.mode.mode = VIOC_WDMA_MODE_NONE;
			wmixer->wdma->data.mode.rgb_swap = VIOC_WDMA_SWAP_RGB;
		}
		else if ( wmix_info->src_fmt < VIOC_IMG_FMT_COMP && wmix_info->dst_fmt > VIOC_IMG_FMT_COMP ) {
			wmixer->wdma->data.mode.mode = VIOC_WDMA_MODE_RGB2YUV;
			wmixer->wdma->data.mode.convert = 1;
			wmixer->wdma->data.mode.rgb_swap = wmix_info->dst_rgb_swap;
		}

		vioc_wdma_set_image(wmixer->wdma, 1);
	}
#else
	// set to RDMA
	VIOC_RDMA_SetImageFormat(pWMIX_rdma_base, wmix_info->src_fmt);
	VIOC_RDMA_SetImageSize(pWMIX_rdma_base, (wmix_info->src_win_right - wmix_info->src_win_left), (wmix_info->src_win_bottom - wmix_info->src_win_top));
	VIOC_RDMA_SetImageOffset(pWMIX_rdma_base, wmix_info->src_fmt, wmix_info->src_img_width);
	VIOC_RDMA_SetImageBase(pWMIX_rdma_base, pSrcBase0, pSrcBase1,  pSrcBase2);

	// set to WMIX
	VIOC_WMIX_SetSize(pWMIX_wmix_base, (wmix_info->dst_win_right - wmix_info->dst_win_left), (wmix_info->dst_win_bottom - wmix_info->dst_win_top));
	VIOC_WMIX_SetUpdate(pWMIX_wmix_base);

	// set to RDMA
	if (((wmix_info->src_fmt > VIOC_IMG_FMT_COMP) && (wmix_info->dst_fmt > VIOC_IMG_FMT_COMP))
		|| ((wmix_info->src_fmt < VIOC_IMG_FMT_COMP) && (wmix_info->dst_fmt < VIOC_IMG_FMT_COMP))) {
			VIOC_RDMA_SetImageRGBSwapMode(pWMIX_rdma_base, 0);
			VIOC_RDMA_SetImageY2REnable(pWMIX_rdma_base, 0);
	} else {
		if (wmix_info->src_fmt > VIOC_IMG_FMT_COMP && wmix_info->dst_fmt < VIOC_IMG_FMT_COMP) {
			VIOC_RDMA_SetImageRGBSwapMode(pWMIX_rdma_base, wmix_info->src_rgb_swap);
			VIOC_RDMA_SetImageY2REnable(pWMIX_rdma_base, 1);
			VIOC_RDMA_SetImageY2RMode(pWMIX_rdma_base, 1); // 0 ~ 3
		}
	}
	VIOC_RDMA_SetImageEnable(pWMIX_rdma_base); // Soc guide info.

	VIOC_WDMA_SetImageFormat(pWMIX_wdma_base, wmix_info->dst_fmt);
	VIOC_WDMA_SetImageSize(pWMIX_wdma_base, (wmix_info->dst_win_right - wmix_info->dst_win_left), (wmix_info->dst_win_bottom - wmix_info->dst_win_top));
	VIOC_WDMA_SetImageOffset(pWMIX_wdma_base, wmix_info->dst_fmt, wmix_info->dst_img_width);

	VIOC_WDMA_SetImageBase(pWMIX_wdma_base, wmix_info->dst_y_addr, wmix_info->dst_u_addr, wmix_info->dst_v_addr);

	if (((wmix_info->src_fmt > VIOC_IMG_FMT_COMP) && (wmix_info->dst_fmt > VIOC_IMG_FMT_COMP))
		|| ((wmix_info->src_fmt < VIOC_IMG_FMT_COMP) && (wmix_info->dst_fmt < VIOC_IMG_FMT_COMP))) {
			VIOC_WDMA_SetImageRGBSwapMode(pWMIX_wdma_base, 0);
			VIOC_WDMA_SetImageR2YEnable(pWMIX_wdma_base, 0);
	} else {
		if ( wmix_info->src_fmt < VIOC_IMG_FMT_COMP && wmix_info->dst_fmt > VIOC_IMG_FMT_COMP ) {
			VIOC_WDMA_SetImageRGBSwapMode(pWMIX_wdma_base, wmix_info->dst_rgb_swap);
			VIOC_WDMA_SetImageR2YEnable(pWMIX_wdma_base, 1);
			VIOC_WDMA_SetImageR2YMode(pWMIX_wdma_base, 1);
		}
	}


	VIOC_WDMA_SetImageEnable(pWMIX_wdma_base, 0/*OFF*/);
	vioc_intr_clear(wmixer->vioc_intr->id, wmixer->vioc_intr->bits);
#endif
	spin_unlock_irq(&(wmixer->data->cmd_lock));

	if (wmix_info->rsp_type == WMIXER_POLLING) {
		ret = wait_event_interruptible_timeout(wmixer->data->poll_wq, wmixer->data->block_operating == 0, msecs_to_jiffies(500));
		if (ret <= 0) {
			wmixer->data->block_operating = 0;
			printk("%s(): %s time-out: %d, line: %d. \n", __func__, wmixer->misc->name, ret, __LINE__);
		}
	}

	return ret;
}

static int wmixer_drv_alpha_scaling_ctrl(struct wmixer_drv_type *wmixer)
{
	WMIXER_ALPHASCALERING_INFO_TYPE	*aps_info = &wmixer->alpha_scalering;
#ifndef CONFIG_OF_VIOC
	volatile PVIOC_RDMA	pWMIX_rdma_base = (PVIOC_RDMA)wmixer->rdma0->reg;
	volatile PVIOC_WMIX	pWMIX_wmix_base = (PVIOC_WMIX)wmixer->wmix->reg;
	volatile PVIOC_WDMA	pWMIX_wdma_base = (PVIOC_WDMA)wmixer->wdma->reg;
	VIOC_SCALER_INFO_Type	sc_info;
#endif

	int ret = 0;
	unsigned int pSrcBase0 = 0, pSrcBase1 = 0, pSrcBase2 = 0;
	unsigned int pDstBase0 = 0, pDstBase1 = 0, pDstBase2 = 0;
	unsigned int crop_width;

	dprintk("%s() : nane: %s\n", __func__, wmixer->misc->name);
	dprintk("Src  : addr:0x%x, 0x%x, 0x%x,  fmt:%d. \n", aps_info->src_y_addr, aps_info->src_u_addr, aps_info->src_v_addr, aps_info->src_fmt);
	dprintk("Dst  : addr:0x%x, 0x%x, 0x%x,  fmt:%d. \n", aps_info->dst_y_addr, aps_info->dst_u_addr, aps_info->dst_v_addr, aps_info->dst_fmt);
	dprintk("Size : src_w:%d, src_h:%d, src_crop_left:%d, src_crop_top:%d, src_crop_right:%d, src_crop_bottom:%d \n dst_w:%d, dst_h:%d, dst_crop_left:%d, dst_crop_top:%d, dst_crop_right:%d, dst_crop_bottom:%d. \n", 
				aps_info->src_img_width, aps_info->src_img_height, aps_info->src_win_left, aps_info->src_win_top, aps_info->src_win_right, aps_info->src_win_bottom,
				aps_info->dst_img_width, aps_info->dst_img_height, aps_info->dst_win_left, aps_info->dst_win_top, aps_info->dst_win_right, aps_info->dst_win_bottom);

#if defined(CONFIG_ARCH_TCC893X) //20150416 ysseung   video last-frame issue.
	if (aps_info->rsp_type == WMIXER_POLLING) {
		vioc_wdma_swreset(wmixer->wdma, 1);
		vioc_wmix_swreset(wmixer->wmix, 1);
		vioc_sc_swreset(wmixer->sc, 1);
		vioc_rdma_swreset(wmixer->rdma0, 1);
		vioc_rdma_swreset(wmixer->rdma1, 1);

		vioc_rdma_swreset(wmixer->rdma0, 0);
		vioc_rdma_swreset(wmixer->rdma1, 0);
		vioc_sc_swreset(wmixer->sc, 0);
		vioc_wmix_swreset(wmixer->wmix, 0);
		vioc_wdma_swreset(wmixer->wdma, 0);
	}
#endif

	pSrcBase0 = aps_info->src_y_addr;
	pSrcBase1 = aps_info->src_u_addr;
	pSrcBase2 = aps_info->src_v_addr;
	if ((aps_info->src_img_width != (aps_info->src_win_right - aps_info->src_win_left)) || (aps_info->src_img_height != (aps_info->src_win_bottom - aps_info->src_win_top))) {
		crop_width 				= aps_info->src_win_right - aps_info->src_win_left;
		aps_info->src_win_left 	= (aps_info->src_win_left >> 3) << 3;
		aps_info->src_win_right = aps_info->src_win_left + crop_width;
		tccxxx_GetAddress(aps_info->src_fmt, aps_info->src_y_addr, aps_info->src_img_width, aps_info->src_img_height,
						     aps_info->src_win_left, aps_info->src_win_top, &pSrcBase0, &pSrcBase1, &pSrcBase2);
	}

	if ((aps_info->dst_img_width != (aps_info->dst_win_right - aps_info->dst_win_left)) || (aps_info->dst_img_height != (aps_info->dst_win_bottom - aps_info->dst_win_top))) {
		crop_width 				= aps_info->dst_win_right - aps_info->dst_win_left;
		aps_info->dst_win_left 	= (aps_info->dst_win_left >> 3) << 3;
		aps_info->dst_win_right = aps_info->dst_win_left + crop_width;
		tccxxx_GetAddress(aps_info->dst_fmt, aps_info->dst_y_addr, aps_info->dst_img_width, aps_info->dst_img_height,
						     aps_info->dst_win_left, aps_info->dst_win_top, &pDstBase0, &pDstBase1, &pDstBase2);
	}

	spin_lock_irq(&(wmixer->data->cmd_lock));

#ifdef CONFIG_OF_VIOC
	wmixer->scaler_plug_status = 1;
	if (wmixer->sc) {
		wmixer->sc->data.bypass = 0;
		wmixer->sc->data.dst_width = aps_info->dst_win_right - aps_info->dst_win_left;
		wmixer->sc->data.dst_height = aps_info->dst_win_bottom - aps_info->dst_win_top;
		wmixer->sc->data.out_x = 0;
		wmixer->sc->data.out_y = 0;
		wmixer->sc->data.out_width = wmixer->sc->data.dst_width;
		wmixer->sc->data.out_height = wmixer->sc->data.dst_height;
		vioc_sc_set_image(wmixer->sc, 1);
	}

	if (wmixer->rdma0) {
		wmixer->rdma0->data.asel = 1;	/* pixel alpha select */
		wmixer->rdma0->data.aen = 1;	/* alpha enable */
		wmixer->rdma0->data.format = aps_info->src_fmt;
		if (aps_info->interlaced) {	/* interlaced frame process ex) MPEG2 */
			wmixer->rdma0->data.width = aps_info->src_win_right - aps_info->src_win_left;
			wmixer->rdma0->data.height = (aps_info->src_win_bottom - aps_info->src_win_top)/2;
			wmixer->rdma0->data.offset = aps_info->src_img_width*2;
		} else {
			wmixer->rdma0->data.width = aps_info->src_win_right - aps_info->src_win_left;
			wmixer->rdma0->data.height = aps_info->src_win_bottom - aps_info->src_win_top;
			wmixer->rdma0->data.offset = aps_info->src_img_width;
		}
		wmixer->rdma0->data.base0 = pSrcBase0;
		wmixer->rdma0->data.base1 = pSrcBase1;
		wmixer->rdma0->data.base2 = pSrcBase2;
		vioc_rdma_set_image(wmixer->rdma0, 1);
	}

	if (wmixer->wmix) {
		wmixer->wmix->mixmode = 0;	/* mixer op is by-pass mode */
		wmixer->wmix->data.width = aps_info->dst_win_right - aps_info->dst_win_left;
		wmixer->wmix->data.height = aps_info->dst_win_bottom - aps_info->dst_win_top;
		vioc_wmix_set_image(wmixer->wmix, 1);
	}


	if (wmixer->wdma) {
		wmixer->wdma->data.format = aps_info->dst_fmt;
		wmixer->wdma->data.width = aps_info->dst_win_right - aps_info->dst_win_left;
		wmixer->wdma->data.height = aps_info->dst_win_bottom - aps_info->dst_win_top;
		wmixer->wdma->data.offset = aps_info->dst_img_width;
		wmixer->wdma->data.base0 = aps_info->dst_y_addr;
		wmixer->wdma->data.base1 = aps_info->dst_u_addr;
		wmixer->wdma->data.base2 = aps_info->dst_v_addr;
		wmixer->wdma->data.cont = 0;
		if ((aps_info->src_fmt < SC_IMG_FMT_FCDEC) && (aps_info->dst_fmt > SC_IMG_FMT_FCDEC)) {
			wmixer->wdma->data.mode.mode = VIOC_WDMA_MODE_RGB2YUV;
		}
		vioc_wdma_set_image(wmixer->wdma, 1);
	}
#else
	VIOC_RDMA_SetImageAlphaSelect(pWMIX_rdma_base, 1);
	VIOC_RDMA_SetImageAlphaEnable(pWMIX_rdma_base, 1);
	VIOC_RDMA_SetImageFormat(pWMIX_rdma_base, aps_info->src_fmt);
	
	//interlaced frame process ex) MPEG2
	if (aps_info->interlaced) {
		VIOC_RDMA_SetImageSize(pWMIX_rdma_base, (aps_info->src_win_right - aps_info->src_win_left), (aps_info->src_win_bottom - aps_info->src_win_top)/2);
		VIOC_RDMA_SetImageOffset(pWMIX_rdma_base, aps_info->src_fmt, aps_info->src_img_width*2);
	} else {
		VIOC_RDMA_SetImageSize(pWMIX_rdma_base, (aps_info->src_win_right - aps_info->src_win_left), (aps_info->src_win_bottom - aps_info->src_win_top));
		VIOC_RDMA_SetImageOffset(pWMIX_rdma_base, aps_info->src_fmt, aps_info->src_img_width);
	}
	VIOC_RDMA_SetImageBase(pWMIX_rdma_base, pSrcBase0, pSrcBase1,  pSrcBase2);


	wmixer->scaler_plug_status = 1;
	sc_info.BYPASS 			= false /* 0 */;
	sc_info.DST_WIDTH 		= (aps_info->dst_win_right - aps_info->dst_win_left);
	sc_info.DST_HEIGHT 		= (aps_info->dst_win_bottom - aps_info->dst_win_top);
	sc_info.OUTPUT_POS_X 	= 0;
	sc_info.OUTPUT_POS_Y 	= 0;
	sc_info.OUTPUT_WIDTH 	= sc_info.DST_WIDTH;
	sc_info.OUTPUT_HEIGHT 	= sc_info.DST_HEIGHT;	
	VIOC_API_SCALER_SetConfig(wmixer->sc->id, &sc_info);
	VIOC_API_SCALER_SetPlugIn(wmixer->sc->id, wmixer->sc->path);
	VIOC_API_SCALER_SetUpdate(wmixer->sc->id);
	VIOC_RDMA_SetImageEnable(pWMIX_rdma_base); // SoC guide info.

	VIOC_CONFIG_WMIXPath(wmixer->wmix->path, 0); // wmixer op is by-pass mode.
	VIOC_WMIX_SetSize(pWMIX_wmix_base, sc_info.DST_WIDTH, sc_info.DST_HEIGHT);
	VIOC_WMIX_SetUpdate(pWMIX_wmix_base);

	VIOC_WDMA_SetImageFormat(pWMIX_wdma_base, aps_info->dst_fmt);
	VIOC_WDMA_SetImageSize(pWMIX_wdma_base, sc_info.DST_WIDTH, sc_info.DST_HEIGHT);
	VIOC_WDMA_SetImageOffset(pWMIX_wdma_base, aps_info->dst_fmt, aps_info->dst_img_width);
	VIOC_WDMA_SetImageBase(pWMIX_wdma_base, aps_info->dst_y_addr, aps_info->dst_u_addr, aps_info->dst_v_addr);
	if ((aps_info->src_fmt < SC_IMG_FMT_FCDEC) && (aps_info->dst_fmt > SC_IMG_FMT_FCDEC)) {
		VIOC_WDMA_SetImageR2YEnable(pWMIX_wdma_base, 1);
		VIOC_WDMA_SetImageY2REnable(pWMIX_wdma_base, 0);
		
	}
	else if ((aps_info->src_fmt > SC_IMG_FMT_FCDEC) && (aps_info->dst_fmt < SC_IMG_FMT_FCDEC)) {
		VIOC_WDMA_SetImageR2YEnable(pWMIX_wdma_base, 0);
		VIOC_WDMA_SetImageY2REnable(pWMIX_wdma_base, 1);
	}
	else
	{
		VIOC_WDMA_SetImageR2YEnable(pWMIX_wdma_base, 0);
		VIOC_WDMA_SetImageY2REnable(pWMIX_wdma_base, 0);	
	}

	VIOC_WDMA_SetImageEnable(pWMIX_wdma_base, 0/*OFF*/);
	vioc_intr_clear(wmixer->vioc_intr->id, wmixer->vioc_intr->bits);
#endif
	spin_unlock_irq(&(wmixer->data->cmd_lock));

	if (aps_info->rsp_type == WMIXER_POLLING) {
		ret = wait_event_interruptible_timeout(wmixer->data->poll_wq, wmixer->data->block_operating == 0, msecs_to_jiffies(500));
		if (ret <= 0) {
			wmixer->data->block_operating = 0;
			printk("%s(): %s time-out: %d, line: %d. \n", __func__, wmixer->misc->name, ret, __LINE__);
		}
	}

	return ret;
}

static int wmixer_drv_alpha_mixing_ctrl(struct wmixer_drv_type *wmixer)
{
	WMIXER_ALPHABLENDING_TYPE *apb_info = (WMIXER_ALPHABLENDING_TYPE*)&wmixer->alpha_blending;
#ifndef CONFIG_OF_VIOC
	volatile PVIOC_RDMA	pWMIX_rdma_base = (PVIOC_RDMA)wmixer->rdma0->reg;
	volatile PVIOC_RDMA	pWMIX_rdma1_base = (PVIOC_RDMA)wmixer->rdma1->reg;
	volatile PVIOC_WMIX	pWMIX_wmix_base = (PVIOC_WMIX)wmixer->wmix->reg;
	volatile PVIOC_WDMA	pWMIX_wdma_base = (PVIOC_WDMA)wmixer->wdma->reg;
	VIOC_SCALER_INFO_Type	sc_info;
#endif
	int ret = 0;

	dprintk("%s(): name:%s\n", __func__, wmixer->misc->name);

	spin_lock_irq(&(wmixer->data->cmd_lock));

#ifdef CONFIG_OF_VIOC
	wmixer->scaler_plug_status = 1;
	if (wmixer->sc) {
		wmixer->sc->data.bypass = 1;
		wmixer->sc->data.dst_width = apb_info->dst_width;
		wmixer->sc->data.dst_height = apb_info->dst_height;
		wmixer->sc->data.out_x = 0;
		wmixer->sc->data.out_y = 0;
		wmixer->sc->data.out_width = wmixer->sc->data.dst_width;
		wmixer->sc->data.out_height = wmixer->sc->data.dst_height;
		vioc_sc_set_image(wmixer->sc, 1);
	}

	if (wmixer->rdma0) {
		wmixer->rdma0->data.asel = 1;
		wmixer->rdma0->data.aen = 1;
		wmixer->rdma0->data.format = apb_info->src0_fmt;
		wmixer->rdma0->data.width = apb_info->src0_width;
		wmixer->rdma0->data.height = apb_info->src0_height;
		wmixer->rdma0->data.offset = apb_info->src0_width;
		wmixer->rdma0->data.base0 = apb_info->src0_Yaddr;
		wmixer->rdma0->data.base1 = apb_info->src0_Uaddr;
		wmixer->rdma0->data.base2 = apb_info->src0_Vaddr;
		vioc_rdma_set_image(wmixer->rdma0, 1);
	}

	if (wmixer->rdma1) {
		wmixer->rdma1->data.asel = 1;
		wmixer->rdma1->data.aen = 1;
		wmixer->rdma1->data.format = apb_info->src1_fmt;
		wmixer->rdma1->data.width = apb_info->src1_width;
		wmixer->rdma1->data.height = apb_info->src1_height;
		wmixer->rdma1->data.offset = apb_info->src1_width;
		wmixer->rdma1->data.base0 = apb_info->src1_Yaddr;
		wmixer->rdma1->data.base1 = apb_info->src1_Uaddr;
		wmixer->rdma1->data.base2 = apb_info->src1_Vaddr;
		vioc_rdma_set_image(wmixer->rdma1, 1);
	}

	if (wmixer->wmix) {
		wmixer->wmix->mixmode = 1;	/* mixer op is mixing mode */
		wmixer->wmix->data.width = apb_info->dst_width;
		wmixer->wmix->data.height = apb_info->dst_height;
		if (1) { /* set to layer0 of WMIX */
			wmixer->wmix->data.layer = apb_info->src0_layer;
			wmixer->wmix->data.region = apb_info->region;
			wmixer->wmix->data.acon0 = apb_info->src0_acon0;
			wmixer->wmix->data.acon1 = apb_info->src0_acon1;
			wmixer->wmix->data.ccon0 = apb_info->src0_ccon0;
			wmixer->wmix->data.ccon1 = apb_info->src0_ccon1;
			wmixer->wmix->data.mode = apb_info->src0_rop_mode;
			wmixer->wmix->data.asel = apb_info->src0_asel;
			wmixer->wmix->data.alpha0 = apb_info->src0_alpha0;
			wmixer->wmix->data.alpha1 = apb_info->src0_alpha1;
		}
		else {
			wmixer->wmix->data.layer = apb_info->src1_layer;
			wmixer->wmix->data.region = apb_info->region;
			wmixer->wmix->data.acon0 = apb_info->src1_acon0;
			wmixer->wmix->data.acon1 = apb_info->src1_acon1;
			wmixer->wmix->data.ccon0 = apb_info->src1_ccon0;
			wmixer->wmix->data.ccon1 = apb_info->src1_ccon1;
			wmixer->wmix->data.mode = apb_info->src1_rop_mode;
			wmixer->wmix->data.asel = apb_info->src1_asel;
			wmixer->wmix->data.alpha0 = apb_info->src1_alpha0;
			wmixer->wmix->data.alpha1 = apb_info->src1_alpha1;
		}
		vioc_wmix_set_image(wmixer->wmix, 1);
	}

	if (wmixer->wdma) {
		wmixer->wdma->data.format = apb_info->dst_fmt;
		wmixer->wdma->data.width = apb_info->dst_width;
		wmixer->wdma->data.height = apb_info->dst_height;
		wmixer->wdma->data.offset = apb_info->dst_width;
		wmixer->wdma->data.base0 = apb_info->dst_Yaddr;
		wmixer->wdma->data.base1 = apb_info->dst_Uaddr;
		wmixer->wdma->data.base2 = apb_info->dst_Vaddr;
		wmixer->wdma->data.cont = 0;
		vioc_wdma_set_image(wmixer->wdma, 1);
	}
#else
	VIOC_RDMA_SetImageAlphaEnable(pWMIX_rdma_base, 1);
	VIOC_RDMA_SetImageAlphaSelect(pWMIX_rdma_base, 1);
	VIOC_RDMA_SetImageFormat(pWMIX_rdma_base, apb_info->src0_fmt);
	VIOC_RDMA_SetImageSize(pWMIX_rdma_base, apb_info->src0_width, apb_info->src0_height);
	VIOC_RDMA_SetImageOffset(pWMIX_rdma_base, apb_info->src0_fmt, apb_info->src0_width);
	VIOC_RDMA_SetImageBase(pWMIX_rdma_base, apb_info->src0_Yaddr, apb_info->src0_Uaddr, apb_info->src0_Vaddr);

	VIOC_RDMA_SetImageAlphaEnable(pWMIX_rdma1_base, 1);
	VIOC_RDMA_SetImageAlphaSelect(pWMIX_rdma1_base, 1);
	VIOC_RDMA_SetImageFormat(pWMIX_rdma1_base, apb_info->src1_fmt);
	VIOC_RDMA_SetImageSize(pWMIX_rdma1_base, apb_info->src1_width, apb_info->src1_height);
	VIOC_RDMA_SetImageOffset(pWMIX_rdma1_base, apb_info->src1_fmt, apb_info->src1_width);
	VIOC_RDMA_SetImageBase(pWMIX_rdma1_base, apb_info->src1_Yaddr, apb_info->src1_Uaddr, apb_info->src1_Vaddr);

	wmixer->scaler_plug_status = 1;
	sc_info.BYPASS 			= true;
	sc_info.DST_WIDTH 		= apb_info->dst_width;
	sc_info.DST_HEIGHT 		= apb_info->dst_height;
	sc_info.OUTPUT_POS_X 	= 0;
	sc_info.OUTPUT_POS_Y 		= 0;
	sc_info.OUTPUT_WIDTH 	= sc_info.DST_WIDTH;
	sc_info.OUTPUT_HEIGHT 	= sc_info.DST_HEIGHT;	
	VIOC_API_SCALER_SetConfig(wmixer->sc->id, &sc_info);
	VIOC_API_SCALER_SetPlugIn(wmixer->sc->id, wmixer->sc->path);
	VIOC_API_SCALER_SetUpdate(wmixer->sc->id);
	VIOC_RDMA_SetImageEnable(pWMIX_rdma_base); // SoC guide info.
	VIOC_RDMA_SetImageEnable(pWMIX_rdma1_base);

	VIOC_CONFIG_WMIXPath(wmixer->wmix->path, 1); // wmixer op is mixing mode.
	VIOC_WMIX_SetSize(pWMIX_wmix_base, apb_info->dst_width, apb_info->dst_height);
	// set to layer0 of WMIX.
	VIOC_API_WMIX_SetOverlayAlphaValueControl(pWMIX_wmix_base, apb_info->src0_layer, apb_info->region, apb_info->src0_acon0, apb_info->src0_acon1);
	VIOC_API_WMIX_SetOverlayAlphaColorControl(pWMIX_wmix_base, apb_info->src0_layer, apb_info->region, apb_info->src0_ccon0, apb_info->src0_ccon1);
	VIOC_API_WMIX_SetOverlayAlphaROPMode(pWMIX_wmix_base, apb_info->src0_layer, apb_info->src0_rop_mode);
	VIOC_API_WMIX_SetOverlayAlphaSelection(pWMIX_wmix_base, apb_info->src0_layer, apb_info->src0_asel);
	VIOC_API_WMIX_SetOverlayAlphaValue(pWMIX_wmix_base, apb_info->src0_layer, apb_info->src0_alpha0, apb_info->src0_alpha1);
	// set to layer1 of WMIX.
	//VIOC_API_WMIX_SetOverlayAlphaValueControl(pWMIX_wmix_base, apb_info->src1_layer, apb_info->region, apb_info->src1_acon0, apb_info->src1_acon1);
	//VIOC_API_WMIX_SetOverlayAlphaColorControl(pWMIX_wmix_base, apb_info->src1_layer, apb_info->region, apb_info->src1_ccon0, apb_info->src1_ccon1);
	//VIOC_API_WMIX_SetOverlayAlphaROPMode(pWMIX_wmix_base, apb_info->src1_layer, apb_info->src1_rop_mode);
	//VIOC_API_WMIX_SetOverlayAlphaSelection(pWMIX_wmix_base, apb_info->src1_layer, apb_info->src1_asel);
	//VIOC_API_WMIX_SetOverlayAlphaValue(pWMIX_wmix_base, apb_info->src1_layer, apb_info->src1_alpha0, apb_info->src1_alpha1);
	// update WMIX.
	VIOC_WMIX_SetUpdate(pWMIX_wmix_base);

	VIOC_WDMA_SetImageFormat(pWMIX_wdma_base, apb_info->dst_fmt);
	VIOC_WDMA_SetImageSize(pWMIX_wdma_base, apb_info->dst_width, apb_info->dst_height);
	VIOC_WDMA_SetImageOffset(pWMIX_wdma_base, apb_info->dst_fmt, apb_info->dst_width);
	VIOC_WDMA_SetImageBase(pWMIX_wdma_base, apb_info->dst_Yaddr, apb_info->dst_Uaddr, apb_info->dst_Vaddr);
	VIOC_WDMA_SetImageEnable(pWMIX_wdma_base, 0 /* OFF */);
	vioc_intr_clear(wmixer->vioc_intr->id, wmixer->vioc_intr->bits);
#endif

	spin_unlock_irq(&(wmixer->data->cmd_lock));

	if (apb_info->rsp_type == WMIXER_POLLING) {
		ret = wait_event_interruptible_timeout(wmixer->data->poll_wq, wmixer->data->block_operating == 0, msecs_to_jiffies(500));
		if (ret <= 0) {
			wmixer->data->block_operating = 0;
			printk("%s(): %s time-out: %d, line: %d. \n", __func__, wmixer->misc->name, ret, __LINE__);
		}
	}

	return ret;
}

static unsigned int wmixer_drv_poll(struct file *filp, poll_table *wait)
{
	struct miscdevice *misc = (struct miscdevice *)filp->private_data;
	struct wmixer_drv_type *wmixer = dev_get_drvdata(misc->parent);
	int ret = 0;

	if (wmixer->data == NULL)
		return -EFAULT;

	poll_wait(filp, &(wmixer->data->poll_wq), wait);
	spin_lock_irq(&(wmixer->data->poll_lock));

	if (wmixer->data->block_operating == 0)
		ret = (POLLIN|POLLRDNORM);

	spin_unlock_irq(&(wmixer->data->poll_lock));

	return ret;
}

static irqreturn_t wmixer_drv_handler(int irq, void *client_data)
{
	struct wmixer_drv_type *wmixer = (struct wmixer_drv_type *)client_data;

	if (is_vioc_intr_activatied(wmixer->vioc_intr->id, wmixer->vioc_intr->bits) == false)
		return IRQ_NONE;
	vioc_intr_clear(wmixer->vioc_intr->id, wmixer->vioc_intr->bits);

	if (wmixer->data->block_operating >= 1) 	
		wmixer->data->block_operating = 0;
		
	wake_up_interruptible(&(wmixer->data->poll_wq));

	if (wmixer->data->block_waiting) 	
		wake_up_interruptible(&wmixer->data->cmd_wq);

	return IRQ_HANDLED;
}

static long wmixer_drv_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct miscdevice *misc = (struct miscdevice *)filp->private_data;
	struct wmixer_drv_type *wmixer = dev_get_drvdata(misc->parent);
	WMIXER_INFO_TYPE *wmix_info = wmixer->info;
	WMIXER_ALPHASCALERING_INFO_TYPE *alpha_scalering = &wmixer->alpha_scalering;
	WMIXER_ALPHABLENDING_TYPE *alpha_blending = &wmixer->alpha_blending;

	int ret = 0;

	dprintk("%s(): %s: cmd(%d) id:%d, block_operating(%d), block_waiting(%d), cmd_count(%d), poll_count(%d). \n", \
		__func__, wmixer->misc->name, cmd, wmixer->id, \
		wmixer->data->block_operating, wmixer->data->block_waiting, wmixer->data->cmd_count, wmixer->data->poll_count);

	switch(cmd) {
		case TCC_WMIXER_IOCTRL:
		case TCC_WMIXER_IOCTRL_KERNEL:
		case TCC_WMIXER_LUT_IOCTRL:
			mutex_lock(&wmixer->data->io_mutex);
			if(wmixer->data->block_operating) {
				wmixer->data->block_waiting = 1;
				ret = wait_event_interruptible_timeout(wmixer->data->cmd_wq, wmixer->data->block_operating == 0, msecs_to_jiffies(200));
				if(ret <= 0) {
					wmixer->data->block_operating = 0;
					printk("[%d]: %s: timed_out block_operation:%d!! cmd_count:%d \n", ret, wmixer->misc->name, wmixer->data->block_waiting, wmixer->data->cmd_count);
				}
				ret = 0;
			}

			if(cmd == TCC_WMIXER_IOCTRL_KERNEL){
				memcpy(wmix_info,(WMIXER_INFO_TYPE*)arg, sizeof(WMIXER_INFO_TYPE));
			}else{
				if(copy_from_user(wmix_info, (WMIXER_INFO_TYPE *)arg, sizeof(WMIXER_INFO_TYPE))) {
					printk(KERN_ALERT "Not Supported copy_from_user(%d). \n", cmd);
					ret = -EFAULT;
				}
			}

			if(ret >= 0) {
				if(wmixer->data->block_operating >= 1) {
					printk("scaler + :: block_operating(%d) - block_waiting(%d) - cmd_count(%d) - poll_count(%d)!!!\n", \
								wmixer->data->block_operating, wmixer->data->block_waiting, wmixer->data->cmd_count, wmixer->data->poll_count);
				}

				wmixer->data->block_waiting = 0;
				wmixer->data->block_operating = 1;

				if(cmd == TCC_WMIXER_LUT_IOCTRL &&  wmixer->id == 1) {
					tcc_set_lut_plugin(TVC_LUT(LUT_COMP0), TVC_RDMA(wmixer->rdma0->id));
					tcc_set_lut_enable(TVC_LUT(LUT_COMP0), 1);
				}

				ret = wmixer_drv_ctrl(wmixer);
				if(ret < 0) 	wmixer->data->block_operating = 0;
			}
			mutex_unlock(&wmixer->data->io_mutex);
			return ret;

		case TCC_WMIXER_ALPHA_SCALING:
		case TCC_WMIXER_ALPHA_SCALING_KERNEL:
			if (wmixer->sc == NULL)
				break;
			mutex_lock(&wmixer->data->io_mutex);
			if(wmixer->data->block_operating) {
				wmixer->data->block_waiting = 1;
				ret = wait_event_interruptible_timeout(wmixer->data->cmd_wq, wmixer->data->block_operating == 0, msecs_to_jiffies(200));
				if(ret <= 0) {
					wmixer->data->block_operating = 0;
					printk("[%d]: %s: timed_out block_operation:%d!! cmd_count:%d \n", \
						ret, wmixer->misc->name, wmixer->data->block_waiting, wmixer->data->cmd_count);
				}
				ret = 0;
			}

			if(cmd == TCC_WMIXER_ALPHA_SCALING_KERNEL)
				memcpy(alpha_scalering,(WMIXER_ALPHASCALERING_INFO_TYPE *)arg, sizeof(WMIXER_ALPHASCALERING_INFO_TYPE));
			else
			{
				if(copy_from_user((void *)alpha_scalering, (const void *)arg, sizeof(WMIXER_ALPHASCALERING_INFO_TYPE))) {
						printk(KERN_ALERT "Not Supported copy_from_user(%d). \n", cmd);
						ret = -EFAULT;
				}
			}

			if(ret >= 0) {
				if(wmixer->data->block_operating >= 1) {
					printk("scaler + :: block_operating(%d) - block_waiting(%d) - cmd_count(%d) - poll_count(%d)!!!\n", 	\
							wmixer->data->block_operating, wmixer->data->block_waiting, wmixer->data->cmd_count, wmixer->data->poll_count);
				}

				wmixer->data->block_waiting = 0;
				wmixer->data->block_operating = 1;
				ret = wmixer_drv_alpha_scaling_ctrl(wmixer);
				if(ret < 0) 	wmixer->data->block_operating = 0;
			}			
			mutex_unlock(&wmixer->data->io_mutex);
			return ret;
			
		case TCC_WMIXER_VIOC_INFO:
		case TCC_WMIXER_VIOC_INFO_KERNEL:
			{
				WMIXER_VIOC_INFO wmixer_id;
				memset(&wmixer_id, 0xFF, sizeof(wmixer_id));
				wmixer_id.rdma[0] = wmixer->rdma0->id;
				wmixer_id.wdma = wmixer->wdma->id;
				wmixer_id.wmixer = wmixer->wmix->id;

				if(cmd == TCC_WMIXER_VIOC_INFO){
					if (copy_to_user((void *)arg, (void *)&wmixer_id, sizeof(wmixer_id)))
						return -EFAULT;
				}
				else
					memcpy((void *)arg, (void *)&wmixer_id, sizeof(wmixer_id));					
			}
			return 0;

		case TCC_WMIXER_ALPHA_MIXING:			
			if (wmixer->sc == NULL)
				break;
			mutex_lock(&wmixer->data->io_mutex);
			if(wmixer->data->block_operating) {
				wmixer->data->block_waiting = 1;
				ret = wait_event_interruptible_timeout(wmixer->data->cmd_wq, wmixer->data->block_operating == 0, msecs_to_jiffies(200));
				if(ret <= 0) {
					wmixer->data->block_operating = 0;
					printk("[%d]: %s: timed_out block_operation:%d!! cmd_count:%d \n", \
						ret, wmixer->misc->name, wmixer->data->block_waiting, wmixer->data->cmd_count);
				}
				ret = 0;
			}

			if(copy_from_user(alpha_blending, (WMIXER_ALPHABLENDING_TYPE *)arg, sizeof(WMIXER_ALPHABLENDING_TYPE))) {
					printk(KERN_ALERT "Not Supported copy_from_user(%d). \n", cmd);
					ret = -EFAULT;
			}

			if(ret >= 0) {
				if(wmixer->data->block_operating >= 1) {
					printk("scaler + :: block_operating(%d) - block_waiting(%d) - cmd_count(%d) - poll_count(%d)!!!\n", 	\
								wmixer->data->block_operating, wmixer->data->block_waiting, wmixer->data->cmd_count, wmixer->data->poll_count);
				}

				wmixer->data->block_waiting = 0;
				wmixer->data->block_operating = 1;
				ret = wmixer_drv_alpha_mixing_ctrl(wmixer);
				if(ret < 0) 	wmixer->data->block_operating = 0;
			}			
			mutex_unlock(&wmixer->data->io_mutex);
			return ret;

		default:
			printk(KERN_ALERT "not supported %s IOCTL(0x%x). \n", wmixer->misc->name, cmd);
			break;			
	}

	return 0;
}

static int wmixer_drv_release(struct inode *inode, struct file *filp)
{
	struct miscdevice *misc = (struct miscdevice *)filp->private_data;
	struct wmixer_drv_type *wmixer = dev_get_drvdata(misc->parent);

	int ret = 1;
	dprintk("wmixer_release IN:  %d'th, block(%d/%d), cmd(%d), irq(%d). \n", wmixer->data->dev_opened, 			\
			wmixer->data->block_operating, wmixer->data->block_waiting, wmixer->data->cmd_count, wmixer->data->irq_reged);

	if (wmixer->data->dev_opened > 0)
		wmixer->data->dev_opened--;

	if (wmixer->data->dev_opened == 0) {
		if (wmixer->data->block_operating) {
			ret = wait_event_interruptible_timeout(wmixer->data->cmd_wq, wmixer->data->block_operating == 0, msecs_to_jiffies(200));
		}

		if (ret <= 0) {
			printk("[%d]: %s timed_out block_operation: %d, cmd_count: %d. \n", \
				ret, wmixer->misc->name, wmixer->data->block_waiting, wmixer->data->cmd_count);
		}

		if(wmixer->data->irq_reged) {
			vioc_intr_disable(wmixer->vioc_intr->id, wmixer->vioc_intr->bits);
			free_irq(wmixer->irq, wmixer);
			wmixer->data->irq_reged = 0;
		}

		// VIOC_CONFIG_PlugOut(wmixer->sc->id);
		
		vioc_wdma_swreset(wmixer->wdma, 1);
		vioc_wmix_swreset(wmixer->wmix, 1);
		//vioc_sc_swreset(wmixer->sc, 1);
		vioc_rdma_swreset(wmixer->rdma0, 1);
		vioc_rdma_swreset(wmixer->rdma1, 1);

		vioc_rdma_swreset(wmixer->rdma0, 0);
		vioc_rdma_swreset(wmixer->rdma1, 0);
		//vioc_sc_swreset(wmixer->sc, 0);
		vioc_wmix_swreset(wmixer->wmix, 0);
		vioc_wdma_swreset(wmixer->wdma, 0);
		wmixer->data->block_operating = wmixer->data->block_waiting = 0;
		wmixer->data->poll_count = wmixer->data->cmd_count = 0;
	}

	if (wmixer->clk)
		clk_disable_unprepare(wmixer->clk);
	dprintk("wmixer_release OUT:  %d'th. \n", wmixer->data->dev_opened);

	return 0;
}

static int wmixer_drv_open(struct inode *inode, struct file *filp)
{	
	struct miscdevice	*misc = (struct miscdevice *)filp->private_data;
	struct wmixer_drv_type	*wmixer = dev_get_drvdata(misc->parent);

	int ret = 0;
	dprintk("%s_open IN:  %d'th, block(%d/%d), cmd(%d), irq(%d). \n", wmixer->misc->name, wmixer->data->dev_opened, 				\
			wmixer->data->block_operating, wmixer->data->block_waiting, wmixer->data->cmd_count, wmixer->data->irq_reged);

	if (wmixer->clk)
		clk_prepare_enable(wmixer->clk);

	if (!wmixer->data->irq_reged) {
		vioc_wdma_swreset(wmixer->wdma, 1);
		vioc_wmix_swreset(wmixer->wmix, 1);
		//vioc_sc_swreset(wmixer->sc, 1);
		vioc_rdma_swreset(wmixer->rdma0, 1);
		vioc_rdma_swreset(wmixer->rdma1, 1);

		vioc_rdma_swreset(wmixer->rdma0, 0);
		vioc_rdma_swreset(wmixer->rdma1, 0);
		//vioc_sc_swreset(wmixer->sc, 0);
		vioc_wmix_swreset(wmixer->wmix, 0);
		vioc_wdma_swreset(wmixer->wdma, 0);
#if defined(CONFIG_ARCH_TCC896X)|| defined(CONFIG_ARCH_TCC897X)
		// Don't STOP_REQ clear because of dual output problem(DISP FIFO under-run), when video is out.
		vioc_config_stop_req(0);
#else
		vioc_config_stop_req(1);
#endif
		synchronize_irq(wmixer->irq);

		vioc_intr_clear(wmixer->vioc_intr->id, wmixer->vioc_intr->bits);
		ret = request_irq(wmixer->irq, wmixer_drv_handler, IRQF_SHARED, wmixer->misc->name, wmixer);
		if (ret) {
			if (wmixer->clk)
				clk_disable_unprepare(wmixer->clk);
			printk("failed to aquire %s request_irq. \n", wmixer->misc->name);
			return -EFAULT;
		}
		vioc_intr_enable(wmixer->vioc_intr->id, wmixer->vioc_intr->bits);
		wmixer->data->irq_reged = 1;
	}

	wmixer->data->dev_opened++;
	dprintk("%s_open OUT:  %d'th. \n", wmixer->misc->name, wmixer->data->dev_opened);

	return ret;
}

static struct file_operations wmixer_drv_fops = {
	.owner			= THIS_MODULE,
	.unlocked_ioctl		= wmixer_drv_ioctl,
	.mmap			= wmixer_drv_mmap,
	.open			= wmixer_drv_open,
	.release		= wmixer_drv_release,
	.poll			= wmixer_drv_poll,
};

static int wmixer_drv_probe(struct platform_device *pdev)
{
	struct wmixer_drv_type *wmixer;
	int ret = -ENODEV;

	wmixer = kzalloc(sizeof(struct wmixer_drv_type), GFP_KERNEL);
	if (!wmixer)
		return -ENOMEM;

	wmixer->clk = of_clk_get(pdev->dev.of_node, 0);
	if (IS_ERR(wmixer->clk))
		wmixer->clk = NULL;

	wmixer->misc = kzalloc(sizeof(struct miscdevice), GFP_KERNEL);
	if (wmixer->misc == 0)
		goto err_misc_alloc;

	wmixer->info = kzalloc(sizeof(WMIXER_INFO_TYPE), GFP_KERNEL);
	if (wmixer->info == 0)
		goto err_info_alloc;

	wmixer->data = kzalloc(sizeof(struct wmixer_data), GFP_KERNEL);
	if (wmixer->data == 0)
		goto err_data_alloc;

	wmixer->vioc_intr = kzalloc(sizeof(struct vioc_intr_type), GFP_KERNEL);
	if (wmixer->vioc_intr == 0)
		goto err_vioc_intr_alloc;

	wmixer->scaler_plug_status = 0;

	/* register wmixer discdevice */
	wmixer->misc->minor = MISC_DYNAMIC_MINOR;
	wmixer->misc->fops = &wmixer_drv_fops;
	wmixer->misc->name = pdev->name;
	wmixer->misc->parent = &pdev->dev;
	ret = misc_register(wmixer->misc);
	if (ret)
		goto err_misc_register;

	wmixer->id = of_alias_get_id(pdev->dev.of_node, "wmixer_drv");

	wmixer->rdma0 = devm_vioc_rdma_get(&pdev->dev, 0);
	if (IS_ERR(wmixer->rdma0)) {
		printk("could not find rdma0 node of %s driver. \n", wmixer->misc->name);
		wmixer->rdma0 = NULL;
	}

	wmixer->rdma1 = devm_vioc_rdma_get(&pdev->dev, 1);
	if (IS_ERR(wmixer->rdma1)) {
		printk("could not find rdma1 node of %s driver. \n", wmixer->misc->name);
		wmixer->rdma1 = NULL;
	}

	wmixer->wmix = devm_vioc_wmix_get(&pdev->dev, 0);
	if (IS_ERR(wmixer->wmix)) {
		printk("could not find wmix node of %s driver. \n", wmixer->misc->name);
		wmixer->wmix = NULL;
	}

	wmixer->sc = devm_vioc_sc_get(&pdev->dev, 0);
	if (IS_ERR(wmixer->sc)) {
		printk("could not find sc node of %s driver. \n", wmixer->misc->name);
		wmixer->sc = NULL;
	}

	wmixer->wdma = devm_vioc_wdma_get(&pdev->dev, 0);
	if (IS_ERR(wmixer->wdma)) {
		printk("could not find wdma node of %s driver. \n", wmixer->misc->name);
		wmixer->wdma = NULL;
	}
	else {
		wmixer->irq		= wmixer->wdma->irq;
		wmixer->vioc_intr->id	= wmixer->wdma->intr->id;
		wmixer->vioc_intr->bits	= wmixer->wdma->intr->bits;
	}

	spin_lock_init(&(wmixer->data->poll_lock));
	spin_lock_init(&(wmixer->data->cmd_lock));

	mutex_init(&(wmixer->data->io_mutex));
	
	init_waitqueue_head(&(wmixer->data->poll_wq));
	init_waitqueue_head(&(wmixer->data->cmd_wq));

	platform_set_drvdata(pdev, wmixer);

	pr_info("%s: id:%d, Wmixer Driver Initialized\n", pdev->name, wmixer->id);
	return 0;

	misc_deregister(wmixer->misc);
err_misc_register:
	kfree(wmixer->vioc_intr);
err_vioc_intr_alloc:
	kfree(wmixer->data);
err_data_alloc:
	kfree(wmixer->info);
err_info_alloc:
	kfree(wmixer->misc);
err_misc_alloc:
	kfree(wmixer);

	printk("%s: %s: err ret:%d \n", __func__, pdev->name, ret);
	return ret;
}

static int wmixer_drv_remove(struct platform_device *pdev)
{
	struct wmixer_drv_type *wmixer = (struct wmixer_drv_type *)platform_get_drvdata(pdev);

	misc_deregister(wmixer->misc);
	kfree(wmixer->vioc_intr);
	kfree(wmixer->data);
	kfree(wmixer->info);
	kfree(wmixer->misc);
	kfree(wmixer);
	return 0;
}

static int wmixer_drv_suspend(struct platform_device *pdev, pm_message_t state)
{
	// TODO:
	return 0;
}

static int wmixer_drv_resume(struct platform_device *pdev)
{
	// TODO:
	return 0;
}

static struct of_device_id wmixer_of_match[] = {
	{ .compatible = "telechips,wmixer_drv" },
	{}
};
MODULE_DEVICE_TABLE(of, wmixer_of_match);

static struct platform_driver wmixer_driver = {
	.probe		= wmixer_drv_probe,
	.remove		= wmixer_drv_remove,
	.suspend	= wmixer_drv_suspend,
	.resume		= wmixer_drv_resume,
	.driver 	= {
		.name	= "wmixer_pdev",
		.owner	= THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table	= of_match_ptr(wmixer_of_match),
#endif
	},
};

static int __init wmixer_drv_init(void)
{
	return platform_driver_register(&wmixer_driver);
}

static void __exit wmixer_drv_exit(void)
{
	platform_driver_unregister(&wmixer_driver);
}

module_init(wmixer_drv_init);
module_exit(wmixer_drv_exit);

MODULE_AUTHOR("linux <linux@telechips.com>");
MODULE_DESCRIPTION("Telechips WMIXER Driver");
MODULE_LICENSE("GPL");

