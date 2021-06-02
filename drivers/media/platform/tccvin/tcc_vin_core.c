/*
 * drivers/media/video/tccvin/tcc_vin_core.c
 *
 * Copyright (C) 2008 Telechips, Inc. 
 * 
 * Video-for-Linux (Version 2) camera capture driver for Telechisp SoC.
 *
 * leverage some code from CEE distribution 
 * Copyright (C) 2003-2004 MontaVista Software, Inc.
 * 
 * This package is free software; you can redistribute it and/or modify 
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation. 
 * 
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR 
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED 
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE. 
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/vmalloc.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/sched.h>
#include <linux/irq.h>
#include <asm/io.h>
#include <asm/scatterlist.h>
#include <asm/mach-types.h>

#include <mach/hardware.h>
#include <mach/irqs.h>
//#include <mach/memory.h>
#include <mach/bsp.h>
#include <mach/reg_physical.h>
#include <mach/vioc_ireq.h>
#include <mach/vioc_config.h>
#include <mach/vioc_wdma.h>
#include <mach/vioc_wmix.h>
#include <mach/vioc_scaler.h>
#include <mach/vioc_vin.h>
#include <mach/vioc_viqe.h>
#include <mach/vioc_global.h>
#include <mach/tcc_cam_ioctrl.h>
#include <mach/tccfb_ioctrl.h>

#include <linux/of.h>
#include <linux/of_irq.h>

#include <linux/of_device.h>
#include <linux/of_address.h>

#include <video/tcc/v4l2-common_tcc.h>
#include "tcc_vin_hw.h"
#include "tcc_vin_core.h"

#ifdef CONFIG_TCC_VIN_DEBUG
#define dprintk(fmt, args...) printk("\e[33m[vin_core]%s(%d) \e[0m" fmt, __func__, __LINE__, ## args);
#else
#define dprintk(fmt, args...)
#endif

void vioc_vin_path_reset(struct tcc_video_device *vdev);
static int vioc_vin_setup(struct tcc_video_device *vdev, VIOC_VIN *pVIN, VIOC_WMIX *pWMIX);
static int vioc_scaler_setup(struct tcc_video_device *vdev, VIOC_SC *pSC, VIOC_VIN *pVIN, VIOC_WMIX *pWMIX, uint sc_plugin);
static int vioc_wdma_setup(struct tcc_video_device *vdev, VIOC_WDMA *pDMA_CH, int restart);
static int tcc_cif_pmap_ctrl(int video_nr, pmap_t *pmap, unsigned int width, unsigned int height, int total_buf);

static unsigned int gtCamSizeTable[NUM_FREQS] = {
	 720 *  480, // D1
	1280 *  720, // HD - 720P
	1920 * 1080  // FHD - 1080p
};


int Get_Index_sizeTable(unsigned int Image_Size)
{
	int i;
	for (i = 0; i < NUM_FREQS; i++) {
		if (gtCamSizeTable[i] >= Image_Size) {
			return i;
		}
	}
	return (NUM_FREQS -1);
}
EXPORT_SYMBOL(Get_Index_sizeTable);

//int tccxxx_isalive_camera(void)
//{
//	return 0;
//}
//EXPORT_SYMBOL(tccxxx_isalive_camera);

static int vin_try_pfmt(unsigned int pixelformat)
{
	enum pix_fmt pfmt;

	switch (pixelformat) {
	/* YUV420 formats */
	case V4L2_PIX_FMT_YVU420:
	case V4L2_PIX_FMT_YUV420:
	case V4L2_PIX_FMT_HM12:
	case V4L2_PIX_FMT_M420:
	case V4L2_PIX_FMT_NV12:
	case V4L2_PIX_FMT_NV21:
	case V4L2_PIX_FMT_NV12M:
	case V4L2_PIX_FMT_NV12MT:
	case V4L2_PIX_FMT_YUV420M:
		pfmt = TCC_PFMT_YUV420;
		break;

	/* YUV422 formats */
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_YYUV:
	case V4L2_PIX_FMT_YVYU:
	case V4L2_PIX_FMT_UYVY:
	case V4L2_PIX_FMT_VYUY:
	case V4L2_PIX_FMT_YUV422P:
	case V4L2_PIX_FMT_NV16:
	case V4L2_PIX_FMT_NV61:
		pfmt = TCC_PFMT_YUV422;
		break;

	/* RGB formats */
	case V4L2_PIX_FMT_RGB332:
	case V4L2_PIX_FMT_RGB444:
	case V4L2_PIX_FMT_RGB555:
	case V4L2_PIX_FMT_RGB565:
	case V4L2_PIX_FMT_RGB555X:
	case V4L2_PIX_FMT_RGB565X:
	case V4L2_PIX_FMT_BGR666:
	case V4L2_PIX_FMT_BGR24:
	case V4L2_PIX_FMT_RGB24:
	case V4L2_PIX_FMT_BGR32:
	case V4L2_PIX_FMT_RGB32:
		pfmt = TCC_PFMT_RGB;
		break;

	default:
		printk(KERN_ERR "%s: Not supported pixelformat(%c%c%c%c)\n",
			__func__, fourcc2char(pixelformat));
		pfmt = -EINVAL;
		break;
	}

	return pfmt;
}
static int vin_try_bpp(unsigned int pixelformat)
{
	int bpp;

	switch (pixelformat) {
	/* RGB formats */
	case V4L2_PIX_FMT_RGB332:
		bpp = 1;
		break;
	case V4L2_PIX_FMT_RGB444:
	case V4L2_PIX_FMT_RGB555:
	case V4L2_PIX_FMT_RGB565:
	case V4L2_PIX_FMT_RGB555X:
	case V4L2_PIX_FMT_RGB565X:
		bpp = 2;
		break;
	case V4L2_PIX_FMT_BGR666:
	case V4L2_PIX_FMT_BGR24:
	case V4L2_PIX_FMT_RGB24:
		bpp = 3;
		break;
	case V4L2_PIX_FMT_BGR32:
	case V4L2_PIX_FMT_RGB32:
		bpp = 4;
		break;

	/* YUV formats */
	case V4L2_PIX_FMT_YVU420:
	case V4L2_PIX_FMT_YUV420:
	case V4L2_PIX_FMT_HM12:
	case V4L2_PIX_FMT_M420:
	case V4L2_PIX_FMT_NV12:
	case V4L2_PIX_FMT_NV21:
	case V4L2_PIX_FMT_NV12M:
	case V4L2_PIX_FMT_NV12MT:
	case V4L2_PIX_FMT_YUV420M:
	case V4L2_PIX_FMT_YUV422P:
	case V4L2_PIX_FMT_NV16:
	case V4L2_PIX_FMT_NV61:
		bpp = 1;
		break;

	/* YUV422 Sequential formats */
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_YYUV:
	case V4L2_PIX_FMT_YVYU:
	case V4L2_PIX_FMT_UYVY:
	case V4L2_PIX_FMT_VYUY:
		bpp = 2;
		break;

	default:
		printk(KERN_ERR "%s: Not supported pixelformat(%c%c%c%c)\n",
			__func__, fourcc2char(pixelformat));
		bpp = -EINVAL;
		break;
	}

	return bpp;
}


static void vin_dma_hw_reg(struct TCCxxxCIF *h, VIOC_WDMA *pWDMA, unsigned char frm_num)
{	
	h->cif_cfg.now_frame_num = frm_num;
	VIOC_WDMA_SetImageBase(pWDMA, (unsigned int)h->cif_cfg.preview_buf[frm_num].p_Y,
								  (unsigned int)h->cif_cfg.preview_buf[frm_num].p_Cb,
								  (unsigned int)h->cif_cfg.preview_buf[frm_num].p_Cr);
}

static void vioc_wdma_base_inter2prog(struct TCCxxxCIF *h, VIOC_WDMA *pWDMA, 
									unsigned char frm_num, unsigned int frm_sq, unsigned int width)
{
	img_buf_t base = {0};
	h->cif_cfg.now_frame_num = frm_num;

	/* line0: even field -> 
	 * line1:  odd field -> 
	 * line2: even ...
	 */
	if (frm_sq == FRM_SQ_EVEN) {
		/* frm_sq == 1: even field */
		base.p_Y  = h->cif_cfg.preview_buf[frm_num].p_Y;
		base.p_Cb = h->cif_cfg.preview_buf[frm_num].p_Cb;
		base.p_Cr = h->cif_cfg.preview_buf[frm_num].p_Cr;
	} else {
		/* frm_sq == 0: odd field */
		switch (h->cam_path_wdma_fmt) {
		case TCC_LCDC_IMG_FMT_YUV420SP:
		case TCC_LCDC_IMG_FMT_YUV422SP:
			base.p_Y  = h->cif_cfg.preview_buf[frm_num].p_Y + (width);
			base.p_Cb = h->cif_cfg.preview_buf[frm_num].p_Cb + (width / 2);
			base.p_Cr = h->cif_cfg.preview_buf[frm_num].p_Cr + (width / 2);
			break;
		case TCC_LCDC_IMG_FMT_YUV420ITL0:
		case TCC_LCDC_IMG_FMT_YUV420ITL1:
		case TCC_LCDC_IMG_FMT_YUV422ITL0:
		case TCC_LCDC_IMG_FMT_YUV422ITL1:
			base.p_Y  = h->cif_cfg.preview_buf[frm_num].p_Y + (width);
			base.p_Cb = h->cif_cfg.preview_buf[frm_num].p_Cb + (width);
			base.p_Cr = base.p_Cb;
			break;
		case TCC_LCDC_IMG_FMT_UYVY:
		case TCC_LCDC_IMG_FMT_VYUY:
		case TCC_LCDC_IMG_FMT_YUYV:
		case TCC_LCDC_IMG_FMT_YVYU:
			base.p_Y  = h->cif_cfg.preview_buf[frm_num].p_Y + (width * 2);
			base.p_Cb = h->cif_cfg.preview_buf[frm_num].p_Cb;	// not use
			base.p_Cr = h->cif_cfg.preview_buf[frm_num].p_Cr;	// not use
			break;
		default:
			printk("[vin_core]%s(%d) not support format\n", __func__, __LINE__);
		break;
		}		
	}

	VIOC_WDMA_SetImageBase(pWDMA, base.p_Y, base.p_Cb, base.p_Cr);
}

static void vioc_wdma_offs_inter2prog(VIOC_WDMA *pWDMA, unsigned int fmt, unsigned int width)
{
	unsigned int offset0 = 0;
	unsigned int offset1 = 0;

	switch (fmt) {
	case TCC_LCDC_IMG_FMT_YUV420SP:		// YCbCr 4:2:0 Separated format
	case TCC_LCDC_IMG_FMT_YUV422SP:		// YCbCr 4:2:2 Separated format
		offset0 = width;
		offset1 = width / 2;
		break;
	case TCC_LCDC_IMG_FMT_YUV420ITL0:	// YCbCr 4:2:0 interleved type 0 format
	case TCC_LCDC_IMG_FMT_YUV420ITL1:	// YCbCr 4:2:0 interleved type 1 format
	case TCC_LCDC_IMG_FMT_YUV422ITL0:	// YCbCr 4:2:2 interleved type 0 format
	case TCC_LCDC_IMG_FMT_YUV422ITL1:	// YCbCr 4:2:2 interleved type 1 format
		offset0 = width;
		offset1 = width;
		break;
	case TCC_LCDC_IMG_FMT_UYVY:			// YCbCr 4:2:2 Sequential format
	case TCC_LCDC_IMG_FMT_VYUY:			// YCbCr 4:2:2 Sequential format
	case TCC_LCDC_IMG_FMT_YUYV:			// YCbCr 4:2:2 Sequential format
	case TCC_LCDC_IMG_FMT_YVYU:			// YCbCr 4:2:2 Sequential format
		offset0 = width * 2;
		break;
	default:
		printk("[vin_core]%s(%d) not support format\n", __func__, __LINE__);
		break;
	}

	/* write odd field, skip even field.
	 * otherwise write even field, skip odd field.
	 */
	offset0 *= 2;
	offset1 *= 2;

	BITCSET(pWDMA->uOFFS.nREG, 0xFFFFFFFF, (offset1 << 16) | offset0);
}

static void vin_data_init(struct TCCxxxCIF *h, struct sensor_info *sinfo)
{
	/* default value */
	h->cif_cfg.oper_mode = OPER_PREVIEW;
	h->cif_cfg.order422 = CIF_YCBYCR;
	h->cif_cfg.zoom_step = 0;
	h->cif_cfg.pp_num = 0;
	h->cif_cfg.main_set.target_x = 0;
	h->cif_cfg.main_set.target_y = 0;
	h->cif_cfg.esd_restart = OFF;
	h->cif_cfg.cap_status = CAPTURE_NONE;
	h->cif_cfg.retry_cnt = 0;
	h->cif_cfg.base_buf = h->cif_buf.addr;
	/* get value from each sensor-module's info */
	h->cif_cfg.polarity_pclk = sinfo->polarity_pclk;
	h->cif_cfg.polarity_vsync = sinfo->polarity_vsync;
	h->cif_cfg.polarity_hsync = sinfo->polarity_hsync;
	// TODO: default set TCC_PFMT_YUV422 because vdev->pix.pixelformat = V4L2_OIX_FMT_YUYV in sensor_if_init()
	h->cif_cfg.pfmt = TCC_PFMT_YUV422;
}

static void vin_path_restart(struct tcc_video_device *vdev)
{
	struct TCCxxxCIF *h = vdev->h;
	struct tcc_vioc *vioc = &vdev->vioc;

	//printk("vsync ecc\n");

	/* disable interrupt */
	BITCLR(vioc->vin_addr->uVIN_INT.nREG, Hw31);
	VIOC_WDMA_SetIreqMask(vioc->wdma_addr, VIOC_WDMA_IREQ_ALL_MASK, 1);

	/* disable wdma & vin */
	VIOC_WDMA_SetImageDisable(vioc->wdma_addr);
	VIOC_VIN_SetEnable(vioc->vin_addr, OFF);

	/* reset vioc path */
	vioc_vin_path_reset(vdev);

	/* resetting vioc */
	h->stream_state = STREAM_ON;
	sensor_if_change_mode(vdev, OPER_PREVIEW);
	vioc_vin_setup(vdev, vioc->vin_addr, vioc->wmix_addr);
	vioc_scaler_setup(vdev, vioc->sc_addr, vioc->vin_addr, vioc->wmix_addr, vioc->sc_plugin);
	vioc_wdma_setup(vdev, vioc->wdma_addr, 1);

	mdelay(10);	// need delay!
}

static irqreturn_t vin_irq_handler(int irq, void *dev)
{
	struct tcc_video_device *vdev = (struct tcc_video_device *)dev;
	struct tcc_vioc *vioc = &vdev->vioc;

	if(vioc->vin_addr->uVIN_INT.nREG & vioc->vin.irq_mask) {
//		dprintk("vin vsync interrupt is set!! \n");
		atomic_inc(&vioc->vs_count);
		BITSET(vioc->vin_addr->uVIN_INT.nREG, vioc->vin.irq_mask);	// clear interrupt
		//printk("\e[32m%d\e[0m\n", atomic_read(&vioc->vs_count));
	}
	else {
//		dprintk("vin vsync interrupt is not set!! \n");
		return IRQ_NONE;
	}
	return IRQ_HANDLED;
}

static irqreturn_t wdma_irq_handler(int irq, void *dev)
{
	struct tcc_video_device *vdev = (struct tcc_video_device *)dev;
	struct TCCxxxCIF *h = vdev->h;
	struct tcc_vioc *vioc = &vdev->vioc;

//	dprintk("__ in \n");
//	dprintk("tcc_vioc addr : 0x%x \n", vioc);
	
	if (vioc->wdma_addr->uIRQSTS.nREG & vioc->wdma.irq_mask)
	{
		if (vioc->vsync_ecc) {
			/* Vsync Error Correcting Code 
			 */
			if (atomic_dec_return(&vioc->vs_count) != 0) {
				//printk("\e[31m%d\e[0m\n", atomic_read(&vioc->vs_count));
				vin_path_restart(vdev);
				atomic_set(&vioc->vs_count, 0);
				h->frame_skip = 1;
			} 
			//else {printk("\e[33m%d\e[0m\n", atomic_read(&vioc->vs_count));}
		}

		#if defined(CONFIG_TCC_DEINTL_WDMA_AUTO)
		/* If DEINTL_WDMA_AUTO is set, 
		 * wdma address should be changed when occur intertupt of bottom field.
		 * known-bug: normal interrupt of DEINTL_WDMA_AUTO should occur only on the bottom.
		 */
		if (vdev->sinfo->de_interlace == DEINTL_WDMA_AUTO 
			&& vioc->wdma_addr->uIRQSTS.bREG.STS_BFIELD == 0) {
			return IRQ_HANDLED;
		}
		#endif

		if (h->stream_state == STREAM_ON) 
		{
			if (vdev->sinfo->need_new_set)
				sensor_if_check_control(vdev);

			if (h->frame_skip == 0)
			{
				if (vdev->skip_buf)
					goto skip_buf;

				if (vdev->core.prev_buf != NULL) {
					list_move_tail(&vdev->core.prev_buf->buf_list, &h->done_list);

					#ifdef DEBUG_BUF
					printk("[irq] %d -> done_list\n", vdev->core.prev_buf->v4lbuf.index);
					#endif
				}
				//else {printk("\e[1;31;43m prev_buf is NULL \e[0m\n");}

				#ifdef DEBUG_BUF
				{
					struct list_head *p;
					struct tccxxx_cif_buffer *tbuf;
					printk("[irq] list: ");
					list_for_each(p, &h->list) {
						tbuf = list_entry(p, struct tccxxx_cif_buffer, buf_list);
						printk("(%d)", tbuf->v4lbuf.index);
					}
					printk("\n");
				}
				#endif

				vdev->core.prev_buf = h->buf + h->cif_cfg.now_frame_num;
				vdev->core.prev_num = h->cif_cfg.now_frame_num;

				#if 0
				/* for cicular buffer (not use) */
				if (h->list.next->next == &h->list) {
					if (!list_empty(&h->done_list)) {
						struct tccxxx_cif_buffer *old_buf;
						old_buf = list_entry(h->done_list.next, struct tccxxx_cif_buffer, buf_list);
						old_buf->v4lbuf.flags |= V4L2_BUF_FLAG_QUEUED;
						old_buf->v4lbuf.flags &= ~V4L2_BUF_FLAG_DONE;
						list_move_tail(h->done_list.next, &h->list);
						dprintk("empty qbuf list, so reuse old buf(%d)\n", old_buf->v4lbuf.index);
					} else {
						dprintk("empty qbuf list, need VIDIOC_DQBUF\n");
					}
				}
				#endif

				vdev->core.next_buf = list_entry(h->list.next->next, struct tccxxx_cif_buffer, buf_list);
				vdev->core.next_num = vdev->core.next_buf->v4lbuf.index;

skip_buf:
				if ((&vdev->core.next_buf->buf_list != &h->list) 
						&& (&vdev->core.prev_buf->buf_list != &vdev->core.next_buf->buf_list) 
						&& !(vdev->core.next_num < 0 || vdev->core.next_num > (h->n_sbufs - 1))) 
				{
					if (vdev->core.prev_num != vdev->core.prev_buf->v4lbuf.index) {
						printk("Frame num mismatch :: true num  :: %d \n", vdev->core.prev_num);
						printk("Frame num mismatch :: false num :: %d \n", vdev->core.prev_buf->v4lbuf.index);
						vdev->core.prev_buf->v4lbuf.index = vdev->core.prev_num;
					}

					if (h->cif_cfg.pfmt == TCC_PFMT_YUV420)
						vdev->core.prev_buf->v4lbuf.bytesused = (h->cif_cfg.main_set.target_x * h->cif_cfg.main_set.target_y * 3) / 2;
					else if (h->cif_cfg.pfmt == TCC_PFMT_YUV422)
						vdev->core.prev_buf->v4lbuf.bytesused = h->cif_cfg.main_set.target_x * h->cif_cfg.main_set.target_y * 2;
					else if (h->cif_cfg.pfmt == TCC_PFMT_RGB)
						vdev->core.prev_buf->v4lbuf.bytesused = h->cif_cfg.main_set.target_x * h->cif_cfg.bpp * h->cif_cfg.main_set.target_y;

					if (vdev->sinfo->de_interlace != DEINTL_WDMA_MANUAL) {
						vin_dma_hw_reg(h, vioc->wdma_addr, vdev->core.next_num);
					} else {
						vdev->frm_sq = vioc->vin_addr->uVIN_INT.bVIN_INT.frm_sq;
						//dprintk("%s\n", frm_sq ? "even" : "odd");
						vioc_wdma_base_inter2prog(h, vioc->wdma_addr, vdev->core.next_num, vdev->frm_sq, h->cif_cfg.main_set.target_x);
						if (vdev->frm_sq == FRM_SQ_EVEN) {
							vdev->skip_buf = 1;
							goto wdma_start;
						} else {
							vdev->skip_buf = 0;
						}
					}

					vdev->core.prev_buf->v4lbuf.flags &= ~V4L2_BUF_FLAG_QUEUED;
					vdev->core.prev_buf->v4lbuf.flags |= V4L2_BUF_FLAG_DONE;

					#if defined(CONFIG_TCC_DEINTL_VIQE_3D)
					if (vdev->sinfo->de_interlace == DEINTL_VIQE_3D) {
						if ((vioc->di_field_cnt == 1) && (vioc->di_field_bottom == 0)) {
							dprintk("change VIQE_DI_3D mode\n");
							VIOC_VIQE_SetDeintlMode(vioc->viqe_addr, VIOC_VIQE_DEINTL_MODE_3D);
						}
						if (vioc->di_field_bottom == 1) {
							vioc->di_field_bottom = 0;
							vioc->di_field_cnt++;
						} else {
							vioc->di_field_bottom = 1;
						}
					}
					#endif

					/* timestamp */
					do_gettimeofday(&vdev->core.prev_buf->v4lbuf.timestamp);

					/* s/w zoom - N/A */
					//if (unlikely(h->cif_cfg.zoom_step != vdev->core.old_zoom_step)) {
					//	unsigned int nCnt;
					//	VIOC_WDMA_SetIreqMask(vioc->wdma_addr, vdev->vioc.wdma.irq_mask, 0x1);	/* disable WDMA's irq */
					//	BITCSET(vioc->wdma_addr->uCTRL.nREG, 1<<28, 0<<28);					/* disable WDMA */
					//	BITCSET(vioc->wdma_addr->uCTRL.nREG, 1<<16, 1<<16);					/* update WDMA */
					//	/* Before camera quit, we have to wait WMDA's SEN signal to low.
					//	 * Note that do not decrease below Cnt.
					//	 */
					//	while (vioc->wdma_addr->uIRQSTS.nREG & VIOC_WDMA_IREQ_STSEN_MASK) {
					//		for (nCnt=0; nCnt<10000; nCnt++)
					//			;
					//	}
					//	VIOC_VIN_SetImageSize(vioc->vin_addr, h->cif_cfg.main_set.source_x, h->cif_cfg.main_set.source_y);
					//	VIOC_VIN_SetImageOffset(vioc->vin_addr,h->cif_cfg.main_set.win_hor_ofst, h->cif_cfg.main_set.win_ver_ofst, 0);
					//	VIOC_SC_SetUpdate(vioc->sc_addr);
					//	VIOC_WDMA_SetImageEnable(vioc->wdma_addr, ON);						/* enable WDMA */
					//	dprintk("VIN WxH[%dx%d] Crop Start XxY[%dx%d]\n",
					//			h->cif_cfg.main_set.source_x, h->cif_cfg.main_set.source_y,
					//			h->cif_cfg.main_set.win_hor_ofst, h->cif_cfg.main_set.win_ver_ofst);
					//}
					//vdev->core.old_zoom_step = h->cif_cfg.zoom_step;
				}
				else
				{
					dprintk("no buf\n");
					vdev->core.prev_buf = NULL;
					h->frame_skipped++;
				}

				if (vdev->sinfo->frame_skip_irq == FRAME_SKIP)
					h->frame_skip = 1;

				h->wakeup_int = 1;
				wake_up_interruptible(&h->frame_wait);
			}
			else
			{
				if (h->frame_skip > 0) {
					//dprintk("frm skip\n");
					h->frame_skip--;
				} else {
					h->frame_skip = 0;
				}
			}

wdma_start:
			BITCSET(vioc->wdma_addr->uCTRL.nREG, 1<<16, 1<<16);											/* update WDMA */
			BITCSET(vioc->wdma_addr->uIRQSTS.nREG, VIOC_WDMA_IREQ_ALL_MASK, VIOC_WDMA_IREQ_ALL_MASK);	/* clear irq status */
			VIOC_WDMA_SetIreqMask(vioc->wdma_addr, vioc->wdma.irq_mask, 0x0);							/* enable WDMA's irq */
			if (!vioc->wdma_continuous_mode)
				VIOC_WDMA_SetImageEnable(vioc->wdma_addr, vioc->wdma_continuous_mode);	/* if wdma isn't continuous mode */
		}
	}
	
//	dprintk("__ out \n");

	return IRQ_HANDLED;
}


/* Enables the camera. Takes camera out of reset. Enables the clocks. */ 
static int vin_enable(void)
{
	/* wait for camera to settle down */
	mdelay(5);
	return 0;
}
 
/* Disables all the camera clocks. Put the camera interface in reset. */
static int vin_disable(struct tcc_video_device *vdev)
{
	return 0;
}

static void vin_timer_register(struct TCCxxxCIF *h, unsigned long timeover);
static void vin_check_handler(unsigned long arg)
{
	struct TCCxxxCIF *h = (struct TCCxxxCIF *)arg;
	unsigned long timeover = 0;
		
	if(h->cif_cfg.oper_mode == OPER_PREVIEW) {
		///* Check Sensor-ESD register!! */
		//if(sensor_if_isESD()) {
		//	h->cif_cfg.esd_restart = ON;
		//	timeover = 0;
		//} else
			timeover = HZ;
		vin_timer_register(h, timeover);
	}
}

static void vin_timer_register(struct TCCxxxCIF *h, unsigned long timeover)
{
	init_timer(&h->cam_timer);
	h->cam_timer.function = vin_check_handler;
	h->cam_timer.data = (unsigned long)h;
	h->cam_timer.expires = get_jiffies_64()+ timeover;
	add_timer(&h->cam_timer);
	h->register_timer = 1;
}

static void vin_timer_deregister(struct TCCxxxCIF *h)
{
	del_timer(&h->cam_timer);
	h->cam_timer.expires = 0;
	h->register_timer = 0;
}

static void vin_scaler_calc(struct tcc_video_device *vdev)
{
	struct sensor_info *sinfo = vdev->sinfo;
	struct TCCxxxCIF *h = vdev->h;
	unsigned int off_x = 0, off_y = 0, width = 0, height = 0;

	off_x  = sinfo->prv_zoffx;
	off_y  = sinfo->prv_zoffy;
	width  = sinfo->prv_w;
	height = sinfo->prv_h;

	h->cif_cfg.main_set.win_hor_ofst = off_x * h->cif_cfg.zoom_step;	
	h->cif_cfg.main_set.win_ver_ofst = off_y * h->cif_cfg.zoom_step;	
	h->cif_cfg.main_set.source_x = width - (off_x * h->cif_cfg.zoom_step) * 2;
	h->cif_cfg.main_set.source_y = height - (off_y * h->cif_cfg.zoom_step) * 2;
}


void vioc_vin_path_mapping(struct tcc_video_device *vdev)
{
	struct device *dev = vdev->dev;
	struct tcc_cif_platform_data *pdata = dev->platform_data;

	/* get vioc compoent information */
	tccxxx_vioc_dt_parse_data(dev->of_node, vdev);

#if 0
	/* if you use vin_demux
	 * - setting the first device only (VIN_DEMUX and sensor module) 
	 */
	if (vdev->vioc.vin_demux.id != VIOC_NULL)
		vdev->vioc.vin_demux.idx = pdata->vioc->vin_demux_idx;
#endif 

#if 0
	/* mapping virt addr (default vioc component) */
	vdev->vioc.config_addr = (VIOC_IREQ_CONFIG *)tcc_p2v(pdata->vioc->config);
	vdev->vioc.vindemux_addr = (VIOC_VIN_DEMUX *)tcc_p2v(pdata->vioc->vindemux);
	vdev->vioc.ddiconfig_addr = (DDICONFIG *)tcc_p2v(pdata->vioc->ddi);
#endif

	dprintk("    vin: %d, 0x%08x, \n", vdev->vioc.vioc_num.vin.index, vdev->vioc.vin_addr);
	dprintk("   wmix: %d, 0x%08x, \n", vdev->vioc.vioc_num.wmixer.index, vdev->vioc.wmix_addr);
	dprintk("   wdma: %d, 0x%08x, \n", vdev->vioc.vioc_num.wdma.index, vdev->vioc.wdma_addr);
	dprintk("     sc: %d, 0x%08x, \n", vdev->vioc.vioc_num.scaler.index, vdev->vioc.sc_addr);
	dprintk("   viqe: %d, 0x%08x, \n", vdev->vioc.vioc_num.viqe.index, vdev->vioc.viqe_addr);
//	dprintk("deintls: %d, 0x%08x, 0x%x\n", vdev->vioc.deintls.id, vdev->vioc.deintls.phys_addr, vdev->vioc.deintls.swrst_bit);
	dprintk("wdma %s mode\n", vdev->vioc.wdma_continuous_mode ? "continuous" : "frame-by-frame");
	dprintk("vsync ecc: %s\n", vdev->vioc.vsync_ecc ? "use" : "not use");
}

void vioc_vin_path_reset(struct tcc_video_device *vdev)
{
	/* reset state */
	dprintk("in \n");

	VIOC_CONFIG_SWReset(vdev->vioc.config_addr, VIOC_CONFIG_WDMA, vdev->vioc.vioc_num.wdma.index, VIOC_CONFIG_RESET);
	VIOC_CONFIG_SWReset(vdev->vioc.config_addr, VIOC_CONFIG_WMIXER, vdev->vioc.vioc_num.wmixer.index, VIOC_CONFIG_RESET);
	VIOC_CONFIG_SWReset(vdev->vioc.config_addr, VIOC_CONFIG_SCALER, vdev->vioc.vioc_num.scaler.index, VIOC_CONFIG_RESET);
//	VIOC_CONFIG_SWReset(vdev->vioc.config_addr, VIOC_CONFIG_DEINTS, vdev->vioc.vioc_num.deintls.index, VIOC_CONFIG_RESET);
//	VIOC_CONFIG_SWReset(vdev->vioc.config_addr, VIOC_CONFIG_VIQE, vdev->vioc.vioc_num.viqe.index, VIOC_CONFIG_RESET);
	VIOC_CONFIG_SWReset(vdev->vioc.config_addr, VIOC_CONFIG_VIN, vdev->vioc.vioc_num.vin.index, VIOC_CONFIG_RESET);
	
	VIOC_CONFIG_SWReset(vdev->vioc.config_addr, VIOC_CONFIG_VIN, vdev->vioc.vioc_num.vin.index,VIOC_CONFIG_CLEAR);
//	VIOC_CONFIG_SWReset(vdev->vioc.config_addr, VIOC_CONFIG_VIQE, vdev->vioc.vioc_num.viqe.index,VIOC_CONFIG_CLEAR);
//	VIOC_CONFIG_SWReset(vdev->vioc.config_addr, VIOC_CONFIG_DEINTS, vdev->vioc.vioc_num.deintls.index,VIOC_CONFIG_CLEAR);
	VIOC_CONFIG_SWReset(vdev->vioc.config_addr, VIOC_CONFIG_SCALER, vdev->vioc.vioc_num.scaler.index,VIOC_CONFIG_CLEAR);
	VIOC_CONFIG_SWReset(vdev->vioc.config_addr, VIOC_CONFIG_WMIXER, vdev->vioc.vioc_num.wmixer.index,VIOC_CONFIG_CLEAR);
	VIOC_CONFIG_SWReset(vdev->vioc.config_addr, VIOC_CONFIG_WDMA, vdev->vioc.vioc_num.wdma.index,VIOC_CONFIG_CLEAR);

	dprintk("out \n");
}
EXPORT_SYMBOL(vioc_vin_path_reset);

static int vioc_viqe_setup(struct tcc_vioc *vioc, struct sensor_info *sinfo)
{
	pmap_t pmap_viqe;
	unsigned int viqe_deintl_base[4] = {0};
	int img_size, viqe_w, viqe_h;
	VIOC_VIQE_DEINTL_MODE mode;

	/* viqe config variable */
	int vmisc_tsdu = 0;			// 0: viqe size is get from vioc module
	int di_dec_misc_fmt = 1;	// 0: YUV420, 1: YUV422
	//int dn_misc1_byc = 1;		// de-noiser bypass coefficient

	img_size = (sinfo->prv_w * sinfo->prv_h * 2);

	if (sinfo->de_interlace == DEINTL_VIQE_3D) {
		mode = VIOC_VIQE_DEINTL_MODE_3D;

		pmap_get_info("viqe", &pmap_viqe);
		dprintk("[PMAP] %s: 0x%08x ~ 0x%08x (0x%08x)\n", 
			pmap_viqe.name, pmap_viqe.base, pmap_viqe.base + pmap_viqe.size, pmap_viqe.size);
	
		viqe_deintl_base[0] = pmap_viqe.base;
		viqe_deintl_base[1] = viqe_deintl_base[0] + img_size;
		viqe_deintl_base[2] = viqe_deintl_base[1] + img_size;
		viqe_deintl_base[3] = viqe_deintl_base[2] + img_size;

		if ((viqe_deintl_base[3] + img_size) > (pmap_viqe.base + pmap_viqe.size)) {
			printk("[%s] pamp_viqe no space\n", __func__);
			return -ENOBUFS;
		}
	} else {
		mode = VIOC_VIQE_DEINTL_MODE_2D;
	}

	if (vmisc_tsdu == 0) {
		viqe_w = 0;
		viqe_h = 0;
	} else {
		viqe_w = sinfo->prv_w;
		viqe_h = sinfo->prv_h;
	}

	VIOC_VIQE_SetControlRegister(vioc->viqe_addr, viqe_w, viqe_h, di_dec_misc_fmt);
	VIOC_VIQE_SetDeintlRegister(vioc->viqe_addr, di_dec_misc_fmt, vmisc_tsdu, viqe_w, viqe_h, 
		mode, viqe_deintl_base[0], viqe_deintl_base[1], viqe_deintl_base[2], viqe_deintl_base[3]);

	//TODO: test
	//VIOC_VIQE_SetDenoise(vioc->viqe_addr, di_dec_misc_fmt, viqe_w, viqe_h, 
	//	dn_misc1_byc, 0, viqe_deintl_base[0], viqe_deintl_base[1]);

	VIOC_VIQE_SetControlEnable(vioc->viqe_addr,
								OFF,	/* histogram CDF or LUT */
								OFF,	/* histogram */
								OFF,	/* gamut mapper */
								OFF,	/* de-noiser */
								ON		/* de-interlacer */
								);
//	VIOC_CONFIG_PlugIn(vioc->viqe.idx, vioc->viqe_plugin);
	VIOC_CONFIG_PlugIn(vioc->vioc_num.viqe.index + VIOC_VIQE, vioc->viqe_plugin);
	return 0;
}

static int vioc_vin_setup(struct tcc_video_device *vdev, VIOC_VIN *pVIN, VIOC_WMIX *pWMIX)
{
	struct TCCxxxCIF *h = vdev->h;
	struct sensor_info *sinfo = vdev->sinfo;
	unsigned int *vin_lut, *vioc_addr;
	unsigned int vin_w, vin_h, wmix_w, wmix_h;
	unsigned int offs_w, offs_h, offs_h_intl;
	unsigned int vin_y2r_mode;
	
	dprintk("__ in \n");
	
	/* vin size
	 * progressive/interlace: width x height
	 *      all de-interlace: width x (height/2)
	 */
	vin_w  = sinfo->prv_w;
	vin_h = sinfo->prv_h;
	if (sinfo->de_interlace)
//	if(sinfo->scan_type == SCAN_INTERLACE)
		vin_h /= 2;

	/* crop setting */
	offs_w		= 0;
	offs_h		= 0;
	offs_h_intl	= 0;

	/* VIN_LUT component
	 * it's address is vin_addr + 0x400
	 */
	//vin_lut = (unsigned int *)vioc_addr;


//	vin_lut = (unsigned int *)tcc_p2v(vdev->vioc.vioc_num.vin.phys_addr + 0x400);	/* VIN_LUT component */

	/* connet vin2/3 to wmix3,4. (1: vin path) */
	if (vdev->vioc.vioc_num.vin.index == VIOC_VIN_02)//vin.id == VIOC_VIN_02)
		VIOC_CONFIG_RDMA12PathCtrl(1);
	else if (vdev->vioc.vioc_num.vin.index == VIOC_VIN_03)
		VIOC_CONFIG_RDMA14PathCtrl(1);

	/* de-interlacer setting 
	 * DEINTL_S or VIQE
	 */
	if (sinfo->de_interlace == DEINTL_SIMPLE) {
		VIOC_CONFIG_SWReset(vdev->vioc.config_addr, VIOC_CONFIG_DEINTS, vdev->vioc.vioc_num.deintls.index, VIOC_CONFIG_RESET);
		VIOC_CONFIG_SWReset(vdev->vioc.config_addr, VIOC_CONFIG_DEINTS, vdev->vioc.vioc_num.deintls.index, VIOC_CONFIG_CLEAR);

		VIOC_CONFIG_PlugIn(VIOC_DEINTLS, vdev->vioc.deintls_plugin);
	} else if (sinfo->de_interlace == DEINTL_VIQE_2D 
			|| sinfo->de_interlace == DEINTL_VIQE_3D) {
		VIOC_CONFIG_SWReset(vdev->vioc.config_addr, VIOC_CONFIG_VIQE, vdev->vioc.vioc_num.viqe.index, VIOC_CONFIG_RESET);
		VIOC_CONFIG_SWReset(vdev->vioc.config_addr, VIOC_CONFIG_VIQE, vdev->vioc.vioc_num.viqe.index, VIOC_CONFIG_CLEAR);

		vioc_viqe_setup(&vdev->vioc, sinfo);
	}

	/* WMIX config
	 *   progressive/interlace: width x height
	 * only DEINTL_WDMA_MANUAL: width x (height/2)
	 */
	wmix_w = h->cif_cfg.main_set.target_x;
	wmix_h = h->cif_cfg.main_set.target_y;
	if (sinfo->de_interlace == DEINTL_WDMA_MANUAL)
		wmix_h /= 2;
	VIOC_CONFIG_WMIXPath(vdev->vioc.vioc_num.wmixer.index, ON);
	VIOC_WMIX_SetSize(pWMIX, wmix_w, wmix_h);
	VIOC_WMIX_SetUpdate(pWMIX);

	/* VIN config */
	VIOC_VIN_SetSyncPolarity(pVIN, 
					!(h->cif_cfg.polarity_hsync), 
					!(h->cif_cfg.polarity_vsync),
					OFF, OFF, OFF, 
					!(h->cif_cfg.polarity_pclk));
	dprintk("hsync : 0x%x vsync : 0x%x pclk ; 0x%x \n", h->cif_cfg.polarity_hsync, h->cif_cfg.polarity_vsync, h->cif_cfg.polarity_pclk);
	
	VIOC_VIN_SetCtrl(pVIN, 
					sinfo->interface_type, 
					sinfo->de2hsync, sinfo->de2hsync, 
					sinfo->input_data_fmt, sinfo->input_data_order);
	VIOC_VIN_SetInterlaceMode(pVIN, sinfo->scan_type, sinfo->intplen);

	if (sinfo->frame_skip_vin == FRAME_SKIP)
		pVIN->uVIN_CTRL.bVIN_CTRL.skip = 1;

	/* If wdma's output is RGB format, we have to set VIN's Y2R register
	 * because WDMA dosen't have Y2R register (It has only R2Y register).
	 */
	switch (h->cam_path_wdma_fmt) {
	case VIOC_IMG_FMT_RGB888:
	case VIOC_IMG_FMT_ARGB8888:
	case VIOC_IMG_FMT_RGB565:
	case VIOC_IMG_FMT_RGB332:
	case VIOC_IMG_FMT_ARGB4444:
	case VIOC_IMG_FMT_ARGB1555:
		vin_y2r_mode = VIN_Y2R_MODE2;
		break;
	default:
		vin_y2r_mode = vdev->sinfo->vin_y2r_mode;
		break;
	}

	VIOC_VIN_SetImageSize(pVIN, vin_w, vin_h);
	dprintk("pVIN->uVIN_SIZE.nREG : 0x%x \n", pVIN->uVIN_SIZE.nREG);
	
	VIOC_VIN_SetImageOffset(pVIN, offs_w, offs_h, offs_h_intl);
	VIOC_VIN_SetY2RMode(pVIN, vin_y2r_mode & 0x0f);
	VIOC_VIN_SetY2REnable(pVIN, (vin_y2r_mode & 0x10) >> 4);
	
	VIOC_VIN_SetLUT(pVIN, vdev->vioc.pLUTBase);//vin_lut);
	
	VIOC_VIN_SetLUTEnable(pVIN, OFF, OFF, OFF);

	VIOC_VIN_SetEnable(pVIN, ON);


	if (vdev->vioc.vsync_ecc) {
		//BITCSET(pVIN->uVIN_INT.nREG, 0xffffffff, Hw18|Hw3|Hw2|Hw1|Hw0);
		BITCSET(pVIN->uVIN_INT.nREG, 0xffffffff, (vdev->vioc.vin.irq_mask << 16) | vdev->vioc.vin.irq_mask);
		BITSET(pVIN->uVIN_INT.nREG, Hw31);
	}

	dprintk("VIOC VIN%d(%dx%d) - WMIX%d(%dx%d)\n", 
		vdev->vioc.vioc_num.vin.index, vin_w, vin_h, 
		vdev->vioc.vioc_num.wmixer.index, wmix_w, wmix_h);
	dprintk("__ out \n");

	dprintk("__________ pvin : 0x%x \n", pVIN);

	return 0;
}

static int vioc_vindemux_setup(struct tcc_video_device *vdev, VIOC_VIN_DEMUX *pVINDEMUX)
{
	unsigned int p0, p1, p2, p3;
	unsigned int chan_nr, chan_id;

	chan_nr = vdev->sinfo->private & VINDEMUX_CH_MASK;

	switch (chan_nr) {
	case VINDEMUX_CH_MUL2:
	case VINDEMUX_CH_MUL2_EXT1:
		p0 = MUL_PORT0;
		p1 = MUL_PORT1;
		p2 = EXT_PORT2;
		p3 = EXT_PORT3;
		break;
	case VINDEMUX_CH_MUL4:
		p0 = MUL_PORT0;
		p1 = MUL_PORT1;
		p2 = MUL_PORT2;
		p3 = MUL_PORT3;
		break;
	default:
		printk("%s: Not support mode of vin_demux channel\n", __func__);
		return -1;
		break;
	}

	VIOC_VIN_SetDemuxPort(pVINDEMUX, p0, p1, p2, p3);
	VIOC_VIN_SetDemuxClock(pVINDEMUX, (vdev->sinfo->private & VINDEMUX_CLK_MASK));
	VIOC_VIN_SetDemuxEnable(pVINDEMUX, ON);

	/* 4ch D1 108Mhz time-multiplex
	 * ----------------------------
	 * If you set 0x4444 in the VD_MISC register, noise is occurred.
	 * So, you set frame index in the VD_MISC using following rules.
	 * Note:
	 *   Should be enable "Channel ID inserted in SAV/EAV codes".
	 *   - TW2867 : CHID_MD(bit[5:4]) of NOVID(0x9E) register set 3.
	 *   - TVP5158: Chan_ID_SAVEAV_En(bit[2]) of AVD Output Control 2 register set 1.
	 */
	if (chan_nr == VINDEMUX_CH_MUL4) {
		msleep(50);
		chan_id = pVINDEMUX->uVIN_DEMUX_STS.nREG;
		dprintk("VIN_DEMUX.VD_STS   : 0x%08x\n", chan_id);
		switch (chan_id) {
		case 0x3210:
			chan_id = 0x3210;
			break;
		case 0x1032:
			chan_id = 0x1032;
			break;
		case 0x2103:
			chan_id = 0x0321;
			break;
		case 0x0321:
			chan_id = 0x2103;
			break;
		default:
			printk("\n\nWarning: detect invalid 'Channel ID'(0x%x)\n\n", chan_id);
			chan_id = 0x3210;
			break;
		}
		BITCSET(pVINDEMUX->uVIN_DEMUX_MISC.nREG, 0x00007777, chan_id);
	}

	dprintk("VIN_DEMUX.VD_CTRL  : 0x%08x\n", (int)pVINDEMUX->uVIN_DEMUX_CTRL.nREG);
	dprintk("         .VD_BLANK0: 0x%08x\n", (int)pVINDEMUX->uVIN_DEMUX_BLANK0.nREG);
	dprintk("         .VD_BLANK1: 0x%08x\n", (int)pVINDEMUX->uVIN_DEMUX_BLANK1.nREG);
	dprintk("         .VD_MISC  : 0x%08x\n", (int)pVINDEMUX->uVIN_DEMUX_MISC.nREG);
	dprintk("         - by %s\n", vdev->name);
	return 0;
}

static int vioc_wdma_setup(struct tcc_video_device *vdev, VIOC_WDMA *pDMA_CH, int restart)
{
	uint fmt;
	uint dw, dh;
	struct TCCxxxCIF *h = vdev->h;
	struct sensor_info *sinfo = vdev->sinfo;
	dprintk("__ in \n");

	/* wdma size
	 * progressive or interlace: width x height
	 * DEINTL_WDMA_MANUAL: width x (height/2)
	 * DEINTL_WDMA_AUTO - TCC892X (except TCC8925S): width x (height/2)
	 * DEINTL_WDMA_AUTO: width x height
	 */
	fmt = h->cam_path_wdma_fmt;			/* image format, set by VIDIOC_S_FMT */
//	fmt= 10;
	dw = h->cif_cfg.main_set.target_x;	/* destination image size */
	dh = h->cif_cfg.main_set.target_y;
	if (vdev->sinfo->de_interlace == DEINTL_WDMA_MANUAL)
		dh /= 2;
	#if defined(CONFIG_ARCH_TCC892X)
	else if (vdev->sinfo->de_interlace == DEINTL_WDMA_AUTO)
		dh /= 2;
	#endif

	if (!restart) {
		vin_dma_hw_reg(h, pDMA_CH, 0);
		dprintk("VIOC WDMA addr Y[0x%x] U[0x%x] V[0x%x], size[%dx%d]\n"
				,h->cif_cfg.preview_buf[0].p_Y
				,h->cif_cfg.preview_buf[0].p_Cb
				,h->cif_cfg.preview_buf[0].p_Cr
				,dw, dh);
	}

	/* DEINTL_WDMA_AUTO !!!
	 * If this reg bit set 1, wdma write progressive image from interlace input image.
	 * known-bug: Sometimes WDMA can not detect even/odd filed (TCC892X series except TCC8925S).
	 */
	if (vdev->sinfo->de_interlace == DEINTL_WDMA_AUTO)
		VIOC_WDMA_SetImageInterlaced(pDMA_CH, 1);

	if (vdev->sinfo->de_interlace != DEINTL_WDMA_MANUAL)
		VIOC_WDMA_SetImageOffset(pDMA_CH, fmt, dw);
	else
		vioc_wdma_offs_inter2prog(pDMA_CH, fmt, dw);

	VIOC_WDMA_SetImageFormat(pDMA_CH, fmt);

	dprintk("_______________ cam path wdma format : 0x%x\n", fmt);
	
	VIOC_WDMA_SetImageSize(pDMA_CH, dw, dh);
	VIOC_WDMA_SetImageR2YMode(pDMA_CH, sinfo->wdma_r2y_mode & 0x0f);
	VIOC_WDMA_SetImageR2YEnable(pDMA_CH, (sinfo->wdma_r2y_mode & 0x10) >> 4);

	VIOC_WDMA_SetImageEnable(pDMA_CH, vdev->vioc.wdma_continuous_mode);
	VIOC_WDMA_SetIreqMask(pDMA_CH, vdev->vioc.wdma.irq_mask, 0x0);
	dprintk("__ out \n");

	return 0;
}

static int vioc_scaler_setup(struct tcc_video_device *vdev, VIOC_SC *pSC, VIOC_VIN *pVIN, VIOC_WMIX *pWMIX, uint sc_plugin)
{
	unsigned int dw, dh;			// destination size in SCALER
	unsigned int width, height;
	unsigned int sc_channel;
	unsigned int mw = 0, mh = 0;	// image size in WMIX
	struct TCCxxxCIF *h = vdev->h;
	struct sensor_info *sinfo = vdev->sinfo;

	dprintk("__ in \n");

	if (vdev->vioc.vioc_num.scaler.index/*sc.id*/ == VIOC_NULL) {
		dprintk("not used scaler\n");
		return 0;
	}

	sc_channel = vdev->vioc.vioc_num.scaler.index;
	dw = h->cif_cfg.main_set.target_x;
	dh = h->cif_cfg.main_set.target_y;

	width  = sinfo->prv_w;
	height = sinfo->prv_h;
	if (sinfo->de_interlace == DEINTL_ONE_FIELD)
		height /= 2;

	if (dw == width && dh == height) {
		dprintk("not used scaler, because input size and output size is same\n");
		VIOC_CONFIG_PlugOut(sc_channel);
		return 0;
	}

	// TODO: scaler size
	switch (sc_plugin) {
	case VIOC_SC_VIN_00:	// vin0-sc-wmix5-wdma
	case VIOC_SC_VIN_01:	// vin1-sc-wmix6-wdma
	case VIOC_SC_RDMA_12:	// vin2-sc-wmix3-wdma
	case VIOC_SC_RDMA_14:	// vin3-sc-wmix4-wdma
		mw = h->cif_cfg.main_set.target_x;
		mh = h->cif_cfg.main_set.target_y;
		break;
	case VIOC_SC_WDMA_03:	// vin2-wmix3-sc-wdma3
	case VIOC_SC_WDMA_04:	// vin3-wmix4-sc-wdma4
	case VIOC_SC_WDMA_05:	// vin0-wmix5-sc-wdma5
	case VIOC_SC_WDMA_06:	// vin0-wmix5-sc-wdma6
	case VIOC_SC_WDMA_07:	// vin1-wmix6-sc-wdma7
	case VIOC_SC_WDMA_08:	// vin1-wmix6-sc-wdma8
		mw = sinfo->prv_w;
		mh = sinfo->prv_h;
		break;
	default:
		break;
	}

	VIOC_SC_SetBypass(pSC, OFF);

#if defined(CONFIG_TCC_REAR_CAMERA_DRV) // && defined(CONFIG_ARCH_TCC893X)
	VIOC_SC_SetDstSize(pSC, dw, (dh+6));
	VIOC_SC_SetOutPosition(pSC, 0, 6);
#else
	VIOC_SC_SetDstSize(pSC, dw, dh);		// set destination size in scaler
#endif

	VIOC_SC_SetOutSize(pSC, dw, dh);		// set output size in scaer
	VIOC_CONFIG_PlugIn(sc_channel, sc_plugin);	// PlugIn position in SCALER
	VIOC_SC_SetUpdate(pSC);					// Scaler update

	VIOC_WMIX_SetSize(pWMIX, mw, mh);		// set WMIX image size
	VIOC_WMIX_SetUpdate(pWMIX);				// WMIX update

	dprintk("VIOC SC%d(%dx%d) WMIX(%dx%d)\n", sc_channel, dw, dh, mw, mh);


	dprintk("__ out \n");

	
	return 0;
}

int vincore_set_bufs(struct tcc_video_device *vdev, struct v4l2_requestbuffers *req)
{
	struct TCCxxxCIF *h = vdev->h;
	dma_addr_t base_addr = h->cif_cfg.base_buf;
	unsigned int y_offset = 0;
	unsigned int uv_offset = 0;
	unsigned int total_off = 0;
	unsigned char i;
	unsigned long total_req_buf_size = 0;
	unsigned char req_buf = req->count;

	h->cif_cfg.now_frame_num = 0;

	if (req->count > TCC_V4L2_MAX_BUFNBRS) 
		req->count = TCC_V4L2_MAX_BUFNBRS;
	if (req->count < 0)
		req->count = TCC_V4L2_MIN_BUFNBRS;

	if (h->cif_cfg.pfmt == TCC_PFMT_RGB) {
		/* RGB total length(offset) = byteperline (width * bpp) * height */
		total_off = (h->cif_cfg.main_set.target_x * h->cif_cfg.bpp) * h->cif_cfg.main_set.target_y;
	} else {
		y_offset = h->cif_cfg.main_set.target_x * h->cif_cfg.main_set.target_y;

		if (h->cif_cfg.pfmt == TCC_PFMT_YUV420)
			uv_offset = y_offset / 4;
		else if (h->cif_cfg.pfmt == TCC_PFMT_YUV422)
			uv_offset = y_offset / 2;

		#if defined(CONFIG_ARCH_TCC892X)
		/* DEINTL_WDMA_AUTO !!!
		 * KNOWN-BUG: It needs large offset...
		 */
		if (vdev->sinfo->de_interlace == DEINTL_WDMA_AUTO) {
			y_offset += 0x200000;
			uv_offset += 0x200000;
		}
		#endif

		total_off = y_offset + uv_offset * 2;
	}

	total_off = PAGE_ALIGN(total_off);
	h->buf->v4lbuf.length = total_off;

retry:
	if (req_buf == 1) {
		#if 0  // ZzaU :: Don't check Buffer in Rolling-Capture.
		if (data->buf->v4lbuf.length > CAPTURE_MEM_SIZE) {
			printk("reqbufs: count invalid\n");
			return -1;
		}
		#endif
	} else {
		if (h->buf->v4lbuf.length * req->count > vdev->core.pmap_preview.size) {
			dprintk("REQUBUF error\n");
			req->count--;
			if (req->count <= 0) {
				printk("reqbufs: count invalid\n");
				return -1;
			}
			goto retry;
		}
	}

	memset(h->static_buf,0x00,req->count * sizeof(struct tccxxx_cif_buffer));

	h->done_list.prev = h->done_list.next = &h->done_list;
	h->list.prev = h->list.next = &h->list;

	for (h->n_sbufs = 0; h->n_sbufs < req->count; (h->n_sbufs++)) {
		struct tccxxx_cif_buffer *buf = &(h->static_buf[h->n_sbufs]);

		INIT_LIST_HEAD(&buf->buf_list);

		buf->v4lbuf.length = total_off;
		total_req_buf_size += buf->v4lbuf.length;
		buf->mapcount = 0;
		buf->cam = h;
		buf->v4lbuf.index = h->n_sbufs;
		buf->v4lbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf->v4lbuf.field = V4L2_FIELD_NONE;
		buf->v4lbuf.memory = V4L2_MEMORY_MMAP;

		/* Offset: must be 32-bit even on a 64-bit system. 
		 * videobuf-dma-sg just uses the length times the index,
		 * but the spec warns against doing just that - vma merging problems.
		 * So we leave a gap between each pair of buffers.
		 */
		//buf->v4lbuf.m.offset = 2 * index * buf->v4lbuf.length;	
	}
	dprintk("-----------------------------------------------------\n");
	dprintk("v4l2_buffer: 0x%x * %d = 0x%x\n", total_off, h->n_sbufs, (unsigned int)total_req_buf_size);

	h->cif_cfg.pp_num = h->n_sbufs;
	req->count = h->cif_cfg.pp_num;

	#if defined(CONFIG_ARCH_TCC892X)
	/* DEINTL_WDMA_AUTO !!!
	 * KNOWN-BUG: It needs large offset...
	 */
	if (vdev->sinfo->de_interlace == DEINTL_WDMA_AUTO) {
		/* If application want offset informations then
		 * application can use below values (optional).
		 * refer to tcc_videobuf_querybuf().
		 */
		vdev->core.offset_total = total_off;
		vdev->core.offset_y = y_offset;
		vdev->core.offset_uv = uv_offset;
	}
	#endif
	dprintk("y_off(0x%x) + uv_off(0x%x)*2 = ALIGN(0x%x)\n", y_offset, uv_offset, total_off);

	/* set base address each buffers
	 */
	dprintk("preview_buf    Y_base     Cb_base    Cr_base\n");
	for (i = 0; i < h->cif_cfg.pp_num; i++) {
		switch (h->cam_path_wdma_fmt) {
		case VIOC_IMG_FMT_YUV420SEP:	/* YCbCr 4:2:0 separated */
		case VIOC_IMG_FMT_YUV422SEP:	/* YCbCr 4:2:2 separated */
			h->cif_cfg.preview_buf[i].p_Y  = (unsigned int)PAGE_ALIGN(base_addr + total_off * i);
			h->cif_cfg.preview_buf[i].p_Cb = (unsigned int)h->cif_cfg.preview_buf[i].p_Y + y_offset;
			h->cif_cfg.preview_buf[i].p_Cr = (unsigned int)h->cif_cfg.preview_buf[i].p_Cb + uv_offset;
			break;
		case VIOC_IMG_FMT_YUV420IL0:	/* YCbCr 4:2:0 interleaved type0 */
		case VIOC_IMG_FMT_YUV420IL1:	/* YCbCr 4:2:0 interleaved type1 */
		case VIOC_IMG_FMT_YUV422IL0:	/* YCbCr 4:2:2 interleaved type0 */
		case VIOC_IMG_FMT_YUV422IL1:	/* YCbCr 4:2:2 interleaved type1 */
			h->cif_cfg.preview_buf[i].p_Y  = (unsigned int)PAGE_ALIGN(base_addr + total_off * i);
			h->cif_cfg.preview_buf[i].p_Cb = (unsigned int)h->cif_cfg.preview_buf[i].p_Y + y_offset;
			h->cif_cfg.preview_buf[i].p_Cr = h->cif_cfg.preview_buf[i].p_Cb;
			break;
		case VIOC_IMG_FMT_UYVY:			/* YCbCr 4:2:2 UYVY Sequential */
		case VIOC_IMG_FMT_VYUY:			/* YCbCr 4:2:2 VYUY Sequential */
		case VIOC_IMG_FMT_YUYV:			/* YCbCr 4:2:2 YUYV Sequential */
		case VIOC_IMG_FMT_YVYU:			/* YCbCr 4:2:2 YVYU Sequential */
			h->cif_cfg.preview_buf[i].p_Y  = (unsigned int)PAGE_ALIGN(base_addr + total_off * i);
			h->cif_cfg.preview_buf[i].p_Cb = (unsigned int)h->cif_cfg.preview_buf[i].p_Cb;	// not use
			h->cif_cfg.preview_buf[i].p_Cr = (unsigned int)h->cif_cfg.preview_buf[i].p_Cr;	// not use
			break;

		/* RGB format */
		case VIOC_IMG_FMT_RGB332:
		case VIOC_IMG_FMT_ARGB4444:
		case VIOC_IMG_FMT_ARGB1555:
		case VIOC_IMG_FMT_RGB565:
		case VIOC_IMG_FMT_RGB888:
		case VIOC_IMG_FMT_ARGB8888:
			h->cif_cfg.preview_buf[i].p_Y = (unsigned int)PAGE_ALIGN(base_addr + total_off * i);
			h->cif_cfg.preview_buf[i].p_Cb = 0;
			h->cif_cfg.preview_buf[i].p_Cr = 0;
			break;

		default:
			printk(KERN_ERR "%s: Not supported VIOC format(%d)\n", __func__, h->cam_path_wdma_fmt);
			return -EINVAL;
			break;
		}
		dprintk("     [%d]      0x%08x 0x%08x 0x%08x\n", i,
			h->cif_cfg.preview_buf[i].p_Y, h->cif_cfg.preview_buf[i].p_Cb, h->cif_cfg.preview_buf[i].p_Cr);
	}
	dprintk("-----------------------------------------------------\n");

	return 0;	
}

int vincore_restart(struct tcc_video_device *vdev)
{
	struct tcc_vioc *vioc = &vdev->vioc;
	struct TCCxxxCIF *h = vdev->h;
	struct tccxxx_cif_buffer *cif_buf;

	dprintk("%s+++\n", __func__);
	vincore_stop_stream(vdev);

	sensor_if_restart(vdev);
	sensor_if_change_mode(vdev, OPER_PREVIEW);

	while (!list_empty(&h->done_list)) {
		cif_buf = list_entry(h->done_list.next, struct tccxxx_cif_buffer, buf_list);
		list_del(&cif_buf->buf_list);
		cif_buf->v4lbuf.flags &= ~V4L2_BUF_FLAG_DONE;	
		cif_buf->v4lbuf.flags |= V4L2_BUF_FLAG_QUEUED;		
		list_add_tail(&cif_buf->buf_list, &h->list);	
	}

	// Sensor setting Again!! 
	sensor_if_set_current_control(vdev);

	vin_dma_hw_reg(h, vioc->wdma_addr, 0);
	h->stream_state = STREAM_ON;
	h->cif_cfg.esd_restart = OFF;
	vin_timer_register(h, HZ);

	dprintk("%s---\n", __func__);
	return 0;
}

int vincore_start_stream(struct tcc_video_device *vdev)
{
	struct TCCxxxCIF *h = vdev->h;
	struct tcc_vioc *vioc = &vdev->vioc;
	struct device *dev = vdev->dev;
	struct tcc_cif_platform_data *pdata = dev->platform_data;

	dprintk("%s+++\n", __func__);
	h->cif_cfg.oper_mode = OPER_PREVIEW;
	h->cif_cfg.cap_status = CAPTURE_NONE;
	h->stream_state = STREAM_ON;
	vdev->core.old_zoom_step = 0;
	vdev->core.prev_buf = NULL;
	vdev->skip_buf = 0;
	vioc->di_field_cnt = 0;
	vioc->di_field_bottom = 0;

	/* reset vioc component
	 */
	vioc_vin_path_reset(vdev);

	/* vin_demux setting
	 */
	dprintk("vdev->vioc.vin_demux.id : 0x%x \n", vdev->vioc.vioc_num.vin_demux.index);

	if (vdev->vioc.vioc_num.vin_demux.index != VIOC_NULL) {
		/* for the management of devices that share a one sensor */
		if (!(pdata->cif_shared_sensor_ctrl != NULL 
				&& pdata->cif_shared_sensor_ctrl(vdev->video_nr, SHARED_SENSOR_INFO_VINDEMUX, 1, NULL))) {
			vioc_vindemux_setup(vdev, vioc->vindemux_addr);
		}
	}

	
	/* vioc path setting
	 */
	dprintk("vioc->vsync_ecc \n");
	
	if (vioc->vsync_ecc)
		atomic_set(&vioc->vs_count, 1);

	sensor_if_change_mode(vdev, OPER_PREVIEW);
	dprintk("_____________ vioc->vin_addr : 0x%8x \n", vioc->vin_addr);
	vioc_vin_setup(vdev, vioc->vin_addr, vioc->wmix_addr);
	vioc_scaler_setup(vdev, vioc->sc_addr, vioc->vin_addr, vioc->wmix_addr, vioc->sc_plugin);
	vioc_wdma_setup(vdev, vioc->wdma_addr, 0);

	dprintk("%s---\n", __func__);
	return 0;
}

int vincore_stop_stream(struct tcc_video_device *vdev)
{
	struct TCCxxxCIF *h = vdev->h;
	struct tcc_vioc *vioc = &vdev->vioc;
	struct device *dev = vdev->dev;
	struct tcc_cif_platform_data *pdata = dev->platform_data;

	dprintk("%s+++\n", __func__);
	VIOC_WDMA_SetIreqMask(vioc->wdma_addr, VIOC_WDMA_IREQ_ALL_MASK, 0x1);
	BITCSET(vioc->wdma_addr->uCTRL.nREG, 1<<28, 0<<28);		// Disable WDMA
	BITCSET(vioc->wdma_addr->uCTRL.nREG, 1<<16, 1<<16);

	msleep(50);

	VIOC_VIN_SetEnable(vioc->vin_addr, OFF);	// Disable VIN

	msleep(50);

	h->stream_state = STREAM_OFF;
	dprintk("skipped frame = %d\n", h->frame_skipped);

	/* disable vin_demux
	 */
	if (vdev->vioc.vioc_num.vin_demux.index != VIOC_NULL) {
		/* for the management of devices that share a one sensor */
		if (!(pdata->cif_shared_sensor_ctrl != NULL 
				&& pdata->cif_shared_sensor_ctrl(vdev->video_nr, SHARED_SENSOR_INFO_VINDEMUX, 0, NULL))) {
			VIOC_VIN_SetDemuxEnable(vioc->vindemux_addr, OFF);
		}
	}

	/* plug-out de-interlacer 
	 * DEINTL_S or VIQE
	 */
	if (vdev->sinfo->de_interlace == DEINTL_SIMPLE) {
		VIOC_CONFIG_PlugOut(vdev->vioc.deintls.idx);
		BITSET(vdev->vioc.config_addr->uSOFTRESET.nREG[1], vdev->vioc.deintls.swrst_bit);
		BITCLR(vdev->vioc.config_addr->uSOFTRESET.nREG[1], vdev->vioc.deintls.swrst_bit);
	} else if (vdev->sinfo->de_interlace == DEINTL_VIQE_2D 
			|| vdev->sinfo->de_interlace == DEINTL_VIQE_3D) {
		VIOC_CONFIG_PlugOut(vdev->vioc.viqe.idx);
		BITSET(vdev->vioc.config_addr->uSOFTRESET.nREG[1], vdev->vioc.viqe.swrst_bit);
		BITCLR(vdev->vioc.config_addr->uSOFTRESET.nREG[1], vdev->vioc.viqe.swrst_bit);
	}

	/* reset vioc component
	 */
	vioc_vin_path_reset(vdev);

	dprintk("%s---\n", __func__);
	return 0;
}

int vincore_set_zoom(struct tcc_video_device *vdev, int arg)
{
	struct TCCxxxCIF *h = vdev->h;
	h->cif_cfg.zoom_step = arg;
	if (h->stream_state != STREAM_OFF) {
		dprintk("zoom level = %d. \n", h->cif_cfg.zoom_step);
		vin_scaler_calc(vdev);
	}
	return 0;
}

int vincore_set_resolution(struct tcc_video_device *vdev, unsigned int pixel_fmt, unsigned short width, unsigned short height)
{
	struct TCCxxxCIF *h = vdev->h;

	h->cam_path_wdma_fmt = fourcc2vioc(pixel_fmt);
	h->cif_cfg.pfmt = vin_try_pfmt(pixel_fmt);
	h->cif_cfg.bpp = vin_try_bpp(pixel_fmt);
	if (h->cif_cfg.pfmt < 0 || h->cif_cfg.bpp < 0)
		return -EINVAL;

	h->cif_cfg.main_set.target_x = width;
	h->cif_cfg.main_set.target_y = height;

	if (h->stream_state != STREAM_OFF) {
		vincore_stop_stream(vdev);
		vincore_start_stream(vdev);
	}

	return 0;
}

int vincore_set_frameskip(struct tcc_video_device *vdev, int arg)
{
	struct tcc_vioc *vioc = &vdev->vioc;
	vioc->vin_addr->uVIN_CTRL.bVIN_CTRL.skip = arg;
	dprintk("%s: vin.ctrl.frameskip=%d\n", vdev->name, vioc->vin_addr->uVIN_CTRL.bVIN_CTRL.skip);
	return 0;
}

int vincore_open(struct tcc_video_device *vdev)
{
	int ret, irq_num;
	struct TCCxxxCIF *h = vdev->h;
	struct device_node *cam_np, *irq_np;
	
	dprintk("__ in \n");
	
	h->done_list.next	= &h->done_list;
	h->list.next		= &h->list;

	/* use wdma's EOFF(EOF Falling) interrupt.
	 * note: SoC recommanded using EOFR(EOF Rising) interrupt.
	 */
	vdev->vioc.wdma.irq_mask = VIOC_WDMA_IREQ_EOFF_MASK;
	vdev->vioc.wdma_addr->uIRQSTS.nREG |= VIOC_WDMA_IREQ_ALL_MASK;  
	
	cam_np = of_parse_phandle(vdev->dev->of_node, "telechips,cif_camera", 0);
	if(cam_np < 1) {
		pr_err("can't find telechips,cif_camera\r\n");
		return ret;
	}
	
	irq_np = of_parse_phandle(cam_np, "camera_wdma", 0);
	
	if(irq_np) 
	{
		irq_num = irq_of_parse_and_map(irq_np, vdev->vioc.vioc_num.wdma.index);

		dprintk("irq_num : %d \n", irq_num);

		ret = request_irq(irq_num, wdma_irq_handler, IRQF_SHARED, vdev->name, vdev);
		if (ret < 0) {
			printk("vdev->vioc.wdma.irq : %d \n", irq_num);
			printk("%s: wdma request_irq failed\n", vdev->name);
			return ret;
		}
	}
	
	if (vdev->vioc.vsync_ecc) {
		/* use vin's VS(VSyng) interrupt for VSync error correcting.
		 *  VIN_INT[INVS, VS, EOF, UPD]
		 *  VIN_INT[  19, 18,  17,  16] is interrupt mask bits.
		 *  VIN_INT[   3,  2,   1,   0] is interrupt status & clear bits.
		 */
		irq_np = of_parse_phandle(cam_np, "camera_videoin", 0);
		if(irq_np) 
		{
			irq_num = irq_of_parse_and_map(irq_np, vdev->vioc.vioc_num.vin.index);

			dprintk("irq_num : %d \n", irq_num);
			
			vdev->vioc.vin_addr->uVIN_INT.nREG |= 4; 
			vdev->vioc.vin.irq_mask = Hw2;	// using clear bit instead mask bit.
			
			ret = request_irq(irq_num, vin_irq_handler, IRQF_SHARED, vdev->name, vdev);
			if (ret < 0) {
				printk("%s: vin request_irq failed\n", vdev->name);
				return ret;
			}		
		}
#if 0
		vdev->vioc.vin.irq_mask = Hw2;	// using clear bit instead mask bit.
		ret = request_irq(vdev->vioc.vin.irq, vin_irq_handler, IRQF_SHARED, vdev->name, vdev);
		if (ret < 0) {
			printk("%s: vin request_irq failed\n", vdev->name);
			return ret;
		}
#endif 
	}

	dprintk("__ out \n");	
	return 0;
}

int vincore_close(struct tcc_video_device *vdev)
{
	struct TCCxxxCIF *h = vdev->h;	
	struct device_node *cam_np, *irq_np;
	unsigned int irq_num;
	
	vin_disable(vdev);

	if (h->cif_buf.area != NULL)
		iounmap(h->cif_buf.area);

	cam_np = of_parse_phandle(vdev->dev->of_node, "telechips,cif_camera", 0);
	if(cam_np < 1) {
		pr_err("can't find telechips,cif_camera\r\n");
		return -1;
	}
	
	irq_np = of_parse_phandle(cam_np, "camera_wdma", 0);
	
	if(irq_np) 
	{
		irq_num = irq_of_parse_and_map(irq_np, vdev->vioc.vioc_num.wdma.index);
	
		dprintk("irq_num : %d \n", irq_num);
		free_irq(irq_num, vdev);
	}

	if (vdev->vioc.vsync_ecc)
	{
		irq_np = of_parse_phandle(cam_np, "camera_videoin", 0);
		if(irq_np) 
		{
			irq_num = irq_of_parse_and_map(irq_np, vdev->vioc.vioc_num.vin.index);
		
			dprintk("irq_num : %d \n", irq_num);
			free_irq(irq_num, vdev);		
		}
	}
//		free_irq(vdev->vioc.vin.irq, vdev);

	if (h->register_timer)
		vin_timer_deregister(h);

	kfree(h);
	return 0;
}

int vincore_init(struct tcc_video_device *vdev)
{
	int ret = 0;
	struct TCCxxxCIF *h;
	struct device *dev = vdev->dev;
	struct tcc_cif_platform_data *pdata = dev->platform_data;
	struct device_node * cam_np, * port_np; 
	unsigned int * cifport_addr;
	unsigned int usingPort, cifport_val;
	
	/* get v4l2 pmap buffer */
	ret = tcc_cif_pmap_ctrl/*pdata->cif_pmap_ctrl*/(vdev->video_nr, &vdev->core.pmap_preview, 
					vdev->sinfo->prv_w, vdev->sinfo->prv_h, TCC_V4L2_MIN_BUFNBRS);
	if (ret)
		return -ENOMEM;

	h = kzalloc(sizeof(struct TCCxxxCIF), GFP_KERNEL);
	if (h == NULL)
		return -ENOMEM;

	h->buf = h->static_buf;
	h->cif_buf.bytes = PAGE_ALIGN(vdev->core.pmap_preview.size);
	h->cif_buf.addr = vdev->core.pmap_preview.base;
	h->cif_buf.area = ioremap_nocache(h->cif_buf.addr, h->cif_buf.bytes);
	dprintk("reamp : [0x%x - 0x%x] -> [0x%x] \n", h->cif_buf.addr, h->cif_buf.bytes, (unsigned int)h->cif_buf.area);		
	if (h->cif_buf.area == NULL) {
		printk(KERN_ERR "%s: cannot remap buffer\n", vdev->name);
		return -ENODEV;
	}

	cif_set_port(vdev);

	vin_data_init(h, vdev->sinfo);	/* Init the camera IF */
	vin_enable();					/* enable it. This is needed for sensor detection */

	init_waitqueue_head(&h->frame_wait);
	spin_lock_init(&h->dev_lock);
	INIT_LIST_HEAD(&h->list);
	INIT_LIST_HEAD(&h->done_list);

	vdev->h = h;

	return ret;
}

static int tcc_cif_pmap_ctrl(int video_nr, pmap_t *pmap, unsigned int width, unsigned int height, int total_buf)
{
	int ret = 0;

#ifdef PMAP_UMP_RESERVED
	if (pmap_info.pmap_list[video_nr].used) {
		pmap->base = pmap_info.pmap_list[video_nr].base;
		pmap->size = pmap_info.pmap_list[video_nr].size;
		goto exit;
	}

	pmap->base = pmap_info.cur_addr;
	pmap->size = PAGE_ALIGN(width * height * total_buf * 2);	// *2 (assuming 4:2:2)

	if ((pmap->base + pmap->size) > pmap_info.end_addr) {
		printk("error: tcc_video_device%d pmap buf allocation failed\n", video_nr);
		ret = -ENOMEM;
		goto exit;
	}

	pmap_info.cur_addr += pmap->size;
	pmap_info.pmap_list[video_nr].used = 1;
	pmap_info.pmap_list[video_nr].base = pmap->base;
	pmap_info.pmap_list[video_nr].size = pmap->size;

exit:
	dprintk("  0x%08x - %s.end\n", pmap_info.end_addr, pmap_info.pmap_ump_reserved.name);
	dprintk("  0x%08x - preview.end\n", pmap->base + pmap->size);
	dprintk("	 tcc_video_device%d buf area\n", video_nr);
	dprintk("  0x%08x - preview.base\n", pmap->base);	
	dprintk("  0x%08x - %s.base\n", pmap_info.pmap_ump_reserved.base, pmap_info.pmap_ump_reserved.name);
#else
	/* get pmap v4l2_videoX */
	char name[36];
	sprintf(name, "v4l2_video%d", video_nr);
//	strcpy(name, "video");
	pmap_get_info(name, pmap);
	dprintk("[PMAP] %s: 0x%08x ~ 0x%08x (0x%08x)\n", 
		name, pmap->base, pmap->base + pmap->size, pmap->size);
#endif
	return ret;
}

