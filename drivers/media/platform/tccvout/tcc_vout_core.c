/*
 * tcc_vout_core.c
 *
 * Copyright (C) 2013 Telechips, Inc. 
 *
 * Video-for-Linux (Version 2) video output driver for Telechips SoC.
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
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <soc/tcc/timer.h>

#include "tcc_vout.h"
#include "tcc_vout_core.h"
#include "tcc_vout_dbg.h"

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_address.h>
#endif

#ifndef CONFIG_ARCH_TCC897X
#include <video/tcc/tcc_types.h>
#endif

#if defined(CONFIG_TCC_VIOCMG)
#include <video/tcc/viocmg.h>
#endif

#ifdef CONFIG_VOUT_USE_VSYNC_INT
static struct tcc_timer *vout_timer = NULL;
static struct timer_list ktimer;
static int ktimer_enable = 0;
#endif

static struct v4l2_buffer *last_cleared_buffer = NULL;

extern int popIdx;
extern int pushIdx;
extern int clearIdx;
extern int displayed_bufs;
extern int readable_bufs;
extern struct v4l2_buffer displaybuffer[MAX_DISPBUF_NUM];

#ifdef CONFIG_ARCH_TCC_MAP_CONVERTER
static unsigned int mc_plugged = 0;

extern void tca_map_convter_driver_set(unsigned int mc_num, unsigned int Fwidth, unsigned int Fheight, unsigned int pos_x, unsigned int pos_y, unsigned int y2r, hevc_dec_MapConv_info_t *mapConv_info);
extern void tca_map_convter_onoff(unsigned int mc_num, unsigned int onoff);
extern void tca_map_convter_wait_done(unsigned int mc_num);
#endif

extern unsigned int hdmi_get_refreshrate(void);

extern OUTPUT_SELECT_MODE Output_SelectMode;

static int tcc_get_base_address(
		unsigned int viocfmt, unsigned int base0_addr,
		unsigned int width, unsigned int height,
		unsigned int pos_x, unsigned int pos_y,
		unsigned int *base0, unsigned int *base1, unsigned int *base2)
{
	unsigned int u_addr, v_addr, y_offset, uv_offset;

	//dprintk("src: 0x%08x, 0x%08x, 0x%08x\n", *base0, *base1, *base2);
	//dprintk("fmt(%d) %d x %d ~ %d x %d\n", viocfmt, pos_x, pos_y, width, height);

	/*
	 * Calculation of base0 address
	 */
	y_offset = (width * pos_y) + pos_x;

	if ((viocfmt >= VIOC_IMG_FMT_RGB332) && (viocfmt <= VIOC_IMG_FMT_ARGB6666_3)) {
		int bpp;

		if(viocfmt == VIOC_IMG_FMT_RGB332)
			bpp = 1;
		else if((viocfmt >= VIOC_IMG_FMT_ARGB4444) && (viocfmt <= VIOC_IMG_FMT_ARGB1555))
			bpp = 2;
		else if((viocfmt >= VIOC_IMG_FMT_ARGB8888) && (viocfmt <= VIOC_IMG_FMT_ARGB6666_3))
			bpp = 4;
		else
			bpp = 2;

		*base0 = base0_addr + y_offset * bpp;
	}

	if ((viocfmt == VIOC_IMG_FMT_UYVY)
		|| (viocfmt == VIOC_IMG_FMT_VYUY)
		|| (viocfmt == VIOC_IMG_FMT_YUYV)
		|| (viocfmt == VIOC_IMG_FMT_YVYU))
	{
		y_offset = y_offset * 2;
	}

	*base0 = base0_addr + y_offset; 	// Set base0 address for image

	/*
	 * Calculation of base1 and base2 address
	 */

	if (*base1 == 0 && *base2 == 0) {
		u_addr = GET_ADDR_YUV42X_spU(base0_addr, width, height);
		if (viocfmt == VIOC_IMG_FMT_YUV420SEP)
			v_addr = GET_ADDR_YUV420_spV(u_addr, width, height);
		else
			v_addr = GET_ADDR_YUV422_spV(u_addr, width, height);
	} else {
		u_addr = *base1;
		v_addr = *base2;
	}

	if ((viocfmt == VIOC_IMG_FMT_YUV420SEP)
		|| (viocfmt == VIOC_IMG_FMT_YUV420IL0)
		|| (viocfmt == VIOC_IMG_FMT_YUV420IL1))
	{
		if (viocfmt == VIOC_IMG_FMT_YUV420SEP)
			uv_offset = ((width * pos_y) / 4) + (pos_x / 2);
		else
			uv_offset = ((width * pos_y) / 2) + (pos_x);
	}
	else
	{
		if (viocfmt == VIOC_IMG_FMT_YUV422IL1)
			uv_offset = (width * pos_y) + (pos_x);
		else
			uv_offset = ((width * pos_y) / 2) + (pos_x / 2);
	}

	*base1 = u_addr + uv_offset;		// Set base1 address for image
	*base2 = v_addr + uv_offset;		// Set base2 address for image

	return 0;
}

static int tcc_get_base_address_of_image(
		unsigned int pixelformat, unsigned int base0_addr,
		unsigned int width, unsigned int height,
		unsigned int *base0, unsigned int *base1, unsigned int *base2)
{
	unsigned int y_offset = 0, uv_offset = 0;

	/* gstv4l2object.c - gst_v4l2_object_get_caps_info()
	 */
	switch (pixelformat) {
	case V4L2_PIX_FMT_YUV420:
	case V4L2_PIX_FMT_YVU420:
		y_offset = ROUND_UP_4(width) * ROUND_UP_2(height);
		uv_offset = (ROUND_UP_4(width) / 2) * (ROUND_UP_2(height) / 2);
		break;
	case V4L2_PIX_FMT_Y41P:
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_UYVY:
	case V4L2_PIX_FMT_YVYU:
		y_offset = (ROUND_UP_2(width) * 2) * height;
		break;
	case V4L2_PIX_FMT_YUV411P:
		y_offset = ROUND_UP_4(width) * height;
		uv_offset = (ROUND_UP_4(width) / 4) * height;
		break;
	case V4L2_PIX_FMT_YUV422P:
		y_offset = ROUND_UP_4(width) * height;
		uv_offset = (ROUND_UP_4(width) / 2) * height;
		break;
	case V4L2_PIX_FMT_NV12:
	case V4L2_PIX_FMT_NV21:
		y_offset = ROUND_UP_4(width) * ROUND_UP_2(height);
		uv_offset = (ROUND_UP_4(width) * height) / 2;
		break;
	}

	*base0 = base0_addr;
	*base1 = *base0 + y_offset;
	*base2 = *base1 + uv_offset;

	return 0;
}

static void vout_update_displayed_info(struct tcc_vout_device *vout, struct v4l2_buffer *buf)
{
	/* update last displayed buffer index */
	vout->last_displayed_buf_idx = buf->index;
	displayed_bufs++;

	return;
}

int vout_get_pmap(pmap_t *pmap)
{
    if (pmap_get_info(pmap->name, pmap) == 1) {
	    dprintk("[PMAP] %s: 0x%08x ~ 0x%08x (0x%08x)\n",
	        pmap->name, pmap->base, pmap->base + pmap->size, pmap->size);
		return 0;
	}
	return -1;
}

int vout_set_deintl_path(int deintl_default, struct tcc_vout_device *vout)
{
	struct device_node *dev_np;
	struct tcc_vout_vioc *vioc = vout->vioc;

	/* get pmap for deintl_bufs */
	memcpy(&vout->deintl_pmap.name, VOUT_DEINTL_PATH_PMAP_NAME, sizeof(VOUT_DEINTL_PATH_PMAP_NAME));
	if (vout_get_pmap(&vout->deintl_pmap)) {
		printk(KERN_ERR VOUT_NAME ": [error] vout_get_pmap(%s)\n", vout->deintl_pmap.name);
		return -1;
	}

	if (deintl_default) {
		/* default 4 buffers */
		vout->deintl_nr_bufs = 4;

		vout->deinterlace = VOUT_DEINTL_VIQE_3D;

		/* bottom-field first */
		vioc->deintl_rdma.bf = 1;

		// set deinterlace rdma
		dev_np = of_parse_phandle(vout->v4l2_np, "deintl_rdma", 0);
		if(dev_np) {
			/* swreser bit */
			of_property_read_u32_index(vout->v4l2_np, "deintl_rdma", 2, &vioc->deintl_rdma.swrst_bit); 
			of_property_read_u32_index(vout->v4l2_np, "deintl_rdma", 1, &vioc->deintl_rdma.id); 
			vioc->deintl_rdma.addr = (PVIOC_RDMA)of_iomap(dev_np, vioc->deintl_rdma.id);
			dprintk("[DEINTL] RDMA < vir_addr = 0x%x , id = %d , reset = %d \n", vioc->deintl_rdma.addr, vioc->deintl_rdma.id, vioc->deintl_rdma.swrst_bit);
		} else {
			printk("[DEINTL] could not find rdma node of vout driver. \n");
		}

		// set deinterlace scaler
		dev_np = of_parse_phandle(vout->v4l2_np, "deintl_scaler", 0);
		if(dev_np) {
			/* path plugin */
			of_property_read_u32(vout->v4l2_np, "sc_path", &vioc->deintl_sc.ch);
			of_property_read_u32(vout->v4l2_np, "sc_plugin", &vioc->deintl_sc.plugin);

			/* swreser bit */
			of_property_read_u32_index(vout->v4l2_np, "deintl_scaler", 2, &vioc->deintl_sc.swrst_bit); 
			of_property_read_u32_index(vout->v4l2_np, "deintl_scaler", 1, &vioc->deintl_sc.id);
			vioc->deintl_sc.addr = (PVIOC_SC)of_iomap(dev_np, vioc->deintl_sc.id);
			dprintk("[DEINTL] SCALER < vir_addr = 0x%x , id = %d , reset = %d \n", vioc->deintl_sc.addr, vioc->deintl_sc.id, vioc->deintl_sc.swrst_bit);
		} else {
			printk("[DEINTL] could not find scaler node of vout driver. \n");
		}
	} else {
		if (!vout->opened) {
			printk(KERN_ERR VOUT_NAME ": [error] not open vout driver.\n");
			return -EBUSY;
		}
		
	}

	switch (vioc->deintl_rdma.id) {
	case VIOC_RDMA_07:
		vioc->deintl_wmix.pos = 3;				// rdma pos 3 is VRDMA
		vioc->sub_wmix.pos = 1;					// rdma pos 1 is GRDMA
		vioc->sub_wmix.ovp = 24;				// ovp 24: 0-1-2-3
		break;
	case VIOC_RDMA_11:
		vioc->deintl_wmix.pos = 3;					// rdma pos 3 is VRDMA
		vioc->sub_wmix.pos = 2;						// rdma pos 0/1/2 is GRDMA8/9/10
		vioc->sub_wmix.ovp = 24;					// ovp 24: 0-1-2-3
		break;
	case VIOC_RDMA_12:
		vioc->deintl_wmix.pos = 0;				// rdma pos 0 is VRDMA
		vioc->sub_wmix.pos = 1;					// rdma pos 1 is GRDMA
		vioc->sub_wmix.ovp = 5;					// ovp 5: 3-2-1-0
		break;
	default:
		dprintk("invalid rdma index\n");
		return -1;
	}

	// set deinterlace wmix
	dev_np = of_parse_phandle(vout->v4l2_np, "deintl_wmix", 0);
	if(dev_np) {
		/* swreser bit */
		of_property_read_u32_index(vout->v4l2_np, "deintl_wmix", 2, &vioc->deintl_wmix.swrst_bit);
		of_property_read_u32_index(vout->v4l2_np, "deintl_wmix", 1, &vioc->deintl_wmix.id); 
		vioc->deintl_wmix.addr = (PVIOC_WMIX)of_iomap(dev_np, vioc->deintl_wmix.id);
		dprintk("[DEINTL] WMIX < vir_addr = 0x%x , id = %d , reset = %d \n", vioc->deintl_wmix.addr, vioc->deintl_wmix.id, vioc->deintl_wmix.swrst_bit);
	} else {
		printk("[DEINTL] could not find wmix node of vout driver. \n");
	}

	// set deinterlace wdma
	dev_np = of_parse_phandle(vout->v4l2_np, "deintl_wdma", 0);
	if(dev_np) {
		/* swreser bit */
		of_property_read_u32_index(vout->v4l2_np, "deintl_wdma", 2, &vioc->deintl_wdma.swrst_bit);
		of_property_read_u32_index(vout->v4l2_np, "deintl_wdma", 1, &vioc->deintl_wdma.id);
		vioc->deintl_wdma.addr = (PVIOC_WDMA)of_iomap(dev_np, vioc->deintl_wdma.id);
		vioc->deintl_wdma.irq = irq_of_parse_and_map(dev_np, vioc->deintl_wdma.id);
		vioc->deintl_wdma.vioc_intr = kzalloc(sizeof(struct vioc_intr_type), GFP_KERNEL);
		if(vioc->deintl_wdma.vioc_intr == 0) {
			printk("memory allocation faiil (vioc->deintl_wdma)\n");
			return -ENOMEM;
		}
		vioc->deintl_wdma.vioc_intr->id = VIOC_INTR_WD0+vioc->deintl_wdma.id;
		vioc->deintl_wdma.vioc_intr->bits = VIOC_WDMA_IREQ_EOFR_MASK;
		dprintk("[DEINTL] WDMA < vir_addr = 0x%x , id = %d , reset = %d , irq = %d \n", vioc->deintl_wdma.addr, vioc->deintl_wdma.id, vioc->deintl_wdma.swrst_bit, vioc->deintl_wdma.irq);
	} else {
		printk("[DEINTL] could not find wdma node of vout driver. \n");
	}

	// set subtitle rdma
	dev_np = of_parse_phandle(vout->v4l2_np, "subtitle_rdma", 0);
	if(dev_np) {
		/* swreser bit */
		of_property_read_u32_index(vout->v4l2_np, "subtitle_rdma", 2, &vioc->sub_rdma.swrst_bit);
		of_property_read_u32_index(vout->v4l2_np, "subtitle_rdma", 1, &vioc->sub_rdma.id);
		vioc->sub_rdma.addr = (PVIOC_RDMA)of_iomap(dev_np, vioc->sub_rdma.id);
		dprintk("[SUB] RDMA < vir_addr = 0x%x , id = %d , reset = %d \n", vioc->sub_rdma.addr, vioc->sub_rdma.id, vioc->sub_rdma.swrst_bit);
	} else {
		printk("[SUB] could not find rdma node of vout driver. \n");
	}

	// set subtitle wmixer
	vioc->sub_wmix.id = vioc->deintl_wmix.id;
	vioc->sub_wmix.addr =  vioc->deintl_wmix.addr;
	vioc->sub_wmix.swrst_bit = vioc->deintl_wmix.swrst_bit;

	// set deintl_viqe information
	dev_np = of_parse_phandle(vout->v4l2_np, "deintl_viqe", 0);
	if(dev_np) {
		/* path plugin */
		of_property_read_u32(vout->v4l2_np, "viqe_path", &vioc->deintl_viqe.ch);
		of_property_read_u32(vout->v4l2_np, "viqe_plugin", &vioc->deintl_viqe.plugin);

		/* swreser bit */
		of_property_read_u32_index(vout->v4l2_np, "deintl_viqe", 2, &vioc->deintl_viqe.swrst_bit);
		of_property_read_u32_index(vout->v4l2_np, "deintl_viqe", 1, &vioc->deintl_viqe.id);
		vioc->deintl_viqe.addr = (PVIQE)of_iomap(dev_np, vioc->deintl_viqe.id);
		dprintk("[DEINTL] VIQE < vir_addr = 0x%x , id = %d , reset = %d \n", vioc->deintl_viqe.addr, vioc->deintl_viqe.id, vioc->deintl_viqe.swrst_bit);
	} else {
		printk("[DEINTL] could not find viqe node of vout driver. \n");
	}

	dprintk("RDMA%d - %sSC%d - WMIX%d - WDMA%d\n", vioc->deintl_rdma.id,
		vout->deinterlace == VOUT_DEINTL_NONE ? "" : "DEINTL - ",
		vioc->deintl_sc.id, vioc->deintl_wmix.id, vioc->deintl_wdma.id);

	#ifdef CONFIG_ARCH_TCC_MAP_CONVERTER
	dev_np = of_find_compatible_node(NULL, NULL, "telechips,vioc_mc");
	if(dev_np) {
		vioc->deintl_mc.id = VIOC_MC_00;
		vioc->deintl_mc.addr = (VIOC_MC *)of_iomap(dev_np, vioc->deintl_mc.id);
		dprintk("[DEINTL] MC < vir_addr = 0x%x , id = %d \n", vioc->deintl_mc.addr, vioc->deintl_mc.id);
	}
	#endif

	#if defined(CONFIG_TCC_DISPLAY_MODE_DUAL_HDMI_CVBS) || defined(CONFIG_TCC_DISPLAY_MODE_DUAL_ALL)
	// set extend display device info
	dev_np = of_parse_phandle(vout->v4l2_np, "ext_disp", 0);
	if(dev_np) {
		of_property_read_u32_index(vout->v4l2_np, "ext_disp", 1, &vioc->ext_disp.id);
		vioc->ext_disp.addr = (PVIOC_DISP)of_iomap(dev_np, vioc->ext_disp.id);
		dprintk("[EXT] DISP < vir_addr = 0x%x , id = %d \n", vioc->ext_disp.addr, vioc->ext_disp.id);
	} else {
		printk("[EXT] could not find disp node of vout driver. \n");
	}

	dev_np = of_parse_phandle(vout->v4l2_np, "ext_rdma", 0);
	if(dev_np) {
		of_property_read_u32_index(vout->v4l2_np, "ext_rdma", 1, &vioc->ext_rdma.id);
		vioc->ext_rdma.addr = (PVIOC_RDMA)of_iomap(dev_np, vioc->ext_rdma.id);
		dprintk("[EXT] RDMA < vir_addr = 0x%x , id = %d \n", vioc->ext_rdma.addr, vioc->ext_rdma.id);
	} else {
		printk("[EXT] could not find rdma node of vout driver. \n");
	}
	#endif

	return 0;
}

int vout_set_vout_path(struct tcc_vout_device *vout)
{
	struct device_node *dev_np;
	struct tcc_vout_vioc *vioc = vout->vioc;

	switch (vioc->rdma.id) {
	case VIOC_RDMA_02:
		vioc->wmix.id = VIOC_WMIX_00;
		vioc->wmix.pos = 3;
		#ifdef VOUT_DISP_PATH_SCALER
		vioc->sc.plugin = VIOC_SC_RDMA_02;
		#endif
		vioc->disp.id = 0;
		break;
	case VIOC_RDMA_03:
		vioc->wmix.id = VIOC_WMIX_00;
		vioc->wmix.pos = 3;
		#ifdef VOUT_DISP_PATH_SCALER
		vioc->sc.plugin = VIOC_SC_RDMA_03;
		#endif
		vioc->disp.id = 0;
		break;
	case VIOC_RDMA_07:
		vioc->wmix.id = VIOC_WMIX_01;
		vioc->wmix.pos = 3;
		#ifdef VOUT_DISP_PATH_SCALER
		vioc->sc.plugin = VIOC_SC_RDMA_07;
		#endif
		vioc->disp.id = 1;
		break;
	default:
		dprintk("invalid rdma index\n");
		return -1;
	}

	// set display rdma
	dev_np = of_parse_phandle(vout->v4l2_np, "rdma", 0);
	if(dev_np) {
		/* swreser bit */
		of_property_read_u32_index(vout->v4l2_np, "rdma", 2, &vioc->rdma.swrst_bit);
		vioc->rdma.addr = (PVIOC_RDMA)of_iomap(dev_np, vioc->rdma.id);
		dprintk("[DISP] RDMA < vir_addr = 0x%x , id = %d , reset = %d \n", vioc->rdma.addr, vioc->rdma.id, vioc->rdma.swrst_bit);
	} else {
		printk("[DISP] could not find rdma node of vout driver. \n");
	}

	// set subtitle rdma for onthefly
	dev_np = of_parse_phandle(vout->v4l2_np, "subtitle_rdma_ext", 0);
	if(dev_np) {
		/* swreser bit */
		of_property_read_u32_index(vout->v4l2_np, "subtitle_rdma_ext", 2, &vioc->sub_rdma_ext.swrst_bit);
		of_property_read_u32_index(vout->v4l2_np, "subtitle_rdma_ext", 1, &vioc->sub_rdma_ext.id);
		vioc->sub_rdma_ext.addr = (PVIOC_RDMA)of_iomap(dev_np, vioc->sub_rdma_ext.id);
		dprintk("[SUB] RDMA < vir_addr = 0x%x , id = %d , reset = %d \n", vioc->sub_rdma_ext.addr, vioc->sub_rdma_ext.id, vioc->sub_rdma_ext.swrst_bit);
	} else {
		printk("[DISP] could not find sub rdma node of vout driver. \n");
	}

	// set display wmix
	dev_np = of_parse_phandle(vout->v4l2_np, "wmix", 0);
	if(dev_np) {
		/* swreser bit */
		of_property_read_u32_index(vout->v4l2_np, "wmix", 2, &vioc->wmix.swrst_bit);
		of_property_read_u32_index(vout->v4l2_np, "wmix", 1, &vioc->wmix.id);
		vioc->wmix.addr = (PVIOC_WMIX)of_iomap(dev_np, vioc->wmix.id);
		dprintk("[DISP] WMIX < vir_addr = 0x%x , id = %d , reset = %d \n", vioc->wmix.addr, vioc->wmix.id, vioc->wmix.swrst_bit);
	} else {
		printk("[DISP] could not find wmix node of vout driver. \n");
	}

	switch (vioc->sub_rdma_ext.id) {
	case VIOC_RDMA_01:
		vioc->sub_wmix_ext.pos = 1;					// rdma pos 1 is GRDMA
		break;
	case VIOC_RDMA_02:
		vioc->sub_wmix_ext.pos = 2;					// rdma pos 0/1/2 is GRDMA8/9/10
		break;
	case VIOC_RDMA_05:
		vioc->sub_wmix_ext.pos = 1;					// rdma pos 1 is GRDMA
		break;
	default:
		dprintk("invalid rdma index\n");
		return -1;
	}

	// set onthefly wmixer
	vioc->sub_wmix_ext.id = vioc->wmix.id;
	vioc->sub_wmix_ext.addr =  vioc->wmix.addr;
	vioc->sub_wmix_ext.swrst_bit = vioc->wmix.swrst_bit;

	dev_np = of_parse_phandle(vout->v4l2_np, "disp", 0);
	if(dev_np) {
		of_property_read_u32_index(vout->v4l2_np, "disp", 1, &vioc->disp.id);
		vioc->disp.addr = (PVIOC_DISP)of_iomap(dev_np, vioc->disp.id);
		vioc->disp.irq = irq_of_parse_and_map(dev_np, vioc->disp.id);

		#ifdef CONFIG_VOUT_USE_VSYNC_INT
		vioc->disp.vioc_intr = kzalloc(sizeof(struct vioc_intr_type), GFP_KERNEL);
		if(vioc->disp.vioc_intr == 0) {
			printk("memory allocation faiil (vioc->disp)\n");
			return -ENOMEM;
		}
		vioc->disp.vioc_intr->id = vioc->disp.id;
		vioc->disp.vioc_intr->bits = (1<<VIOC_DISP_INTR_VSF);
		#endif

		dprintk("[DISP] DISP < vir_addr = 0x%x , id = %d , irq = %d \n", vioc->disp.addr, vioc->disp.id, vioc->disp.irq);
	} else {
		printk("[DISP] could not find disp node of vout driver. \n");
	}

	dev_np = of_parse_phandle(vout->v4l2_np, "config", 0);
	if(dev_np) {
		vioc->config = (PVIOC_IREQ_CONFIG)of_iomap(dev_np, 0);
		dprintk("CONFIG < vir_addr = 0x%x \n", vioc->config);
	} else {
		printk("could not find config node of vout driver. \n");
	}

#ifdef VOUT_DISP_PATH_SCALER
	if (vioc->sc.id < 0)
		goto not_use_sc;

	switch (vioc->sc.id) {
	case VIOC_SC_00:
		vioc->sc.ch = VIOC_SC0;
		break;
	case VIOC_SC_01:
		vioc->sc.ch = VIOC_SC1;
		break;
	case VIOC_SC_02:
		vioc->sc.ch = VIOC_SC2;
		break;
	case VIOC_SC_03:
		vioc->sc.ch = VIOC_SC3;
		break;
	default:
		dprintk("invalid scaler index. so not used scaler.\n");
		vioc->sc.id = -1;
		goto not_use_sc;
	}

	// display scaler
	dev_np = of_parse_phandle(vout->v4l2_np, "scaler", 0);
	if(dev_np) {
		/* swreser bit */
		of_property_read_u32_index(vout->v4l2_np, "scaler", 2, &vioc->sc.swrst_bit);
		vioc->sc.addr = (PVIOC_SC)of_iomap(dev_np, vioc->sc.id);
		dprintk("[DISP] SC < vir_addr = 0x%x , id = %d , reset = %d \n", vioc->sc.addr, vioc->sc.id, vioc->sc.swrst_bit);
	} else {
		printk("[DISP] could not find scaler node of vout driver. \n");
	}

	dprintk("RDMA%d - SC%d - WMIX%d - DISP%d\n", vioc->rdma.id, vioc->sc.id, vioc->wmix.id, vioc->disp.id);
	return 0;
not_use_sc:
#endif
	dprintk("RDMA%d - WMIX%d - DISP%d\n", vioc->rdma.id, vioc->wmix.id, vioc->disp.id);
	return 0;
}

/* call by tcc_vout_v4l2_probe()
 */
int vout_vioc_set_default(struct tcc_vout_device *vout)
{
	int ret;
	struct tcc_vout_vioc *vioc = vout->vioc;
	struct device_node *dev_np;

	dev_np = of_parse_phandle(vout->v4l2_np, "rdma", 0);
	if(dev_np) {
		of_property_read_u32_index(vout->v4l2_np, "rdma", 1, &vioc->rdma.id);
		dprintk("RDMA id: %d \n", vioc->rdma.id);
	}

	vioc->sc.id = -1/*VIOC_SC_00*/;

	/* |========= WMIX Overlay Priority =========|
	 * | OVP | LAYER3 | LAYER2 | LAYER1 | LAYER0 |
	 * |-----------------------------------------|
	 * |  24 | image0 | image1 | image2 | image3 |
	 * |  25 | image0 | image2 | image1 | image3 |
	 * |=========================================|
	 * (LAYER3 is highest layer)
	 */
	vioc->wmix.ovp = 24;

	/* get pmap for disp_bufs (only V4L2_MEMORY_MMAP) */
	memcpy(&vout->pmap.name, VOUT_DISP_PATH_PMAP_NAME, sizeof(VOUT_DISP_PATH_PMAP_NAME));
	ret = vout_set_vout_path(vout);

	ret |= vout_set_deintl_path(1, vout);

	#ifdef CONFIG_VOUT_USE_VSYNC_INT
	vout_timer = tcc_register_timer(NULL, 1000/*msec*/, NULL);
	if (IS_ERR(vout_timer)) {
		printk(KERN_ERR "%s: cannot register tcc timer. ret:0x%x\n", __func__, (unsigned)vout_timer);
		vout_timer = NULL;
	}
	#endif

	return ret;
}

void vout_disp_ctrl(struct tcc_vout_vioc *vioc, int enable)
{
	if (enable)
		VIOC_RDMA_SetImageEnable(vioc->rdma.addr);
	else
		VIOC_RDMA_SetImageDisable(vioc->rdma.addr);
	dprintk("%d\n", enable);
}

void vout_chromakey_enable(struct tcc_vout_vioc *vioc)
{
	/* overlay open function doesn't have chromakey enable code.
	 * but, overlay release function disable chromakey disable.
	 * (ticket #1681)
	 */
	VIOC_RDMA *rdma_ui = (VIOC_RDMA *)tcc_p2v(vioc->disp.id ? HwVIOC_RDMA04 : HwVIOC_RDMA00);
	unsigned int lcd_lcdc_fmt = rdma_ui->uCTRL.bREG.FMT;
	if (lcd_lcdc_fmt == TCC_LCDC_IMG_FMT_RGB565) {
		vioc->wmix.addr->uKEY0.bREG.KEYEN = 1;
	}
	dprintk("\n");
}

void vout_rdma_setup(struct tcc_vout_device *vout)
{
	struct tcc_vout_vioc *vioc = vout->vioc;
	struct vioc_rdma *disp_rdma = &vioc->rdma;

	disp_rdma->width = vioc->deintl_wdma.width;
	disp_rdma->height = vioc->deintl_wdma.height;
	VIOC_RDMA_SetImageSize(disp_rdma->addr, disp_rdma->width, disp_rdma->height);
	VIOC_RDMA_SetImageFormat(disp_rdma->addr, disp_rdma->fmt);
	VIOC_RDMA_SetImageOffset(disp_rdma->addr, disp_rdma->fmt, disp_rdma->width);
	VIOC_RDMA_SetImageY2REnable(disp_rdma->addr, disp_rdma->y2r);
	VIOC_RDMA_SetImageY2RMode(disp_rdma->addr, disp_rdma->y2rmd);
	VIOC_RDMA_SetImageDisable(disp_rdma->addr);
}

void vout_scaler_setup(struct tcc_vout_device *vout)
{
	struct tcc_vout_vioc *vioc = vout->vioc;
	unsigned int sw, sh;	// src image size in rdma
	unsigned int dw, dh;	// destination size in scaler
	int bypass = 0;

	sw = vout->src_pix.width;
	sh = vout->src_pix.height;
	dw = vout->disp_rect.width;
	dh = vout->disp_rect.height;

	if (dw == sw && dh == sh) {
		dprintk("Bypass scaler (same size)\n");
		bypass = 1;
	}

	VIOC_SC_SetBypass(vioc->sc.addr, bypass);
	VIOC_SC_SetDstSize(vioc->sc.addr, dw, dh);				// set destination size in scaler
	VIOC_SC_SetOutSize(vioc->sc.addr, dw, dh);				// set output size in scaler
	VIOC_CONFIG_PlugIn(vioc->sc.ch, vioc->sc.plugin);		// plugin position in scaler
	VIOC_SC_SetUpdate(vioc->sc.addr);

	dprintk("%dx%d->[scaler]->%dx%d\n", sw, sh, dw, dh);
}

void vout_wmix_setup(struct tcc_vout_device *vout)
{
	struct tcc_vout_vioc *vioc = vout->vioc;

	VIOC_WMIX_SetPosition(vioc->wmix.addr,
							vioc->wmix.pos,
							vout->disp_rect.left, vout->disp_rect.top);
	#if defined(CONFIG_TCC_VIOCMG)
	viocmg_set_wmix_ovp(VIOCMG_CALLERID_VOUT, vioc->wmix.id, vioc->wmix.ovp);
	#endif
	VIOC_WMIX_SetUpdate(vioc->wmix.addr);

	dprintk("wmix position(%d,%d)\n", vout->disp_rect.left, vout->disp_rect.top);
}

void vout_wmix_getsize(struct tcc_vout_device *vout, unsigned int *w, unsigned int *h)
{
	struct tcc_vout_vioc *vioc = vout->vioc;

	VIOC_WMIX_GetSize(vioc->wmix.addr, w, h);
}

void vout_path_reset(struct tcc_vout_vioc *vioc)
{
	/* reset state */
#ifdef VOUT_DISP_PATH_SCALER
	BITSET(vioc->config->uSOFTRESET.nREG[0], (0x1 << vioc->sc.swrst_bit));		// scaler
#endif
	BITSET(vioc->config->uSOFTRESET.nREG[0], (0x1 << vioc->rdma.swrst_bit));	// rdma

	/* normal state */
	BITCLR(vioc->config->uSOFTRESET.nREG[0], (0x1 << vioc->rdma.swrst_bit));	// rdma
#ifdef VOUT_DISP_PATH_SCALER
	BITCLR(vioc->config->uSOFTRESET.nREG[0], (0x1 << vioc->sc.swrst_bit));		// scaler
#endif
	
	dprintk("[DISP] RDMA: %d \n", vioc->rdma.swrst_bit);
}

void deintl_path_reset(struct tcc_vout_vioc *vioc)
{
	/* reset state */
	BITCSET(vioc->config->uSOFTRESET.nREG[1], (0x1 << vioc->deintl_wdma.swrst_bit), (0x1 << vioc->deintl_wdma.swrst_bit));	// wdma
	BITCSET(vioc->config->uSOFTRESET.nREG[0], (0x1 << vioc->deintl_sc.swrst_bit), (0x1 << vioc->deintl_sc.swrst_bit));		// sc
	BITCSET(vioc->config->uSOFTRESET.nREG[1], (0x1 << vioc->deintl_wmix.swrst_bit), (0x1 << vioc->deintl_wmix.swrst_bit));	// wmix
	BITCSET(vioc->config->uSOFTRESET.nREG[0], (0x1 << vioc->deintl_rdma.swrst_bit), (0x1 << vioc->deintl_rdma.swrst_bit));	// rdma

	/* normal state */
	BITCSET(vioc->config->uSOFTRESET.nREG[0], (0x1 << vioc->deintl_rdma.swrst_bit), (0x0 << vioc->deintl_rdma.swrst_bit));	// rdma
	BITCSET(vioc->config->uSOFTRESET.nREG[1], (0x1 << vioc->deintl_wmix.swrst_bit), (0x0 << vioc->deintl_wmix.swrst_bit));	// wmix
	BITCSET(vioc->config->uSOFTRESET.nREG[0], (0x1 << vioc->deintl_sc.swrst_bit), (0x0 << vioc->deintl_sc.swrst_bit));		// sc
	BITCSET(vioc->config->uSOFTRESET.nREG[1], (0x1 << vioc->deintl_wdma.swrst_bit), (0x0 << vioc->deintl_wdma.swrst_bit));	// wdma

	dprintk("[DEINTL] RDMA: %d , WMIX: %d , WDMA: %d \n", vioc->deintl_rdma.swrst_bit, vioc->deintl_wmix.swrst_bit, vioc->deintl_wdma.swrst_bit);
}

void deintl_rdma_setup(struct vioc_rdma *rdma)
{
	rdma->addr->uCTRL.bREG.SWAP = rdma->swap;

	VIOC_RDMA_SetImageBfield(rdma->addr, rdma->bf);
	VIOC_RDMA_SetImageIntl(rdma->addr, rdma->intl);

	VIOC_RDMA_SetImageY2REnable(rdma->addr, rdma->y2r);
	VIOC_RDMA_SetImageY2RMode(rdma->addr, rdma->y2rmd);

	VIOC_RDMA_SetImageBase(rdma->addr, rdma->img.base0, rdma->img.base1, rdma->img.base2);
	VIOC_RDMA_SetImageSize(rdma->addr, rdma->width, rdma->height);
	VIOC_RDMA_SetImageFormat(rdma->addr, rdma->fmt);

	if (rdma->y_stride) {
		if(rdma->fmt > VIOC_IMG_FMT_ARGB6666_3)
			VIOC_RDMA_SetImageOffset(rdma->addr, rdma->fmt, rdma->y_stride);
		else
			BITCSET(rdma->addr->uOFFSET.nREG, 0xFFFFFFFF, rdma->y_stride);
	} else {
		VIOC_RDMA_SetImageOffset(rdma->addr, rdma->fmt, rdma->width);
	}
	VIOC_RDMA_SetImageUpdate(rdma->addr);
}

static void deintl_scaler_setup(struct tcc_vout_device *vout)
{
	struct tcc_vout_vioc *vioc = vout->vioc;
	unsigned int sw, sh;	// src image size in rdma
	unsigned int dw, dh;	// destination size in scaler
	int bypass = 0;

	sw = vout->src_pix.width;
	sh = vout->src_pix.height;
	dw = vout->disp_rect.width;
	dh = vout->disp_rect.height;

	if (dw == sw && dh == sh) {
		dprintk("Bypass scaler (same size)\n");
		bypass = 1;
	}

	VIOC_SC_SetBypass(vioc->deintl_sc.addr, bypass);
	VIOC_SC_SetDstSize(vioc->deintl_sc.addr, dw, dh);				// set destination size in scaler
	VIOC_SC_SetOutSize(vioc->deintl_sc.addr, dw, dh);				// set output size in scaler
	VIOC_CONFIG_PlugIn(vioc->deintl_sc.ch, vioc->deintl_sc.plugin);	// plugin position in scaler
	VIOC_SC_SetUpdate(vioc->deintl_sc.addr);

	dprintk("%dx%d->[scaler]->%dx%d\n", sw, sh, dw, dh);
}

static void deintl_wdma_setup(struct vioc_wdma *wdma)
{
	VIOC_WDMA_SetImageR2YEnable(wdma->addr, wdma->r2y);
	VIOC_WDMA_SetImageR2YMode(wdma->addr, wdma->r2ymd);
	VIOC_WDMA_SetImageBase(wdma->addr, wdma->img.base0, wdma->img.base1, wdma->img.base2);
	VIOC_WDMA_SetImageSize(wdma->addr, wdma->width, wdma->height);
	VIOC_WDMA_SetImageFormat(wdma->addr, wdma->fmt);
	VIOC_WDMA_SetImageOffset(wdma->addr, wdma->fmt, wdma->width);
	VIOC_WDMA_SetContinuousMode(wdma->addr, wdma->cont);
	VIOC_WDMA_SetImageUpdate(wdma->addr);
}

static int deintl_s_setup(struct tcc_vout_device *vout)
{
	struct tcc_vout_vioc *vioc = vout->vioc;
	volatile unsigned long *vioc_deintls = (volatile unsigned long *)tcc_p2v(HwVIOC_DEINTLS);
	int ret = 0;

	/* reset deintl_s */
	BITCSET(vioc->config->uSOFTRESET.nREG[1], 0x1 << 17, 0x1 << 17);
	BITCSET(vioc->config->uSOFTRESET.nREG[1], 0x1 << 17, 0x0 << 17);

	/*soc guide*/
	BITCSET(*vioc_deintls, 0x7, 0x3);

	ret = VIOC_CONFIG_PlugIn(VIOC_DEINTLS, vioc->deintl_viqe.plugin);

	dprintk("%s\n", ret ? "failed" : "success");
	return ret == 0 ? 0 : -EFAULT;
}

static int deintl_viqe_setup(struct tcc_vout_device *vout, enum deintl_type deinterlace, int plugin)
{
	struct tcc_vout_vioc *vioc = vout->vioc;
	pmap_t pmap_viqe;
	unsigned int viqe_deintl_base[4] = {0};
	unsigned int framebufferWidth, framebufferHeight;
	int img_size, ret = 0;

	/* viqe config variable */
	int vmisc_tsdu = 0;									// 0: viqe size is get from vioc module
	VIOC_VIQE_DEINTL_MODE di_mode;
	VIOC_VIQE_FMT_TYPE di_fmt = VIOC_VIQE_FMT_YUV420;	// DI_DECn_MISC.FMT, DI_FMT.F422 register
														//TODO: VIOC_VIQE_FMT_YUV420 = 0, VIOC_VIQE_FMT_YUV422 = 1
	framebufferWidth = (vioc->deintl_rdma.width >> 3) << 3;
	framebufferHeight = (vioc->deintl_rdma.height >> 2) << 2;

	/* reset viqe */
	BITCSET(vioc->config->uSOFTRESET.nREG[1], (0x1 << vioc->deintl_viqe.swrst_bit), (0x1 << vioc->deintl_viqe.swrst_bit));
	BITCSET(vioc->config->uSOFTRESET.nREG[1], (0x1 << vioc->deintl_viqe.swrst_bit), (0x0 << vioc->deintl_viqe.swrst_bit));

	img_size = (framebufferWidth * framebufferHeight * 2);

	if (deinterlace == VOUT_DEINTL_VIQE_3D) {
		di_mode = VIOC_VIQE_DEINTL_MODE_2D;

		pmap_get_info("viqe", &pmap_viqe);
		dprintk("[PMAP] %s: 0x%08x ~ 0x%08x (0x%08x)\n",
			pmap_viqe.name, pmap_viqe.base, pmap_viqe.base + pmap_viqe.size, pmap_viqe.size);

		viqe_deintl_base[0] = pmap_viqe.base;
		viqe_deintl_base[1] = viqe_deintl_base[0] + img_size;
		viqe_deintl_base[2] = viqe_deintl_base[1] + img_size;
		viqe_deintl_base[3] = viqe_deintl_base[2] + img_size;

		if ((viqe_deintl_base[3] + img_size) > (pmap_viqe.base + pmap_viqe.size)) {
			printk(KERN_ERR VOUT_NAME ": [error] pmap_viqe no space\n");
			//return -ENOBUFS;
		}
	} else if (deinterlace == VOUT_DEINTL_VIQE_2D) {
		di_mode = VIOC_VIQE_DEINTL_MODE_2D;
	} else {
		di_mode = VIOC_VIQE_DEINTL_MODE_BYPASS;
	}

	if(vmisc_tsdu == OFF) {
		framebufferWidth = 0;
		framebufferHeight = 0;
	}

	VIOC_VIQE_SetControlRegister(vioc->deintl_viqe.addr, framebufferWidth, framebufferHeight, di_fmt);
	VIOC_VIQE_SetDeintlRegister(vioc->deintl_viqe.addr,
								di_fmt, vmisc_tsdu, framebufferWidth, framebufferHeight, di_mode,
								viqe_deintl_base[0], viqe_deintl_base[1], viqe_deintl_base[2], viqe_deintl_base[3]);
	VIOC_VIQE_SetControlEnable(vioc->deintl_viqe.addr,
								OFF,	/* histogram CDF or LUT */
								OFF,	/* histogram */
								OFF,	/* gamut mapper */
								OFF,	/* de-noiser */
								ON		/* de-interlacer */
								);
	VIOC_VIQE_SetImageY2REnable(vioc->deintl_viqe.addr, vioc->deintl_viqe.y2r);
	VIOC_VIQE_SetImageY2RMode(vioc->deintl_viqe.addr, vioc->deintl_viqe.y2rmd);

	if (plugin)
		ret = VIOC_CONFIG_PlugIn(vioc->deintl_viqe.ch, vioc->deintl_viqe.plugin);

	return ret == 0 ? 0 : -EFAULT;
}

#ifdef CONFIG_VOUT_USE_VSYNC_INT
inline static void tcc_vout_set_time(struct tcc_vout_device *vout, int time)
{
	vout->baseTime = tcc_get_timer_count(vout_timer) - time;
}

inline static int tcc_vout_get_time(struct tcc_vout_device *vout)
{
	return tcc_get_timer_count(vout_timer) - vout->baseTime;
}

static int vout_check_syncTime(struct tcc_vout_device *vout, struct v4l2_buffer *buf, unsigned long base_time)
{
	unsigned long current_time;
	int diff_time, time_zone;
	unsigned int display_hz = hdmi_get_refreshrate();

	if(display_hz == 59)
		display_hz = 60;
	else if(display_hz == 23)
		display_hz = 24;
	vout->update_gap_time = 1000/display_hz;
	time_zone = vout->update_gap_time / 2;

	current_time = ((buf->timestamp.tv_sec*1000)+(buf->timestamp.tv_usec/1000));
	diff_time = (int)(current_time - base_time);

	if(abs(diff_time) <= time_zone/*max time gap*/) {
		dbprintk("[%d]buf->timestamp(%ld msec) base_time(%ld msec) diff_time(%d)\n", buf->index,current_time, base_time, diff_time);
	} else {
		if(diff_time <= -(time_zone)/*limit delay time*/) {
			dprintk("exception status!!! frame drop(%dmsec)\n", diff_time);
			if(diff_time <= -(vout->update_gap_time * 2))
				vout->force_sync = ON;	// to recalculate kernel time
			return VOUT_DRV_ERR_DROPFRM;
		}
		return VOUT_DRV_ERR_WAITFRM;
	}
	return VOUT_DRV_NOERR;
}
#endif

void vout_onthefly_display_update(struct tcc_vout_device *vout, struct v4l2_buffer *buf)
{
	struct tcc_vout_vioc *vioc = vout->vioc;
	unsigned int base0=0, base1=0, base2=0;
	int res_change = OFF;

	if(2 == buf->m.planes[MPLANE_VID].reserved[VID_NUM]) {
		memcpy(&vioc->sub_alpha, buf->m.planes[MPLANE_SUB].reserved, sizeof(struct vioc_alpha));
		vioc->sub_rdma_ext.img.base0 = buf->m.planes[MPLANE_SUB].m.mem_offset;
		vout_subplane_onthefly_qbuf(vout);
	} else {
		if(vioc->sub_rdma_ext.addr->uCTRL.nREG & Hw28) {
			dprintk("subtitle layer OFF!!! \n");
			VIOC_RDMA_SetImageDisableNW(vioc->sub_rdma_ext.addr);
		}
	}

	/* get base address */
	if (V4L2_MEMORY_USERPTR == vout->memory) {
		if (STILL_IMGAGE != buf->timecode.type) {
			#ifdef VOUT_SUB_PLANE
			base0 = buf->m.planes[MPLANE_VID].m.userptr;
			base1 = buf->m.planes[MPLANE_VID].reserved[VID_BASE1];
			base2 = buf->m.planes[MPLANE_VID].reserved[VID_BASE2];
			#else
			base0 = buf->m.offset;
			base1 = buf->input;
			base2 = buf->reserved;
			#endif
		} else {
			#ifdef VOUT_SUB_PLANE
			base0 = buf->m.planes[MPLANE_VID].m.userptr;
			#else
			base0 = buf->m.offset;
			#endif
			base1 = base2 = 0;

			/*
			 * If the input is YUV format.
			 */
			if (TCC_PFMT_RGB != vout->pfmt) {
				tcc_get_base_address_of_image(
					vout->src_pix.pixelformat, base0,
					vout->src_pix.width, vout->src_pix.height,
					&base0, &base1, &base2);
			}
		}
	} else if (V4L2_MEMORY_MMAP == vout->memory) {
		base0 = vout->qbufs[buf->index].img_base0;
		base1 = vout->qbufs[buf->index].img_base1;
		base2 = vout->qbufs[buf->index].img_base2;
	} else {
		printk(KERN_ERR VOUT_NAME ": [error] invalid qbuf v4l2_memory\n");
	}

	if (V4L2_FIELD_INTERLACED_BT == vout->src_pix.field)
		vioc->rdma.bf = 1;
	else
		vioc->rdma.bf = 0;

	/* To prevent frequent switching of the scan type */
	if (vout->first_frame) {
		vout->first_field = vout->src_pix.field;
		vout->first_frame = 0;
		dprintk("first_field(%s)\n", v4l2_field_names[vout->first_field]);
	} else {
		vout->src_pix.field = vout->first_field;
	}

	// plug-out VIQE, Scaler
	if(vioc->deintl_sc.plugin != VIOC_RDMA_03)
	{
		dprintk("viqe %d plug out!!! \n", vioc->deintl_viqe.id);
		VIOC_CONFIG_PlugOut(vioc->deintl_viqe.ch);
		VIOC_CONFIG_SWReset(vioc->config, VIOC_CONFIG_VIQE, vioc->deintl_viqe.id, VIOC_CONFIG_RESET);
		VIOC_CONFIG_SWReset(vioc->config, VIOC_CONFIG_VIQE, vioc->deintl_viqe.id, VIOC_CONFIG_CLEAR);

		dprintk("scaler %d plug out!!! \n", vioc->deintl_sc.id);
		VIOC_CONFIG_PlugOut(vioc->deintl_sc.ch);
		VIOC_CONFIG_SWReset(vioc->config, VIOC_CONFIG_SCALER, vioc->deintl_sc.id, VIOC_CONFIG_RESET);
		VIOC_CONFIG_SWReset(vioc->config, VIOC_CONFIG_SCALER, vioc->deintl_sc.id, VIOC_CONFIG_CLEAR);
	}

	#ifdef CONFIG_ARCH_TCC_MAP_CONVERTER
	if(buf->m.planes[MPLANE_VID].reserved[HEVC_COMP_MAP_EN]) {
		int enable;
		VIOC_RDMA_GetImageEnable(vioc->rdma.addr, &enable);
		if(enable) {
			dprintk("_________________________________%s enable check \n", __func__);

			/* rdma disable */
			VIOC_RDMA_SetImageDisable(vioc->rdma.addr);
		}

		/* update map_converter info */
		vioc->deintl_mc.mapConv_info.m_uiLumaStride = buf->m.planes[MPLANE_VID].reserved[HEVC_LUMA_STRIDE];
		vioc->deintl_mc.mapConv_info.m_uiChromaStride = buf->m.planes[MPLANE_VID].reserved[HEVC_CHROMA_STRIDE];
		vioc->deintl_mc.mapConv_info.m_uiLumaBitDepth = buf->m.planes[MPLANE_VID].reserved[HEVC_LUMA_BIT_DEPTH];
		vioc->deintl_mc.mapConv_info.m_uiChromaBitDepth = buf->m.planes[MPLANE_VID].reserved[HEVC_CHROMA_BIT_DEPTH];
		vioc->deintl_mc.mapConv_info.m_uiFrameEndian = buf->m.planes[MPLANE_VID].reserved[HEVC_FRMAE_ENDIAN];

		vioc->deintl_mc.mapConv_info.m_CompressedY[0] = buf->m.planes[MPLANE_VID].reserved[HEVC_FRAME_BASE_Y0];
		vioc->deintl_mc.mapConv_info.m_CompressedY[1] = buf->m.planes[MPLANE_VID].reserved[HEVC_FRAME_BASE_Y1];
		vioc->deintl_mc.mapConv_info.m_CompressedCb[0] = buf->m.planes[MPLANE_VID].reserved[HEVC_FRAME_BASE_C0];
		vioc->deintl_mc.mapConv_info.m_CompressedCb[1] = buf->m.planes[MPLANE_VID].reserved[HEVC_FRAME_BASE_C1];

		vioc->deintl_mc.mapConv_info.m_FbcYOffsetAddr[0] = buf->m.planes[MPLANE_VID].reserved[HEVC_OFFSET_BASE_Y0];
		vioc->deintl_mc.mapConv_info.m_FbcYOffsetAddr[1] = buf->m.planes[MPLANE_VID].reserved[HEVC_OFFSET_BASE_Y1];
		vioc->deintl_mc.mapConv_info.m_FbcCOffsetAddr[0] = buf->m.planes[MPLANE_VID].reserved[HEVC_OFFSET_BASE_C0];
		vioc->deintl_mc.mapConv_info.m_FbcCOffsetAddr[1] = buf->m.planes[MPLANE_VID].reserved[HEVC_OFFSET_BASE_C1];

		dprintk(" m_uiLumaStride %d m_uiChromaStride %d\n m_uiLumaBitDepth %d m_uiChromaBitDepth %d m_uiFrameEndian %d \n m_CompressedY[0] 0x%08x m_CompressedCb[0] 0x%08x m_CompressedY[1] 0x%08x m_CompressedCb[1] 0x%08x\n m_FbcYOffsetAddr[0] 0x%08x m_FbcCOffsetAddr[0] 0x%08x m_FbcYOffsetAddr[1] 0x%08x m_FbcCOffsetAddr[1] 0x%08x \n",
			vioc->deintl_mc.mapConv_info.m_uiLumaStride, vioc->deintl_mc.mapConv_info.m_uiChromaStride, 
			vioc->deintl_mc.mapConv_info.m_uiLumaBitDepth, vioc->deintl_mc.mapConv_info.m_uiChromaBitDepth, vioc->deintl_mc.mapConv_info.m_uiFrameEndian,
			vioc->deintl_mc.mapConv_info.m_CompressedY[0], vioc->deintl_mc.mapConv_info.m_CompressedCb[0],
			vioc->deintl_mc.mapConv_info.m_CompressedY[1], vioc->deintl_mc.mapConv_info.m_CompressedCb[1],
			vioc->deintl_mc.mapConv_info.m_FbcYOffsetAddr[0], vioc->deintl_mc.mapConv_info.m_FbcCOffsetAddr[0],
			vioc->deintl_mc.mapConv_info.m_FbcYOffsetAddr[1], vioc->deintl_mc.mapConv_info.m_FbcCOffsetAddr[1]);

		dprintk("%s map converter: size: %dx%d  \n", __func__, vout->src_pix.width, vout->src_pix.height);

		/* map converter */
		if(vioc->deintl_sc.plugin != VIOC_RDMA_03){
			dprintk("____________________________________map converter connect to RDMA%d\n", VIOC_RDMA_03);
			tca_set_MC_Connect_to_RDMA(VIOC_MC_00, VIOC_RDMA_03, 1);
			
			vioc->deintl_sc.plugin = VIOC_RDMA_03;
			deintl_scaler_setup(vout);
		}
		tca_map_convter_driver_set(VIOC_MC_00, vout->src_pix.width, vout->src_pix.height, 0, 0, ON, &vioc->deintl_mc.mapConv_info);
		tca_map_convter_onoff(VIOC_MC_00, 1);
	}
	else
	#endif
	{
		/*
		 *	Change source info
		 */
		if((vout->src_pix.width != buf->m.planes[MPLANE_VID].reserved[VID_WIDTH]
			|| vout->src_pix.height != buf->m.planes[MPLANE_VID].reserved[VID_HEIGHT])
			&& (buf->m.planes[MPLANE_VID].reserved[VID_WIDTH] && buf->m.planes[MPLANE_VID].reserved[VID_HEIGHT]))
		{
			dprintk("changing source (%dx%d)->(%dx%d)\n",
				vout->src_pix.width, vout->src_pix.height,
				buf->m.planes[MPLANE_VID].reserved[VID_WIDTH], buf->m.planes[MPLANE_VID].reserved[VID_HEIGHT]);

			vout->src_pix.width = buf->m.planes[MPLANE_VID].reserved[VID_WIDTH];
			vout->src_pix.height = buf->m.planes[MPLANE_VID].reserved[VID_HEIGHT];
			vout->src_pix.bytesperline = vout->src_pix.width * tcc_vout_try_bpp(vout->src_pix.pixelformat, &vout->src_pix.colorspace);

			/* reinit crop_src */
			vout->crop_src.left = 0;
			vout->crop_src.top = 0;
			vout->crop_src.width = vout->src_pix.width;
			vout->crop_src.height = vout->src_pix.height;

			switch(tcc_vout_try_pix(vout->src_pix.pixelformat)){
			case TCC_PFMT_YUV422:
				vout->src_pix.sizeimage = vout->src_pix.width * vout->src_pix.height * 2;
				break;
			case TCC_PFMT_YUV420:
				vout->src_pix.sizeimage = vout->src_pix.width * vout->src_pix.height * 3 / 2;
				break;
			case TCC_PFMT_RGB:
			default:
				vout->src_pix.sizeimage = vout->src_pix.bytesperline * vout->src_pix.height;
				break;
			}
			vout->src_pix.sizeimage = PAGE_ALIGN(vout->src_pix.sizeimage);

			vioc->rdma.width = vout->src_pix.width;
			if(vout->src_pix.height % 4) {
				vioc->rdma.height = ROUND_UP_4(vout->src_pix.height);
				dprintk(" 4-line align: %d -> %d \n", vout->src_pix.height, vioc->deintl_rdma.height);
			} else {
				vioc->rdma.height = vout->src_pix.height;
			}

			vioc->rdma.y_stride = buf->m.planes[MPLANE_VID].bytesused ? buf->m.planes[MPLANE_VID].bytesused : vout->src_pix.bytesperline && 0x0000FFFF;
			VIOC_RDMA_SetImageSize(vioc->rdma.addr, vioc->rdma.width, vioc->rdma.height);
			VIOC_RDMA_SetImageOffset(vioc->rdma.addr, vioc->rdma.fmt, vioc->rdma.y_stride);

			res_change = ON;
		}

		/*
		 *	Change crop info
		 */
		if ((vout->crop_src.left != buf->m.planes[MPLANE_VID].reserved[VID_CROP_LEFT]
			|| vout->crop_src.top != buf->m.planes[MPLANE_VID].reserved[VID_CROP_TOP]
			|| vout->crop_src.width != buf->m.planes[MPLANE_VID].reserved[VID_CROP_WIDTH]
			|| vout->crop_src.height != buf->m.planes[MPLANE_VID].reserved[VID_CROP_HEIGHT])
			&& (buf->m.planes[MPLANE_VID].reserved[VID_CROP_WIDTH] && buf->m.planes[MPLANE_VID].reserved[VID_CROP_HEIGHT])
			&& (vout->src_pix.width >= buf->m.planes[MPLANE_VID].reserved[VID_CROP_WIDTH] && vout->src_pix.height >= buf->m.planes[MPLANE_VID].reserved[VID_CROP_HEIGHT]))
		{
			dprintk("changing crop (%d,%d)(%dx%d)->(%d,%d)(%dx%d)\n",
				vout->crop_src.left, vout->crop_src.top, vout->crop_src.width, vout->crop_src.height,
				buf->m.planes[MPLANE_VID].reserved[VID_CROP_LEFT], buf->m.planes[MPLANE_VID].reserved[VID_CROP_TOP],
				buf->m.planes[MPLANE_VID].reserved[VID_CROP_WIDTH], buf->m.planes[MPLANE_VID].reserved[VID_CROP_HEIGHT]);

			vout->crop_src.left = buf->m.planes[MPLANE_VID].reserved[VID_CROP_LEFT];
			vout->crop_src.top = buf->m.planes[MPLANE_VID].reserved[VID_CROP_TOP];
			vout->crop_src.width = buf->m.planes[MPLANE_VID].reserved[VID_CROP_WIDTH];
			vout->crop_src.height = buf->m.planes[MPLANE_VID].reserved[VID_CROP_HEIGHT];

			tcc_get_base_address(vioc->rdma.fmt, base0,
				vioc->rdma.y_stride ? vioc->rdma.y_stride : vioc->rdma.width,
				vioc->rdma.height,
				vout->crop_src.left, vout->crop_src.top,
				&base0, &base1, &base2);

			vioc->rdma.width = vout->crop_src.width;
			if(vout->crop_src.height % 4) {
				vioc->rdma.height = ROUND_UP_4(vout->crop_src.height);
				dprintk(" 4-line align: %d -> %d \n", vout->crop_src.height, vioc->rdma.height);
			} else {
				vioc->rdma.height = vout->crop_src.height;
			}

			VIOC_RDMA_SetImageSize(vioc->rdma.addr, vioc->rdma.width, vioc->rdma.height);
			res_change = ON;
		}

		/* rdma stride */
		if(buf->m.planes[MPLANE_VID].bytesused && (vioc->rdma.y_stride != buf->m.planes[MPLANE_VID].bytesused)) {
			dprintk("update rdma stride(%d -> %d)\n", vioc->rdma.y_stride, buf->m.planes[MPLANE_VID].bytesused);

			/*  change rdma stride */
			vioc->rdma.y_stride = vout->src_pix.bytesperline = buf->m.planes[MPLANE_VID].bytesused;
			VIOC_RDMA_SetImageOffset(vioc->rdma.addr, vioc->rdma.fmt, vioc->rdma.y_stride);
		}

		if(vioc->deintl_sc.plugin != VIOC_RDMA_03) {
			/*
			 *  Onthefly RDMA
			 */
			vioc->rdma.swap = vioc->deintl_rdma.swap;
			vioc->rdma.y2r = vioc->deintl_rdma.y2r;
			vioc->rdma.fmt = vioc->deintl_rdma.fmt;
			vioc->rdma.intl = vout->deinterlace > VOUT_DEINTL_VIQE_BYPASS ? 1 : 0;
			vioc->rdma.width = res_change ? vioc->rdma.width : vioc->deintl_rdma.width;
			vioc->rdma.height = res_change ? vioc->rdma.height : vioc->deintl_rdma.height;
			vioc->rdma.y_stride = vout->src_pix.bytesperline & 0x0000ffff;

			vioc->rdma.addr->uCTRL.bREG.SWAP = vioc->rdma.swap;
			VIOC_RDMA_SetImageIntl(vioc->rdma.addr, vioc->rdma.intl);
			VIOC_RDMA_SetImageY2REnable(vioc->rdma.addr, vioc->rdma.y2r);
			VIOC_RDMA_SetImageY2RMode(vioc->rdma.addr, vioc->rdma.y2rmd);
			VIOC_RDMA_SetImageSize(vioc->rdma.addr, vioc->rdma.width, vioc->rdma.height);
			VIOC_RDMA_SetImageFormat(vioc->rdma.addr, vioc->rdma.fmt);
			if (vioc->rdma.y_stride) {
				if(vioc->rdma.fmt > VIOC_IMG_FMT_ARGB6666_3)
					VIOC_RDMA_SetImageOffset(vioc->rdma.addr, vioc->rdma.fmt, vioc->rdma.y_stride);
				else
					BITCSET(vioc->rdma.addr->uOFFSET.nREG, 0xFFFFFFFF, vioc->rdma.y_stride);
			} else {
				VIOC_RDMA_SetImageOffset(vioc->rdma.addr, vioc->rdma.fmt, vioc->rdma.width);
			}

			/*
			 * Onthefly VIQE
			 */
			vioc->deintl_s.plugin = vioc->deintl_viqe.plugin = VIOC_RDMA_03;
			if(vout->deinterlace == VOUT_DEINTL_VIQE_3D ||
				vout->deinterlace == VOUT_DEINTL_VIQE_2D )
				deintl_viqe_setup(vout, vout->deinterlace, 1);
			else
				deintl_s_setup(vout);
		}


		/*
		 * Onthefly Scaler
		 */
		if(res_change)
		{
			if (V4L2_FIELD_INTERLACED_TB == vout->src_pix.field
				|| V4L2_FIELD_INTERLACED_BT == vout->src_pix.field
				|| V4L2_FIELD_INTERLACED == vout->src_pix.field)
				vout->frame_count = 0;

			if(vioc->wmix.width == vioc->rdma.width && vioc->wmix.height == vioc->rdma.height) {
				dprintk("BYPASS 	src(%dx%d) -> dst(%dx%d) \n", vioc->rdma.width, vioc->rdma.height, vioc->wmix.width, vioc->wmix.height);
				VIOC_SC_SetBypass(vioc->deintl_sc.addr, ON);
				if(vioc->deintl_sc.plugin != VIOC_RDMA_03) {
					vioc->deintl_sc.plugin = VIOC_RDMA_03;
					VIOC_CONFIG_PlugIn(vioc->deintl_sc.ch, vioc->deintl_sc.plugin); // plugin position in scaler
				}
				VIOC_SC_SetUpdate(vioc->deintl_sc.addr);
			} else {
				dprintk("SCALING 	src(%dx%d) -> dst(%dx%d) \n", vioc->rdma.width, vioc->rdma.height, vioc->wmix.width, vioc->wmix.height);
				VIOC_SC_SetBypass(vioc->deintl_sc.addr, OFF);
				VIOC_SC_SetDstSize(vioc->deintl_sc.addr, vioc->wmix.width, vioc->wmix.height);	// set destination size in scaler
				VIOC_SC_SetOutSize(vioc->deintl_sc.addr, vioc->wmix.width, vioc->wmix.height);	// set output size in scaler
				if(vioc->deintl_sc.plugin != VIOC_RDMA_03) {
					vioc->deintl_sc.plugin = VIOC_RDMA_03;
					VIOC_CONFIG_PlugIn(vioc->deintl_sc.ch, vioc->deintl_sc.plugin); // plugin position in scaler
				}
				VIOC_SC_SetUpdate(vioc->deintl_sc.addr);
			}
		}
		else
		{
			if(vioc->deintl_sc.plugin != VIOC_RDMA_03) {
				vioc->deintl_sc.plugin = VIOC_RDMA_03;
				deintl_scaler_setup(vout);
			}
		}

		if (vout->previous_field != vout->src_pix.field)
		{
			vout->previous_field = vout->src_pix.field;
			if (V4L2_FIELD_INTERLACED_TB == vout->src_pix.field
				|| V4L2_FIELD_INTERLACED_BT == vout->src_pix.field
				|| V4L2_FIELD_INTERLACED == vout->src_pix.field) {
				switch (vout->deinterlace) {
				case VOUT_DEINTL_VIQE_3D:
				case VOUT_DEINTL_VIQE_2D:
					deintl_viqe_setup(vout, vout->deinterlace, 0);
					break;
				case VOUT_DEINTL_S:
					deintl_s_setup(vout);
					break;
				default:
					break;
				}
				if(vout->src_pix.colorspace == V4L2_COLORSPACE_JPEG)
					VIOC_RDMA_SetImageY2REnable(vioc->rdma.addr, 0);	// force disable y2r
				VIOC_RDMA_SetImageIntl(vioc->rdma.addr, 1);
				dprintk("enable deintl(%d)\n", vout->deinterlace);
			} else {
				switch (vout->deinterlace) {
				case VOUT_DEINTL_VIQE_3D:
				case VOUT_DEINTL_VIQE_2D:
					deintl_viqe_setup(vout, VOUT_DEINTL_VIQE_BYPASS, 0);
					vout->frame_count = 0;
					break;
				case VOUT_DEINTL_S:
					VIOC_CONFIG_PlugOut(vioc->deintl_s.ch);
					break;
				default:
					break;
				}
				if(vout->src_pix.colorspace == V4L2_COLORSPACE_JPEG)
					VIOC_RDMA_SetImageY2REnable(vioc->rdma.addr, 1); // force enable y2r
				VIOC_RDMA_SetImageIntl(vioc->rdma.addr, 0);
				dprintk("disable deintl(%d)\n", vout->deinterlace);
			}
		}
		VIOC_RDMA_SetImageBfield(vioc->rdma.addr, vioc->rdma.bf);
		VIOC_RDMA_SetImageBase(vioc->rdma.addr, base0, base1, base2);
		VIOC_RDMA_SetImageEnable(vioc->rdma.addr);
	}

	if (V4L2_FIELD_INTERLACED_TB == vout->src_pix.field
		|| V4L2_FIELD_INTERLACED_BT == vout->src_pix.field
		|| V4L2_FIELD_INTERLACED == vout->src_pix.field)
	{
		dtprintk("%s-field done\n", vioc->rdma.addr->uCTRL.bREG.BFIELD ? "bot" : "top");

		if ((vout->deinterlace == VOUT_DEINTL_VIQE_3D) && (vout->frame_count != -1)) {
			if (vout->frame_count >= 3) {
				VIOC_VIQE_SetDeintlMode(vioc->deintl_viqe.addr, VIOC_VIQE_DEINTL_MODE_3D);
				vout->frame_count = -1;
				dprintk("Change VIQE_3D mode\n");

				/* enhancement de-interlace */
				{
					int deintl_judder_cnt = (vioc->rdma.width + 64) / 64 - 1;
					BITCSET(vioc->deintl_viqe.addr->cDEINTL.nPD_JUDDER, 0x000000FF, deintl_judder_cnt);
					BITCSET(vioc->deintl_viqe.addr->cDEINTL.nPD_JUDDER_M, 0x00000001, 1);
					BITCSET(vioc->deintl_viqe.addr->cDEINTL.nPD_THRES0, 0x00f0000f, 0xf << 20);
				}
			} else {
				VIOC_VIQE_SetDeintlMode(vioc->deintl_viqe.addr, VIOC_VIQE_DEINTL_MODE_2D);
				vout->frame_count++;
			}
		}
		last_cleared_buffer = buf;

		if(vout->clearFrameMode) {
			vout->frame_count = 0;

			vout->firstFieldFlag = 0;
			vout_update_displayed_info(vout, last_cleared_buffer);

			vout->wakeup_int = 1;
			wake_up_interruptible(&vout->frame_wait);
			return;
		}
	}
	else
	{
		// update DD flag
		vout->display_done = ON;
		last_cleared_buffer = buf;

		if(vout->clearFrameMode) {
			vout->display_done = OFF;
			vout_update_displayed_info(vout, last_cleared_buffer);

			vout->wakeup_int = 1;
			wake_up_interruptible(&vout->frame_wait);
		}
	}
	return;
}

void vout_m2m_display_update(struct tcc_vout_device *vout, struct v4l2_buffer *buf)
{
	struct tcc_vout_vioc *vioc = vout->vioc;
	unsigned int base0=0, base1=0, base2=0;
	int index=0, res_change = OFF;

	#if defined(CONFIG_TCC_VIOCMG)
	unsigned int viqe_locked = 0;
	unsigned int force_process = 0;
	unsigned int restore_deintl_type = 0;
	enum deintl_type preview_deinterlace;
	#endif

	/* This is the code for only m2m path without vsync */
	if(!vout->deintl_force)
		vout->src_pix.field = buf->field;

	#ifndef CONFIG_VOUT_USE_VSYNC_INT
	if(vout->firstFieldFlag == 0) {		// first field
		if (V4L2_FIELD_INTERLACED_TB == vout->src_pix.field
			|| V4L2_FIELD_INTERLACED_BT == vout->src_pix.field
			|| V4L2_FIELD_INTERLACED == vout->src_pix.field)
			vout->firstFieldFlag++;
	}
	#endif

	if(2 == buf->m.planes[MPLANE_VID].reserved[VID_NUM]) {
		memcpy(&vioc->sub_alpha, buf->m.planes[MPLANE_SUB].reserved, sizeof(struct vioc_alpha));
		vioc->sub_rdma.img.base0 = buf->m.planes[MPLANE_SUB].m.mem_offset;
		vout_subplane_qbuf(vout, &vioc->sub_alpha);
	} else {
		if(vioc->sub_rdma.addr->uCTRL.nREG & Hw28) {
			dprintk("subtitle layer OFF!!! \n");
			VIOC_RDMA_SetImageDisableNW(vioc->sub_rdma.addr);
		}
	}

	/* get base address */
	if (V4L2_MEMORY_USERPTR == vout->memory) {
		if (STILL_IMGAGE != buf->timecode.type) {
			#ifdef VOUT_SUB_PLANE
			base0 = buf->m.planes[MPLANE_VID].m.userptr;
			base1 = buf->m.planes[MPLANE_VID].reserved[VID_BASE1];
			base2 = buf->m.planes[MPLANE_VID].reserved[VID_BASE2];
			#else
			base0 = buf->m.offset;
			base1 = buf->input;
			base2 = buf->reserved;
			#endif
		} else {
			#ifdef VOUT_SUB_PLANE
			base0 = buf->m.planes[MPLANE_VID].m.userptr;
			#else
			base0 = buf->m.offset;
			#endif
			base1 = base2 = 0;

			/*
			 * If the input is YUV format.
			 */
			if (TCC_PFMT_RGB != vout->pfmt) {
				/*
				 * Re-store src_pix.height from "The VIQE need 4-line align"
				 */
				if (vioc->deintl_rdma.height != vout->src_pix.height) {
					vioc->deintl_rdma.height = vout->src_pix.height;
					deintl_rdma_setup(&vioc->deintl_rdma);
				}

				tcc_get_base_address_of_image(
					vout->src_pix.pixelformat, base0,
					vout->src_pix.width, vout->src_pix.height,
					&base0, &base1, &base2);
			}
		}
	} else if (V4L2_MEMORY_MMAP == vout->memory) {
		base0 = vout->qbufs[buf->index].img_base0;
		base1 = vout->qbufs[buf->index].img_base1;
		base2 = vout->qbufs[buf->index].img_base2;
	} else {
		printk(KERN_ERR VOUT_NAME ": [error] invalid qbuf v4l2_memory\n");
	}

	if (V4L2_FIELD_INTERLACED_BT == vout->src_pix.field)
		vioc->deintl_rdma.bf = 1;
	else
		vioc->deintl_rdma.bf = 0;

	/* To prevent frequent switching of the scan type */
	if (vout->first_frame) {
		vout->first_field = vout->src_pix.field;
		vout->first_frame = 0;
		dprintk("first_field(%s)\n", v4l2_field_names[vout->first_field]);
	} else {
		vout->src_pix.field = vout->first_field;
	}

	index = (vout->deintl_nr_bufs_count++ % vout->deintl_nr_bufs);
	if (vout->deintl_nr_bufs_count == vout->deintl_nr_bufs)
		vout->deintl_nr_bufs_count = 0;

	/*
	 *	Change source info
	 */
	if((vout->src_pix.width != buf->m.planes[MPLANE_VID].reserved[VID_WIDTH]
		|| vout->src_pix.height != buf->m.planes[MPLANE_VID].reserved[VID_HEIGHT])
		&& (buf->m.planes[MPLANE_VID].reserved[VID_WIDTH] && buf->m.planes[MPLANE_VID].reserved[VID_HEIGHT]))
	{
		dprintk("changing source (%dx%d)->(%dx%d)\n",
			vout->src_pix.width, vout->src_pix.height,
			buf->m.planes[MPLANE_VID].reserved[VID_WIDTH], buf->m.planes[MPLANE_VID].reserved[VID_HEIGHT]);

		vout->src_pix.width = buf->m.planes[MPLANE_VID].reserved[VID_WIDTH];
		vout->src_pix.height = buf->m.planes[MPLANE_VID].reserved[VID_HEIGHT];
		vout->src_pix.bytesperline = vout->src_pix.width * tcc_vout_try_bpp(vout->src_pix.pixelformat, &vout->src_pix.colorspace);

		/* reinit crop_src */
		vout->crop_src.left = 0;
		vout->crop_src.top = 0;
		vout->crop_src.width = vout->src_pix.width;
		vout->crop_src.height = vout->src_pix.height;

		switch(tcc_vout_try_pix(vout->src_pix.pixelformat)){
		case TCC_PFMT_YUV422:
			vout->src_pix.sizeimage = vout->src_pix.width * vout->src_pix.height * 2;
			break;
		case TCC_PFMT_YUV420:
			vout->src_pix.sizeimage = vout->src_pix.width * vout->src_pix.height * 3 / 2;
			break;
		case TCC_PFMT_RGB:
		default:
			vout->src_pix.sizeimage = vout->src_pix.bytesperline * vout->src_pix.height;
			break;
		}
		vout->src_pix.sizeimage = PAGE_ALIGN(vout->src_pix.sizeimage);

		vioc->deintl_rdma.width = vout->src_pix.width;
		if(vout->src_pix.height % 4) {
			vioc->deintl_rdma.height = ROUND_UP_4(vout->src_pix.height);
			dprintk(" 4-line align: %d -> %d \n", vout->src_pix.height, vioc->deintl_rdma.height);
		} else {
			vioc->deintl_rdma.height = vout->src_pix.height;
		}
		vioc->deintl_rdma.y_stride = buf->m.planes[MPLANE_VID].bytesused ? buf->m.planes[MPLANE_VID].bytesused : vout->src_pix.bytesperline && 0x0000FFFF;
		VIOC_RDMA_SetImageSize(vioc->deintl_rdma.addr, vioc->deintl_rdma.width, vioc->deintl_rdma.height);
		VIOC_RDMA_SetImageOffset(vioc->deintl_rdma.addr, vioc->deintl_rdma.fmt, vioc->deintl_rdma.y_stride);

		vioc->deintl_wmix.width = vioc->deintl_rdma.width;
		vioc->deintl_wmix.height = vioc->deintl_rdma.height;
		VIOC_WMIX_SetSize(vioc->deintl_wmix.addr, vioc->deintl_wmix.width, vioc->deintl_wmix.height);

		res_change = ON;
	}

	/*
	 *	Change crop info
	 */
	if ((vout->crop_src.left != buf->m.planes[MPLANE_VID].reserved[VID_CROP_LEFT]
		|| vout->crop_src.top != buf->m.planes[MPLANE_VID].reserved[VID_CROP_TOP]
		|| vout->crop_src.width != buf->m.planes[MPLANE_VID].reserved[VID_CROP_WIDTH]
		|| vout->crop_src.height != buf->m.planes[MPLANE_VID].reserved[VID_CROP_HEIGHT])
		&& (buf->m.planes[MPLANE_VID].reserved[VID_CROP_WIDTH] && buf->m.planes[MPLANE_VID].reserved[VID_CROP_HEIGHT])
		&& (vout->src_pix.width >= buf->m.planes[MPLANE_VID].reserved[VID_CROP_WIDTH] && vout->src_pix.height >= buf->m.planes[MPLANE_VID].reserved[VID_CROP_HEIGHT]))
	{
		dprintk("changing crop (%d,%d)(%dx%d)->(%d,%d)(%dx%d)\n",
			vout->crop_src.left, vout->crop_src.top, vout->crop_src.width, vout->crop_src.height,
			buf->m.planes[MPLANE_VID].reserved[VID_CROP_LEFT], buf->m.planes[MPLANE_VID].reserved[VID_CROP_TOP],
			buf->m.planes[MPLANE_VID].reserved[VID_CROP_WIDTH], buf->m.planes[MPLANE_VID].reserved[VID_CROP_HEIGHT]);

		vout->crop_src.left = buf->m.planes[MPLANE_VID].reserved[VID_CROP_LEFT];
		vout->crop_src.top = buf->m.planes[MPLANE_VID].reserved[VID_CROP_TOP];
		vout->crop_src.width = buf->m.planes[MPLANE_VID].reserved[VID_CROP_WIDTH];
		vout->crop_src.height = buf->m.planes[MPLANE_VID].reserved[VID_CROP_HEIGHT];

		tcc_get_base_address(vioc->deintl_rdma.fmt, base0,
			vioc->deintl_rdma.y_stride ? vioc->deintl_rdma.y_stride : vioc->deintl_rdma.width,
			vioc->deintl_rdma.height,
			vout->crop_src.left, vout->crop_src.top,
			&base0, &base1, &base2);

		vioc->deintl_rdma.width = vout->crop_src.width;
		if(vout->crop_src.height % 4) {
			vioc->deintl_rdma.height = ROUND_UP_4(vout->crop_src.height);
			dprintk(" 4-line align: %d -> %d \n", vout->crop_src.height, vioc->deintl_rdma.height);
		} else {
			vioc->deintl_rdma.height = vout->crop_src.height;
		}
		VIOC_RDMA_SetImageSize(vioc->deintl_rdma.addr, vioc->deintl_rdma.width, vioc->deintl_rdma.height);

		vioc->deintl_wmix.width = vioc->deintl_rdma.width;
		vioc->deintl_wmix.height = vioc->deintl_rdma.height;
		VIOC_WMIX_SetSize(vioc->deintl_wmix.addr, vioc->deintl_wmix.width, vioc->deintl_wmix.height);

		res_change = ON;
	}

	if(res_change)
	{
		if (V4L2_FIELD_INTERLACED_TB == vout->src_pix.field
			|| V4L2_FIELD_INTERLACED_BT == vout->src_pix.field
			|| V4L2_FIELD_INTERLACED == vout->src_pix.field)
		{
			vout->frame_count = 0;
		}

		VIOC_WMIX_SetUpdate(vioc->deintl_wmix.addr);

		if(vioc->deintl_wmix.width == vioc->deintl_wdma.width && vioc->deintl_wmix.height == vioc->deintl_wdma.height)
		{
			dprintk("BYPASS 	src(%dx%d) -> dst(%dx%d) \n", vioc->deintl_wmix.width, vioc->deintl_wmix.height, vioc->deintl_wdma.width, vioc->deintl_wdma.height);
			VIOC_SC_SetBypass(vioc->deintl_sc.addr, ON);
			VIOC_SC_SetUpdate(vioc->deintl_sc.addr);
		} else {
			dprintk("SCALING	src(%dx%d) -> dst(%dx%d) \n", vioc->deintl_wmix.width, vioc->deintl_wmix.height, vioc->deintl_wdma.width, vioc->deintl_wdma.height);
			VIOC_SC_SetBypass(vioc->deintl_sc.addr, OFF);
			VIOC_SC_SetDstSize(vioc->deintl_sc.addr, vioc->deintl_wdma.width, vioc->deintl_wdma.height);	// set destination size in scaler
			VIOC_SC_SetOutSize(vioc->deintl_sc.addr, vioc->deintl_wdma.width, vioc->deintl_wdma.height);	// set output size in scaler
			VIOC_SC_SetUpdate(vioc->deintl_sc.addr);
		}
	}

	/* rdma stride */
	if(buf->m.planes[MPLANE_VID].bytesused && (vioc->deintl_rdma.y_stride != buf->m.planes[MPLANE_VID].bytesused)) {
		dprintk("update rdma stride(%d -> %d)\n", vioc->deintl_rdma.y_stride, buf->m.planes[MPLANE_VID].bytesused);

		/*  change rdma stride */
		vioc->deintl_rdma.y_stride = vout->src_pix.bytesperline = buf->m.planes[MPLANE_VID].bytesused;
		VIOC_RDMA_SetImageOffset(vioc->deintl_rdma.addr, vioc->deintl_rdma.fmt, vioc->deintl_rdma.y_stride);
	}

	/* rdma base address */
	vioc->deintl_rdma.img.base0 = base0;
	vioc->deintl_rdma.img.base1 = base1;
	vioc->deintl_rdma.img.base2 = base2;
	/* wdma base address */
	vioc->deintl_wdma.img.base0 = vout->deintl_bufs[index].img_base0;
	vioc->deintl_wdma.img.base1 = vout->deintl_bufs[index].img_base1;
	vioc->deintl_wdma.img.base2 = vout->deintl_bufs[index].img_base2;

	#ifdef CONFIG_ARCH_TCC_MAP_CONVERTER
	if(buf->m.planes[MPLANE_VID].reserved[HEVC_COMP_MAP_EN]) {
		int enable;
		/*
		 * plugout VIQE (temporary)
		 */
		VIOC_RDMA_GetImageEnable(vioc->deintl_rdma.addr, &enable);
		if(enable) {
			/* rdma disable */
			VIOC_RDMA_SetImageDisable(vioc->deintl_rdma.addr);

			/* plug-out viqe */
			VIOC_CONFIG_PlugOut(vioc->deintl_viqe.ch);
			VIOC_CONFIG_SWReset(vioc->config, VIOC_CONFIG_VIQE, vioc->deintl_viqe.id, VIOC_CONFIG_RESET);
			VIOC_CONFIG_SWReset(vioc->config, VIOC_CONFIG_VIQE, vioc->deintl_viqe.id, VIOC_CONFIG_CLEAR);
		}

		/* update map_converter info */
		vioc->deintl_mc.mapConv_info.m_uiLumaStride = buf->m.planes[MPLANE_VID].reserved[HEVC_LUMA_STRIDE];
		vioc->deintl_mc.mapConv_info.m_uiChromaStride = buf->m.planes[MPLANE_VID].reserved[HEVC_CHROMA_STRIDE];
		vioc->deintl_mc.mapConv_info.m_uiLumaBitDepth = buf->m.planes[MPLANE_VID].reserved[HEVC_LUMA_BIT_DEPTH];
		vioc->deintl_mc.mapConv_info.m_uiChromaBitDepth = buf->m.planes[MPLANE_VID].reserved[HEVC_CHROMA_BIT_DEPTH];
		vioc->deintl_mc.mapConv_info.m_uiFrameEndian = buf->m.planes[MPLANE_VID].reserved[HEVC_FRMAE_ENDIAN];

		vioc->deintl_mc.mapConv_info.m_CompressedY[0] = buf->m.planes[MPLANE_VID].reserved[HEVC_FRAME_BASE_Y0];
		vioc->deintl_mc.mapConv_info.m_CompressedY[1] = buf->m.planes[MPLANE_VID].reserved[HEVC_FRAME_BASE_Y1];
		vioc->deintl_mc.mapConv_info.m_CompressedCb[0] = buf->m.planes[MPLANE_VID].reserved[HEVC_FRAME_BASE_C0];
		vioc->deintl_mc.mapConv_info.m_CompressedCb[1] = buf->m.planes[MPLANE_VID].reserved[HEVC_FRAME_BASE_C1];

		vioc->deintl_mc.mapConv_info.m_FbcYOffsetAddr[0] = buf->m.planes[MPLANE_VID].reserved[HEVC_OFFSET_BASE_Y0];
		vioc->deintl_mc.mapConv_info.m_FbcYOffsetAddr[1] = buf->m.planes[MPLANE_VID].reserved[HEVC_OFFSET_BASE_Y1];
		vioc->deintl_mc.mapConv_info.m_FbcCOffsetAddr[0] = buf->m.planes[MPLANE_VID].reserved[HEVC_OFFSET_BASE_C0];
		vioc->deintl_mc.mapConv_info.m_FbcCOffsetAddr[1] = buf->m.planes[MPLANE_VID].reserved[HEVC_OFFSET_BASE_C1];

		//dprintk(" m_uiLumaStride %d m_uiChromaStride %d\n m_uiLumaBitDepth %d m_uiChromaBitDepth %d m_uiFrameEndian %d \n m_CompressedY[0] 0x%08x m_CompressedCb[0] 0x%08x m_CompressedY[1] 0x%08x m_CompressedCb[1] 0x%08x\n m_FbcYOffsetAddr[0] 0x%08x m_FbcCOffsetAddr[0] 0x%08x m_FbcYOffsetAddr[1] 0x%08x m_FbcCOffsetAddr[1] 0x%08x \n",
		//	vioc->deintl_mc.mapConv_info.m_uiLumaStride, vioc->deintl_mc.mapConv_info.m_uiChromaStride, 
		//	vioc->deintl_mc.mapConv_info.m_uiLumaBitDepth, vioc->deintl_mc.mapConv_info.m_uiChromaBitDepth, vioc->deintl_mc.mapConv_info.m_uiFrameEndian,
		//	vioc->deintl_mc.mapConv_info.m_CompressedY[0], vioc->deintl_mc.mapConv_info.m_CompressedCb[0],
		//	vioc->deintl_mc.mapConv_info.m_CompressedY[1], vioc->deintl_mc.mapConv_info.m_CompressedCb[1],
		//	vioc->deintl_mc.mapConv_info.m_FbcYOffsetAddr[0], vioc->deintl_mc.mapConv_info.m_FbcCOffsetAddr[0],
		//	vioc->deintl_mc.mapConv_info.m_FbcYOffsetAddr[1], vioc->deintl_mc.mapConv_info.m_FbcCOffsetAddr[1]);

		//dprintk("%s map converter: size: %dx%d pos: (0,0) \n", __func__, vout->src_pix.width, vout->src_pix.height);

		/* map converter */
		if(mc_plugged == 0) {
			mc_plugged = 1;
			dprintk("map converter connect to RDMA%d\n", VIOC_RDMA_13);
			tca_set_MC_Connect_to_RDMA(VIOC_MC_01, VIOC_RDMA_13, 1);
		}
		tca_map_convter_driver_set(VIOC_MC_01, vout->src_pix.width, vout->src_pix.height, 0, 0, OFF, &vioc->deintl_mc.mapConv_info);
		tca_map_convter_onoff(VIOC_MC_01, 1);

		if(vioc->deintl_sc.addr->uSRCSIZE.bREG.WIDTH != vout->src_pix.width ||
			vioc->deintl_sc.addr->uSRCSIZE.bREG.HEIGHT != vout->src_pix.height) {
			dprintk("change source : %dx%d -> %dx%d\n", vioc->deintl_sc.addr->uSRCSIZE.bREG.WIDTH, vioc->deintl_sc.addr->uSRCSIZE.bREG.HEIGHT
				,vout->src_pix.width, vout->src_pix.height);
			deintl_scaler_setup(vout);
		}

		/* update wdma base address */
		VIOC_WDMA_SetImageR2YEnable(vioc->deintl_wdma.addr, OFF);
		VIOC_WDMA_SetImageBase(vioc->deintl_wdma.addr,
			vioc->deintl_wdma.img.base0, vioc->deintl_wdma.img.base1, vioc->deintl_wdma.img.base2);
		vout_deintl_ctrl(vioc, 1);
	}
	else
	#endif
	{
		#if defined(CONFIG_TCC_VIOCMG)
		if(vout->deinterlace == VOUT_DEINTL_VIQE_3D || vout->deinterlace == VOUT_DEINTL_VIQE_2D)
		{
			if(viocmg_lock_viqe(VIOCMG_CALLERID_VOUT) < 0) {
				// VIQE may be used for rear camera !!
				viqe_locked = 0;

				if (V4L2_FIELD_INTERLACED_TB == vout->src_pix.field
					|| V4L2_FIELD_INTERLACED_BT == vout->src_pix.field
					|| V4L2_FIELD_INTERLACED == vout->src_pix.field)
				{
					// backup previous deinterlaced mode..!!
					preview_deinterlace = vout->deinterlace;
					vout->deinterlace = VOUT_DEINTL_S;

					// backup previous wdma r2y and disable wdma r2y
					VIOC_WDMA_SetImageR2YEnable(vioc->deintl_wdma.addr, 0);

					restore_deintl_type = 1;

					// fill information..!
					if(!vout->viqe_to_deints) {
						vout->viqe_to_deints = 1;
						force_process = 1;
						dprintk(" force_process = 1\r\n");
					}
				}
			} else {
				VIOC_PlugInOutCheck plugin_state;

				if(vout->viqe_to_deints) {
					vout->viqe_to_deints = 0;
					force_process = 1;
					dprintk(" force disconnect deints\r\n");

					/* reset deintl_s */
					BITSET(vioc->config->uSOFTRESET.nREG[1], vioc->deintl_s.swrst_bit);
					BITCLR(vioc->config->uSOFTRESET.nREG[1], vioc->deintl_s.swrst_bit);

					VIOC_CONFIG_PlugOut(VIOC_DEINTLS);
				}

				VIOC_CONFIG_Device_PlugState(vioc->deintl_viqe.ch, &plugin_state);
				if(plugin_state.enable && plugin_state.connect_device != vioc->deintl_viqe.plugin) {
					dprintk(" force disconnect viqe\r\n");
					VIOC_CONFIG_PlugOut(vioc->deintl_viqe.ch);
				}

				VIOC_CONFIG_Device_PlugState(vioc->deintl_viqe.ch, &plugin_state);
				if(!plugin_state.enable || plugin_state.connect_statue != VIOC_PATH_CONNECTED) {
					dprintk(" need reconnect viqe\r\n");
					VIOC_CONFIG_PlugIn(vioc->deintl_viqe.ch, vioc->deintl_viqe.plugin);
				}
				viqe_locked = 1;
			}
		}
		#endif

		/* change deinterlace
		 *	case1: VOUT_DEINTL_VIQE_3D/2D <-> VOUT_DEINTL_VIQE_BYPASS
		 *	case2: VOUT_DEINTL_S <-> VOUT_DEINTL_NONE
		 */
		#if defined(CONFIG_TCC_VIOCMG)
		if (res_change || force_process || vout->previous_field != vout->src_pix.field)
		#else
		if (res_change || vout->previous_field != vout->src_pix.field)
		#endif
		{
			vout->previous_field = vout->src_pix.field;
			if (V4L2_FIELD_INTERLACED_TB == vout->src_pix.field
				|| V4L2_FIELD_INTERLACED_BT == vout->src_pix.field
				|| V4L2_FIELD_INTERLACED == vout->src_pix.field) {
				switch (vout->deinterlace) {
				case VOUT_DEINTL_VIQE_3D:
				case VOUT_DEINTL_VIQE_2D:
					deintl_viqe_setup(vout, vout->deinterlace, 0);
					break;
				case VOUT_DEINTL_S:
					deintl_s_setup(vout);
					break;
				default:
					break;
				}
				if(vout->src_pix.colorspace == V4L2_COLORSPACE_JPEG)
					VIOC_RDMA_SetImageY2REnable(vioc->deintl_rdma.addr, 0);	// force disable y2r
				VIOC_RDMA_SetImageIntl(vioc->deintl_rdma.addr, 1);
				dprintk("enable deintl(%d)\n", vout->deinterlace);
			} else {
				switch (vout->deinterlace) {
				case VOUT_DEINTL_VIQE_3D:
				case VOUT_DEINTL_VIQE_2D:
					#if defined(CONFIG_TCC_VIOCMG)
					if(viqe_locked)
					#endif
					{
						deintl_viqe_setup(vout, VOUT_DEINTL_VIQE_BYPASS, 0);
						vout->frame_count = 0;
					}
					break;
				case VOUT_DEINTL_S:
					VIOC_CONFIG_PlugOut(vioc->deintl_s.ch);
					break;
				default:
					break;
				}
				if(vout->src_pix.colorspace == V4L2_COLORSPACE_JPEG)
					VIOC_RDMA_SetImageY2REnable(vioc->deintl_rdma.addr, 1); // force enable y2r
				VIOC_RDMA_SetImageIntl(vioc->deintl_rdma.addr, 0);
				dprintk("disable deintl(%d)\n", vout->deinterlace);
			}
		}

		/* update rdma (bfield and src addr.).
		 */
		VIOC_RDMA_SetImageBfield(vioc->deintl_rdma.addr, vioc->deintl_rdma.bf);
		VIOC_RDMA_SetImageBase(vioc->deintl_rdma.addr,
			vioc->deintl_rdma.img.base0, vioc->deintl_rdma.img.base1, vioc->deintl_rdma.img.base2);
		VIOC_RDMA_SetImageEnable(vioc->deintl_rdma.addr);

		/* update wdma (dst addr.)
		 * the wdma was enabled only by vout_deintl_ctrl
		 */
		VIOC_WDMA_SetImageBase(vioc->deintl_wdma.addr,
			vioc->deintl_wdma.img.base0, vioc->deintl_wdma.img.base1, vioc->deintl_wdma.img.base2);
		vout_deintl_ctrl(vioc, 1);
	}

	#ifndef CONFIG_VOUT_USE_VSYNC_INT	
	if (V4L2_FIELD_INTERLACED_TB == vout->src_pix.field
				|| V4L2_FIELD_INTERLACED_BT == vout->src_pix.field
				|| V4L2_FIELD_INTERLACED == vout->src_pix.field)
	{
next_field:
		if (wait_event_interruptible_timeout(vout->frame_wait, vout->wakeup_int == 1, msecs_to_jiffies(200)) <= 0) {
			printk(KERN_ERR VOUT_NAME ": [error][interlace] handler timeout\n");
			return;
		}
		vout->wakeup_int = 0;

		if(vout->firstFieldFlag) {
			int index = (vout->deintl_nr_bufs_count++ % vout->deintl_nr_bufs);
			if (vout->deintl_nr_bufs_count == vout->deintl_nr_bufs)
				vout->deintl_nr_bufs_count = 0;

			/* wdma base address */
			vioc->deintl_wdma.img.base0 = vout->deintl_bufs[index].img_base0;
			vioc->deintl_wdma.img.base1 = vout->deintl_bufs[index].img_base1;
			vioc->deintl_wdma.img.base2 = vout->deintl_bufs[index].img_base2;
			VIOC_WDMA_SetImageBase(vioc->deintl_wdma.addr, vioc->deintl_wdma.img.base0, vioc->deintl_wdma.img.base1, vioc->deintl_wdma.img.base2);

			/* change rdma bfield */
			VIOC_RDMA_SetImageBfield(vioc->deintl_rdma.addr, !(vioc->deintl_rdma.addr->uCTRL.bREG.BFIELD));
			VIOC_RDMA_SetImageUpdate(vioc->deintl_rdma.addr);

			vout->firstFieldFlag = 0;

			/* enable wdma */
			vout_deintl_ctrl(vioc, 1);
			goto next_field;
		}
	}
	else
	{
		if (wait_event_interruptible_timeout(vout->frame_wait, vout->wakeup_int == 1, msecs_to_jiffies(200)) <= 0) {
			printk(KERN_ERR VOUT_NAME ": [error][progressive] handler timeout\n");
			return;
		}
		vout->wakeup_int = 0;
	}

END_PROCESS:
	#if defined(CONFIG_TCC_VIOCMG)
	if(restore_deintl_type) {
		// restore interlace
		vout->deinterlace = preview_deinterlace;

		// restore wdma y2r
		VIOC_WDMA_SetImageR2YEnable(vioc->deintl_wdma.addr, vioc->deintl_wdma.r2y);

		// clean frame count..for viqe 3d mode..!!
		vout->frame_count = 0;

		// fill information..!
		//vout_set_deintl_path(0, vout);
	} else if(viqe_locked) {
		viocmg_free_viqe(VIOCMG_CALLERID_VOUT);
	}
	#endif
	#endif

	return;
}

#ifdef CONFIG_VOUT_USE_VSYNC_INT
static void display_update(struct tcc_vout_device *vout)
{
	struct tcc_vout_vioc *vioc = vout->vioc;
	int ret = 0;

search_nextfrm:
	if(vout->onthefly_mode) {
		if(vout->display_done) {
			vout_update_displayed_info(vout, last_cleared_buffer);

			// update DD flag
			vout->display_done = OFF;
		}
	}

	if(readable_bufs || vout->firstFieldFlag) {
		struct v4l2_buffer *buf = &displaybuffer[popIdx];
		if(vout->firstFieldFlag == 0)		// first field
		{
			if(buf->reserved2 || vout->force_sync) {
				if(vout->force_sync)
					vout->force_sync = OFF;
				tcc_vout_set_time(vout, ((buf->timestamp.tv_sec*1000)+(buf->timestamp.tv_usec/1000)));
			}

			/* check video time */
			ret = vout_check_syncTime(vout, buf, tcc_vout_get_time(vout));

			if(ret == VOUT_DRV_ERR_DROPFRM) {
				if((readable_bufs-1) == 0)
					goto force_disp;

				if(++popIdx >= MAX_DISPBUF_NUM)
					popIdx = 0;
				readable_bufs--;

				displayed_bufs++;
				goto search_nextfrm;
			} else if(ret == VOUT_DRV_ERR_WAITFRM) {
				return;
			}
force_disp:
			dbprintk("d(%d %d)(%d %d)\n", popIdx, readable_bufs, buf->index, buf->reserved2);

			if(!vout->deintl_force)
				vout->src_pix.field = buf->field;

			if (V4L2_FIELD_INTERLACED_TB == vout->src_pix.field
				|| V4L2_FIELD_INTERLACED_BT == vout->src_pix.field
				|| V4L2_FIELD_INTERLACED == vout->src_pix.field)
				vout->firstFieldFlag++;

			if((vout->disp_rect.width > 1920 && vout->disp_rect.height > 1080)
				|| (buf->m.planes[MPLANE_VID].reserved[HEVC_COMP_MAP_EN])) {
				vout->onthefly_mode = ON;
				vout_onthefly_display_update(vout, buf);
			} else {
				vout->onthefly_mode = OFF;
				vout_m2m_display_update(vout, buf);
			}
			if(++popIdx >= MAX_DISPBUF_NUM)
				popIdx = 0;
			readable_bufs--;
		}
		else		// next field
		{
			vout->firstFieldFlag = 0;

			if(readable_bufs) {
				struct v4l2_buffer *next_buf = &displaybuffer[popIdx];
				if(vout_check_syncTime(vout, next_buf, tcc_vout_get_time(vout)) == VOUT_DRV_NOERR) {
					vout->frame_count = 0;	// to change viqe de-interlace mode

					if(vout->onthefly_mode)
						vout_update_displayed_info(vout, last_cleared_buffer);	// to clear current buffer

					goto force_disp;
				}
			}

			if(vout->onthefly_mode) {
				if (V4L2_FIELD_INTERLACED_TB == vout->src_pix.field
					|| V4L2_FIELD_INTERLACED_BT == vout->src_pix.field
					|| V4L2_FIELD_INTERLACED == vout->src_pix.field)
				{
					/* change rdma bfield */
					VIOC_RDMA_SetImageBfield(vioc->rdma.addr, !(vioc->rdma.addr->uCTRL.bREG.BFIELD));
					VIOC_RDMA_SetImageUpdate(vioc->rdma.addr);

					dtprintk("%s-field done\n", vioc->deintl_rdma.addr->uCTRL.bREG.BFIELD ? "bot" : "top");

					if ((vout->deinterlace == VOUT_DEINTL_VIQE_3D) && (vout->frame_count != -1)) {
						if (vout->frame_count >= 3) {
							VIOC_VIQE_SetDeintlMode(vioc->deintl_viqe.addr, VIOC_VIQE_DEINTL_MODE_3D);
							vout->frame_count = -1;
							dprintk("Change VIQE_3D mode\n");

							/* enhancement de-interlace */
							{
								int deintl_judder_cnt = (vioc->rdma.width + 64) / 64 - 1;
								BITCSET(vioc->deintl_viqe.addr->cDEINTL.nPD_JUDDER, 0x000000FF, deintl_judder_cnt);
								BITCSET(vioc->deintl_viqe.addr->cDEINTL.nPD_JUDDER_M, 0x00000001, 1);
								BITCSET(vioc->deintl_viqe.addr->cDEINTL.nPD_THRES0, 0x00f0000f, 0xf << 20);
							}
						} else {
							VIOC_VIQE_SetDeintlMode(vioc->deintl_viqe.addr, VIOC_VIQE_DEINTL_MODE_2D);
							vout->frame_count++;
						}
					}

					if(vout->firstFieldFlag == 0)
						vout->display_done = ON;

					if(vout->clearFrameMode) {
						vout->frame_count = 0;

						vout_update_displayed_info(vout, last_cleared_buffer);	// to clear current buffer
						vout->display_done = OFF;

						vout->wakeup_int = 1;
						wake_up_interruptible(&vout->frame_wait);
					}
				}
			} else {
				int index = (vout->deintl_nr_bufs_count++ % vout->deintl_nr_bufs);
				if (vout->deintl_nr_bufs_count == vout->deintl_nr_bufs)
					vout->deintl_nr_bufs_count = 0;

				/* wdma base address */
				vioc->deintl_wdma.img.base0 = vout->deintl_bufs[index].img_base0;
				vioc->deintl_wdma.img.base1 = vout->deintl_bufs[index].img_base1;
				vioc->deintl_wdma.img.base2 = vout->deintl_bufs[index].img_base2;

				VIOC_WDMA_SetImageBase(vioc->deintl_wdma.addr, vioc->deintl_wdma.img.base0, vioc->deintl_wdma.img.base1, vioc->deintl_wdma.img.base2);

				/* change rdma bfield */
				VIOC_RDMA_SetImageBfield(vioc->deintl_rdma.addr, !(vioc->deintl_rdma.addr->uCTRL.bREG.BFIELD));
				VIOC_RDMA_SetImageUpdate(vioc->deintl_rdma.addr);

				/* enable wdma */
				vout_deintl_ctrl(vioc, 1);
			}
		}
	}
	return;
}

static irqreturn_t vsync_irq_handler(int irq, void *client_data)
{
	struct tcc_vout_device *vout = (struct tcc_vout_device *)client_data;
	struct tcc_vout_vioc *vioc = vout->vioc;

	if (!is_vioc_intr_activatied(vioc->disp.vioc_intr->id, vioc->disp.vioc_intr->bits))
		return IRQ_NONE;

	vioc_intr_clear(vioc->disp.vioc_intr->id, vioc->disp.vioc_intr->bits);

	if (vout->status == TCC_VOUT_RUNNING)
		display_update(vout);	// update video frame

	return IRQ_HANDLED;
}
#endif

static irqreturn_t wdma_irq_handler(int irq, void *client_data)
{
	struct tcc_vout_device *vout = (struct tcc_vout_device *)client_data;
	struct tcc_vout_vioc *vioc = vout->vioc;

	if (!is_vioc_intr_activatied(vioc->deintl_wdma.vioc_intr->id, vioc->deintl_wdma.vioc_intr->bits))
		return IRQ_NONE;

	if (vioc->deintl_wdma.addr->uIRQSTS.nREG & VIOC_WDMA_IREQ_EOFR_MASK) {
		/* clear wdma interrupt status */
		vioc->deintl_wdma.addr->uIRQSTS.nREG = 0xFFFFFFFF;   // wdma status register all clear.

		if(VIOC_DISP_Get_TurnOnOff(vioc->disp.addr)) {
			VIOC_RDMA_SetImageBase(vioc->rdma.addr, vioc->deintl_wdma.addr->uBASE0, vioc->deintl_wdma.addr->uBASE1, vioc->deintl_wdma.addr->uBASE2);
			VIOC_RDMA_SetImageEnable(vioc->rdma.addr);
		}

		if (V4L2_FIELD_INTERLACED_TB == vout->src_pix.field
			|| V4L2_FIELD_INTERLACED_BT == vout->src_pix.field
			|| V4L2_FIELD_INTERLACED == vout->src_pix.field)
		{
			dtprintk("%s-field done\n", vioc->deintl_rdma.addr->uCTRL.bREG.BFIELD ? "bot" : "top");

			if ((vout->deinterlace == VOUT_DEINTL_VIQE_3D) && (vout->frame_count != -1)) {
				if (vout->frame_count >= 3) {
					VIOC_VIQE_SetDeintlMode(vioc->deintl_viqe.addr, VIOC_VIQE_DEINTL_MODE_3D);
					vout->frame_count = -1;
					dprintk("Change VIQE_3D mode\n");

					/* enhancement de-interlace */
					{
						int deintl_judder_cnt = (vioc->deintl_rdma.width + 64) / 64 - 1;
						BITCSET(vioc->deintl_viqe.addr->cDEINTL.nPD_JUDDER, 0x000000FF, deintl_judder_cnt);
						BITCSET(vioc->deintl_viqe.addr->cDEINTL.nPD_JUDDER_M, 0x00000001, 1);
						BITCSET(vioc->deintl_viqe.addr->cDEINTL.nPD_THRES0, 0x00f0000f, 0xf << 20);
					}
				} else {
					VIOC_VIQE_SetDeintlMode(vioc->deintl_viqe.addr, VIOC_VIQE_DEINTL_MODE_2D);
					vout->frame_count++;
				}
			}

			#ifdef CONFIG_VOUT_USE_VSYNC_INT
			if(vout->clearFrameMode) {
				vout->firstFieldFlag = 0;	// reinit first field flag
				displayed_bufs++;			// for dequeue

				vout->wakeup_int = 1;
				wake_up_interruptible(&vout->frame_wait);
				goto end_handler;
			}
			#else
			vout->wakeup_int = 1;
			wake_up_interruptible(&vout->frame_wait);
			#endif

			if(vout->firstFieldFlag == 0)
				displayed_bufs++;
		} else {
			displayed_bufs++;
			#ifdef CONFIG_VOUT_USE_VSYNC_INT
			if(vout->clearFrameMode) {
				vout->wakeup_int = 1;
				wake_up_interruptible(&vout->frame_wait);
			}
			#else
			vout->wakeup_int = 1;
			wake_up_interruptible(&vout->frame_wait);
			#endif
		}
	}
end_handler:
	return IRQ_HANDLED;
}
void vout_deintl_ctrl(struct tcc_vout_vioc *vioc, int enable)
{
	if (enable)
		VIOC_WDMA_SetImageEnable(vioc->deintl_wdma.addr, vioc->deintl_wdma.cont);
	else
		VIOC_WDMA_SetImageDisable(vioc->deintl_wdma.addr);
}

void vout_deintl_overlay(struct tcc_vout_device *vout)
{
	struct tcc_vout_vioc *vioc = vout->vioc;
	struct vioc_wdma *deintl_wdma = &vioc->deintl_wdma;
	struct vioc_rdma *disp_rdma = &vioc->rdma;
	struct vioc_wmix *disp_wmix = &vioc->wmix;
	unsigned int sw, sh;	// src image size in rdma
	unsigned int dw, dh;	// destination size in scaler
	int bypass = 0;

	/* deintl_path
	 * [rdma]-[viqe]-[wmix]-[SC]-[WDMA]-->
	 */
	/* sc */
	sw = vioc->deintl_rdma.width;
	sh = vioc->deintl_rdma.height;
	dw = vout->disp_rect.width;
	dh = vout->disp_rect.height;
	if (dw == sw && dh == sh)
		bypass = 1;
	VIOC_SC_SetBypass(vioc->deintl_sc.addr, bypass);
	VIOC_SC_SetDstSize(vioc->deintl_sc.addr, dw, dh);	// set destination size in scaler
	VIOC_SC_SetOutSize(vioc->deintl_sc.addr, dw, dh);	// set output size in scaler
	VIOC_SC_SetUpdate(vioc->deintl_sc.addr);

	/* wdma */
	deintl_wdma->width = dw;							// depend on scaled image
	deintl_wdma->height = dh;
	VIOC_WDMA_SetImageSize(deintl_wdma->addr, deintl_wdma->width, deintl_wdma->height);
	VIOC_WDMA_SetImageOffset(deintl_wdma->addr, deintl_wdma->fmt, deintl_wdma->width);
	VIOC_WDMA_SetImageUpdate(deintl_wdma->addr);

	/* disp_path
	 * -->[RDMA]-[WMIX]-[disp]
	 */
	/* rdma */
	disp_rdma->width = vout->crop_rect.width;
	disp_rdma->height = vout->crop_rect.height;
	VIOC_RDMA_SetImageSize(disp_rdma->addr, disp_rdma->width, disp_rdma->height);
	VIOC_RDMA_SetImageOffset(disp_rdma->addr, disp_rdma->fmt, dw);
	VIOC_RDMA_SetImageUpdate(disp_rdma->addr);

	/* wmix */
	disp_wmix->left = vout->crop_rect.left;
	disp_wmix->top = vout->crop_rect.top;
	VIOC_WMIX_SetPosition(disp_wmix->addr, disp_wmix->pos, disp_wmix->left, disp_wmix->top);
	VIOC_WMIX_SetUpdate(disp_wmix->addr);
}

int vout_deintl_init(struct tcc_vout_device *vout)
{
	struct tcc_vout_vioc *vioc = vout->vioc;
	struct vioc_rdma *rdma = &vioc->deintl_rdma;
	struct vioc_wdma *wdma = &vioc->deintl_wdma;
	int ret = 0;

	vout->wakeup_int = 0;
	vout->frame_count = 0;

	/* 0. reset deintl_path */
	deintl_path_reset(vioc);

	/* 1. rdma */
	rdma->intl = vout->deinterlace > VOUT_DEINTL_VIQE_BYPASS ? 1 : 0;
	rdma->img.base0 = 0;
	rdma->img.base1 = 0;
	rdma->img.base2 = 0;
	rdma->width = vout->src_pix.width;

	/*
	 * The VIQE need 4-line align
	 */
	if ((vout->src_pix.height % 4) &&
		(vout->deinterlace == VOUT_DEINTL_VIQE_3D || vout->deinterlace == VOUT_DEINTL_VIQE_2D)) {
		rdma->height = ROUND_UP_4(vout->src_pix.height);
		dprintk("viqe 4-line align: %d -> %d\n", vout->src_pix.height, rdma->height);
	} else {
		rdma->height = vout->src_pix.height;
	}

	rdma->y_stride = vout->src_pix.bytesperline & 0x0000ffff;
	dprintk("Y-stride(%d) UV-stride(%d)\n", rdma->y_stride,
			(vout->src_pix.bytesperline & 0xffff0000) >> 16);

	deintl_rdma_setup(rdma);

	/* 2. deinterlacer */
	switch (vout->deinterlace) {
	case VOUT_DEINTL_VIQE_3D:
	case VOUT_DEINTL_VIQE_2D:
	case VOUT_DEINTL_VIQE_BYPASS:
		#if defined(CONFIG_TCC_VIOCMG)
		dprintk("skip deintl_viqe_setup\r\n");
		#else
		ret = deintl_viqe_setup(vout, vout->deinterlace, 1);
		#endif
		break;
	case VOUT_DEINTL_S:
		ret = deintl_s_setup(vout);
		break;
	case VOUT_DEINTL_NONE:
		/* only use rdma-wmix-wdma */
		break;
	default:
		return -EINVAL;
		break;
	}

	/* 3. scaler */
	deintl_scaler_setup(vout);

	/* 4. wdma */
	wdma->width = vout->disp_rect.width;	// depend on scaled image
	wdma->height = vout->disp_rect.height;
	wdma->cont = 0;							// 0: frame-by-frame mode
	deintl_wdma_setup(wdma);

	vioc_config_stop_req(0);

	/* 5. wdma interrupt */
	if (vout->deinterlace > VOUT_DEINTL_VIQE_BYPASS && wdma->irq_enable == 0) {
		wdma->irq_enable++;		// set interrupt flag
		synchronize_irq(wdma->irq);
		vioc_intr_clear(wdma->vioc_intr->id, wdma->vioc_intr->bits);
		ret = request_irq(wdma->irq, wdma_irq_handler, IRQF_SHARED, "vout_wdma", vout);
		if(ret)
			printk(KERN_ERR VOUT_NAME ": [error] wdma_irq_handler failed\n");
		vioc_intr_enable(wdma->vioc_intr->id, wdma->vioc_intr->bits);
	}
	print_vioc_deintl_path(vout, "vout_deintl_init");
	return ret;
}

void vout_deintl_deinit(struct tcc_vout_device *vout)
{
	struct tcc_vout_vioc *vioc = vout->vioc;

	vout_deintl_ctrl(vioc, 0);

	#ifdef CONFIG_ARCH_TCC_MAP_CONVERTER
	{
		if(vout->onthefly_mode) {
			unsigned int MC_NUM;
			if(tca_get_MC_Connect_to_RDMA(&MC_NUM, VIOC_RDMA_03))
			{
				dprintk("MAP_CONV%d plug-out \n", MC_NUM);
				mc_plugged = 0;

				tca_map_convter_onoff(MC_NUM, 0);
				tca_map_convter_wait_done(MC_NUM);

				tca_set_MC_Connect_to_RDMA(MC_NUM, VIOC_RDMA_03, 0);
				VIOC_CONFIG_SWReset(vioc->config, VIOC_CONFIG_MC, MC_NUM, VIOC_CONFIG_RESET);
				VIOC_CONFIG_SWReset(vioc->config, VIOC_CONFIG_MC, MC_NUM, VIOC_CONFIG_CLEAR);
			}
		} else {
			unsigned int MC_NUM;
			if(tca_get_MC_Connect_to_RDMA(&MC_NUM, VIOC_RDMA_13))
			{
				dprintk("MAP_CONV%d plug-out \n", MC_NUM);
				mc_plugged = 0;

				tca_map_convter_onoff(MC_NUM, 0);
				tca_map_convter_wait_done(MC_NUM);

				tca_set_MC_Connect_to_RDMA(MC_NUM, VIOC_RDMA_13, 0);
				VIOC_CONFIG_SWReset(vioc->config, VIOC_CONFIG_MC, MC_NUM, VIOC_CONFIG_RESET);
				VIOC_CONFIG_SWReset(vioc->config, VIOC_CONFIG_MC, MC_NUM, VIOC_CONFIG_CLEAR);
			}

		}
	}
	#endif

	VIOC_CONFIG_PlugOut(vioc->deintl_sc.ch);

	switch (vout->deinterlace) {
	case VOUT_DEINTL_VIQE_2D:
	case VOUT_DEINTL_VIQE_3D:
		if (vioc->deintl_wdma.irq_enable) {
			vioc->deintl_wdma.irq_enable--;
			vioc_intr_disable(vioc->deintl_wdma.id, vioc->deintl_wdma.vioc_intr->bits);
			free_irq(vioc->deintl_wdma.irq, vout);
		}
	case VOUT_DEINTL_VIQE_BYPASS:
		#if defined(CONFIG_TCC_VIOCMG)
		if(viocmg_lock_viqe(VIOCMG_CALLERID_VOUT) == 0) {
			VIOC_CONFIG_PlugOut(vioc->deintl_viqe.ch);
			viocmg_free_viqe(VIOCMG_CALLERID_VOUT);
		}
		#else
		VIOC_CONFIG_PlugOut(vioc->deintl_viqe.ch);
		#endif
		break;
	case VOUT_DEINTL_S:
		if (vioc->deintl_wdma.irq_enable) {
			vioc->deintl_wdma.irq_enable--;
			vioc_intr_disable(vioc->deintl_wdma.id, vioc->deintl_wdma.vioc_intr->bits);
			free_irq(vioc->deintl_wdma.irq, vout);
		}
		VIOC_CONFIG_PlugOut(vioc->deintl_s.ch);
		break;
	case VOUT_DEINTL_NONE:
	default:
		break;
	}
	#if defined(CONFIG_TCC_VIOCMG)
	if(vout->viqe_to_deints) {
		vout->viqe_to_deints = 0;

		/* reset deintl_s */
		BITSET(vioc->config->uSOFTRESET.nREG[1], vioc->deintl_s.swrst_bit);
		BITCLR(vioc->config->uSOFTRESET.nREG[1], vioc->deintl_s.swrst_bit);

		dprintk(" plugout-DEINTS\r\n");
		VIOC_CONFIG_PlugOut(VIOC_DEINTLS);
	}
	#endif
}

static void sub_rdma_setup(struct vioc_rdma *sub_rdma)
{
	sub_rdma->addr->uCTRL.bREG.SWAP = sub_rdma->swap;
	VIOC_RDMA_SetImageSize(sub_rdma->addr, sub_rdma->width, sub_rdma->height);
	VIOC_RDMA_SetImageFormat(sub_rdma->addr, sub_rdma->fmt);
	VIOC_RDMA_SetImageOffset(sub_rdma->addr, sub_rdma->fmt, sub_rdma->width);
	VIOC_RDMA_SetImageAlphaSelect(sub_rdma->addr, RDMA_ALPHA_ASEL_PIXEL);
	VIOC_RDMA_SetImageAlphaEnable(sub_rdma->addr, 1);
	VIOC_RDMA_SetImageUpdate(sub_rdma->addr);
}

static void sub_wmix_setup(struct vioc_wmix *wmix)
{
	if(wmix->id == VIOC_WMIX_01) {
		VIOC_CONFIG_WMIXPath(WMIX10, 0);		// WMIX1 0'th layer is Bypass PATH
		VIOC_CONFIG_WMIXPath(WMIX13, 1);		// WMIX1 3'rd layer is Mixing PATH
	}

    #if defined(CONFIG_TCC_VIOCMG)
    viocmg_set_wmix_ovp(VIOCMG_CALLERID_VOUT, wmix->id, wmix->ovp);
    #else
	VIOC_WMIX_SetOverlayPriority(wmix->addr, wmix->ovp);
	#endif
	VIOC_WMIX_SetSize(wmix->addr, wmix->width, wmix->height);
	VIOC_WMIX_SetPosition(wmix->addr, wmix->pos, wmix->left, wmix->top);
	VIOC_WMIX_SetUpdate(wmix->addr);
}

#ifdef CONFIG_VOUT_USE_VSYNC_INT
void vout_pop_all_buffer(struct tcc_vout_device *vout)
{
	vout->clearFrameMode = 1;	// enable clear frame mode

	if(wait_event_interruptible_timeout(vout->frame_wait, vout->wakeup_int == 1, msecs_to_jiffies(vout->update_gap_time * 2)) <= 0)
		printk(KERN_ERR VOUT_NAME": [error] wdma interrupt timeout\n");
	vout->wakeup_int = 0;

	dprintk("[Before]readable_bufs(%d) pushIdx(%d) popIdx(%d) clearIdx(%d) displayed_bufs(%d)\n", readable_bufs, pushIdx, popIdx, clearIdx, displayed_bufs);
	displayed_bufs += readable_bufs;
	readable_bufs = 0;
	dprintk("[After]readable_bufs(%d) pushIdx(%d) popIdx(%d) clearIdx(%d) displayed_bufs(%d)\n", readable_bufs, pushIdx, popIdx, clearIdx, displayed_bufs);

	// for interlace
	vout->firstFieldFlag = 0;

	dprintk("\n");
	return;
}
#endif

int vout_subplane_onthefly_init(struct tcc_vout_device *vout)
{
	struct tcc_vout_vioc *vioc = vout->vioc;
	struct vioc_rdma *rdma = &vioc->sub_rdma_ext;
	struct vioc_wmix *wmix = &vioc->sub_wmix_ext;

	rdma->fmt = VIOC_IMG_FMT_ARGB8888;
	rdma->swap = 5;
	rdma->width = vioc->sub_alpha.width;
	rdma->height = vioc->sub_alpha.height;

	rdma->addr->uCTRL.bREG.SWAP = rdma->swap;
	VIOC_RDMA_SetImageSize(rdma->addr, rdma->width, rdma->height);
	VIOC_RDMA_SetImageFormat(rdma->addr, rdma->fmt);
	VIOC_RDMA_SetImageOffset(rdma->addr, rdma->fmt, rdma->width);
	VIOC_RDMA_SetImageAlphaSelect(rdma->addr, RDMA_ALPHA_ASEL_PIXEL);
	VIOC_RDMA_SetImageAlphaEnable(rdma->addr, ON);

	wmix->width = vout->disp_rect.width;
	wmix->height = vout->disp_rect.height;
	VIOC_WMIX_GetOverlayPriority(wmix->addr, &wmix->ovp);
}

int vout_subplane_onthefly_qbuf(struct tcc_vout_device * vout)
{
	struct tcc_vout_vioc *vioc = vout->vioc;
	struct vioc_rdma *rdma = &vioc->sub_rdma_ext;
	struct vioc_wmix *wmix = &vioc->sub_wmix_ext;

	/* check alpha on/off */
	if (0 == vioc->sub_alpha.on) {
		vout_subplane_ctrl(vout, 0);
		return 1;
	}

	if(vioc->sub_buf_init == 0) {
		vioc->sub_buf_init = 1;
		vout_subplane_onthefly_init(vout);
	}

	/* check buf changing */
	if (vioc->sub_buf_current_index == vioc->sub_alpha.buf_index)
		return 2;
	
	vioc->sub_buf_current_index = vioc->sub_alpha.buf_index;
	
	/* check subtitle size */
	if ((rdma->width != vioc->sub_alpha.width) || (rdma->height != vioc->sub_alpha.height)) {
		rdma->width = vioc->sub_alpha.width;
		rdma->height = vioc->sub_alpha.height;

		/* change rdma base address */
		VIOC_RDMA_SetImageSize(rdma->addr, ((rdma->width >> 1) << 1), rdma->height);
		VIOC_RDMA_SetImageOffset(rdma->addr, rdma->fmt, rdma->width);
	}
	
	VIOC_RDMA_SetImageBase(rdma->addr, rdma->img.base0, rdma->img.base1, rdma->img.base2);

	/* check subtitle start position (x,y) */
	if ((wmix->left != vioc->sub_alpha.offset_x) || (wmix->top != vioc->sub_alpha.offset_y)) {
		wmix->left = vioc->sub_alpha.offset_x;
		wmix->top = vioc->sub_alpha.offset_y;
		sub_wmix_setup(wmix);
	}

	vout_subplane_ctrl(vout, 1);	// on
	return 0;
}

int vout_subplane_init(struct tcc_vout_device *vout)
{
	struct tcc_vout_vioc *vioc = vout->vioc;
	struct vioc_rdma *sub_rdma = &vioc->sub_rdma;
	struct vioc_wmix *sub_wmix = &vioc->sub_wmix;

	vioc->sub_buf_init = 0;
	vioc->sub_buf_current_index = -1;

	/* reset rdma */
	BITCSET(vioc->config->uSOFTRESET.nREG[0], (0x1 << vioc->sub_rdma.swrst_bit), (0x1 << vioc->sub_rdma.swrst_bit));
	BITCSET(vioc->config->uSOFTRESET.nREG[0], (0x1 << vioc->sub_rdma.swrst_bit), (0x0 << vioc->sub_rdma.swrst_bit));

	/* 1. rdma */
	sub_rdma->fmt = VIOC_IMG_FMT_ARGB8888;		// default: ARGB8888
	sub_rdma->width = vout->src_pix.width;		// default: video size
	sub_rdma->height = vout->src_pix.height;
	sub_rdma->swap = 5;							// RGB3, or 5 (BGR3)
	sub_rdma_setup(sub_rdma);

	/* 2. wmix */
	sub_wmix->left = 0;		// default: to assume sub-plane size same video size
	sub_wmix->top = 0;
	sub_wmix->width = sub_rdma->width;
	sub_wmix->height = sub_rdma->height;
	sub_wmix_setup(sub_wmix);

	print_vioc_subplane_info(vout, "vout_subplane_init");
	return 0;
}

int vout_subplane_qbuf(struct tcc_vout_device *vout, struct vioc_alpha *alpha)
{
	struct tcc_vout_vioc *vioc = vout->vioc;
	struct vioc_rdma *sub_rdma = &vioc->sub_rdma;
	struct vioc_wmix *sub_wmix = &vioc->sub_wmix;

	/* check alpha on/off */
	if (0 == alpha->on) {
		vout_subplane_ctrl(vout, 0);
		return 1;
	}

	/* check buf changing */
	if (vioc->sub_buf_current_index == alpha->buf_index)
		return 2;

	vioc->sub_buf_current_index = alpha->buf_index;

	/* check subtitle size */
	if ((sub_rdma->width != alpha->width) || (sub_rdma->height != alpha->height)) {
		sub_rdma->width = alpha->width;
		sub_rdma->height = alpha->height;

		/* change rdma base address */
		VIOC_RDMA_SetImageSize(sub_rdma->addr, ((sub_rdma->width >> 1) << 1), sub_rdma->height);
		VIOC_RDMA_SetImageOffset(sub_rdma->addr, sub_rdma->fmt, sub_rdma->width);
	}
	/* change rdma base address */
	VIOC_RDMA_SetImageBase(sub_rdma->addr, sub_rdma->img.base0, sub_rdma->img.base1, sub_rdma->img.base2);

	/* check subtitle start position (x,y) */
	if ((sub_wmix->left != alpha->offset_x) || (sub_wmix->top != alpha->offset_y)) {
		sub_wmix->left = alpha->offset_x;
		sub_wmix->top = alpha->offset_y;
		sub_wmix_setup(sub_wmix);
	}

	vout_subplane_ctrl(vout, 1);	// on
	return 0;
}

void vout_subplane_deinit(struct tcc_vout_device *vout)
{
	if(vout->onthefly_mode)
		VIOC_RDMA_SetImageDisable(vout->vioc->sub_rdma_ext.addr);
	else
		VIOC_RDMA_SetImageDisable(vout->vioc->sub_rdma.addr);
	vout->vioc->sub_buf_init = 0;
	vout->vioc->sub_buf_current_index = -1;
}

void vout_subplane_ctrl(struct tcc_vout_device *vout, int enable)
{
	struct tcc_vout_vioc *vioc = vout->vioc;
	if (enable) {
		if(vout->onthefly_mode)
			VIOC_RDMA_SetImageEnable(vioc->sub_rdma_ext.addr);
		else
			VIOC_RDMA_SetImageEnable(vioc->sub_rdma.addr);
	} else {
		if(vout->onthefly_mode)
			VIOC_RDMA_SetImageDisable(vioc->sub_rdma_ext.addr);
		else
			VIOC_RDMA_SetImageDisable(vioc->sub_rdma.addr);
	}
	dprintk("%d\n", enable);
}

#ifdef CONFIG_VOUT_USE_VSYNC_INT
void vsync_intr_enable(struct tcc_vout_device *vout)
{
	struct tcc_vout_vioc *vioc = vout->vioc;
	int ret = 0;

	if(vioc->disp.irq_enable == 0)
	{
		vioc->disp.irq_enable++;        // set interrupt flag
	    vioc_intr_disable(vioc->disp.id, vioc->disp.vioc_intr->bits);
		ret = request_irq(vioc->disp.irq, vsync_irq_handler, IRQF_SHARED, "vout_vsync", vout);
		vioc_intr_enable(vioc->disp.id, vioc->disp.vioc_intr->bits);
	}
	dprintk("\n");
}
EXPORT_SYMBOL(vsync_intr_enable);

void vsync_intr_disable(struct tcc_vout_device *vout)
{
	struct tcc_vout_vioc *vioc = vout->vioc;

	if(vioc->disp.irq_enable) {
		vioc->disp.irq_enable--;
		vioc_intr_disable(vioc->disp.id, vioc->disp.vioc_intr->bits);
		free_irq(vioc->disp.irq, vout);
	}
	dprintk("\n");
}
EXPORT_SYMBOL(vsync_intr_disable);

/*
 * kernel timer functions
 */
void ktimer_handler(unsigned long arg)
{
	struct tcc_vout_device *vout = (struct tcc_vout_device *)arg;

	if(readable_bufs)
	{
		struct v4l2_buffer *buf = &displaybuffer[popIdx];
		if((((buf->timestamp.tv_sec*1000)+(buf->timestamp.tv_usec/1000)) - tcc_vout_get_time(vout)) < 0) {
			printk(KERN_ERR VOUT_NAME": DROP FRAME!!! [%d/%ldmsec]\n", buf->index, ((buf->timestamp.tv_sec*1000)+(buf->timestamp.tv_usec/1000)));
			if(++popIdx >= MAX_DISPBUF_NUM)
				popIdx = 0;
			readable_bufs--;
			displayed_bufs++;
		}
	}

	if(ktimer_enable) {
		ktimer.expires = get_jiffies_64() + 3*HZ/1000;
		add_timer(&ktimer);
	}
}

void vout_ktimer_ctrl(struct tcc_vout_device *vout, unsigned int onoff)
{
	if(onoff)
	{
		memset(&ktimer, 0x0, sizeof(struct timer_list));

		init_timer(&ktimer);
		ktimer.expires = get_jiffies_64() + 3*HZ/1000;
		ktimer.data = (unsigned long)vout;
		ktimer.function = ktimer_handler;

		ktimer_enable = ON;
		add_timer(&ktimer);
	}
	else
	{
		ktimer_enable = OFF;
		msleep(10);
		del_timer(&ktimer);
	}
	dprintk("(%d)\n", onoff);
}
EXPORT_SYMBOL(vout_ktimer_ctrl);
#endif

/* call by tcc_vout_open()
 */
int vout_vioc_init(struct tcc_vout_device *vout)
{
	int ret = 0, index;
	struct tcc_vout_vioc *vioc = vout->vioc;

	ret = vout_vioc_set_default(vout);//vout_set_vout_path(vout);
	if (ret < 0)
		return ret;

	/* Get WMIX size
	 */
	vout_wmix_getsize(vout, &vioc->wmix.width, &vioc->wmix.height);
	vout->disp_rect.width = vout->panel_rect.width = vioc->wmix.width;
	vout->disp_rect.height= vout->panel_rect.height = vioc->wmix.height;
	if(vout->disp_rect.width == 0 || vout->disp_rect.height == 0) {
		printk(KERN_ERR VOUT_NAME": [error] output device size(%dx%d) \n", vout->disp_rect.width, vout->disp_rect.height);
		return -EBUSY;
	}

	/* first field flag */
	vout->firstFieldFlag = 0;

	/* init driver data */
	vout->qbufs = kzalloc(sizeof(struct tcc_v4l2_buffer) * VIDEO_MAX_FRAME, GFP_KERNEL);
	if(!vout->qbufs)
		return -ENOMEM;

	vout->nr_qbufs = 0; // current number of created buffers
	vout->mapped = OFF;

	for(index = 0; index < MAX_DISPBUF_NUM; index++) {
		displaybuffer[index].m.planes = kzalloc(sizeof(struct v4l2_plane) * MPLANE_NUM, GFP_KERNEL);
		if(!displaybuffer[index].m.planes) {
			printk(KERN_ERR VOUT_NAME": [error] memory allocation fail \n");
			return -ENOMEM;
		}
	}

	// onthefly
	vout->onthefly_mode = OFF;
	vout->display_done = OFF;
	vout->last_displayed_buf_idx = 0;

	#ifdef CONFIG_VOUT_USE_VSYNC_INT
	if(vioc->disp.irq_enable == 0) {
		vioc->disp.irq_enable++;        // set interrupt flag
		vioc_intr_disable(vioc->disp.id, vioc->disp.vioc_intr->bits);
		ret = request_irq(vioc->disp.irq, vsync_irq_handler, IRQF_SHARED, "vout_vsync", vout);
		vioc_intr_enable(vioc->disp.id, vioc->disp.vioc_intr->bits);
	}

	if(vout_timer)
		tcc_timer_enable(vout_timer);
	#endif
	dprintk("\n");

	return ret;
}

void vout_deinit(struct tcc_vout_device *vout)
{
	struct tcc_vout_vioc *vioc = vout->vioc;

	VIOC_RDMA_SetImageDisable(vioc->rdma.addr);

	#ifdef VOUT_DISP_PATH_SCALER
	if (vioc->sc.id >= 0)
		VIOC_CONFIG_PlugOut(vioc->sc.ch);
	#endif

	#ifdef CONFIG_VOUT_USE_VSYNC_INT
	if(ktimer_enable) {
		ktimer_enable = OFF;
		del_timer(&ktimer);
	}

	if(vioc->disp.irq_enable) {
		vioc->disp.irq_enable--;
		vioc_intr_disable(vioc->disp.id, vioc->disp.vioc_intr->bits);
		free_irq(vioc->disp.irq, vout);
	}

	if(vout_timer) {
		tcc_timer_disable(vout_timer);
		tcc_unregister_timer(vout_timer);
	}
	#endif
	dprintk("\n");
}

