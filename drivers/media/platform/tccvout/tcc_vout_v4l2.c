/*
 * tcc_vout_v4l2.c
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
 * ChangeLog:
 *   
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/mm.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_address.h>
#endif

#include <asm/io.h>

#include <video/tcc/tcc_vout_ioctl.h>

#include "tcc_vout.h"
#include "tcc_vout_core.h"
#include "tcc_vout_dbg.h"
#include "tcc_vout_attr.h"


#ifdef CONFIG_OF
static struct of_device_id tccvout_0_of_match[] = {
	{ .compatible = "telechips,v4l2_vout0" },
	{}
};
MODULE_DEVICE_TABLE(of, tccvout_0_of_match);
#endif

static struct tcc_vout_device *g_pvout = NULL;

int popIdx = 0;
int pushIdx = -1;
int clearIdx = 0;
int displayed_bufs = 0;
int readable_bufs = 0;
struct v4l2_buffer displaybuffer[MAX_DISPBUF_NUM];

#ifdef CONFIG_VOUT_USE_VSYNC_INT
extern void vsync_intr_enable(struct tcc_vout_device *vout);
extern void vsync_intr_disable(struct tcc_vout_device *vout);
extern void vout_ktimer_ctrl(struct tcc_vout_device *vout, unsigned int onoff);
#endif

/* list of image formats supported by tcc video pipelines */
const struct v4l2_fmtdesc tcc_fmtdesc[] = {
	/* .reserved[0] : vioc image format */
/* YUV formats */
	{
		.description = "'YU12' 12 YUV 4:2:0",
		.pixelformat = V4L2_PIX_FMT_YUV420,
		.reserved[0] = VIOC_IMG_FMT_YUV420SEP,
	},
	{
		.description = "'YV12' 12 YVU 4:2:0",
		.pixelformat = V4L2_PIX_FMT_YVU420,
		.reserved[0] = VIOC_IMG_FMT_YUV420SEP,
	},
	{
		.description = "'422P' 16 YUV422 planar",
		.pixelformat = V4L2_PIX_FMT_YUV422P,
		.reserved[0] = VIOC_IMG_FMT_YUV422SEP,
	},
	{
		.description = "'NV12' 12 Y/CbCr 4:2:0",
		.pixelformat = V4L2_PIX_FMT_NV12,
		.reserved[0] = VIOC_IMG_FMT_YUV420IL0,
	},
/* RGB formats */
	{
		.description = "'RGB1' 8 RGB-3-3-2",
		.pixelformat = V4L2_PIX_FMT_RGB332,
		.reserved[0] = VIOC_IMG_FMT_RGB332,		/* R[7:5] G[4:2] B[1:0] */
	},
	{
		.description = "'R444' 16 RGB-4-4-4",
		.pixelformat = V4L2_PIX_FMT_RGB444,		/* xxxxrrrr ggggbbbb */
		.reserved[0] = VIOC_IMG_FMT_ARGB4444,	/* A[15:12] R[11:8] G[7:4] B[3:0] */
	},
	{
		.description = "'RGB0' 16 RGB-5-5-5",
		.pixelformat = V4L2_PIX_FMT_RGB555,
		.reserved[0] = VIOC_IMG_FMT_ARGB1555,	/* A[15] R[14:10] G[9:5] B[4:0] */
	},
	{
		.description = "'RGBP' 16 RGB-5-6-5",
		.pixelformat = V4L2_PIX_FMT_RGB565,
		.reserved[0] = VIOC_IMG_FMT_RGB565,		/* R[15:11] G[10:5] B[4:0] */
	},
	{
		.description = "'RBG3' 24 RGB-8-8-8",
		.pixelformat = V4L2_PIX_FMT_RGB24,
		.reserved[0] = VIOC_IMG_FMT_RGB888,		/* B1[31:24] R[23:16] G[15:8] B0[7:0] */
	},
	{
		.description = "'BGR3' 24 BGR-8-8-8",
		.pixelformat = V4L2_PIX_FMT_BGR24,
		.reserved[0] = VIOC_IMG_FMT_RGB888,		/* RGB swap -> BGR */
	},
	{
		.description = "'RGB4' 32 RGB-8-8-8-8",
		.pixelformat = V4L2_PIX_FMT_RGB32,
		.reserved[0] = VIOC_IMG_FMT_ARGB8888,	/* A[31:24] R[23:16] G[15:8] B[7:0] */
	},
	{
		.description = "'BGR4' 32 BGR-8-8-8-8",
		.pixelformat = V4L2_PIX_FMT_BGR32,
		.reserved[0] = VIOC_IMG_FMT_ARGB8888,	/* RGB swap -> BGR */
	},
};
#define NUM_TCC_FMTDESC (ARRAY_SIZE(tcc_fmtdesc))

/*
 * export symbol
 */
unsigned int tcc_vout_get_status( void )
{
	if(g_pvout == NULL)
		return 0;
	else
		return g_pvout->status;
}
EXPORT_SYMBOL(tcc_vout_get_status);

#ifdef CONFIG_VOUT_USE_VSYNC_INT
void tcc_vout_hdmi_end( void )
{
	struct tcc_vout_vioc *vioc = g_pvout->vioc;
	if(g_pvout->status == TCC_VOUT_RUNNING) {
		printk("vsync interrupt disable\n");

		vioc_intr_disable(vioc->disp.id, vioc->disp.vioc_intr->bits);
		//vsync_intr_disable(g_pvout);

		vout_ktimer_ctrl(g_pvout, ON);
	}
}
EXPORT_SYMBOL(tcc_vout_hdmi_end);

void tcc_vout_hdmi_start( void )
{
	struct tcc_vout_vioc *vioc = g_pvout->vioc;
	if(g_pvout->status == TCC_VOUT_RUNNING) {
		printk("vsync interrupt enable\n");

		vout_ktimer_ctrl(g_pvout, OFF);
		vioc_intr_enable(vioc->disp.id, vioc->disp.vioc_intr->bits);
		//vsync_intr_enable(g_pvout);
	}
}
EXPORT_SYMBOL(tcc_vout_hdmi_start);
#endif

static int tcc_vout_try_fmt(unsigned int pixelformat)
{
	int ifmt;

	for (ifmt = 0; ifmt < NUM_TCC_FMTDESC; ifmt++) {
		if (tcc_fmtdesc[ifmt].pixelformat == pixelformat) {
			return ifmt;
		}
	}

	return ifmt;
}

int tcc_vout_try_bpp(unsigned int pixelformat, enum v4l2_colorspace *colorspace)
{
	int bpp, color;

	switch (pixelformat) {
	/* RGB formats */
	case V4L2_PIX_FMT_RGB332:
		color = V4L2_COLORSPACE_SRGB;
		bpp = 1;
		break;
	case V4L2_PIX_FMT_RGB444:
	case V4L2_PIX_FMT_RGB555:
	case V4L2_PIX_FMT_RGB565:
	case V4L2_PIX_FMT_RGB555X:
	case V4L2_PIX_FMT_RGB565X:
		color = V4L2_COLORSPACE_SRGB;
		bpp = 2;
		break;
	case V4L2_PIX_FMT_BGR666:
	case V4L2_PIX_FMT_BGR24:
	case V4L2_PIX_FMT_RGB24:
		color = V4L2_COLORSPACE_SRGB;
		bpp = 3;
		break;
	case V4L2_PIX_FMT_BGR32:
	case V4L2_PIX_FMT_RGB32:
		color = V4L2_COLORSPACE_SRGB;
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
		color = V4L2_COLORSPACE_JPEG;
		bpp = 1;
		break;

	/* YUV422 Sequential formats */
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_YYUV:
	case V4L2_PIX_FMT_YVYU:
	case V4L2_PIX_FMT_UYVY:
	case V4L2_PIX_FMT_VYUY:
		color = V4L2_COLORSPACE_JPEG;
		bpp = 2;
		break;

	default:
		printk(VOUT_NAME
			": Not supported pixelformat(%c%c%c%c)\n",
			fourcc2char(pixelformat));
		color = V4L2_COLORSPACE_JPEG;
		bpp = 2;
		break;
	}

	if (colorspace != NULL)
		*colorspace = color;

	return bpp;
}

int tcc_vout_try_pix(unsigned int pixelformat)
{
	enum tcc_pix_fmt pfmt;

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
		printk(VOUT_NAME
			": Not supported pixelformat(%c%c%c%c)\n",
			fourcc2char(pixelformat));
		pfmt = TCC_PFMT_YUV420;
		break;
	}

	return pfmt;
}

static void tcc_vout_buffer_copy(struct v4l2_buffer *dst, struct v4l2_buffer *src)
{
	/* remove previous frame info  */
	memset(dst->m.planes, 0x0, sizeof(struct v4l2_plane) * MPLANE_NUM);

	/* copy 'v4l2_buffer' structure */
	dst->index = src->index;
	dst->type = src->type;
	dst->bytesused = src->bytesused;
	dst->flags = src->flags;
	dst->field = src->field;
	dst->timestamp = src->timestamp;
	dst->timecode = src->timecode;
	dst->sequence = src->sequence;
	dst->memory = src->memory;
	dst->length = src->length;
	dst->reserved2 = src->reserved2;
	dst->reserved = src->reserved;

	memcpy(dst->m.planes, src->m.planes, sizeof(struct v4l2_plane) * MPLANE_NUM);
}

static int tcc_v4l2_buffer_set(struct tcc_vout_device *vout, struct v4l2_requestbuffers *req)
{
	struct tcc_v4l2_buffer *b = vout->qbufs;
	unsigned int index, y_offset = 0, uv_offset = 0, total_offset = 0;

	#ifdef VOUT_SUB_PLANE
	/* for sub_plane */
	for (index = vout->nr_qbufs; index < (vout->nr_qbufs + req->count); index++) {
		b[index].buf.m.planes = kzalloc(sizeof(struct v4l2_plane) * MPLANE_NUM, GFP_KERNEL);
	}
	#endif

	if (V4L2_MEMORY_USERPTR == req->memory) {
		for (index = vout->nr_qbufs; index < (vout->nr_qbufs + req->count); index++)
			b[index].index = index;

		/* update current number of created buffer */
		vout->nr_qbufs += req->count;

		/*
		 * We don't have to set v4l2 bufs.
		 */
		return 0;
	}

	switch (vout->pfmt) {
	case TCC_PFMT_YUV422:
		y_offset = vout->src_pix.width * vout->src_pix.height;
		uv_offset = y_offset / 2;
		break;
	case TCC_PFMT_YUV420:
		y_offset = vout->src_pix.width * vout->src_pix.height;
		uv_offset = y_offset / 4;
		break;
	case TCC_PFMT_RGB:
	default:
		break;
	}
	total_offset = y_offset + uv_offset * 2;
	total_offset = PAGE_ALIGN(total_offset);

	/* set base address each buffers
	 */
	dprintk("  alloc_buf    img_base0   img_base1   img_base2\n");
	for (index = vout->nr_qbufs; index < (vout->nr_qbufs + req->count); index++) {
		b[index].index = index;

		switch (vout->src_pix.pixelformat) {
		case V4L2_PIX_FMT_YUV420:	/* YCbCr 4:2:0 separated (VIOC_IMG_FMT_YUV420SEP) */
		case V4L2_PIX_FMT_YVU420:	/* YCbCr 4:2:0 separated (VIOC_IMG_FMT_YUV420SEP) */
		case V4L2_PIX_FMT_YUV422P:	/* YCbCr 4:2:2 separated (VIOC_IMG_FMT_YUV422SEP) */
			b[index].img_base0 = PAGE_ALIGN(vout->pmap.base + total_offset * index);
			b[index].img_base1 = b[index].img_base0 + y_offset;
			b[index].img_base2 = b[index].img_base1 + uv_offset;
			break;
		case V4L2_PIX_FMT_NV12:		/* YCbCr 4:2:0 interleaved type0 (VIOC_IMG_FMT_YUV420IL0) */
		case V4L2_PIX_FMT_NV21:		/* YCbCr 4:2:0 interleaved type1 (VIOC_IMG_FMT_YUV420IL1) */
		case V4L2_PIX_FMT_NV16:		/* YCbCr 4:2:2 interleaved type0 (VIOC_IMG_FMT_YUV422IL0) */
		case V4L2_PIX_FMT_NV61:		/* YCbCr 4:2:2 interleaved type1 (VIOC_IMG_FMT_YUV422IL1) */
			b[index].img_base0 = PAGE_ALIGN(vout->pmap.base + total_offset * index);
			b[index].img_base1 = b[index].img_base0 + y_offset;
			b[index].img_base2 = 0;
			break;
		case V4L2_PIX_FMT_UYVY:		/* YCbCr 4:2:2 UYVY Sequential (VIOC_IMG_FMT_UYVY) */
		case V4L2_PIX_FMT_VYUY:		/* YCbCr 4:2:2 VYUY Sequential (VIOC_IMG_FMT_VYUY) */
		case V4L2_PIX_FMT_YUYV:		/* YCbCr 4:2:2 YUYV Sequential (VIOC_IMG_FMT_YUYV) */
		case V4L2_PIX_FMT_YVYU:		/* YCbCr 4:2:2 YVYU Sequential (VIOC_IMG_FMT_YVYU) */
			b[index].img_base0 = PAGE_ALIGN(vout->pmap.base + total_offset * index);
			b[index].img_base1 = 0;
			b[index].img_base2 = 0;
			break;
		default:					/* RGB and etc... */
			b[index].img_base0 = PAGE_ALIGN(vout->pmap.base + vout->src_pix.sizeimage * index);
			b[index].img_base1 = 0;
			b[index].img_base2 = 0;
		}
		dprintk("     [%02d]     0x%08x  0x%08x  0x%08x\n", index, b[index].img_base0, b[index].img_base1, b[index].img_base2);
	}
	dprintk("-----------------------------------------------------\n");

	// update current number of created buffer
	vout->nr_qbufs += req->count;

	print_v4l2_reqbufs_format(vout->pfmt, vout->src_pix.pixelformat, "tcc_v4l2_buffer_set");
	return 0;
}

static int tcc_v4l2_buffer_add(struct tcc_vout_device *vout, struct v4l2_create_buffers* create)
{
	struct tcc_v4l2_buffer *b = vout->qbufs;
	unsigned int index, y_offset = 0, uv_offset = 0, total_offset = 0;

	#ifdef VOUT_SUB_PLANE
	/* for sub_plane */
	for (index = vout->nr_qbufs; index < (vout->nr_qbufs + create->count); index++) {
		b[index].buf.m.planes = kzalloc(sizeof(struct v4l2_plane) * MPLANE_NUM, GFP_KERNEL);
	}
	#endif

	if (V4L2_MEMORY_USERPTR == create->memory) {
		for (index = vout->nr_qbufs; index < (vout->nr_qbufs + create->count); index++)
			b[index].index = index;

		/* update current number of created buffer */
		vout->nr_qbufs += create->count;

		/*
		 * We don't have to set v4l2 bufs.
		 */
		return 0;
	}

	switch (vout->pfmt) {
	case TCC_PFMT_YUV422:
		y_offset = vout->src_pix.width * vout->src_pix.height;
		uv_offset = y_offset / 2;
		break;
	case TCC_PFMT_YUV420:
		y_offset = vout->src_pix.width * vout->src_pix.height;
		uv_offset = y_offset / 4;
		break;
	case TCC_PFMT_RGB:
	default:
		break;
	}
	total_offset = y_offset + uv_offset * 2;
	total_offset = PAGE_ALIGN(total_offset);

	/* set base address each buffers
	 */
	dprintk("  alloc_buf    img_base0   img_base1   img_base2\n");
	for (index = vout->nr_qbufs; index < (vout->nr_qbufs + create->count); index++) {
		b[index].index = index;

		switch (vout->src_pix.pixelformat) {
		case V4L2_PIX_FMT_YUV420:	/* YCbCr 4:2:0 separated (VIOC_IMG_FMT_YUV420SEP) */
		case V4L2_PIX_FMT_YVU420:	/* YCbCr 4:2:0 separated (VIOC_IMG_FMT_YUV420SEP) */
		case V4L2_PIX_FMT_YUV422P:	/* YCbCr 4:2:2 separated (VIOC_IMG_FMT_YUV422SEP) */
			b[index].img_base0 = PAGE_ALIGN(vout->pmap.base + total_offset * index);
			b[index].img_base1 = b[index].img_base0 + y_offset;
			b[index].img_base2 = b[index].img_base1 + uv_offset;
			break;
		case V4L2_PIX_FMT_NV12:		/* YCbCr 4:2:0 interleaved type0 (VIOC_IMG_FMT_YUV420IL0) */
		case V4L2_PIX_FMT_NV21:		/* YCbCr 4:2:0 interleaved type1 (VIOC_IMG_FMT_YUV420IL1) */
		case V4L2_PIX_FMT_NV16:		/* YCbCr 4:2:2 interleaved type0 (VIOC_IMG_FMT_YUV422IL0) */
		case V4L2_PIX_FMT_NV61:		/* YCbCr 4:2:2 interleaved type1 (VIOC_IMG_FMT_YUV422IL1) */
			b[index].img_base0 = PAGE_ALIGN(vout->pmap.base + total_offset * index);
			b[index].img_base1 = b[index].img_base0 + y_offset;
			b[index].img_base2 = 0;
			break;
		case V4L2_PIX_FMT_UYVY:		/* YCbCr 4:2:2 UYVY Sequential (VIOC_IMG_FMT_UYVY) */
		case V4L2_PIX_FMT_VYUY:		/* YCbCr 4:2:2 VYUY Sequential (VIOC_IMG_FMT_VYUY) */
		case V4L2_PIX_FMT_YUYV:		/* YCbCr 4:2:2 YUYV Sequential (VIOC_IMG_FMT_YUYV) */
		case V4L2_PIX_FMT_YVYU:		/* YCbCr 4:2:2 YVYU Sequential (VIOC_IMG_FMT_YVYU) */
			b[index].img_base0 = PAGE_ALIGN(vout->pmap.base + total_offset * index);
			b[index].img_base1 = 0;
			b[index].img_base2 = 0;
			break;
		default:					/* RGB and etc... */
			b[index].img_base0 = PAGE_ALIGN(vout->pmap.base + vout->src_pix.sizeimage * index);
			b[index].img_base1 = 0;
			b[index].img_base2 = 0;
		}
		dprintk("     [%02d]     0x%08x  0x%08x  0x%08x\n", index, b[index].img_base0, b[index].img_base1, b[index].img_base2);
	}
	dprintk("-----------------------------------------------------\n");

	// update current number of created buffer
	vout->nr_qbufs += create->count;

	print_v4l2_reqbufs_format(vout->pfmt, vout->src_pix.pixelformat, "tcc_v4l2_buffer_set");

	return 0;
}

static int tcc_deintl_buffer_set(struct tcc_vout_device *vout)
{
	struct tcc_v4l2_buffer *b;
	unsigned int total_offset, y_offset = 0, uv_offset = 0;
	unsigned int base, i;
	int actual_bufs;
	unsigned int width, height;

	/*
	 * CAUTION!!!: deintl_buf (deintl_wdma buf) is always YUV420 Interleaved Type0.
	 */

	/* calculate buf count
	 * Size of display image can't be bigger than panel size
	 */
	actual_bufs = vout->deintl_pmap.size / vout->deintl_buf_size;

	if (2 > actual_bufs) {
		printk(KERN_ERR VOUT_NAME ": [error] deintl_nr_bufs is not enough (%d).\n", actual_bufs);
		return -ENOMEM;
	}
	dprintk("request deintl_nr_bufs(%d), actual buf(%d)\n", vout->deintl_nr_bufs, actual_bufs);
	if ((vout->deintl_nr_bufs > actual_bufs) || (vout->deintl_nr_bufs <= 0))
		vout->deintl_nr_bufs = actual_bufs;

	vout->deintl_bufs = kzalloc(sizeof(struct tcc_v4l2_buffer) * vout->deintl_nr_bufs, GFP_KERNEL);
	if (!vout->deintl_bufs)
		return -ENOMEM;
	b = vout->deintl_bufs;

	width = vout->disp_rect.width;
	height = vout->disp_rect.height;

	y_offset = width * height;
	uv_offset = y_offset / 4;
	total_offset = y_offset + uv_offset * 2;
	total_offset = PAGE_ALIGN(total_offset);
	base = vout->deintl_pmap.base;

	/* set base address each buffers
	 */
	dprintk("  alloc_buf    img_base0   img_base1   img_base2\n");
	for (i = 0; i < vout->deintl_nr_bufs; i++) {
		b[i].index = i;
		b[i].img_base0 = PAGE_ALIGN(base + total_offset * i);
		b[i].img_base1 = b[i].img_base0 + y_offset;
		b[i].img_base2 = 0;
		dprintk("     [%02d]     0x%08x  0x%08x  0x%08x\n", i, b[i].img_base0, b[i].img_base1, b[i].img_base2);
	}
	dprintk("-----------------------------------------------------\n");

	print_v4l2_reqbufs_format(vout->pfmt, vout->src_pix.pixelformat, "tcc_v4l2_buffer_set");
	return 0;
}

/*
 * V4L2 ioctls
 */

//TODO: gst_v4l2_fill_lists()
static int vidioc_enum_input(struct file *file, void *fh, struct v4l2_input *input)
{
	diprintk("\n");
	return -EINVAL;
}

static int vidioc_queryctrl(struct file *file, void *fh, struct v4l2_queryctrl *ctrl)
{
	int ret = 0;

	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		ctrl->flags = V4L2_CTRL_FLAG_DISABLED;
		break;
	default:
		ctrl->name[0] = '\0';
		ret = -EINVAL;
		break;
	}

	diprintk("\n");
	return ret;
}

static int vidioc_g_ctrl(struct file *file, void *fh, struct v4l2_control *ctrl)
{
	int ret = 0;
	diprintk("v4l2_control id(0x%x)\n", ctrl->id);
	return ret;
}

static int vidioc_s_ctrl(struct file *file, void *fh, struct v4l2_control *ctrl)
{
	int ret = 0;
	diprintk("v4l2_control id(0x%x) value(0x%x)\n", ctrl->id, ctrl->value);
	return ret;
}

static int vidioc_querycap(struct file *file, void *fh, struct v4l2_capability *cap)
{
	struct tcc_vout_device *vout = fh;

	strlcpy(cap->driver, VOUT_NAME, sizeof(cap->driver));
	snprintf(cap->card, sizeof(cap->card), "Telechips video output");
	snprintf(cap->bus_info, sizeof(cap->bus_info), "vout:%d", vout->id);
	cap->capabilities = V4L2_CAP_STREAMING | V4L2_CAP_VIDEO_OUTPUT;
	cap->version = VOUT_VERSION;

	diprintk("\n");
	#ifdef CONFIG_TCC_VOUT_DBG_INFO
	print_v4l2_capability(cap, "vidioc_querycap");
	#endif
	return 0;
}

static int vidioc_enum_fmt_vid_out(struct file *file, void *fh, struct v4l2_fmtdesc *fmt)
{
	int index = fmt->index;

	dprintk("\n");

	if (index >= NUM_TCC_FMTDESC){
		return -EINVAL;
	}

	memcpy(fmt, &tcc_fmtdesc[index], sizeof(struct v4l2_fmtdesc));

	#ifdef CONFIG_TCC_VOUT_DBG_INFO
	print_v4l2_fmtdesc(fmt, "vidioc_enum_fmt_vid_out");
	#endif
	return 0;
}

static int vidioc_try_fmt_vid_out(struct file *file, void *fh, struct v4l2_format *f)
{
	int /*ifmt,*/ bpp, pfmt;
	unsigned int sizeimage;
	unsigned int bytesperline;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	dprintk("\n");
	//ifmt = tcc_vout_try_fmt(f->fmt.pix_mp.pixelformat);
	bpp = tcc_vout_try_bpp(pix->pixelformat, &pix->colorspace);
	pfmt = tcc_vout_try_pix(pix->pixelformat);
	bytesperline = pix->width * bpp;

	switch (pfmt) {
	case TCC_PFMT_YUV422:
		sizeimage = pix->width * pix->height * 2;
		break;
	case TCC_PFMT_YUV420:
		sizeimage = pix->width * pix->height * 3 / 2;
		break;
	case TCC_PFMT_RGB:
	default:
		sizeimage = bytesperline * pix->height;
		break;
	}

	pix->bytesperline = bytesperline;
	pix->sizeimage = PAGE_ALIGN(sizeimage);

	#ifdef CONFIG_TCC_VOUT_DBG_INFO
	dprintk("pixelformat: %c%c%c%c\n", fourcc2char(pix->pixelformat));
	#endif
	return 0;
}

//static int vidioc_try_fmt_vid_out_mplane(struct file *file, void *fh, struct v4l2_format *f)
//{
//	int /*ifmt,*/ bpp, pfmt;
//	unsigned int sizeimage;
//	unsigned short bytesperline;
//	struct v4l2_pix_format_mplane *pix_mp = &f->fmt.pix_mp;
//	unsigned int subtitle_w, subtitle_h, subtitle_fmt;
//
//	/* 1st plane (video)
//	 */
//	//ifmt = tcc_vout_try_fmt(f->fmt.pix_mp.pixelformat);
//	bpp = tcc_vout_try_bpp(pix_mp->pixelformat, &pix_mp->colorspace);
//	pfmt = tcc_vout_try_pix(pix_mp->pixelformat);
//	bytesperline = pix_mp->width * bpp;
//
//	switch (pfmt) {
//	case TCC_PFMT_YUV422:
//		sizeimage = pix_mp->width * pix_mp->height * 2;
//		break;
//	case TCC_PFMT_YUV420:
//		sizeimage = pix_mp->width * pix_mp->height * 3 / 2;
//		break;
//	case TCC_PFMT_RGB:
//	default:
//		sizeimage = bytesperline * pix_mp->height;
//		break;
//	}
//
//	pix_mp->plane_fmt[0].bytesperline = bytesperline;
//	pix_mp->plane_fmt[0].sizeimage = PAGE_ALIGN(sizeimage);
//
//	/* 2nd plnae (subtitle)
//	 */
//	/* Fill tcc subtitle information */
//	pix_mp->plane_fmt[1].reserved[0] = TCC_V4L2_OUTPUT_SRC_SUBTITLE;
//	subtitle_fmt = V4L2_PIX_FMT_RGB32;
//	subtitle_w = 1920;
//	subtitle_h = 1080;
//
//	if (pix_mp->plane_fmt[1].reserved[0] == TCC_V4L2_OUTPUT_SRC_SUBTITLE) {
//		bpp = tcc_vout_try_bpp(subtitle_fmt, NULL);
//		pfmt = tcc_vout_try_pix(subtitle_fmt);
//		bytesperline = subtitle_w * bpp;
//
//		switch (pfmt) {
//		case TCC_PFMT_YUV422:
//			sizeimage = subtitle_w * subtitle_h * 2;
//			break;
//		case TCC_PFMT_YUV420:
//			sizeimage = subtitle_w * subtitle_h * 3 / 2;
//			break;
//		case TCC_PFMT_RGB:
//		default:
//			sizeimage = bytesperline * subtitle_h;
//			break;
//		}
//
//		pix_mp->plane_fmt[1].bytesperline = bytesperline;
//		pix_mp->plane_fmt[1].sizeimage = PAGE_ALIGN(sizeimage);
//	} else {
//		pix_mp->plane_fmt[1].bytesperline = pix_mp->plane_fmt[0].bytesperline;
//		pix_mp->plane_fmt[1].sizeimage = pix_mp->plane_fmt[0].sizeimage;
//	}
//
//	dprintk("pixelformat: %c%c%c%c\n", fourcc2char(pix_mp->pixelformat));
//	return 0;
//}

/**
 * vidioc_g_fmt_vid_out
 * @f: return current image src informations.
 */
static int vidioc_g_fmt_vid_out(struct file *file, void *fh, struct v4l2_format *f)
{
	struct tcc_vout_device *vout = fh;

	f->fmt.pix = vout->src_pix;

	print_v4l2_pix_format(&f->fmt.pix, "vidioc_g_fmt_vid_out");
	return 0;
}

//static int vidioc_g_fmt_vid_out_mplane(struct file *file, void *fh, struct v4l2_format *f)
//{
//	struct tcc_vout_device *vout = fh;
//
//	f->fmt.pix_mp.width = vout->src_pix.width;
//	f->fmt.pix_mp.height = vout->src_pix.height;
//	f->fmt.pix_mp.pixelformat = vout->src_pix.pixelformat;
//	f->fmt.pix_mp.field = vout->src_pix.field;
//	f->fmt.pix_mp.colorspace = vout->src_pix.colorspace;
//
//	f->fmt.pix_mp.num_planes = MPLANE_NUM;
//
//	f->fmt.pix_mp.plane_fmt[MPLANE_VID].reserved[0] = MPLANE_VID;
//	f->fmt.pix_mp.plane_fmt[MPLANE_VID].sizeimage = vout->src_pix.sizeimage;
//	f->fmt.pix_mp.plane_fmt[MPLANE_VID].bytesperline = vout->src_pix.bytesperline;
//
//	f->fmt.pix_mp.plane_fmt[MPLANE_SUB].reserved[0] = MPLANE_SUB;
//	f->fmt.pix_mp.plane_fmt[MPLANE_SUB].reserved[1] = (vout->sub_pix.pixelformat & 0x0000ffff);
//	f->fmt.pix_mp.plane_fmt[MPLANE_SUB].reserved[2] = (vout->sub_pix.pixelformat & 0xffff0000) >> 16;
//	f->fmt.pix_mp.plane_fmt[MPLANE_SUB].sizeimage = vout->sub_pix.sizeimage;
//	f->fmt.pix_mp.plane_fmt[MPLANE_SUB].bytesperline = vout->sub_pix.bytesperline;
//
//	print_v4l2_pix_format_mplane(&f->fmt.pix_mp, "vidioc_g_fmt_vid_out_mplane");
//	return 0;
//}

/**
 * vidioc_g_fmt_vid_overlay
 * @f: return current panel (screen) informations.
 */
static int vidioc_g_fmt_vid_out_overlay(struct file *file, void *fh, struct v4l2_format *f)
{
	struct tcc_vout_device *vout = fh;

	/* display info - OVERLAY_GET_POSITION */
	f->fmt.win.w.left = vout->disp_rect.left;
	f->fmt.win.w.top = vout->disp_rect.top;
	f->fmt.win.w.width = vout->disp_rect.width;
	f->fmt.win.w.height = vout->disp_rect.height;

	print_v4l2_rect(&f->fmt.win.w, "vidioc_g_fmt_overlay output info");

	return 0;
}

/**
 * vidioc_s_fmt_vid_out
 * @f: src image format from application
 */
static int vidioc_s_fmt_vid_out(struct file *file, void *fh, struct v4l2_format *f)
{
	struct tcc_vout_device *vout = fh;
	struct tcc_vout_vioc *vioc = vout->vioc;
	int panel_xres, panel_yres;
	int ret = 0;

	if(f->fmt.pix.width == 0 || f->fmt.pix.height == 0) {
		printk(KERN_WARNING VOUT_NAME ": [warrning] retry %s(%dx%d)\n", __func__,f->fmt.pix.width, f->fmt.pix.height);
		return ret;
	}

	#if 0
	if (f->fmt.pix.width > 4000 || f->fmt.pix.height > 4000) {
		printk(KERN_ERR VOUT_NAME ": [error] Image resolution not supported (%dx%d)\n",
			f->fmt.pix.width, f->fmt.pix.height);
		return -EINVAL;
	}
	#endif

	#if defined(CONFIG_TCC_DISPLAY_MODE_DUAL_HDMI_CVBS) || defined(CONFIG_TCC_DISPLAY_MODE_DUAL_ALL)
	if(vioc->deintl_wmix.id == VIOC_WMIX_01) {
		VIOC_DISP_TurnOff(vioc->ext_disp.addr);
		VIOC_DISP_Wait_DisplayDone(vioc->ext_disp.addr);
		VIOC_RDMA_SetImageDisable(vioc->ext_rdma.addr);
	}
	#endif

	mutex_lock(&vout->lock);

	/* Set src image infomations */
	memcpy(&vout->src_pix, &f->fmt.pix, sizeof(struct v4l2_pix_format));

	/* init crop_src = src_pix */
	vout->crop_src.left = 0;
	vout->crop_src.top = 0;
	vout->crop_src.width = vout->src_pix.width;
	vout->crop_src.height = vout->src_pix.height;

	/* FORCE SETUP: display size = panel size */
	vout_wmix_getsize(vout, &vout->panel_rect.width, &vout->panel_rect.height);
	panel_xres = vout->panel_rect.width;
	panel_yres = vout->panel_rect.height;
	vout->disp_rect.left = 0;
	vout->disp_rect.top = 0;
	vout->disp_rect.width = panel_xres;
	vout->disp_rect.height = panel_yres;

	vout->fmt_idx = tcc_vout_try_fmt(vout->src_pix.pixelformat);
	vout->bpp = tcc_vout_try_bpp(vout->src_pix.pixelformat, &vout->src_pix.colorspace);
	vout->pfmt = tcc_vout_try_pix(f->fmt.pix.pixelformat);
	vout->previous_field = vout->src_pix.field;

	switch (vout->pfmt) {
	case TCC_PFMT_YUV422:
		vout->src_pix.sizeimage = PAGE_ALIGN(vout->src_pix.width * vout->src_pix.height * 2);
		//vout->deintl_buf_size = PAGE_ALIGN(vout->disp_rect.width * vout->disp_rect.height * 2);
		break;
	case TCC_PFMT_YUV420:
		vout->src_pix.sizeimage = PAGE_ALIGN(vout->src_pix.width * vout->src_pix.height * 3 / 2);
		//vout->deintl_buf_size = PAGE_ALIGN(vout->disp_rect.width * vout->disp_rect.height * 3 / 2);
		break;
	case TCC_PFMT_RGB:
	default:
		vout->src_pix.sizeimage = PAGE_ALIGN((vout->src_pix.width * vout->bpp) * vout->src_pix.height);
		//vout->deintl_buf_size = PAGE_ALIGN(vout->disp_rect.width * vout->bpp * vout->disp_rect.height);
		break;
	}

	/* Gst1.4 needs sizeimage */
	f->fmt.pix.sizeimage = vout->src_pix.sizeimage;

	/*
	 * CAUTION!!!: deintl_buf (deintl_wdma buf) is always YUV420 Interleaved Type0.
	 */
	vout->deintl_buf_size = PAGE_ALIGN(vout->disp_rect.width * vout->disp_rect.height * 3 / 2);

	vioc->deintl_rdma.fmt = tcc_fmtdesc[vout->fmt_idx].reserved[0];
	vioc->deintl_rdma.swap = 0;	// R-G-B
	switch (vout->src_pix.pixelformat) {
		case V4L2_PIX_FMT_YUV420:
		case V4L2_PIX_FMT_YVU420:
		case V4L2_PIX_FMT_YUV422P:
			vout->src_pix.width = ROUND_UP_4(vout->src_pix.width);
			vout->src_pix.height = ROUND_UP_2(vout->src_pix.height);
			break;
		case V4L2_PIX_FMT_RGB24:
		case V4L2_PIX_FMT_RGB32:
			vioc->deintl_rdma.swap = 5;	// B-G-R in the rdma
			break;
		default:
			vioc->deintl_rdma.swap = 0;	// R-G-B
			break;
	}

	/* FORMAT CONVERSION
	 *===================
	 * - VIQE and DEINTL_S must receive YUV format.
	 * - WMIX must receive RGB format.
	 * - Interlaced video has only YUV format.
	 *
	 * .y2r is YUV-to-RGB converter enable bit. (*** USE .y2rmd = 1)
	 * .r2y is RGB-to-YUV converter enable bit. (*** USE .r2ymd = 3)
	 *
	 * | type of input frame | deintl_rdma | deinterlacer | deintl_wdma | (disp) rdma |
	 * |---------------------|-------------|--------------|-------------|-------------|
	 * | Progressive RGB     | y2r = 0     | N/A          | r2y = 1     | y2r = 1     |
	 * | Progressive YUV     | y2r = 1     | N/A          | r2y = 1     | y2r = 1     |
	 * | Interlace YUV       | y2r = 0     | VIQE.y2r = 1 | r2y = 1     | y2r = 1     |
	 * | Interlace YUV       | y2r = 0     | DEINTL_S N/A | r2y = 0     | y2r = 1     |
	 *
	 * In case deintl_path uses VIQE,
	 * the properties of the frame can be changed between interlace and progessive.
	 * Therefore, 'deintl_rdma.y2r changing codes' existed in the vout_deintl_qbuf().
	 *
	 * Known-bug:
	 * In case of DEINTL_S + subtitle, It has a alpha mixing issue,
	 * because DEINTL_S doesn't exist y2r register.
	 */
	vioc->deintl_rdma.y2r = 0;
	vioc->deintl_rdma.y2rmd = 2;    // 2 = Studio Color
	vioc->deintl_viqe.y2r = 1;
	vioc->deintl_viqe.y2rmd = 2;    // 2 = Studio Color
	vioc->deintl_wdma.r2y = 1;
	vioc->deintl_wdma.r2ymd = 2;	// 2 = Studio Color
	vioc->rdma.y2r = 1;
	vioc->rdma.y2rmd = 2;			// 2 = Studio Color

	if (VOUT_DEINTL_S == vout->deinterlace) {
		vioc->deintl_rdma.y2r = 0;
		vioc->deintl_wdma.r2y = 0;
	}

	dprintk("rdma.y2r(%d)-viqe.y2r(%d)-wdma.r2y(%d)-rdma.y2r(%d)",
		vioc->deintl_rdma.y2r, vioc->deintl_viqe.y2r, vioc->deintl_wdma.r2y, vioc->rdma.y2r);

	/* de-interlace path setting */
	vioc->deintl_wdma.fmt = VIOC_IMG_FMT_YUV420IL0;
	vout->first_frame = 1;
	ret = vout_deintl_init(vout);
	if (ret) {
		printk(KERN_ERR VOUT_NAME ": [error] deinterlace setup\n");
		goto out_exit;
	}

	#ifdef VOUT_SUB_PLANE
	/* sub plane path setting
	 * default value: vout->src_pix, ARGB8888
	 */
	ret = vout_subplane_init(vout);
	if (ret) {
		printk(KERN_ERR VOUT_NAME ": [error] subplane setup\n");
		goto out_exit;
	}
	#endif

	/* vout path setting */
	vioc->rdma.fmt = vioc->deintl_wdma.fmt;
	vout_rdma_setup(vout);
	#ifdef VOUT_DISP_PATH_SCALER
	if (vioc->sc.id >= 0)
		vout_scaler_setup(vout);
	#endif
	vout_wmix_setup(vout);

out_exit:
	mutex_unlock(&vout->lock);

	#if defined(CONFIG_TCC_DISPLAY_MODE_DUAL_HDMI_CVBS) || defined(CONFIG_TCC_DISPLAY_MODE_DUAL_ALL)
	if(vioc->deintl_wmix.id == VIOC_WMIX_01) {
		VIOC_RDMA_SetImageEnable(vioc->ext_rdma.addr);
		VIOC_DISP_TurnOn(vioc->ext_disp.addr);
	}
	#endif

	dprintk("pixelformat: %c%c%c%c -> %s\n", fourcc2char(f->fmt.pix.pixelformat), tcc_fmtdesc[vout->fmt_idx].description);
	print_v4l2_pix_format(&vout->src_pix, "vidioc_s_fmt_vid_out user args");
	print_v4l2_rect(&vout->panel_rect, "vidioc_s_fmt_vid_out lcdc panel");
	print_v4l2_rect(&vout->disp_rect, "vidioc_s_fmt_vid_out output display");
	print_vioc_vout_path(vout, "vidioc_s_fmt_vid_out");
	return ret;
}

//static int vidioc_s_fmt_vid_out_mplane(struct file *file, void *fh, struct v4l2_format *f)
//{
//	struct tcc_vout_device *vout = fh;
//	struct v4l2_format main_f;
//	int sub_bpp, sub_pfmt;
//	int ret = 0;
//
//	if (f->fmt.pix_mp.num_planes != MPLANE_NUM) {
//		printk(KERN_ERR VOUT_NAME " error: num of mplane mismatch: %d (expected %d)\n",
//			f->fmt.pix_mp.num_planes, MPLANE_NUM);
//		return -EINVAL;
//	}
//
//	/*
//	 * main path (video src)
//	 */
//	main_f.type = f->type;
//	main_f.fmt.pix.width = f->fmt.pix_mp.width;
//	main_f.fmt.pix.height = f->fmt.pix_mp.height;
//	main_f.fmt.pix.pixelformat = f->fmt.pix_mp.pixelformat;
//	main_f.fmt.pix.field = f->fmt.pix_mp.field;
//	main_f.fmt.pix.colorspace = f->fmt.pix_mp.colorspace;
//	main_f.fmt.pix.bytesperline = f->fmt.pix_mp.plane_fmt[0].bytesperline;
//	main_f.fmt.pix.sizeimage = f->fmt.pix_mp.plane_fmt[0].sizeimage;
//
//	ret = vidioc_s_fmt_vid_out(file, fh, &main_f);
//	if (ret)
//		goto main_exit;
//
//	/*
//	 * sub path (subtitle src)
//	 */
//	mutex_lock(&vout->lock);
//
//	vout->sub_pix.width = f->fmt.pix_mp.width;
//	vout->sub_pix.height = f->fmt.pix_mp.height;
//	vout->sub_pix.pixelformat = f->fmt.pix_mp.plane_fmt[1].reserved[2];
//	vout->sub_pix.pixelformat = (vout->sub_pix.pixelformat << 16) | f->fmt.pix_mp.plane_fmt[1].reserved[1];
//	vout->sub_pix.field = V4L2_FIELD_NONE;
//
//	sub_bpp = tcc_vout_try_bpp(vout->sub_pix.pixelformat, &vout->sub_pix.colorspace);
//	sub_pfmt = tcc_vout_try_pix(vout->sub_pix.pixelformat);
//	if (sub_pfmt != TCC_PFMT_RGB) {
//		printk(KERN_ERR VOUT_NAME " error: invalid subtile pixel format\n");
//		goto sub_exit;
//	}
//
//	vout->sub_pix.bytesperline = vout->sub_pix.width * sub_bpp;
//	vout->sub_pix.sizeimage = PAGE_ALIGN(vout->sub_pix.bytesperline * vout->sub_pix.height);
//
//	ret = vout_subplane_init(vout);
//	if (ret) {
//		printk(KERN_ERR VOUT_NAME " error: subplane setup\n");
//		goto sub_exit;
//	}
//
//sub_exit:
//	mutex_unlock(&vout->lock);
//
//main_exit:
//	dprintk("pixelformat: %c%c%c%c -> %s\n", fourcc2char(f->fmt.pix.pixelformat), tcc_fmtdesc[vout->fmt_idx].description);
//	print_v4l2_pix_format(&vout->src_pix, "vidioc_s_fmt_vid_out_mplane [main]");
//	print_v4l2_pix_format(&vout->sub_pix, "vidioc_s_fmt_vid_out_mplane [sub]");
//	print_v4l2_rect(&vout->panel_rect, "vidioc_s_fmt_vid_out_mplane lcdc panel");
//	print_v4l2_rect(&vout->disp_rect, "vidioc_s_fmt_vid_out_mplane output display");
//	print_vioc_vout_path(vout, "vidioc_s_fmt_vid_out_mplane [main]");
//	dprintk("sub-plane RDMA%d\n", vout->vioc->sub_rdma.id);
//	return ret;
//}

/**
 * vidioc_s_fmt_vid_overlay
 * @f: modified src image format by scaling or crop.
 */
static int vidioc_s_fmt_vid_out_overlay(struct file *file, void *fh, struct v4l2_format *f)
{
	struct tcc_vout_device *vout = fh;
	int panel_xres, panel_yres;

	mutex_lock(&vout->lock);

	/* 2-pixel alignment */
	f->fmt.win.w.left = ROUND_DOWN_2(f->fmt.win.w.left);
	f->fmt.win.w.top = ROUND_DOWN_2(f->fmt.win.w.top);
	f->fmt.win.w.width = ROUND_DOWN_2(f->fmt.win.w.width);
	f->fmt.win.w.height = ROUND_DOWN_2(f->fmt.win.w.height);

	if (vout->disp_rect.left == f->fmt.win.w.left &&
		vout->disp_rect.top == f->fmt.win.w.top &&
		vout->disp_rect.width == f->fmt.win.w.width &&
		vout->disp_rect.height == f->fmt.win.w.height) {
		dprintk("Nothing to do because setting the same size\n");
		goto overlay_exit;
	}

	memcpy(&vout->disp_rect, &f->fmt.win.w, sizeof(struct v4l2_rect));
	memcpy(&vout->crop_rect, &f->fmt.win.w, sizeof(struct v4l2_rect));
	vout_wmix_getsize(vout, &vout->panel_rect.width, &vout->panel_rect.height);
	panel_xres = vout->panel_rect.width;
	panel_yres = vout->panel_rect.height;

	/* The image fits in the panel by cropping.
	 */
	if (vout->disp_rect.left < 0) {
		vout->crop_rect.left = 0;
		vout->crop_rect.width = vout->disp_rect.width + vout->disp_rect.left;
	}
	if (vout->disp_rect.top < 0) {
		vout->crop_rect.top = 0;
		vout->crop_rect.height = vout->disp_rect.height + vout->disp_rect.top;
	}
	if (panel_xres < vout->disp_rect.left + vout->disp_rect.width) {
		vout->crop_rect.width = panel_xres - vout->disp_rect.left;
		vout->disp_rect.width = vout->crop_rect.width;					//And_TODO
	}
	if (panel_yres < vout->disp_rect.top + vout->disp_rect.height) {
		vout->crop_rect.height = panel_yres - vout->disp_rect.top;
		vout->disp_rect.height = vout->crop_rect.height;				//And_TODO
	}
	vout->crop_rect.width = ROUND_DOWN_2(vout->crop_rect.width);
	vout->crop_rect.height = ROUND_DOWN_2(vout->crop_rect.height);
	if (vout->crop_rect.width <= 0 || vout->crop_rect.height <= 0) {
		vout->status = TCC_VOUT_STOP;
		goto overlay_exit;
	} else {
		vout->status = TCC_VOUT_RUNNING;
	}
	vout_deintl_overlay(vout);

	#ifdef VOUT_DISP_PATH_SCALER
	if (vout->vioc->sc.id >= 0) {
		/* The image fits in the panel by scaling.
		 */
		if (vout->disp_rect.left + vout->disp_rect.width > panel_xres) {
			vout->disp_rect.width = panel_xres - vout->disp_rect.left;
		}
		if (vout->disp_rect.top + vout->disp_rect.height > panel_yres) {
			vout->disp_rect.height = panel_yres - vout->disp_rect.top;
		}
		vout_scaler_setup(vout);
		vout_wmix_setup(vout);
	}
	#endif

overlay_exit:
	mutex_unlock(&vout->lock);

	dprintk("\n");
	print_v4l2_rect(&vout->panel_rect, "vidioc_s_fmt_overlay: panel_rect");
	print_v4l2_rect(&f->fmt.win.w, "vidioc_s_fmt_overlay: user args");
	print_v4l2_rect(&vout->disp_rect, "vidioc_s_fmt_overlay: disp_rect");
	print_v4l2_rect(&vout->crop_rect, "vidioc_s_fmt_overlay: crop_rect");
	return 0;
}

static int vidioc_cropcap(struct file *file, void *fh, struct v4l2_cropcap *cropcap)
{
	struct tcc_vout_device *vout = fh;

	if (cropcap->type != V4L2_BUF_TYPE_VIDEO_OUTPUT)
		return -EINVAL;

	/* We crop scaled image.
	 * input image -> scaling -> cropping -> display
	 */
	cropcap->bounds.width = vout->disp_rect.width;
	cropcap->bounds.height = vout->disp_rect.height;
	cropcap->defrect.width = vout->disp_rect.width;
	cropcap->defrect.height = vout->disp_rect.height;
	cropcap->defrect.left = 0;
	cropcap->defrect.top = 0;
	cropcap->pixelaspect.numerator = 1;
	cropcap->pixelaspect.denominator = 1;

	dprintk("%d/%d\n", cropcap->pixelaspect.numerator, cropcap->pixelaspect.denominator);
	print_v4l2_rect(&cropcap->bounds, "vidioc_cropcap - bounds");
	print_v4l2_rect(&cropcap->defrect, "vidioc_cropcap - defrect");
	return 0;
}

static int vidioc_g_crop(struct file *file, void *fh, struct v4l2_crop *crop)
{
	struct tcc_vout_device *vout = fh;

	dprintk("\n");

	if (crop->type != V4L2_BUF_TYPE_VIDEO_OUTPUT)
		return -EINVAL;
	crop->c = vout->crop_rect;
	return 0;
}

static int vidioc_s_crop(struct file *file, void *fh, const struct v4l2_crop *crop)
{
	int ret = -EINVAL;
	struct tcc_vout_device *vout = fh;

	dprintk("\n");

	mutex_lock(&vout->lock);

	if (((crop->c.left) < 0 || (crop->c.left >= vout->disp_rect.left))
		&& ((crop->c.top) < 0 || (crop->c.top >= vout->disp_rect.top))
		&& ((crop->c.width <= 0) || ((crop->c.left + crop->c.width) >= vout->disp_rect.width))
		&& ((crop->c.height <= 0) || ((crop->c.top + crop->c.height) >= vout->disp_rect.height)))
	{
		printk(KERN_ERR VOUT_NAME ": [error] Invalid crop area (%d,%d)~(%dx%d)\n",
			crop->c.left, crop->c.top, crop->c.width, crop->c.height);
		goto s_crop_exit;
	}

	/* TODO: cropping codes */

s_crop_exit:
	mutex_unlock(&vout->lock);

	print_v4l2_rect(&crop->c, "vidioc_s_crop - user args");
	return ret;
}

static int vidioc_reqbufs(struct file *file, void *fh, struct v4l2_requestbuffers *req)
{
	struct tcc_vout_device *vout = fh;
	unsigned int max_frame = VIDEO_MAX_FRAME - vout->nr_qbufs;
	int i, ret = 0;

	dprintk("\n");

	/* free all buffers  */
	if(req->count == 0)
	{
		dprintk("free all buffers(%d) -> 0 \n", vout->nr_qbufs);
		if(vout->nr_qbufs) {
			for(i = 0; i < vout->nr_qbufs; i++)
				kfree_safe(vout->qbufs[i].buf.m.planes);
		}
		vout->nr_qbufs = 0;
		memset(vout->qbufs, 0x0, sizeof(struct tcc_v4l2_buffer) * VIDEO_MAX_FRAME);

		/* update buffer status */
		vout->mapped = OFF;
		goto reqbufs_exit;
	}

	if(!vout->mapped) {
		if (unlikely(V4L2_BUF_TYPE_VIDEO_OUTPUT != req->type)) {
			printk(KERN_ERR VOUT_NAME ": [error] Invalid buf type(%d)\n", req->type);
			return -EINVAL;
		}

		mutex_lock(&vout->lock);

		vout->memory = vout->force_userptr ? V4L2_MEMORY_USERPTR : req->memory;

		if (V4L2_MEMORY_MMAP == vout->memory) {
			unsigned int actual_bufs;

			/* get pmap */
			ret = vout_get_pmap(&vout->pmap);
			if (ret) {
				printk(KERN_ERR VOUT_NAME ": [error] vout_get_pmap(%s)\n", vout->pmap.name);
				ret = -ENOMEM;
				goto reqbufs_exit_err;
			}

			/* calculate buf count */
			actual_bufs = vout->pmap.size / vout->src_pix.sizeimage;
			dprintk("V4L2_MEMORY_MMAP: request buf(%d), actual buf(%d)\n", req->count, actual_bufs);
			if ((req->count > actual_bufs) || (req->count < 0))
				req->count = actual_bufs;
		} else if (V4L2_MEMORY_USERPTR == vout->memory) {
			dprintk("V4L2_MEMORY_USERPTR: request buf(%d)\n", req->count);
		} else {
			printk(KERN_ERR VOUT_NAME ": [error] Invalid memory type(%d)\n", req->memory);
			ret = -EINVAL;
			goto reqbufs_exit_err;
		}

		if (max_frame < req->count) {
			req->count = max_frame;
		}

		ret = tcc_v4l2_buffer_set(vout, req);
		if (ret < 0) {
			printk(KERN_ERR VOUT_NAME ": [error] tcc_v4l2_buffer_set(%d)\n", ret);
			goto reqbufs_exit;
		}

		vout->deintl_nr_bufs_count = 0;
		ret = tcc_deintl_buffer_set(vout);
		if (ret < 0) {
			printk(KERN_ERR VOUT_NAME ": [error] tcc_deintl_buffer_set(%d)\n", ret);
			goto reqbufs_deintl_err;
		}

		mutex_unlock(&vout->lock);

		/* update buffer status */
		vout->mapped = ON;

		print_v4l2_pix_format(&vout->src_pix, "vidioc_reqbufs src info");
		print_v4l2_buf_type(req->type, "vidioc_reqbufs");
		print_v4l2_memory(req->memory, "vidioc_reqbufs");
		return ret;

reqbufs_deintl_err:
		/* free tcc_v4l2_buffer_set */
		#ifdef VOUT_SUB_PLANE
		for (i = 0; i < vout->nr_qbufs; i++)
			kfree(vout->qbufs[i].buf.m.planes);
		#endif
		kfree(vout->qbufs);
reqbufs_exit_err:
		mutex_unlock(&vout->lock);
	} else {
		printk(KERN_WARNING VOUT_NAME": [warning] any buffers are still mapped(%d)\n", vout->nr_qbufs);
		req->count = vout->nr_qbufs;
	}
reqbufs_exit:
	return ret;
}

static int vidioc_create_bufs(struct file *file, void *fh, struct v4l2_create_buffers *create)
{
	struct tcc_vout_device *vout = fh;
	unsigned int max_frame = VIDEO_MAX_FRAME - vout->nr_qbufs;
	int ret = 0;

	/* check the validity of format.type field */
	if(unlikely(V4L2_BUF_TYPE_VIDEO_OUTPUT != create->format.type)) {
		printk(KERN_ERR VOUT_NAME ": [error] Invalid buf type(%d)\n", create->format.type);
		ret = -EINVAL;
		goto create_bufs_exit;
	}

	/* check the validity of memory field */
	if (V4L2_MEMORY_MMAP == create->memory) {
		unsigned int actual_bufs;

		/* get pmap */
		ret = vout_get_pmap(&vout->pmap);
		if (ret) {
			printk(KERN_ERR VOUT_NAME ": [error] vout_get_pmap(%s)\n", vout->pmap.name);
			ret = -ENOMEM;
			goto create_bufs_exit;
		}

		/* calculate buf count */
		actual_bufs = vout->pmap.size / vout->src_pix.sizeimage;
		dprintk("V4L2_MEMORY_MMAP: create buf(%d), actual buf(%d)\n", create->count, actual_bufs);
		if ((create->count > actual_bufs) || (create->count < 0))
			create->count = actual_bufs;
	} else if (V4L2_MEMORY_USERPTR == create->memory) {
		dprintk("V4L2_MEMORY_USERPTR: create buf(%d)\n", create->count);
	} else {
		printk(KERN_ERR VOUT_NAME ": [error] Invalid memory type(%d)\n", create->memory);
		ret = -EINVAL;
		goto create_bufs_exit;
	}

	if(create->count == 0) {
		dprintk(" current number of created buffers(%d)\n", vout->nr_qbufs);
		create->index = vout->nr_qbufs;
		goto create_bufs_exit;
	}

	mutex_lock(&vout->lock);

	if (max_frame < create->count) {
		create->count = max_frame;
	}

	ret = tcc_v4l2_buffer_add(vout, create);
	if(ret < 0 ) {
		printk(KERN_ERR VOUT_NAME ": [error] tcc_v4l2_buffer_add(%d)\n", ret);
		goto create_bufs_exit_err;
	}

	mutex_unlock(&vout->lock);

	dprintk("buffer cout %d\n", create->count);

	print_v4l2_pix_format(&vout->src_pix, "vidioc_create_bufs src info");
	print_v4l2_buf_type(create->format.type, "vidioc_create_bufs");
	print_v4l2_memory(create->memory, "vidioc_create_bufs");

create_bufs_exit_err:
	mutex_unlock(&vout->lock);
create_bufs_exit:
	return ret;
}

static int vidioc_querybuf(struct file *file, void *fh, struct v4l2_buffer *buf)
{
	struct tcc_vout_device *vout = fh;
	struct tcc_v4l2_buffer *qbuf = vout->qbufs + buf->index;
	dprintk("buffer index(%d) type(%d) memory(%d)\n", buf->index, buf->type, buf->memory);

	/* input (from v4l2sink) */
	qbuf->buf.index = buf->index;
	qbuf->buf.type = buf->type;
	qbuf->buf.memory = buf->memory;

	/* output (to v4l2sink) */
	buf->flags = qbuf->buf.flags = V4L2_BUF_FLAG_MAPPED;

	#ifdef VOUT_SUB_PLANE
	qbuf->buf.m.planes[MPLANE_VID].m.mem_offset = qbuf->img_base0;
	qbuf->buf.m.planes[MPLANE_VID].length = vout->src_pix.sizeimage;
	buf->m.planes[MPLANE_VID].m.mem_offset = qbuf->img_base0;	// for v4l2_mmap()
	buf->m.planes[MPLANE_VID].length = vout->src_pix.sizeimage;	// for v4l2_mmap()
	buf->length = qbuf->buf.length = vout->src_pix.sizeimage;
	#else
	buf->m.offset = qbuf->buf.m.offset = vout->qbufs[buf->index].img_base0;
	buf->length = qbuf->buf.length = vout->src_pix.sizeimage;
	#endif

	return 0;
}

static int vidioc_qbuf(struct file *file, void *fh, struct v4l2_buffer *buf)
{
	struct tcc_vout_device *vout = fh;

	if(readable_bufs == MAX_DISPBUF_NUM) {
		printk(KERN_ERR VOUT_NAME": buffer full \n");
		return -EIO;
	}

	mutex_lock(&vout->lock);
	/* buffer handling */
	buf->flags |= V4L2_BUF_FLAG_QUEUED;
	buf->flags &= ~V4L2_BUF_FLAG_DONE;

	if(++pushIdx >= MAX_DISPBUF_NUM)
		pushIdx = 0;

	tcc_vout_buffer_copy(&displaybuffer[pushIdx], buf);
	readable_bufs++;

	dbprintk("q(%d %d)(%d %d)(%ldmsec)\n", pushIdx, readable_bufs, buf->index, buf->reserved2, (buf->timestamp.tv_sec*1000)+(buf->timestamp.tv_usec/1000));

	#ifndef CONFIG_VOUT_USE_VSYNC_INT
	vout->onthefly_mode = OFF;
	vout_m2m_display_update(vout, &displaybuffer[popIdx]);
	if(++popIdx >= MAX_DISPBUF_NUM)
		popIdx = 0;
	readable_bufs--;
	#endif
	
	mutex_unlock(&vout->lock);

	return 0;
}

static int vidioc_dqbuf(struct file *file, void *fh, struct v4l2_buffer *buf)
{
	struct tcc_vout_device *vout = fh;

	if(displayed_bufs == 0) {
		//printk(KERN_ERR VOUT_NAME": empty displayed_bufs(%d) \n", displayed_bufs);
		return -EIO;
	}

	#ifdef CONFIG_VOUT_USE_VSYNC_INT
	tcc_vout_buffer_copy(buf, &displaybuffer[clearIdx]);
	#else
	buf->index = displaybuffer[clearIdx].index;
	buf->sequence = displaybuffer[clearIdx].sequence;
	#endif

	if(vout->onthefly_mode) {
		if(vout->clearFrameMode)
			goto force_dqbuf;

		// check buffer info
		if(buf->index == vout->last_displayed_buf_idx)
			return -EIO;
	}
force_dqbuf:
	mutex_lock(&vout->lock);
	dbprintk("dq(%d %d)(%d %d)\n", clearIdx, displayed_bufs, buf->index, buf->reserved2);

	if(++clearIdx >= MAX_DISPBUF_NUM)
		clearIdx = 0;
	displayed_bufs--;

	/* buffer handling */
	buf->flags &= ~V4L2_BUF_FLAG_QUEUED;
	buf->flags &= ~V4L2_BUF_FLAG_DONE;
	if(displayed_bufs > 0) {
		buf->reserved = true;
	} else {
		buf->reserved = false;
		if(vout->clearFrameMode) {
			vout->clearFrameMode = 0;	// disable celar frame mode
			popIdx = clearIdx = 0;
			pushIdx = -1;
		}
	}
	mutex_unlock(&vout->lock);
	return 0;
}

static int vidioc_streamon(struct file *file, void *fh, enum v4l2_buf_type i)
{
	struct tcc_vout_device *vout = fh;
	struct tcc_vout_vioc *vioc = vout->vioc;

	mutex_lock(&vout->lock);

	if (V4L2_BUF_TYPE_VIDEO_OUTPUT != i) {
		printk(KERN_ERR VOUT_NAME ": [error] invalid v4l2_buf_type(%d)\n", i);
		return -EINVAL;
	}

	/* If status is TCC_VOUT_STOP then we have to enable */
	if (vout->status == TCC_VOUT_STOP) {
		vout_disp_ctrl(vioc, 1);		// enable disp_path
		vout_deintl_ctrl(vioc, 1);		// enable deintl_path
		#ifdef VOUT_SUB_PLANE
		vout_subplane_ctrl(vout, 1);	// enable subplane_path
		#endif
	}

	vout->status = TCC_VOUT_RUNNING;

	mutex_unlock(&vout->lock);
	dprintk("\n");
	return 0;
}

static int vidioc_streamoff(struct file *file, void *fh, enum v4l2_buf_type i)
{
	struct tcc_vout_device *vout = fh;
	struct tcc_vout_vioc *vioc = vout->vioc;

	mutex_lock(&vout->lock);

	if (V4L2_BUF_TYPE_VIDEO_OUTPUT != i) {
		printk(KERN_ERR VOUT_NAME ": [error] invalid v4l2_buf_type(%d)\n", i);
		return -EINVAL;
	}

	vout->status = TCC_VOUT_STOP;

	vout_disp_ctrl(vioc, 0);		// disable disp_path
	vout_deintl_ctrl(vioc, 0);		// disable deintl_path
	#ifdef VOUT_SUB_PLANE
	vout_subplane_ctrl(vout, 0);	// disalbe subplane_path
	#endif

	mutex_unlock(&vout->lock);
	dprintk("\n");
	return 0;
}

static long vout_param_handler(struct file *file, void *fh, bool valid_prio, unsigned int cmd, void *arg)
{
	struct tcc_vout_device *vout = fh;
	int ret = 0;

	if (vout->status != TCC_VOUT_RUNNING) {
		/* only allowed if streaming is started */
		printk(KERN_ERR VOUT_NAME": device not started yet(%d)\n", vout->status);
		return -EBUSY;
	}

	mutex_lock(&vout->lock);

	switch (cmd) {
	case VIDIOC_USER_CLEAR_FRAME:
		#ifdef CONFIG_VOUT_USE_VSYNC_INT
		if(!vout->first_frame) {
			dprintk("Flushing all buffers [R:%d D:%d] \n", readable_bufs, displayed_bufs);
			vout_pop_all_buffer(vout);
		}
		#endif
		break;
	default:
		ret = -ENOTTY;
	}

	mutex_unlock(&vout->lock);
	dprintk("\n");

	return ret;
}

/* 
 * File operations
 */
extern int range_is_allowed(unsigned long pfn, unsigned long size);
static int tcc_vout_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct video_device *vdev = video_devdata(file);
	struct tcc_vout_device *vout = video_get_drvdata(vdev);

	if (V4L2_MEMORY_USERPTR == vout->memory) {
		vma->vm_pgoff = 0;
		goto exit;
	}

	if (range_is_allowed(vma->vm_pgoff, vma->vm_end - vma->vm_start) < 0) {
		printk(KERN_ERR VOUT_NAME ": [error] range_is_allowed() failed\n");
		return -EAGAIN;
	}

	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	if (remap_pfn_range(vma,vma->vm_start, vma->vm_pgoff , vma->vm_end - vma->vm_start, vma->vm_page_prot)) {
		printk(KERN_ERR VOUT_NAME ": [error] remap_pfn_range() failed\n");
		return -EAGAIN;
	}

exit:
	vma->vm_ops = NULL;
	vma->vm_flags |= (VM_IO | VM_DONTEXPAND | VM_DONTDUMP);

	return 0;
}

static void tcc_vout_setup_video_data(struct tcc_vout_device *vout)
{
	struct lcd_panel *panel = tccfb_get_panel();    /* get panel info */
	vout->status = TCC_VOUT_IDLE;
	
	/* lcdc panel area */
	vout->panel_rect.left = 0;
	vout->panel_rect.top = 0;
	vout->panel_rect.width = panel->xres;
	vout->panel_rect.height = panel->yres;

	/* display output area */
	vout->disp_rect.left = 0;
	vout->disp_rect.top = 0;
	vout->disp_rect.width = panel->xres;
	vout->disp_rect.height = panel->yres;

	/* TCC solution uses only V4L2_MEMORY_USERPTR by gstreamer */
	vout->force_userptr = 0;
	vout->deintl_force = 0;
}

static int tcc_vout_open(struct file *file)
{
	struct video_device *vdev = video_devdata(file);
	struct tcc_vout_device *vout = video_get_drvdata(vdev);
	struct tcc_vout_vioc *vioc = vout->vioc;
	int ret = 0;

	if (vout == NULL) {
		printk(KERN_ERR VOUT_NAME ": [error] open failed(nodev)\n");
		return -ENODEV;
	}
	if (vout->opened) {
		printk(KERN_ERR VOUT_NAME ": [error] tcc_vout only supports single open.\n");
		return -EBUSY;
	}

	vout->qbufs = NULL;
	vout->deintl_bufs = NULL;

	file->private_data = vout;
	clk_prepare_enable(vioc->vout_clk);
	vout->status = TCC_VOUT_INITIALISING;

	/* Telechips specific video data
	 */
	tcc_vout_setup_video_data(vout);

	#if defined(CONFIG_TCC_VIOCMG)
	vout->viqe_to_deints = 0;
	#endif
		
	/*
	 * Initialize vioc
	 */
	ret = vout_vioc_init(vout);
	if (ret < 0) {
		printk(KERN_ERR VOUT_NAME ": [error] tcc_vout_open failed\n");
		vout->status = TCC_VOUT_IDLE;
		return ret;
	}

	init_waitqueue_head(&vout->frame_wait);
	vioc->deintl_wdma.irq_enable = 0;

	vout->opened++;
	dprintk("\n");
	return ret;
}

static int tcc_vout_release(struct file *file)
{
	struct video_device *vdev = video_devdata(file);
	struct tcc_vout_device *vout = video_get_drvdata(vdev);
	struct tcc_vout_vioc *vioc = vout->vioc;
	int index;

	vout_deinit(vout);				// deinit disp_path
	vout_deintl_deinit(vout);		// deinit deintl_path
	#ifdef VOUT_SUB_PLANE
	vout_subplane_deinit(vout);		// deinit subplane_path
	#endif

	/* we need to re-initialize becase gst_v4l2_object_set_format() */
	memset(&vout->src_pix, 0, sizeof(struct v4l2_pix_format));

	memset(displaybuffer, 0, sizeof(struct v4l2_buffer) * MAX_DISPBUF_NUM);
	popIdx = clearIdx = displayed_bufs = readable_bufs = 0;
	pushIdx = -1;

	for(index = 0; index < MAX_DISPBUF_NUM; index++)
		kfree_safe(displaybuffer[index].m.planes);

	clk_disable_unprepare(vioc->vout_clk);
	#ifdef VOUT_SUB_PLANE
	if (vout->qbufs) {
		for (index = 0; index < vout->nr_qbufs; index++)
			kfree_safe(vout->qbufs[index].buf.m.planes);
	}
	#endif
	kfree_safe(vout->qbufs);
	kfree_safe(vout->deintl_bufs);

	/* deinit buffer info */
	vout->nr_qbufs = 0;
	vout->mapped = OFF;

	vout->status = TCC_VOUT_IDLE;
	vout->opened--;
	dprintk("\n");
	return 0;
}


static const struct v4l2_ioctl_ops tcc_vout_ioctl_ops = {
	/* VIDIOC_QUERYCAP handler */
	.vidioc_querycap        		= vidioc_querycap,

	/* VIDIOC_ENUM_FMT handlers */
	.vidioc_enum_fmt_vid_out		= vidioc_enum_fmt_vid_out,
	//.vidioc_enum_fmt_vid_out_mplane = vidioc_enum_fmt_vid_out,

	/* VIDIOC_G_FMT handlers */
	.vidioc_g_fmt_vid_out		 	= vidioc_g_fmt_vid_out,
	.vidioc_g_fmt_vid_out_overlay	= vidioc_g_fmt_vid_out_overlay,
	//.vidioc_g_fmt_vid_out_mplane 	= vidioc_g_fmt_vid_out_mplane,

	/* VIDIOC_S_FMT handlers */
	.vidioc_s_fmt_vid_out		 	= vidioc_s_fmt_vid_out,
	.vidioc_s_fmt_vid_out_overlay	= vidioc_s_fmt_vid_out_overlay,
	//.vidioc_s_fmt_vid_out_mplane 	= vidioc_s_fmt_vid_out_mplane,

	/* VIDIOC_TRY_FMT handlers */
	.vidioc_try_fmt_vid_out			= vidioc_try_fmt_vid_out,
	//.vidioc_try_fmt_vid_out_mplane 	= vidioc_try_fmt_vid_out_mplane,

	/* Buffer handlers */
	.vidioc_reqbufs					= vidioc_reqbufs,
	.vidioc_create_bufs				= vidioc_create_bufs,
	.vidioc_querybuf				= vidioc_querybuf,
	.vidioc_qbuf					= vidioc_qbuf,
	.vidioc_dqbuf					= vidioc_dqbuf,

	/* Stream on/off */
	.vidioc_streamon				= vidioc_streamon,
	.vidioc_streamoff				= vidioc_streamoff,

	/* Standard handling (ENUMSTD is handled by videodev.c) */
	//.vidioc_s_std					= vidioc_s_std,
	//.vidioc_g_std					= vidioc_g_std,

	/* Input handling */
	.vidioc_enum_input				= vidioc_enum_input,

	/* Output handling */
	//.vidioc_enum_output			= vidioc_enum_output,
	//.vidioc_g_output				= vidioc_g_output,
	//.vidioc_s_output				= vidioc_s_output,

	/* Control handling */
	.vidioc_queryctrl    			= vidioc_queryctrl,
	.vidioc_g_ctrl       			= vidioc_g_ctrl,
	.vidioc_s_ctrl       			= vidioc_s_ctrl,

	/* Crop ioctls */
	.vidioc_cropcap					= vidioc_cropcap,
	.vidioc_g_crop					= vidioc_g_crop,
	.vidioc_s_crop					= vidioc_s_crop,

	/* Stream type-dependent parameter ioctls */
	//.vidioc_g_parm				= vidioc_g_parm,
	//.vidioc_s_parm				= vidioc_s_parm,

	/* Debugging ioctls */
	//.vidioc_enum_framesizes		= vidioc_enum_framesizes,
	.vidioc_default					= vout_param_handler,
};

static const struct v4l2_file_operations tcc_vout_fops = {
	.owner			= THIS_MODULE,
	.open			= tcc_vout_open,
	.release		= tcc_vout_release,
	.unlocked_ioctl	= video_ioctl2,
	.mmap			= tcc_vout_mmap,
};

/* 
 * platfome driver functions
 */
static int tcc_vout_v4l2_probe(struct platform_device *pdev)
{
	struct video_device *vdev = NULL;
	struct tcc_vout_device *vout = NULL;
	struct tcc_vout_vioc *vioc = NULL;
	
	int ret;

	vout = kzalloc(sizeof(struct tcc_vout_device), GFP_KERNEL);
	if (!vout)
		return -ENOMEM;

	vioc = kzalloc(sizeof(struct tcc_vout_vioc), GFP_KERNEL);
	if (!vioc)
		return -ENOMEM;

	if(pdev->dev.of_node) {
		vout->v4l2_np = pdev->dev.of_node;
	} else {
		printk(KERN_ERR VOUT_NAME ": [error] could not find vout driver node. \n");
		ret = -ENODEV;
		goto probe_err0;
	}

	if(v4l2_device_register(&pdev->dev, &vout->v4l2_dev) < 0){
		printk(KERN_ERR VOUT_NAME ": [error] v4l2 register failed\n");
		ret = -ENODEV;
		goto probe_err0;
	}


	/* Register the Video device with V4L2
	 */
	vdev = vout->vdev = video_device_alloc();
	if(!vdev) {
		printk(KERN_ERR VOUT_NAME": [error] could not allocate video device struct \n");
		ret = -ENODEV;
		goto probe_err1;
	}

	vdev->release = video_device_release;
	vdev->ioctl_ops = &tcc_vout_ioctl_ops;

	strlcpy(vdev->name, VOUT_NAME, sizeof(vdev->name));

	vdev->fops = &tcc_vout_fops;
	vdev->v4l2_dev = &vout->v4l2_dev;
	vdev->vfl_dir = VFL_DIR_TX;
	mutex_init(&vout->lock);
	
	//vdev->minor = -1;
	vdev->minor = 10;

	vout->vioc = vioc;
	pdev->dev.platform_data = vout;

	/* default vioc path
	 */
	ret = vout_vioc_set_default(vout);
	if (ret < 0) {
		printk(KERN_ERR VOUT_NAME ": [error] invalid vout vioc path\n");
		goto probe_err1;
	}

	/* Register the Video device width V4L2
	 */
	if(video_register_device(vdev, VFL_TYPE_GRABBER, vdev->minor) < 0){
		printk(KERN_ERR VOUT_NAME ": [error] Counld not register Video for Linux device\n");
		ret = -ENODEV;
		goto probe_err2;
	}
	video_set_drvdata(vdev, vout);

	/* get vout clock 
	 */
	vioc->vout_clk = of_clk_get(vout->v4l2_np, 0);

	tcc_vout_attr_create(pdev);

	g_pvout = vout;

	dprintk(": registered and initialized video device %d\n", vdev->minor);
	return 0;

probe_err2:
	video_unregister_device(vdev);
	video_device_release(vdev);
probe_err1:
	v4l2_device_unregister(&vout->v4l2_dev);
probe_err0:
	kfree(vout);
	return ret;
}

static int tcc_vout_v4l2_remove(struct platform_device *pdev)
{
	tcc_vout_attr_remove(pdev);

	dprintk("\n");
	return 0;
}

//static int tcc_vout_v4l2_suspend(struct platform_device *pdev, pm_message_t state)
//{
//	dprintk("\n");
//	return 0;
//}

//static int tcc_vout_v4l2_resume(struct platform_device *pdev)
//{
//	dprintk("\n");
//	return 0;
//}

static struct platform_driver tcc_vout_v4l2_pdrv0 = {
	.driver = {
		.name	= VOUT_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(tccvout_0_of_match),
	},
	.probe		= tcc_vout_v4l2_probe,
	.remove 	= tcc_vout_v4l2_remove,
	//.suspend	= tcc_vout_v4l2_suspend,
	//.resume	= tcc_vout_v4l2_resume,
};

static int __init tcc_vout_v4l2_init(void)
{
	dprintk("\n");
	return 0;
}

static void __exit tcc_vout_v4l2_exit(void)
{
	dprintk("\n");
}

module_init(tcc_vout_v4l2_init);
module_exit(tcc_vout_v4l2_exit);
module_platform_driver(tcc_vout_v4l2_pdrv0);
MODULE_AUTHOR("Telechips Inc.");
MODULE_DESCRIPTION("VIOC VOUT driver");
MODULE_VERSION("v1.0");
MODULE_LICENSE("GPL");
