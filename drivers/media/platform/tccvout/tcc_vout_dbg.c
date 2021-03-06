/*
 * tcc_vout_dbg.c
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
#include <linux/videodev2.h>

#include "tcc_vout.h"
#include "tcc_vout_core.h"
#include "tcc_vout_dbg.h"

#ifdef CONFIG_TCC_VOUT_DEBUG
const char *name_v4l2_colorspace[] ={
	/* enum v4l2_colorspace */
	/* ITU-R 601 -- broadcast NTSC/PAL */
	[1] = "V4L2_COLORSPACE_SMPTE170M",

	/* 1125-Line (US) HDTV */
	[2] = "V4L2_COLORSPACE_SMPTE240M",

	/* HD and modern captures. */
	[3] = "V4L2_COLORSPACE_REC709",

	/* broken BT878 extents (601, luma range 16-253 instead of 16-235) */
	[4] = "V4L2_COLORSPACE_BT878",

	/* These should be useful.  Assume 601 extents. */
	[5] = "V4L2_COLORSPACE_470_SYSTEM_M",
	[6] = "V4L2_COLORSPACE_470_SYSTEM_BG",

	/* I know there will be cameras that send this.  So, this is
	 * unspecified chromaticities and full 0-255 on each of the
	 * Y'CbCr components
	 */
	[7] = "V4L2_COLORSPACE_JPEG",

	/* For RGB colourspaces, this is probably a good start. */
	[8] = "V4L2_COLORSPACE_SRGB",
};

const char *name_v4l2_field[] = {
	/* enum v4l2_field */
	[0] = "V4L2_FIELD_ANY",				/* driver can choose from none, top, bottom, interlaced
										   depending on whatever it thinks is approximate ... */
	[1] = "V4L2_FIELD_NONE",			/* this device has no fields ... */
	[2] = "V4L2_FIELD_TOP",				/* top field only */
	[3] = "V4L2_FIELD_BOTTOM",			/* bottom field only */
	[4] = "V4L2_FIELD_INTERLACED",		/* both fields interlaced */
	[5] = "V4L2_FIELD_SEQ_TB",			/* both fields sequential into one buffer, top-bottom order */
	[6] = "V4L2_FIELD_SEQ_BT",			/* same as above + bottom-top order */
	[7] = "V4L2_FIELD_ALTERNATE",		/* both fields alternating into separate buffers */
	[8] = "V4L2_FIELD_INTERLACED_TB",	/* both fields interlaced, top field
										   first and the top field is transmitted first */
	[9] = "V4L2_FIELD_INTERLACED_BT",	/* both fields interlaced, top field
										   first and the bottom field is transmitted first */
};

const char *name_v4l2_buf_type[] = {
	/* enum v4l2_buf_type */
	[1] = "V4L2_BUF_TYPE_VIDEO_CAPTURE",
	[2] = "V4L2_BUF_TYPE_VIDEO_OUTPUT",
	[3] = "V4L2_BUF_TYPE_VIDEO_OVERLAY",
	[4] = "V4L2_BUF_TYPE_VBI_CAPTURE",
	[5] = "V4L2_BUF_TYPE_VBI_OUTPUT",
	[6] = "V4L2_BUF_TYPE_SLICED_VBI_CAPTURE",
	[7] = "V4L2_BUF_TYPE_SLICED_VBI_OUTPUT",
	[8] = "V4L2_BUF_TYPE_VIDEO_OUTPUT_OVERLAY",	/* Experimental */
	[9] = "V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE",
	[10] = "V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE",
	[0x80] = "V4L2_BUF_TYPE_PRIVATE",
};

const char *name_v4l2_memory[] = {
	/* enum v4l2_memory */
	[1] = "V4L2_MEMORY_MMAP",
	[2] = "V4L2_MEMORY_USERPTR",
	[3] = "V4L2_MEMORY_OVERLAY",
};

const char *name_vioc_deintl_type[] = {
	[0] = "DEINTL_NONE",
	[1] = "VIQE_BYPASS",
	[2] = "VIQE_2D",
	[3] = "VIQE_3D",
	[4] = "DEINTL_S",
};

char *fourcc2str(unsigned int fourcc, char buf[4])
{
	buf[0] = (fourcc >>  0) & 0xFF;
	buf[1] = (fourcc >>  8) & 0xFF;
	buf[2] = (fourcc >> 16) & 0xFF;
	buf[3] = (fourcc >> 24) & 0xFF;
	return buf;
}

void print_v4l2_capability(struct v4l2_capability *cap, const char *str)
{
	printk("  [v4l2_capability] %s\n", str);
	printk("         driver: %s\n", cap->driver);
	printk("           card: %s\n", cap->card);
	printk("       bus_info: %s\n", cap->bus_info);
	printk("        version: %d\n", cap->version);
	printk("   capabilities: 0x%08x\n", cap->capabilities);
}

void print_v4l2_buf_type(enum v4l2_buf_type type, const char *str)
{
	printk("  [v4l2_buf_type] %s\n", str);
	printk("   type: %s\n", name_v4l2_buf_type[type]);
}

void print_v4l2_memory(enum v4l2_memory mem, const char *str)
{
	printk("  [v4l2_memory] %s\n", str);
	printk("   type: %s\n", name_v4l2_memory[mem]);
}

void print_v4l2_pix_format(struct v4l2_pix_format *pix, const char *str)
{
	char buf[5];
	buf[4] = 0;
	printk("[v4l2_pix_format] %s\n", str);
	printk("           width: %d\n", pix->width);
	printk("          height: %d\n", pix->height);
	printk("     pixelformat: %s\n", fourcc2str(pix->pixelformat, buf));
	printk("           field: %s\n", name_v4l2_field[pix->field]);
	printk("    bytesperline: %d\n", pix->bytesperline);
	printk("       sizeimage: %d\n", pix->sizeimage);
	printk("      colorspace: %s\n", name_v4l2_colorspace[pix->colorspace]);
	printk("            priv: %d\n", pix->priv);
}

void print_v4l2_pix_format_mplane(struct v4l2_pix_format_mplane *pix_mp, const char *str)
{
	char buf[5];
	buf[4] = 0;
	printk("  [v4l2_pix_format_mplane] %s\n", str);
	printk("                    width: %d\n", pix_mp->width);
	printk("                   height: %d\n", pix_mp->height);
	printk("              pixelformat: %s\n", fourcc2str(pix_mp->pixelformat, buf));
	printk("                    field: %s\n", name_v4l2_field[pix_mp->field]);
	printk("               colorspace: %s\n", name_v4l2_colorspace[pix_mp->colorspace]);
	printk("               num_planes: %d\n", pix_mp->num_planes);
	printk("plane_fmt[0].bytesperline: %d\n", pix_mp->plane_fmt[0].bytesperline);
	printk("plane_fmt[0].sizeimage   : %d\n", pix_mp->plane_fmt[0].sizeimage);
	printk("plane_fmt[0].reserved[0] : %d\n", pix_mp->plane_fmt[0].reserved[0]);
	printk("plane_fmt[1].bytesperline: %d\n", pix_mp->plane_fmt[1].bytesperline);
	printk("plane_fmt[1].sizeimage   : %d\n", pix_mp->plane_fmt[1].sizeimage);
	printk("plane_fmt[1].reserved[0] : %d\n", pix_mp->plane_fmt[1].reserved[0]);
	printk("plane_fmt[1].reserved[1] : %d\n", pix_mp->plane_fmt[1].reserved[1]);
	printk("plane_fmt[1].reserved[2] : %d\n", pix_mp->plane_fmt[1].reserved[2]);
}

void print_v4l2_rect(struct v4l2_rect *rect, const char *str)
{
	printk("  [v4l2_rect] %s\n", str);
	printk("     left: %d\n", rect->left);
	printk("      top: %d\n", rect->top);
	printk("    width: %d\n", rect->width);
	printk("   height: %d\n", rect->height);
}

void print_v4l2_fmtdesc(struct v4l2_fmtdesc *fmt, const char *str)
{
	char buf[5];
	buf[4] = 0;
	printk("  [v4l2_fmtdesc] %s\n", str);
	printk("                    type: %s\n", name_v4l2_buf_type[fmt->type]);
	printk("                   flags: %d\n", fmt->flags);
	printk("             description: %s\n", fmt->description);
	printk("             pixelformat: %s\n", fourcc2str(fmt->pixelformat, buf));
	printk("   reserved[0](vioc fmt): %d\n", fmt->reserved[0]);
	printk("     reserved[1](y2r en): %d\n", fmt->reserved[1]);
}

void print_v4l2_buffer(struct v4l2_buffer *buf, const char *str)
{
	printk("  [v4l2_buffer] %s\n", str);
	printk("       index: %d\n", buf->index);
	printk("        type: %s\n", name_v4l2_buf_type[buf->type]);
	printk("   bytesused: %d\n", buf->bytesused);
	printk("       flags: %d\n", buf->flags);
	printk("       field: %s\n", name_v4l2_field[buf->field]);
	printk("      memory: %s\n", name_v4l2_memory[buf->memory]);
	printk("     userptr: 0x%08lx\n", buf->m.userptr);
	printk("      length: %d\n", buf->length);
//	printk("       input: %d\n", buf->input);
	printk("    reserved: %d\n", buf->reserved);
}

void print_v4l2_reqbufs_format(enum tcc_pix_fmt pfmt, unsigned int pixelformat, const char *str)
{
	char buf[5];
	buf[4] = 0;
	printk("  [v4l2_reqbufs_format] %s\n", str);
	printk("  pixelformat: %s\n", fourcc2str(pixelformat, buf));

	switch (pfmt) {
	case TCC_PFMT_YUV422:
		printk("         pfmt: TCC_PFMT_YUV422\n");
		break;
	case TCC_PFMT_YUV420:
		printk("         pfmt: TCC_PFMT_YUV420\n");
		break;
	case TCC_PFMT_RGB:
		printk("         pfmt: TCC_PFMT_RGB\n");
		break;
	default:
		printk("         pfmt: N/A\n");
		break;
	}

	switch (pixelformat) {
		case V4L2_PIX_FMT_YUV420:	/* YCbCr 4:2:0 separated (VIOC_IMG_FMT_YUV420SEP) */
		case V4L2_PIX_FMT_YVU420:	/* YCbCr 4:2:0 separated (VIOC_IMG_FMT_YUV420SEP) */
		case V4L2_PIX_FMT_YUV422P:	/* YCbCr 4:2:2 separated (VIOC_IMG_FMT_YUV422SEP) */
			printk("         type: separated\n");
			break;
		case V4L2_PIX_FMT_NV12:		/* YCbCr 4:2:0 interleaved type0 (VIOC_IMG_FMT_YUV420IL0) */
		case V4L2_PIX_FMT_NV21:		/* YCbCr 4:2:0 interleaved type1 (VIOC_IMG_FMT_YUV420IL1) */
		case V4L2_PIX_FMT_NV16:		/* YCbCr 4:2:2 interleaved type0 (VIOC_IMG_FMT_YUV422IL0) */
		case V4L2_PIX_FMT_NV61:		/* YCbCr 4:2:2 interleaved type1 (VIOC_IMG_FMT_YUV422IL1) */
			printk("         type: interleaved\n");
			break;
		case V4L2_PIX_FMT_UYVY:		/* YCbCr 4:2:2 UYVY Sequential (VIOC_IMG_FMT_UYVY) */
		case V4L2_PIX_FMT_VYUY:		/* YCbCr 4:2:2 VYUY Sequential (VIOC_IMG_FMT_VYUY) */
		case V4L2_PIX_FMT_YUYV:		/* YCbCr 4:2:2 YUYV Sequential (VIOC_IMG_FMT_YUYV) */
		case V4L2_PIX_FMT_YVYU:		/* YCbCr 4:2:2 YVYU Sequential (VIOC_IMG_FMT_YVYU) */
			printk("         type: Sequential\n");
			break;
		default:					/* RGB and etc... */
			printk("         type: RGB or etc...\n");
	}
}

void print_vioc_vout_path(struct tcc_vout_device *vout, const char *str)
{
	struct tcc_vout_vioc *vioc = vout->vioc;
	char buf_sc[8];

	sprintf(buf_sc, " - SC%d", vioc->sc.id);

	printk("\n\e[33m[RDMA%d%s - WMIX%d - DISP%d]\e[0m %s\n", vioc->rdma.id,
		vioc->sc.id < 0 ? "" : buf_sc, vioc->wmix.id, vioc->disp.id, str);
	printk("  RDMA%d: %dx%d\n", vioc->rdma.id,
		vioc->rdma.addr->uSIZE.bREG.WIDTH, vioc->rdma.addr->uSIZE.bREG.HEIGHT);
	if (vioc->sc.id >= 0) {
		printk("  SC%d: %dx%d -> %dx%d\n", vioc->sc.id,
			vioc->rdma.width, vioc->rdma.height,
			vioc->sc.addr->uDSTSIZE.bREG.WIDTH, vioc->sc.addr->uDSTSIZE.bREG.HEIGHT);
	}
	printk("  WMIX%d: %dx%d\n", vioc->wmix.id,
		vioc->wmix.addr->uSIZE.bREG.WIDTH, vioc->wmix.addr->uSIZE.bREG.HEIGHT);
	printk("  DISP%d\n\n", vioc->disp.id);
}

void print_vioc_deintl_path(struct tcc_vout_device *vout, const char *str)
{
	struct tcc_vout_vioc *vioc = vout->vioc;

	printk("\n\e[33m[RDMA%d - %s - SC%d - WMIX%d - WDMA%d]\e[0m %s\n", vioc->deintl_rdma.id,
		name_vioc_deintl_type[vout->deinterlace], vioc->deintl_sc.id, vioc->deintl_wmix.id, vioc->deintl_wdma.id, str);
	printk("  RDMA%d: %dx%d\n", vioc->deintl_rdma.id,
		vioc->deintl_rdma.addr->uSIZE.bREG.WIDTH, vioc->deintl_rdma.addr->uSIZE.bREG.HEIGHT);
	printk("  %s\n", name_vioc_deintl_type[vout->deinterlace]);
	printk("  SC%d: %dx%d -> %dx%d\n", vioc->deintl_sc.id,
		vioc->deintl_rdma.width, vioc->deintl_rdma.height,
		vioc->deintl_sc.addr->uDSTSIZE.bREG.WIDTH, vioc->deintl_sc.addr->uDSTSIZE.bREG.HEIGHT);
	printk("  WMIX%d: %dx%d\n", vioc->deintl_wmix.id,
		vioc->deintl_wmix.addr->uSIZE.bREG.WIDTH, vioc->deintl_wmix.addr->uSIZE.bREG.HEIGHT);
	printk("  WDMA%d: %dx%d\n\n", vioc->deintl_wdma.id,
		vioc->deintl_wdma.addr->uSIZE.bREG.WIDTH, vioc->deintl_wdma.addr->uSIZE.bREG.HEIGHT);
}

void print_vioc_subplane_info(struct tcc_vout_device *vout, const char *str)
{
	struct tcc_vout_vioc *vioc = vout->vioc;
	unsigned int xpos = 0, ypos = 0;
	VIOC_WMIX_ALPHA *wmix_alpha;

	switch (vout->vioc->sub_wmix.pos) {
	case 0:
		xpos = vioc->sub_wmix.addr->uPOS0.bREG.XPOS;
		ypos = vioc->sub_wmix.addr->uPOS0.bREG.YPOS;
		break;
	case 1:
		xpos = vioc->sub_wmix.addr->uPOS1.bREG.XPOS;
		ypos = vioc->sub_wmix.addr->uPOS1.bREG.YPOS;
		break;
	case 2:
		xpos = vioc->sub_wmix.addr->uPOS2.bREG.XPOS;
		ypos = vioc->sub_wmix.addr->uPOS2.bREG.YPOS;
		break;
	case 3:
		xpos = vioc->sub_wmix.addr->uPOS3.bREG.XPOS;
		ypos = vioc->sub_wmix.addr->uPOS3.bREG.YPOS;
		break;
	}
	printk("\n\e[33m[RDMA%d - WMIX%d]\e[0m %s\n", vioc->sub_rdma.id, vioc->sub_wmix.id, str);
	printk("  RDMA%d: %dx%d\n", vioc->sub_rdma.id,
		vioc->sub_rdma.addr->uSIZE.bREG.WIDTH, vioc->sub_rdma.addr->uSIZE.bREG.HEIGHT);
	printk("  WMIX%d: (%dx%d) ~ (%dx%d)\n", vioc->sub_wmix.id, xpos, ypos,
		vioc->sub_wmix.addr->uSIZE.bREG.WIDTH, vioc->sub_wmix.addr->uSIZE.bREG.HEIGHT);

	wmix_alpha = (VIOC_WMIX_ALPHA *)((unsigned int)vioc->sub_wmix.addr + 0x40 + (0x10 * vioc->wmix_alpha.src0_layer));
	printk("  alpha base 0x%08x\n", (unsigned int)wmix_alpha);
	printk("       .region   : %d\n", vioc->wmix_alpha.region);
	printk("       .layer    : %d\n", vioc->wmix_alpha.src0_layer);
	printk("       .rop_mode : %d = %d\n", vioc->wmix_alpha.src0_rop_mode >> 3, wmix_alpha->uROPC.bREG.ROPMODE);
	printk("       .asel     : %d = %d\n", vioc->wmix_alpha.src0_asel, wmix_alpha->uROPC.bREG.ASEL);
	printk("       .acon0    : %d = %d\n", vioc->wmix_alpha.src0_acon0, wmix_alpha->uACON.bREG.ACON0_11);
	printk("       .acon1    : %d = %d\n", vioc->wmix_alpha.src0_acon1, wmix_alpha->uACON.bREG.ACON1_11);
	printk("       .ccon0    : %d = %d\n", vioc->wmix_alpha.src0_ccon0, wmix_alpha->uCCON.bREG.CCON0_11);
	printk("       .ccon1    : %d = %d\n", vioc->wmix_alpha.src0_ccon1, wmix_alpha->uCCON.bREG.CCON1_11);
	printk("       .alpha0   : %d = %d\n", vioc->wmix_alpha.src0_alpha0, wmix_alpha->uROPC.bREG.ALPHA0);
	printk("       .alpha1   : %d = %d\n", vioc->wmix_alpha.src0_alpha1, wmix_alpha->uROPC.bREG.ALPHA1);
	printk("  MACON%d (0x%08lx)\n", vioc->wmix_alpha.src0_layer, wmix_alpha->uACON.nREG);
	printk("  MCCON%d (0x%08lx)\n", vioc->wmix_alpha.src0_layer, wmix_alpha->uCCON.nREG);
}

#endif	//CONFIG_TCC_VOUT_DEBUG
