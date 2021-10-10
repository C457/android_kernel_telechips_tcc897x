/*
 * tcc_vout.h
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
#ifndef __TCC_VOUT_H__
#define __TCC_VOUT_H__

#include <linux/videodev2.h>
#include <media/v4l2-common.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>

#include <soc/tcc/pmap.h>
#ifdef CONFIG_ARCH_TCC897X
#include <mach/bsp.h>

#include <mach/tcc_fb.h>
#include <mach/vioc_global.h>
#include <mach/tccfb_ioctrl.h>
#include <mach/vioc_rdma.h>
#include <mach/vioc_wdma.h>
#include <mach/vioc_scaler.h>
#include <mach/vioc_wmix.h>
#include <mach/vioc_viqe.h>
#include <mach/vioc_config.h>
#include <mach/vioc_disp.h>
#include <mach/tcc_wmixer_ioctrl.h>
#include <mach/vioc_intr.h>

#ifdef CONFIG_ARCH_TCC_MAP_CONVERTER
#include <mach/vioc_mc.h>
#include <mach/TCC_HEVCDEC.h>
#endif

#else
#include <video/tcc/tcc_fb.h>
#include <video/tcc/vioc_global.h>
#include <video/tcc/tccfb_ioctrl.h>
#include <video/tcc/vioc_rdma.h>
#include <video/tcc/vioc_wdma.h>
#include <video/tcc/vioc_scaler.h>
#include <video/tcc/vioc_wmix.h>
#include <video/tcc/vioc_viqe.h>
#include <video/tcc/vioc_config.h>
#include <video/tcc/vioc_disp.h>
#include <video/tcc/tcc_wmixer_ioctrl.h>
#include <video/tcc/vioc_intr.h>

#ifdef CONFIG_ARCH_TCC_MAP_CONVERTER
#include <video/tcc/vioc_mc.h>
#include <video/tcc/TCC_HEVCDEC.h>
#endif

#endif

#include <video/tcc/tcc_vout_v4l2.h>

#define VOUT_NAME		"tcc-vout-video"
#define VOUT_VERSION	KERNEL_VERSION(0, 1, 0)
#define VOUT_MAX_CH		2
#define VOUT_CLK_SRC	"lcdc1"
#define VOUT_DEINTL_PATH_PMAP_NAME	"v4l2_vout"
#define VOUT_DISP_PATH_PMAP_NAME	"video"		/* using vpu pmap */

#ifdef CONFIG_VOUT_USE_VSYNC_INT
#define VOUT_DRV_ERR_DROPFRM	(-2)
#define VOUT_DRV_ERR_WAITFRM	(-1)
#define VOUT_DRV_NOERR			(0)
#endif

/* to check error state */
#define MAX_ERROR_CNT		10

/* disp_path uses 2nd scaler */
//#define VOUT_DISP_PATH_SCALER

/* subtile mplane */
#define VOUT_SUB_PLANE

/* Rounds an integer value up to the next multiple of num */
#define ROUND_UP_2(num)		(((num)+1)&~1)
#define ROUND_UP_4(num)		(((num)+3)&~3)
#define ROUND_UP_8(num)		(((num)+7)&~7)
#define ROUND_UP_16(num)	(((num)+15)&~15)
#define ROUND_UP_32(num)	(((num)+31)&~31)
#define ROUND_UP_64(num)	(((num)+63)&~63)
/* Rounds an integer value down to the next multiple of num */
#define ROUND_DOWN_2(num)	((num)&(~1))
#define ROUND_DOWN_4(num)	((num)&(~3))
#define ROUND_DOWN_8(num)	((num)&(~7))
#define ROUND_DOWN_16(num)	((num)&(~15))
#define ROUND_DOWN_32(num)	((num)&(~31))
#define ROUND_DOWN_64(num)	((num)&(~63))

/* Still-image format */
#define STILL_IMGAGE			0xFF00

/* wmix alpha */
#define WMIX_ALPHA_LAYER_0		0
#define WMIX_ALPHA_LAYER_1		1
#define WMIX_ALPHA_LAYER_2		2
#define WMIX_ALPHA_REGION_A		0
#define WMIX_ALPHA_REGION_B		1
#define WMIX_ALPHA_REGION_C		2
#define WMIX_ALPHA_REGION_D		3
#define WMIX_ALPHA_ROP_GLOBAL	0x10
#define WMIX_ALPHA_ROP_PIXEL	0x18

/* rdma alpha */
#define RDMA_ALPHA_ASEL_GLOBAL	0
#define RDMA_ALPHA_ASEL_PIXEL	1

/* mplane */
#define MPLANE_NUM	2
#define MPLANE_VID	0
#define MPLANE_SUB	1

#define MAX_DISPBUF_NUM	(12)
#ifdef CONFIG_ARCH_TCC_MAP_CONVERTER
enum mplane_hevc_component {
	/* map_converter information */
	HEVC_COMP_MAP_EN = 10,
	HEVC_FRAME_BASE_Y0 = 11,
	HEVC_FRAME_BASE_Y1 = 12,
	HEVC_FRAME_BASE_C0 = 13,
	HEVC_FRAME_BASE_C1 = 14,
	HEVC_OFFSET_BASE_Y0 = 15,
	HEVC_OFFSET_BASE_Y1 = 16,
	HEVC_OFFSET_BASE_C0 = 17,
	HEVC_OFFSET_BASE_C1 = 18,
	HEVC_LUMA_STRIDE = 19,
	HEVC_CHROMA_STRIDE =20,
	HEVC_LUMA_BIT_DEPTH	= 21,
	HEVC_CHROMA_BIT_DEPTH = 22,
	HEVC_FRMAE_ENDIAN = 23,
};

enum vioc_mapconv_list {
	VIOC_MC_00,
	VIOC_MC_01,
};
#endif

enum mplane_vid_component {
	/* video information */
	VID_SRC = 0,			// MPLANE_VID 0x0
	VID_NUM = 1,			// num of mplanes
	VID_BASE1 = 2,			// base1 address of video src (U/Cb)
	VID_BASE2 = 3,			// base1 address of video src (V/Cr)
	VID_WIDTH = 4,			// width/height of video src
	VID_HEIGHT = 5,
	VID_CROP_LEFT = 6,		// crop-[left/top/width/height] of video src
	VID_CROP_TOP = 7,
	VID_CROP_WIDTH = 8,
	VID_CROP_HEIGHT = 9,
};

enum mplane_sub_component {
	/* subtitle information */
	SUB_SRC = 0,			// MPLANE_SUB 0x1
	SUB_ON = 1,				// subtitle on(1) or off(0)
	SUB_BUF_INDEX = 2,		// index of subtitle buf
	SUB_FOURCC = 3,			// forcc format
	SUB_WIDTH = 4,			// width
	SUB_HEIGHT = 5,			// height
	SUB_OFFSET_X = 6,		// x position
	SUB_OFFSET_Y = 7,		// y position
};

enum vioc_rdma_list {
	VIOC_RDMA_00,
	VIOC_RDMA_01,
	VIOC_RDMA_02,
	VIOC_RDMA_03,
	VIOC_RDMA_04,
	VIOC_RDMA_05,
	VIOC_RDMA_06,
	VIOC_RDMA_07,
	VIOC_RDMA_08,
	VIOC_RDMA_09,
	VIOC_RDMA_10,
	VIOC_RDMA_11,
	VIOC_RDMA_12 = 12,
	VIOC_RDMA_13,
	VIOC_RDMA_14,
	VIOC_RDMA_16 = 16,
	VIOC_RDMA_17,
};

enum vioc_wmix_list {
	VIOC_WMIX_00,
	VIOC_WMIX_01,
	VIOC_WMIX_02,
	VIOC_WMIX_03,
	VIOC_WMIX_04,
	VIOC_WMIX_05,
	VIOC_WMIX_06,
};

enum vioc_sc_list {
	VIOC_SC_00,
	VIOC_SC_01,
	VIOC_SC_02,
	VIOC_SC_03,
	VIOC_SC_04,
};

enum vioc_wdma_list {
	VIOC_WDMA_00,
	VIOC_WDMA_01,
	VIOC_WDMA_02,
	VIOC_WDMA_03,
	VIOC_WDMA_04,
	VIOC_WDMA_05,
	VIOC_WDMA_06,
	VIOC_WDMA_07,
	VIOC_WDMA_08,
};

enum vioc_viqe_list {
	VIOC_VIQE_00,
	VIOC_VIQE_01,
};

enum vioc_deintls_list {
	VIOC_DEINTLS_00,
};

struct vioc_component_list {
	unsigned int phys_addr;
	unsigned int swrst_bit;
};

struct tcc_v4l2_img {
	unsigned int base0;
	unsigned int base1;
	unsigned int base2;
};

struct vioc_disp {
	int id;
	VIOC_DISP *addr;
	unsigned int irq;
	unsigned int irq_enable;			// avoid overlapping irq
	struct vioc_intr_type	*vioc_intr;
};

struct vioc_rdma {
	int id;
	unsigned int swrst_bit;
	VIOC_RDMA *addr;
	struct tcc_v4l2_img img;
	unsigned int width;
	unsigned int height;
	unsigned int fmt;
	unsigned int bf;		// BFIELD indication. 0: top, 1: bottom
	unsigned int y2r;
	unsigned int y2rmd;
	unsigned int intl;
	unsigned int swap;		// RGB Swap Register
	unsigned int y_stride;	// Y-stride
};

struct vioc_wdma {
	int id;
	unsigned int swrst_bit;
	VIOC_WDMA *addr;
	struct tcc_v4l2_img img;
	unsigned int width;
	unsigned int height;
	unsigned int fmt;
	unsigned int r2y;
	unsigned int r2ymd;
	unsigned int cont;
	unsigned int irq;
	unsigned int irq_enable;			// avoid overlapping irq
	struct vioc_intr_type	*vioc_intr;
};

struct vioc_wmix {
	int id;
	unsigned int swrst_bit;
	VIOC_WMIX *addr;
	unsigned int width;
	unsigned int height;
	unsigned int pos;					// wmix image position
	unsigned int ovp;					// wmix overlay priority
	int left;
	int top;
};

struct vioc_alpha {
	unsigned int src_type;		// v4l2_buffer.m.planes[1].reserved[0]
	unsigned int on;			// v4l2_buffer.m.planes[1].reserved[1]
	unsigned int buf_index;		// v4l2_buffer.m.planes[1].reserved[2]
	unsigned int fourcc;		// v4l2_buffer.m.planes[1].reserved[3]
	unsigned int width;			// v4l2_buffer.m.planes[1].reserved[4]
	unsigned int height;		// v4l2_buffer.m.planes[1].reserved[5]
	unsigned int offset_x;		// v4l2_buffer.m.planes[1].reserved[6]
	unsigned int offset_y;		// v4l2_buffer.m.planes[1].reserved[7]

	unsigned int reserved8;
	unsigned int reserved9;
	unsigned int reserved10;
};

struct vioc_sc {
	int id;
	unsigned int swrst_bit;
	VIOC_SC *addr;
	unsigned int ch;					// scaler channel
	unsigned int plugin;				// scaler plugin index
};

struct vioc_viqe {
	int id;
	unsigned int swrst_bit;
	VIQE *addr;
	unsigned int ch;
	unsigned int plugin;
	unsigned int y2r;
	unsigned int y2rmd;
};

struct vioc_deintls {
	int id;
	unsigned int swrst_bit;
	unsigned int ch;
	unsigned int plugin;
};

#ifdef CONFIG_ARCH_TCC_MAP_CONVERTER
struct vioc_mc {
	int id;
	VIOC_MC *addr;
	hevc_dec_MapConv_info_t mapConv_info;
};
#endif

enum tcc_pix_fmt {
	TCC_PFMT_YUV420,
	TCC_PFMT_YUV422,
	TCC_PFMT_RGB,
};

enum deintl_type {
	VOUT_DEINTL_NONE,
	VOUT_DEINTL_VIQE_BYPASS,
	VOUT_DEINTL_VIQE_2D,
	VOUT_DEINTL_VIQE_3D,
	VOUT_DEINTL_S,
};

struct tcc_v4l2_buffer {
	int index;

	struct v4l2_buffer buf;
	unsigned int img_base0;				// RDMABASE0
	unsigned int img_base1;				// RDMABASE1
	unsigned int img_base2;				// RDMABASE2
};

struct tcc_vout_vioc {
	struct clk *vout_clk;

	VIOC_IREQ_CONFIG *config;

	/* display path */
	struct vioc_disp disp;
	struct vioc_rdma rdma;
	struct vioc_wmix wmix;
	struct vioc_sc sc;

	/* deinterlace path */
	struct vioc_rdma deintl_rdma;
	struct vioc_wmix deintl_wmix;
	struct vioc_wdma deintl_wdma;
	struct vioc_viqe deintl_viqe;
	struct vioc_deintls deintl_s;
	struct vioc_sc deintl_sc;

	/* subtitle path */
	struct vioc_rdma sub_rdma;
	struct vioc_rdma sub_rdma_ext;
	struct vioc_wmix sub_wmix;
	struct vioc_wmix sub_wmix_ext;
	WMIXER_ALPHABLENDING_TYPE wmix_alpha;
	struct vioc_alpha sub_alpha;
	int sub_buf_init;
	int sub_buf_current_index;
#ifdef CONFIG_ARCH_TCC_MAP_CONVERTER
	struct vioc_mc deintl_mc;
#endif

#if defined(CONFIG_TCC_DISPLAY_MODE_DUAL_HDMI_CVBS) || defined(CONFIG_TCC_DISPLAY_MODE_DUAL_ALL)
	/* extend display path */
	struct vioc_disp ext_disp;
	struct vioc_rdma ext_rdma;
#endif
};

struct tcc_vout_device {
	int id;
	int opened;
	struct device_node *v4l2_np;
	struct v4l2_device v4l2_dev;
	struct video_device *vdev;
	void __iomem *base;
	struct mutex lock;					// lock to protect the shared data in ioctl

	struct tcc_vout_vioc *vioc;
	enum tcc_vout_status status;

	int fmt_idx;
	int bpp;
	int pfmt;

	struct v4l2_pix_format src_pix;		// src image format
	struct v4l2_rect panel_rect;		// panel area
	struct v4l2_rect disp_rect;			// display output area (before cropping)
	struct v4l2_rect crop_rect;			// clopping area (finally display area)
	struct v4l2_rect crop_src;			// to crop source video (deintl_rdma crop)

	/* vout */
	unsigned int force_userptr;			// force V4L2_MEMORY_USERPTR
	pmap_t pmap;						// for only V4L2_MEMORY_MMAP

	/* reqbuf */
	struct tcc_v4l2_buffer *qbufs;
	enum v4l2_memory memory;
	int nr_qbufs;
	int mapped;
	int clearFrameMode;					//

	/* deinterlce */
	enum v4l2_field previous_field;		// previous field for deinterlace
	enum deintl_type deinterlace;		// deinterlacer type
	pmap_t deintl_pmap;
	int deintl_nr_bufs, deintl_nr_bufs_count;
	unsigned int deintl_buf_size;		// full size of deintl_buf, it is depended on panel size.
	struct tcc_v4l2_buffer *deintl_bufs;
	wait_queue_head_t frame_wait;
	int wakeup_int;
	int frame_count;
	int deintl_force;					// 0: depend on stream info 1: depend on 'deinterlace'
	//int change_deinterlaced_mode;		// To prevent frequent switching of the scan type
	int first_frame;
	enum v4l2_field first_field;
	int firstFieldFlag;

	int baseTime;
	unsigned int update_gap_time;

	//onthefly
	int onthefly_mode;
	int display_done;
	int last_displayed_buf_idx;
	int force_sync;

	#if defined(CONFIG_TCC_VIOCMG)
	int viqe_to_deints;
	#endif
};

int tcc_vout_try_bpp(unsigned int pixelformat, enum v4l2_colorspace *colorspace);
int tcc_vout_try_pix(unsigned int pixelformat);
#endif //__TCC_VOUT_H__
