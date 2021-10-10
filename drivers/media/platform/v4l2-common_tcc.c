/* linux/arch/arm/plat-tcc/v4l2-common_tcc.c
 *
 * Copyright (C) 2012 Telechips, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/videodev2.h>
#ifdef CONFIG_ARCH_TCC897X
#include <mach/vioc_global.h>
#else
#include <video/tcc/vioc_global.h>
#endif

/**
 * Convert image format
 *   fourcc2vioc: fourcc format to vioc format
 *   vioc2fourcc: vioc format to fourcc format
 */
unsigned int vioc2fourcc(unsigned int vioc_fmt)
{
	unsigned int fourcc;
	switch(vioc_fmt) {
	/* sequential (YUV packed) */
	case VIOC_IMG_FMT_UYVY:				// LSB [Y/U/Y/V] MSB : YCbCr 4:2:2 sequential
		fourcc = V4L2_PIX_FMT_UYVY;			// 'UYVY' 16 YUV 4:2:2
		break;
	case VIOC_IMG_FMT_VYUY:				// LSB [Y/V/Y/U] MSB : YCbCr 4:2:2 sequential
		fourcc = V4L2_PIX_FMT_VYUY;			// 'VYUY' 16 YUV 4:2:2
		break;
	case VIOC_IMG_FMT_YUYV:				// LSB [Y/U/Y/V] MSB : YCbCr 4:2:2 sequential
		fourcc = V4L2_PIX_FMT_YUYV;			// 'YUYV' 16 YUV 4:2:2
		break;
	case VIOC_IMG_FMT_YVYU:				// LSB [Y/V/Y/U] MSB : YCbCr 4:2:2 sequential
		fourcc = V4L2_PIX_FMT_YVYU;			// 'YVYU' 16 YVU 4:2:2
		break;

	/* sepatated (Y, U, V planar) */
	case VIOC_IMG_FMT_YUV420SEP:		// YCbCr 4:2:0 separated
		fourcc = V4L2_PIX_FMT_YVU420;		// 'YV12' 12 YVU 4:2:0
		break;
	case VIOC_IMG_FMT_YUV422SEP:		// YCbCr 4:2:2 separated
		fourcc = V4L2_PIX_FMT_YUV422P;		// '422P' 16 YVU422 Planar
		break;

	/* interleaved (Y palnar, UV planar) */
	case VIOC_IMG_FMT_YUV420IL0:		// YCbCr 4:2:0 interleaved type0
		fourcc = V4L2_PIX_FMT_NV12;			// 'NV12' 12 Y/CbCr 4:2:0
		break;
	case VIOC_IMG_FMT_YUV420IL1:		// YCbCr 4:2:0 interleaved type1
		fourcc = V4L2_PIX_FMT_NV21;			// 'NV21' 12 Y/CrCb 4:2:0
		break;
	case VIOC_IMG_FMT_YUV422IL0:		// YCbCr 4:2:2 interleaved type0
		fourcc = V4L2_PIX_FMT_NV16;			// 'NV16' 16 Y/CbCr 4:2:2
		break;
	case VIOC_IMG_FMT_YUV422IL1:		// YCbCr 4:2:2 interleaved type1
		fourcc = V4L2_PIX_FMT_NV61;			// 'NV61' 16 Y/CrCb 4:2:2
		break;

	/* RGB */
	case VIOC_IMG_FMT_RGB332:			// R[7:5] G[4:2] B[1:0]
		fourcc = V4L2_PIX_FMT_RGB332;		// 'RGB1' 8 RGB-3-3-2
		break;
	case VIOC_IMG_FMT_ARGB4444:			// A[15:12] R[11:8] G[7:4] B[3:0]
		fourcc = V4L2_PIX_FMT_RGB444;		// 'R444' 16 RGB-4-4-4 (xxxxrrrr ggggbbbb)
		break;
	case VIOC_IMG_FMT_ARGB1555:			// A[15] R[14:10] G[9:5] B[4:0]
		fourcc = V4L2_PIX_FMT_RGB555;		// 'RGB0' 16 RGB-5-5-5
		break;
	case VIOC_IMG_FMT_RGB565:			// R[15:11] G[10:5] B[4:0]
		fourcc = V4L2_PIX_FMT_RGB565;		// 'RGBP' 16 RGB-5-6-5
		break;
	case VIOC_IMG_FMT_RGB888:			// B1[31:24] R[23:16] G[15:8] B0[7:0]
		fourcc = V4L2_PIX_FMT_RGB24;		// 'RBG3' 24 RGB-8-8-8
		//fourcc = V4L2_PIX_FMT_BGR24;		// 'BGR3' 24 BGR-8-8-8
		break;
	case VIOC_IMG_FMT_ARGB8888:			// A[31:24] R[23:16] G[15:8] B[7:0]
		fourcc = V4L2_PIX_FMT_RGB32;		// 'RGB4' 32 RGB-8-8-8-8
		//fourcc = V4L2_PIX_FMT_BGR32;		// 'BGR4' 32 BGR-8-8-8-8	
		break;

	default:
		fourcc = V4L2_PIX_FMT_YVU420;
		break;
	}

	return fourcc;
}

unsigned int fourcc2vioc(unsigned int fourcc)
{
	unsigned int vioc_fmt;
	switch(fourcc) {
	/* sequential (YUV packed) */
	case V4L2_PIX_FMT_UYVY:				// 'UYVY' 16 YUV 4:2:2
		vioc_fmt = VIOC_IMG_FMT_UYVY;		// LSB [Y/U/Y/V] MSB : YCbCr 4:2:2 sequential
		break;
	case V4L2_PIX_FMT_VYUY:				// 'VYUY' 16 YUV 4:2:2
		vioc_fmt = VIOC_IMG_FMT_VYUY;		// LSB [Y/V/Y/U] MSB : YCbCr 4:2:2 sequential
		break;
	case V4L2_PIX_FMT_YUYV:				// 'YUYV' 16 YUV 4:2:2
		vioc_fmt = VIOC_IMG_FMT_YUYV;		// LSB [Y/U/Y/V] MSB : YCbCr 4:2:2 sequential
		break;
	case V4L2_PIX_FMT_YVYU:				// 'YVYU' 16 YVU 4:2:2
		vioc_fmt = VIOC_IMG_FMT_YVYU;		// LSB [Y/V/Y/U] MSB : YCbCr 4:2:2 sequential
		break;

	/* sepatated (Y, U, V planar) */
	case V4L2_PIX_FMT_YVU420:			// 'YV12' 12 YVU 4:2:0
		vioc_fmt = VIOC_IMG_FMT_YUV420SEP;	// YCbCr 4:2:0 separated
		break;
	case V4L2_PIX_FMT_YUV422P:			// '422P' 16 YVU422 Planar
		vioc_fmt = VIOC_IMG_FMT_YUV422SEP;	// YCbCr 4:2:2 separated
		break;

	/* interleaved (Y palnar, UV planar) */
	case V4L2_PIX_FMT_NV12:				// 'NV12' 12 Y/CbCr 4:2:0
		vioc_fmt = VIOC_IMG_FMT_YUV420IL0;	// YCbCr 4:2:0 interleaved type0
		break;
	case V4L2_PIX_FMT_NV21:				// 'NV21' 12 Y/CrCb 4:2:0
		vioc_fmt = VIOC_IMG_FMT_YUV420IL1;	// YCbCr 4:2:0 interleaved type1
		break;
	case V4L2_PIX_FMT_NV16:				// 'NV16' 16 Y/CbCr 4:2:2
		vioc_fmt = VIOC_IMG_FMT_YUV422IL0;	// YCbCr 4:2:2 interleaved type0
		break;
	case V4L2_PIX_FMT_NV61:				// 'NV61' 16 Y/CrCb 4:2:2
		vioc_fmt = VIOC_IMG_FMT_YUV422IL1;	// YCbCr 4:2:2 interleaved type1
		break;

	/* RGB */
	case V4L2_PIX_FMT_RGB332:			// 'RGB1' 8 RGB-3-3-2
		vioc_fmt = VIOC_IMG_FMT_RGB332;		// R[7:5] G[4:2] B[1:0]
		break;
	case V4L2_PIX_FMT_RGB444:			// 'R444' 16 RGB-4-4-4 (xxxxrrrr ggggbbbb)
		vioc_fmt = VIOC_IMG_FMT_ARGB4444;	// A[15:12] R[11:8] G[7:4] B[3:0]
		break;
	case V4L2_PIX_FMT_RGB555:			// 'RGB0' 16 RGB-5-5-5
		vioc_fmt = VIOC_IMG_FMT_ARGB1555;	// A[15] R[14:10] G[9:5] B[4:0]
		break;
	case V4L2_PIX_FMT_RGB565:			// 'RGBP' 16 RGB-5-6-5
		vioc_fmt = VIOC_IMG_FMT_RGB565;		// R[15:11] G[10:5] B[4:0]
		break;
	case V4L2_PIX_FMT_RGB24:			// 'RBG3' 24 RGB-8-8-8
	case V4L2_PIX_FMT_BGR24:			// 'BGR3' 24 BGR-8-8-8
		vioc_fmt = VIOC_IMG_FMT_RGB888;		// B1[31:24] R[23:16] G[15:8] B0[7:0]
		break;
	case V4L2_PIX_FMT_RGB32:			// 'RGB4' 32 RGB-8-8-8-8
	case V4L2_PIX_FMT_BGR32:			// 'BGR4' 32 BGR-8-8-8-8	
		vioc_fmt = VIOC_IMG_FMT_ARGB8888;	// A[31:24] R[23:16] G[15:8] B[7:0]
		break;

	default:
		vioc_fmt = VIOC_IMG_FMT_YUV420SEP;
		break;
	}

	return vioc_fmt;
}

