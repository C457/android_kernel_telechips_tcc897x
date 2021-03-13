/* linux/arch/arm/mach-tcc897x/include/mach/tcc_pmap.h
 *
 * Copyright (C) 2010 Telechips, Inc.
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

#ifndef _TCC_PMAP_STB_H

#ifdef CONFIG_TCC_VPU_DRV_MODULE
#include <mach/tcc_video_private.h>
#endif

#ifndef VPU_BUFFER_MANAGE_COUNT // to access bootloader!!
#define VPU_BUFFER_MANAGE_COUNT 6
#endif

#define _TCC_PMAP_STB_H 

#undef SZ_1MB
#define SZ_1MB (1024*1024)
#define ARRAY_16MBYTE(x)          ((((x) + ((16*SZ_1MB)-1))>> 24) << 24)
#define ARRAY_MBYTE(x)            ((((x) + (SZ_1MB-1))>> 20) << 20)
#define ARRAY_256KBYTE(x)         ((((x) + ((SZ_1MB/4)-1))>> 18) << 18)

#ifdef CONFIG_ARM_TRUSTZONE
#define SUPPORT_SECURED_INBUFER
#else
//#define SUPPORT_SECURED_INBUFER     //for WiFi Display
#endif




//#########################################################################
/********************* Need to be optimized by customer ******************/

// Caution!! Contact engineer who is in charge of scurity in Telechips
// if you modified anything in this file with enabled CONFIG_ARM_TRUSTZONE Feature.

//#define SUPPORT_ROTATION  // Enable only in case the device support rotation (90/180/270)

//Should sync with hardware/telechips/omx/omx_videodec_interface/include/vdec.h
#if defined(CONFIG_SUPPORT_TCC_HEVC_4K)
#define SUPPORT_VIDEO_MAX_WIDTH		4096
#define SUPPORT_VIDEO_MAX_HEIGHT	2160
#else
#define SUPPORT_VIDEO_MAX_WIDTH		1920	// 1280 / 1920 / 4096
#define SUPPORT_VIDEO_MAX_HEIGHT	1080	// 720  / 1080 / 2160
#endif

//Choose more bigger resolution between Primary and Secondary display device.
//and Should sync with hardware/telechips/common/hwcomposer/Hwrenderer.h
#define SUPPORT_DISPLAY_MAX_WIDTH	1920
#define SUPPORT_DISPLAY_MAX_HEIGHT	1080

#if defined(CONFIG_LCD_HDMI1920X1080) // 1080p UI ?
#define FB_PRIMARY_WIDTH       		1920
#define FB_PRIMARY_HEIGHT       	1080
#else
#define FB_PRIMARY_WIDTH       		1280
#define FB_PRIMARY_HEIGHT       	720
#endif
#define FB_SECONDARY_WIDTH          0
#define FB_SECONDARY_HEIGHT         0

//#########################################################################

#if defined(CONFIG_SUPPORT_TCC_NSK)
#define NSK_REV_MEM_SIZE       (25*SZ_1MB)
#else
#define NSK_REV_MEM_SIZE       0
#endif

#define UMP_BUFFER_MANAGE_COUNT 	12  // 9 by default. 12 for flash/netflix using ACodec 

#ifdef SUPPORT_SECURED_INBUFER
#define SECURED_INBUFF_SIZE	(6*SZ_1MB) // 1.5MB/buffer for compressed input stream!
#define VIDEO_SBACKUP_SIZE	(2*SZ_1MB)
#define VIDEO_THUMB_SIZE	(6*SZ_1MB)
#else
#define SECURED_INBUFF_SIZE	(0)
#define VIDEO_SBACKUP_SIZE	(0)
#define VIDEO_THUMB_SIZE	(0)
#endif

#define OVERLAY_SIZE		ARRAY_MBYTE(SUPPORT_DISPLAY_MAX_WIDTH * SUPPORT_DISPLAY_MAX_HEIGHT * 2 * VPU_BUFFER_MANAGE_COUNT)
#define OVERLAY1_SIZE		ARRAY_MBYTE(SUPPORT_DISPLAY_MAX_WIDTH * SUPPORT_DISPLAY_MAX_HEIGHT * 2 * VPU_BUFFER_MANAGE_COUNT) // Should fix it!!
#ifdef SUPPORT_ROTATION
#define OVERLAY_LOT_SIZE	ARRAY_MBYTE(FB_PRIMARY_WIDTH * FB_PRIMARY_HEIGHT * 2 * 2)
#else
#define OVERLAY_LOT_SIZE	(0)
#endif

#if defined(CONFIG_VDEC_CNT_3) || defined(CONFIG_VDEC_CNT_4)
#define VIDEO_EXT_SIZE  	(24*SZ_1MB)	  // at least 24 for D1 and 42 for 720p and 68 for 1080p
#else
#define VIDEO_EXT_SIZE  	(0)
#endif

#if defined(CONFIG_SUPPORT_TCC_HEVC_4K)
#define VIDEO_TEMP_SIZE		(603*SZ_1MB)  // Compressed/3840x2160  :: 9.ref = 567MB , max.ref = 873MB ::  Compressed/4096x2160  :: 9.ref = 603MB , max.ref = 930MB
#elif defined(CONFIG_SUPPORT_TCC_HEVC)
#define VIDEO_TEMP_SIZE		(208*SZ_1MB)  // Uncompressed/1080p  :: 4.ref = 150MB , 9.ref = 208MB, max.ref = 312MB
#else
#define VIDEO_TEMP_SIZE		(120*SZ_1MB) // to support adaptive-streaming with vpu's crop function!!
#endif

#define EXTRA_SIZE			(SECURED_INBUFF_SIZE + VIDEO_SBACKUP_SIZE + OVERLAY_SIZE + OVERLAY1_SIZE + OVERLAY_LOT_SIZE)
#ifdef SUPPORT_SECURED_INBUFER
#if defined(CONFIG_SUPPORT_TCC_HEVC)
#define VIDEO_WORK			(15*SZ_1MB)
#else
#define VIDEO_WORK			(2*SZ_1MB)
#endif
#define SECURED_SIZE		(ARRAY_16MBYTE(EXTRA_SIZE + VIDEO_EXT_SIZE + VIDEO_TEMP_SIZE - VIDEO_WORK))
#define VIDEO_SIZE			((SECURED_SIZE) - (EXTRA_SIZE + VIDEO_EXT_SIZE) + VIDEO_WORK)
#else //SUPPORT_SECURED_INBUFER
#define SECURED_SIZE		(EXTRA_SIZE + VIDEO_EXT_SIZE + VIDEO_TEMP_SIZE)
#define VIDEO_SIZE			((SECURED_SIZE) - (EXTRA_SIZE + VIDEO_EXT_SIZE))
#endif


#define UMP_RESERVED_SIZE	(ARRAY_MBYTE((ARRAY_256KBYTE(SUPPORT_VIDEO_MAX_WIDTH * SUPPORT_VIDEO_MAX_HEIGHT * 3 / 2)) * UMP_BUFFER_MANAGE_COUNT) + (1*SZ_1MB)) // 1MB for specific streaming case!!
#define FB_WMIXER_SIZE		ARRAY_MBYTE(SUPPORT_VIDEO_MAX_WIDTH * SUPPORT_VIDEO_MAX_HEIGHT * 2)
#define RAM_CONSOLE_SIZE	(0)

//#########################################################################
/******** only reference for JPEG (Not allocated mem) in VIQE mem ********/
#define JPEG_HEADER_SIZE	(1*SZ_1MB)
#define JPEG_STREAM_SIZE	(5*SZ_1MB)
#define JPEG_RAW_SIZE		(15*SZ_1MB)
//#########################################################################
#define VIQE_SIZE			(JPEG_HEADER_SIZE+JPEG_STREAM_SIZE+JPEG_RAW_SIZE)

#if defined(CONFIG_USB_VIDEO_CLASS)   // for USB Camera Image
#define EXT_CAMERA_SIZE     (21*SZ_1MB)
#else
#define EXT_CAMERA_SIZE     (0)
#endif

#define FB_VIDEO_SIZE		(ARRAY_MBYTE(FB_PRIMARY_WIDTH * FB_PRIMARY_HEIGHT * 4 * 3) + (ARRAY_MBYTE(FB_SECONDARY_WIDTH * FB_SECONDARY_HEIGHT * 4 * 3))) 
#define FB_SCALE_SIZE		(0)
#define FB_SCALE0_SIZE		(0)
#define FB_SCALE1_SIZE		(0)
#define FB_G2D0_SIZE		(0)
#define FB_G2D1_SIZE		(0)
#define VIDEO_DUAL_SIZE		(0)
#define NAND_MEM_SIZE		(1*SZ_1MB)
#if defined(CONFIG_SPI_TCCXXXX_TSIF_SLAVE)
#define TSIF_MEM_SIZE		(4*SZ_1MB)
#define JPG_ENC_DXB_SIZE	(3*SZ_1MB)
#define JPG_RAW_DXB_SIZE	(3*SZ_1MB)
#else
#define TSIF_MEM_SIZE		(0)
#define JPG_ENC_DXB_SIZE	(0)
#define JPG_RAW_DXB_SIZE	(0)
#endif
#if defined(CONFIG_TCC_DISPLAY_MODE_USE)
#define OUTPUT_ATTACH_SIZE	(4*SZ_1MB)
#else
#define OUTPUT_ATTACH_SIZE	(0*SZ_1MB)
#endif

#if defined(CONFIG_VIDEO_TCCXX_HDMI_IN)
#define V4L2_HDIN_SIZE		(32*SZ_1MB)
#else
#define V4L2_HDIN_SIZE		(0)
#endif

#if defined(CONFIG_VIDEO_TCC_VOUT)
#define V4L2_VOUT_SIZE		(12*SZ_1MB)
#define V4L2_SUBTITLE_SIZE	(32*SZ_1MB)	/* (1920 * 1080 * 4byte) * 4 = 32MB */
#else
#define V4L2_VOUT_SIZE		(0)
#define V4L2_SUBTITLE_SIZE	(0)
#endif

#if defined(CONFIG_FB1_SUPPORT)
#define FB1_VIDEO_SIZE         (CONFIG_FB1_MEMORY_SIZE *SZ_1MB)
#else
#define FB1_VIDEO_SIZE         (0)
#endif//

#define PMAP_BASE			(0x88000000-FB_VIDEO_SIZE)
#define FB_VIDEO_BASE		PMAP_BASE
#define NSK_REV_MEM_BASE	(FB_VIDEO_BASE+FB_VIDEO_SIZE) // should be fixed address 0x88000000
#define SECURED_INBUFF_BASE	(NSK_REV_MEM_BASE+NSK_REV_MEM_SIZE)
#define VIDEO_SBACKUP_BASE	(SECURED_INBUFF_BASE+SECURED_INBUFF_SIZE)
#define OVERLAY_BASE		(VIDEO_SBACKUP_BASE+VIDEO_SBACKUP_SIZE)
#define OVERLAY1_BASE		(OVERLAY_BASE+OVERLAY_SIZE)
#define OVERLAY_LOT_BASE	(OVERLAY1_BASE+OVERLAY1_SIZE)
#define VIDEO_EXT_BASE		(OVERLAY_LOT_BASE+OVERLAY_LOT_SIZE)
#define VIDEO_BASE			(VIDEO_EXT_BASE+VIDEO_EXT_SIZE)
#define VIDEO_THUMB_BASE	(VIDEO_BASE+VIDEO_SIZE)
#define UMP_RESERVED_BASE	(VIDEO_THUMB_BASE+VIDEO_THUMB_SIZE)
#define FB_WMIXER_BASE		(UMP_RESERVED_BASE+UMP_RESERVED_SIZE)
#define RAM_CONSOLE_BASE	(FB_WMIXER_BASE+FB_WMIXER_SIZE)
#define JPEG_HEADER_BASE	(RAM_CONSOLE_BASE+RAM_CONSOLE_SIZE)
#define VIQE_BASE			JPEG_HEADER_BASE
#define JPEG_STREAM_BASE	(JPEG_HEADER_BASE+JPEG_HEADER_SIZE)
#define JPEG_RAW_BASE		(JPEG_STREAM_BASE+JPEG_STREAM_SIZE)
#define EXT_CAMERA_BASE		(JPEG_RAW_BASE+JPEG_RAW_SIZE)
#define FB_SCALE_BASE		(EXT_CAMERA_BASE+EXT_CAMERA_SIZE)
#define FB_SCALE0_BASE		(FB_SCALE_BASE+FB_SCALE_SIZE)
#define FB_SCALE1_BASE		(FB_SCALE0_BASE+FB_SCALE0_SIZE)
#define FB_G2D0_BASE		(FB_SCALE1_BASE+FB_SCALE1_SIZE)
#define FB_G2D1_BASE		(FB_G2D0_BASE+FB_G2D0_SIZE)
#define VIDEO_DUAL_BASE		(FB_G2D1_BASE+FB_G2D1_SIZE)
#define NAND_MEM_BASE		(VIDEO_DUAL_BASE+VIDEO_DUAL_SIZE)
#define TSIF_MEM_BASE		(NAND_MEM_BASE+NAND_MEM_SIZE)
#define JPG_ENC_DXB_BASE	(TSIF_MEM_BASE+TSIF_MEM_SIZE)
#define JPG_RAW_DXB_BASE	(JPG_ENC_DXB_BASE+JPG_ENC_DXB_SIZE)
#define OUTPUT_ATTACH_BASE	(JPG_RAW_DXB_BASE+JPG_RAW_DXB_SIZE)
#define V4L2_HDIN_BASE		(OUTPUT_ATTACH_BASE+OUTPUT_ATTACH_SIZE)
#define V4L2_VOUT_BASE		(V4L2_HDIN_BASE+V4L2_HDIN_SIZE)
#define V4L2_SUBTITLE_BASE	(V4L2_VOUT_BASE+V4L2_VOUT_SIZE)
#define FB1_VIDEO_BASE         (V4L2_SUBTITLE_BASE+V4L2_SUBTITLE_SIZE)

#define PMAP_TOTAL			((FB1_VIDEO_BASE-PMAP_BASE)+FB1_VIDEO_SIZE)

#endif
