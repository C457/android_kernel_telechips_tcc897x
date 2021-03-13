/****************************************************************************
 * arch/arm/mach-tcc897x/pmap.c
 * Copyright (C) 2014 Telechips Inc.
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
#include <soc/tcc/pmap.h>
#include <mach/tcc_pmap.h>

static pmap_t tcc897x_pmap_table[] = {
	{ "pmem",		PMAP_BASE,				0 },
	{ "fb_video",		FB_VIDEO_BASE,			FB_VIDEO_SIZE },
	{ "secured_inbuff",	SECURED_INBUFF_BASE,		SECURED_INBUFF_SIZE },
	{ "video_sbackup",	VIDEO_SBACKUP_BASE,		VIDEO_SBACKUP_SIZE },
	{ "overlay",		OVERLAY_BASE,			OVERLAY_SIZE },
	{ "overlay1",		OVERLAY1_BASE,			OVERLAY1_SIZE },
	{ "overlay_rot",	OVERLAY_LOT_BASE,		OVERLAY_LOT_SIZE },
	{ "video_ext",		VIDEO_EXT_BASE,			VIDEO_EXT_SIZE },
	{ "video",		VIDEO_BASE,			VIDEO_SIZE },
	{ "secure_all",		SECURED_INBUFF_BASE,		SECURED_SIZE },
	{ "video_thumb",	VIDEO_THUMB_BASE,		VIDEO_THUMB_SIZE },
	{ "ump_reserved",	UMP_RESERVED_BASE,		UMP_RESERVED_SIZE },
	{ "fb_wmixer",		FB_WMIXER_BASE,			FB_WMIXER_SIZE },
	{ "ram_console",	RAM_CONSOLE_BASE,		RAM_CONSOLE_SIZE },
	{ "viqe",		VIQE_BASE,			VIQE_SIZE },
	{ "ext_camera",		EXT_CAMERA_BASE,		EXT_CAMERA_SIZE },
	{ "fb_scale",		FB_SCALE_BASE,			FB_SCALE_SIZE },
	{ "fb_scale0",		FB_SCALE0_BASE,			FB_SCALE0_SIZE },
	{ "fb_scale1",		FB_SCALE1_BASE,			FB_SCALE1_SIZE },
	{ "fb_g2d0",		FB_G2D0_BASE,			FB_G2D0_SIZE },
	{ "fb_g2d1",		FB_G2D1_BASE,			FB_G2D1_SIZE },
	{ "video_dual",		VIDEO_DUAL_BASE,		VIDEO_DUAL_SIZE },
	{ "jpeg_header",	JPEG_HEADER_BASE,		JPEG_HEADER_SIZE },
	{ "jpeg_stream",	JPEG_STREAM_BASE,		JPEG_STREAM_SIZE },
	{ "jpeg_raw",		JPEG_RAW_BASE,			JPEG_RAW_SIZE },
	{ "nand_mem",		NAND_MEM_BASE,			NAND_MEM_SIZE },
	{ "tsif",		TSIF_MEM_BASE,			TSIF_MEM_SIZE },
	{ "jpg_enc_dxb",	JPG_ENC_DXB_BASE,		JPG_ENC_DXB_SIZE },
	{ "jpg_raw_dxb",	JPG_RAW_DXB_BASE,		JPG_RAW_DXB_SIZE },
	{ "output_attach",	OUTPUT_ATTACH_BASE,		OUTPUT_ATTACH_SIZE},
	{ "total",		PMAP_BASE,			PMAP_TOTAL },
};

pmap_t * tcc_get_pmap_table(void)
{
	return (void *)tcc897x_pmap_table;
}

unsigned tcc_get_pmap_size(void)
{
	return ARRAY_SIZE(tcc897x_pmap_table);
}

EXPORT_SYMBOL(tcc_get_pmap_table);
EXPORT_SYMBOL(tcc_get_pmap_size);

