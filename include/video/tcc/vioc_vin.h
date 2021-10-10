/*
 * linux/arch/arm/mach-tcc893x/include/mach/vioc_vin.h
 * Author:  <linux@telechips.com>
 * Created: June 10, 2008
 * Description: TCC VIOC h/w block 
 *
 * Copyright (C) 2008-2009 Telechips
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see the file COPYING, or write
 * to the Free Software Foundation, Inc.,
 * 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#ifndef _VIOC_VIN_H_
#define	_VIOC_VIN_H_

#define 	ORDER_RGB 			0
#define 	ORDER_RBG 			1
#define 	ORDER_GRB 			2
#define 	ORDER_GBR 			3
#define 	ORDER_BRG 			4
#define 	ORDER_BGR 			5

#define 	FMT_YUV422_16BIT 			0
#define 	FMT_YUV422_8BIT 			1
#define 	FMT_YUVK4444_16BIT 		2
#define 	FMT_YUVK4224_24BIT 		3
#define 	FMT_RGBK4444_16BIT 		4
#define 	FMT_RGB444_24BIT 			9
#define 	FMT_SD_PROG 				12 // NOT USED

#define 	EXT_PORT0 			0
#define 	EXT_PORT1 			1
#define 	EXT_PORT2 			2
#define 	EXT_PORT3 			3
#define 	MUL_PORT0 			4
#define 	MUL_PORT1 			5
#define 	MUL_PORT2 			6
#define 	MUL_PORT3 			7

#define 	CLK_DOUBLE_EDGE 				0
#define 	CLK_DOUBLE_FREQ 				1
#define 	CLK_DOUBLE_EDGE_FREQ 		2
#define 	CLK_DOUBLE_4TIME_FREQ 		3

#define 	ON 				1
#define 	OFF 			0


/************************************************************************
*   3.6.3   VIDEO IN                         (Base Addr = 0x12004000)
*************************************************************************/

typedef struct
{
	unsigned		enable				:	 1;
	unsigned		conv_en				:	 1;
	unsigned		intl_en				:	 1;
	unsigned		intpl_en			: 	 1;
	unsigned		hsde_connect_en		:	 1;
	unsigned		flush				:	 1;
	unsigned		vs_mask				:	 1;
	unsigned		RESERVE0			:	 1;
	unsigned		pxclk_pol			:	 1;
	unsigned		hs_active_low		:	 1;
	unsigned		vs_active_low		:	 1;
	unsigned		field_bfield_low	:	 1;
	unsigned		de_active_low		:	 1;
	unsigned		gen_field_en		:	 1;
	unsigned		stream_en			:	 1;
	unsigned		RESERVE1			:	 1;
	unsigned		fmt					:	 4;
	unsigned		data_order			:	 3;
	unsigned		RESERVE2			:	 1;
	unsigned		skip				:	 4;
	unsigned		RESERVE3			:	 3;
	unsigned		capture				:	 1;
}	VIN_CTRL;

typedef union
{
	unsigned long		nREG;
	VIN_CTRL			bVIN_CTRL;
}	VIN_CTRL_u;

typedef struct
{
	unsigned	lut0_en				:	 1;
	unsigned	lut1_en				:	 1;
	unsigned	lut2_en				:	 1;
	unsigned	lut_apbif_en		:	 1;
	unsigned	y2r_en				:	 1;
	unsigned	y2r_mode			:	 2;
	unsigned	RESERVE0			:   25;
}	VIN_MISC;

typedef union
{
	unsigned	long		nREG;
	VIN_MISC				bVIN_MISC;
}	VIN_MISC_u;

typedef struct
{
	unsigned	hblank				:	 4;
	unsigned	vblank				:	 4;
	unsigned	field_pos				:	 4;
	unsigned	RESERVE0			:	 4;
	unsigned	pre_sta_loc			:	 2;
	unsigned	sync_byte			:	 2;
	unsigned	RESERVE1			:	12;
}	VIN_SYNC_M0;

typedef union
{
	unsigned	long		nREG;
	VIN_SYNC_M0		bREG;
}	VIN_SYNC_M0_u;

typedef struct
{
	unsigned	pre_first			:	 8;
	unsigned	pre_second			:	 8;
	unsigned	pre_third			:	 8;
	unsigned	RESERVE0			:	 8;
}	VIN_SYNC_M1;

typedef union
{
	unsigned	long		nREG;
	VIN_SYNC_M1		bREG;
}	VIN_SYNC_M1_u;

typedef	struct
{
	unsigned	width				:	16;
	unsigned	height				:	16;
}	VIN_SIZE;

typedef union
{
	unsigned	long			nREG;
	VIN_SIZE				bVIN_SIZE;
}	VIN_SIZE_u;

typedef	struct
{
	unsigned	offs_width			:	16;
	unsigned	offs_height			:	16;
}	VIN_OFFS;

typedef union
{
	unsigned	long		nREG;
	VIN_OFFS				bVIN_OFFS;
}	VIN_OFFS_u;

typedef	struct
{
	unsigned	update_int				:	 1;
	unsigned	eof_int					:	 1;
	unsigned	vs_int					:	 1;
	unsigned	invs_int					:	 1;
	unsigned	RESERVE0			:	 7;	
	unsigned	frm_sq					:	 1;	
	unsigned RESERVE1			:	4;
	unsigned	update_int_mask			:	 1;
	unsigned	eof_int_mask				:	 1;
	unsigned	vs_int_mask				: 	 1;
	unsigned	invs_int_mask				: 	 1;
	unsigned	RESERVE2			: 	 11;	
	unsigned	int_en					:	 1;
}	VIN_INT;

typedef	union
{
	unsigned	long		nREG;
	VIN_INT				bVIN_INT;
}	VIN_INT_u;

typedef	struct
{
	unsigned	width_cnt			:	16;
	unsigned	height_cnt			:	16;
}	VIN_STAT_CNT;

typedef union
{
	unsigned	long		nREG;
	VIN_STAT_CNT			bREG;
}	VIN_STAT_CNT_u;

typedef	struct
{
	unsigned	value0			:	8;
	unsigned	value1			:	8;
	unsigned	value2			:	8;
	unsigned	value3			:	8;
}	VIN_LUT;

typedef union
{
	unsigned	long 		nREG[63];
	VIN_LUT 				bREG;
}	VIN_LUT_C0_u;

typedef union
{
	unsigned	long 		nREG[63];
	VIN_LUT 				bREG;
}	VIN_LUT_C1_u;

typedef union
{
	unsigned	long 		nREG[63];
	VIN_LUT 				bREG;
}	VIN_LUT_C2_u;

typedef	struct
{
	unsigned	width			:	16;
	unsigned	height			:	16;
}	VIN_CROP_SIZE;

typedef union
{
	unsigned long			nREG;
	VIN_CROP_SIZE			bREG;
}	VIN_CROP_SIZE_u;

typedef	struct
{
	unsigned	offs_width		:	16;
	unsigned	offs_height		:	16;
}	VIN_CROP_OFFS;

typedef union
{
	unsigned long			nREG;
	VIN_CROP_OFFS			bREG;
}	VIN_CROP_OFFS_u;

typedef struct _VIOC_VIN
{
	volatile VIN_CTRL_u 			uVIN_CTRL; 			// 	0x000  R/W  0x00000000 VIN Control Register
	volatile VIN_MISC_u 			uVIN_MISC; 			// 	0x004  R/W  0x00000000 VIN Misc Register
	volatile VIN_SYNC_M0_u 			uVIN_SYNC_M0; 		// 	0x008  R/W  0x00030CB9 VIN Sync Misc. 0 Register
	volatile VIN_SYNC_M1_u 			uVIN_SYNC_M1; 		// 	0x00c  R/W  0x000000FF VIN Sync misc. 1 Register
	volatile VIN_SIZE_u 			uVIN_SIZE; 			// 	0x010  R/W  0x00000000 VIN Size Register
	volatile VIN_OFFS_u 			uVIN_OFFS; 			// 	0x014  R/W  0x00000000 VIN Offset Register
	volatile VIN_OFFS_u 			uVIN_OFFS_INTL; 	// 	0x018  R/W  0x00000000 VIN Offset Interlaced Register
	volatile VIN_CROP_SIZE_u 		uVIN_CROP_SIZE; 	// 	0x01c  R/W  0x00000000 VIN Crop Size Register
	volatile VIN_CROP_OFFS_u 		uVIN_CROP_OFFS; 	// 	0x020  R/W  0x00000000 VIN Crop Offset Register
	unsigned int 					undef0[15]; 		// 	0x024 ~ 0x5c
	volatile VIN_INT_u 				uVIN_INT; 			// 	0x060  R/W  0x00000000 VIN Iterrupt Register	
	unsigned int 					undef1[231]; 		// 	0x064 ~ 0x3FC
	volatile VIN_LUT_C0_u 			uVIN_LUT_C0;		// 	0x400 ~ 0x4FB R/W VIN Look-up Table set in Channel 0
	unsigned int					undef2;				//	0x4FC ~ 0x4FF
	volatile VIN_LUT_C1_u 			uVIN_LUT_C1;		// 	0x500 ~ 0x5FB R/W VIN Look-up Table set in Channel 1
	unsigned int					undef3;				//	0x5FC ~ 0x5FF	
	volatile VIN_LUT_C2_u 			uVIN_LUT_C2;		// 	0x600 ~ 0x6FB R/W VIN Look-up Table set in Channel 2
	unsigned int					undef4;				//	0x6FC ~ 0x6FF
} VIOC_VIN, *PVIOC_VIN;

typedef struct
{
	unsigned	enable				:	 1;
	unsigned					:	 3;
	unsigned	byte_select			:	 2;
	unsigned					:	 2;
	unsigned	clk_mode			:	 3;
	unsigned					:	 5;
	unsigned	sel0					:	 3;
	unsigned					:	 1;
	unsigned	sel1					:	 3;
	unsigned 					:	 1;
	unsigned	sel2					: 	 3;
	unsigned 					:	 1;
	unsigned	sel3					:	 3;
	unsigned 					:	 1;
}	VIN_DEMUX_CTRL;

typedef	union
{
	unsigned	long			nREG;
	VIN_DEMUX_CTRL		bVIN_DEMUX_CTRL;
}	VIN_DEMUX_CTRL_u;

typedef	struct
{
	unsigned	hb					:	 4;
	unsigned	vb					:	 4;
	unsigned	fp					:	 4;
	unsigned					:	 4;
	unsigned	psl					:	 2;
	unsigned	sync_byte			:	 2;
	unsigned					:	12;
}	VIN_DEMUX_BLANK0;

typedef union
{
	unsigned	long		nREG;
	VIN_DEMUX_BLANK0		bREG;
}	VIN_DEMUX_BLANK0_u;

typedef struct
{
	unsigned	pf					:	 8;
	unsigned	ps					:	 8;
	unsigned	pt					:	 8;
	unsigned	RESERVE0			:	 8;
}	VIN_DEMUX_BLANK1;

typedef union
{
	unsigned	long		nREG;
	VIN_DEMUX_BLANK1		bREG;
}	VIN_DEMUX_BLANK1_u;

typedef struct
{
	unsigned	dsel0				:	 4;
	unsigned	dsel1				:	 4;
	unsigned	dsel2				:	 4;
	unsigned	dsel3				:	 4;
	unsigned	RESERVE0			:	16;
}	VIN_DEMUX_MISC;

typedef union
{
	unsigned	long		nREG;
	VIN_DEMUX_MISC		bREG;
}	VIN_DEMUX_MISC_u;

typedef struct
{
	unsigned	idx0				:	 4;
	unsigned	idx1				:	 4;
	unsigned	idx2				:	 4;
	unsigned	idx3				:	 4;
	unsigned	RESERVE0			:	16;
}	VIN_DEMUX_STS;

typedef	union
{
	unsigned	long		nREG;
	VIN_DEMUX_STS			bREG;
}	VIN_DEMUX_STS_u;


typedef	struct _VOIC_VIN_DEMUX
{
	volatile VIN_DEMUX_CTRL_u		uVIN_DEMUX_CTRL;	//	0x000 R/W 0x32100000 VIN DEMUX Control register
	volatile VIN_DEMUX_BLANK0_u		uVIN_DEMUX_BLANK0;	//	0x004 R/W 0x00030CB9 VIN DEMUX Blank Set 0 Register
	volatile VIN_DEMUX_BLANK1_u		uVIN_DEMUX_BLANK1;	//	0x008 R/W 0x000000FF VIN DEMUX Black Set 1 Register
	volatile VIN_DEMUX_MISC_u		uVIN_DEMUX_MISC;	//	0x00C R/W 0x00004444 VIN DEMUX Miscellaneous Register
	volatile VIN_DEMUX_STS_u		uVIN_DEMUX_STS;		//	0x010 R     -                VIN DEMUX Status Register
}	VIOC_VIN_DEMUX,*PVIOC_VIN_DEMUX;

/* Interface APIs. */
extern void VIOC_VIN_SetSyncPolarity (VIOC_VIN *pVIN, unsigned int hs_active_low, unsigned int vs_active_low, unsigned int field_bfield_low, unsigned int de_active_low, unsigned int gen_field_en, unsigned int pxclk_pol);
extern void VIOC_VIN_SetCtrl (VIOC_VIN *pVIN, unsigned int conv_en, unsigned int hsde_connect_en, unsigned int vs_mask, unsigned int fmt, unsigned int data_order);
extern void VIOC_VIN_SetInterlaceMode (VIOC_VIN *pVIN, unsigned int intl_en, unsigned int intpl_en);
extern void VIOC_VIN_SetCaptureModeEnable(VIOC_VIN *pVIN, unsigned int cap_en);
extern void VIOC_VIN_SetEnable (VIOC_VIN *pVIN, unsigned int vin_en);
extern void VIOC_VIN_SetImageSize (VIOC_VIN *pVIN, unsigned int width, unsigned int height);
extern void VIOC_VIN_SetImageOffset (VIOC_VIN *pVIN, unsigned int offs_width, unsigned int offs_height, unsigned int offs_height_intl);
extern void VIOC_VIN_SetImageCropSize(VIOC_VIN *pVIN, unsigned int width, unsigned int height);
extern void VIOC_VIN_SetImageCropOffset(VIOC_VIN *pVIN, unsigned int offs_width, unsigned int offs_height);
extern void VIOC_VIN_SetY2RMode (VIOC_VIN *pVIN, unsigned int y2r_mode);
extern void VIOC_VIN_SetY2REnable (VIOC_VIN *pVIN, unsigned int y2r_en);
extern void VIOC_VIN_SetLUT(VIOC_VIN *pVIN, unsigned int *pLUT);
extern void VIOC_VIN_SetLUT_by_table(VIOC_VIN *pVIN, unsigned int * pTable);
extern void VIOC_VIN_SetLUTEnable(VIOC_VIN *pVIN, unsigned int lut0_en, unsigned int lut1_en, unsigned int lut2_en);
extern void VIOC_VIN_SetDemuxPort(VIOC_VIN_DEMUX *pVINDEMUX, unsigned int p0, unsigned int p1, unsigned int p2, unsigned int p3);
extern void VIOC_VIN_SetDemuxClock(VIOC_VIN_DEMUX *pVINDEMUX, unsigned int mode);
extern void VIOC_VIN_SetDemuxEnable(VIOC_VIN_DEMUX *pVINDEMUX, unsigned int enable);
extern void VIOC_VIN_IreqHandler(unsigned int vectorID);
extern unsigned int VIOC_VIN_IsEnable(VIOC_VIN *pVIN);

#endif
