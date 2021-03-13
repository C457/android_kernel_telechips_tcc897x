/*
 * linux/arch/arm/mach-tcc893x/include/mach/vioc_viqe.h
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
#ifndef _VIOC_VIQE_H_
#define	_VIOC_VIQE_H_

#include <mach/reg_physical.h>

#define FMT_FC_YUV420	0
#define FMT_FC_YUV422	1
#define VIQE_WIDTH		0
#define VIQE_HEIGHT	0
#define ON				1
#define OFF				0

#define	NORMAL_MODE	0		//normal mode
#define	DUPLI_MODE		1		//duplicate mode
#define	SKIP_MODE		2		// skip mode

typedef enum
{
	VIOC_VIQE_DEINTL_MODE_BYPASS = 0,
	VIOC_VIQE_DEINTL_MODE_2D,
	VIOC_VIQE_DEINTL_MODE_3D, 
	VIOC_VIQE_DEINTL_S
}VIOC_VIQE_DEINTL_MODE;

typedef enum
{
	VIOC_VIQE_FMT_YUV420 = 0,
	VIOC_VIQE_FMT_YUV422
}VIOC_VIQE_FMT_TYPE;

typedef	struct
{
	unsigned	his_cdf_or_lut_en				:	 1;
	unsigned	his_en							:	 1;
	unsigned	gamut_en						:	 1;
	unsigned	denoise3d_en					:	 1;
	unsigned	deintl_en						: 	 1;
	unsigned	RESERVE0						:	 3; // bit  7
	unsigned	no_hor_intpl					:	 1;
	unsigned	RESERVE1						:	 7; // bit 15
	unsigned	fmt_conv_disable				:	 1;
	unsigned	fmt_conv_disable_using_fmt	:	 1;
	unsigned	fmt_conv_flush				:	 1;
	unsigned	RESERVE2						:	 1;
	unsigned	update_disable				:	 1;
	unsigned	cfgupd							:	 1;
	unsigned	RESERVE3						:	 2;	// bit 23
	unsigned	clkgate_deintl_disable		:	 1;
	unsigned	clkgate_d3d_disable			:	 1;
	unsigned	clkgate_pm_disable			:	 1;
	unsigned	RESERVE4						:	 5; // bit 31
}	VIQE_CTRL;

typedef	union
{
	unsigned long		nREG;
	VIQE_CTRL			bREG;
}	VIQE_CTRL_u;

typedef	struct
{
	unsigned	width					:	16;
	unsigned	height					:	16;
}	VIQE_SIZE;

typedef	union
{
	unsigned long		nREG;
	VIQE_SIZE			bREG;
}	VIQE_SIZE_u;

typedef	struct
{
	unsigned	h2h						:	 8;
	unsigned	y2r_en					:	 1;
	unsigned	y2r_mode				:	 2;
	unsigned	RESERVE0				:	21;
}	VIQE_CTRL_Y2R;

typedef	union
{
	unsigned long		nREG;
	VIQE_CTRL_Y2R		bREG;
}	VIQE_CTRL_Y2R_u;

typedef	struct
{
	unsigned	deintl_IRQ				:	1;
	unsigned	denoise_IRQ			:	1;
	unsigned	pm_IRQ					:	1;
	unsigned	RESERVE0				:	5;
	unsigned	deintl_IRQ_mask		:	1;
	unsigned	denoise_IRQ_mask		:	1;
	unsigned	pm_IRQ_mask			:	1;
	unsigned							:  21;
}	VIQE_IRQ;

typedef	union
{
	unsigned long		nREG;
	VIQE_IRQ			bREG;
}	VIQE_IRQ_u;

typedef	struct
{
	unsigned	global_en_dont_use			:	 1;
	unsigned	top_size_dont_use				:	 1;
	unsigned	stream_deintl_info_dont_use :	 1;
	unsigned	generate_eof_viqe_signal 	:	 1;
	unsigned	RESERVE0						:	28;
}	VIQE_CTRL_MISC;

typedef union
{
	unsigned long	nREG;
	VIQE_CTRL_MISC	bREG;
}	VIQE_CTRL_MISC_u;

typedef	struct
{
	unsigned	base					:	32;
}	VIQE_BASE;

typedef	union
{
	unsigned long		nREG;
	VIQE_BASE			bREG;
}	VIQE_BASE_u;

typedef	struct
{
	unsigned	offs0					:	16;
	unsigned	offs1					:	16;
}	VIQE_OFFS;

typedef	union
{
	unsigned long		nREG;
	VIQE_OFFS			bREG;
}	VIQE_OFFS_u;

typedef	struct
{
	unsigned	top_size_dont_use				:	 1;
	unsigned	stream_deintl_info_dont_use	:	 1;
	unsigned	uvintpl						:	 1;
	unsigned	RESERVE0						:	 5;
	unsigned 	frmnum							: 	 2;
	unsigned	RESERVE1						:	 6;
	unsigned	enable							:	 1;
	unsigned	cfgupd							:	 1;
	unsigned	frame_rate_disable			:	 1;
	unsigned	RESERVE2						:	 5;
	unsigned	h2h								:	 8;
}	DEINTL_CTRL;

typedef	union
{
	unsigned long	nREG;
	DEINTL_CTRL		bREG;
}	DEINTL_CTRL_u;

typedef	struct
{
	unsigned	detect_end				:	 1;
	unsigned	flush					:	 1;
	unsigned	eof_control_ready		:	 1;
	unsigned	size_fix				:	 1;
	unsigned	RESERVE0				:	 4;
	unsigned	divisor				:	 2;
	unsigned	RESERVE1				:	 2;
	unsigned	fmt						:	 5;
	unsigned	RESERVE2				:	15;
}	VIQE_FC_MISC;

typedef	union
{
	unsigned long		nREG;
	VIQE_FC_MISC		bREG;
}	VIQE_FC_MISC_u;

typedef struct
{
	unsigned	width_dwalign			:	16;
	unsigned	full					:	 4;
	unsigned	empty					:	 4;
	unsigned	eof_in					:	 1;
	unsigned	eof_out				:	 1;
	unsigned	hr_err					:	 1;
	unsigned	decoder_err			:	 1;
	unsigned	decoder_state			:	 4;
}	VIQE_FC_STATUS;

typedef union
{
	unsigned long	nREG;
	VIQE_FC_STATUS	bREG;
}	VIQE_FC_STATUS_u;

typedef	struct
{
	unsigned	fc_select				:	 8;
	unsigned	err_check				:	 1;
	unsigned	hren_en				:	 1;
	unsigned	RESERVE0				:	 6;
	unsigned	fc_stat				:	 8;
	unsigned	RESERVE1				:	 7;
	unsigned	fc_enable				:	 1;
}	VIQE_FC_CTRL;

typedef union
{
	unsigned long		nREG;
	VIQE_FC_CTRL		bREG;
}	VIQE_FC_CTRL_u;

typedef	struct
{
	unsigned	k0_ac_length_limit		:	 6;
	unsigned	RESERVE0					:	 2;
	unsigned	k1_ac_length_limit		:	 6;
	unsigned	RESERVE1					:	 2;
	unsigned	k2_ac_length_limit		:	 6;
	unsigned	RESERVE2					:	 2;
	unsigned	RESERVE3					:	 8;
}	VIQE_FC_LIMIT;

typedef	union
{
	unsigned long		nREG;
	VIQE_FC_LIMIT		bREG;
}	VIQE_FC_LIMIT_u;

typedef	struct
{
	unsigned	top_size_dont_use		:	 1;
	unsigned	RESERVE0				: 	 1;
	unsigned	uvintpl				:	 1;
	unsigned	RESERVE1				:	 5;
	unsigned	frmnum					:	 2;
	unsigned	RESERVE2				:	 6;
	unsigned	enable					:	 1;
	unsigned	cfgupd					:	 1;
	unsigned	RESERVE3				:	 1;
	unsigned	RESERVE4				:	 5;
	unsigned	h2h						:	 8;
}	D3D_CTRL;

typedef	union
{
	unsigned long		nREG;
	D3D_CTRL			bREG;
}	D3D_CTRL_u;

typedef	struct
{
	unsigned	pcnt_y					:	11;
	unsigned	RESERVE0				:	 5;
	unsigned	lcnt_y					:	11;
	unsigned	RESERVE1				:	 2;
	unsigned	int_mask				:	 1;
	unsigned	int_busy				:	 1;
	unsigned	busy_stat				:	 1;
}	D3D_COUNT;

typedef union
{
	unsigned long		nREG;
	D3D_COUNT			bREG;
}	D3D_COUNT_u;

typedef struct
{
	unsigned	h2h						:	 8;
	unsigned	d3in					:	 1;
	unsigned	cfgupd					:	 1;
	unsigned	frmupd_disable		:	 1;
	unsigned	lut_init				:	 1;
	unsigned	RESERVE0				:	 4;
	unsigned	top_size_dont_use		:	 1;
	unsigned	RESERVE1				:	 7;
	unsigned	frmnum					:	 2;
	unsigned	RESERVE2				:	 6;
}	D3D_MISC;

typedef union
{
	unsigned long		nREG;
	D3D_MISC			bREG;
}	D3D_MISC_u;

typedef struct
{
	unsigned	div_pos				:	16;
	unsigned	div_toggle				:	 1;
	unsigned	div_en					:	 1;
	unsigned	RESERVE0				:	14;
}	D3D_DIV;

typedef union
{
	unsigned long	nREG;
	D3D_DIV			bREG;
}	D3D_DIV_u;

typedef	struct
{
	unsigned	bypass					:	 1;
	unsigned	RESERVE0				:	29;
	unsigned	flush					:	 2;
}	D3D_MISC2;

typedef	union
{
	unsigned long		nREG;
	D3D_MISC2			bREG;
}	D3D_MISC2_u;

typedef	struct _VIOC_VIQE_CTRL
{
	volatile VIQE_CTRL_u			nVIQE_CTRL;			//  0x000
	volatile VIQE_SIZE_u			nVIQE_CTRL_SIZE;		//	0x004
	volatile VIQE_CTRL_Y2R_u		nVIQE_CTRL_Y2R;		//	0x008
	volatile VIQE_IRQ_u				nVIQE_IRQ;				//	0x00C
	volatile VIQE_CTRL_MISC_u		nVIQE_CTRL_MISC;		//	0x010
	unsigned	int					nHIDDEN;				//	0x014
	unsigned	int					undef_0x018[26];		// 	0x018 ~ 0x07C
}	VIOC_VIQE_CTRL,*PVIOC_VIQE_CTRL;

typedef	struct _VIQE_DEINTL_DMA
{
	volatile VIQE_BASE_u	nDEINTL_BASE0;				//	0x080
	volatile VIQE_BASE_u	nDEINTL_BASE1;				//	0x084
	volatile VIQE_BASE_u	nDEINTL_BASE2;				//	0x088
	volatile VIQE_BASE_u	nDEINTL_BASE3;				//	0x08C
	volatile VIQE_SIZE_u	nDEINTL_SIZE;				//  0x090
	volatile VIQE_OFFS_u	nDEINTL_OFFS;				//	0x094
	volatile DEINTL_CTRL_u	nDEINTL_CTRL;				//	0x098
	unsigned	int			undef_0x09c[1];			//  0x09C
	volatile VIQE_BASE_u	nDEINTL_BASE0A;			//  0x0A0
	volatile VIQE_BASE_u	nDEINTL_BASE1A;			//  0x0A4
	volatile VIQE_BASE_u	nDEINTL_BASE2A;			//  0x0A8
	volatile VIQE_BASE_u	nDEINTL_BASE3A;			//	0x0AC
	volatile VIQE_BASE_u	nDEINTL_BASE0B;			//  0x0B0
	volatile VIQE_BASE_u	nDEINTL_BASE1B;			//  0x0B4
	volatile VIQE_BASE_u	nDEINTL_BASE2B;			//  0x0B8
	volatile VIQE_BASE_u	nDEINTL_BASE3B;			//	0x0BC
	volatile VIQE_BASE_u	nDEINTL_BASE0C;			//  0x0C0
	volatile VIQE_BASE_u	nDEINTL_BASE1C;			//  0x0C4
	volatile VIQE_BASE_u	nDEINTL_BASE2C;			//  0x0C8
	volatile VIQE_BASE_u	nDEINTL_BASE3C;			//	0x0CC
	unsigned	int			nDEINTL_CUR_BASE0;		//	0x0D0
	unsigned	int			nDEINTL_CUR_BASE1;		//	0x0D4
	unsigned	int			nDEINTL_CUR_BASE2;		//	0x0D8
	unsigned	int			nDEINTL_CUR_BASE3;		//	0x0DC
	unsigned	int			nDEINTL_CUR_WDMA;			//	0x0E0
	unsigned	int			nDEINTL_CUR_RDMA;			//	0x0E4
	unsigned	int			undef_0x0b8[6];			//	0x0E8 ~ 0x0FC
}	VIQE_DEINTL_DMA,*PVIQE_DEINTL_DMA;

typedef	struct
{
	volatile VIQE_FC_MISC_u			nVIQE_FC_MISC;			// 0x100
	volatile VIQE_SIZE_u			nVIQE_FC_SIZE;			// 0x104
	volatile VIQE_FC_STATUS_u		nVIQE_FC_STATUS;		// 0x108
	volatile VIQE_FC_CTRL_u			nVIQE_FC_CTRL;			// 0x10C
	unsigned	int					undef0[4];				// 0x110~0x11C
}	VIQE_FC;

typedef	struct
{
	volatile VIQE_FC_MISC_u			nVIQE_FC_MISC;			// 0x160
	unsigned	int					undef_0x164;			// 0x164
	volatile VIQE_FC_STATUS_u		nVIQE_FC_STATUS;		// 0x168
	volatile VIQE_FC_CTRL_u			nVIQE_FC_CTRL;			// 0x16C
	volatile VIQE_FC_LIMIT_u		nVIQE_FC_LIMIT;		// 0x170
	unsigned	int					undef0[3];				// 0x174~0x17C
}	VIQE_FC_COMP;

typedef struct
{
	volatile unsigned int 		nDI_CTRL;			//0x280 
	volatile unsigned int 		nDI_ENGINE0;		//0x284 
	volatile unsigned int 		nDI_ENGINE1;		//0x288 
	volatile unsigned int 		nDI_ENGINE2;		//0x28C 
	volatile unsigned int 		nDI_ENGINE3;		//0x290 
	volatile unsigned int 		nDI_ENGINE4;		//0x294 
	volatile unsigned int 		nPD_THRES0;		//0x298 
	volatile unsigned int 		nPD_THRES1;		//0x29C 
	volatile unsigned int 		nPD_JUDDER;		//0x2A0 
	volatile unsigned int 		nPD_JUDDER_M;		//0x2A4 
	volatile unsigned int 		nDI_MISCC;			//0x2A8 
	volatile unsigned int 		nDI_STATUS;		//0x2AC 
	volatile unsigned int 		nPD_STATUS;		//0x2B0 
	volatile unsigned int 		nDI_REGION0;		//0x2B4 
	volatile unsigned int 		nDI_REGION1;		//0x2B8 
	volatile unsigned int 		nDI_INT;			//0x2BC 
	volatile unsigned			undef_0[8];		//  0x2C0 ~ 0x2DC (deintl)    	
	volatile unsigned int 		nPD_SAW;			//0x2E0 
	volatile unsigned int 		nDI_CSIZE;			//0x2E4 
	volatile unsigned int 		nDI_FMT;			//0x2E8 
	volatile unsigned			undef_1[5];		//  0x2EC ~ 0x2FC (deintl)    
	
}	VIQE_DEINTL;

typedef	struct _VIOC_VIQE
{
	volatile VIOC_VIQE_CTRL				cVIQE_CTRL;			//	0x000 ~ 0x07C
	volatile VIQE_DEINTL_DMA			cDEINTL_DMA;			//	0x080 ~ 0x0FC
	volatile VIQE_FC					cDEINTL_DECOMP0;		//  0x100 ~ 0x11C
	volatile VIQE_FC					cDEINTL_DECOMP1;		//	0x120 ~ 0x13C
	volatile VIQE_FC					cDEINTL_DECOMP2;		//	0x140 ~ 0x15C	
	volatile VIQE_FC_COMP				cDEINTL_COMP;			//	0x160 ~ 0x17C
	unsigned	int						undef_0[64];			//	0x180 ~ 0x27C
	volatile VIQE_DEINTL				cDEINTL;				//  0x280 ~ 0x2FC (deintl)	
}	VIQE,*PVIQE;

/* Interface APIs */
extern void VIOC_VIQE_InitDeintlCoreTemporal(VIQE *pVIQE); 
extern void VIOC_VIQE_SetImageSize(VIQE *pVIQE, unsigned int width, unsigned int height);
extern void VIOC_VIQE_SetImageY2RMode(VIQE *pVIQE, unsigned int y2r_mode);
extern void VIOC_VIQE_SetImageY2REnable(VIQE *pVIQE, unsigned int enable);
extern void VIOC_VIQE_SetControlMisc(VIQE *pVIQE, unsigned int no_hor_intpl, unsigned int fmt_conv_disable, unsigned int fmt_conv_disable_using_fmt, unsigned int update_disable, unsigned int cfgupd, unsigned int h2h);
extern void VIOC_VIQE_SetControlDontUse(VIQE *pVIQE, unsigned int global_en_dont_use, unsigned int top_size_dont_use, unsigned int stream_deintl_info_dont_use);
extern void VIOC_VIQE_SetControlClockGate(VIQE *pVIQE, unsigned int deintl_dis, unsigned int d3d_dis, unsigned int pm_dis);
extern void VIOC_VIQE_SetControlEnable(VIQE *pVIQE, unsigned int his_cdf_or_lut_en, unsigned int his_en, unsigned int gamut_en, unsigned int denoise3d_en, unsigned int deintl_en);
extern void VIOC_VIQE_SetControlMode(VIQE *pVIQE, unsigned int his_cdf_or_lut_en, unsigned int his_en, unsigned int gamut_en, unsigned int denoise3d_en, unsigned int deintl_en);
extern void VIOC_VIQE_SetControlRegister(VIQE *pVIQE, unsigned int width, unsigned int height, unsigned int fmt);

extern void VIOC_VIQE_SetDeintlBase(VIQE *pVIQE, unsigned int frmnum, unsigned int base0, unsigned int base1, unsigned int base2, unsigned int base3);
extern void VIOC_VIQE_SwapDeintlBase(VIQE *pVIQE, int mode);
extern void VIOC_VIQE_SetDeintlSize(VIQE *pVIQE, unsigned int width, unsigned int height);
extern void VIOC_VIQE_SetDeintlMisc(VIQE *pVIQE, unsigned int uvintpl, unsigned int cfgupd, unsigned int dma_enable, unsigned int h2h, unsigned int top_size_dont_use);
extern void VIOC_VIQE_SetDeintlControl(VIQE *pVIQE, unsigned int fmt, unsigned int eof_control_ready, unsigned int dec_divisor, unsigned int ac_k0_limit, unsigned int ac_k1_limit, unsigned int ac_k2_limit);
extern void VIOC_VIQE_SetDeintlFMT(VIQE *pVIQE, int enable);
extern void VIOC_VIQE_SetDeintlMode(VIQE *pVIQE, VIOC_VIQE_DEINTL_MODE mode);
extern void VIOC_VIQE_SetDeintlRegion(VIQE *pVIQE, int region_enable, int region_idx_x_start, int region_idx_x_end, int region_idx_y_start, int region_idx_y_end);
extern void VIOC_VIQE_SetDeintlCore(VIQE *pVIQE, unsigned int width, unsigned int height, VIOC_VIQE_FMT_TYPE fmt, unsigned int bypass, unsigned int top_size_dont_use);
extern void VIOC_VIQE_SetDeintlRegister(VIQE *pVIQE, unsigned int fmt, unsigned int top_size_dont_use, unsigned int width, unsigned int height, VIOC_VIQE_DEINTL_MODE mode, unsigned int base0, unsigned int base1, unsigned int base2, unsigned int base3);
extern void VIOC_VIQE_SetDeintlJudderCnt(VIQE *pVIQE, unsigned int cnt);
extern VIQE* VIOC_VIQE_GetAddress(void);
extern void VIOC_VIQE_InitDeintlCoreVinMode(VIQE *pVIQE);
extern void VIOC_VIQE_SetY2R(VIQE *pVIQE, unsigned int uiEnableY2R);
extern void VIOC_VIQE_DUMP(VIQE *pVIQE);
#endif //__VIOC_VIQE_H__

