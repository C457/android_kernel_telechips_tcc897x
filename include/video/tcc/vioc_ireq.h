/*
 * linux/arch/arm/mach-tcc893x/include/mach/vioc_ireq.h
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

#ifndef __VIOC_IREQ_H__
#define	__VIOC_IREQ_H__

#include  "tcc_video_regs.h"

typedef enum
{
	VIOC_IREQ_DEV0   = 0,
	VIOC_IREQ_DEV1   = 1,
	VIOC_IREQ_DEV2   = 2,
	VIOC_IREQ_TIMER  = 3,
	VIOC_IREQ_RDMA00 = 4,
	VIOC_IREQ_RDMA01 = 5,
	VIOC_IREQ_RDMA02 = 6,
	VIOC_IREQ_RDMA03 = 7,
	VIOC_IREQ_RDMA04 = 8,
	VIOC_IREQ_RDMA05 = 9,
	VIOC_IREQ_RDMA06 = 10,
	VIOC_IREQ_RDMA07 = 11,
	VIOC_IREQ_RDMA08 = 12,
	VIOC_IREQ_RDMA09 = 13,
	VIOC_IREQ_RDMA10 = 14,
	VIOC_IREQ_RDMA11 = 15,
	VIOC_IREQ_RDMA12 = 16,
	VIOC_IREQ_RDMA13 = 17,
	VIOC_IREQ_RDMA14 = 18,
	VIOC_IREQ_RDMA15 = 19,
	VIOC_IREQ_RDMA16 = 20,
	VIOC_IREQ_RDMA17 = 21,
	VIOC_IREQ_MC0    = 22,
	VIOC_IREQ_MC1    = 23,
	VIOC_IREQ_MMU    = 24,
	VIOC_IREQ_DTRC0  = 25,
	VIOC_IREQ_DTRC1  = 26,
	VIOC_IREQ_RESERVED_27,
	VIOC_IREQ_FIFO0  = 28,
	VIOC_IREQ_FIFO1  = 29,
	VIOC_IREQ_RESERVED_30,
	VIOC_IREQ_RESERVED_31,
	VIOC_IREQ_WDMA00 = 32,
	VIOC_IREQ_WDMA01 = 33,
	VIOC_IREQ_WDMA02 = 34,
	VIOC_IREQ_WDMA03 = 35,
	VIOC_IREQ_WDMA04 = 36,
	VIOC_IREQ_WDMA05 = 37,
	VIOC_IREQ_WDMA06 = 38,
	VIOC_IREQ_WDMA07 = 39,
	VIOC_IREQ_WDMA08 = 40,
	VIOC_IREQ_RESERVED_41,
	VIOC_IREQ_RESERVED_42,
	VIOC_IREQ_SC4    = 43,
	VIOC_IREQ_RESERVED_44,
	VIOC_IREQ_VIQE1  = 45,
	VIOC_IREQ_RESERVED_46,
	VIOC_IREQ_RESERVED_47,
	VIOC_IREQ_WMIX0  = 48,
	VIOC_IREQ_WMIX1  = 49,
	VIOC_IREQ_WMIX2  = 50,
	VIOC_IREQ_WMIX3  = 51,
	VIOC_IREQ_WMIX4  = 52,
	VIOC_IREQ_WMIX5  = 53,
	VIOC_IREQ_WMIX6  = 54,
	VIOC_IREQ_VIN0   = 55,
	VIOC_IREQ_VIN1   = 56,
	VIOC_IREQ_VIN2   = 57,
	VIOC_IREQ_VIN3   = 58,
	VIOC_IREQ_VIQE   = 59,
	VIOC_IREQ_SC0    = 60,
	VIOC_IREQ_SC1    = 61,
	VIOC_IREQ_SC2    = 62,
	VIOC_IREQ_SC3    = 63,
	VIOC_IREQ_MAX    = VIOC_IREQ_SC3
} VIOC_IREQ_VECTOR_ID_Type;

typedef	struct 
{
	unsigned				DEV0	: 1;	// 0
	unsigned				DEV1	: 1;	// 1
	unsigned				DEV2	: 1;	// 2
	unsigned				TIMER	: 1;	// 3
	unsigned				RDMA00	: 1;	// 4
	unsigned				RDMA01	: 1;	// 5
	unsigned				RDMA02	: 1;	// 6
	unsigned				RDMA03	: 1;	// 7
	unsigned				RDMA04	: 1;	// 8
	unsigned				RDMA05	: 1;	// 9
	unsigned				RDMA06	: 1;	// 10
	unsigned				RDMA07	: 1;	// 11
	unsigned				RDMA08	: 1;	// 12
	unsigned				RDMA09	: 1;	// 13
	unsigned				RDMA10	: 1;	// 14
	unsigned				RDMA11	: 1;	// 15
	unsigned				RDMA12	: 1;	// 16
	unsigned				RDMA13	: 1;	// 17
	unsigned				RDMA14	: 1;	// 18
	unsigned				RDMA15	: 1;	// 19
	unsigned				RDMA16	: 1;	// 20
	unsigned				RDMA17	: 1;	// 21
	unsigned 				MC0		: 1;	// 22
	unsigned 				MC1		: 1;	// 23
	unsigned				MMU		: 1;	// 24
	unsigned 				DTRC0	: 1;	// 25
	unsigned 				DTRC1	: 1;	// 26
	unsigned 						: 1;	// 27
	unsigned				FIFO0	: 1;	// 28
	unsigned				FIFO1	: 1;	// 29
	unsigned 						: 2;	// 30, 31
	unsigned				WDMA00	: 1;	// 32
	unsigned				WDMA01	: 1;	// 33
	unsigned				WDMA02	: 1;	// 34
	unsigned				WDMA03	: 1;	// 35
	unsigned				WDMA04	: 1;	// 36
	unsigned				WDMA05	: 1;	// 37
	unsigned				WDMA06	: 1;	// 38
	unsigned				WDMA07	: 1;	// 39
	unsigned				WDMA08	: 1;	// 40
	unsigned 						: 2;	// 41, 42
	unsigned				SC4		: 1;	// 43
	unsigned 						: 1;	// 44
	unsigned 				VIQE1	: 1;	// 45
	unsigned 						: 2;	// 46, 47
	unsigned				WMIX0	: 1;	// 48
	unsigned				WMIX1	: 1;	// 49
	unsigned				WMIX2	: 1;	// 50
	unsigned				WMIX3	: 1;	// 51
	unsigned				WMIX4	: 1;	// 52
	unsigned				WMIX5	: 1;	// 53
	unsigned				WMIX6	: 1;	// 54
	unsigned				VIN0	: 1;	// 55
	unsigned				VIN1	: 1;	// 56
	unsigned				VIN2	: 1;	// 57
	unsigned				VIN3	: 1;	// 58
	unsigned				VIQE	: 1;	// 59
	unsigned				SC0		: 1;	// 60
	unsigned				SC1		: 1;	// 61
	unsigned				SC2		: 1;	// 62
	unsigned				SC3		: 1;	// 63
}	VIOC_IREQ_IREQ;

typedef	union {
	unsigned long	nREG[2];
	VIOC_IREQ_IREQ	bREG;
}	VIOC_IREQ_IREQ_u;

typedef struct {
	unsigned 				INDEX	: 6;
	unsigned 		 				: 10;
	unsigned 						: 15;
	unsigned 				IVALID	: 1 ;
}VIOC_IREQ_VECTORID;

typedef union
{
	unsigned long		nREG;
	VIOC_IREQ_VECTORID	bREG;
}VIOC_IREQ_VECTORID_u;

typedef struct {
	unsigned 				LVIN0	: 2;
	unsigned 				LVIN1	: 2;
	unsigned 				LVIN2	: 2;
	unsigned 				LVIN3	: 2;
	unsigned 		 				: 24;
} VIOC_CONFIG_TEST_LOOP;

typedef union
{
	unsigned long			nREG;
	VIOC_CONFIG_TEST_LOOP	bREG;
} VIOC_CONFIG_TEST_LOOP_u;

typedef struct {
	unsigned 			RDMA03		: 1;
	unsigned 			RDMA07		: 1;
	unsigned 			RDMA11		: 1;
	unsigned 			RDMA13		: 1;
	unsigned 			RDMA15		: 1;
	unsigned 			RDMA16		: 1;
	unsigned 			RDMA17		: 1;
	unsigned 						: 9;
	unsigned 			MC1_SEL		: 4;
	unsigned 			MC0_SEL		: 4;
	unsigned 			DTRC0_SEL	: 4;
	unsigned 			DRTC1_SEL	: 4;
} VIOC_CONFIG_PATH_MC;

typedef union
{
	unsigned long			nREG;
	VIOC_CONFIG_PATH_MC		bREG;
} VIOC_CONFIG_PATH_MC_u;

typedef struct 
{
	unsigned 				L0EVSEL	:  3;				// TCC8910 | TCC8010
	unsigned 						:  1;
	unsigned 				L1EVSEL	:  3;				// TCC8910
	unsigned 						:  1;
	unsigned 				L2EVSEL	:  3;				// TCC8910
	unsigned 						:  1;
	unsigned 						:  4;
	unsigned 				WMIX0_0	:  1;	// 16		// TCC8910 | TCC8010
	unsigned 				WMIX0_1	:  1;	// 17		// TCC8910 | TCC8010
	unsigned 				WMIX1_0	:  1;	// 18		// TCC8910 | TCC8010
	unsigned 				WMIX1_1	:  1;	// 19		// TCC8910 | TCC8010
	unsigned 				WMIX2_0	:  1;	// 20		// TCC8910 | TCC8010
	unsigned 				WMIX2_1	:  1;	// 21		// TCC8910 | TCC8010
	unsigned 				WMIX3_0	:  1;	// 22		// TCC8910
	unsigned 				WMIX3_1	:  1;	// 23		// TCC8910
	unsigned 				WMIX4_0	:  1;	// 24		// TCC8910
	unsigned 				WMIX4_1	:  1;	// 25		// TCC8910
	unsigned 				WMIX5_0	:  1;	// 26		// TCC8910
	unsigned 				WMIX5_1	:  1;	// 27		// TCC8910
	unsigned 				WMIX6_0	:  1;	// 28		// TCC8910
	unsigned 				WMIX6_1	:  1;	// 29		// TCC8910
	unsigned 				RDMA12	:  1;	// 30
	unsigned 				RDMA14	:  1;	// 31
}VIOC_CONFIG_ETC;

typedef	union {
	unsigned long		nREG;
	VIOC_CONFIG_ETC		bREG;
}	VIOC_CONFIG_ETC_u;

typedef struct {
	unsigned 			AXIWD		:  1;
	unsigned 						:  7;
	unsigned 			AXIRD		:  1;
	unsigned 						:  3;
	unsigned 			AXIRD_M0_TR	:  2;
	unsigned 			AXIRD_M1_TR	:  2;
	unsigned 						:  7;
	unsigned 			S_REQ		:  1;
	unsigned 			LCD0_SEL	:  2;
	unsigned 			LCD1_SEL	:  2;
	unsigned 			LCD2_SEL	:  2;
	unsigned 						:  2;
} VIOC_CONFIG_MISC1;

typedef	union {
	unsigned long		nREG;
	VIOC_CONFIG_MISC1	bREG;
} VIOC_CONFIG_MISC1_u;

typedef struct {
	unsigned 			VIN0_PATH	:  2;
	unsigned 						:  5;
	unsigned 			VIN0_EN		:  1;

	unsigned 			VIN1_PATH	:  2;
	unsigned 						:  5;
	unsigned 			VIN1_EN		:  1;

	unsigned 			VIN2_PATH	:  2;
	unsigned 						:  5;
	unsigned 			VIN2_EN		:  1;

	unsigned 			VIN3_PATH	:  2;
	unsigned 						:  5;
	unsigned 			VIN3_EN		:  1;
} VIOC_CONFIG_VIN_SEL;

typedef	union {
	unsigned long		nREG;
	VIOC_CONFIG_VIN_SEL	bREG;
} VIOC_CONFIG_VIN_SEL_u;

typedef struct {
	unsigned 			DEV0_PATH	:  2;
	unsigned 						:  5;
	unsigned 			DEV0_EN		:  1;

	unsigned 			DEV1_PATH	:  2;
	unsigned 						:  5;
	unsigned 			DEV1_EN		:  1;

	unsigned 			DEV2_PATH	:  2;
	unsigned 						:  5;
	unsigned 			DEV2_EN		:  1;

	unsigned 						:  8;
} VIOC_CONFIG_DEV_SEL;

typedef	union {
	unsigned long		nREG;
	VIOC_CONFIG_DEV_SEL	bREG;
} VIOC_CONFIG_DEV_SEL_u;

typedef struct {
	unsigned 				SELECT    	:  8;
	unsigned 							:  8;
	unsigned 				STATUS     	:  2;
	unsigned 				ERR      	:  1;
	unsigned 							:  5;
	unsigned 							:  7;
	unsigned 				EN		     	:  1;
}VIOC_CONFIG_PATH;

typedef	union {
	unsigned long			nREG;
	VIOC_CONFIG_PATH		bREG;
}VIOC_CONFIG_PATH_u;

typedef	struct {
	unsigned				RDMA 	: 1;
	unsigned				SCALER	: 1;
	unsigned				MIXER	: 1;
	unsigned				WDMA	: 1;
	unsigned				FDLY	: 1;
	unsigned				VIQE	: 1;
	unsigned				DEINTLS	: 1;
	unsigned				FILT2D	: 1;
	unsigned				FCENC	: 1;
	unsigned				FCDEC	: 1;
	unsigned				CPUIF	: 1;
	unsigned				LCD2AHB	: 1;
	unsigned				VIDEOIN	: 1;
	unsigned				DEBLOCK	: 1;
	unsigned				DEVMX	: 1;
	unsigned				MC		: 1;
	unsigned				DTRC	: 1;
	unsigned						: 15;
}	VIOC_POWER_AUTOPD;

typedef	union {
	unsigned long		nREG;
	VIOC_POWER_AUTOPD	bREG;
}	VIOC_POWER_AUTOPD_u;

typedef	struct {
	unsigned				MIN			: 4;
	unsigned							: 4;
	unsigned				EN			: 1;
	unsigned				PROF_EN		: 1;
	unsigned							: 5;
	unsigned							: 1;
	unsigned							: 16;
}	VIOC_POWER_CLKCTRL;

typedef	union {
	unsigned long		nREG;
	VIOC_POWER_CLKCTRL	bREG;
}	VIOC_POWER_CLKCTRL_u;

typedef	struct {
	unsigned				RDMA00		: 1;
	unsigned				RDMA01		: 1;
	unsigned				RDMA02		: 1;
	unsigned				RDMA03		: 1;
	unsigned				RDMA04		: 1;
	unsigned				RDMA05		: 1;
	unsigned				RDMA06		: 1;
	unsigned				RDMA07		: 1;
	unsigned				RDMA08		: 1;
	unsigned				RDMA09		: 1;
	unsigned				RDMA10		: 1;
	unsigned				RDMA11		: 1;
	unsigned				RDMA12		: 1;
	unsigned				RDMA13		: 1;
	unsigned				RDMA14		: 1;
	unsigned				RDMA15		: 1;
	unsigned				RDMA16		: 1;
	unsigned				RDMA17		: 1;
	unsigned				FCDEC0		: 1;
	unsigned				FCDEC1		: 1;
	unsigned				FCDEC2		: 1;
	unsigned				FCDEC3		: 1;
	unsigned				FCENC0		: 1;
	unsigned				FCENC1		: 1;
	unsigned				VIDEOIN0	: 1;
	unsigned				VIDEOIN1	: 1;
	unsigned				VIDEOIN2	: 1;
	unsigned				VIDEOIN3	: 1;
	unsigned				SCALER0		: 1;
	unsigned				SCALER1		: 1;
	unsigned				SCALER2		: 1;
	unsigned				SCALER3		: 1;	// bit 31
	unsigned				WDMA00		: 1;	// bit 0
	unsigned				WDMA01		: 1;
	unsigned				WDMA02		: 1;
	unsigned				WDMA03		: 1;
	unsigned				WDMA04		: 1;
	unsigned				WDMA05		: 1;
	unsigned				WDMA06		: 1;
	unsigned				WDMA07		: 1;
	unsigned				WDMA08		: 1;	// bit 8
	unsigned				WMIX0 		: 1;
	unsigned				WMIX1 		: 1;
	unsigned				WMIX2 		: 1;
	unsigned				WMIX3 		: 1;
	unsigned				WMIX4 		: 1;
	unsigned				WMIX5 		: 1;
	unsigned				WMIX6 		: 1;
	unsigned				VIQE 		: 1;	// bit 16
	unsigned				DEINTLS		: 1;
	unsigned				VIQE1		: 1;
	unsigned				DTRC0		: 1;
	unsigned				DEV0 		: 1;
	unsigned				DEV1 		: 1;
	unsigned				DEV2 		: 1;
	unsigned				MMU  		: 1;
	unsigned				FILT2D 		: 1;	// bit 24
	unsigned				DEBLOCK		: 1;
	unsigned				MC0			: 1;
	unsigned				MC1			: 1;
	unsigned				FDLY0 		: 1;
	unsigned				FDLY1 		: 1;
	unsigned				DTRC1		: 1;
	unsigned				FIFO		: 1;	// bit 31
}	VIOC_POWER_BLOCKS;

typedef	union {
	unsigned long		nREG[2];
	VIOC_POWER_BLOCKS	bREG;
}	VIOC_POWER_BLOCKS_u;

typedef	struct {
	unsigned				FCENC0		: 1;
	unsigned				FCENC1		: 1;
	unsigned							: 6;
	unsigned				FCDEC0		: 1;
	unsigned				FCDEC1		: 1;
	unsigned				FCDEC2		: 1;
	unsigned				FCDEC3		: 1;
	unsigned							: 4;
	unsigned				SC0			: 1;
	unsigned				SC1			: 1;
	unsigned				SC2			: 1;
	unsigned				SC3			: 1;
	unsigned				SC4			: 1;
	unsigned							:11;
} VIOC_POWER_BLOCKS2;

typedef	union {
	unsigned long		nREG;
	VIOC_POWER_BLOCKS2	bREG;
} VIOC_POWER_BLOCKS2_u;

typedef	struct {
	unsigned		PORT_FIX_WDMA0		: 1;
	unsigned		PORT_FIX_WDMA1		: 1;
	unsigned		PORT_FIX_WDMA2		: 1;
	unsigned		PORT_FIX_WDMA3		: 1;
	unsigned		PORT_FIX_WDMA4		: 1;
	unsigned		PORT_FIX_WDMA5		: 1;
	unsigned		PORT_FIX_WDMA6		: 1;
	unsigned		PORT_FIX_WDMA7		: 1;
	unsigned		PORT_FIX_WDMA8		: 1;
	unsigned		PORT_FIX_NOT_USED0	: 1;
	unsigned		PORT_FIX_NOT_USED1	: 1;
	unsigned		PORT_FIX_VIQE0		: 1;
	unsigned		PORT_FIX_NOT_USED2	: 1;
	unsigned		PORT_FIX_VIQE1		: 1;
	unsigned		PORT_FIX_NOT_USED3	: 1;
	unsigned							: 1;
	unsigned		PORT_CH_WDMA0		: 1;
	unsigned		PORT_CH_WDMA1		: 1;
	unsigned		PORT_CH_WDMA2		: 1;
	unsigned		PORT_CH_WDMA3		: 1;
	unsigned		PORT_CH_WDMA4		: 1;
	unsigned		PORT_CH_WDMA5		: 1;
	unsigned		PORT_CH_WDMA6		: 1;
	unsigned		PORT_CH_WDMA7		: 1;
	unsigned		PORT_CH_WDMA8		: 1;
	unsigned		PORT_CH_NOT_USED0	: 1;
	unsigned		PORT_CH_NOT_USED1	: 1;
	unsigned		PORT_CH_VIQE0		: 1;
	unsigned		PORT_CH_NOT_USED2	: 1;
	unsigned		PORT_CH_VIQE1		: 1;
	unsigned		PORT_CH_NOT_USED3	: 1;
	unsigned							: 1;
} VIOC_CONFIG_WDMA_MISC;

typedef	union {
	unsigned long			nREG[2];
	VIOC_CONFIG_WDMA_MISC	bREG;
} VIOC_CONFIG_WDMA_MISC_u;

typedef	struct	_VIOC_IREQ_CONFIG
{
	volatile VIOC_IREQ_IREQ_u 			uRAWSTATUS;			// 0x00, 0x04
	volatile VIOC_IREQ_IREQ_u 			uSYNCSTATUS;		// 0x08, 0x0C
	volatile VIOC_IREQ_IREQ_u 			uIREQSELECT;		// 0x10, 0x14
	volatile VIOC_IREQ_IREQ_u 			nIRQMASKSET;		// 0x18, 0x1C
	volatile VIOC_IREQ_IREQ_u 			nIRQMASKCLR;		// 0x20, 0x24
	volatile VIOC_IREQ_VECTORID_u 		nVECTORID;			// 0x28
	unsigned int 						reserved0[3];		// 0x2C, 0x30, 0x34

	volatile VIOC_CONFIG_TEST_LOOP_u	TEST_LOOP;			// 0x38
	volatile VIOC_CONFIG_PATH_MC_u 		uMC;				// 0x3C
	volatile VIOC_CONFIG_ETC_u 			uMISC;				// 0x40
	volatile VIOC_CONFIG_PATH_u 		uSC0;				// 0x44
	volatile VIOC_CONFIG_PATH_u 		uSC1;				// 0x48
	volatile VIOC_CONFIG_PATH_u 		uSC2;				// 0x4C
	volatile VIOC_CONFIG_PATH_u 		uSC3;				// 0x50
	volatile VIOC_CONFIG_PATH_u 		uVIQE;				// 0x54
	volatile VIOC_CONFIG_PATH_u 		uDEINTLS;			// 0x58
	unsigned int 						reserved1;			// 0x5C

	volatile VIOC_CONFIG_PATH_u 		uFCDEC0;			// 0x60
	volatile VIOC_CONFIG_PATH_u 		uFCDEC1;			// 0x64
	unsigned int 						reserved2[2];		// 0x68, 0x6C
	volatile VIOC_CONFIG_PATH_u 		uFCENC0;			// 0x70
	volatile VIOC_CONFIG_PATH_u 		uFCENC1;			// 0x74
	volatile VIOC_CONFIG_PATH_u 		uFDELAY0;			// 0x78
	volatile VIOC_CONFIG_PATH_u 		uFDELAY1;			// 0x7C
	unsigned int 						reserved3;			// 0x80
	volatile VIOC_CONFIG_MISC1_u 		uMISC1;				// 0x84

	unsigned int 						reserved4[12];		// 0x88~0xB4
	volatile VIOC_CONFIG_VIN_SEL_u		VIN_SEL;			// 0xB8
	volatile VIOC_CONFIG_DEV_SEL_u		DEV_SEL;			// 0xBC

	unsigned int						nARID;				// 0xC0
	unsigned int						nAWID;				// 0xC4
	volatile VIOC_POWER_AUTOPD_u 		uAUTOPD;			// 0xC8
	volatile VIOC_POWER_CLKCTRL_u 		uCLKCTRL;			// 0xCC
	volatile VIOC_POWER_BLOCKS_u 		uPOWERDOWN;			// 0xD0~0xD4
	volatile VIOC_POWER_BLOCKS_u 		uSOFTRESET;			// 0xD8~0xDC

	unsigned int 						reserved5[6];		// 0xE0~0xF4
	volatile VIOC_CONFIG_PATH_u 		uSC4;				// 0xF8
	unsigned int 						reserved6[11];		// 0xFC~0x124
	volatile VIOC_POWER_BLOCKS2_u 		uPOWERDOWN2;		// 0x128
	volatile VIOC_POWER_BLOCKS2_u 		uSOFTRESET2;			// 0x12C
	volatile VIOC_CONFIG_PATH_u 		uVIQE1;				// 0x130
	unsigned int 						reserved7[3];		// 0x134~13C
	volatile VIOC_CONFIG_WDMA_MISC_u	uWDMA_MISC;			// 0x140
}	VIOC_IREQ_CONFIG,*PVIOC_IREQ_CONFIG;

/* Interface APIs */
extern void VIOC_IREQ_DummyHandler(int index, int irq, void *client_data);
extern void VIOC_IREQ_IrqSyncModeSet(unsigned int select);
extern void VIOC_IREQ_IrqAsyncModeSet(unsigned int select);
extern void VIOC_IREQ_IrqMaskSet(unsigned int mask);
#if 1
extern void VIOC_IREQ_IrqMaskClear(unsigned int mask);
#else
extern void VIOC_IREQ_IrqMaskClear(PVIOC_IREQ_CONFIG pIREQConfig, unsigned int mask0, unsigned int mask1);
#endif
extern void VIOC_IREQ_SetInterruptInit(void);
extern int VIOC_IREQ_RegisterHandle(void);
#endif
