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

#include  <mach/reg_physical.h>

typedef enum
{
	VIOC_IREQ_DEV0 = 0, 	/* IREQ0 */
	VIOC_IREQ_DEV1,
	VIOC_IREQ_DEV2,
	VIOC_IREQ_TIMER,
	VIOC_IREQ_RDMA00,
	VIOC_IREQ_RDMA01,
	VIOC_IREQ_RDMA02,
	VIOC_IREQ_RDMA03,
	VIOC_IREQ_RDMA04,
	VIOC_IREQ_RDMA05,
	VIOC_IREQ_RDMA06,
	VIOC_IREQ_RDMA07,
	VIOC_IREQ_RDMA08,
	VIOC_IREQ_RDMA09,
	VIOC_IREQ_RDMA10,
	VIOC_IREQ_RDMA11,
	VIOC_IREQ_RDMA12,
	VIOC_IREQ_RDMA13,
	VIOC_IREQ_RDMA14,
	VIOC_IREQ_RDMA15,
	VIOC_IREQ_RDMA16,
	VIOC_IREQ_RDMA17,
	VIOC_IREQ_RESERVE0,
	VIOC_IREQ_RESERVE1,
	VIOC_IREQ_MMU,
	VIOC_IREQ_RESERVE2,
	VIOC_IREQ_RESERVE3,
	VIOC_IREQ_RESERVE4,
	VIOC_IREQ_FIFO0,
	VIOC_IREQ_FIFO1,
	VIOC_IREQ_RESERVE5,
	VIOC_IREQ_RESERVE6,
	VIOC_IREQ_WDMA00 = 32, 	/* IREQ1 */
	VIOC_IREQ_WDMA01,
	VIOC_IREQ_WDMA02,
	VIOC_IREQ_WDMA03,
	VIOC_IREQ_WDMA04,
	VIOC_IREQ_WDMA05,
	VIOC_IREQ_WDMA06,
	VIOC_IREQ_WDMA07,
	VIOC_IREQ_WDMA08,
	VIOC_IREQ_RESERVE7,
	VIOC_IREQ_RESERVE8,
	VIOC_IREQ_RESERVE9,
	VIOC_IREQ_RESERVE10,
	VIOC_IREQ_RESERVE11,
	VIOC_IREQ_RESERVE12,
	VIOC_IREQ_RESERVE13,
	VIOC_IREQ_WMIX0,
	VIOC_IREQ_WMIX1,
	VIOC_IREQ_WMIX2,
	VIOC_IREQ_WMIX3,
	VIOC_IREQ_WMIX4,
	VIOC_IREQ_WMIX5,
	VIOC_IREQ_RESERVE14,
	VIOC_IREQ_RESERVE15,
	VIOC_IREQ_RESERVE16,
	VIOC_IREQ_RESERVE17,
	VIOC_IREQ_RESERVE18,
	VIOC_IREQ_VIQE,
	VIOC_IREQ_SC0,
	VIOC_IREQ_SC1,
	VIOC_IREQ_SC2,
	VIOC_IREQ_SC3,
	VIOC_IREQ_MAX
} VIOC_IREQ_VECTOR_ID_Type;

typedef	struct 
{
	unsigned				DEV0	: 1;
	unsigned				DEV1	: 1;
	unsigned				DEV2	: 1;
	unsigned				TIMER	: 1;
	unsigned				RDMA00	: 1;
	unsigned				RDMA01	: 1;
	unsigned				RDMA02	: 1;
	unsigned				RDMA03	: 1;
	unsigned				RDMA04	: 1;
	unsigned				RDMA05	: 1;
	unsigned				RDMA06	: 1;
	unsigned				RDMA07	: 1;
	unsigned				RDMA08	: 1;
	unsigned				RDMA09	: 1;
	unsigned				RDMA10	: 1;
	unsigned				RDMA11	: 1;
	unsigned				RDMA12	: 1;
	unsigned				RDMA13	: 1;
	unsigned				RDMA14	: 1;
	unsigned				RDMA15	: 1;
	unsigned				RDMA16	: 1;
	unsigned				RDMA17	: 1;
	unsigned 						: 2;
	unsigned				MMU		: 1;
	unsigned 						: 3;
	unsigned				FIFO0	: 1;
	unsigned				FIFO1	: 1;
	unsigned 						: 2;	
	unsigned				WDMA00	: 1;
	unsigned				WDMA01	: 1;
	unsigned				WDMA02	: 1;
	unsigned				WDMA03	: 1;
	unsigned				WDMA04	: 1;
	unsigned				WDMA05	: 1;
	unsigned				WDMA06	: 1;
	unsigned				WDMA07	: 1;
	unsigned				WDMA08	: 1;
	unsigned 						: 7;
	unsigned				WMIX0	: 1;
	unsigned				WMIX1	: 1;
	unsigned				WMIX2	: 1;
	unsigned				WMIX3	: 1;
	unsigned				WMIX4	: 1;
	unsigned				WMIX5	: 1;
	unsigned 						: 5;
	unsigned				VIQE	: 1;
	unsigned				SC0		: 1;
	unsigned				SC1		: 1;
	unsigned				SC2		: 1;
	unsigned				SC3		: 1;
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
	unsigned						: 18;
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
	unsigned							: 1;
	unsigned				FRAMEFIFO	: 1;
	unsigned				SCALER0		: 1;
	unsigned				SCALER1		: 1;
	unsigned				SCALER2		: 1;
	unsigned				SCALER3		: 1;		// 32	
	unsigned				WDMA00		: 1;
	unsigned				WDMA01		: 1;
	unsigned				WDMA02		: 1;
	unsigned				WDMA03		: 1;
	unsigned				WDMA04		: 1;
	unsigned				WDMA05		: 1;
	unsigned				WDMA06		: 1;
	unsigned				WDMA07		: 1;
	unsigned				WDMA08		: 1;
	unsigned							: 1;
	unsigned				WMIX0 		: 1;
	unsigned				WMIX1 		: 1;
	unsigned				WMIX2 		: 1;
	unsigned				WMIX3 		: 1;
	unsigned				WMIX4 		: 1;
	unsigned				WMIX5 		: 1;
	unsigned				VIQE 		: 1;
	unsigned				DEINTLS		: 1;
	unsigned							: 2;
	unsigned				DEV0 		: 1;
	unsigned				DEV1 		: 1;
	unsigned				DEV2 		: 1;
	unsigned				MMU  		: 1;
	unsigned				FILT2D 		: 1;
	unsigned				DEBLOCK		: 1;
	unsigned				FDLY0 		: 1;
	unsigned				FDLY1 		: 1;
	unsigned							: 2;
	unsigned							: 2;
}	VIOC_POWER_BLOCKS;

typedef	union {
	unsigned long		nREG[2];
	VIOC_POWER_BLOCKS	bREG;
}	VIOC_POWER_BLOCKS_u;

typedef	struct	_VIOC_IREQ_CONFIG
{
	volatile VIOC_IREQ_IREQ_u 			uRAWSTATUS;			// 0x00~0x04
	volatile VIOC_IREQ_IREQ_u 			uSYNCSTATUS;		// 0x08~0x0C
	volatile VIOC_IREQ_IREQ_u 			uIREQSELECT;		// 0x10~0x14
	volatile VIOC_IREQ_IREQ_u 			nIRQMASKSET;		// 0x18~0x1C
	volatile VIOC_IREQ_IREQ_u 			nIRQMASKCLR;		// 0x20~0x24
	volatile VIOC_IREQ_VECTORID_u 		nVECTORID;			// 0x28
	unsigned int 						reserved0[3];		// 0x2C~0x34
	unsigned int 						TEST_LOOG;			// 0x38
	volatile  VIOC_CONFIG_PATH_u 		uMC;			// 0x3C
	volatile VIOC_CONFIG_ETC_u 		uMISC;				// 0x40
	volatile VIOC_CONFIG_PATH_u 		uSC0;				// 0x44
	volatile VIOC_CONFIG_PATH_u 		uSC1;				// 0x48
	volatile VIOC_CONFIG_PATH_u 		uSC2;				// 0x4C
	volatile VIOC_CONFIG_PATH_u 		uSC3;				// 0x50
	volatile VIOC_CONFIG_PATH_u 		uVIQE;				// 0x54
	volatile VIOC_CONFIG_PATH_u 		uDEINTLS;			// 0x58
	unsigned int 						reserved1;			// 0x5C
	volatile VIOC_CONFIG_PATH_u 		uFCDEC0;			// 0x60
	volatile VIOC_CONFIG_PATH_u 		uFCDEC1;			// 0x64
	unsigned int 						reserved2[2];		// 0x68~0x6C
	volatile VIOC_CONFIG_PATH_u 		uFCENC0;			// 0x70
	volatile VIOC_CONFIG_PATH_u 		uFCENC1;			// 0x74
	volatile VIOC_CONFIG_PATH_u 		uFDELAY0;			// 0x78
	volatile VIOC_CONFIG_PATH_u 		uFDELAY1;			// 0x7C
	unsigned int 						reserved3;			// 0x80
	volatile VIOC_CONFIG_ETC_u 			uMISC1;				// 0x84
	unsigned int 						reserved4[14];		// 0x88~0xBC
	unsigned int						nARID;				// 0xC0
	unsigned int						nAWID;				// 0xC4
	volatile VIOC_POWER_AUTOPD_u 		uAUTOPD;			// 0xC8
	volatile VIOC_POWER_CLKCTRL_u 		uCLKCTRL;			// 0xCC
	volatile VIOC_POWER_BLOCKS_u 		uPOWERDOWN;			// 0xD0~0xD4
	volatile VIOC_POWER_BLOCKS_u 		uSOFTRESET;			// 0xD8~0xDC	
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
