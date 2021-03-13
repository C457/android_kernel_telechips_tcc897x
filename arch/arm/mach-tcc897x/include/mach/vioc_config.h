/*
 * linux/arch/arm/mach-tcc893x/include/mach/vioc_config.h
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

#ifndef __VIOC_CONFIG_H__
#define	__VIOC_CONFIG_H__

#include  <mach/vioc_ireq.h>
#include  <mach/reg_physical.h>

typedef struct {
	unsigned VIOC		:  1;
	unsigned NTSCPAL		:  1;
	unsigned HDMI		:  1;
	unsigned	G2D			:  1; 
	unsigned	RESERVED0	:28;
} DDI_CONFIG_IDX_TYPE;

typedef union {
	unsigned long			nREG;
	DDI_CONFIG_IDX_TYPE		bREG;
} DDI_CONFIG_TYPE;

typedef struct {
	unsigned CIF0			:3;
	unsigned				:1;
	unsigned CIF1			:3;
	unsigned				:1;
	unsigned CIF2			:3;
	unsigned				:1;
	unsigned CIF3			:3;
	unsigned				:1;
	unsigned DEMUX			:3;
	unsigned				:13;
} DDI_CIFPORT_IDX_TYPE;

typedef union {
	unsigned long			nREG;
	DDI_CIFPORT_IDX_TYPE	bREG;
} DDI_CIFPORT_TYPE;

typedef struct {
	unsigned RESET		:4;	// bit 0 : hdmi reset bit 1 : spdif reset  bit 2 : tmds reset bit 3 : not used
	unsigned 	SPDIF_SEL	:2;	//spdif sellect 0:spdif_tx(audio1) 1:spdif_tx(audio0) 2:ext_spdif_tx 3: not used
	unsigned				:2;
	unsigned	CLK_SEL		:2;	//hdmiphy clock sellect 0:ext_clk0 1:ext_clk1 2:internal 27MHz 3:XIN
	unsigned				:5;	
	unsigned EN			:1;
	unsigned	CMU_MODE	:4;	// hdmi phy output : CMU_MODE
	unsigned	VPLL_MODE	:4;	// hdmi phy output : VPLL_CODE
	unsigned CMUL		:1;	// hdmi phy output : CMU_LOCK
	unsigned	SDET		:1; 	// hdmi phy output : SINK_DET
	unsigned VPLLL		:1;	// hdmi phy output : VPLLL
	unsigned 				:5;	
} DDI_HDMICTRL_IDX_TYPE;

typedef union {
	unsigned long			nREG;
	DDI_HDMICTRL_IDX_TYPE	bREG;
} DDI_HDMICTRL_TYPE;

typedef struct {
	unsigned VLD			:1;
	unsigned				:31;
} DDI_HDMIAES_IDX_TYPE;

typedef union {
	unsigned long			nREG;
	DDI_HDMIAES_IDX_TYPE	bREG;
} DDI_HDMIAES_TYPE;

typedef struct {
	unsigned DATA			:32;
} DDI_HDMIAESD0_IDX_TYPE;

typedef union {
	unsigned long			nREG;
	DDI_HDMIAESD0_IDX_TYPE	bREG;
} DDI_HDMIAESD0_TYPE;

typedef struct {
	unsigned DATA			:1;
	unsigned				:31;
} DDI_HDMIAESD1_IDX_TYPE;

typedef union {
	unsigned long			nREG;
	DDI_HDMIAESD1_IDX_TYPE	bREG;
} DDI_HDMIAESD1_TYPE;

typedef struct {
	unsigned HW				:32;
} DDI_HDMIAESHW_IDX_TYPE;

typedef union {
	unsigned long			nREG;
	DDI_HDMIAESHW_IDX_TYPE	bREG;
} DDI_HDMIAESHW_TYPE;

typedef struct {
	unsigned HW				:31;
	unsigned				:1;
} DDI_HDMIAESHW2_IDX_TYPE;

typedef union {
	unsigned long			nREG;
	DDI_HDMIAESHW2_IDX_TYPE	bREG;
} DDI_HDMIAESHW2_TYPE;

typedef struct {
	unsigned SEL			:1;
	unsigned				:31;
} DDI_NTSCPALEN_IDX_TYPE;

typedef union {
	unsigned long			nREG;
	DDI_NTSCPALEN_IDX_TYPE	bREG;
} DDI_NTSCPALEN_TYPE;

typedef struct {
	unsigned 					:1;
	unsigned RST				:1;
	unsigned EN				:1;
	unsigned OC				:1;
	unsigned VSEL			:1;
	unsigned S				:3;
	unsigned M				:7;
	unsigned P				:6;
	unsigned TC				:3;
	unsigned					:6;
	unsigned	SEL				:2;
} DDI_LVDSCTRL_IDX_TYPE;

typedef union {
	unsigned long			nREG;
	DDI_LVDSCTRL_IDX_TYPE	bREG;
} DDI_LVDSCTRL_TYPE;

typedef struct {
	unsigned SEL_00			:8;
	unsigned SEL_01			:8;
	unsigned SEL_02			:8;
	unsigned SEL_03			:8;
} DDI_LVDSTXSEL0_IDX_TYPE;

typedef union {
	unsigned long			nREG;
	DDI_LVDSTXSEL0_IDX_TYPE	bREG;
} DDI_LVDSTXSEL0_TYPE;

typedef struct {
	unsigned SEL_04			:8;
	unsigned SEL_05			:8;
	unsigned SEL_06			:8;
	unsigned SEL_07			:8;
} DDI_LVDSTXSEL1_IDX_TYPE;

typedef union {
	unsigned long			nREG;
	DDI_LVDSTXSEL1_IDX_TYPE	bREG;
} DDI_LVDSTXSEL1_TYPE;

typedef struct {
	unsigned SEL_08			:8;
	unsigned SEL_09			:8;
	unsigned SEL_10			:8;
	unsigned SEL_11			:8;
} DDI_LVDSTXSEL2_IDX_TYPE;

typedef union {
	unsigned long			nREG;
	DDI_LVDSTXSEL2_IDX_TYPE	bREG;
} DDI_LVDSTXSEL2_TYPE;

typedef struct {
	unsigned SEL_12			:8;
	unsigned SEL_13			:8;
	unsigned SEL_14			:8;
	unsigned SEL_15			:8;
} DDI_LVDSTXSEL3_IDX_TYPE;

typedef union {
	unsigned long			nREG;
	DDI_LVDSTXSEL3_IDX_TYPE	bREG;
} DDI_LVDSTXSEL3_TYPE;

typedef struct {
	unsigned SEL_16			:8;
	unsigned SEL_17			:8;
	unsigned SEL_18			:8;
	unsigned SEL_19			:8;
} DDI_LVDSTXSEL4_IDX_TYPE;

typedef union {
	unsigned long			nREG;
	DDI_LVDSTXSEL4_IDX_TYPE	bREG;
} DDI_LVDSTXSEL4_TYPE;

typedef struct {
	unsigned SEL_20			:8;
	unsigned SEL_21			:8;
	unsigned SEL_22			:8;
	unsigned SEL_23			:8;
} DDI_LVDSTXSEL5_IDX_TYPE;

typedef union {
	unsigned long			nREG;
	DDI_LVDSTXSEL5_IDX_TYPE	bREG;
} DDI_LVDSTXSEL5_TYPE;

typedef struct {
	unsigned SEL_24			:8;
	unsigned SEL_25			:8;
	unsigned SEL_26			:8;
	unsigned SEL_27			:8;
} DDI_LVDSTXSEL6_IDX_TYPE;

typedef union {
	unsigned long			nREG;
	DDI_LVDSTXSEL6_IDX_TYPE	bREG;
} DDI_LVDSTXSEL6_TYPE;

typedef struct {
	unsigned SEL_28			:8;
	unsigned SEL_29			:8;
	unsigned SEL_30			:8;
	unsigned SEL_31			:8;
} DDI_LVDSTXSEL7_IDX_TYPE;

typedef union {
	unsigned long			nREG;
	DDI_LVDSTXSEL7_IDX_TYPE	bREG;
} DDI_LVDSTXSEL7_TYPE;

typedef struct {
	unsigned SEL_32			:8;
	unsigned SEL_33			:8;
	unsigned SEL_34			:8;
	unsigned				:8;
} DDI_LVDSTXSEL8_IDX_TYPE;

typedef union {
	unsigned long			nREG;
	DDI_LVDSTXSEL8_IDX_TYPE	bREG;
} DDI_LVDSTXSEL8_TYPE;

typedef struct {
    unsigned    INTTYPE     :   1; // 0: egde, 1:level
    unsigned                :   15;
    unsigned    RESETN      :   1;
    unsigned                :   15;
} MIPICFG;
typedef union {
    unsigned long        nReg;
    MIPICFG     bReg;
} MIPICFG_U;

typedef struct 
{
	unsigned		AWCACHM0_SEL	:	4;
	unsigned		AWCHCHEM0		:	4;
	unsigned		ARCACHEM0_SEL	:	4;
	unsigned 		ARCACHEM0		:	4;
	unsigned		AWCACHM1_SEL	:	4;
	unsigned		AWCHCHEM1		:	4;
	unsigned		ARCACHEM1_SEL	:	4;
	unsigned 		ARCACHEM1		:	4;
} DDI_LVDS_CACHECTRL;

typedef union
{
	unsigned 	long			nREG;
	DDI_LVDS_CACHECTRL	bREG;
}	DDI_LVDS_CACHECTRL_u;

typedef struct
{
	unsigned		VF	:	1;
	unsigned		AS	:	1;
	unsigned		DSK_CNTS	:	12;
	unsigned		PPMS	:	6;
	unsigned		SK_CUR	:	2;
	unsigned		SDA		:	1;
	unsigned		C_TDY	:	1;
	unsigned		SKEH	:	1;
	unsigned		SKIN		:	1;
	unsigned		SK_BIAS	:	4;
	unsigned		ADS		:	1;
	unsigned		CPOL 	:	1;
}	DDI_LVDS_MISC0;

typedef union
{
	unsigned	long		nREG;
	DDI_LVDS_MISC0	bREG;
}	DDI_LVDS_MISC0_u;

typedef struct
{
	unsigned		FC 		:	3;
	unsigned		CPH		:	8;
	unsigned		CVH		:	8;
	unsigned		ST		:	1;
	unsigned		VHS		:	1;
	unsigned		LC		:	1;
	unsigned		CC		:	2;
	unsigned		CMS		:	1;
	unsigned		VOC		:	1;
	unsigned		FLT_CNT		:	1;
	unsigned		RESERVED0	:	3;
	unsigned		TCM		:	1;
	unsigned		AM		:	1;
}	DDI_LVDS_MISC1;

typedef union
{
	unsigned	long		nREG;
	DDI_LVDS_MISC1	bREG;
}	DDI_LVDS_MISC1_u;

typedef struct
{
	unsigned		SKCCK	:	3;
	unsigned		SKC0	:	3;
	unsigned		SKC1	:	3;
	unsigned		SKC2	:	3;
	unsigned		SKC3	:	3;
	unsigned		SKC4	:	3;
	unsigned		TXD		:	3;
	unsigned		ITDX	:	3;
	unsigned		RESERVED	:	8;
}	DDI_LVDS_MISC2;

typedef union
{
	unsigned	long		nREG;
	DDI_LVDS_MISC2	bREG;
}	DDI_LVDS_MISC2_u;

typedef	struct
{
	unsigned		BE 		:	1;
	unsigned		BR 		: 	1;
	unsigned		BCS		:	3;
	unsigned		BDI		:	2;
	unsigned		BCI 		: 	2;
	unsigned		BSC		:	6;
	unsigned		BFE		:	1;
	unsigned		BUP		:	7;
	unsigned		BPS		:	2;
	unsigned		DB		:	1;
	unsigned		SB		:	1;	
	unsigned		RESERVED0	:	1;
	unsigned		ON_3D	:	1;
	unsigned		DLYS_BST	:	1;
	unsigned		RESERVED1	:	2;
}	DDI_LVDS_MISC3;

typedef union
{
	unsigned	long		nREG;
	DDI_LVDS_MISC3	bREG;
}	DDI_LVDS_MISC3_u;


typedef struct _DDICONFIG{
	volatile DDI_CONFIG_TYPE		PWDN;			// 0x000  R/W  0x00000000	Power Down
	volatile DDI_CONFIG_TYPE		SWRESET;		// 0x004  R/W  0x00000000	Soft Reset
	unsigned :32;
	volatile DDI_CIFPORT_TYPE		CIFPORT;		// 0x00C  R/W  0x00043210	CIF select
	volatile DDI_HDMICTRL_TYPE		HDMI_CTRL;		// 0x010  R/W  0x00000000	HDMI Control
	volatile DDI_HDMIAES_TYPE		HDMI_AES;		// 0x014  R/W  0x00000000	HDMI AES
	volatile DDI_HDMIAESD0_TYPE		HDMI_AES_DATA0;	// 0x018  R/W  0x00000000	HDMI AES DATA #0
	volatile DDI_HDMIAESD1_TYPE		HDMI_AES_DATA1;	// 0x01C  R/W  0x00000000	HDMI AES DATA #1
	volatile DDI_HDMIAESHW_TYPE		HDMI_AES_HW0;	// 0x020  R/W  0x00000000	HDMI AES HW #0
	volatile DDI_HDMIAESHW_TYPE		HDMI_AES_HW1;	// 0x024  R/W  0x00000000	HDMI AES HW #1
	volatile DDI_HDMIAESHW2_TYPE	HDMI_AES_HW2;	// 0x028  R/W  0x00000000	HDMI AES HW #2
	unsigned :32;
	volatile DDI_NTSCPALEN_TYPE		NTSCPAL_EN;		// 0x030  R/W  0x00000001	NTSCPAL Encoder Enable
	unsigned :32;
	volatile MIPICFG_U           uMIPICFG       ; // 14
	unsigned :32;
	volatile DDI_LVDSCTRL_TYPE		LVDS_CTRL;		// 0x040  R/W  0x00850A01	LVDS Control register
	volatile DDI_LVDSTXSEL0_TYPE	LVDS_TXO_SEL0;	// 0x044  R/W  0x03020100	LVDS TXOUT select #0
	volatile DDI_LVDSTXSEL1_TYPE	LVDS_TXO_SEL1;	// 0x048  R/W  0x09080504	LVDS TXOUT select #1
	volatile DDI_LVDSTXSEL2_TYPE	LVDS_TXO_SEL2;	// 0x04C  R/W  0x0D0C0B0A	LVDS TXOUT select #2
	volatile DDI_LVDSTXSEL3_TYPE	LVDS_TXO_SEL3;	// 0x050  R/W  0x13121110	LVDS TXOUT select #3
	volatile DDI_LVDSTXSEL4_TYPE	LVDS_TXO_SEL4;	// 0x054  R/W  0x1A191514	LVDS TXOUT select #4
	volatile DDI_LVDSTXSEL5_TYPE	LVDS_TXO_SEL5;	// 0x058  R/W  0x0E070618	LVDS TXOUT select #5
	volatile DDI_LVDSTXSEL6_TYPE	LVDS_TXO_SEL6;	// 0x05C  R/W  0x1B17160F	LVDS TXOUT select #6
	volatile DDI_LVDSTXSEL7_TYPE	LVDS_TXO_SEL7;	// 0x060  R/W  0x1F1E1F1E	LVDS TXOUT select #7
	volatile DDI_LVDSTXSEL8_TYPE	LVDS_TXO_SEL8;	// 0x064  R/W  0x001E1F1E	LVDS TXOUT select #8
	volatile DDI_LVDS_CACHECTRL	LVDS_CACHE_CTRL	; // 0x068  R/W  0x22222222  DMA CACHE Control
	unsigned :32;									  // 0x06C RESERVED
	volatile DDI_LVDS_MISC0_u		LVDS_MISC0;		  // 0x070  R/W  0x00000000 LVDS Miscellaneous 0 Register
	volatile DDI_LVDS_MISC1_u		LVDS_MISC1;		  // 0x074  R/W  0x00000000 LVDS Miscellaneous 1 Register
	volatile DDI_LVDS_MISC2_u		LVDS_MISC2;		  // 0x074  R/W  0x00000000 LVDS Miscellaneous 2 Register
	volatile DDI_LVDS_MISC3_u		LVDS_MISC3;		  // 0x074  R/W  0x00000000 LVDS Miscellaneous 3Register
}DDICONFIG, *PDDICONFIG;

typedef enum {
	WMIX00 = 0,
	WMIX03,
	WMIX10,
	WMIX13,
	WMIX30,
	WMIX40,
	WMIX50,
	WMIX60,
	WMIX_MAX
} VIOC_CONFIG_WMIX_PATH;

typedef enum {
	VIOC_CONFIG_WMIXER = 0,
	VIOC_CONFIG_WDMA,
	VIOC_CONFIG_RDMA,
	VIOC_CONFIG_SCALER,
	VIOC_CONFIG_VIQE,
	VIOC_CONFIG_DEINTS,
	VIOC_CONFIG_VIN,
	VIOC_CONFIG_MC, 
	VIOC_CONFIG_FCENC,
	VIOC_CONFIG_FCDEC,
	VIOC_CONFIG_FIFO,
} VIOC_SWRESET_Component;

typedef enum {
	VIOC_CONFIG_CLEAR = 0,
	VIOC_CONFIG_RESET
} VIOC_SWRESET_Mode;

typedef struct{
	unsigned int enable;
	unsigned int connect_statue;
	unsigned int connect_device;
}VIOC_PlugInOutCheck;

// Power Down
#define HwDDIC_PWDN_G2D					Hw3		//G2D
#define HwDDIC_PWDN_HDMI                      Hw2           // HDMI
#define HwDDIC_PWDN_NTSC                      Hw1            // NTSL/PAL
#define HwDDIC_PWDN_LCDC                       Hw0         // LCDC

// Soft Reset
#define HwDDIC_SWRESET_G2D					Hw3		//G2D
#define HwDDIC_SWRESET_HDMI                  Hw2           // HDMI
#define HwDDIC_SWRESET_NTSC                  Hw1              // NTSL/PAL
#define HwDDIC_SWRESET_LCDC                   Hw0             // LCDC



/* Interface APIs */
extern VIOC_CONFIG_PATH_u *VIOC_CONFIG_GetPathStruct(unsigned int nType);
extern int VIOC_CONFIG_PlugIn(unsigned int nType, unsigned int nValue);
extern int VIOC_CONFIG_PlugOut(unsigned int nType);
extern void VIOC_CONFIG_RDMA12PathCtrl(unsigned int Path);
extern void VIOC_CONFIG_RDMA14PathCtrl(unsigned int Path);
extern void VIOC_CONFIG_WMIXPath(unsigned int Path, unsigned int Mode);
extern void VIOC_CONFIG_SWReset(VIOC_IREQ_CONFIG* pVIOCConfig, unsigned int vioc,unsigned int vioc_index,unsigned int Mode);
extern int VIOC_CONFIG_CheckPlugInOut(unsigned int nDevice);
extern int VIOC_CONFIG_Device_PlugState(unsigned int nDevice, VIOC_PlugInOutCheck *VIOC_PlugIn);
extern int tca_get_RDMA_PlugIn_to_Scaler(unsigned int RdmaN);

//map converter vioc config set
extern int tca_get_MC_Connect_to_RDMA(unsigned int *MC_NUM, unsigned int RdmaN);
extern int tca_set_MC_Connect_to_RDMA(unsigned int McN, unsigned int RdmaN, unsigned int SetClr);

extern VIOC_IREQ_CONFIG* VIOC_IREQConfig_GetAddress(void);

extern void vioc_config_stop_req(unsigned int en);
#endif



