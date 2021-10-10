#ifndef __VIOC_MC_H__
#define	__VIOC_MC_H__

typedef struct	{
	uint				START		:  1;
	uint							:  3;
	uint				BITDEPTH_Y	:  2;
	uint				BITDEPTH_C	:  2;
	uint				START_LINE	:  4;
	uint							:  4;
	uint				UPD			:  1;
	uint				PAD			:  1;
	uint							:  2;
	uint				Y2RMD		:  3;
	uint				Y2REN		:  1;
	uint							:  7;
	uint				SWR			:  1;
}	VIOC_MC_CTRL;

typedef union {
	uint				nReg;
	VIOC_MC_CTRL		bReg;
}	VIOC_MC_CTRL_U;

typedef struct	{
	uint				base		: 32;
}	VIOC_MC_BASE;

typedef union	{
	uint				nReg;
	VIOC_MC_BASE		bReg;
}	VIOC_MC_BASE_U;

typedef struct	{
	uint				pic_height		: 16;
//	uint				frame_stride	: 16;
	uint								: 16;
}	VIOC_MC_PIC;

typedef union {
	uint				nReg;
	VIOC_MC_PIC			bReg;
}	VIOC_MC_PIC_U;

typedef struct 	{
	uint				frame_stride_y	: 16;
	uint				frame_stride_c  : 16;
}	VIOC_MC_STRIDE;

typedef union 	{
	uint				nReg;
	VIOC_MC_STRIDE		bReg;
}	VIOC_MC_STRIDE_U;


typedef struct	{
	uint				xpos			: 16;
	uint				ypos			: 16;
}	VIOC_MC_POS;
typedef	union	{
	uint				nReg;
	VIOC_MC_POS			bReg;
}	VIOC_MC_POS_U;

typedef	struct	{
	uint				xsize			: 16;
	uint				ysize			: 16;
}	VIOC_MC_SIZE;
typedef union	{
	uint				nReg;
	VIOC_MC_SIZE		bReg;
}	VIOC_MC_SIZE_U;

typedef struct	{
	uint				out_mode		:  3;
	uint								:  1;
	uint				ofs_endian		:  4;
	uint				comp_endian		:  4;
	uint								:  4;
	uint				hs_period		: 14;
	uint								:  2;
}	VIOC_MC_MISC0;

typedef union 	{
	uint				nReg;
	VIOC_MC_MISC0		bReg;
}	VIOC_MC_MISC0_U;

typedef struct	{
	uint				outmux0			:  3;
	uint								:  1;
	uint				outmux1			:  3;
	uint								:  1;
	uint				outmux2			:  3;
	uint								:  1;
	uint				outmux3			:  3;
	uint								:  1;
	uint				outmux4			:  3;
	uint								:  1;
	uint				outmux5			:  3;
	uint								:  1;
	uint				outmux6			:  3;
	uint								:  1;
	uint				outmux7			:  3;
	uint								:  1;
}	VIOC_MC_MISC1;

typedef union	{
	uint				nReg;
	VIOC_MC_MISC1		bReg;
}	VIOC_MC_MISC1_U;

typedef struct	{
	uint				timeout			: 16;
	uint				alpha			:  8;
	uint								:  8;
}	VIOC_MC_TIMEOUT;

typedef union	{
	uint				nReg;
	VIOC_MC_TIMEOUT		bReg;
}	VIOC_MC_TIMEOUT_U;

typedef struct	{
	uint				dith_en_y		:  4;
	uint				dith_en_c		:  4;
	uint				dith_sel_y		:  2;
	uint				dith_sel_c		:  2;
	uint				dith_mode		:  2;
	uint								:  2;
	uint								: 16;
}	VIOC_MC_DITH_CTRL;

typedef union	{
	uint				nReg;
	VIOC_MC_DITH_CTRL	bReg;
}	VIOC_MC_DITH_CTRL_U;

typedef struct	{
	unsigned	DITH00	: 3;
	unsigned			: 1;
	unsigned	DITH01	: 3;
	unsigned			: 1;
	unsigned	DITH02	: 3;
	unsigned			: 1;
	unsigned	DITH03	: 3;
	unsigned			: 1;
	unsigned	DITH10	: 3;
	unsigned			: 1;
	unsigned	DITH11	: 3;
	unsigned			: 1;
	unsigned	DITH12	: 3;
	unsigned			: 1;
	unsigned	DITH13	: 3;
	unsigned			: 1;

	unsigned	DITH20	: 3;
	unsigned			: 1;
	unsigned	DITH21	: 3;
	unsigned			: 1;
	unsigned	DITH22	: 3;
	unsigned			: 1;
	unsigned	DITH23	: 3;
	unsigned			: 1;
	unsigned	DITH30	: 3;
	unsigned			: 1;
	unsigned	DITH31	: 3;
	unsigned			: 1;
	unsigned	DITH32	: 3;
	unsigned			: 1;
	unsigned	DITH33	: 3;
	unsigned			: 1;
}	VIOC_MC_DITH_MAT;

typedef union	{
	uint				nReg[2];
	VIOC_MC_DITH_MAT	bReg;
}	VIOC_MC_DITH_MAT_U;

typedef union	{
	uint				nReg;
	VIOC_MC_DITH_MAT	bReg;
}	VIOC_MC_OFS_RADDR_Y_U;

typedef struct	{
	uint				base		: 32;
}	VIOC_MC_OFS_RADDR;

typedef union	{
	uint				nReg;
	VIOC_MC_OFS_RADDR		bReg;
}	VIOC_MC_OFS_RADDR_U;

typedef struct	{
	uint				base		: 32;
}	VIOC_MC_COM_RADDR;

typedef union	{
	uint				nReg;
	VIOC_MC_COM_RADDR		bReg;
}	VIOC_MC_COM_RADDR_U;

typedef struct	{
	uint				odma_y			:  2;
	uint								:  2;
	uint				odma_axi_y		:  3;
	uint								:  1;
	uint				odma_c			:  2;
	uint								:  2;
	uint				odma_axi_c		:  3;
	uint								:  1;	
	uint				cdma_y			:  4;
	uint				cdma_axi_y		:  2;
	uint								:  2;
	uint				cdma_C			:  4;
	uint				cdma_axi_c		:  2;
	uint								:  2;		
}	VIOC_MC_DMA_STAT;

typedef union	{
	uint				nReg;
	VIOC_MC_DMA_STAT		bReg;
}	VIOC_MC_DMA_STAT_U;


typedef struct	{
	uint				buf_cnt0		:  10;
	uint								:  6;
	uint				buf_cnt1		:  10;
	uint								:  6;
}	VIOC_MC_LBUF_CNT;

typedef union	{
	uint				nReg;
	VIOC_MC_LBUF_CNT		bReg;
}	VIOC_MC_LBUF_CNT_U;

typedef struct	{
	uint				pixel_cnt		:  14;
	uint								:  2;
	uint				line_cnt			:  14;
	uint								:  2;
}	VIOC_MC_POS_STAT;

typedef union	{
	uint							nReg;
	VIOC_MC_POS_STAT			bReg;
}	VIOC_MC_POS_STAT_U;

typedef struct	{
	uint				tg_stat		:  3;
	uint							:  1;
	uint				busy		:  1;
	uint				done		:  1;
	uint				err			:  1;
	uint				udr			:  1;	
	uint				bm_y		:  2;	
	uint				bm_c		:  2;	
	uint							:  20;		
}	VIOC_MC_STAT;

typedef union	{
	uint							nReg;
	VIOC_MC_STAT		bReg;
}	VIOC_MC_STAT_U;

typedef struct	{
	uint				md			:  1;
	uint				udp			:  1;
	uint				err			:  1;
	uint				udr			:  1;
	uint							:  12;
	uint				mmd		:  1;	
	uint				mupd		:  1;	
	uint				merr		:  1;	
	uint				mudr		:  1;	
	uint							:  12;		
}	VIOC_MC_IRQ;

typedef union	{
	uint							nReg;
	VIOC_MC_IRQ		bReg;
}	VIOC_MC_IRQ_U;

typedef struct	{
	VIOC_MC_CTRL_U			uCTRL;				// 0x0
	VIOC_MC_BASE_U			uOFS_BASE_Y;		// 0x4
	VIOC_MC_BASE_U			uOFS_BASE_C;		// 0x8
	VIOC_MC_PIC_U			uPIC;				// 0xC
	VIOC_MC_STRIDE_U		uFRM_STRIDE;		// 0x10
	VIOC_MC_BASE_U			uFRM_BASE_Y;		// 0x14
	VIOC_MC_BASE_U			uFRM_BASE_C;		// 0x18
	VIOC_MC_POS_U			uFRM_POS;			// 0x1C
	VIOC_MC_SIZE_U			uFRM_SIZE;			// 0x20
	VIOC_MC_MISC0_U			uFRM_MISC0;			// 0x24
	VIOC_MC_MISC1_U			uFRM_MISC1;			// 0x28
	VIOC_MC_TIMEOUT_U		uTIMEOUT;			// 0x2C
	VIOC_MC_DITH_CTRL_U		uDITH_CTRL;			// 0x30
	VIOC_MC_DITH_MAT_U		uDITH_MAT;			// 0x34, 0x38
	unsigned  int 			reserved0[5]; 		// 0x3C ~ 0x43
	VIOC_MC_OFS_RADDR_U		OFS_RADDR_Y;		// 0X50
	VIOC_MC_OFS_RADDR_U		OFS_RADDR_C;		// 0X54
	VIOC_MC_COM_RADDR_U		COM_RADDR_Y;		// 0X58
	VIOC_MC_COM_RADDR_U		COM_RADDR_C;		// 0X5C
	VIOC_MC_DMA_STAT_U		uDMA_STAT_C;		// 0X60
	VIOC_MC_LBUF_CNT_U		uLBUF_CNT0;			// 0X64
	VIOC_MC_LBUF_CNT_U		uLBUF_CNT1;			// 0X68
	VIOC_MC_LBUF_CNT_U		uCBUF_CNT;			// 0X6c
	VIOC_MC_POS_STAT_U		uPOS_STAT;			// 0x70
	VIOC_MC_STAT_U			uSTAT;				// 0x74
	VIOC_MC_IRQ_U			uIRQ;				// 0x78	
}	VIOC_MC;


extern void VIOC_MC_Start_OnOff(VIOC_MC *pMC, uint OnOff);
extern void VIOC_MC_UPD (VIOC_MC *pMC);
extern void VIOC_MC_Y2R_OnOff(VIOC_MC *pMC, uint OnOff);
extern void VIOC_MC_Start_BitDepth(VIOC_MC *pMC, uint Chroma, uint Luma);
extern void VIOC_MC_OFFSET_BASE (VIOC_MC *pMC, uint base_y, uint base_c);
extern void VIOC_MC_FRM_BASE    (VIOC_MC *pMC, uint base_y, uint base_c);
extern void VIOC_MC_FRM_SIZE    (VIOC_MC *pMC, uint xsize, uint ysize);
extern void VIOC_MC_FRM_SIZE_MISC   (VIOC_MC *pMC, uint pic_height, uint stride_y, uint stride_c);
extern void VIOC_MC_FRM_POS (VIOC_MC *pMC,  uint xpos, uint ypos);
extern void VIOC_MC_ENDIAN  (VIOC_MC *pMC, uint ofs_endian, uint comp_endian);
extern void VIOC_MC_DITH_CONT(VIOC_MC *pMC, uint en, uint sel);
extern void VIOC_MC_DUMP(unsigned int Num);

#endif
