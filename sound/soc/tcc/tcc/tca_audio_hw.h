/*
 * sound/soc/tcc/tcc/tca_audio_hw.h
 *
 */
#ifndef _TCC_AUDIO_HW_H_
#define _TCC_AUDIO_HW_H_

#include "tca_audio_globals.h"
 
#if defined(CONFIG_ARCH_TCC897X) || defined(CONFIG_ARCH_TCC570X)

#ifndef GIC_SPI_OFFSET
#define GIC_SPI_OFFSET			32
#define INT_ADMA0			(60+GIC_SPI_OFFSET)
#define INT_ADMA1			(97+GIC_SPI_OFFSET)
#define INT_ADMA2			(99+GIC_SPI_OFFSET)
#define INT_ADMA3			(58+GIC_SPI_OFFSET)
#endif

#define BASE_ADDR_ADMA0                      (0x76200000)
#define BASE_ADDR_DAI0                       (0x76201000)
#define BASE_ADDR_CDIF0	                     (0x76201080)
#define BASE_ADDR_SPDIFTX0                   (0x76202000)
#define BASE_ADDR_SPDIFRX0                   (0x76202800)

#define CLK_NAME_ADMA0       "adma0"
#define CLK_NAME_DAI0        "dai0"
#define CLK_NAME_SPDIF0      "spdif0"

#define BASE_ADDR_ADMA1                      (0x76400000)
#define BASE_ADDR_DAI1                       (0x76401000)

#define CLK_NAME_ADMA1       "adma1"
#define CLK_NAME_DAI1        "dai1"

#define BASE_ADDR_ADMA2                      (0x76B00000)
#define BASE_ADDR_DAI2                       (0x76B01000)

#define CLK_NAME_ADMA2      "adma2"
#define CLK_NAME_DAI2       "dai2"

#define BASE_ADDR_ADMA3                      (0x76100000)
#define BASE_ADDR_DAI3                       (0x76101000)
#define BASE_ADDR_CDIF3                      (0x76101080)
#define BASE_ADDR_SPDIFTX3                   (0x76102000)

#define CLK_NAME_ADMA3       "adma3"
#define CLK_NAME_DAI3        "dai3"
#define CLK_NAME_SPDIF3      "spdif3"

#define BASE_ADDR_GPIOE                     (0x74200100)
#define BASE_ADDR_GPIOG                     (0x74200180)

#elif defined(CONFIG_ARCH_TCC898X) || defined(CONFIG_ARCH_TCC802X)

#ifndef INT_ADMA0
#define GIC_OFFSET			32	// ref. "driver/soc/tcc/irq.c"
#define INT_ADMA0			(54+GIC_OFFSET)
#define INT_ADMA1			(56+GIC_OFFSET)
#endif

#define BASE_ADDR_ADMA0                      (0x16100000)
#define BASE_ADDR_DAI0                       (0x16101000)
#define BASE_ADDR_CDIF0	                     (0x16101080)
#define BASE_ADDR_SPDIFTX0                   (0x16102000)
#define BASE_ADDR_SPDIFRX0                   (0x16102800)

#define BASE_ADDR_ADMA1                      (0x16200000)
#define BASE_ADDR_DAI1                       (0x16201000)
#define BASE_ADDR_CDIF1	                     (0x16201080)
#define BASE_ADDR_SPDIFTX1                   (0x16202000)
#define BASE_ADDR_SPDIFRX1                   (0x16202800)

#if defined(CONFIG_ARCH_TCC898X)
#define BASE_ADDR_CHSEL		(0x1605105C)
#elif defined(CONFIG_ARCH_TCC802X)
#define INT_ADMA2			(76+GIC_OFFSET)
#define INT_ADMA3			(78+GIC_OFFSET)

#define BASE_ADDR_ADMA2						(0x16700000)
#define BASE_ADDR_DAI2						(0x16701000)
#define BASE_ADDR_CDIF2						(0x16701080)
#define BASE_ADDR_SPDIFTX2					(0x16702000)
#define BASE_ADDR_SPDIFRX2					(0x16702800)

#define BASE_ADDR_ADMA3						(0x16800000)
#define BASE_ADDR_DAI3						(0x16801000)
#define BASE_ADDR_CDIF3						(0x16801080)
#define BASE_ADDR_SPDIFTX3					(0x16802000)
#define BASE_ADDR_SPDIFRX3					(0x16802800)

#define BASE_ADDR_A0_PCFG					(0x16103000)
#define BASE_ADDR_A1_PCFG					(0x16203000)
#define BASE_ADDR_A2_PCFG					(0x16703000)
#define BASE_ADDR_A3_PCFG					(0x16803000)
#endif
#define BASE_ADDR_GPIOA                     (0x14200000)
#define BASE_ADDR_GPIOF                     (0x14200140)
#define BASE_ADDR_GPIOG                     (0x14200180)
#define BASE_ADDR_GPIOSD0                   (0x14200280)
#define BASE_ADDR_GPIOH                     (0x14200400)
#endif

/*******************************************************************************
*   8. Audio0(7.1ch)                                    (Base Addr = 0x76200000)
********************************************************************************/
typedef struct _ADMA{
    volatile unsigned int   RxDaDar;        // 0x000  R/W  0x00000000   DAI Rx (Right) Data Desti     nation Address
    volatile unsigned int   RxDaParam;      // 0x004  R/W  0x00000000   DAI Rx Parameters
    volatile unsigned int   RxDaTCnt;       // 0x008  R/W  0x00000000   DAI Rx Transmission Count     er Register
    volatile unsigned int   RxDaCdar;       // 0x00C  R    0x00000000   DAI Rx (Right) Data Curre     nt Destination Address
    volatile unsigned int   RxCdDar;        // 0x010  R/W  0x00000000   CDIF(SPDIF) Rx (Right) Da     ta Destination Address
    volatile unsigned int   RxCdParam;      // 0x014  R/W  0x00000000   CDIF(SPDIF) Rx Parameters
    volatile unsigned int   RxCdTCnt;       // 0x018  R/W  0x00000000   CDIF(SPDIF) Rx Transmissi     on Counter Register
    volatile unsigned int   RxCdCdar;       // 0x01C  R    0x00000000   CDIF(SPDIF) Rx (Right) Da     ta Current Destination Address
    unsigned :32; unsigned :32;
    volatile unsigned int   RxDaDarL;       // 0x028  R/W  0x00000000   DAI Rx Left Data Destinat     ion Address
    volatile unsigned int   RxDaCdarL;      // 0x02C  R    0x00000000   DAI Rx Left Data Current      Destination Address
    volatile unsigned int   RxCdDarL;       // 0x030  R/W  0x00000000   CDIF(SPDIF) Rx Left Data      Destination Address
    volatile unsigned int   RxCdCdarL;      // 0x034  R    0x00000000   CDIF(SPDIF) Rx Left Data      Current Destination Address
    volatile unsigned int   TransCtrl;      // 0x038  R/W  0x0000AA00   DMA Transfer Control Regi     ster
    volatile unsigned int   RptCtrl;        // 0x03C  R/W  0x00000000   DMA Repeat Control Regist     er
    volatile unsigned int   TxDaSar;        // 0x040  R/W  0x00000000   DAI Tx (Right) Data Sourc     e Address
    volatile unsigned int   TxDaParam;      // 0x044  R/W  0x00000000   DAI Tx Parameters
    volatile unsigned int   TxDaTCnt;       // 0x048  R/W  0x00000000   DAI Tx Transmission Count     er Register
    volatile unsigned int   TxDaCsar;       // 0x04C  R    0x00000000   DAI Tx (Right) Data Curre     nt Source Address
    volatile unsigned int   TxSpSar;        // 0x050  R/W  0x00000000   SPDIF Tx (Right) Data Sou     rce Address
    volatile unsigned int   TxSpParam;      // 0x054  R/W  0x00000000   SPDIF Tx Parameters
    volatile unsigned int   TxSpTCnt;       // 0x058  R/W  0x00000000   SPDIF Tx Transmission Cou     nter Register
    volatile unsigned int   TxSpCsar;       // 0x05C  R    0x00000000   SPDIF Tx (Right) Data Cur     rent Source Address
    unsigned :32; unsigned :32;
    volatile unsigned int   TxDaSarL;       // 0x068  R/W  0x00000000   DAI Tx Left Data Source A     ddress
    volatile unsigned int   TxDaCsarL;      // 0x06C  R    0x00000000   DAI Tx Left Data Current      Source Address
    volatile unsigned int   TxSpSarL;       // 0x070  R/W  0x00000000   SPDIF Tx Left Data Source      Address
    volatile unsigned int   TxSpCsarL;      // 0x074  R    0x00000000   SPDIF Tx Left Data Curren     t Source address
    volatile unsigned int   ChCtrl;         // 0x078  R/W  0x00008000   DMA Channel Control Regis     ter
    volatile unsigned int   IntStatus;      // 0x07C  R/W  0x00000000   DMA Interrupt Status Regi     ster
    volatile unsigned int   GIntReq;        // 0x080  R/W  0x00000000   General Interrupt Request
    volatile unsigned int   GIntStatus;     // 0x084  R    0x00000000   General Interrupt Status
    unsigned :32; unsigned :32;				// 0x090
    unsigned :32; unsigned :32; unsigned :32; unsigned :32;	//0x0A0
    unsigned :32; unsigned :32; unsigned :32; unsigned :32;	//0x0B0
    unsigned :32; unsigned :32; unsigned :32; unsigned :32;	//0x0C0
    unsigned :32; unsigned :32; unsigned :32; unsigned :32;	//0x0D0
    unsigned :32; unsigned :32; unsigned :32; unsigned :32;	//0x0E0
    unsigned :32; unsigned :32; unsigned :32; unsigned :32;	//0x0F0
    volatile unsigned int   RxDaDar1;       // 0x100  R/W  0x00000000   DAI1 Rx (Right) Data Dest     ination Address
    volatile unsigned int   RxDaDar2;       // 0x104  R/W  0x00000000   DAI2 Rx (Right) Data Dest     ination Address
    volatile unsigned int   RxDaDar3;       // 0x108  R/W  0x00000000   DAI3 Rx (Right) Data Dest     ination Address
    volatile unsigned int   RxDaCar1;       // 0x10C  R    0x00000000   DAI1 Rx (Right) Data Curr     ent Destination Address
    volatile unsigned int   RxDaCar2;       // 0x110  R    0x00000000   DAI2 Rx (Right) Data Curr     ent Destination Address
    volatile unsigned int   RxDaCar3;       // 0x114  R    0x00000000   DAI3 Rx (Right) Data Curr     ent Destination Address
    volatile unsigned int   RxDaDarL1;      // 0x118  R/W  0x00000000   DAI1 Rx Left Data Destina     tion Address
    volatile unsigned int   RxDaDarL2;      // 0x11C  R/W  0x00000000   DAI2 Rx Left Data Destina     tion Address
    volatile unsigned int   RxDaDarL3;      // 0x120  R/W  0x00000000   DAI3 Rx Left Data Destina     tion Address
    volatile unsigned int   RxDaCarL1;      // 0x124  R    0x00000000   DAI1 Rx Left Data Current      Destination Address
    volatile unsigned int   RxDaCarL2;      // 0x128  R    0x00000000   DAI2 Rx Left Data Current      Destination Address
    volatile unsigned int   RxDaCarL3;      // 0x12C  R    0x00000000   DAI3 Rx Left Data Current      Destination Address
    volatile unsigned int   TxDaSar1;       // 0x130  R/W  0x00000000   DAI1 Tx (Right) Data Sour     ce Address
    volatile unsigned int   TxDaSar2;       // 0x134  R/W  0x00000000   DAI2 Tx (Right) Data Sour     ce Address
    volatile unsigned int   TxDaSar3;       // 0x138  R/W  0x00000000   DAI3 Tx (Right) Data Sour     ce Address
    volatile unsigned int   TxDaCsar1;      // 0x13C  R    0x00000000   DAI1 Tx (Right) Data Curr     ent Source Address
    volatile unsigned int   TxDaCsar2;      // 0x140  R    0x00000000   DAI2 Tx (Right) Data Curr     ent Source Address
    volatile unsigned int   TxDaCsar3;      // 0x144  R    0x00000000   DAI3 Tx (Right) Data Curr     ent Source Address
    volatile unsigned int   TxDaDarL1;      // 0x148  R/W  0x00000000   DAI1 Tx Left Data Source      Address
    volatile unsigned int   TxDaDarL2;      // 0x14C  R/W  0x00000000   DAI2 Tx Left Data Source      Address
    volatile unsigned int   TxDaDarL3;      // 0x150  R/W  0x00000000   DAI3 Tx Left Data Source      Address
    volatile unsigned int   TxDaCarL1;      // 0x154  R    0x00000000   DAI1 Tx Left Data Current      Source Address
    volatile unsigned int   TxDaCarL2;      // 0x158  R    0x00000000   DAI2 Tx Left Data Current      Source Address
    volatile unsigned int   TxDaCarL3;      // 0x15C  R    0x00000000   DAI3 Tx Left Data Current      Source Address
#if defined(CONFIG_ARCH_TCC898X) || defined(CONFIG_ARCH_TCC802X)
    volatile unsigned int   TxDaSar4;      // 0x160  R    0x00000000	DAI4 Tx (Right) Data Source Address *only for 9.1Ch
    volatile unsigned int   TxDaCar4;      // 0x164  R    0x00000000  DAI4 Tx (Right) Data Current Source Address *only for 9.1Ch  
    volatile unsigned int   TxDaDarL4;      // 0x168  R    0x00000000  DAI4 Tx Left Data Source Address *only for 9.1Ch 
    volatile unsigned int   TxDaCarL4;      // 0x16C  R    0x00000000  DAI4 Tx Left Data Current Source Address *only for 9.1Ch
    volatile unsigned int   RxDaDar4;      // 0x170  R    0x00000000  DAI4 Rx (Right) Data Destination Address *only for 9.1Ch 
    volatile unsigned int   RxDaCar4;      // 0x174  R    0x00000000  DAI4 Rx (Right) Data Current Destination Address *only for 9.1Ch 
    volatile unsigned int   RxDaDarL4;      // 0x178  R    0x00000000  DAI4 Rx Left Data Destination Address *only for 9.1Ch 
    volatile unsigned int   RxDaCarL4;      // 0x17C  R    0x00000000  DAI4 Rx Left Data Current Destination Address *only for 9.1Ch 
    volatile unsigned int   ADMARST;      // 0x180  R    0x00000000  Audio DMA Reset 
#endif
} ADMA, *PADMA;

typedef struct _ADMADAI{
    volatile unsigned int   DADIR0;         // 0x000  R    -            Digital Audio Input Regis     ter 0
    volatile unsigned int   DADIR1;         // 0x004  R    -            Digital Audio Input Regis     ter 1
    volatile unsigned int   DADIR2;         // 0x008  R    -            Digital Audio Input Regis     ter 2
    volatile unsigned int   DADIR3;         // 0x00C  R    -            Digital Audio Input Regis     ter 3
    volatile unsigned int   DADIR4;         // 0x010  R    -            Digital Audio Input Regis     ter 4
    volatile unsigned int   DADIR5;         // 0x014  R    -            Digital Audio Input Regis     ter 5
    volatile unsigned int   DADIR6;         // 0x018  R    -            Digital Audio Input Regis     ter 6
    volatile unsigned int   DADIR7;         // 0x01C  R    -            Digital Audio Input Regis     ter 7
    volatile unsigned int   DADOR0;         // 0x020  R/W  -            Digital Audio Output Regi     ster 0
    volatile unsigned int   DADOR1;         // 0x024  R/W  -            Digital Audio Output Regi     ster 1
    volatile unsigned int   DADOR2;         // 0x028  R/W  -            Digital Audio Output Regi     ster 2
    volatile unsigned int   DADOR3;         // 0x02C  R/W  -            Digital Audio Output Regi     ster 3
    volatile unsigned int   DADOR4;         // 0x030  R/W  -            Digital Audio Output Regi     ster 4
    volatile unsigned int   DADOR5;         // 0x034  R/W  -            Digital Audio Output Regi     ster 5
    volatile unsigned int   DADOR6;         // 0x038  R/W  -            Digital Audio Output Regi     ster 6
    volatile unsigned int   DADOR7;         // 0x03C  R/W  -            Digital Audio Output Regi     ster 7
    volatile unsigned int   DAMR;           // 0x040  R/W  0x00000000   Digital Audio Mode Regist     er
    volatile unsigned int   DAVC;           // 0x044  R/W  0x0000       Digital Audio Volume Cont     rol Register
    volatile unsigned int   MCCR0;          // 0x048  R/W  0x00000000   Multi Channel Control Reg     ister 0
    volatile unsigned int   MCCR1;          // 0x04C  R/W  0x00000000   Multi Channel Control Reg     ister 1
    volatile unsigned int   DRMR;          // 0x050  R/W  0x00000000   Digital Radio Mode Register
} ADMADAI, *PADMADAI;
typedef struct _ADMACDIF{
    volatile unsigned int   CDDI_0;         // 0x080  R    -            CD Digital Audio Input Re     gister 0
    volatile unsigned int   CDDI_1;         // 0x084  R    -            CD Digital Audio Input Re     gister 1
    volatile unsigned int   CDDI_2;         // 0x088  R    -            CD Digital Audio Input Re     gister 2
    volatile unsigned int   CDDI_3;         // 0x08C  R    -            CD Digital Audio Input Re     gister 3
    volatile unsigned int   CDDI_4;         // 0x090  R    -            CD Digital Audio Input Re     gister 4
    volatile unsigned int   CDDI_5;         // 0x094  R    -            CD Digital Audio Input Re     gister 5
    volatile unsigned int   CDDI_6;         // 0x098  R    -            CD Digital Audio Input Re     gister 6
    volatile unsigned int   CDDI_7;         // 0x09C  R    -            CD Digital Audio Input Re     gister 7
    volatile unsigned int   CICR;           // 0x0A0  R/W  0x0000       CD Interface Control Regi     ster
} ADMACDIF, *PADMACDIF;

typedef struct _ADMASPDIFTX{
    volatile unsigned int   TxVersion;      // 0x000  R    0x00003111   Version Register
    volatile unsigned int   TxConfig;       // 0x004  R/W  0x00000000   Configuration Register
    volatile unsigned int   TxChStat;       // 0x008  R/W  0x00000000   Channel Status Control Re     gister
    volatile unsigned int   TxIntMask;      // 0x00C  R/W  0x00000000   Interrupt Mask Register
    volatile unsigned int   TxIntStat;      // 0x010  R/W  0x00000000   Interrupt Status Register
    volatile unsigned int   NOTDEF0[27];
    volatile unsigned int   UserData[24];   // 0x080~0x0DC  W  -        User Data Buffer
    volatile unsigned int   NOTDEF1[8];
    volatile unsigned int   ChStatus[24];   // 0x100~0x15C  W  -        Channel Status Buffer
    volatile unsigned int   NOTDEF2[40];
    volatile unsigned int   TxBuffer[16];   // 0x200~0x23C  W  -        Transmit Data Buffer
    volatile unsigned int   NOTDEF3[112];
    volatile unsigned int   DMACFG;         // 0x400  R/W  0x00000007   Additional Configuration      for DMA
    volatile unsigned int   NOTDEF4[159]; //kch 287->159
    volatile unsigned int   CSBUDB[24];     // 0x680~0x6DC  W  -        Merged Window for CSB/UDB
} ADMASPDIFTX, *PADMASPDIFTX;

typedef union _RXCAP{
    volatile unsigned int   RxCapCtln[16];      //  0x840~0x87C(even) W -   Channel Status Captur     e Control Register
    volatile unsigned int   RxCapn[16];         //  0x840~0x87C(odd)  W -   Captured Channel Stat     us / user bit
} RXCAP;

typedef struct _ADMASPDIFRX{
    volatile unsigned int   RxVersion;      // 0x800  R    0x00080111   Version Register
    volatile unsigned int   RxConfig;       // 0x804  R/W  0x00000000   Configuration Register
    volatile unsigned int   RxStatus;       // 0x808  R    0x00000000   Signal Status Buffer
    volatile unsigned int   RxIntMask;      // 0x80C  R/W  0x00000000   Interrupt Mask Register
    volatile unsigned int   RxIntStat;      // 0x810  R/W  0x00000000   Interrupt Status register
    volatile unsigned int   RxPhaseDet;     // 0x814  R/W  0x00000000
    unsigned :32; unsigned :32;
    unsigned :32; unsigned :32; unsigned :32; unsigned :32;
    unsigned :32; unsigned :32; unsigned :32; unsigned :32;
    volatile RXCAP              RxCap;          // 0x840~0x870  W  -        RxCapCtl[n] / RxCap[n     ]
    volatile unsigned int   NOTDEF0[96];
    volatile unsigned int   RxBuffer[8];    // 0xA00~0xA1C  W  -        Receive Data Buffer
} ADMASPDIFRX, *PADMASPDIFRX;
/*******************************************************************************
*   9. Audio1(Stereo)                                   (Base Addr = 0x76100000)
********************************************************************************/
//add by y00209195 2012-09-11
typedef struct _ADMA1{
    volatile unsigned int   RxDaDar;        // 0x000  R/W  0x00000000   DAI Rx (Right) Data Desti     nation Address
    volatile unsigned int   RxDaParam;      // 0x004  R/W  0x00000000   DAI Rx Parameters
    volatile unsigned int   RxDaTCnt;       // 0x008  R/W  0x00000000   DAI Rx Transmission Count     er Register
    volatile unsigned int   RxDaCdar;       // 0x00C  R    0x00000000   DAI Rx (Right) Data Curre     nt Destination Address
    volatile unsigned int   RxCdDar;        // 0x010  R/W  0x00000000   CDIF(SPDIF) Rx (Right) Da     ta Destination Address
    volatile unsigned int   RxCdParam;      // 0x014  R/W  0x00000000   CDIF(SPDIF) Rx Parameters
    volatile unsigned int   RxCdTCnt;       // 0x018  R/W  0x00000000   CDIF(SPDIF) Rx Transmissi     on Counter Register
    volatile unsigned int   RxCdCdar;       // 0x01C  R    0x00000000   CDIF(SPDIF) Rx (Right) Da     ta Current Destination Address
    unsigned :32;
    unsigned :32;
    volatile unsigned int   RxDaDarL;       // 0x028  R/W  0x00000000   DAI Rx Left Data Destinat     ion Address
    volatile unsigned int   RxDaCdarL;      // 0x02C  R    0x00000000   DAI Rx Left Data Current      Destination Address
    volatile unsigned int   RxCdDarL;       // 0x030  R/W  0x00000000   CDIF(SPDIF) Rx Left Data      Destination Address
    volatile unsigned int   RxCdCdarL;      // 0x034  R    0x00000000   CDIF(SPDIF) Rx Left Data      Current Destination Address
    volatile unsigned int   TransCtrl;      // 0x038  R/W  0x0000AA00   DMA Transfer Control Regi     ster
    volatile unsigned int   RptCtrl;        // 0x03C  R/W  0x00000000   DMA Repeat Control Regist     er
    volatile unsigned int   TxDaSar;        // 0x040
    volatile unsigned int   TxDaParam;      // 0x044
    volatile unsigned int   TxDaTCnt;       // 0x048
    volatile unsigned int   TxDaCsar;       // 0x04C
    volatile unsigned int   TxSpSar;        // 0x050
    volatile unsigned int   TxSpParam;      // 0x054
    volatile unsigned int   TxSpTCnt;       // 0x058
    volatile unsigned int   TxSpCsar;       // 0x05c
    unsigned :32;
    unsigned :32;
    volatile unsigned int   TxDaSarL;       // 0x068  R/W  0x00000000   DAI Tx Left Data Source A     ddress
    volatile unsigned int   TxDaCsarL;      // 0x06C  R    0x00000000   DAI Tx Left Data Current      Source Address
    volatile unsigned int   TxSpSarL;       // 0x070  R/W  0x00000000   SPDIF Tx Left Data Source      Address
    volatile unsigned int   TxSpCsarL;      // 0x074  R
    volatile unsigned int   ChCtrl;         // 0x078  R/W  0x00008000   DMA Channel Control Regis     ter
    volatile unsigned int   IntStatus;      // 0x07C  R/W  0x00000000   DMA Interrupt Status Regi     ster
} ADMA1, *PADMA1;

typedef struct _ADMADAI1{
    volatile unsigned int   DADIR0;         // 0x000  R    -            Digital Audio Input Regis     ter 0
    volatile unsigned int   DADIR1;         // 0x004  R    -            Digital Audio Input Regis     ter 1
    volatile unsigned int   DADIR2;         // 0x008  R    -            Digital Audio Input Regis     ter 2
    volatile unsigned int   DADIR3;         // 0x00C  R    -            Digital Audio Input Regis     ter 3
    volatile unsigned int   DADIR4;         // 0x010  R    -            Digital Audio Input Regis     ter 4
    volatile unsigned int   DADIR5;         // 0x014  R    -            Digital Audio Input Regis     ter 5
    volatile unsigned int   DADIR6;         // 0x018  R    -            Digital Audio Input Regis     ter 6
    volatile unsigned int   DADIR7;         // 0x01C  R    -            Digital Audio Input Regis     ter 7
    volatile unsigned int   DADOR0;         // 0x020  R/W  -            Digital Audio Output Regi     ster 0
    volatile unsigned int   DADOR1;         // 0x024  R/W  -            Digital Audio Output Regi     ster 1
    volatile unsigned int   DADOR2;         // 0x028  R/W  -            Digital Audio Output Regi     ster 2
    volatile unsigned int   DADOR3;         // 0x02C  R/W  -            Digital Audio Output Regi     ster 3
    volatile unsigned int   DADOR4;         // 0x030  R/W  -            Digital Audio Output Regi     ster 4
    volatile unsigned int   DADOR5;         // 0x034  R/W  -            Digital Audio Output Regi     ster 5
    volatile unsigned int   DADOR6;         // 0x038  R/W  -            Digital Audio Output Regi     ster 6
    volatile unsigned int   DADOR7;         // 0x03C  R/W  -            Digital Audio Output Regi     ster 7
    volatile unsigned int   DAMR;           // 0x040  R/W  0x00000000   Digital Audio Mode Regist     er
    volatile unsigned int   DAVC;           // 0x044  R/W  0x0000       Digital Audio Volume Cont     rol Register
} ADMADAI1, *PADMADAI1;

typedef struct gpio_regs {
	unsigned data;         /* data */
	unsigned out_en;       /* output enable */
	unsigned out_or;       /* OR fnction on output data */
	unsigned out_bic;      /* BIC function on output data */
	unsigned out_xor;      /* XOR function on output data */
	unsigned strength0;    /* driver strength control 0 */
	unsigned strength1;    /* driver strength control 1 */
	unsigned pull_enable;  /* pull-up/down enable */
	unsigned pull_select;  /* pull-up/down select */
	unsigned in_en;        /* input enable */
	unsigned in_type;      /* input type (Shmitt / CMOS) */
	unsigned slew_rate;    /* slew rate */
	unsigned func_select0; /* port configuration 0 */
	unsigned func_select1; /* port configuration 1 */
	unsigned func_select2; /* port configuration 2 */
	unsigned func_select3; /* port configuration 3 */
} TCC_GPIO, *PTCC_GPIO;


#if defined(CONFIG_ARCH_TCC898X)
typedef struct audio_ch_sel {
	volatile unsigned int DAI0CH_SEL;	//0x5C AUDIO0 DAI CHMUX Configuration Register 
	volatile unsigned int DAI1CH_SEL;	//0x60 AUDIO1 DAI CHMUX Configuration Register 
	volatile unsigned int CDIF0CH_SEL;	//0x64 AUDIO0 CDIF CHMUX Configuration Register 
	volatile unsigned int CDIF1CH_SEL;	//0x68 AUDIO1 CDIF CHMUX Configuration Register 
	volatile unsigned int SPDIF0CH_SEL;	//0x6C AUDIO0 SPDIF CHMUX Configuration Register 
	volatile unsigned int SPDIF1CH_SEL;	//0x70 AUDIO1 SPDIF CHMUX Configuration Register 
} AIFCHSEL, *PAIFCHSEL;		//Audio Interface Channel Select
#endif
#if defined(CONFIG_ARCH_TCC802X)
typedef struct audio_port_config {
	volatile unsigned int PCFG0;	//0x00 Port Configuration Register0 
	volatile unsigned int PCFG1;	//0x04 Port Configuration Register1 
	volatile unsigned int PCFG2;	//0x08 Port Configuration Register2 
	volatile unsigned int PCFG3;	//0x0C Port Configuration Register3 
} APCFG, *PAPCFG;		//Port Configuration Register Map
#endif
#endif /* _TCC_AUDIO_HW_H_ */
