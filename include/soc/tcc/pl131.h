/* include/soc/tcc/pl131.h
 *
 * Copyright (C) 2009, 2010 Telechips, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/types.h>
#include <linux/ioctl.h>
 
#ifndef _PL131_H_
#define _PL131_H_

#define PL131_FIFO_SIZE					8

/* -----------------------------------------------------------------------------
//                     SCI PL131 Register Map
// ---------------------------------------------------------------------------*/
#define SCIDATA 						       0x000
#define SCICR0						       0x004
#define SCICR1       						0x008 
#define SCICR2       						0x00C
#define SCICLKICC    						0x010
#define SCIVALUE     						0x014
#define SCIBAUD     						0x018
#define SCITIDE     						0x01C
#define SCIDMACR     						0x020
#define SCISTABLE    						0x024
#define SCIATIME    						0x028
#define SCIDTIME     						0x02C
#define SCIATRSTIME   					0x030
#define SCIATRDTIME   					0x034
#define SCISTOPTIME  					0x038
#define SCISTARTTIME 					0x03C
#define SCIRETRY      						0x040
#define SCICHTIMELS   					0x044
#define SCICHTIMEMS  					0x048
#define SCIBLKTIMELS  					0x04C
#define SCIBLKTIMEMS  					0x050
#define SCICHGUARD    					0x054
#define SCIBLKGUARD  					0x058
#define SCIRXTIME     						0x05C
#define SCIFIFOSTATUS  					0x060
#define SCITXCOUNT    					0x064
#define SCIRXCOUNT  						0x068
#define SCIIMSC       						0x06C
#define SCIRIS     						0x070
#define SCIMIS 						       0x074
#define SCIICR       						0x078
#define SCISYNCACT  						0x07C
#define SCISYNCTX     						0x080
#define SCISYNCRX    						0x084
#define SCIPeriphID0 						0xFE0
#define SCIPeriphID1 						0xFE4
#define SCIPeriphID2 						0xFE8
#define SCIPeriphID3  					0xFEC
#define SCIPCellID0   						0xFF0
#define SCIPCellID1   						0xFF4
#define SCIPCellID2   						0xFF8
#define SCIPCellID3 						0xFFC

// Error Definition
#define PL131_SC_SUCCESS						0
#define PL131_SC_ERROR_UNKNOWN				-1
#define PL131_SC_ERROR_UNSUPPORT			-2
#define PL131_SC_ERROR_INVALID_STATE		-3
#define PL131_SC_ERROR_INVALID_PARAM		-4
#define PL131_SC_ERROR_INVALID_ATR			-5
#define PL131_SC_ERROR_PARITY				-6
#define PL131_SC_ERROR_SIGNAL				-7
#define PL131_SC_ERROR_TIMEOUT				-8
#define PL131_SC_ERROR_IO					-9
#define PL131_SC_ERROR_COMPLEMENT			-10
#define PL131_SC_ERROR_NACK				-11

// State Definition
enum
{
	PL131_SC_STATE_NONE = 0,
	PL131_SC_STATE_OPEN,
	PL131_SC_STATE_ENABLE,
	PL131_SC_STATE_ACTIVATE,
	PL131_SC_STATE_MAX
};

// IOCTL Command Definition
enum
{
	PL131_SC_IOCTL_SET_VCC_LEVEL = 10,
	PL131_SC_IOCTL_ENABLE,
	PL131_SC_IOCTL_ACTIVATE,
	PL131_SC_IOCTL_RESET,
	PL131_SC_IOCTL_DETECT_CARD,
	PL131_SC_IOCTL_SET_CONFIG,
	PL131_SC_IOCTL_GET_CONFIG,
	PL131_SC_IOCTL_SET_IO_PIN_CONFIG,
	PL131_SC_IOCTL_GET_IO_PIN_CONFIG,
	PL131_SC_IOCTL_SEND,
	PL131_SC_IOCTL_RECV,
	PL131_SC_IOCTL_SEND_RCV,
	PL131_SC_IOCTL_NDS_SEND_RCV,
	PL131_SC_IOCTL_MAX
};

// Voltage Level
enum
{
	PL131_SC_VOLTAGE_LEVEL_3V = 0,
	PL131_SC_VOLTAGE_LEVEL_5V,
	PL131_sC_VOLTAGE_LEVEL_MAX
};

// Direction Definition
enum
{
	PL131_SC_DIRECTION_FROM_CARD = 0,
	PL131_SC_DIRECTION_TO_CARD,
	PL131_SC_DIRECTION_MAX
};

// Protocol Definition
enum
{
	PL131_SC_PROTOCOL_T0 = 0,
	PL131_SC_PROTOCOL_T1,
	PL131_SC_PROTOCOL_MAX
};

// Direction Definition
enum
{
	PL131_SC_DIRECT_CONVENTION = 0x3b,
	PL131_SC_INVERSE_CONVENTION = 0x3f,
};

// Parity Definition
enum
{
	PL131_SC_PARITY_DISABLE = 0,
	PL131_SC_PARITY_ODD,
	PL131_SC_PARITY_EVEN,
	PL131_SC_PARITY_MAX
};

// Error Signal Definition
enum
{
	PL131_SC_ERROR_SIGNAL_DISABLE = 0,
	PL131_SC_ERROR_SIGNAL_ENABLE,
	PL131_SC_ERROR_SIGNAL_MAX
};

// Flow Control Definition for NDS
enum
{
	PL131_SC_FLOW_CONTROL_DISABLE = 0,
	PL131_SC_FLOW_CONTROL_ENABLE,
	PL131_SC_FLOW_CONTROL_MAX
};

// I/O Pin Mask Definition for NDS
enum
{
	PL131_SC_PIN_MASK_IO_C7_ON		= 0x04,
	PL131_SC_PIN_MASK_IO_C4_ON		= 0x08,
	PL131_SC_PIN_MASK_IO_C8_ON		= 0x10,
	PL131_SC_PIN_MASK_C7_ON			= 0x80,
	PL131_SC_PIN_MASK_C4_ON			= 0x20,
	PL131_SC_PIN_MASK_C8_ON			= 0x40,
	PL131_SC_PIN_MASK_MAX
};

// SmartCard Clock Conversion Factor Definition
#define SCCLK_FI_372				372 //Internal Clock
#define SCCLK_FI_558				558
#define SCCLK_FI_744				744
#define SCCLK_FI_1116				1116
#define SCCLK_FI_1488				1488
#define SCCLK_FI_1860				1860
#define SCCLK_FI_512				512
#define SCCLK_FI_768				768
#define SCCLK_FI_1024				1024		
#define SCCLK_FI_1536				1536		
#define SCCLK_FI_2048				2048
#define SCCLK_FACTOR_RFU           		372 // for NDS

// SmartCard Default Clock Definition
#define SCCLK_FMAX_4MHz			4000*1000 
#define SCCLK_FMAX_5MHz			5000*1000 
#define SCCLK_FMAX_6MHz			6000*1000 
#define SCCLK_FMAX_8MHz			6750*1000 // 6.75 MHz 
#define SCCLK_FMAX_12MHz			12000*1000
#define SCCLK_FMAX_16MHz			13500*1000 // 13.5 MHz 
#define SCCLK_FMAX_20MHz			13500*1000 // 13.5 MHz 
#define SCCLK_FMAX_7MHz			7500*1000
#define SCCLK_FMAX_10MHz			10000*1000
#define SCCLK_FMAX_15MHz			15000*1000
#define SCCLK_FMAX_RFU				4000*1000 //Reserved

// SmartCard Time Out Definition
#define SC_TIME_OUT					2000 // 2.0 ms

#define SC_MAX_CH					1

// Configuration Parameter
typedef struct
{
	unsigned int		uiProtocol;
	unsigned int		uiConvention;
	unsigned int		uiParity;
	unsigned int		uiErrorSignal;
	unsigned int		uiFlowControl;

	unsigned int		uiClock;
	unsigned int		uiBaudrate;
	
	unsigned int		uiWaitTime;
	unsigned int		uiGuardTime;
} stPL131_SC_CONFIG;

// Send/Receive Buffer
typedef struct
{
	unsigned char		*pucTxBuf;
	unsigned int		uiTxBufLen;
	unsigned char		*pucRxBuf;
	unsigned int		*puiRxBufLen;
	unsigned int 		uiDirection;
	unsigned int		uiTimeOut;
} stPL131_SC_BUF;

// UART Port Data Structure
typedef struct
{
	int	Rst;
	int	Detect;
	int    PortNum;
	int    Uart_chNum;
	int	PwrEn;
	int	VccSel;
	int	Clk;
	int	DataC7;
	int	DataC4;
	int	DataC8;
	int	PinMask;

	int	Fn;
	int	FnC4;
	int	FnC8;

	int	TDA8024;
} port_info_t;

// UART Clock Structure
typedef struct
{
	unsigned int		Fi;
	unsigned int		Di;
	unsigned int		N;
	unsigned int		WI;
	
	unsigned int		Fuart;		// Peripherial Clock 
	unsigned int 		Fout;		// smartcard Clock
	unsigned int		Baud;		// Baud Rate Clock

	unsigned int		ETU;
	unsigned int		GT;
	unsigned int		WT;
	unsigned int		CWT;
	unsigned int		BWT;
} baud_info_t;

// Buffer for Smart Card
#define SC_RX_BUF_SIZE		512

typedef struct
{
	unsigned short	usRxHead;
	unsigned short	usRxTail;
	unsigned char		pRxBuf[SC_RX_BUF_SIZE];
} sc_buf_t;

// ATR(Answer To Reset) Information
#define TA_CHECK_BIT		0x10
#define TB_CHECK_BIT		0x20
#define TC_CHECK_BIT		0x40
#define TD_CHECK_BIT		0x80

typedef struct
{
	unsigned int	uiProtocol;
	unsigned int	uiConvention;
	
	unsigned short 	usTS;
	unsigned short 	usT0;
	unsigned short 	usTA[4];
	unsigned short	usTB[4];
	unsigned short	usTC[4];
	unsigned short 	usTD[4];
	unsigned char 	ucHC[15]; // Historical Characters - Max 15 Bytes
	unsigned char	ucTCK;
} sc_atr_t;



#endif 

