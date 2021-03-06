/****************************************************************************
 * Copyright (C) 2016 Telechips Inc.
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
 * this program; if not	 write to the Free Software Foundation, Inc., 59 Temple Place, 
 * Suite 330, Boston, MA 02111-1307 USA
 ****************************************************************************/

#ifndef __DT_TCC898X_CLKS_H__
#define __DT_TCC898X_CLKS_H__

/* CLKCTRL channel index */
#define FBUS_CPU0		0	//   Cortex-A15 MP
#define FBUS_CPU1		1	//   Cortex-A7 SP
#define FBUS_CBUS		2	// CLKCTRL2
#define FBUS_CMBUS		3	// CLKCTRL3 (Cortex-M4)
#define FBUS_MEM		4	// CLKCTRL4
#define FBUS_VBUS		5	// CLKCTRL5
#define FBUS_HSIO		6	// CLKCTRL6
#define FBUS_SMU		7	// CLKCTRL7
#define FBUS_GPU		8	//   GRAPHIC 3D
#define FBUS_DDI		9	// CLKCTRL9
#define FBUS_G2D		10	//   GRAPHIC 2D
#define FBUS_IO			11	// CLKCTRL11
#define FBUS_CODA		12	// CLKCTRL12
#define FBUS_CHEVC		13	// CLKCTRL13
#define FBUS_VHEVC		14	// CLKCTRL14
#define FBUS_BHEVC		15	// CLKCTRL15
#define FBUS_MEM_PHY		16	//   Memory PHY.
#define FBUS_MAX		17

/* Peripheral Clocks */
#define PERI_TCX		0
#define PERI_TCT		1
#define PERI_TCZ		2
#define PERI_LCDSI0		3
#define PERI_LCD0		4
#define PERI_LCD1		5
#define PERI_HDMI_PHY27M	6
#define PERI_HDMI_SPDIFCLK	7
#define PERI_HDMI_PCLK		8
#define PERI_HDMI_CEC		9
#define PERI_HDMI_HDCP14	10
#define PERI_HDMI_HDCP22	11
#define PERI_GMAC		12
#define PERI_GMAC_PTP		13
#define PERI_PCIE_AUX		14
#define PERI_PCIE_REF_EXT	15
#define PERI_USB20H		16
#define PERI_USB_OTG_ADP	17
#define PERI_USB_OTG		18
#define PERI_RMT		19
#define PERI_SDMMC0		20
#define PERI_SDMMC1		21
#define PERI_SDMMC2		22
#define PERI_CEC1_CORE		23
#define PERI_MDAI0		24
#define PERI_MFLT0_DAI		25
#define PERI_MSPDIF0		26
#define PERI_MDAI1		27
#define PERI_MFLT1_DAI		28
#define PERI_MSPDIF1		29
#define PERI_PDM		30
#define PERI_CEC1_SFR		31
#define PERI_ADC		32
#define PERI_I2C0		33
#define PERI_I2C1		34
#define PERI_I2C2		35
#define PERI_I2C3		36
#define PERI_UART0		37
#define PERI_UART1		38
#define PERI_UART2		39
#define PERI_UART3		40
#define PERI_UART4		41
#define PERI_UART5		42
#define PERI_UART6		43
#define PERI_UART7		44
#define PERI_GPSB0		45
#define PERI_GPSB1		46
#define PERI_GPSB2		47
#define PERI_GPSBMS0		48
#define PERI_GPSBMS1		49
#define PERI_GPSBMS2		50
#define PERI_CEC0		51
#define PERI_I2C4		52
#define PERI_I2C5		53
#define PERI_TSTX0		54
#define PERI_MFLT0_CDIF		55
#define PERI_I2C6		56
#define PERI_I2C7		57
#define PERI_MFLT1_CDIF		58
#define PERI_IC_TC		59
#define PERI_TSRX0		60
#define PERI_TSRX1		61
#define PERI_TSRX2		62
#define PERI_TSRX3		63
#define PERI_CB_WDT		64
#define PERI_CB_STAMP		65
#define PERI_OUT0		66
#define PERI_OUT1		67
#define PERI_OUT2		68
#define PERI_OUT3		69
#define PERI_OUT4		70
#define PERI_OUT5		71
#define PERI_PCIE_APB		72
#define PERI_TSRX_FLT		73
#define PERI_MAX		74

/* I/O Bus pwdn/swreset */
#define IOBUS_IC_TC		0
#define IOBUS_CEC1		1
#define IOBUS_RESERVED02	2
#define IOBUS_RESERVED03	3
#define IOBUS_RESERVED04	4
#define IOBUS_RESERVED05	5
#define IOBUS_RESERVED06	6
#define IOBUS_DMA0		7
#define IOBUS_DMA1		8
#define IOBUS_DMA2		9
#define IOBUS_DMA		10
#define IOBUS_RESERVED11	11
#define IOBUS_PWM		12
#define IOBUS_RESERVED13	13
#define IOBUS_I2C_S2		14
#define IOBUS_RESERVED15	15
#define IOBUS_REMOCON		16
#define IOBUS_TSADC		17
#define IOBUS_RESERVED18	18
#define IOBUS_RESERVED19	19
#define IOBUS_PRT		20
#define IOBUS_ADMA0		21
#define IOBUS_DAI0		22
#define IOBUS_SPDIF0		23
#define IOBUS_AUDIO0		24
#define IOBUS_ADMA1		25
#define IOBUS_DAI1		26
#define IOBUS_SPDIF1		27
#define IOBUS_AUDIO1		28
#define IOBUS_I2C_M0		29
#define IOBUS_I2C_M1		30
#define IOBUS_I2C_M2		31
#define IOBUS_I2C_M3		32	//  0
#define IOBUS_I2C_S0		33	//  1
#define IOBUS_I2C_S1		34	//  2
#define IOBUS_I2C0		35	//  3
#define IOBUS_I2C1		36	//  4
#define IOBUS_MUSTBE0_37	37	//  5
#define IOBUS_MUSTBE0_38	38	//  6
#define IOBUS_MUSTBE0_39	39	//  7
#define IOBUS_MUSTBE0_40	40	//  8
#define IOBUS_AUDIO1_1		41	//  9
#define IOBUS_RESERVED42	42	// 10
#define IOBUS_RESERVED43	43	// 11
#define IOBUS_RESERVED44	44	// 12
#define IOBUS_RESERVED45	45	// 13
#define IOBUS_RESERVED46	46	// 14
#define IOBUS_RESERVED47	47	// 15
#define IOBUS_RESERVED48	48	// 16
#define IOBUS_GPSB_2		49	// 17
#define IOBUS_TS0		50	// 18
#define IOBUS_GPSB0		51	// 19
#define IOBUS_GPSB1		52	// 20
#define IOBUS_GPSB2		53	// 21
#define IOBUS_GPSB3		54	// 22
#define IOBUS_GPSB4		55	// 23
#define IOBUS_GPSB5		56	// 24
#define IOBUS_GPSB		57	// 25
#define IOBUS_CEC0		58	// 26
#define IOBUS_I2C_S3		59	// 27
#define IOBUS_I2C_M4		60	// 28
#define IOBUS_I2C_M5		61	// 29
#define IOBUS_I2C_M6		62	// 30
#define IOBUS_I2C_M7		63	// 31
#define IOBUS_UART0		64	//  0
#define IOBUS_UART1		65	//  1
#define IOBUS_UART2		66	//  2
#define IOBUS_UART3		67	//  3
#define IOBUS_UART4		68	//  4
#define IOBUS_UART5		69	//  5
#define IOBUS_UART6		70	//  6
#define IOBUS_UART7		71	//  7
#define IOBUS_SMARTCARD0	72	//  8
#define IOBUS_SMARTCARD1	73	//  9
#define IOBUS_SMARTCARD2	74	// 10
#define IOBUS_SMARTCARD3	75	// 11
#define IOBUS_SMARTCARD4	76	// 12
#define IOBUS_SMARTCARD5	77	// 13
#define IOBUS_SMARTCARD6	78	// 14
#define IOBUS_SMARTCARD7	79	// 15
#define IOBUS_UDMA0		80	// 16
#define IOBUS_UDMA1		81	// 17
#define IOBUS_UDMA2		82	// 18
#define IOBUS_UDMA3		83	// 19
#define IOBUS_UART_SMARTCARD0	84	// 20
#define IOBUS_UART_SMARTCARD1	85	// 21
#define IOBUS_RESERVED86	86	// 22
#define IOBUS_RESERVED87	87	// 23
#define IOBUS_RESERVED88	88	// 24
#define IOBUS_RESERVED89	89	// 25
#define IOBUS_RESERVED90	90	// 26
#define IOBUS_RESERVED91	91	// 27
#define IOBUS_RESERVED92	92	// 28
#define IOBUS_RESERVED93	93	// 29
#define IOBUS_RESERVED94	94	// 30
#define IOBUS_RESERVED95	95	// 31
#define IOBUS_GDMA1		96	//  0
#define IOBUS_CIPHER		97	//  1
#define IOBUS_SDMMC0		98	//  2
#define IOBUS_SDMMC1		99	//  3
#define IOBUS_SDMMC2		100	//  4
#define IOBUS_SDMMC		101	//  5
#define IOBUS_SMC		102	//  6
#define IOBUS_NFC		103	//  7
#define IOBUS_EDICFG		104	//  8
#define IOBUS_EDI		105	//  9
#define IOBUS_RTC		106	// 10
#define IOBUS_RESERVED107	107	// 11
#define IOBUS_RESERVED108	108	// 12
#define IOBUS_RESERVED109	109	// 13
#define IOBUS_RESERVED110	110	// 14
#define IOBUS_RESERVED111	111	// 15
#define IOBUS_RESERVED112	112	// 16
#define IOBUS_RESERVED113	113	// 17
#define IOBUS_RESERVED114	114	// 18
#define IOBUS_RESERVED115	115	// 19
#define IOBUS_RESERVED116	116	// 20
#define IOBUS_RESERVED117	117	// 21
#define IOBUS_RESERVED118	118	// 22
#define IOBUS_RESERVED119	119	// 23
#define IOBUS_RESERVED120	120	// 24
#define IOBUS_RESERVED121	121	// 25
#define IOBUS_RESERVED122	122	// 26
#define IOBUS_RESERVED123	123	// 27
#define IOBUS_RESERVED124	124	// 28
#define IOBUS_RESERVED125	125	// 29
#define IOBUS_RESERVED126	126	// 30
#define IOBUS_RESERVED127	127	// 31
#define IOBUS_MAX		128

/* Display Bus pwdn/swreset */
#define DDIBUS_VIOC		0
#define DDIBUS_NTSCPAL		1
#define DDIBUS_HDMI		2
#define DDIBUS_MAX		16

/* Graphic 3D/2D Bus pwdn/swreset */
#define GPUBUS_GRP		0
#define GPUBUS_MAX		1

/* Video Bus pwdn/swreset */
#define VIDEOBUS_JENC		0
#define VIDEOBUS_CODA_CORE	1
#define VIDEOBUS_HEVC_BUS	2
#define VIDEOBUS_HEVC_CORE	3
#define VIDEOBUS_VP9		4
#define VIDEOBUS_CODA_BUS	5
#define VIDEOBUS_MAX		6

/* High-Speed I/O Bus pwdn/swreset */
#define HSIOBUS_RESERVED00	0
#define HSIOBUS_RESERVED01	1
#define HSIOBUS_GMAC		2
#define HSIOBUS_RESERVED03	3
#define HSIOBUS_RESERVED04	4
#define HSIOBUS_RESERVED05	5
#define HSIOBUS_USB20H		6
#define HSIOBUS_RESERVED07	7
#define HSIOBUS_RESERVED08	8
#define HSIOBUS_RESERVED09	9
#define HSIOBUS_RESERVED10	10
#define HSIOBUS_RESERVED11	11
#define HSIOBUS_RESERVED12	12
#define HSIOBUS_RESERVED13	13
#define HSIOBUS_RESERVED14	14
#define HSIOBUS_RESERVED15	15
#define HSIOBUS_RESERVED16	16
#define HSIOBUS_RESERVED17	17
#define HSIOBUS_CIPHER		18
#define HSIOBUS_DWC_OTG		19
#define HSIOBUS_RESERVED20	20
#define HSIOBUS_SECURE_WRAP	21
#define HSIOBUS_TRNG		22
#define HSIOBUS_RESERVED23	23
#define HSIOBUS_RESERVED24	24
#define HSIOBUS_RESERVED25	25
#define HSIOBUS_RESERVED26	26
#define HSIOBUS_RESERVED27	27
#define HSIOBUS_RESERVED28	28
#define HSIOBUS_RESERVED29	29
#define HSIOBUS_RESERVED30	30
#define HSIOBUS_RESERVED31	31
#define HSIOBUS_MAX		32

/* TOP Isolation Control */
#define ISOIP_TOP_VLD		0
#define ISOIP_TOP_POR		1
#define ISOIP_TOP_ECID		2
#define ISOIP_TOP_OSC		3
#define ISOIP_TOP_TSADC		4
#define ISOIP_TOP_RTC		5
#define ISOIP_TOP_U30		6
#define ISOIP_TOP_U20H		7
#define ISOIP_TOP_OTG		8
#define ISOIP_TOP_PCIE		9
#define ISOIP_TOP_OTP		10
#define ISOIP_TOP_TSEN		11
#define ISOIP_TOP_DPLL		12
#define ISOIP_TOP_NPLL		13
#define ISOIP_TOP_MBPLL		14
#define ISOIP_TOP_MAX		15

/* Display Bus Isolation Control */
#define ISOIP_DDB_HDMI		0
#define ISOIP_DDB_LVDS		1
#define ISOIP_DDB_VDAC		2
#define ISOIP_DDB_MAX		3

#endif
