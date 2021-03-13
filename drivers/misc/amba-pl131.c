/*
 * linux/drivers/char/smartcard/amba-pl131.c
 *
 * Author:  <linux@telechips.com>
 * Created: March 8, 2016
 * Description: PrimeCell PL131 SmartCard driver
 *
 * Copyright (C) 2015-2016 Telechips 
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/wait.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/gpio.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/div64.h>
#include <asm/mach/map.h>
#include <soc/tcc/pl131.h>
#ifdef CONFIG_PM
#include <linux/pm.h>
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>


#define SC_DEVICE_NAME		"tcc-smartcard"
#define MAJOR_ID				235
#define MINOR_ID				0

#define sc_readl				__raw_readl
#define sc_writel				__raw_writel

/****************************************************************************
DEFINITION OF LOCAL VARIABLES
****************************************************************************/
static unsigned char gSCInvertTable[] = {
	0xFF, 0x7F, 0xBF, 0x3F, 0xDF, 0x5F, 0x9F, 0x1F, 0xEF, 0x6F, 0xAF, 0x2F, 0xCF, 0x4F, 0x8F, 0x0F,
	0xF7, 0x77, 0xB7, 0x37, 0xD7, 0x57, 0x97, 0x17, 0xE7, 0x67, 0xA7, 0x27, 0xC7, 0x47, 0x87, 0x07,
	0xFB, 0x7B, 0xBB, 0x3B, 0xDB, 0x5B, 0x9B, 0x1B, 0xEB, 0x6B, 0xAB, 0x2B, 0xCB, 0x4B, 0x8B, 0x0B,
	0xF3, 0x73, 0xB3, 0x33, 0xD3, 0x53, 0x93, 0x13, 0xE3, 0x63, 0xA3, 0x23, 0xC3, 0x43, 0x83, 0x03,
	0xFD, 0x7D, 0xBD, 0x3D, 0xDD, 0x5D, 0x9D, 0x1D, 0xED, 0x6D, 0xAD, 0x2D, 0xCD, 0x4D, 0x8D, 0x0D,
	0xF5, 0x75, 0xB5, 0x35, 0xD5, 0x55, 0x95, 0x15, 0xE5, 0x65, 0xA5, 0x25, 0xC5, 0x45, 0x85, 0x05,
	0xF9, 0x79, 0xB9, 0x39, 0xD9, 0x59, 0x99, 0x19, 0xE9, 0x69, 0xA9, 0x29, 0xC9, 0x49, 0x89, 0x09,
	0xF1, 0x71, 0xB1, 0x31, 0xD1, 0x51, 0x91, 0x11, 0xE1, 0x61, 0xA1, 0x21, 0xC1, 0x41, 0x81, 0x01,
	0xFE, 0x7E, 0xBE, 0x3E, 0xDE, 0x5E, 0x9E, 0x1E, 0xEE, 0x6E, 0xAE, 0x2E, 0xCE, 0x4E, 0x8E, 0x0E,
	0xF6, 0x76, 0xB6, 0x36, 0xD6, 0x56, 0x96, 0x16, 0xE6, 0x66, 0xA6, 0x26, 0xC6, 0x46, 0x86, 0x06,
	0xFA, 0x7A, 0xBA, 0x3A, 0xDA, 0x5A, 0x9A, 0x1A, 0xEA, 0x6A, 0xAA, 0x2A, 0xCA, 0x4A, 0x8A, 0x0A,
	0xF2, 0x72, 0xB2, 0x32, 0xD2, 0x52, 0x92, 0x12, 0xE2, 0x62, 0xA2, 0x22, 0xC2, 0x42, 0x82, 0x02,
	0xFC, 0x7C, 0xBC, 0x3C, 0xDC, 0x5C, 0x9C, 0x1C, 0xEC, 0x6C, 0xAC, 0x2C, 0xCC, 0x4C, 0x8C, 0x0C,
	0xF4, 0x74, 0xB4, 0x34, 0xD4, 0x54, 0x94, 0x14, 0xE4, 0x64, 0xA4, 0x24, 0xC4, 0x44, 0x84, 0x04,
	0xF8, 0x78, 0xB8, 0x38, 0xD8, 0x58, 0x98, 0x18, 0xE8, 0x68, 0xA8, 0x28, 0xC8, 0x48, 0x88, 0x08,
	0xF0, 0x70, 0xB0, 0x30, 0xD0, 0x50, 0x90, 0x10, 0xE0, 0x60, 0xA0, 0x20, 0xC0, 0x40, 0x80, 0x00
};

static unsigned gSCClkFactor[16] =
{
 	SCCLK_FI_372,
	SCCLK_FI_372,
	SCCLK_FI_558, 
	SCCLK_FI_744,
	SCCLK_FI_1116,
	SCCLK_FI_1488,
	SCCLK_FI_1860,
	SCCLK_FACTOR_RFU,
	SCCLK_FACTOR_RFU,
	SCCLK_FI_512,
	SCCLK_FI_768,
	SCCLK_FI_1024,
	SCCLK_FI_1536,
	SCCLK_FI_2048, 
	SCCLK_FACTOR_RFU,
	SCCLK_FACTOR_RFU
};

static unsigned gSCClk[16] =
{
	SCCLK_FMAX_4MHz,
	SCCLK_FMAX_5MHz,
	SCCLK_FMAX_6MHz,
	SCCLK_FMAX_8MHz,
	SCCLK_FMAX_12MHz,
	SCCLK_FMAX_16MHz,
	SCCLK_FMAX_20MHz,
	SCCLK_FMAX_RFU,
	SCCLK_FMAX_RFU,
	SCCLK_FMAX_5MHz,
	SCCLK_FMAX_7MHz,
	SCCLK_FMAX_10MHz,
	SCCLK_FMAX_15MHz,
	SCCLK_FMAX_20MHz,
	SCCLK_FMAX_RFU,
	SCCLK_FMAX_RFU
};

struct pl131_sc_data{
	struct miscdevice			*misc;
	struct device				*dev;
	struct clk				*pclk;
	struct clk				*hclk;
	struct clk				*shclk;
	baud_info_t				*baud_info;
	port_info_t				*port_info;
	sc_buf_t					*sc_buf;
	sc_atr_t					*sc_atr;
	struct mutex				lock;

	unsigned int				sc_clk_rate;
	unsigned int				card_clk_rate;
	unsigned int				irq;
	int						giSCState;
	int						rst_retry;
	void __iomem				*regs;
	void __iomem				*port_cfg_reg;
};


//---------------------------------------------------------------------------
// DEFINITION OF LOCAL FUNCTIONS
//---------------------------------------------------------------------------

static void sc_init_factor(struct file *filp)
{
	struct miscdevice *misc = filp->private_data;
	struct pl131_sc_data *pl131 = dev_get_drvdata(misc->parent);

	pl131->baud_info->Fi = SCCLK_FI_372;
	pl131->baud_info->Di = 1;
	pl131->baud_info->N = 0;
	pl131->baud_info->WI = 10;
	pl131->baud_info->Fout = SCCLK_FMAX_4MHz;
	pl131->baud_info->Baud = pl131->baud_info->Fout/(pl131->baud_info->Fi/pl131->baud_info->Di);
	pl131->baud_info->GT = 0;
	pl131->baud_info->WT = 0;
	pl131->baud_info->CWT = 0;
	pl131->baud_info->BWT = 0;
}

static void sc_config(struct file *filp)
{
	struct miscdevice *misc = filp->private_data;
	struct pl131_sc_data *pl131 = dev_get_drvdata(misc->parent);
	int sciclk_value;
	unsigned int baud_div, scibaud, scivalue=1;	

	sciclk_value = (pl131->baud_info->Fuart/(2*pl131->baud_info->Fout) -1); // initial value by device tree
	sc_writel(sciclk_value, pl131->regs + SCICLKICC); // set smartcard clock , initial value by device tree

	pl131->baud_info->Baud = (pl131->baud_info->Fout * pl131->baud_info->Di)/pl131->baud_info->Fi;
	baud_div = pl131->baud_info->Fuart/pl131->baud_info->Baud;

	scivalue = 10; // minimum value 5
	baud_div = baud_div/scivalue;
	
	while(baud_div>0xFFFF){
		scivalue++;
		baud_div = baud_div/scivalue;
	}
	
	scibaud = baud_div;
	sc_writel(scivalue, pl131->regs + SCIVALUE); // set baudrate with SCIVALUE and SCIBAUD
	sc_writel(scibaud, pl131->regs + SCIBAUD);
}

unsigned inline char sc_ts_pattern_inverse(char cData)
{
	return gSCInvertTable[(int)cData];
}

static void sc_calc_factor_T0(struct file *filp, unsigned short usTA1, unsigned short usTC1, unsigned short usTC2)
{
	struct miscdevice *misc = filp->private_data;
	struct pl131_sc_data *pl131 = dev_get_drvdata(misc->parent);
	unsigned uHighNibble, uLowNibble;
	
	//Calculate Fout, Fi, Di

	if(usTA1 != 0xFFFF) {
		uHighNibble = (usTA1 & 0xF0) >> 4;
		uLowNibble = (usTA1 & 0x0F);

		pl131->baud_info->Fout = gSCClk[uHighNibble];
		pl131->baud_info->Fi = gSCClkFactor[uHighNibble];

		if(uLowNibble > 0 && uLowNibble < 8) {
			pl131->baud_info->Di = 1<<(uLowNibble-1);
		}
		else if(uLowNibble == 8) {
			pl131->baud_info->Di = 12;
		}
		else if(uLowNibble == 9) {
			pl131->baud_info->Di = 20;
		}
		else if(uLowNibble == 15) {
			pl131->baud_info->Di = 93;
		}
		else {
			pl131->baud_info->Di = 4;
		}
	}
	
	//Calculate N
	if(usTC1 != 0xFFFF) {
		pl131->baud_info->N = usTC1;
	}
	else {
		pl131->baud_info->N = 0;
	}

	//Calculate WI
	if(usTC2 != 0xFFFF) {
		pl131->baud_info->WI = usTC2;
	}
	else {
		pl131->baud_info->WI = 10;
	}

	//Calculate Baud	
	pl131->baud_info->Baud = (pl131->baud_info->Fout * pl131->baud_info->Di) / pl131->baud_info->Fi;

	//Calculate GT, WT
	pl131->baud_info->ETU = (pl131->baud_info->Fi * 1000000) / (pl131->baud_info->Di * (pl131->baud_info->Fout / 1000));	//nsec
	pl131->baud_info->GT = ((12 * pl131->baud_info->ETU) + (pl131->baud_info->N * pl131->baud_info->ETU)) / 1000;			//usec
	pl131->baud_info->WT = (pl131->baud_info->WI * 960 * pl131->baud_info->Fi) / (pl131->baud_info->Fout / 1000);				//msec

#if 0
	printk("%s(Fi:%d, Di:%d, N:%d)\n", __func__, pl131->baud_info->Fi, pl131->baud_info->Di, pl131->baud_info->N);
	printk("%s(Fuart:%d, Fout:%d, Baud:%d)\n", __func__, pl131->baud_info->Fuart, pl131->baud_info->Fout, pl131->baud_info->Baud);
	printk("%s(ETU:%d, GT:%d, WT:%d)\n", __func__, pl131->baud_info->ETU, pl131->baud_info->GT, pl131->baud_info->WT);
#endif

	return;
}

static void sc_calc_factor_T1(struct file *filp, unsigned short usTA1, unsigned short usTC1, unsigned short usTB3)
{
	struct miscdevice *misc = filp->private_data;
	struct pl131_sc_data *pl131 = dev_get_drvdata(misc->parent);
	unsigned uHighNibble, uLowNibble;
	int iCWI=1, iBWI=1;
	int i;

	//Calculate Fout, Fi, Di
	if(usTA1 != 0xFFFF) {
		uHighNibble = (usTA1 & 0xF0) >> 4;
		uLowNibble = (usTA1 & 0x0F);

		pl131->baud_info->Fout = gSCClk[uHighNibble];
		pl131->baud_info->Fi = gSCClkFactor[uHighNibble];

		if(uLowNibble > 0 && uLowNibble < 8) {
			pl131->baud_info->Di = 1<<(uLowNibble-1);
		}
		else if(uLowNibble == 8) {
			pl131->baud_info->Di = 12;
		}
		else if(uLowNibble == 9) {
			pl131->baud_info->Di = 20;
		}
		else {
			pl131->baud_info->Di = 4;
		}
	}

	//Calculate CWT, BWT
	if(usTB3 != 0xFFFF) {
		uHighNibble =(usTB3 & 0xF0) >> 4;	//BWI
		uLowNibble = (usTB3 & 0x0F);		//CWI
	}
	else {
		uHighNibble = 4;
		uLowNibble = 13;
	}

	pl131->baud_info->Baud = pl131->baud_info->Fout / (pl131->baud_info->Fi / pl131->baud_info->Di);
	pl131->baud_info->ETU = (pl131->baud_info->Fi * 1000000) / (pl131->baud_info->Di * (pl131->baud_info->Fout / 1000));	// nsec

	for(i=0; i<uLowNibble; i++)
	{
		iCWI *= 2;
	}
	
	for(i=0; i<uHighNibble; i++)
	{
		iBWI *= 2;
	}	
	
	pl131->baud_info->CWT = ((iCWI * pl131->baud_info->ETU) + (11 * pl131->baud_info->ETU)) / 1000;		// usec
	pl131->baud_info->BWT = (((iBWI * 960 * 372 * 10000) / (pl131->baud_info->Fout / 100000)) + (11 * pl131->baud_info->ETU)) / 1000;	// usec

#if 0
	printk("%s(Fi:%d, Di:%d, N:%d, iCWI:%d, iBWI:%d)\n", __func__, pl131->baud_info->Fi, pl131->baud_info->Di, pl131->baud_info->N, iCWI, iBWI);
	printk("%s(Fuart:%d, Fout:%d, Baud:%d)\n", __func__, pl131->baud_info->Fuart, pl131->baud_info->Fout, pl131->baud_info->Baud);
	printk("%s(ETU:%d, GT:%d, WT:%d)\n", __func__, pl131->baud_info->ETU, pl131->baud_info->GT, pl131->baud_info->WT);
	printk("%s(CWT:%d, BWT:%d)\n", __func__, pl131->baud_info->CWT, pl131->baud_info->BWT);
#endif

	return;
}

static int sc_parse_atr(struct file *filp, unsigned char *pATR, unsigned uATRLen)
{
	struct miscdevice *misc = filp->private_data;
	struct pl131_sc_data *pl131 = dev_get_drvdata(misc->parent);
	sc_atr_t *pstATR = pl131->sc_atr;
	unsigned uHCharNum = 0;
	unsigned uBufLen = 0;
	int i;
	
	dev_dbg(pl131->dev, "%s\n", __func__);

	//Check ATR Direction 
	pstATR->usTS = *(pATR + uBufLen++);
	if(pstATR->usTS == PL131_SC_DIRECT_CONVENTION) {
		pstATR->uiConvention = PL131_SC_DIRECT_CONVENTION;
	}
	else if(pstATR->usTS == 0x03) {
		pstATR->uiConvention = PL131_SC_INVERSE_CONVENTION;

		for(i=0; i<uATRLen; i++)
		{
			pATR[i] =  sc_ts_pattern_inverse(pATR[i]); 
		}
		pstATR->usTS = *pATR;
	}
	else  {
		dev_err(pl131->dev, "%s(0x%04X)\n", __func__, pstATR->usTS);
		return PL131_SC_ERROR_INVALID_ATR;
	}
	
	pstATR->usT0 = *(pATR + uBufLen++);

	//Get TA1, TB1, TC1, TD1 Characters
	if(pstATR->usT0 & TA_CHECK_BIT) {
		pstATR->usTA[0] = *(pATR + uBufLen++);
	}
	if(pstATR->usT0 & TB_CHECK_BIT) {
		pstATR->usTB[0] = *(pATR + uBufLen++);
	}
	if(pstATR->usT0 & TC_CHECK_BIT) {
		pstATR->usTC[0] = *(pATR + uBufLen++);
	}
	if(pstATR->usT0 & TD_CHECK_BIT) {
		pstATR->usTD[0] = *(pATR + uBufLen++);
	}

	//Get TA2~4, TB2~4, TC2~4, TD2~4 Characters
	for(i=0; i<3; i++)
	{
		if(pstATR->usTD[i] != 0xFFFF) {
			if(pstATR->usTD[i] & TA_CHECK_BIT) {
				pstATR->usTA[i+1] = *(pATR + uBufLen++);
			}
			if(pstATR->usTD[i] & TB_CHECK_BIT) {
				pstATR->usTB[i+1] = *(pATR + uBufLen++);
			}
			if(pstATR->usTD[i] & TC_CHECK_BIT) {
				pstATR->usTC[i+1] = *(pATR + uBufLen++);
			}
			if(pstATR->usTD[i] & TD_CHECK_BIT) {
				pstATR->usTD[i+1] = *(pATR + uBufLen++);
			}

			if(i==0 || i== 1) {
				pstATR->uiProtocol = pstATR->usTD[i] & 0x0F;
			}
		}
	}
	
	//Get Historical Characters
	uHCharNum = pstATR->usT0 & 0x0F;
	for(i=0; i<uHCharNum; i++)
	{
		pstATR->ucHC[i] = *(pATR + uBufLen++);
	}

	//Get TCK Characters
	if(pstATR->uiProtocol != 0) {
		pstATR->ucTCK = *(pATR + uBufLen++);
	}

	//Check ATR Length
	if(uATRLen != uBufLen) {
		dev_err(pl131->dev, "%s(%d, %d)\n", __func__, uATRLen, uBufLen);
		memset(pstATR, 0xFF, sizeof(sc_atr_t));
		
		return PL131_SC_ERROR_INVALID_ATR;
	}

#if 0
	printk("TS:0x%04x\n", pstATR->usTS);
	printk("TO:0x%04x\n", pstATR->usT0);
	printk("TA1:0x%04x, TB1:0x%04x, TC1:0x%04x, TD1:0x%04x\n", pstATR->usTA[0], pstATR->usTB[0], pstATR->usTC[0], pstATR->usTD[0]);
	printk("TA2:0x%04x, TB2:0x%04x, TC2:0x%04x, TD2:0x%04x\n", pstATR->usTA[1], pstATR->usTB[1], pstATR->usTC[1], pstATR->usTD[1]);
	printk("TA3:0x%04x, TB3:0x%04x, TC3:0x%04x, TD3:0x%04x\n", pstATR->usTA[2], pstATR->usTB[2], pstATR->usTC[2], pstATR->usTD[2]);
	printk("TA4:0x%04x, TB4:0x%04x, TC4:0x%04x, TD4:0x%04x\n", pstATR->usTA[3], pstATR->usTB[3], pstATR->usTC[3], pstATR->usTD[3]);
	printk("Historical Charaters : %s\n", pstATR->ucHC);
	printk("TCK:0x%02x\n", pstATR->ucTCK);
#endif		

	if(pstATR->uiProtocol == 0) {
		sc_calc_factor_T0(filp, pstATR->usTA[0], pstATR->usTC[0], pstATR->usTC[1]);
	}
	else if(pstATR->uiProtocol == 1) {
		sc_calc_factor_T1(filp, pstATR->usTA[0], pstATR->usTC[0], pstATR->usTB[2]);
	}
	else {
		dev_err(pl131->dev, "%s(%d)\n", __func__, pstATR->uiProtocol);
		return PL131_SC_ERROR_INVALID_ATR;
	}

	return 0;
}

static int pl131_sc_enable(struct file *filp, unsigned int uEnable)
{
	struct miscdevice *misc = filp->private_data;
	struct pl131_sc_data *pl131 = dev_get_drvdata(misc->parent);
	
	dev_dbg(pl131->dev, "%s(%d)\n", __func__, uEnable);
	
	if(uEnable) {
		if(pl131->giSCState != PL131_SC_STATE_OPEN) {
			dev_err(pl131->dev, "%s : invalid state(%d)\n", __func__, pl131->giSCState);
			return PL131_SC_ERROR_INVALID_STATE;
		}

		sc_init_factor(filp);
		
		pl131->giSCState = PL131_SC_STATE_ENABLE;
	}
	else {
		pl131->giSCState = PL131_SC_STATE_OPEN;
	}
	
	return PL131_SC_SUCCESS;
}


static int pl131_sc_activate(struct file *filp, unsigned uActivate)
{
	struct miscdevice *misc = filp->private_data;
	struct pl131_sc_data *pl131 = dev_get_drvdata(misc->parent);
	
	dev_dbg(pl131->dev, "%s(%d)\n", __func__, uActivate);
	
	if(uActivate) {
		if(pl131->giSCState != PL131_SC_STATE_ENABLE) {
			dev_err(pl131->dev, "%s : invalid state(%d)\n", __func__, pl131->giSCState);
			return PL131_SC_ERROR_INVALID_STATE;
		}

		sc_writel(sc_readl(pl131->regs + SCICR2) | 0x1, pl131->regs + SCICR2);	// pl131 enable
		gpio_direction_output(pl131->port_info->Rst, 1);		
		gpio_set_value_cansleep(pl131->port_info->Rst, 1); // Reset high
		pl131->giSCState = PL131_SC_STATE_ACTIVATE;
	}
	else {
		if(pl131->giSCState != PL131_SC_STATE_ACTIVATE) {
			dev_err(pl131->dev, "%s : invalid state(%d)\n", __func__, pl131->giSCState);
			return PL131_SC_ERROR_INVALID_STATE;
		}

		gpio_set_value_cansleep(pl131->port_info->Rst, 0); // Reset low
		sc_writel(sc_readl(pl131->regs + SCICR2) & ~(0x1), pl131->regs + SCICR2); // pl131 disable
		gpio_set_value_cansleep(pl131->port_info->VccSel, 0); // VCC Selection low
		pl131->giSCState = PL131_SC_STATE_ENABLE;
	}

	return PL131_SC_SUCCESS;
}

static int pl131_sc_reset(struct file *filp, unsigned char *pATR, unsigned *puATRLen)
{
	struct miscdevice *misc = filp->private_data;
	struct pl131_sc_data *pl131 = dev_get_drvdata(misc->parent);
	sc_buf_t *pstBuf = pl131->sc_buf;
	sc_atr_t *pstATR = pl131->sc_atr;
	unsigned long orig_jiffies = jiffies;
	unsigned char *pBuf = pATR;
	unsigned char rx_data = 0;
	unsigned uBufLen = 0;
	int i,ret;

	dev_dbg(pl131->dev, "%s\n", __func__);
	
	if(pl131->giSCState != PL131_SC_STATE_OPEN) {
		sc_init_factor(filp);
		sc_config(filp); 
	}

	if(pl131->giSCState == PL131_SC_STATE_OPEN) {
		pl131_sc_enable(filp, 1);	//enable
		pl131->giSCState = PL131_SC_STATE_ENABLE;
	}
		
	if(pl131->giSCState == PL131_SC_STATE_ENABLE) {
		pl131_sc_activate(filp, 1); //activate
		pl131->giSCState = PL131_SC_STATE_ACTIVATE;
	}

	// Init Rx Buffer
	memset(pstBuf, 0x00, sizeof(sc_buf_t));
	
	//Set Default Value
	memset(pstATR, 0xFF, sizeof(sc_atr_t));
	pstATR->uiConvention = PL131_SC_DIRECT_CONVENTION;	

	sc_writel((sc_readl(pl131->regs + SCICR1) & ~(1<<2)), pl131->regs + SCICR1); // rx mode

	pl131->rst_retry = 1; // not yet retry function
	for(i=0; i<pl131->rst_retry; i++)
	{
		uBufLen = 0;
		pBuf = pATR;
		gpio_set_value_cansleep(pl131->port_info->Rst, 0); // RST is low
		msleep(40); // temporary delay		
		gpio_set_value_cansleep(pl131->port_info->Rst, 1); // RST is high

		sc_writel(0x1FFF, pl131->regs + SCIICR); // Clear all SCI interrupt
		
		while(1)
		{
			if(sc_readl(pl131->regs + SCIRIS) & 0x2000){
				orig_jiffies = jiffies;
				rx_data = sc_readl(pl131->regs + SCIDATA);
				*pBuf++ = rx_data;
				uBufLen++;
			}

			if (time_after(jiffies, orig_jiffies + msecs_to_jiffies(120))) {			
				//dev_dbg
				break;
			}

		}
		
		pATR[uBufLen] = 0;
		*puATRLen = uBufLen;

		if(*puATRLen > 0){
			ret = sc_parse_atr(filp, pATR, *puATRLen);
			if(ret){
				dev_err(pl131->dev, "failed to parse ATR\n");
				return PL131_SC_ERROR_INVALID_ATR;
			}
			else {
				sc_config(filp); // set smartcard clock and baud by ATR	
				return PL131_SC_SUCCESS;
			}
		}

		msleep(30); // It needs debounce time
	}
	return PL131_SC_ERROR_INVALID_ATR;
}

static int pl131_sc_select_vcc_level(struct file *filp, unsigned uVccLevel)
{
	struct miscdevice *misc = filp->private_data;
	struct pl131_sc_data *pl131 = dev_get_drvdata(misc->parent);
	
	dev_dbg(pl131->dev, "%s(%d)\n", __func__, uVccLevel);
	
	if(uVccLevel != PL131_SC_VOLTAGE_LEVEL_3V \
		&& uVccLevel != PL131_SC_VOLTAGE_LEVEL_5V) {
		dev_err(pl131->dev, "%s : invalid param(%d)\n", __func__, uVccLevel);
		return PL131_SC_ERROR_INVALID_PARAM;
	}

	if(uVccLevel == PL131_SC_VOLTAGE_LEVEL_3V)
		gpio_set_value_cansleep(pl131->port_info->VccSel, 0);
	else
		gpio_set_value_cansleep(pl131->port_info->VccSel, 1);

	return 0;
}

static unsigned pl131_sc_detect_card(struct file *filp)
{
	struct miscdevice *misc = filp->private_data;
	struct pl131_sc_data *pl131 = dev_get_drvdata(misc->parent);
	int ret;

	dev_dbg(pl131->dev, "%s\n", __func__);

	gpio_set_value_cansleep(pl131->port_info->Detect, 1);
	ret = gpio_get_value_cansleep(pl131->port_info->Detect);
	if(!ret) {
		if(pl131->giSCState == PL131_SC_STATE_ACTIVATE) {
			pl131_sc_activate(filp, 0); //deactivate
			pl131->giSCState = PL131_SC_STATE_ENABLE;
		}

		if(pl131->giSCState == PL131_SC_STATE_ENABLE) {
			pl131_sc_enable(filp, 0); // disable
			pl131->giSCState = PL131_SC_STATE_OPEN;
		}
	}

	return ret;
}

static void pl131_sc_set_config(struct file *filp, stPL131_SC_CONFIG stSCConfig)
{
	struct miscdevice *misc = filp->private_data;
	struct pl131_sc_data *pl131 = dev_get_drvdata(misc->parent);
	
	dev_dbg(pl131->dev, "%s(%d, %d, %d, %d, %d, %d, %d, %d, %d)\n", __func__, \
			stSCConfig.uiProtocol, stSCConfig.uiConvention, stSCConfig.uiParity, stSCConfig.uiErrorSignal, \
			stSCConfig.uiFlowControl, stSCConfig.uiClock, stSCConfig.uiBaudrate, stSCConfig.uiWaitTime, stSCConfig.uiGuardTime);

	pl131->baud_info->N = stSCConfig.uiGuardTime;
	pl131->baud_info->WT = stSCConfig.uiWaitTime;
	
	//Calculate GT
	pl131->baud_info->ETU = (pl131->baud_info->Fi * 1000000) / (pl131->baud_info->Di * (pl131->baud_info->Fout / 1000)); //nsec
	pl131->baud_info->GT = ((12 * pl131->baud_info->ETU) + (pl131->baud_info->N * pl131->baud_info->ETU)) / 1000; //usec
	return;
}

static void pl131_sc_get_config(struct file *filp, stPL131_SC_CONFIG *pstSCConfig)
{
	struct miscdevice *misc = filp->private_data;
	struct pl131_sc_data *pl131 = dev_get_drvdata(misc->parent);
	
	dev_dbg(pl131->dev, "%s\n", __func__);
	
	pstSCConfig->uiProtocol = pl131->sc_atr->uiProtocol;

	pstSCConfig->uiClock = pl131->baud_info->Fout;
	pstSCConfig->uiBaudrate = pl131->baud_info->Fi/pl131->baud_info->Di;
	pstSCConfig->uiGuardTime = pl131->baud_info->N;
	pstSCConfig->uiWaitTime = pl131->baud_info->WT;

	dev_dbg(pl131->dev, "%s(%d, %d, %d, %d, %d, %d, %d, %d, %d)\n", __func__, \
			pstSCConfig->uiProtocol, pstSCConfig->uiConvention, pstSCConfig->uiParity, pstSCConfig->uiErrorSignal, \
			pstSCConfig->uiFlowControl, pstSCConfig->uiClock, pstSCConfig->uiBaudrate, pstSCConfig->uiWaitTime, pstSCConfig->uiGuardTime);

	return;
}

static unsigned pl131_sc_set_io_pin_config(struct file *filp, unsigned uPinMask)
{
	struct miscdevice *misc = filp->private_data;
	struct pl131_sc_data *pl131 = dev_get_drvdata(misc->parent);
	
	int ret=0;

	dev_dbg(pl131->dev, "%s(%d)\n", __func__, uPinMask);

	// by needs

	return ret;
}

static void pl131_sc_get_io_pin_config(struct file *filp, unsigned *puPinMask)
{
	struct miscdevice *misc = filp->private_data;
	struct pl131_sc_data *pl131 = dev_get_drvdata(misc->parent);
	
	dev_dbg(pl131->dev, "%s\n", __func__);
	
	// by needs

	dev_dbg(pl131->dev, "%s(%d)\n", __func__, *puPinMask);

	return;
}

static int pl131_sc_send_receive_len(struct file *filp, stPL131_SC_BUF stSCBuf)
{
	struct miscdevice *misc = filp->private_data;
	struct pl131_sc_data *pl131 = dev_get_drvdata(misc->parent);
	int BWTmsec = (pl131->baud_info->BWT/1000)+1;
	unsigned long orig_jiffies = jiffies;
	unsigned char *pBuf;
	unsigned uBufLen = 0;
	unsigned rx_data = 0;
	int i,ret;
	
	dev_dbg(pl131->dev, "%s\n", __func__);
	
	if(pl131->giSCState != PL131_SC_STATE_ACTIVATE) {
		dev_err(pl131->dev, "%s : invalid state(%d)\n", __func__, pl131->giSCState);
		return PL131_SC_ERROR_INVALID_STATE;
	}

	// Init Rx Buffer
	//memset(pstBuf, 0x00, sizeof(sc_buf_t));

	//Send Command
	if(stSCBuf.pucTxBuf && stSCBuf.uiTxBufLen){
		
		if(stSCBuf.uiTxBufLen > 0){
			pBuf = stSCBuf.pucTxBuf;
			uBufLen = stSCBuf.uiTxBufLen;
			sc_writel((sc_readl(pl131->regs + SCICR1) | (1<<2)), pl131->regs + SCICR1); // tx mode
			sc_writel((sc_readl(pl131->regs + SCICR0) & (~(1<<3))), pl131->regs + SCICR0); 
			if(pl131->sc_atr->uiProtocol == 0)
				sc_writel(sc_readl(pl131->regs + SCICR0) | (1<<3), pl131->regs +SCICR0); // T0
			else
				sc_writel(sc_readl(pl131->regs + SCICR0) | (0<<3), pl131->regs +SCICR0); // T1

			orig_jiffies = jiffies;
			while(uBufLen)
			{
				if(sc_readl(pl131->regs + SCIFIFOSTATUS) & 0x2){
					for(i=0; i<PL131_FIFO_SIZE; i++){
						if(pl131->sc_atr->uiConvention == PL131_SC_INVERSE_CONVENTION)
							sc_writel(sc_ts_pattern_inverse(*pBuf), pl131->regs + SCIDATA);
						else
							sc_writel(*pBuf, pl131->regs + SCIDATA);

						orig_jiffies = jiffies;
						pBuf++;
						uBufLen--;
						if(uBufLen == 0) break;
					}
				}
				
				if (time_after(jiffies, orig_jiffies + msecs_to_jiffies(120))) {			
					//dev_dbg
					break;
				}
				
			}
		}
	}

	udelay(BWTmsec);

	if(stSCBuf.pucRxBuf && stSCBuf.puiRxBufLen){
		pBuf = stSCBuf.pucRxBuf;
		uBufLen = 0;
		sc_writel((sc_readl(pl131->regs + SCICR1) & ~(1<<2)), pl131->regs + SCICR1); // rx mode

		while(1)
		{
			if(sc_readl(pl131->regs + SCIRIS) & 0x2000){
				orig_jiffies = jiffies;
				rx_data = sc_readl(pl131->regs + SCIDATA);
				*pBuf++ = rx_data;
				uBufLen++;
			}

			if (time_after(jiffies, orig_jiffies + msecs_to_jiffies(100))) {			
				//dev_dbg
				break;
			}

		}
		
		*stSCBuf.puiRxBufLen = uBufLen;
	}
	
	if(*stSCBuf.puiRxBufLen <= 0) {
		ret = PL131_SC_ERROR_TIMEOUT;
		dev_err(pl131->dev, "%s(%d)\n", __func__, ret);
		return ret;
	}

	return PL131_SC_SUCCESS;
}

//---------------------------------------------------------------------------
// DEFINITION OF FILE OPERATION FUNCTIONS
//---------------------------------------------------------------------------
static int pl131_sc_open(struct inode *inode, struct file *filp)
{
	struct miscdevice *misc = filp->private_data;
	struct pl131_sc_data *pl131 = dev_get_drvdata(misc->parent);
	
	int ret = 0;
	dev_dbg(pl131->dev, "%s\n", __func__);
	mutex_lock(&pl131->lock);

	if(pl131->giSCState != PL131_SC_STATE_NONE) {
		dev_err(pl131->dev, "%s : invalid state(%d)\n", __func__, pl131->giSCState);
		mutex_unlock(&pl131->lock);
		return PL131_SC_ERROR_INVALID_STATE;
	}

	if(ret) {
		dev_err(pl131->dev, "%s(%d)\n", __func__, ret);
		mutex_unlock(&pl131->lock);
		return ret;
	}

	pl131->giSCState = PL131_SC_STATE_OPEN;
	
	mutex_unlock(&pl131->lock);
	return ret;
}

static ssize_t pl131_sc_read(struct file *filp, char *buf, size_t count, loff_t *f_pos)
{
	struct miscdevice *misc = filp->private_data;
	struct pl131_sc_data *pl131 = dev_get_drvdata(misc->parent);
	
	dev_err(pl131->dev, "%s : unsupport function\n", __func__);
	
	return 0;
}

static ssize_t pl131_sc_write(struct file *filp, const char *buf, size_t count, loff_t *f_pos)
{
	struct miscdevice *misc = filp->private_data;
	struct pl131_sc_data *pl131 = dev_get_drvdata(misc->parent);
	
	dev_err(pl131->dev, "%s : unsupport function\n", __func__);
	
	return 0;
}

static long pl131_sc_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct miscdevice *misc = filp->private_data;
	struct pl131_sc_data *pl131 = dev_get_drvdata(misc->parent);
	
	int ret = PL131_SC_SUCCESS; 
	
	dev_dbg(pl131->dev, "%s(%d)\n", __func__, cmd);
	mutex_lock(&pl131->lock);

	if(pl131->giSCState == PL131_SC_STATE_NONE) {
		dev_err(pl131->dev, "%s : invalid state(%d)\n", __func__, pl131->giSCState);
		mutex_unlock(&pl131->lock);
		return PL131_SC_ERROR_INVALID_STATE;
	}
	
	switch(cmd)
	{
		case PL131_SC_IOCTL_ENABLE:
			{
				unsigned uEnable;	
				if(copy_from_user((void *)&uEnable, (const void *)arg, sizeof(unsigned))) {
					dev_err(pl131->dev, "%s : copy_from_user failed\n", __func__);
					mutex_unlock(&pl131->lock);
					return PL131_SC_ERROR_UNKNOWN;
				}
				ret = pl131_sc_enable(filp, uEnable);
			}
			break;

		case PL131_SC_IOCTL_ACTIVATE:
			{
				unsigned uActivate;	
				if(copy_from_user((void *)&uActivate, (const void *)arg, sizeof(unsigned))) {
					dev_err(pl131->dev, "%s : copy_from_user failed\n", __func__);
					mutex_unlock(&pl131->lock);
					return PL131_SC_ERROR_UNKNOWN;
				}
				ret = pl131_sc_activate(filp, uActivate);
			}
			break;

		case PL131_SC_IOCTL_RESET:
			{
				stPL131_SC_BUF stSCBuf;	
				if(copy_from_user((void *)&stSCBuf, (const void *)arg, sizeof(stPL131_SC_BUF))) {
					dev_err(pl131->dev, "%s : copy_from_user failed\n", __func__);
					mutex_unlock(&pl131->lock);
					return PL131_SC_ERROR_UNKNOWN;
				}
				ret = pl131_sc_reset(filp, stSCBuf.pucRxBuf, stSCBuf.puiRxBufLen);
			}
			break;

		case PL131_SC_IOCTL_SET_VCC_LEVEL:
			{
				unsigned uVccLevel;	
				if(copy_from_user((void *)&uVccLevel, (const void *)arg, sizeof(unsigned))) {
					dev_err(pl131->dev, "%s : copy_from_user failed\n", __func__);
					mutex_unlock(&pl131->lock);
					return PL131_SC_ERROR_UNKNOWN;
				}
				ret = pl131_sc_select_vcc_level(filp, uVccLevel);
			}
			break;

		case PL131_SC_IOCTL_DETECT_CARD:
			{
				unsigned uPresence;
				uPresence = pl131_sc_detect_card(filp); 
				if(copy_to_user((unsigned *)arg, &uPresence, sizeof(unsigned))) {
					dev_err(pl131->dev, "%s : copy_to_user failed\n", __func__);
					mutex_unlock(&pl131->lock);
					return PL131_SC_ERROR_UNKNOWN;
				}
			}
			break;
			
		case PL131_SC_IOCTL_SET_CONFIG:
			{
				stPL131_SC_CONFIG stSCConfig;
				if(copy_from_user((void *)&stSCConfig, (const void *)arg, sizeof(stPL131_SC_CONFIG))) {
					dev_err(pl131->dev, "%s : copy_from_user failed\n", __func__);
					mutex_unlock(&pl131->lock);
					return PL131_SC_ERROR_UNKNOWN;
				}
				 pl131_sc_set_config(filp, stSCConfig);
			}
			break;

		case PL131_SC_IOCTL_GET_CONFIG:
			{
				stPL131_SC_CONFIG stSCConfig;	
				pl131_sc_get_config(filp, &stSCConfig);
				if(copy_to_user((unsigned *)arg, &stSCConfig, sizeof(stPL131_SC_CONFIG))) {
					dev_err(pl131->dev, "%s : copy_to_user failed\n", __func__);
					mutex_unlock(&pl131->lock);
					return PL131_SC_ERROR_UNKNOWN;
				}
			}
			break;

		case PL131_SC_IOCTL_SET_IO_PIN_CONFIG:
			{
				unsigned uPinMask;	
				if(copy_from_user((void *)&uPinMask, (const void *)arg, sizeof(unsigned))) {
					dev_err(pl131->dev, "%s : copy_from_user failed\n", __func__);
					mutex_unlock(&pl131->lock);
					return PL131_SC_ERROR_UNKNOWN;
				}
				ret  = pl131_sc_set_io_pin_config(filp, uPinMask);				
			}
			break;

		case PL131_SC_IOCTL_GET_IO_PIN_CONFIG:
			{
				unsigned uPinMask;	
				pl131_sc_get_io_pin_config(filp, &uPinMask);
				if(copy_to_user((unsigned *)arg, &uPinMask, sizeof(unsigned))) {
					dev_err(pl131->dev, "%s : copy_to_user failed\n", __func__);
					mutex_unlock(&pl131->lock);
					return PL131_SC_ERROR_UNKNOWN;
				}
			}
			break;

		case PL131_SC_IOCTL_SEND_RCV:
			{
				stPL131_SC_BUF stSCBuf;	
				if(copy_from_user((void *)&stSCBuf, (const void *)arg, sizeof(stPL131_SC_BUF))) {
					dev_err(pl131->dev, "%s : copy_from_user failed\n", __func__);
					mutex_unlock(&pl131->lock);
					return PL131_SC_ERROR_UNKNOWN;
				}
				ret = pl131_sc_send_receive_len(filp, stSCBuf);
			}
			break;
		default:
			{
				dev_err(pl131->dev, "%s: unkown cmd(%d)\n", __func__, cmd);
				ret = -1;
			}
			break;
	}

	mutex_unlock(&pl131->lock);
	
	return ret;
}

static int pl131_sc_close(struct inode *inode, struct file *filp)
{
	struct miscdevice *misc = filp->private_data;
	struct pl131_sc_data *pl131 = dev_get_drvdata(misc->parent);
	
	mutex_lock(&pl131->lock);

	if(pl131->giSCState == PL131_SC_STATE_NONE) {
		dev_err(pl131->dev, "%s : invalid state(%d)\n", __func__, pl131->giSCState);
		mutex_unlock(&pl131->lock);
		return -1;
	}

	if(pl131->giSCState == PL131_SC_STATE_ACTIVATE) {
		pl131_sc_activate(filp, 0); //deactivate
		pl131->giSCState = PL131_SC_STATE_ENABLE;
	}

	if(pl131->giSCState == PL131_SC_STATE_ENABLE) {
		pl131_sc_enable(filp, 0); // disable
		pl131->giSCState = PL131_SC_STATE_OPEN;
	}

	// TODO: free resource
	
	pl131->giSCState = PL131_SC_STATE_NONE;
	
	mutex_unlock(&pl131->lock);
		
	return 0;
}

static struct file_operations pl131_sc_fops = 
{
	.owner		= THIS_MODULE,
	.open		= pl131_sc_open,
	.read		= pl131_sc_read,
	.write		= pl131_sc_write,
	.unlocked_ioctl	= pl131_sc_ioctl,
	.release	= pl131_sc_close,	
};

static inline struct pl131_sc_data *pl131_sc_parse_dt(struct platform_device *pdev)
{
	struct pl131_sc_data *pdata;
	struct device_node *np = pdev->dev.of_node;
	struct device_node *uart_dt;

	pdata = devm_kzalloc(&pdev->dev, sizeof(struct pl131_sc_data), GFP_KERNEL);
	pdata->baud_info = devm_kzalloc(&pdev->dev, sizeof(baud_info_t), GFP_KERNEL);
	pdata->port_info = devm_kzalloc(&pdev->dev, sizeof(port_info_t), GFP_KERNEL);

	if(of_property_read_u32(np, "clock-frequency", &pdata->baud_info->Fuart))
	{
		pr_err("failed to get clock-frequency from dt\n");
		goto err_parse_dt;
	}

	if(of_property_read_u32(np, "smartcard-clock", &pdata->baud_info->Fout))
	{
		pr_err("failed to get smartcard-clock from dt\n");
		goto err_parse_dt;
	}

	if(of_property_read_u32(np, "Fi", &pdata->baud_info->Fi))
	{
		pr_err("failed to get clock rate conversion integer from dt\n");
		goto err_parse_dt;
	}

	if(of_property_read_u32(np, "Di", &pdata->baud_info->Di))
	{
		pr_err("failed to baud rate adjustment integer from dt\n");
		goto err_parse_dt;
	}	
	
	if(of_property_read_u32(np, "port-num", &pdata->port_info->PortNum))
	{
		pr_err("failed to get port-num from dt\n");
		goto err_parse_dt;
	}

	if(of_property_read_u32(np, "uart_chnum", &pdata->port_info->Uart_chNum))
	{
		pr_err("failed to get uart channel num from dt\n");
		goto err_parse_dt;
	}
	
	if(of_property_read_u32(np, "tda8024-use", &pdata->port_info->TDA8024))
	{
		pr_err("failed to get dta8024-use from dt\n");
		goto err_parse_dt;
	}
	
	pdata->port_info->Rst = of_get_named_gpio(np, "rst_gpios",0);
	if(!gpio_is_valid(pdata->port_info->Rst)){
		pr_err("failed to get rst_gpios from dt\n");
	}
	
	pdata->port_info->Detect = of_get_named_gpio(np, "detect_gpios",0);
	if(!gpio_is_valid(pdata->port_info->Detect)){
		pr_err("failed to get detect_gpios from dt\n");
	}
	
	pdata->port_info->PwrEn = of_get_named_gpio(np, "pwren_gpios",0);
	if(!gpio_is_valid(pdata->port_info->PwrEn)){
		pr_err("failed to get pwren_gpios from dt\n");
	}

	uart_dt = of_parse_phandle(np, "uart_ch", 0); // uart and smartcard share clock.
	pdata->irq = irq_of_parse_and_map(uart_dt, 0); //uart and smartcard share irq number.

	pdata->pclk = of_clk_get(uart_dt, 0);
	if(pdata->pclk == NULL){
		pr_err("failed to get pclk, %s\n", __func__);
		goto err_parse_dt;
	}

	pdata->hclk = of_clk_get(uart_dt, 1);
	if(pdata->hclk == NULL){
		pr_err("failed to get hclk, %s\n", __func__);
		goto err_parse_dt;
	}

	pdata->shclk = of_clk_get(np, 0);

	return pdata;

err_parse_dt:
	//dev_err(pdev->dev, "Parsing device tree data error.\n");
	return NULL;
}

static int pl131_init(struct pl131_sc_data *pdata)
{
	int sciclk_value;
	unsigned int baud_div, scibaud, scivalue=1;	
	
	//clk init
	if(clk_prepare_enable(pdata->hclk) != 0){
		dev_err(pdata->dev, "can't do smartcard hclk clock enable\n");
		goto err_init;
	}

	if(clk_prepare_enable(pdata->shclk) != 0){
		dev_err(pdata->dev, "can't do smartcard hclk clock enable\n");
		goto err_init;
	}


	if(clk_prepare_enable(pdata->pclk) != 0){
		dev_err(pdata->dev, "can't do smartcard pclk clock enable\n");
		goto err_init;
	}

	clk_set_rate(pdata->pclk, pdata->baud_info->Fuart);

	//port CFG 
	if(pdata->port_info->Uart_chNum > 3)
		sc_writel(sc_readl(pdata->port_cfg_reg + 4) |(pdata->port_info->PortNum << ((pdata->port_info->Uart_chNum-4)*8)), pdata->port_cfg_reg + 4);
	else
		sc_writel(sc_readl(pdata->port_cfg_reg) |(pdata->port_info->PortNum << (pdata->port_info->Uart_chNum*8)), pdata->port_cfg_reg);

	sc_writel(0x1f001f1f, pdata->port_cfg_reg + 4);
	
	// power init
	if(pdata->port_info->TDA8024)
		gpio_direction_output(pdata->port_info->PwrEn, 0);
	else
		gpio_direction_output(pdata->port_info->PwrEn, 1);
	
	//initial configuration of registers
	sc_writel(0x0, pdata->regs + SCICR0); // direct convention, even parity, receive/transmit handshaking disabled
	sc_writel(0x0, pdata->regs + SCICR1); // reveive mode, timeouts initially disabled
	sc_writel(0x7FFF, pdata->regs + SCIIMSC); // // enable all interrupts
	sc_writel(0x96, pdata->regs + SCISTABLE); 
	sc_writel(45000, pdata->regs + SCIATIME);  // recommand 40000~45000 clock cycle 
	sc_writel(0xC8, pdata->regs + SCIDTIME);
	sc_writel(40000, pdata->regs + SCIATRSTIME); // ATR sequence must occur within 40000 clock cycle
	sc_writel(19200, pdata->regs + SCIATRDTIME); // The complete ATR character sequence must be received within 19200 etu
	
	sciclk_value = (pdata->baud_info->Fuart/(2*pdata->baud_info->Fout) -1); // initial value by device tree
	sc_writel(sciclk_value, pdata->regs + SCICLKICC); // set smartcard clock , initial value by device tree

	pdata->baud_info->Baud = (pdata->baud_info->Fout * pdata->baud_info->Di)/pdata->baud_info->Fi;
	baud_div = pdata->baud_info->Fuart/pdata->baud_info->Baud;

	scivalue = 10; // minimum value 5
	baud_div = baud_div/scivalue;
	
	while(baud_div>0xFFFF){
		scivalue++;
		baud_div = baud_div/scivalue;
	}
	
	scibaud = baud_div;
	sc_writel(scivalue, pdata->regs + SCIVALUE); // set baudrate with SCIVALUE and SCIBAUD
	sc_writel(scibaud, pdata->regs + SCIBAUD);

	return 0;

err_init:
	return -1;
}
	
static int pl131_sc_probe(struct platform_device *pdev)
{
	struct pl131_sc_data *pdata = NULL;
	struct device_node *np = pdev->dev.of_node;
	int ret;

	if(!pdata)
		pdata = pl131_sc_parse_dt(pdev);

	dev_dbg(pdata->dev, "%s\n", __func__);

	if(np)
		pdata->regs = of_iomap(np, 0);
	else
	       ; // TODO: get platform resource

	if(np)
		pdata->port_cfg_reg= of_iomap(np, 1);
	else
	       ; // TODO: get platform resource

	pdata->sc_buf = devm_kzalloc(&pdev->dev, sizeof(sc_buf_t), GFP_KERNEL);
	if (!pdata->sc_buf)
       {
           dev_err(&pdev->dev, "Failed to allocate SC Buf driver structure\n");
           return -ENOMEM;
       }
	
	pdata->sc_atr = devm_kzalloc(&pdev->dev, sizeof(sc_atr_t), GFP_KERNEL);
	if (!pdata->sc_atr)
       {
           dev_err(&pdev->dev, "Failed to allocate SC ATR driver structure\n");
           return -ENOMEM;
       }

	pdata->misc = devm_kzalloc(&pdev->dev, sizeof(struct miscdevice), GFP_KERNEL);
	if (!pdata->misc)
       {
           dev_err(&pdev->dev, "Failed to allocate sc misc driver structure\n");
           return -ENOMEM;
       }
	
	
	pdata->misc->minor = MISC_DYNAMIC_MINOR;
	pdata->misc->fops = &pl131_sc_fops;
	pdata->misc->name = SC_DEVICE_NAME;
	pdata->misc->parent = &pdev->dev;

	ret = misc_register(pdata->misc);
	if(ret){
		dev_err(pdata->dev, "[SmartCard]: Couldn't register device %d.\n", MINOR_ID);
		goto err_sc;
	}

	pdata->dev = &pdev->dev;
	platform_set_drvdata(pdev, pdata);
	mutex_init(&pdata->lock);
	
	ret = pl131_init(pdata);
	if(ret){
		dev_err(pdata->dev, "Failed to init smartcard\n");
		goto err_sc;
	}

	return 0;
	
err_sc:
	platform_set_drvdata(pdev, NULL);
	return ret;
}

static int pl131_sc_remove(struct platform_device *pdev)
{
	struct pl131_sc_data *pl131 = (struct pl131_sc_data *)platform_get_drvdata(pdev);
//    tca_sc_remove(); //todo

    misc_deregister(pl131->misc);

    return 0;
}

static struct of_device_id sc_dt_ids[] = {
    { .compatible = "telechips,pl131-smartcard", },
    { },
};
MODULE_DEVICE_TABLE(of, sc_dt_ids);


static struct platform_driver sc_driver =
{
    .driver = {
        .name   = SC_DEVICE_NAME,
        .owner  = THIS_MODULE,
        .of_match_table = of_match_ptr(sc_dt_ids),
    },
    .probe  = pl131_sc_probe,
    .remove = pl131_sc_remove,
};
module_platform_driver(sc_driver);


MODULE_AUTHOR("bsp <bsp@telechips.com>");
MODULE_DESCRIPTION("Telechips PL131 SmartCard driver");
MODULE_LICENSE("GPL");
