/*
 * linux/arch/arm/mach-tcc893x/tca_cipher.c
 *
 * Author:  <linux@telechips.com>
 * Created: March 18, 2010
 * Description: TCC Cipher driver
 *
 * Copyright (C) 20010-2011 Telechips 
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
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/wait.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/cpufreq.h>
#ifdef CONFIG_PM
#include <linux/pm.h>
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/div64.h>
#include <asm/mach/map.h>
#include <mach/bsp.h>
#include <mach/gpio.h>
#include <mach/tcc_cipher_ioctl.h>
#include <mach/tca_cipher.h>


#define MAX_CIPHER_BUFFER_LENGTH	4096
#define MIN_CIPHER_BLOCK_SIZE		8
#define DES_BLOCK_SIZE				8
#define AES_BLOCK_SIZE				16
#define MULTI2_BLOCK_SIZE			8
#define CSA_BLOCK_SIZE				1
#define HASH_BLOCK_SIZE			1


static int debug = 0;
#define dprintk(msg...)	if(debug) { printk( "tca_cipher: " msg); }

static int iDoneIrqHandled = FALSE;
static dma_addr_t SrcDma;	/* physical */
static u_char *pSrcCpu;		/* virtual */
static dma_addr_t DstDma;	/* physical */
static u_char *pDstCpu;		/* virtual */

#ifndef CONFIG_OF
#ifdef CONFIG_CLOCK_TABLE
#include <mach/clocktable.h>
struct func_clktbl_t *cipher_clktbl = NULL;
#endif
static struct clk *cipher_clk = NULL;
#endif

//#define UESD_IRQ

static char key_swapbuf[128];
static char iv_swapbuf[128];

static unsigned int	uiDefault_BlockSize = MIN_CIPHER_BLOCK_SIZE;
static struct cipher_device tca_cipher_dev;

static DEFINE_MUTEX(tca_cipher_mutex);

static void tca_cipher_data_swap (unsigned char *srcdata, unsigned char *destdata, int size)
{
	unsigned int		i;
	for(i=0;i<=size; i+=4)
	{
		destdata[i] = srcdata[i+3];
		destdata[i+1] = srcdata[i+2];
		destdata[i+2] = srcdata[i+1];
		destdata[i+3] = srcdata[i];
	}
}

void tca_cipher_reg_init(void)
{
	unsigned data;

	data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_DCTRL);
	data =0x78000; 
	writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_DCTRL);

	data =0; 
	writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_CCTRL);
	writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_ADM);
	writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_HASH);
	writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_CSA3);
	writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_ROUND);
	writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY0);
	writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY1);
	writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY2);
	writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY3);
	writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY4);
	writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY5);
	writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY6);
	writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY7);
	writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY8);
	writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY9);
	writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_IV0);
	writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_IV1);
	writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_IV2);
	writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_IV3);

}

void tca_cipher_interrupt_enable(unsigned  int uEnable)
{
	unsigned data;

	dprintk("%s, Enabel=%d\n", __func__, uEnable);

	if(uEnable)
	{
		/* Cipher Block Enable Start*/
		data = readl(tca_cipher_dev.vpic_regs + TCC897x_VPIC_CLR1);
		data |= Hw20;
		writel(data, tca_cipher_dev.vpic_regs + TCC897x_VPIC_CLR1);

		data = readl(tca_cipher_dev.vpic_regs + TCC897x_VPIC_SEL1);
		data |=Hw20;
		writel(data, tca_cipher_dev.vpic_regs + TCC897x_VPIC_SEL1);

		data = readl(tca_cipher_dev.vpic_regs + TCC897x_VPIC_POL1);
		data &=~Hw20;
		writel(data, tca_cipher_dev.vpic_regs + TCC897x_VPIC_POL1);

		data = readl(tca_cipher_dev.vpic_regs + TCC897x_VPIC_MODE1);
		data &=~Hw20;
		writel(data, tca_cipher_dev.vpic_regs + TCC897x_VPIC_MODE1);

		data = readl(tca_cipher_dev.vpic_regs + TCC897x_VPIC_IEN1);
		data |=Hw20;
		writel(data, tca_cipher_dev.vpic_regs + TCC897x_VPIC_IEN1);
		/* Cipher Block Enable End*/
	}
	else
	{
		data = readl(tca_cipher_dev.vpic_regs + TCC897x_VPIC_INTMSK1);
		data &=~Hw20;
		writel(data, tca_cipher_dev.vpic_regs + TCC897x_VPIC_INTMSK1);

		data = readl(tca_cipher_dev.vpic_regs + TCC897x_VPIC_IEN1);
		data &=~Hw20;
		writel(data, tca_cipher_dev.vpic_regs + TCC897x_VPIC_IEN1);
	}
}

irqreturn_t tca_cipher_interrupt_handler(int irq, void *dev_id)
{
	unsigned data;
	dprintk("%s\n", __func__);

	/* Set Done Interrupt Flag */
	iDoneIrqHandled = FALSE;
	dprintk("%s iDoneIrqHandled = %d \n", __func__, iDoneIrqHandled);
	data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_INTSTS);
	writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_INTSTS);
	return IRQ_HANDLED;
}

void tca_cipher_wait_done(void)
{
	unsigned data;
	dprintk("%s iDoneIrqHandled = %d \n", __func__, iDoneIrqHandled);

	/* Wait for Done Interrupt */
	while(1)
	{
		dprintk("%s iDoneIrqHandled = %d \n", __func__, iDoneIrqHandled);
		if(!iDoneIrqHandled)
		{
			dprintk("Receive Done IRQ Handled\n");
			break;
		}
	}
}

void tca_cipher_set_opmode(unsigned  int uOpMode, unsigned  int uDmaMode)
{
	unsigned data;
	dprintk("%s, Operation Mode: %d\n", __func__, uOpMode);
	
	data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_ADM);
	data |=TCC897x_CIPHER_ADM_OPMODE(uOpMode); 
	writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_ADM);
	dprintk("%s, Operation Mode Set End\n", __func__);
	
}

void tca_cipher_set_algorithm(unsigned  int uAlgorithm, unsigned  int uArg1, unsigned  int uArg2)
{
	unsigned data;
	dprintk("%s, Algorithm: %d, uArg1=%d, uArg2 = %d \n", __func__, uAlgorithm, uArg1, uArg2);
	
	switch(uAlgorithm)
	{
		case TCC_CIPHER_ALGORITM_BYPASS:
			{
				data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_CCTRL);
				data |=TCC897x_CIPHER_CCTRL_SELECT(HwCIPHER_CCTRL_Algorithm_BYPASS); 
				writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_CCTRL);
			}
			break;

		case TCC_CIPHER_ALGORITM_AES:
			{
				/* uArg1: The Key Length in AES */
				/* uArg2: None                  */
				data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_CCTRL);
				data |=TCC897x_CIPHER_CCTRL_SELECT(HwCIPHER_CCTRL_Algorithm_AES); 
				writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_CCTRL);

				data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_ADM);
				data |=TCC897x_CIPHER_ADM_AESMODE(uArg1);
				writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_ADM);
				uiDefault_BlockSize = AES_BLOCK_SIZE;
			}
			break;

		case TCC_CIPHER_ALGORITM_DES:
			{
				/* uArg1: The Mode in DES     */
				/* uArg2: Parity Bit Location */
				data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_CCTRL);
				data |=TCC897x_CIPHER_CCTRL_SELECT(HwCIPHER_CCTRL_Algorithm_DES);
				writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_CCTRL);

				data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_ADM);
				data |=TCC897x_CIPHER_ADM_DESMODE(uArg1);
				writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_ADM);

				data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_ADM);
				if(uArg2)
					data |=HwCIPHER_ADM_ParityBit;
				else
					data &=~HwCIPHER_ADM_ParityBit;
				writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_ADM);
				uiDefault_BlockSize = DES_BLOCK_SIZE;
			}
			break;

		case TCC_CIPHER_ALGORITM_MULTI2:
			{
				/* uArg1: Round in Multi2 */
				/* uArg2: None */
				data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_CCTRL);
				data |=TCC897x_CIPHER_CCTRL_SELECT(HwCIPHER_CCTRL_Algorithm_MULTI2);
				writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_CCTRL);

				data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_ROUND);
				data =uArg1*8;
				writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_ROUND);
				uiDefault_BlockSize = MULTI2_BLOCK_SIZE;
			}
			break;
			
		case TCC_CIPHER_ALGORITM_CSA2:
			{
				data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_CCTRL);
				data |=TCC897x_CIPHER_CCTRL_SELECT(HwCIPHER_CCTRL_Algorithm_CSA2);
				writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_CCTRL);

				data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_ROUND);
				data =uArg1;
				writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_ROUND);
				uiDefault_BlockSize = CSA_BLOCK_SIZE;
			}
			break;

		case TCC_CIPHER_ALGORITM_CSA3:
			{
				data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_CCTRL);
				data |=TCC897x_CIPHER_CCTRL_SELECT(HwCIPHER_CCTRL_Algorithm_CSA3);
				writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_CCTRL);

				data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_ROUND);
				data =uArg1;
				writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_ROUND);

				uiDefault_BlockSize = CSA_BLOCK_SIZE;
			}
			break;

		case TCC_CIPHER_ALGORITM_HASH:
			{
				/* uArg1: Mode in Hash */
				/* uArg2: Round in Hash */
				data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_CCTRL);
				data |=TCC897x_CIPHER_CCTRL_SELECT(HwCIPHER_CCTRL_Algorithm_HASH);
				writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_CCTRL);

				data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_HASH);
				data =TCC897x_CIPHER_HASH_MODE(uArg1);
				writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_HASH);

				data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_ROUND);
				data =uArg2;
				writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_ROUND);
				uiDefault_BlockSize = CSA_BLOCK_SIZE;
			}
			break;

		default:
			dprintk("%s, Err: Invalid Algorithm\n", __func__);
			break;
	}

	dprintk("%s, Algorithm Set End\n", __func__);
	
}

void tca_cipher_set_baseaddr(unsigned  int uTxRx, unsigned char *pBaseAddr)
{
	unsigned data;

	dprintk("%s\n", __func__);

	if(uTxRx == TCC_CIPHER_BASEADDR_TX)
	{
		data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_SBASE);
		data =(unsigned long)pBaseAddr;
		writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_SBASE);
	}
	else
	{
		data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_DBASE);
		data =(unsigned long)pBaseAddr;
		writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_DBASE);
	}
}

/*keymode : "0= use normal key", "1= use key from OTP"  */
int tca_cipher_set_keymode(unsigned  int keymode)
{
	unsigned data;

	if(keymode>1)
	{
		printk("[%s] : KeyMode Set Error.. inupt_keymode = %d \n", __func__, keymode);
		return -1;
	}
	else
	{
		if(keymode)	
		{
			data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_CCTRL);
			data |=HwCIPHER_ADM_KEYLD;
			writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_CCTRL);
		}
		else
		{
			data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_CCTRL);
			data &=~HwCIPHER_ADM_KEYLD;
			writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_CCTRL);
		}
	}
	return 0;
}

void tca_cipher_set_key(unsigned char *pucData, unsigned  int uLength, unsigned  int uOption)
{
	unsigned data;
	unsigned long ulAlgorthm, ulAESKeyLength, ulDESMode;
	unsigned long *pulKeyData;

	dprintk("%s, Lenth: %d, Option: %d\n", __func__, uLength, uOption);

	data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_CCTRL);
	ulAlgorthm =data & HwCIPHER_CCTRL_Algorithm_Mask;
	dprintk("%s, ulAlgorthm: %x\n", __func__, ulAlgorthm);

	pulKeyData = (unsigned long *)pucData;

	if(debug)
	{
		int i;
		unsigned int *pDataAddr;

		pDataAddr = (unsigned int *)pulKeyData;
		printk("\n[ Key data ]\n");
		for(i=0; i<(uLength/4); i+=4)
		{
			printk("0x%08x 0x%08x 0x%08x 0x%08x\n", pDataAddr[i+0], pDataAddr[i+1], pDataAddr[i+2], pDataAddr[i+3]);
		}
	}

	/* Write Key Data */
	switch(ulAlgorthm)
	{
		case HwCIPHER_CCTRL_Algorithm_AES:
			{

				data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_ADM);
				ulAESKeyLength =data & HwCIPHER_ADM_KeyLength_Mask;
				dprintk("%s, ulAESKeyLength: %x\n", __func__, ulAESKeyLength);

				if(ulAESKeyLength == HwCIPHER_ADM_keyLength_128)
				{
					data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY0);
					data =*pulKeyData++;
					writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY0);
					
					data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY1);
					data =*pulKeyData++;
					writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY1);
					
					data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY2);
					data =*pulKeyData++;
					writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY2);
					
					data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY3);
					data =*pulKeyData++;
					writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY3);
				}
				else if(ulAESKeyLength == HwCIPHER_ADM_KeyLength_192)
				{
					data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY0);
					data =*pulKeyData++;
					writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY0);
					
					data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY1);
					data =*pulKeyData++;
					writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY1);
					
					data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY2);
					data =*pulKeyData++;
					writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY2);
					
					data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY3);
					data =*pulKeyData++;
					writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY3);

					data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY4);
					data =*pulKeyData++;
					writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY4);
					
					data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY5);
					data =*pulKeyData++;
					writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY5);
				}
				else if(ulAESKeyLength == HwCIPHER_ADM_KeyLength_256)
				{
					data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY0);
					data =*pulKeyData++;
					writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY0);
					
					data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY1);
					data =*pulKeyData++;
					writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY1);
					
					data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY2);
					data =*pulKeyData++;
					writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY2);
					
					data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY3);
					data =*pulKeyData++;
					writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY3);

					data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY4);
					data =*pulKeyData++;
					writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY4);
					
					data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY5);
					data =*pulKeyData++;
					writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY5);

					data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY6);
					data =*pulKeyData++;
					writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY6);

					data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY7);
					data =*pulKeyData++;
					writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY7);
				}
			}
			break;

		case HwCIPHER_CCTRL_Algorithm_DES:
			{
				data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_ADM);
				ulDESMode =data & HwCIPHER_ADM_DESMode_Mask;
				dprintk("%s, ulDESMode: %x\n", __func__, ulDESMode);

				if(ulDESMode == HwCIPHER_ADM_DESMode_SingleDES)
				{
					data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY0);
					data =*pulKeyData++;
					writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY0);
					
					data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY1);
					data =*pulKeyData++;
					writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY1);
				}
				else if(ulDESMode == HwCIPHER_ADM_DESMode_DoubleDES)
				{
					data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY0);
					data =*pulKeyData++;
					writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY0);
					
					data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY1);
					data =*pulKeyData++;
					writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY1);
					
					data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY2);
					data =*pulKeyData++;
					writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY2);
					
					data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY3);
					data =*pulKeyData++;
					writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY3);
				}
				else if(ulDESMode == HwCIPHER_ADM_DESMode_TripleDES2)
				{
					data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY0);
					data =*pulKeyData++;
					writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY0);
					
					data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY1);
					data =*pulKeyData++;
					writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY1);
					
					data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY2);
					data =*pulKeyData++;
					writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY2);
					
					data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY3);
					data =*pulKeyData++;
					writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY3);
				}
				else if(ulDESMode == HwCIPHER_ADM_DESMode_TripleDES3)
				{
					data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY0);
					data =*pulKeyData++;
					writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY0);
					
					data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY1);
					data =*pulKeyData++;
					writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY1);
					
					data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY2);
					data =*pulKeyData++;
					writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY2);
					
					data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY3);
					data =*pulKeyData++;
					writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY3);

					data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY4);
					data =*pulKeyData++;
					writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY4);
					
					data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY5);
					data =*pulKeyData++;
					writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY5);
				}
			}
			break;

		case HwCIPHER_CCTRL_Algorithm_MULTI2:
			{
				if(uOption == TCC_CIPHER_KEY_MULTI2_DATA)
				{

					data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY0);
					data =*pulKeyData++;
					writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY0);
					
					data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY1);
					data =*pulKeyData++;
					writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY1);
				}
				else
				{
					data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY2);
					data =*pulKeyData++;
					writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY2);
					
					data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY3);
					data =*pulKeyData++;
					writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY3);

					data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY4);
					data =*pulKeyData++;
					writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY4);
					
					data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY5);
					data =*pulKeyData++;
					writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY5);

					data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY6);
					data =*pulKeyData++;
					writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY6);

					data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY7);
					data =*pulKeyData++;
					writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY7);

					data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY8);
					data =*pulKeyData++;
					writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY8);
					
					data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY9);
					data =*pulKeyData++;
					writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY9);
				}
			}
			break;

		case HwCIPHER_CCTRL_Algorithm_CSA2:
			{
				data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY0);
				data =*pulKeyData++;
				writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY0);
				
				data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY1);
				data =*pulKeyData++;
				writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY1);
			}
			break;
			
		case HwCIPHER_CCTRL_Algorithm_CSA3:
			{
				data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY0);
				data =*pulKeyData++;
				writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY0);
				
				data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY1);
				data =*pulKeyData++;
				writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY1);

				data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY2);
				data =*pulKeyData++;
				writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY2);

				data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY3);
				data =*pulKeyData++;
				writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_KEY3);

				data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_CSA3);
				data =1;
				writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_CSA3);
			}
			break;
			

		default:
			break;
	}

	/* Load Key Data */
	tca_cipher_set_keymode(TCC_CIPHER_KEY_NORMAL);
	data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_ADM);
	data |=TCC897x_CIPHER_ADM_KEYLD;
	writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_ADM);
}

void tca_cipher_set_vector(unsigned char *pucData, unsigned  int uLength)
{
	unsigned data;
	unsigned long ulAlgorthm;
	unsigned long *pulVectorData;

	dprintk("%s, Length: %d\n", __func__, uLength);

	data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_CCTRL);
	ulAlgorthm =data & HwCIPHER_CCTRL_Algorithm_Mask;
	dprintk("%s, ulAlgorthm: %x\n", __func__, ulAlgorthm);
	
	pulVectorData = (unsigned long *)pucData;
	if(uLength ==0 )
		return;

	dprintk("%s, ulAlgorthm: %x\n", __func__, ulAlgorthm);

	if(debug)
	{
		int i;
		unsigned int *pDataAddr;
	
		pDataAddr = (unsigned int *)pulVectorData;
		printk("\n[ IV data ]\n");
		for(i=0; i<(uLength/4); i+=4)
		{
			printk("0x%08x 0x%08x 0x%08x 0x%08x\n", pDataAddr[i+0], pDataAddr[i+1], pDataAddr[i+2], pDataAddr[i+3]);
		}
	}


	/* Write Initial Vector */
	switch(ulAlgorthm)
	{
		case HwCIPHER_CCTRL_Algorithm_AES:
			{
				data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_IV0);
				data =*pulVectorData++;
				writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_IV0);
				
				data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_IV1);
				data =*pulVectorData++;
				writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_IV1);
				
				data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_IV2);
				data =*pulVectorData++;
				writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_IV2);
				
				data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_IV3);
				data =*pulVectorData++;
				writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_IV3);
			}
			break;

		case HwCIPHER_CCTRL_Algorithm_DES:
			{
				data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_IV0);
				data =*pulVectorData++;
				writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_IV0);
				
				data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_IV1);
				data =*pulVectorData++;
				writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_IV1);
			}
			break;

		case HwCIPHER_CCTRL_Algorithm_MULTI2:
			{
				data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_IV0);
				data =*pulVectorData++;
				writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_IV0);
				
				data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_IV1);
				data =*pulVectorData++;
				writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_IV1);
			}
			break;

		default:
			break;
	}

	/* Load Initial Vector */
	data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_ADM);
	data |=TCC897x_CIPHER_ADM_IVLD;
	writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_ADM);
}

int tca_cipher_run(unsigned char *pucSrcAddr, unsigned char *pucDstAddr, unsigned  int uLength, unsigned int option)
{
	unsigned data;
	uint		uiresidual=0;
	uint		err = 0;
	unsigned long ulAlgorthm;

 	dprintk("%s, Length=%d\n", __func__, uLength);
	if(uLength<uiDefault_BlockSize)
		return -1;
	
	uiresidual = uLength&(uiDefault_BlockSize-1);
 	dprintk("%s, %d, uiresidual=%d\n", __func__, __LINE__, uiresidual);
	
	if(uiresidual)
	{
	 	dprintk("%s, %d, Length=%d\n", __func__, __LINE__, uLength);
		uLength -= uiresidual;
	 	dprintk("%s, %d, Length=%d, uiresidual = %d \n", __func__, __LINE__, uLength, uiresidual);
	}
	
	if(uLength> MAX_CIPHER_BUFFER_LENGTH)
	{
		if(pSrcCpu != NULL)
			dma_free_coherent(0, MAX_CIPHER_BUFFER_LENGTH, pSrcCpu, SrcDma);
		if(pDstCpu != NULL)
			dma_free_coherent(0, MAX_CIPHER_BUFFER_LENGTH, pDstCpu, DstDma);

		/* Allocate Physical & Virtual Address for DMA */
		pSrcCpu = dma_alloc_coherent(0, uLength, &SrcDma, GFP_KERNEL);
		pDstCpu = dma_alloc_coherent(0, uLength, &DstDma, GFP_KERNEL);
	}

	if((pSrcCpu == NULL) || (pDstCpu == NULL))
	{
//		dprintk("pSrcCpu = %x, pDstCpu = %x\n", pSrcCpu, pDstCpu)
		return -1;
	}

	/* Init Virtual Address */
	memset(pSrcCpu, 0x00, uLength);
	memset(pDstCpu, 0x00, uLength);

	/* Copy Plain Text from Source Buffer */
	err = copy_from_user(pSrcCpu, pucSrcAddr, uLength);
	
	/* Select Encryption */
	data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_ADM);
	if(option == 1)	//encrypt
		data |=TCC897x_CIPHER_ADM_ENC;
	else
		data &=~TCC897x_CIPHER_ADM_ENC;
	writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_ADM);

	data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_SBASE);
	data =SrcDma;
	writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_SBASE);

	data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_DBASE);
	data =DstDma;
	writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_DBASE);

	data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_SIZE);
	data =uLength;
	writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_SIZE);

	data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_DCTRL);
	data =uLength;
	writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_DCTRL);

	data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_ADM);
	data |=TCC897x_CIPHER_ADM_KEYLD;
	writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_ADM);

	data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_ADM);
	data |=TCC897x_CIPHER_ADM_IVLD;
	writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_ADM);

	data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_CCTRL);
	data |=TCC897x_CIPHER_CCTRL_ENDIAN(0);
	writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_CCTRL);

#ifdef UESD_IRQ
	data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_DCTRL);
	data |=TCC897x_CIPHERDCTRL_IEN;
	writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_DCTRL);
#endif
	data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_DCTRL);
	data |=TCC897x_CIPHERDCTRL_EN;
	writel(data, tca_cipher_dev.cipher_regs + TCC897x_CIPHER_DCTRL);


#ifdef UESD_IRQ
	tca_cipher_interrupt_enable(TRUE);
	tca_cipher_wait_done();
#else
	while(1)
	{
		data = readl(tca_cipher_dev.cipher_regs + TCC897x_CIPHER_FIFO0);
		data = data & HwCIPHER_FIFO0_IDLE;

		dprintk("%s, %d, data=%x\n", __func__, __LINE__, data);
		if(data)
			break;
	}
#endif

	/* Copy Cipher Text to Destination Buffer */
	err = copy_to_user(pucDstAddr, pDstCpu, uLength);

	#if 1 /* For Debugging */
	if(debug)
	{
		int i;
		unsigned int *pDataAddr;
		unsigned data1, data2, data3, data4;

		printk("\n[ Register Setting ]\n");
		for(i=0; i<=0x1e0; i+=16)
		{
			data1 = readl(tca_cipher_dev.cipher_regs + i);
			data2 = readl(tca_cipher_dev.cipher_regs + i + 4);
			data3 = readl(tca_cipher_dev.cipher_regs + i + 8);
			data4 = readl(tca_cipher_dev.cipher_regs + i + 12);
			printk("[0x%3x]:0x%08x 0x%08x 0x%08x 0x%08x\n", i, data1, data2, data3, data4);
		}

		pDataAddr = (unsigned int *)pSrcCpu;
		printk("\n[ Source Text ]\n");
		for(i=0; i<(uLength/4); i+=4)
		{
			printk("0x%08x 0x%08x 0x%08x 0x%08x\n", pDataAddr[i+0], pDataAddr[i+1], pDataAddr[i+2], pDataAddr[i+3]);
		}

		pDataAddr = (unsigned int *)pDstCpu;
		printk("\n[ Dest Text ]\n");
		for(i=0; i<((uLength*2)/4); i+=4)
		{
			printk("0x%08x 0x%08x 0x%08x 0x%08x\n", pDataAddr[i+0], pDataAddr[i+1], pDataAddr[i+2], pDataAddr[i+3]);
		}
		printk("\n");
	}
	#endif

	tca_cipher_reg_init();

	if(uLength> MAX_CIPHER_BUFFER_LENGTH)
	{
		dma_free_coherent(0, uLength, pSrcCpu, SrcDma);
		dma_free_coherent(0, uLength, pDstCpu, DstDma);
		
		pSrcCpu = dma_alloc_coherent(0, MAX_CIPHER_BUFFER_LENGTH, &SrcDma, GFP_KERNEL);
		pDstCpu = dma_alloc_coherent(0, MAX_CIPHER_BUFFER_LENGTH, &DstDma, GFP_KERNEL);
	}	

	return 0;
}

int tca_cipher_encrypt(unsigned char *pucSrcAddr, unsigned char *pucDstAddr, unsigned  int uLength)
{
	uint err =0;
	err =tca_cipher_run(pucSrcAddr, pucDstAddr, uLength, 1);
	return err;
}

int tca_cipher_decrypt(unsigned char *pucSrcAddr, unsigned char *pucDstAddr, unsigned  int uLength)
{
	uint err =0;
	err =tca_cipher_run(pucSrcAddr, pucDstAddr, uLength, 0);
	return err;
}


int tca_cipher_open(struct inode *inode, struct file *filp, struct cipher_device *cipher_dev)
{
	mutex_lock(&tca_cipher_mutex);
	tca_cipher_dev = *cipher_dev;

	dprintk("%s cipher_regs =%x \n", __func__, tca_cipher_dev.cipher_regs);
	dprintk("%s vpic_regs =%x \n", __func__, tca_cipher_dev.vpic_regs);

#ifndef CONFIG_OF
#ifdef CONFIG_CLOCK_TABLE
	cipher_clktbl = clocktable_get("cipher_clktbl");
	if (IS_ERR(cipher_clktbl)) {
		cipher_clktbl = NULL;
		return -EINVAL;
	}
	clocktable_ctrl(cipher_clktbl, 0, CLKTBL_ENABLE);
#endif

	/* Cipher block enable */
	cipher_clk = clk_get(NULL, "cipher");
	if(IS_ERR(cipher_clk)) {
		cipher_clk = NULL;
		printk("cipher clock error : cannot get clock\n");
		mutex_unlock(&tca_cipher_mutex);
		return -EINVAL;
	}
	clk_enable(cipher_clk);

#endif

	pSrcCpu = dma_alloc_coherent(0, MAX_CIPHER_BUFFER_LENGTH, &SrcDma, GFP_KERNEL);
	pDstCpu = dma_alloc_coherent(0, MAX_CIPHER_BUFFER_LENGTH, &DstDma, GFP_KERNEL);

	return 0;
}

int tca_cipher_release(struct inode *inode, struct file *file)
{
	dprintk("%s\n", __func__);

	dma_free_coherent(0, MAX_CIPHER_BUFFER_LENGTH, pSrcCpu, SrcDma);
	dma_free_coherent(0, MAX_CIPHER_BUFFER_LENGTH, pDstCpu, DstDma);

	/* Enable Cipher Interrupt */
	tca_cipher_interrupt_enable(FALSE);

#ifndef CONFIG_OF
	if (cipher_clk) {
		clk_disable(cipher_clk);
		clk_put(cipher_clk);
		cipher_clk = NULL;
	}
	
#ifdef CONFIG_CLOCK_TABLE
	if (cipher_clktbl) {
		clocktable_ctrl(cipher_clktbl, 0, CLKTBL_DISABLE);
		clocktable_put(cipher_clktbl);
		cipher_clktbl = NULL;
	}
#endif

#endif

	mutex_unlock(&tca_cipher_mutex);

	return 0;
}

