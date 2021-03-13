/*
 * linux/drivers/serial/tcc_cipher.c
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
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>
#include <linux/clk.h>
#include <linux/of.h>

#include <asm/io.h>
#include <asm/uaccess.h>
#ifdef CONFIG_PM
#include <linux/pm.h>
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#include "tcc_cipher.h"

static int cipher_data_debug = 0;
module_param(cipher_data_debug, int, 0644);
MODULE_PARM_DESC(cipher_data_debug, "Turn on/off device data debugging (default:off).");
static int cipher_debug = 0;
module_param(cipher_debug, int, 0644);
MODULE_PARM_DESC(cipher_debug, "Turn on/off device debugging (default:off).");
#define dprintk(msg...)	if(cipher_debug) { printk( "tcc_cipher: " msg); }

#define DEVICE_NAME		"cipher"

//#define MAJOR_ID		250
//#define MINOR_ID		0

#define DES_BLOCK_SIZE				8
#define AES_BLOCK_SIZE				16
#define MULTI2_BLOCK_SIZE			8
#define CSA_BLOCK_SIZE				1

//#define USE_IRQ
#ifdef USE_IRQ
#include <linux/interrupt.h>
#endif

#define USE_REV_MEMORY
#ifdef USE_REV_MEMORY
#include <soc/tcc/pmap.h>
#endif

static struct cipher_device
{
	struct device		*dev;

	struct clk			*cipher_clk;
	void __iomem		*cipher_regs;
	int					used;

#ifdef USE_IRQ
	void __iomem		*vpic_regs;
	u32					vpic_cipherbit;

	int					irq_cipher;
	int					iIrqData;
	int					irqDone;
#endif

	unsigned int		blockSize;

	dma_addr_t			srcPhy;
	u_char				*srcVir;
	dma_addr_t			dstPhy;
	u_char				*dstVir;
} *cipher_dev;

/*****************************************************************************
 *
 * Defines START
 *
 ******************************************************************************/
#define CIPHER_WRITE(VALUE, X)			__raw_writel(VALUE, X)
#define CIPHER_READ(X)					__raw_readl(X)
#define CIPHER_BITSET(X, MASK)			(CIPHER_WRITE(((CIPHER_READ(X)) | ((u32)(MASK))), X))
#define CIPHER_BITCLR(X, MASK)			(CIPHER_WRITE(((CIPHER_READ(X)) & ~((u32)(MASK))), X))
#define CIPHER_BITSCLR(X, SMASK, CMASK)	(CIPHER_WRITE((((CIPHER_READ(X)) | ((u32)(SMASK))) & ~((u32)(CMASK))) , X))
#define CIPHER_BITCSET(X, CMASK, SMASK)	(CIPHER_WRITE((((CIPHER_READ(X)) & ~((u32)(CMASK))) | ((u32)(SMASK))) , X))
/*
 * bit operations
 */
#define Hw32	(1LL << 32)
#define Hw31	0x80000000
#define Hw30	0x40000000
#define Hw29	0x20000000
#define Hw28	0x10000000
#define Hw27	0x08000000
#define Hw26	0x04000000
#define Hw25	0x02000000
#define Hw24	0x01000000
#define Hw23	0x00800000
#define Hw22	0x00400000
#define Hw21	0x00200000
#define Hw20	0x00100000
#define Hw19	0x00080000
#define Hw18	0x00040000
#define Hw17	0x00020000
#define Hw16	0x00010000
#define Hw15	0x00008000
#define Hw14	0x00004000
#define Hw13	0x00002000
#define Hw12	0x00001000
#define Hw11	0x00000800
#define Hw10	0x00000400
#define Hw9		0x00000200
#define Hw8		0x00000100
#define Hw7		0x00000080
#define Hw6		0x00000040
#define Hw5		0x00000020
#define Hw4		0x00000010
#define Hw3		0x00000008
#define Hw2		0x00000004
#define Hw1		0x00000002
#define Hw0		0x00000001
#define HwZERO	0x00000000

/* DMA Control Register */
#define HwCIPHER_DCTRL_CH(x)				((x)*Hw24)
#define HwCIPHER_DCTRL_CH_Mask				(Hw28+Hw27+Hw26+Hw25+Hw24)
#define HwCIPHER_DCTRL_CH0					HwCIPHER_DMACTR_CH(0)
#define HwCIPHER_DCTRL_CH1					HwCIPHER_DMACTR_CH(1)
#define HwCIPHER_DCTRL_CH2					HwCIPHER_DMACTR_CH(2)
#define HwCIPHER_DCTRL_CH3					HwCIPHER_DMACTR_CH(3)
#define HwCIPHER_DCTRL_CH4					HwCIPHER_DMACTR_CH(4)
#define HwCIPHER_DCTRL_CH5					HwCIPHER_DMACTR_CH(5)
#define HwCIPHER_DCTRL_CH6					HwCIPHER_DMACTR_CH(6)
#define HwCIPHER_DCTRL_CH7					HwCIPHER_DMACTR_CH(7)
#define HwCIPHER_DCTRL_CH8					HwCIPHER_DMACTR_CH(8)
#define HwCIPHER_DCTRL_CH9					HwCIPHER_DMACTR_CH(9)
#define HwCIPHER_DCTRL_CH10					HwCIPHER_DMACTR_CH(10)
#define HwCIPHER_DCTRL_CH11					HwCIPHER_DMACTR_CH(11)
#define HwCIPHER_DCTRL_CH12					HwCIPHER_DMACTR_CH(12)
#define HwCIPHER_DCTRL_CH13					HwCIPHER_DMACTR_CH(13)
#define HwCIPHER_DCTRL_CH14					HwCIPHER_DMACTR_CH(14)
#define HwCIPHER_DCTRL_CH15					HwCIPHER_DMACTR_CH(15)
#define HwCIPHER_DCTRL_IEN					Hw23
#define HwCIPHER_DCTRL_RWAIT(x)				((x)*Hw20)
#define HwCIPHER_DCTRL_RWAIT_Mask			(Hw22+Hw21+Hw20)
#define HwCIPHER_DCTRL_RPAUSE(x)			((x)*Hw17)
#define HwCIPHER_DCTRL_RPAUSE_Mask			(Hw19+Hw18+Hw17)
#define HwCIPHER_DCTRL_REN					Hw16
#define HwCIPHER_DCTRL_WB					Hw15
#define HwCIPHER_DCTRL_EN					Hw0

/*FIFO Status 0 Register*/
#define HwCIPHER_FIFO0_FULL					Hw31
#define HwCIPHER_FIFO0_EMP					Hw30
#define HwCIPHER_FIFO0_IDLE					Hw29
#define HwCIPHER_FIFO0_NEMPTY(x)			((x)*Hw0)
#define HwCIPHER_FIFO0_NEMPTY_Mask			(Hw8-Hw0)

/* Cipher Control Register */
#define HwCIPHER_CCTRL_IDLE					Hw15
#define HwCIPHER_CCTRL_KEY					Hw8
#define HwCIPHER_CCTRL_KEY_Mask				(Hw10+Hw9+Hw8)
#define HwCIPHER_CCTRL_ENDIAN(x)			((x)*Hw4)
#define HwCIPHER_CCTRL_ENDIAN_MASK			(Hw8-Hw4)
#define HwCIPHER_CCTRL_SELECT(x)			((x)*Hw0)
#define HwCIPHER_CCTRL_SELECT_Mask			(Hw4-Hw0)
#define HwCIPHER_CCTRL_SELECT_BYPASS		HwCIPHER_CCTRL_SELECT(0)
#define HwCIPHER_CCTRL_SELECT_AES			HwCIPHER_CCTRL_SELECT(1)
#define HwCIPHER_CCTRL_SELECT_DES			HwCIPHER_CCTRL_SELECT(2)
#define HwCIPHER_CCTRL_SELECT_MULTI2		HwCIPHER_CCTRL_SELECT(3)
#define HwCIPHER_CCTRL_SELECT_CSA2			HwCIPHER_CCTRL_SELECT(4)
#define HwCIPHER_CCTRL_SELECT_CSA3			HwCIPHER_CCTRL_SELECT(5)
#define HwCIPHER_CCTRL_SELECT_HASH			HwCIPHER_CCTRL_SELECT(6)

/*AES, DES, Multi2 Register (ADM) */
#define HwCIPHER_ADM_KEYLD					Hw31
#define HwCIPHER_ADM_IVLD					Hw30
#define HwCIPHER_ADM_ENC					Hw27
#define HwCIPHER_ADM_OPMODE(x)				((x)*Hw24)
#define HwCIPHER_ADM_OPMODE_Mask			(Hw26+Hw25+Hw24)
#define HwCIPHER_ADM_OPMODE_ECB				HwCIPHER_ADM_OPMODE(0)
#define HwCIPHER_ADM_OPMODE_CBC				HwCIPHER_ADM_OPMODE(1)
#define HwCIPHER_ADM_OPMODE_CFB				HwCIPHER_ADM_OPMODE(2)
#define HwCIPHER_ADM_OPMODE_OFB				HwCIPHER_ADM_OPMODE(3)
#define HwCIPHER_ADM_OPMODE_CTR				HwCIPHER_ADM_OPMODE(4)
#define HwCIPHER_ADM_PRT					Hw6
#define HwCIPHER_ADM_DESMODE(x)				((x)*Hw4)
#define HwCIPHER_ADM_DESMODE_Mask			(Hw5+Hw4)
#define HwCIPHER_ADM_DESMODE_SingleDES		HwCIPHER_ADM_DESMODE(0)
#define HwCIPHER_ADM_DESMODE_DoubleDES		HwCIPHER_ADM_DESMODE(1)
#define HwCIPHER_ADM_DESMODE_TripleDES2		HwCIPHER_ADM_DESMODE(2)
#define HwCIPHER_ADM_DESMODE_TripleDES3		HwCIPHER_ADM_DESMODE(3)
#define HwCIPHER_ADM_AESMODE(x)				((x)*Hw0)
#define HwCIPHER_ADM_AESMODE_Mask			(Hw1+Hw0)
#define HwCIPHER_ADM_AESMODE_128			HwCIPHER_ADM_AESMODE(0)
#define HwCIPHER_ADM_AESMODE_192			HwCIPHER_ADM_AESMODE(1)
#define HwCIPHER_ADM_AESMODE_256			HwCIPHER_ADM_AESMODE(2)

/*Hash Register*/
#define HwCIPHER_HASH_MODE(x)				((x)*Hw0)
#define HwCIPHER_HASH_MODE_Mask				(Hw2+Hw1+Hw0)
#define HwCIPHER_HASH_MODE_MD5				HwCIPHER_HASH_MODE(0)
#define HwCIPHER_HASH_MODE_SHA1				HwCIPHER_HASH_MODE(1)
#define HwCIPHER_HASH_MODE_SHA224			HwCIPHER_HASH_MODE(2)
#define HwCIPHER_HASH_MODE_SHA256			HwCIPHER_HASH_MODE(3)
#define HwCIPHER_HASH_MODE_SHA384			HwCIPHER_HASH_MODE(4)
#define HwCIPHER_HASH_MODE_SHA512			HwCIPHER_HASH_MODE(5)
#define HwCIPHER_HASH_MODE_SHA512_224		HwCIPHER_HASH_MODE(6)
#define HwCIPHER_HASH_MODE_SHA512_256		HwCIPHER_HASH_MODE(7)

#define HwCIPHER_VPICREG(x) (x)	//Vectored Priority Interrupt Controller

#define HwCIPHER_VPIC_IEN1	  		 		HwCIPHER_VPICREG(0x04)
#define HwCIPHER_VPIC_CLR1	   				HwCIPHER_VPICREG(0x0c)
#define HwCIPHER_VPIC_SEL1	   				HwCIPHER_VPICREG(0x1c)
#define HwCIPHER_VPIC_POL1	   				HwCIPHER_VPICREG(0x3c)
#define HwCIPHER_VPIC_MODE1	   				HwCIPHER_VPICREG(0x64)
#define HwCIPHER_VPIC_INTMSK1				HwCIPHER_VPICREG(0x104)

#define HwCIPHERREG(x) (x)

#define HwCIPHER_DCTRL			   			HwCIPHERREG(0x00)
#define HwCIPHER_SBASE					   	HwCIPHERREG(0x04)
#define HwCIPHER_DBASE					   	HwCIPHERREG(0x08)
#define HwCIPHER_SIZE					   	HwCIPHERREG(0x0C)
#define HwCIPHER_FIFO0					   	HwCIPHERREG(0x10)
#define HwCIPHER_FIFO1					   	HwCIPHERREG(0x14)
#define HwCIPHER_FIFO2					   	HwCIPHERREG(0x18)
#define HwCIPHER_FIFO3			   			HwCIPHERREG(0x1C)
#define HwCIPHER_INTMSK						HwCIPHERREG(0x20)
#define HwCIPHER_INTSTS			   			HwCIPHERREG(0x24)
#define HwCIPHER_CCTRL						HwCIPHERREG(0x100)
#define HwCIPHER_ADM						HwCIPHERREG(0x104)
#define HwCIPHER_HASH						HwCIPHERREG(0x110)
#define HwCIPHER_CSA3						HwCIPHERREG(0x114)
#define HwCIPHER_ROUND						HwCIPHERREG(0x120)
#define HwCIPHER_KEY0						HwCIPHERREG(0x180)
#define HwCIPHER_KEY1						HwCIPHERREG(0x184)
#define HwCIPHER_KEY2						HwCIPHERREG(0x188)
#define HwCIPHER_KEY3						HwCIPHERREG(0x18C)
#define HwCIPHER_KEY4						HwCIPHERREG(0x190)
#define HwCIPHER_KEY5						HwCIPHERREG(0x194)
#define HwCIPHER_KEY6						HwCIPHERREG(0x198)
#define HwCIPHER_KEY7						HwCIPHERREG(0x19C)
#define HwCIPHER_KEY8						HwCIPHERREG(0x1A0)
#define HwCIPHER_KEY9						HwCIPHERREG(0x1A4)
#define HwCIPHER_IV0						HwCIPHERREG(0x1C0)
#define HwCIPHER_IV1						HwCIPHERREG(0x1C4)
#define HwCIPHER_IV2						HwCIPHERREG(0x1C8)
#define HwCIPHER_IV3						HwCIPHERREG(0x1CC)

/*****************************************************************************
 *
 * Defines END
 *
 ******************************************************************************/
static int cipher_buffer_length = 4096;

static void tcc_cipher_reg_init(void)
{
	unsigned int data = 0;
	CIPHER_WRITE(HwCIPHER_DCTRL_RPAUSE(3) | HwCIPHER_DCTRL_REN | HwCIPHER_DCTRL_WB, cipher_dev->cipher_regs + HwCIPHER_DCTRL);

	CIPHER_WRITE(data, cipher_dev->cipher_regs + HwCIPHER_SBASE);
	CIPHER_WRITE(data, cipher_dev->cipher_regs + HwCIPHER_DBASE);
	CIPHER_WRITE(data, cipher_dev->cipher_regs + HwCIPHER_SIZE);
	CIPHER_WRITE(data, cipher_dev->cipher_regs + HwCIPHER_INTMSK);
	//CIPHER_WRITE(data, cipher_dev->cipher_regs + HwCIPHER_CCTRL);
	CIPHER_WRITE(data, cipher_dev->cipher_regs + HwCIPHER_ADM);
	CIPHER_WRITE(data, cipher_dev->cipher_regs + HwCIPHER_HASH);
	CIPHER_WRITE(data, cipher_dev->cipher_regs + HwCIPHER_CSA3);
	CIPHER_WRITE(data, cipher_dev->cipher_regs + HwCIPHER_ROUND);
	CIPHER_WRITE(data, cipher_dev->cipher_regs + HwCIPHER_KEY0);
	CIPHER_WRITE(data, cipher_dev->cipher_regs + HwCIPHER_KEY1);
	CIPHER_WRITE(data, cipher_dev->cipher_regs + HwCIPHER_KEY2);
	CIPHER_WRITE(data, cipher_dev->cipher_regs + HwCIPHER_KEY3);
	CIPHER_WRITE(data, cipher_dev->cipher_regs + HwCIPHER_KEY4);
	CIPHER_WRITE(data, cipher_dev->cipher_regs + HwCIPHER_KEY5);
	CIPHER_WRITE(data, cipher_dev->cipher_regs + HwCIPHER_KEY6);
	CIPHER_WRITE(data, cipher_dev->cipher_regs + HwCIPHER_KEY7);
	CIPHER_WRITE(data, cipher_dev->cipher_regs + HwCIPHER_KEY8);
	CIPHER_WRITE(data, cipher_dev->cipher_regs + HwCIPHER_KEY9);
	CIPHER_WRITE(data, cipher_dev->cipher_regs + HwCIPHER_IV0);
	CIPHER_WRITE(data, cipher_dev->cipher_regs + HwCIPHER_IV1);
	CIPHER_WRITE(data, cipher_dev->cipher_regs + HwCIPHER_IV2);
	CIPHER_WRITE(data, cipher_dev->cipher_regs + HwCIPHER_IV3);
}

#ifdef USE_IRQ
static void tcc_cipher_interrupt_enable(unsigned int uEnable)
{
	dprintk("%s, Enable=%d\n", __func__, uEnable);

	if (uEnable) {
		CIPHER_BITSET(cipher_dev->vpic_regs + HwCIPHER_VPIC_CLR1, cipher_dev->vpic_cipherbit);
		CIPHER_BITSET(cipher_dev->vpic_regs + HwCIPHER_VPIC_SEL1, cipher_dev->vpic_cipherbit);
		CIPHER_BITCLR(cipher_dev->vpic_regs + HwCIPHER_VPIC_POL1, cipher_dev->vpic_cipherbit);
		CIPHER_BITCLR(cipher_dev->vpic_regs + HwCIPHER_VPIC_MODE1, cipher_dev->vpic_cipherbit);
		CIPHER_BITSET(cipher_dev->vpic_regs + HwCIPHER_VPIC_IEN1, cipher_dev->vpic_cipherbit);
	}
	else {
		CIPHER_BITCLR(cipher_dev->vpic_regs + HwCIPHER_VPIC_INTMSK1, cipher_dev->vpic_cipherbit);
		CIPHER_BITCLR(cipher_dev->vpic_regs + HwCIPHER_VPIC_IEN1, cipher_dev->vpic_cipherbit);
	}
}

static irqreturn_t tcc_cipher_interrupt_handler(int irq, void *dev_id)
{
	/* Set Done Interrupt Flag */
	cipher_dev->irqDone = 1;
	dprintk("%s irq[%d]dev_id[0x%p]cipher_dev->irqDone = %d \n", __func__, irq, dev_id, cipher_dev->irqDone);
	CIPHER_WRITE(0, cipher_dev->cipher_regs + HwCIPHER_INTMSK);
	return IRQ_HANDLED;
}

static void tcc_cipher_wait_done(void)
{
	while (1) {
		dprintk("%s cipher_dev->irqDone = %d \n", __func__, cipher_dev->irqDone);
		if (cipher_dev->irqDone) {
			dprintk("Receive Done IRQ Handled\n");
			break;
		}
		udelay(1);
	}
}
#endif

void tcc_cipher_set_opmode(unsigned  int uOpMode)
{
	dprintk("%s, Operation Mode: %d\n", __func__, uOpMode);

	CIPHER_BITCSET(cipher_dev->cipher_regs + HwCIPHER_ADM, HwCIPHER_ADM_OPMODE_Mask, HwCIPHER_ADM_OPMODE(uOpMode));
}

void tcc_cipher_set_algorithm(unsigned  int uAlgorithm, unsigned  int uArg1, unsigned  int uArg2)
{
	u32 cctrl =	0;
	u32 adm =	0;
	u32 hash =	0;
	u32 csa3 =	0;
	u32 round =	0;

	dprintk("%s, Algorithm: %d, uArg1=%d, uArg2 = %d \n", __func__, uAlgorithm, uArg1, uArg2);

	switch(uAlgorithm)
	{
		case TCC_CIPHER_ALGORITM_BYPASS:
			{
				cctrl |=	HwCIPHER_CCTRL_SELECT_BYPASS;
			}
			break;

		case TCC_CIPHER_ALGORITM_AES:
			{
				/* uArg1: The Key Length in AES */
				/* uArg2: None                  */
				cctrl |=	HwCIPHER_CCTRL_SELECT_AES;
				adm |=		HwCIPHER_ADM_AESMODE(uArg1);
				cipher_dev->blockSize = AES_BLOCK_SIZE;
			}
			break;

		case TCC_CIPHER_ALGORITM_DES:
			{
				/* uArg1: The Mode in DES     */
				/* uArg2: Parity Bit Location */
				cctrl |=	HwCIPHER_CCTRL_SELECT_DES;
				adm |=		HwCIPHER_ADM_DESMODE(uArg1);
				if (uArg2) {
					adm |=	HwCIPHER_ADM_PRT;
				}
				cipher_dev->blockSize = DES_BLOCK_SIZE;
			}
			break;

		case TCC_CIPHER_ALGORITM_MULTI2:
			{
				/* uArg1: Round in Multi2 */
				/* uArg2: None */
				cctrl |=	HwCIPHER_CCTRL_SELECT_MULTI2;
				round |=	uArg1 * 8;
				cipher_dev->blockSize = MULTI2_BLOCK_SIZE;
			}
			break;

		case TCC_CIPHER_ALGORITM_CSA2:
			{
				/* uArg1: Round in CSA2 */
				/* uArg2: None */
				cctrl |=	HwCIPHER_CCTRL_SELECT_CSA2;
				round |=	uArg1;
				cipher_dev->blockSize = CSA_BLOCK_SIZE;
			}
			break;

		case TCC_CIPHER_ALGORITM_CSA3:
			{
				/* uArg1: Round in CSA2 */
				/* uArg2: None */
				cctrl |=	HwCIPHER_CCTRL_SELECT_CSA3;
				csa3 |=		1;
				round |=	uArg1;
				cipher_dev->blockSize = CSA_BLOCK_SIZE;
			}
			break;

		case TCC_CIPHER_ALGORITM_HASH:
			{
				/* uArg1: Mode in Hash */
				/* uArg2: Round in Hash */
				cctrl |=	HwCIPHER_CCTRL_SELECT_HASH;
				hash |=		HwCIPHER_HASH_MODE(uArg1);
				round |=	uArg2;
				cipher_dev->blockSize = CSA_BLOCK_SIZE;
			}
			break;

		default:
			dprintk("%s, Err: Invalid Algorithm\n", __func__);
			break;
	}
	CIPHER_BITCSET(cipher_dev->cipher_regs + HwCIPHER_CCTRL, HwCIPHER_CCTRL_SELECT_Mask, cctrl);
	CIPHER_BITCSET(cipher_dev->cipher_regs + HwCIPHER_ADM, HwCIPHER_ADM_PRT | HwCIPHER_ADM_DESMODE_Mask | HwCIPHER_ADM_AESMODE_Mask, adm);
	CIPHER_BITCSET(cipher_dev->cipher_regs + HwCIPHER_HASH, HwCIPHER_HASH_MODE_Mask, hash);
	CIPHER_BITCSET(cipher_dev->cipher_regs + HwCIPHER_CSA3, Hw32 - Hw0, csa3);
	CIPHER_BITCSET(cipher_dev->cipher_regs + HwCIPHER_ROUND, Hw32 - Hw0, round);
}

/*keymode : "0= use normal key", "1= use key from OTP"  */
static int tcc_cipher_set_keymode(unsigned  int keymode)
{
	if (keymode > 1) {
		printk("[%s] : KeyMode Set Error.. inupt_keymode = %d \n", __func__, keymode);
		return -1;
	}
	else {
		if (keymode) {
			CIPHER_BITSET(cipher_dev->cipher_regs + HwCIPHER_ADM, HwCIPHER_ADM_KEYLD);
		}
		else {
			CIPHER_BITCLR(cipher_dev->cipher_regs + HwCIPHER_ADM, HwCIPHER_ADM_KEYLD);
		}
	}
	return 0;
}

void tcc_cipher_set_key(unsigned char *pucData, unsigned int uLength, unsigned int uOption)
{
	unsigned int *pulKeyData;
	unsigned int ulAlgorthm, ulAESKeyLength, ulDESMode;

	dprintk("%s, Lenth: %d, Option: %d\n", __func__, uLength, uOption);

	pulKeyData = (unsigned int *)pucData;

	if (cipher_data_debug) {
		unsigned int i;
		unsigned int *pDataAddr;

		pDataAddr = (unsigned int *)pulKeyData;
		printk("\n[ Key data ]\n");
		for(i=0;i<(uLength/4);i+=4) {
			printk("0x%08x 0x%08x 0x%08x 0x%08x\n", pDataAddr[i+0], pDataAddr[i+1], pDataAddr[i+2], pDataAddr[i+3]);
		}
	}

	ulAlgorthm = CIPHER_READ(cipher_dev->cipher_regs + HwCIPHER_CCTRL) & HwCIPHER_CCTRL_SELECT_Mask;
	dprintk("%s, Algorthm: %x\n", __func__, ulAlgorthm);

	/* Write Key Data */
	switch(ulAlgorthm)
	{
		case HwCIPHER_CCTRL_SELECT_AES:
			{
				ulAESKeyLength  = CIPHER_READ(cipher_dev->cipher_regs + HwCIPHER_ADM) & HwCIPHER_ADM_AESMODE_Mask;
				dprintk("%s, ulAESKeyLength: %x\n", __func__, ulAESKeyLength);

				if (ulAESKeyLength == HwCIPHER_ADM_AESMODE_128) {
					CIPHER_WRITE(*pulKeyData++, cipher_dev->cipher_regs + HwCIPHER_KEY0);
					CIPHER_WRITE(*pulKeyData++, cipher_dev->cipher_regs + HwCIPHER_KEY1);
					CIPHER_WRITE(*pulKeyData++, cipher_dev->cipher_regs + HwCIPHER_KEY2);
					CIPHER_WRITE(*pulKeyData++, cipher_dev->cipher_regs + HwCIPHER_KEY3);
				}
				else if (ulAESKeyLength == HwCIPHER_ADM_AESMODE_192) {
					CIPHER_WRITE(*pulKeyData++, cipher_dev->cipher_regs + HwCIPHER_KEY0);
					CIPHER_WRITE(*pulKeyData++, cipher_dev->cipher_regs + HwCIPHER_KEY1);
					CIPHER_WRITE(*pulKeyData++, cipher_dev->cipher_regs + HwCIPHER_KEY2);
					CIPHER_WRITE(*pulKeyData++, cipher_dev->cipher_regs + HwCIPHER_KEY3);
					CIPHER_WRITE(*pulKeyData++, cipher_dev->cipher_regs + HwCIPHER_KEY4);
					CIPHER_WRITE(*pulKeyData++, cipher_dev->cipher_regs + HwCIPHER_KEY5);
				}
				else if (ulAESKeyLength == HwCIPHER_ADM_AESMODE_256) {
					CIPHER_WRITE(*pulKeyData++, cipher_dev->cipher_regs + HwCIPHER_KEY0);
					CIPHER_WRITE(*pulKeyData++, cipher_dev->cipher_regs + HwCIPHER_KEY1);
					CIPHER_WRITE(*pulKeyData++, cipher_dev->cipher_regs + HwCIPHER_KEY2);
					CIPHER_WRITE(*pulKeyData++, cipher_dev->cipher_regs + HwCIPHER_KEY3);
					CIPHER_WRITE(*pulKeyData++, cipher_dev->cipher_regs + HwCIPHER_KEY4);
					CIPHER_WRITE(*pulKeyData++, cipher_dev->cipher_regs + HwCIPHER_KEY5);
					CIPHER_WRITE(*pulKeyData++, cipher_dev->cipher_regs + HwCIPHER_KEY6);
					CIPHER_WRITE(*pulKeyData++, cipher_dev->cipher_regs + HwCIPHER_KEY7);
				}
			}
			break;

		case HwCIPHER_CCTRL_SELECT_DES:
			{
				ulDESMode  = CIPHER_READ(cipher_dev->cipher_regs + HwCIPHER_ADM) & HwCIPHER_ADM_DESMODE_Mask;
				dprintk("%s, ulDESMODE: %x\n", __func__, ulDESMode);

				if (ulDESMode == HwCIPHER_ADM_DESMODE_SingleDES) {
					CIPHER_WRITE(*pulKeyData++, cipher_dev->cipher_regs + HwCIPHER_KEY0);
					CIPHER_WRITE(*pulKeyData++, cipher_dev->cipher_regs + HwCIPHER_KEY1);
				}
				else if (ulDESMode == HwCIPHER_ADM_DESMODE_DoubleDES) {
					CIPHER_WRITE(*pulKeyData++, cipher_dev->cipher_regs + HwCIPHER_KEY0);
					CIPHER_WRITE(*pulKeyData++, cipher_dev->cipher_regs + HwCIPHER_KEY1);
					CIPHER_WRITE(*pulKeyData++, cipher_dev->cipher_regs + HwCIPHER_KEY2);
					CIPHER_WRITE(*pulKeyData++, cipher_dev->cipher_regs + HwCIPHER_KEY3);
				}
				else if (ulDESMode == HwCIPHER_ADM_DESMODE_TripleDES2) {
					CIPHER_WRITE(*pulKeyData++, cipher_dev->cipher_regs + HwCIPHER_KEY0);
					CIPHER_WRITE(*pulKeyData++, cipher_dev->cipher_regs + HwCIPHER_KEY1);
					CIPHER_WRITE(*pulKeyData++, cipher_dev->cipher_regs + HwCIPHER_KEY2);
					CIPHER_WRITE(*pulKeyData++, cipher_dev->cipher_regs + HwCIPHER_KEY3);
				}
				else if (ulDESMode == HwCIPHER_ADM_DESMODE_TripleDES3) {
					CIPHER_WRITE(*pulKeyData++, cipher_dev->cipher_regs + HwCIPHER_KEY0);
					CIPHER_WRITE(*pulKeyData++, cipher_dev->cipher_regs + HwCIPHER_KEY1);
					CIPHER_WRITE(*pulKeyData++, cipher_dev->cipher_regs + HwCIPHER_KEY2);
					CIPHER_WRITE(*pulKeyData++, cipher_dev->cipher_regs + HwCIPHER_KEY3);
					CIPHER_WRITE(*pulKeyData++, cipher_dev->cipher_regs + HwCIPHER_KEY4);
					CIPHER_WRITE(*pulKeyData++, cipher_dev->cipher_regs + HwCIPHER_KEY5);
				}
			}
			break;

		case HwCIPHER_CCTRL_SELECT_MULTI2:
			{
				if (uOption == TCC_CIPHER_KEY_MULTI2_DATA) {
					CIPHER_WRITE(*pulKeyData++, cipher_dev->cipher_regs + HwCIPHER_KEY0);
					CIPHER_WRITE(*pulKeyData++, cipher_dev->cipher_regs + HwCIPHER_KEY1);
				}
				else {
					CIPHER_WRITE(*pulKeyData++, cipher_dev->cipher_regs + HwCIPHER_KEY2);
					CIPHER_WRITE(*pulKeyData++, cipher_dev->cipher_regs + HwCIPHER_KEY3);
					CIPHER_WRITE(*pulKeyData++, cipher_dev->cipher_regs + HwCIPHER_KEY4);
					CIPHER_WRITE(*pulKeyData++, cipher_dev->cipher_regs + HwCIPHER_KEY5);
					CIPHER_WRITE(*pulKeyData++, cipher_dev->cipher_regs + HwCIPHER_KEY6);
					CIPHER_WRITE(*pulKeyData++, cipher_dev->cipher_regs + HwCIPHER_KEY7);
					CIPHER_WRITE(*pulKeyData++, cipher_dev->cipher_regs + HwCIPHER_KEY8);
					CIPHER_WRITE(*pulKeyData++, cipher_dev->cipher_regs + HwCIPHER_KEY9);
				}
			}
			break;

		case HwCIPHER_CCTRL_SELECT_CSA2:
			{
				CIPHER_WRITE(*pulKeyData++, cipher_dev->cipher_regs + HwCIPHER_KEY0);
				CIPHER_WRITE(*pulKeyData++, cipher_dev->cipher_regs + HwCIPHER_KEY1);
			}
			break;

		case HwCIPHER_CCTRL_SELECT_CSA3:
			{
				CIPHER_WRITE(*pulKeyData++, cipher_dev->cipher_regs + HwCIPHER_KEY0);
				CIPHER_WRITE(*pulKeyData++, cipher_dev->cipher_regs + HwCIPHER_KEY1);
				CIPHER_WRITE(*pulKeyData++, cipher_dev->cipher_regs + HwCIPHER_KEY2);
				CIPHER_WRITE(*pulKeyData++, cipher_dev->cipher_regs + HwCIPHER_KEY3);
			}
			break;


		default:
			break;
	}

	/* Load Key Data */
	//tcc_cipher_set_keymode(TCC_CIPHER_KEY_NORMAL);
	//CIPHER_BITSET(cipher_dev->cipher_regs + HwCIPHER_ADM, HwCIPHER_ADM_KEYLD);
}

void tcc_cipher_set_vector(unsigned char *pucData, unsigned  int uLength)
{
	unsigned int *pulVectorData;
	unsigned int ulAlgorthm;

	dprintk("%s, Length: %d\n", __func__, uLength);

	pulVectorData = (unsigned int *)pucData;
	if(uLength == 0)
		return;

	if (cipher_data_debug) {
		unsigned int i;
		unsigned int *pDataAddr;

		pDataAddr = (unsigned int *)pulVectorData;
		printk("\n[ IV data ]\n");
		for(i=0;i<(uLength/4);i+=4) {
			printk("0x%08x 0x%08x 0x%08x 0x%08x\n", pDataAddr[i+0], pDataAddr[i+1], pDataAddr[i+2], pDataAddr[i+3]);
		}
	}

	ulAlgorthm  = CIPHER_READ(cipher_dev->cipher_regs + HwCIPHER_CCTRL) & HwCIPHER_CCTRL_SELECT_Mask;
	dprintk("%s, Algorthm: %x\n", __func__, ulAlgorthm);

	/* Write Initial Vector */
	switch(ulAlgorthm)
	{
		case HwCIPHER_CCTRL_SELECT_AES:
			{
				CIPHER_WRITE(*pulVectorData++, cipher_dev->cipher_regs + HwCIPHER_IV0);
				CIPHER_WRITE(*pulVectorData++, cipher_dev->cipher_regs + HwCIPHER_IV1);
				CIPHER_WRITE(*pulVectorData++, cipher_dev->cipher_regs + HwCIPHER_IV2);
				CIPHER_WRITE(*pulVectorData++, cipher_dev->cipher_regs + HwCIPHER_IV3);
			}
			break;

		case HwCIPHER_CCTRL_SELECT_DES:
			{
				CIPHER_WRITE(*pulVectorData++, cipher_dev->cipher_regs + HwCIPHER_IV0);
				CIPHER_WRITE(*pulVectorData++, cipher_dev->cipher_regs + HwCIPHER_IV1);
			}
			break;

		case HwCIPHER_CCTRL_SELECT_MULTI2:
			{
				CIPHER_WRITE(*pulVectorData++, cipher_dev->cipher_regs + HwCIPHER_IV0);
				CIPHER_WRITE(*pulVectorData++, cipher_dev->cipher_regs + HwCIPHER_IV1);
			}
			break;

		default:
			break;
	}

	/* Load Initial Vector */
	//CIPHER_BITSET(cipher_dev->cipher_regs + HwCIPHER_ADM, HwCIPHER_ADM_IVLD);
}

void tcc_cipher_set(unsigned char keyLoad, unsigned char ivLoad, unsigned char enc)
{
	if (keyLoad == 1)
		CIPHER_BITSET(cipher_dev->cipher_regs + HwCIPHER_ADM, HwCIPHER_ADM_KEYLD);

	if(ivLoad == 1)
		CIPHER_BITSET(cipher_dev->cipher_regs + HwCIPHER_ADM, HwCIPHER_ADM_IVLD);

	/* Select Encryption */
	if (enc == 1)
		CIPHER_BITSET(cipher_dev->cipher_regs + HwCIPHER_ADM, HwCIPHER_ADM_ENC);
	else
		CIPHER_BITCLR(cipher_dev->cipher_regs + HwCIPHER_ADM, HwCIPHER_ADM_ENC);

	/* CIPHER_BITCSET(cipher_dev->cipher_regs + HwCIPHER_CCTRL, HwCIPHER_CCTRL_ENDIAN_MASK, HwCIPHER_CCTRL_ENDIAN(0)); */
}

int tcc_cipher_run(unsigned char *pucSrcAddr, unsigned char *pucDstAddr, unsigned int uLength, int fromuser, int iPKCS7Pad, int *piPKCS7PadLength)
{
	uint	uiresidual = 0;
	uint	uiEncode = 0;
	int 	i;

	if(iPKCS7Pad) {
		uiEncode = CIPHER_READ(cipher_dev->cipher_regs + HwCIPHER_ADM) & HwCIPHER_ADM_ENC;
		*piPKCS7PadLength = 0;
	}

	dprintk("%s, Length=%d\n", __func__, uLength);
	if(uLength < cipher_dev->blockSize)
		return -1;

	uiresidual = uLength & (cipher_dev->blockSize-1);
	if (uiresidual) {
		uLength -= uiresidual;
		dprintk("%s Length=%d, uiresidual = %d\n", __func__, uLength, uiresidual);
	}

#ifdef USE_REV_MEMORY
	if (uLength > cipher_buffer_length) {
		dprintk("uLength = %d, cipher_buffer_length = %d\n", uLength, cipher_buffer_length);
		return -1;
	}
#else //USE_REV_MEMORY
	if (uLength > cipher_buffer_length) {
		if(cipher_dev->srcVir != NULL)
			dma_free_coherent(cipher_dev->dev, cipher_buffer_length, cipher_dev->srcVir, cipher_dev->srcPhy);
		if(cipher_dev->dstVir != NULL)
			dma_free_coherent(cipher_dev->dev, cipher_buffer_length, cipher_dev->dstVir, cipher_dev->dstPhy);

		/* Allocate Physical & Virtual Address for DMA */
		cipher_dev->srcVir = dma_alloc_coherent(cipher_dev->dev, uLength, &cipher_dev->srcPhy, GFP_KERNEL);
		cipher_dev->dstVir = dma_alloc_coherent(cipher_dev->dev, uLength, &cipher_dev->dstPhy, GFP_KERNEL);
	}
#endif //USE_REV_MEMORY

	if ((cipher_dev->srcVir == NULL) || (cipher_dev->dstVir == NULL)) {
		dprintk("cipher_dev->srcVir = %p, cipher_dev->dstVir = %p\n", cipher_dev->srcVir, cipher_dev->dstVir);
		return -1;
	}

	/* Init Virtual Address */
	memset(cipher_dev->srcVir, 0x00, uLength);
	memset(cipher_dev->dstVir, 0x00, uLength);

	/* Copy Plain Text from Source Buffer */
	if (fromuser) {
		if (copy_from_user(cipher_dev->srcVir, pucSrcAddr, uLength)) {
			return -EFAULT;
		}
	}
	else {
		memcpy(cipher_dev->srcVir, pucSrcAddr, uLength);
	}

	if((1 == iPKCS7Pad) && (0 != uiEncode)) {
		*piPKCS7PadLength = 16 - (uLength % 16);
		for(i=1; i <= *piPKCS7PadLength; i++)
		{
			cipher_dev->dstVir[uLength+i] = *piPKCS7PadLength;
		}
	}

	udelay(1);

	CIPHER_WRITE(cipher_dev->srcPhy, cipher_dev->cipher_regs + HwCIPHER_SBASE);
	CIPHER_WRITE(cipher_dev->dstPhy, cipher_dev->cipher_regs + HwCIPHER_DBASE);
	CIPHER_WRITE(uLength + *piPKCS7PadLength, cipher_dev->cipher_regs + HwCIPHER_SIZE);
	//CIPHER_WRITE(uLength + *piPKCS7PadLength, cipher_dev->cipher_regs + HwCIPHER_DCTRL);
#ifdef USE_IRQ
	CIPHER_WRITE(Hw32-Hw0, cipher_dev->cipher_regs + HwCIPHER_INTMSK);
	cipher_dev->irqDone = 0;
	CIPHER_BITSET(cipher_dev->cipher_regs + HwCIPHER_DCTRL, HwCIPHER_DCTRL_IEN);
#endif
	CIPHER_BITSET(cipher_dev->cipher_regs + HwCIPHER_DCTRL, HwCIPHER_DCTRL_EN);
#ifdef USE_IRQ
	tcc_cipher_wait_done();
	udelay(1);
#else
	while(!(CIPHER_READ(cipher_dev->cipher_regs + HwCIPHER_FIFO0)&HwCIPHER_FIFO0_IDLE)) {
		dprintk("%s, %d\n", __func__, __LINE__);
		udelay(1);
	}
#endif

#ifndef USE_REV_MEMORY
	dma_sync_single_for_cpu(cipher_dev->dev, cipher_dev->dstPhy, uLength, DMA_FROM_DEVICE);
#endif //USE_REV_MEMORY

	/* Remove PKCS7 PAD */
	if((1 == iPKCS7Pad) && ( 0 == uiEncode)) {
		*piPKCS7PadLength = cipher_dev->dstVir[uLength-1];
		if((*piPKCS7PadLength <= 16) &&(*piPKCS7PadLength < uLength)) {
			for(i=1; i <= *piPKCS7PadLength; i++)
			{
				cipher_dev->dstVir[uLength-i] = 0;
			}
			*piPKCS7PadLength = -(*piPKCS7PadLength);
		}
		else {
			*piPKCS7PadLength = 0;
		}
	}

	/* Copy Cipher Text to Destination Buffer */
	if (fromuser) {
		if (copy_to_user(pucDstAddr, cipher_dev->dstVir, uLength + *piPKCS7PadLength)) {
			return -EFAULT;
		}
	}
	else {
		memcpy(pucDstAddr, cipher_dev->dstVir, uLength + *piPKCS7PadLength);
	}

	if (cipher_data_debug) {
		unsigned int i;
		unsigned int *pDataAddr;
		unsigned data1, data2, data3, data4;

		printk("\n[ Register Setting ]\n");
		for(i=0;i<=0x1e0;i+=16) {
			data1 = CIPHER_READ(cipher_dev->cipher_regs + i);
			data2 = CIPHER_READ(cipher_dev->cipher_regs + i + 4);
			data3 = CIPHER_READ(cipher_dev->cipher_regs + i + 8);
			data4 = CIPHER_READ(cipher_dev->cipher_regs + i + 12);
			printk("[0x%3x]:0x%08x 0x%08x 0x%08x 0x%08x\n", i, data1, data2, data3, data4);
		}

		pDataAddr = (unsigned int *)cipher_dev->srcVir;
		printk("\n[ Source Text ]\n");
		for(i=0;i<(uLength/4);i+=4) {
			printk("0x%08x 0x%08x 0x%08x 0x%08x\n", pDataAddr[i+0], pDataAddr[i+1], pDataAddr[i+2], pDataAddr[i+3]);
		}

		pDataAddr = (unsigned int *)cipher_dev->dstVir;
		printk("\n[ Dest Text ]\n");
		for(i=0;i<((uLength*2)/4);i+=4) {
			printk("0x%08x 0x%08x 0x%08x 0x%08x\n", pDataAddr[i+0], pDataAddr[i+1], pDataAddr[i+2], pDataAddr[i+3]);
		}
		printk("\n");
	}

	//tcc_cipher_reg_init();

#ifndef USE_REV_MEMORY
	if (uLength > cipher_buffer_length) {
		dma_free_coherent(cipher_dev->dev, uLength, cipher_dev->srcVir, cipher_dev->srcPhy);
		dma_free_coherent(cipher_dev->dev, uLength, cipher_dev->dstVir, cipher_dev->dstPhy);

		cipher_dev->srcVir = dma_alloc_coherent(cipher_dev->dev, cipher_buffer_length, &cipher_dev->srcPhy, GFP_KERNEL);
		cipher_dev->dstVir = dma_alloc_coherent(cipher_dev->dev, cipher_buffer_length, &cipher_dev->dstPhy, GFP_KERNEL);
	}
#endif //USE_REV_MEMORY

	return 0;
}

static long tcc_cipher_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long err = 0;
	dprintk("%s, cmd=%d\n", __func__, cmd);

	switch(cmd)
	{
		case TCC_CIPHER_IOCTL_SET_ALGORITHM:
			{
				stCIPHER_ALGORITHM stAlgorithm;
				if (copy_from_user((void *)&stAlgorithm, (const void *)arg, sizeof(stCIPHER_ALGORITHM))) {
					err = -EFAULT;
					break;
				}

				tcc_cipher_set_opmode(stAlgorithm.uOperationMode);
				tcc_cipher_set_algorithm(stAlgorithm.uAlgorithm, stAlgorithm.uArgument1, stAlgorithm.uArgument2);
			}
			break;

		case TCC_CIPHER_IOCTL_SET_KEY:
			{
				stCIPHER_KEY stKeyInfo;
				if (copy_from_user((void *)&stKeyInfo, (const void *)arg, sizeof(stCIPHER_KEY))) {
					err = -EFAULT;
					break;
				}
				tcc_cipher_set_key(stKeyInfo.pucData, stKeyInfo.uLength, stKeyInfo.uOption);
			}
			break;

		case TCC_CIPHER_IOCTL_SET_VECTOR:
			{
				stCIPHER_VECTOR stVectorInfo;
				if (copy_from_user((void *)&stVectorInfo, (const void *)arg, sizeof(stCIPHER_VECTOR))) {
					err = -EFAULT;
					break;
				}
				tcc_cipher_set_vector(stVectorInfo.pucData, stVectorInfo.uLength);
			}
			break;

		case TCC_CIPHER_IOCTL_SET:
			{
				stCIPHER_SET stSetInfo;
				if (copy_from_user((void *)&stSetInfo, (const void *)arg, sizeof(stCIPHER_SET))) {
					err = -EFAULT;
					break;
				}
				tcc_cipher_set(stSetInfo.keyLoad, stSetInfo.ivLoad, stSetInfo.enc);
			}
			break;

		case TCC_CIPHER_IOCTL_RUN:
			{
				stCIPHER_RUN stRunInfo;
				int iPKCS7PadLength = 0;

				if (copy_from_user((void *)&stRunInfo, (const void *)arg, sizeof(stCIPHER_RUN))) {
					err = -EFAULT;
					break;
				}
				err = tcc_cipher_run(stRunInfo.pucSrcAddr, stRunInfo.pucDstAddr, stRunInfo.uLength, 1, stRunInfo.uPKCS7Pad, &iPKCS7PadLength);

				if(stRunInfo.uPKCS7Pad) {
					stRunInfo.uLength += iPKCS7PadLength;
				}

				if (copy_to_user( (const void *)arg, (void *)&stRunInfo, sizeof(stCIPHER_RUN))) {
					err = -EFAULT;
					break;
				}
			}
			break;

		default:
			printk("err: unkown command(%d)\n", cmd);
			err = -ENOTTY;
			break;
	}

	return err;
}

int tcc_cipher_open(struct inode *inode, struct file *filp)
{
	if (cipher_dev->used) {
		printk("%s device already used", __func__);
		return -EBUSY;
	}
	cipher_dev->used = 1;
#ifdef USE_IRQ
	tcc_cipher_interrupt_enable(1);
#endif
	tcc_cipher_reg_init();

	dprintk("%s\n", __func__);

	return 0;
}

int tcc_cipher_release(struct inode *inode, struct file *file)
{
	dprintk("%s\n", __func__);

	cipher_dev->used = 0;

	return 0;
}

static const struct file_operations tcc_cipher_fops =
{
	.owner		= THIS_MODULE,
	.open		= tcc_cipher_open,
	.unlocked_ioctl	= tcc_cipher_ioctl,
	.release	= tcc_cipher_release,
};

static struct miscdevice cipher_misc_device =
{
	.minor = MISC_DYNAMIC_MINOR,
	.name = DEVICE_NAME,
	.fops = &tcc_cipher_fops,
};

static int tcc_cipher_probe(struct platform_device *pdev)
{
#ifdef USE_REV_MEMORY
	pmap_t pmap_secure_hash;
#endif
#ifdef USE_IRQ
	int ret;
#endif
	struct resource *regs;

	if (!pdev->dev.of_node) {
		dev_err(&pdev->dev, "no platform data\n");
		return -EINVAL;
	}

	cipher_dev = devm_kzalloc(&pdev->dev, sizeof(struct cipher_device), GFP_KERNEL);
	if (cipher_dev == NULL) {
		printk("%s failed to allocate cipher_device\n", __func__);
		return -ENOMEM;
	}

	cipher_dev->dev = &pdev->dev;
	cipher_dev->cipher_clk = of_clk_get(pdev->dev.of_node, 0);
	if (IS_ERR(cipher_dev->cipher_clk)) {
		printk("%s failed to get cipher clock\n", __func__);
		cipher_dev->cipher_clk = NULL;
		return -ENODEV;
	}
	clk_prepare_enable(cipher_dev->cipher_clk);

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!regs) {
		printk("%s Couldn't get resource0\n", __func__);
		return -EBUSY;
	}
	cipher_dev->cipher_regs = devm_ioremap_resource(&pdev->dev, regs);
	if (IS_ERR(cipher_dev->cipher_regs)) {
		printk("%s cipher_regs error", __func__);
		return PTR_ERR(cipher_dev->cipher_regs);
	}
	printk("%s cipher_clk[%p] cipher_regs[%p]\n", __func__, cipher_dev->cipher_clk, cipher_dev->cipher_regs);

#ifdef USE_IRQ
	regs = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!regs) {
		printk("%s Couldn't get resource1\n", __func__);
		return -EBUSY;
	}
	cipher_dev->vpic_regs = devm_ioremap_resource(&pdev->dev, regs);
	if (IS_ERR(cipher_dev->vpic_regs)) {
		printk("%s vpic_regs error", __func__);
		return PTR_ERR(cipher_dev->vpic_regs);
	}

	if (of_machine_is_compatible("telechips,tcc898x")) {
		cipher_dev->vpic_cipherbit = Hw13; //tcc898x
	}
	else {
		cipher_dev->vpic_cipherbit = Hw20; //tcc897x
	}

	cipher_dev->irqDone = 0;
	cipher_dev->irq_cipher = platform_get_irq(pdev, 0);
	ret = request_irq(cipher_dev->irq_cipher, tcc_cipher_interrupt_handler, IRQF_SHARED, "tcc_cipher", &cipher_dev->iIrqData);
	if (ret) {
		printk("%s cannot get irq ret[%d]\n", __func__, ret);
		free_irq(cipher_dev->irq_cipher, &cipher_dev->iIrqData);
		return -1;
	}
	printk("%s vpic_regs[%p] irq[%d]\n", __func__, cipher_dev->vpic_regs, cipher_dev->irq_cipher);
#endif

#ifdef USE_REV_MEMORY
	if (pmap_get_info("secure_hash", &pmap_secure_hash)) {
		printk("pmap_secure_hash.base = %x, pmap_secure_hash.size = %d\n", pmap_secure_hash.base, pmap_secure_hash.size);

		cipher_buffer_length = pmap_secure_hash.size;
		cipher_dev->srcPhy = pmap_secure_hash.base;
		cipher_dev->srcVir = ioremap_nocache(cipher_dev->srcPhy, cipher_buffer_length);
		if (!cipher_dev->srcVir) {
			printk("cipher_dev->srcPhy = %x, cipher_dev->srcVir = %p\n", cipher_dev->srcPhy, cipher_dev->srcVir);
			return -1;
		}

		cipher_dev->dstPhy = cipher_dev->srcPhy;
		cipher_dev->dstVir = cipher_dev->srcVir;
	}
	else {
		printk("pmap_get_info failed\n");
		return -1;
	}
#else //USE_REV_MEMORY
	cipher_dev->srcVir = dma_alloc_coherent(0, cipher_buffer_length, &cipher_dev->srcPhy, GFP_KERNEL);
	cipher_dev->dstVir = dma_alloc_coherent(0, cipher_buffer_length, &cipher_dev->dstPhy, GFP_KERNEL);
	if ((cipher_dev->srcVir == NULL) || (cipher_dev->dstVir == NULL)) {
		dprintk("%s cipher_dev->srcVir = %p, cipher_dev->dstVir = %p\n", __func__, cipher_dev->srcVir, cipher_dev->dstVir);
		return -1;
	}
#endif //USE_REV_MEMORY

	if (misc_register(&cipher_misc_device))
	{
		printk("%s Couldn't register device\n", __func__);
		return -EBUSY;
	}

	cipher_dev->used = 0;

	return 0;
}

static int tcc_cipher_remove(struct platform_device *pdev)
{
	misc_deregister(&cipher_misc_device);

#ifdef USE_REV_MEMORY
	iounmap((void*)cipher_dev->srcPhy);
#else //USE_REV_MEMORY
	dma_free_coherent(cipher_dev->dev, cipher_buffer_length, cipher_dev->srcVir, cipher_dev->srcPhy);
	dma_free_coherent(cipher_dev->dev, cipher_buffer_length, cipher_dev->dstVir, cipher_dev->dstPhy);
#endif //USE_REV_MEMORY

#ifdef USE_IRQ
	free_irq(cipher_dev->irq_cipher, &cipher_dev->iIrqData);
#endif
	if (cipher_dev->cipher_clk) {
		clk_disable_unprepare(cipher_dev->cipher_clk);
		cipher_dev->cipher_clk = NULL;
	}

	devm_kfree(cipher_dev->dev, cipher_dev);

	return 0;
}

#ifdef CONFIG_PM
static int tcc_cipher_suspend(struct platform_device *pdev, pm_message_t state)
{
	dprintk("%s\n", __func__);
	return 0;
}

static int tcc_cipher_resume(struct platform_device *pdev)
{
	dprintk("%s\n", __func__);
	return 0;
}
#else
#define tcc_cipher_suspend NULL
#define tcc_cipher_resume NULL
#endif

#ifdef CONFIG_OF
static struct of_device_id cipher_of_match[] = {
	{ .compatible = "telechips,tcc-cipher" },
	{ "", "", "", NULL },
};
MODULE_DEVICE_TABLE(of, cipher_of_match);
#endif

static struct platform_driver cipher_driver =
{
	.driver	=
	{
		.name	= "cipher",
		.owner 	= THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(cipher_of_match)
#endif
	},
	.probe		= tcc_cipher_probe,
	.remove		= tcc_cipher_remove,
#ifdef CONFIG_PM
	.suspend		= tcc_cipher_suspend,
	.resume		= tcc_cipher_resume,
#endif
};

static int __init tcc_cipher_init(void)
{
	int	result =0;
	dprintk("Telechips Cipher Driver Init\n");
	result = platform_driver_register(&cipher_driver);
	if (result) {
		dprintk("%s platform_driver_register err \n", __func__);
		return 0;
	}

	return result;
}

static void __exit tcc_cipher_exit(void)
{
	dprintk("Telechips Cipher Driver Exit \n");
	platform_driver_unregister(&cipher_driver);
}

module_init(tcc_cipher_init);
module_exit(tcc_cipher_exit);

MODULE_AUTHOR("linux <linux@telechips.com>");
MODULE_DESCRIPTION("Telechips TCC CIPHER driver");
MODULE_LICENSE("GPL");

