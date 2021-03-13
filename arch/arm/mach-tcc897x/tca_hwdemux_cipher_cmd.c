/*
 * linux/arch/arm/mach-tcc893x/tca_hwdemux_cipher_cmd.c
 *
 * Author:  <linux@telechips.com>
 * Created: March 18, 2010
 * Description: Telechips hwdemux cipher driver
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
#include <mach/tca_hwdemux_cipher_cmd.h>
#include <mach/TCC893xHWDemux_cmd.h>

#if 0
static int debug = 0;
#define dprintk(msg...)	if(debug) { printk( "tca_hwdemux_cipher_cmd: " msg); }
#endif
	
int HWDEMUXCIPHERCMD_Set_Algorithm(stHWDEMUXCIPHER_ALGORITHM *pARG)
{
#if 0
		printk("%s, %d  \n", __func__, __LINE__); 
		printk("%s, %d  uAlgorithm = %d, uArgument1 = %d, uArgument2 = %d, uOperationMode = %d \n", __func__, __LINE__, 
			pARG->uAlgorithm, pARG->uArgument1, pARG->uArgument2, pARG->uOperationMode); 
#endif

    volatile PMAILBOX pMailBox = (volatile PMAILBOX)tcc_p2v(HwCORTEXM4_MAILBOX0_BASE);
    BITCLR( pMailBox->uMBOX_CTL_016.nREG, Hw5); //OEN low
    pMailBox->uMBOX_TX0.nREG = (HW_CM_CIPHER_SET_ALGORITHM << 24 |	(pARG->uDemuxid &0xff <<16) |
							(pARG->uOperationMode & 0xff) << 8 | (pARG->uAlgorithm & 0xff) );
    pMailBox->uMBOX_TX1.nREG =( (pARG->uArgument1 & 0xff) <<24 | (pARG->uArgument1 & 0xff)<<16  |
							(pARG->uArgument1 & 0xff)<<8 |(pARG->uArgument2 & 0xff)) ;
    pMailBox->uMBOX_TX2.nREG = 0;
    pMailBox->uMBOX_TX3.nREG = 0;
    pMailBox->uMBOX_TX4.nREG = 0;
    pMailBox->uMBOX_TX5.nREG = 0;
    pMailBox->uMBOX_TX6.nREG = 0;
    pMailBox->uMBOX_TX7.nREG = 0;
    BITSET( pMailBox->uMBOX_CTL_016.nREG, Hw5); //OEN high

    return 0;
}

int HWDEMUXCIPHERCMD_Set_Key(stHWDEMUXCIPHER_KEY *pARG, unsigned int uiTotalIndex, unsigned int uiCurrentIndex)
{
#if 0
	printk("%s, %d  \n", __func__, __LINE__); 
	printk("%s, %d  uiTotalIndex = %d, uiCurrentIndex = %d, key_uLength = %d, uOption = %d\n", __func__, __LINE__, uiTotalIndex, uiCurrentIndex, pARG->uLength , pARG->uOption); 
	{
		printk("%s, %d  [0x%x][0x%x][0x%x][0x%x][0x%x][0x%x][0x%x][0x%x]\n", __func__, __LINE__,
			*pARG->pucData, *(pARG->pucData+1), *(pARG->pucData+2), *(pARG->pucData+3), *(pARG->pucData+4), 
			*(pARG->pucData+5), *(pARG->pucData+6), *(pARG->pucData+7)); 
	}
#endif

	unsigned long *pulKeyData = (unsigned long *)pARG->pucData;
	volatile PMAILBOX pMailBox = (volatile PMAILBOX)tcc_p2v(HwCORTEXM4_MAILBOX0_BASE);
	BITCLR( pMailBox->uMBOX_CTL_016.nREG, Hw5); //OEN low
	pMailBox->uMBOX_TX0.nREG = (HW_CM_CIPHER_SET_KEY << 24 |(pARG->uDemuxid &0xff <<16) |
					(pARG->uLength& 0xff) << 8 |(pARG->uOption& 0xff) );
	pMailBox->uMBOX_TX1.nREG = (uiTotalIndex&0xf)<<28 | (uiCurrentIndex &0xf) <<24;

	if(uiCurrentIndex == 1)
	{	
		pMailBox->uMBOX_TX2.nREG = *pulKeyData++;
		pMailBox->uMBOX_TX3.nREG = *pulKeyData++;

		if(pARG->uLength>8)
		{	
			pMailBox->uMBOX_TX4.nREG = *pulKeyData++;
			pMailBox->uMBOX_TX5.nREG = *pulKeyData++;
		}
		else
		{
			pMailBox->uMBOX_TX4.nREG = 0;
			pMailBox->uMBOX_TX5.nREG = 0;
		}

		if(pARG->uLength>16)
		{	
			pMailBox->uMBOX_TX6.nREG = *pulKeyData++;
			pMailBox->uMBOX_TX7.nREG = *pulKeyData++;
		}
		else
		{
			pMailBox->uMBOX_TX6.nREG = 0;
			pMailBox->uMBOX_TX7.nREG = 0;
		}
	}
	else
	{
		pMailBox->uMBOX_TX2.nREG = *(pulKeyData + ((uiCurrentIndex-1)*6));
		pMailBox->uMBOX_TX3.nREG = *(pulKeyData + ((uiCurrentIndex-1)*6)+1);
		pMailBox->uMBOX_TX4.nREG = 0;
		pMailBox->uMBOX_TX5.nREG = 0;
		pMailBox->uMBOX_TX6.nREG = 0;
		pMailBox->uMBOX_TX7.nREG = 0;
	}	

	BITSET( pMailBox->uMBOX_CTL_016.nREG, Hw5); //OEN high
//	printk("%s, %d  end\n", __func__, __LINE__); 

    return 0;
}

int HWDEMUXCIPHERCMD_Set_IV(stHWDEMUXCIPHER_VECTOR *pARG)
{
	unsigned long *pulVectorData = (unsigned long *)pARG->pucData;
	volatile PMAILBOX pMailBox = (volatile PMAILBOX)tcc_p2v(HwCORTEXM4_MAILBOX0_BASE);
#if 0
	printk("%s, %d  \n", __func__, __LINE__); 
	printk("%s, %d uDemuxid = %d,  IV_uLength = %d, uOption = %d \n", __func__, __LINE__, pARG->uDemuxid, pARG->uLength, pARG->uOption); 
	{
		printk("%s, %d  [0x%x][0x%x][0x%x][0x%x][0x%x][0x%x][0x%x][0x%x]\n", __func__, __LINE__,
			*pARG->pucData, *(pARG->pucData+1), *(pARG->pucData+2), *(pARG->pucData+3), *(pARG->pucData+4), 
			*(pARG->pucData+5), *(pARG->pucData+6), *(pARG->pucData+7)); 
	}
#endif

	BITCLR( pMailBox->uMBOX_CTL_016.nREG, Hw5); //OEN low
	pMailBox->uMBOX_TX0.nREG = (HW_CM_CIPHER_SET_IV << 24 |(pARG->uDemuxid &0xff <<16) |
						(pARG->uLength & 0xff) << 8 | (pARG->uOption & 0xff));
	pMailBox->uMBOX_TX1.nREG =  *pulVectorData++ ;
	pMailBox->uMBOX_TX2.nREG =  *pulVectorData++;
	if(pARG->uLength>8)
	{	
		pMailBox->uMBOX_TX3.nREG = *pulVectorData++;
		pMailBox->uMBOX_TX4.nREG = *pulVectorData++;
	}
	else
	{
		pMailBox->uMBOX_TX3.nREG = 0;
		pMailBox->uMBOX_TX4.nREG = 0;
	}
	pMailBox->uMBOX_TX5.nREG = 0;
	pMailBox->uMBOX_TX6.nREG = 0;
	pMailBox->uMBOX_TX7.nREG = 0;
	BITSET( pMailBox->uMBOX_CTL_016.nREG, Hw5); //OEN high

//	printk("%s, %d  end\n", __func__, __LINE__); 

	return 0;
}

int HWDEMUXCIPHERCMD_Cipher_Run(stHWDEMUXCIPHER_EXECUTE *pARG)
{
//	printk("%s, %d  \n", __func__, __LINE__); 

    volatile PMAILBOX pMailBox = (volatile PMAILBOX)tcc_p2v(HwCORTEXM4_MAILBOX0_BASE);
    BITCLR( pMailBox->uMBOX_CTL_016.nREG, Hw5); //OEN low
    pMailBox->uMBOX_TX0.nREG = (HW_CM_CIPHER_EXECUTE << 24 |(pARG->uDemuxid &0xff <<16) |
							(pARG->uExecuteOption& 0xff)<<8 |(pARG->uCipherExecute & 0xff) );
    pMailBox->uMBOX_TX1.nREG = 0;
    pMailBox->uMBOX_TX2.nREG = 0;
    pMailBox->uMBOX_TX3.nREG = 0;
    pMailBox->uMBOX_TX4.nREG = 0;
    pMailBox->uMBOX_TX5.nREG = 0;
    pMailBox->uMBOX_TX6.nREG = 0;
    pMailBox->uMBOX_TX7.nREG = 0;
    BITSET( pMailBox->uMBOX_CTL_016.nREG, Hw5); //OEN high

//	printk("%s, %d  \n", __func__, __LINE__); 
    return 0;
}

