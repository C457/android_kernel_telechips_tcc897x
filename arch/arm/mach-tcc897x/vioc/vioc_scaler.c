/*
 * linux/arch/arm/mach-tcc893x/vioc_scaler.c
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
#include <linux/kernel.h>
#include <mach/bsp.h>
#include <mach/io.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <mach/vioc_global.h>
#include <mach/vioc_scaler.h>
#include <mach/vioc_ireq.h>

void VIOC_SC_SetBypass(PVIOC_SC pSCALER, unsigned int nOnOff)
{
	//pSCALER->uCTRL.bREG.BP  = nOnOff;
	BITCSET(pSCALER->uCTRL.nREG, 0x1, nOnOff);
}

void VIOC_SC_SetUpdate(PVIOC_SC pSCALER)
{
	//pSCALER->uCTRL.bREG.UPD  = 1;
	BITCSET(pSCALER->uCTRL.nREG, (0x1<<16), (0x1<<16));
}EXPORT_SYMBOL(VIOC_SC_SetUpdate);

void VIOC_SC_SetSrcSize(PVIOC_SC pSCALER, unsigned int nWidth, unsigned int nHeight)
{
	//pSCALER->uSRCSIZE.bREG.WIDTH    = nWidth;
	//pSCALER->uSRCSIZE.bREG.HEIGHT   = nHeight;
	BITCSET(pSCALER->uSRCSIZE.nREG, (0xFFFF<<16), (nHeight<<16));
	BITCSET(pSCALER->uSRCSIZE.nREG, 0xFFFF, nWidth);
}EXPORT_SYMBOL(VIOC_SC_SetSrcSize);

void VIOC_SC_SetDstSize(PVIOC_SC pSCALER, unsigned int nWidth, unsigned int nHeight)
{
	//pSCALER->uDSTSIZE.bREG.WIDTH    = nWidth;
	//pSCALER->uDSTSIZE.bREG.HEIGHT   = nHeight;
	BITCSET(pSCALER->uDSTSIZE.nREG, (0xFFFF<<16), (nHeight<<16));
	BITCSET(pSCALER->uDSTSIZE.nREG, 0xFFFF, nWidth);
}EXPORT_SYMBOL(VIOC_SC_SetDstSize);

void VIOC_SC_SetOutSize(PVIOC_SC pSCALER, unsigned int nWidth, unsigned int nHeight)
{
	//pSCALER->uOUTSIZE.bREG.WIDTH    = nWidth;
	//pSCALER->uOUTSIZE.bREG.HEIGHT   = nHeight;
	BITCSET(pSCALER->uOUTSIZE.nREG, (0xFFFF<<16), (nHeight<<16));
	BITCSET(pSCALER->uOUTSIZE.nREG, 0xFFFF, nWidth);
}EXPORT_SYMBOL(VIOC_SC_SetOutSize);

void VIOC_SC_SetOutPosition(PVIOC_SC pSCALER, unsigned int nXpos, unsigned int nYpos)
{
	//pSCALER->uOUTPOS.bREG.XPOS  = nXpos;
	//pSCALER->uOUTPOS.bREG.YPOS  = nYpos;
	BITCSET(pSCALER->uOUTPOS.nREG, (0xFFFF<<16), (nYpos<<16));
	BITCSET(pSCALER->uOUTPOS.nREG, 0xFFFF, nXpos);
}

void VIOC_SC_SetInterruptMask(PVIOC_SC pSCALER, unsigned int nMask, unsigned int set)
{
/*
	if(set == 0) // Interrupt Enable
	{
		pSCALER->uIRQMSK.nREG &= ~nMask;
	}
	else // Interrupt Diable
	{
		pSCALER->uIRQMSK.nREG |= nMask;
	}
*/
	BITCSET(pSCALER->uIRQMSK.nREG, nMask, set);
}

void VIOC_SC_IreqHandler(int index, int irq, void *client_data)
{
	//unsigned int idx = 0; // temp value.
	volatile PVIOC_SC pScaler;
	pScaler = (volatile PVIOC_SC)tcc_p2v((unsigned int)HwVIOC_SC0 + index*0x100);

#if 0
	if(vectorID > VIOC_IREQ_MAX)
		return;

	idx = vectorID - VIOC_IREQ_SC0;
#endif

	if(pScaler->uSTATUS.nREG & VIOC_SC_IREQ_UPDDONE_MASK && pScaler->uIRQMSK.bREG.UPDDONE == 0)
	{
		/* Interrput Source Clear */
		/* Status Clear */
		pScaler->uSTATUS.bREG.UPDDONE = 1;
		/* DO Action for Interrupt */

		/* Interrput Re-Setting */
	}

	if(pScaler->uSTATUS.nREG & VIOC_SC_IREQ_EOFRISE_MASK && pScaler->uIRQMSK.bREG.EOFRISE == 0)
	{
		/* Interrput Source Clear */
		/* Status Clear */
		pScaler->uSTATUS.bREG.EOFRISE = 1;

		/* DO Action for Interrupt */

		/* Interrput Re-Setting */
	}

	if(pScaler->uSTATUS.nREG & VIOC_SC_IREQ_EOFFALL_MASK && pScaler->uIRQMSK.bREG.EOFFALL == 0)
	{
		/* Interrput Source Clear */
		/* Status Clear */
		pScaler->uSTATUS.bREG.EOFFALL = 1;

		/* DO Action for Interrupt */

		/* Interrput Re-Setting */
	}

	if(pScaler->uSTATUS.nREG & VIOC_SC_IREQ_ERROR_MASK && pScaler->uIRQMSK.bREG.ERR == 0)
	{
		/* Interrput Source Clear */
		/* Status Clear */
		pScaler->uSTATUS.bREG.ERR = 1;

		/* DO Action for Interrupt */

		/* Interrput Re-Setting */
	}
}

void VIOC_SC_SetSWReset(unsigned int SC, unsigned int RDMA, unsigned int WDMA)
{
	volatile PVIOC_IREQ_CONFIG pIREQConfig;
	pIREQConfig = (volatile PVIOC_IREQ_CONFIG)tcc_p2v((unsigned int)HwVIOC_IREQ);

	if(SC <= VIOC_SC2) {
		BITCSET(pIREQConfig->uSOFTRESET.nREG[0], (0x1<<(28+SC)), (0x1<<(28+SC))); // scaler reset
		BITCSET(pIREQConfig->uSOFTRESET.nREG[0], (0x1<<(28+SC)), (0x0<<(28+SC))); // scaler reset
	}

	if(RDMA <= VIOC_SC_RDMA_17) {
		BITCSET(pIREQConfig->uSOFTRESET.nREG[0], (0x1<<RDMA), (0x1<<RDMA)); // rdma reset
		BITCSET(pIREQConfig->uSOFTRESET.nREG[0], (0x1<<RDMA), (0x0<<RDMA)); // rdma reset
	}

	if(WDMA <= VIOC_SC_WDMA_08) {
		BITCSET(pIREQConfig->uSOFTRESET.nREG[1], (0x1<<WDMA), (0x1<<WDMA)); // wdma reset
		BITCSET(pIREQConfig->uSOFTRESET.nREG[1], (0x1<<WDMA), (0x00<<WDMA)); // wdma reset
	}
}




static struct device_node *ViocSC_np;
VIOC_SC* VIOC_SC_GetAddress(unsigned int sc_num)
{
	VIOC_SC *pScaler = NULL;

	if(ViocSC_np == NULL)
		pr_err("%s VIOC_SC NULL : %p \n", __func__, ViocSC_np);
	else
	{
		switch(sc_num)
		{
			case VIOC_SC0:
			case VIOC_SC1:
			case VIOC_SC2:
			case VIOC_SC3:
				pScaler = (VIOC_SC *)of_iomap(ViocSC_np, sc_num);
				break;
//			case VIOC_SC4:
//				pScaler = (VIOC_SC *)of_iomap(ViocSC_np, 4);
//				break;
			default:
				pr_err("Error : scaler number over range %d : function:%s \n  ", sc_num, __func__);
				break;
		}
	}

	return pScaler;
}

void VIOC_SCALER_DUMP(VIOC_SC *pSCALER)
{
	unsigned int cnt = 0;
	char *pReg = (char*)pSCALER;

	printk("SCALER :: %p \n", pSCALER);
	while(cnt < 0x100)
	{
		printk("%p: 0x%08lx 0x%08lx 0x%08lx 0x%08lx\n",
			pReg+cnt,
			*((unsigned long*)(pReg+cnt)),
			*((unsigned long*)(pReg+cnt+0x4)),
			*((unsigned long*)(pReg+cnt+0x8)),
			*((unsigned long*)(pReg+cnt+0xC)));
		cnt += 0x10;
	}
}

static int __init vioc_sc_init(void)
{
	ViocSC_np = of_find_compatible_node(NULL, NULL, "telechips,scaler");

	if(ViocSC_np == NULL) {
		pr_err("cann't find vioc_scaler \n");
	}
	return 0;
}
arch_initcall(vioc_sc_init);
