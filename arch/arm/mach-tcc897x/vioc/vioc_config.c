/*
 * linux/arch/arm/mach-tcc893x/vioc_config.c
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
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/of_address.h>

#include <mach/bsp.h>
#include <mach/io.h>
#include <mach/vioc_global.h>
#include <mach/vioc_config.h>



#define dprintk(msg...)	if (0) { printk( "Vioc_config: " msg); }


VIOC_CONFIG_PATH_u *VIOC_CONFIG_GetPathStruct (unsigned int nType)
{
	VIOC_CONFIG_PATH_u *pConfigPath = NULL;
	volatile PVIOC_IREQ_CONFIG pIREQConfig =VIOC_IREQConfig_GetAddress();

	switch(nType) {
		case (VIOC_SC0):
			pConfigPath = (VIOC_CONFIG_PATH_u *)&pIREQConfig->uSC0.nREG;
			break;
		case (VIOC_SC1):
			pConfigPath = (VIOC_CONFIG_PATH_u *)&pIREQConfig->uSC1.nREG;
			break;
		case (VIOC_SC2):
			pConfigPath = (VIOC_CONFIG_PATH_u *)&pIREQConfig->uSC2.nREG;
			break;
		case (VIOC_SC3):
			pConfigPath = (VIOC_CONFIG_PATH_u *)&pIREQConfig->uSC3.nREG;
			break;
		case (VIOC_VIQE):
			pConfigPath = (VIOC_CONFIG_PATH_u *)&pIREQConfig->uVIQE.nREG;
			break;
		case (VIOC_DEINTLS):
			pConfigPath = (VIOC_CONFIG_PATH_u *)&pIREQConfig->uDEINTLS.nREG;
			break;
		case (VIOC_FCDEC0   ) : 
			pConfigPath = (VIOC_CONFIG_PATH_u *)&pIREQConfig->uFCDEC0.nREG;
			break;
		case (VIOC_FCDEC1   ) : 
			pConfigPath = (VIOC_CONFIG_PATH_u *)&pIREQConfig->uFCDEC1.nREG;
			break;
		case (VIOC_FCENC0   ) : 
			pConfigPath = (VIOC_CONFIG_PATH_u *)&pIREQConfig->uFCENC0.nREG;
			break;
		case (VIOC_FCENC1   ) : 
			pConfigPath = (VIOC_CONFIG_PATH_u *)&pIREQConfig->uFCENC1.nREG;
			break;

		//case (VIOC_VIQE):
		//case (VIOC_DEINTLS):
		//case (VIOC_FILT2D   ) : return (&gpConfig->uFILT2D.bReg);
		//case (VIOC_FDELAY0  ) : return (&gpConfig->uFDELAY0.bReg);
		//case (VIOC_FDELAY1  ) : return (&gpConfig->uFDELAY1.bReg);
		//case (VIOC_DEBLOCK  ) : return (&gpConfig->uDEBLOCK.bReg);
		default:
			break;
	}

	return pConfigPath;
}

int VIOC_CONFIG_PlugIn (unsigned int nType, unsigned int nValue)
{
	VIOC_CONFIG_PATH_u *pConfigPath = NULL;
	unsigned int nStatus, nSelect, loop = 100;

	pConfigPath = VIOC_CONFIG_GetPathStruct(nType);
	if(pConfigPath == NULL) {
		printk("VIOC_CONFIG_PlugIn:  invalid path type. \n");
		return VIOC_DEVICE_INVALID;
	}

	nStatus = (pConfigPath->nREG>>16) & 0x3;
	nSelect = pConfigPath->nREG & 0xFF;
	if((nStatus == VIOC_PATH_CONNECTED) && (nSelect != nValue))
	{
		printk("%s, Type(%d) is plugged-out by force!!\n", __func__, nType);
		VIOC_CONFIG_PlugOut(nType);
	}
		
	BITCSET(pConfigPath->nREG, 0xFF, nValue);
	BITCSET(pConfigPath->nREG, (0x1<<31), (0x1<<31));

	if((pConfigPath->nREG>>18) & 0x1) {
		printk("VIOC_CONFIG_PlugIn:  path configuration error(1). device is busy. Type:%d Value:%d\n", nType, nValue);
		BITCSET(pConfigPath->nREG, (0x1<<31), (0x0<<31));
		return VIOC_DEVICE_BUSY;
	}

	while(1) {
		mdelay(1);
		loop--;
		nStatus = (pConfigPath->nREG>>16) & 0x3;
		if(nStatus == VIOC_PATH_CONNECTED) 	break;
		if(loop < 1) {
			printk("VIOC_CONFIG_PlugIn:  path configuration error(2). device is busy. Type:%d Value:%d\n", nType, nValue);
			return VIOC_DEVICE_BUSY;
		}
	}
    dprintk(" @@@@@ PlugIn: device(%d) in RDMA%d \n", nType, nValue);

	return VIOC_DEVICE_CONNECTED;
}EXPORT_SYMBOL(VIOC_CONFIG_PlugIn);

int VIOC_CONFIG_PlugOut(unsigned int nType)
{
	VIOC_CONFIG_PATH_u *pConfigPath = NULL;
	unsigned int nStatus, loop = 100;

	pConfigPath = VIOC_CONFIG_GetPathStruct(nType);
	if(pConfigPath == NULL) {
		printk("VIOC_CONFIG_PlugOut:  invalid path type. \n");
		return VIOC_DEVICE_INVALID;
	}

	nStatus = (pConfigPath->nREG>>16) & 0x3;
	if(nStatus == VIOC_PATH_DISCONNECTED)
	{
		pConfigPath->nREG = 0;	// bugfix: force clear by DY
		printk("%s, Type(%d) was already plugged-out!!\n", __func__, nType);
		return VIOC_PATH_DISCONNECTED;
	}

	BITCSET(pConfigPath->nREG, (0x1<<31), (0x0<<31));

	if((pConfigPath->nREG>>18) & 0x1) {
		printk("VIOC_CONFIG_PlugOut:  path configuration error(1). device is busy. Type:%d\n", nType);
		BITCSET(pConfigPath->nREG, (0x1<<31), (0x0<<31));
		return VIOC_DEVICE_BUSY;
	}

	while(1) {
		mdelay(1);
		loop--;
		nStatus = (pConfigPath->nREG>>16) & 0x3;
		if(nStatus == VIOC_PATH_DISCONNECTED) 	break;
		if(loop < 1) {
			printk("VIOC_CONFIG_PlugOut:  path configuration error(2). device is busy. Type:%d\n", nType);
			return VIOC_DEVICE_BUSY;
		}
	}
	pConfigPath->nREG = 0;	// bugfix: force clear by DY
    dprintk(" @@@@@ PlugOut: device(%d) \n", nType);

	return VIOC_PATH_DISCONNECTED;
}EXPORT_SYMBOL(VIOC_CONFIG_PlugOut);

void VIOC_CONFIG_RDMA12PathCtrl(unsigned int Path)
{
	/* Path - 0:  RDMA12 PATH , 	1:  VIDEOIN2 PATH */
	volatile PVIOC_IREQ_CONFIG pRDMAPath = (volatile PVIOC_IREQ_CONFIG)VIOC_IREQConfig_GetAddress();
	//pRDMAPath->RDMA12 = Path;
	BITCSET(pRDMAPath->uMISC.nREG, (0x1<<30), (Path<<30));
}

void VIOC_CONFIG_RDMA14PathCtrl(unsigned int Path)
{
	/* Path - 0:  RDMA14 PATH , 	1:  VIDEOIN3 PATH */
	volatile PVIOC_IREQ_CONFIG pRDMAPath = (volatile PVIOC_IREQ_CONFIG)VIOC_IREQConfig_GetAddress();
	//pRDMAPath->RDMA14 = Path;
	BITCSET(pRDMAPath->uMISC.nREG, (0x1<<31), (Path<<31));
}

void VIOC_CONFIG_WMIXPath(unsigned int Path, unsigned int Mode)
{
	/* Mode - 0: BY-PSSS PATH , 	1:  WMIX PATH */
	volatile PVIOC_IREQ_CONFIG pWMIXPath = (volatile PVIOC_IREQ_CONFIG)VIOC_IREQConfig_GetAddress();

	switch(Path) {
		case WMIX00:
			//pWMIXPath->WMIX0_0 = Mode;
			BITCSET(pWMIXPath->uMISC.nREG, (0x1<<16), (Mode<<16));
			break;
		case WMIX03:
			//pWMIXPath->WMIX0_1 = Mode;
			BITCSET(pWMIXPath->uMISC.nREG, (0x1<<17), (Mode<<17));
			break;
		case WMIX10:
			//pWMIXPath->WMIX1_0 = Mode;
			BITCSET(pWMIXPath->uMISC.nREG, (0x1<<18), (Mode<<18));
			break;
		case WMIX13:
			//pWMIXPath->WMIX1_1 = Mode;
			BITCSET(pWMIXPath->uMISC.nREG, (0x1<<19), (Mode<<19));
			break;
		case WMIX30:
			//pWMIXPath->WMIX3_0 = Mode;
			BITCSET(pWMIXPath->uMISC.nREG, (0x1<<22), (Mode<<22));
			break;
		case WMIX40:
			//pWMIXPath->WMIX4_0 = Mode;
			BITCSET(pWMIXPath->uMISC.nREG, (0x1<<24), (Mode<<24));
			break;
		case WMIX50:
			//pWMIXPath->WMIX5_0 = Mode;
			BITCSET(pWMIXPath->uMISC.nREG, (0x1<<26), (Mode<<26));
			break;
		case WMIX60:
			//pWMIXPath->WMIX6_0 = Mode;
			BITCSET(pWMIXPath->uMISC.nREG, (0x1<<28), (Mode<<28));
			break;
	}
}
EXPORT_SYMBOL(VIOC_CONFIG_WMIXPath);

void VIOC_CONFIG_SWReset(VIOC_IREQ_CONFIG* pVIOCConfig, unsigned int vioc,unsigned int vioc_index,unsigned int Mode)
{
	if(Mode == VIOC_CONFIG_RESET)
	{
		switch(vioc)
		{
			case VIOC_CONFIG_WMIXER:
				BITCSET(pVIOCConfig->uSOFTRESET.nREG[1], (1<<(vioc_index+9)), (1<<(vioc_index+9)));
				break;
				
			case VIOC_CONFIG_WDMA:
				BITCSET(pVIOCConfig->uSOFTRESET.nREG[1], (1<<(vioc_index)), (1<<(vioc_index)));
				break;
				
			case VIOC_CONFIG_RDMA:
				BITCSET(pVIOCConfig->uSOFTRESET.nREG[0], (1<<(vioc_index)), (1<<(vioc_index)));
				break;

			case VIOC_CONFIG_SCALER:
				BITCSET(pVIOCConfig->uSOFTRESET.nREG[0], (1<<(vioc_index+28)), (1<<(vioc_index+28)));
				break;
				
			case VIOC_CONFIG_VIQE:
				BITCSET(pVIOCConfig->uSOFTRESET.nREG[1], (1<<(vioc_index+16)), (1<<(vioc_index+16)));
				break;
				
			case VIOC_CONFIG_VIN:
				BITCSET(pVIOCConfig->uSOFTRESET.nREG[0], (1<<(vioc_index+24)), (1<<(vioc_index+24)));
				break;

			case VIOC_CONFIG_MC:
				BITCSET(pVIOCConfig->uSOFTRESET.nREG[1], (1<<(vioc_index+26)), (1<<(vioc_index+26)));
				break;

			case VIOC_CONFIG_FCENC:
				BITCSET(pVIOCConfig->uSOFTRESET.nREG[0], (1<<(vioc_index+22)), (1<<(vioc_index+22)));
				break;
				
			case VIOC_CONFIG_FCDEC:
				BITCSET(pVIOCConfig->uSOFTRESET.nREG[0], (1<<(vioc_index+18)), (1<<(vioc_index+18)));
				break;
			case VIOC_CONFIG_DEINTS:
				BITCSET(pVIOCConfig->uSOFTRESET.nREG[1], (1<<(vioc_index+17)), (1<<(vioc_index+17)));
				break;

			case VIOC_CONFIG_FIFO:
				BITCSET(pVIOCConfig->uSOFTRESET.nREG[1], (1U<<(vioc_index+31)), (1U<<(vioc_index+31)));
				break;

		}
		
	}
	else if(Mode == VIOC_CONFIG_CLEAR)
	{
		switch(vioc)
		{
			case VIOC_CONFIG_WMIXER:
				BITCSET(pVIOCConfig->uSOFTRESET.nREG[1], (1<<(vioc_index+9)), (0<<(vioc_index+9)));
				break;
				
			case VIOC_CONFIG_WDMA:
				BITCSET(pVIOCConfig->uSOFTRESET.nREG[1], (1<<(vioc_index)), (0<<(vioc_index)));
				break;

			case VIOC_CONFIG_RDMA:
				BITCSET(pVIOCConfig->uSOFTRESET.nREG[0], (1<<(vioc_index)), (0<<(vioc_index)));
				break;
				
			case VIOC_CONFIG_SCALER:
				BITCSET(pVIOCConfig->uSOFTRESET.nREG[0], (1<<(vioc_index+28)), (0<<(vioc_index+28)));
				break;
				
			case VIOC_CONFIG_VIQE:
				BITCSET(pVIOCConfig->uSOFTRESET.nREG[1], (1<<(vioc_index+16)), (0<<(vioc_index+16)));
				break;
				
			case VIOC_CONFIG_VIN:
				BITCSET(pVIOCConfig->uSOFTRESET.nREG[0], (1<<(vioc_index+24)), (0<<(vioc_index+24)));
				break;

			case VIOC_CONFIG_MC:
				BITCSET(pVIOCConfig->uSOFTRESET.nREG[1], (1<<(vioc_index+26)), (0<<(vioc_index+26)));
				break;	
				
			case VIOC_CONFIG_FCENC:
				BITCSET(pVIOCConfig->uSOFTRESET.nREG[0], (1<<(vioc_index+22)), (0<<(vioc_index+22)));
				break;
				
			case VIOC_CONFIG_FCDEC:
				BITCSET(pVIOCConfig->uSOFTRESET.nREG[0], (1<<(vioc_index+18)), (0<<(vioc_index+18)));
				break;

			case VIOC_CONFIG_DEINTS:
				BITCSET(pVIOCConfig->uSOFTRESET.nREG[1], (1<<(vioc_index+17)), (0<<(vioc_index+17)));
				break;		

			case VIOC_CONFIG_FIFO:
				BITCSET(pVIOCConfig->uSOFTRESET.nREG[1], (1U<<(vioc_index+31)), (0U<<(vioc_index+31)));
				break;

		}
	}
}
EXPORT_SYMBOL(VIOC_CONFIG_SWReset);

int VIOC_CONFIG_CheckPlugInOut(unsigned int nDevice)
{
	VIOC_CONFIG_PATH_u *pConfigPath = NULL;
	unsigned int nStatus, tmp;
	//int ret = -1;

	pConfigPath = VIOC_CONFIG_GetPathStruct(nDevice);
	if(pConfigPath == NULL) {
		printk("Invalid Path Type. \n");
		return (VIOC_DEVICE_INVALID);
	}

	for(tmp=0; tmp < 100; tmp++) {
		mdelay(1);
		nStatus = (pConfigPath->nREG>>16) & 0x3;
		if(nStatus == VIOC_PATH_DISCONNECTED) 	break;
		if(tmp == 99) 	printk("VIOC_CONFIG_CheckPlugInOut:  device is busy. \n");
	}

	return VIOC_PATH_DISCONNECTED;
}


/*
VIOC_CONFIG_Device_PlugState 
Check PlugInOut status of VIOC SCALER, VIQE, DEINTLS.
nDevice : VIOC_SC0, VIOC_SC1, VIOC_SC2, VIOC_VIQE, VIOC_DEINTLS
pDstatus : Pointer of status value.
return value : Device name of Plug in.
*/
int VIOC_CONFIG_Device_PlugState(unsigned int nDevice, VIOC_PlugInOutCheck *VIOC_PlugIn)
{
	VIOC_CONFIG_PATH_u *pConfigPath = NULL;
	//unsigned int nStatus = VIOC_PATH_DISCONNECTED;
	//unsigned int tmp;
	//int ret = -1;

	pConfigPath = VIOC_CONFIG_GetPathStruct(nDevice);
	if(pConfigPath == NULL)	{
		return (VIOC_DEVICE_INVALID);
	}
	
	VIOC_PlugIn->enable = pConfigPath->bREG.EN;
	VIOC_PlugIn->connect_device = pConfigPath->bREG.SELECT;
	VIOC_PlugIn->connect_statue = pConfigPath->bREG.STATUS;

	return VIOC_DEVICE_CONNECTED;
}

#define tca_SC_PlugIn_RdmaN(x)		(x)
int tca_get_RDMA_PlugIn_to_Scaler(unsigned int RdmaN)
{
	int i, ret;
	VIOC_PlugInOutCheck VIOC_PlugIn;

	for(i =0; i <= VIOC_SC3; i++)
	{
		ret = VIOC_CONFIG_Device_PlugState(i, &VIOC_PlugIn);
		if(ret == VIOC_DEVICE_CONNECTED &&
			(VIOC_PlugIn.enable && VIOC_PlugIn.connect_device == tca_SC_PlugIn_RdmaN(RdmaN)))
			return i;
	}
	return -1;
}


int tca_get_MC_Connect_to_RDMA(unsigned int *MC_NUM, unsigned int RdmaN)
{
	int ret = 0;
	unsigned int MC1_SEL;
	volatile PVIOC_IREQ_CONFIG pIREQConfig = VIOC_IREQConfig_GetAddress();
	MC1_SEL = ((pIREQConfig->uMC.nREG >> 16) & 0x7);
	*MC_NUM = 1;
	
	switch(RdmaN)
	{
		case 3:
			ret  =  ISSET(pIREQConfig->uMC.nREG,  0x1);
			*MC_NUM = 0;
			break;

		case 7:
			if(ISSET(pIREQConfig->uMC.nREG,  0x1<< 1) && (MC1_SEL == 0))
				ret = 1;
			break;

		case 11:
			if(ISSET(pIREQConfig->uMC.nREG,  0x1<< 2) && (MC1_SEL == 1))
				ret = 1;
			break;
			
		case 13:
			if(ISSET(pIREQConfig->uMC.nREG,  0x1<< 3) && (MC1_SEL == 2))
				ret = 1;
			break;			

		case 15:
			if(ISSET(pIREQConfig->uMC.nREG,  0x1<< 4) && (MC1_SEL == 3))
				ret = 1;
			break;
			
		case 16:
			if(ISSET(pIREQConfig->uMC.nREG,  0x1<< 5) && (MC1_SEL == 4))
				ret = 1;
			break;
			
		case 17:
			if(ISSET(pIREQConfig->uMC.nREG,  0x1<< 6) && (MC1_SEL == 5))
				ret = 1;
			break;

		default :
			*MC_NUM = 0xFF;
			ret = 0;
	}

	return ret;
}


int tca_set_MC_Connect_to_RDMA(unsigned int McN, unsigned int RdmaN, unsigned int SetClr)
{
	int ret = 0;
	volatile PVIOC_IREQ_CONFIG pIREQConfig = VIOC_IREQConfig_GetAddress();
	switch(RdmaN)
	{
		case 3:
			ret  =  BITCSET(pIREQConfig->uMC.nREG, 0x1, SetClr);
			break;

		case 7:
			BITCSET(pIREQConfig->uMC.nREG,  (0x1<< 1) | (0x7<< 16), (SetClr << 1) | (0x0<< 16));
			break;

		case 11:
			BITCSET(pIREQConfig->uMC.nREG,  (0x1<< 2) | (0x7<< 16) , (SetClr << 2) | (0x1<< 16));
			break;

		case 13:
			BITCSET(pIREQConfig->uMC.nREG,  (0x1<< 3) | (0x7<< 16), (SetClr << 3) | (0x2<< 16));

			break;			
		case 15:
			BITCSET(pIREQConfig->uMC.nREG,  (0x1<< 4) | (0x7<< 16), (SetClr << 4) | (0x3<< 16));
			break;			
		case 16:
			BITCSET(pIREQConfig->uMC.nREG,  (0x1<< 5) | (0x7<< 16), (SetClr << 5) | (0x4<< 16));
			break;			
		case 17:
			BITCSET(pIREQConfig->uMC.nREG,  (0x1<< 6) | (0x7<< 16), (SetClr << 6) | (0x5<< 16));
			break;

	}

	return ret;
}

static PVIOC_IREQ_CONFIG pIREQ;
VIOC_IREQ_CONFIG* VIOC_IREQConfig_GetAddress(void)
{
	if(pIREQ == NULL)
		pr_err("%s pDDICONFIG:%p \n", __func__, pIREQ);

	return pIREQ;
}

#include <asm/io.h>
#define VIOC_CFG_MISC1		0x0084
void vioc_config_stop_req(unsigned int en)
{
	void __iomem *reg = (void __iomem *)pIREQ;

	if (en)
		__raw_writel(__raw_readl(reg+VIOC_CFG_MISC1) & ~(1<<23), reg+VIOC_CFG_MISC1);
	else
		__raw_writel(__raw_readl(reg+VIOC_CFG_MISC1) | (1<<23), reg+VIOC_CFG_MISC1);
}

static int __init vioc_config_init(void)
{
	struct device_node *ViocConfig_np;

	ViocConfig_np = of_find_compatible_node(NULL, NULL, "telechips,vioc_config");

	if(ViocConfig_np == NULL) {
		pr_err("cann't find vioc_config \n");
	}
	else {
		pIREQ = of_iomap(ViocConfig_np, 0);
		pr_info("%s ViocConfig :%p \n", __func__, pIREQ);
	}
	return 0;
}
arch_initcall(vioc_config_init);