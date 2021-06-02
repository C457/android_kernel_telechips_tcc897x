
/****************************************************************************
 *   FileName    : tca_tcchwcontrol.c
 *   Description : 
 ****************************************************************************
 *
 *   TCC Version 1.0
 *   Copyright (c) Telechips, Inc.
 *   ALL RIGHTS RESERVED
 *
 ****************************************************************************/
/*****************************************************************************
 *
 * includes
 *
 ******************************************************************************/
#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/gpio.h>

#include <asm/mach-types.h>

#include "tca_audio_hw.h"
#include "tca_tcchwcontrol.h"


/* System revision Info */
extern unsigned int system_rev; 

void tca_gpio_func(unsigned int base_addr, int pin_num, int func)
{
	PTCC_GPIO pGP = (PTCC_GPIO)tcc_p2v(base_addr);
	unsigned int breg=0xF;
	if((pin_num >= 0)&&(pin_num < 8)){
		breg = 0xF << (pin_num*4);
		BITCLR(pGP->func_select0, breg);
		breg = func << (pin_num*4);
		BITSET(pGP->func_select0, breg);
	}else if((pin_num >= 8)&&(pin_num < 16)){
		breg = 0xF << ((pin_num-8)*4);
		BITCLR(pGP->func_select1, breg);
		breg = func << ((pin_num-8)*4);
		BITSET(pGP->func_select1, breg);
	}else if((pin_num >= 16)&&(pin_num < 24)){
		breg = 0xF << ((pin_num-16)*4);
		BITCLR(pGP->func_select2, breg);
		breg = func << ((pin_num-16)*4);
		BITSET(pGP->func_select2, breg);
	}else if((pin_num >= 24)&&(pin_num < 32)){
		breg = 0xF << ((pin_num-24)*4);
		BITCLR(pGP->func_select3, breg);
		breg = func << ((pin_num-24)*4);
		BITSET(pGP->func_select3, breg);
	}else{
		printk("[%s] ERROR! Invalid input [pin_num:%d]\n", __func__, pin_num);
	}
}
/*****************************************************************************
 * Function Name : tca_tcc_DAO_mute()
 ******************************************************************************
 * Desription    : mute for TX. This is H/W mute for DAO that connected codec.
 * Parameter     : mute enable
 * Return        :
 ******************************************************************************/
int line_out_mute=0;
unsigned int tca_tcc_DAO_mute(bool enable)
{
	int ret=0;
	if(gpio_is_valid(line_out_mute)){
		gpio_request(line_out_mute, "UN_MUTE");
		if(enable){
#if defined(CONFIG_SND_SOC_WM8581) || defined(CONFIG_SND_SOC_WM8524)
			 gpio_direction_output(line_out_mute, 1);
#else
			 gpio_direction_output(line_out_mute, 0);
#endif
		}else{
#if defined(CONFIG_SND_SOC_WM8581) || defined(CONFIG_SND_SOC_WM8524)
			 gpio_direction_output(line_out_mute, 0);
#else
			 gpio_direction_output(line_out_mute, 1);
#endif
		}
		ret=0;
	}else{
		printk("WARN!! line_out_mute pin is invalid. Please check Device Tree file.\n");
		ret=-1;
	}

	return ret;
}
/*****************************************************************************
 * Function Name : tca_tcc_DAI_mute()
 ******************************************************************************
 * Desription    : mute for RX
 * Parameter     : Audio index, the number of I2S port, mute enable
 * Return        :
 ******************************************************************************/
void tca_tcc_DAI_mute(int index, int chs, bool enable)
{
	switch(index)
	{
		case 0:
			if(enable){
				if(chs > 2){
					tca_gpio_func(BASE_ADDR_GPIOG, 10, 0);		/* AUDIO0 DAI-0 */
					tca_gpio_func(BASE_ADDR_GPIOG, 12, 0);		/* AUDIO0 DAI-1 */
					tca_gpio_func(BASE_ADDR_GPIOG, 14, 0);		/* AUDIO0 DAI-2 */
					tca_gpio_func(BASE_ADDR_GPIOG, 16, 0);		/* AUDIO0 DAI-3 */
				}else{
					tca_gpio_func(BASE_ADDR_GPIOG, 10, 0);		/* AUDIO0 DAI-0 */
				}
			}else{
				if(chs > 2){
					tca_gpio_func(BASE_ADDR_GPIOG, 10, 1);		/* AUDIO0 DAI-0 */
					tca_gpio_func(BASE_ADDR_GPIOG, 12, 1);		/* AUDIO0 DAI-1 */
					tca_gpio_func(BASE_ADDR_GPIOG, 14, 1);		/* AUDIO0 DAI-2 */
					tca_gpio_func(BASE_ADDR_GPIOG, 16, 1);		/* AUDIO0 DAI-3 */
				}else{                                         
					tca_gpio_func(BASE_ADDR_GPIOG, 10, 1);		/* AUDIO0 DAI-0 */
				}
			}
			break;
		case 1:
			if(enable){
				if(chs > 2){
					tca_gpio_func(BASE_ADDR_GPIOG, 15, 0);		/* AUDIO1 DAI-0 */
					tca_gpio_func(BASE_ADDR_GPIOG, 17, 0);		/* AUDIO1 DAI-1 */
					tca_gpio_func(BASE_ADDR_GPIOG, 19, 0);		/* AUDIO1 DAI-2 */
					tca_gpio_func(BASE_ADDR_GPIOG, 6, 0);		/* AUDIO1 DAI-3 */
				}else{                                                   
					tca_gpio_func(BASE_ADDR_GPIOG, 15, 0);		/* AUDIO1 DAI-0 */
				}
			}else{
				if(chs > 2){
					tca_gpio_func(BASE_ADDR_GPIOG, 15, 9);		/* AUDIO1 DAI-0 */
					tca_gpio_func(BASE_ADDR_GPIOG, 17, 9);		/* AUDIO1 DAI-1 */
					tca_gpio_func(BASE_ADDR_GPIOG, 19, 9);		/* AUDIO1 DAI-2 */
					tca_gpio_func(BASE_ADDR_GPIOG, 6, 9);		/* AUDIO1 DAI-3 */
				}else{                                                   
					tca_gpio_func(BASE_ADDR_GPIOG, 15, 9);		/* AUDIO1 DAI-0 */
				}
			}
			break;
		case 2:
			if(enable){
				if(chs > 2){
					tca_gpio_func(BASE_ADDR_GPIOE, 26, 0);		/* AUDIO2 DAI-0 */
					tca_gpio_func(BASE_ADDR_GPIOE, 28, 0);		/* AUDIO2 DAI-1 */
					tca_gpio_func(BASE_ADDR_GPIOE, 30, 0);		/* AUDIO2 DAI-2 */
					tca_gpio_func(BASE_ADDR_GPIOE, 21, 0);		/* AUDIO2 DAI-3 */
				}else{                                                  
					tca_gpio_func(BASE_ADDR_GPIOE, 26, 0);		/* AUDIO2 DAI-0 */
				}
			}else{
				if(chs > 2){
					tca_gpio_func(BASE_ADDR_GPIOE, 26, 3);		/* AUDIO2 DAI-0 */
					tca_gpio_func(BASE_ADDR_GPIOE, 28, 3);		/* AUDIO2 DAI-1 */
					tca_gpio_func(BASE_ADDR_GPIOE, 30, 3);		/* AUDIO2 DAI-2 */
					tca_gpio_func(BASE_ADDR_GPIOE, 21, 3);		/* AUDIO2 DAI-3 */
				}else{                                                  
					tca_gpio_func(BASE_ADDR_GPIOE, 26, 3);		/* AUDIO2 DAI-0 */
				}
			}
			break;
		case 3:
			if(enable){
				if(chs > 2){
					printk("Error!! %s Invalid channels:%d. AUDIO3-I2S has only 1port for stereo channel!!\n", __func__, chs);
				}else{
					tca_gpio_func(BASE_ADDR_GPIOG, 4, 0);		/* AUDIO3 DAI-0 */
				}
			}else{
				if(chs > 2){
					printk("Error!! %s Invalid channels:%d. AUDIO3-I2S has only 1port for stereo channel!!\n", __func__, chs);
				}else{
					tca_gpio_func(BASE_ADDR_GPIOG, 4, 1);		/* AUDIO3 DAI-0 */
				}
			}
			break;
		default:
			printk("Error!! %s Invalid index:%d. It should be under 3!!\n", __func__, index);
			break;
	}

}
/*****************************************************************************
 * Function Name : tca_dma_clrstatus()
 ******************************************************************************
 * Desription    :
 * Parameter     : 
 * Return        : success(SUCCESS) 
 ******************************************************************************/
unsigned int tca_dma_clrstatus(void *pADMABaseAddr, unsigned int nDmanum)
{
    ADMA *pStrADMABaseReg = (ADMA *)pADMABaseAddr;
    if (nDmanum) {
	//If you use this register to write, set interrupt status is cleared.
	pStrADMABaseReg->IntStatus |= (Hw6 | Hw2);
    } 
    else 
    {
	//If you use this register to write, set interrupt status is cleared.
	pStrADMABaseReg->IntStatus |= (Hw4 | Hw0);
    }

    return 0/*SUCCESS*/;
}

/*****************************************************************************
 * Function Name : tca_irq_getnumber()
 ******************************************************************************
 * Desription    :
 * Parameter     : 
 * Return        : DMA number 
 ******************************************************************************/
unsigned int tca_irq_getnumber(void)
{
#if 1 // defined(CONFIG_ARCH_TCC892X) || defined(CONFIG_ARCH_TCC893X)
    return INT_ADMA0;
#else
    return IRQ_ADMA;
#endif
}

/*****************************************************************************
 * Function Name : tca_dma_getstatus()
 ******************************************************************************
 * Desription    : dma status check
 * Parameter     : 
 * Return        :  
 ******************************************************************************/
unsigned int tca_dma_getstatus(void *pADMABaseAddr, unsigned int nDmanum)
{
    ADMA *pStrADMABaseReg = (ADMA *)pADMABaseAddr;
    return nDmanum ? (pStrADMABaseReg->IntStatus & (Hw6 | Hw2)) : (pStrADMABaseReg->IntStatus & (Hw4 | Hw0));
}

/*****************************************************************************
 * Function Name : tca_dma_setsrcaddr()
 ******************************************************************************
 * Desription    : dma set source address
 * Parameter     : dma number
 * Return        : address
 ******************************************************************************/
unsigned int tca_dma_setsrcaddr(void *pADMABaseAddr, unsigned int DADONum, unsigned int nDmanum, unsigned int nAddr)	
{
    ADMA *pStrADMABaseReg = (ADMA *)pADMABaseAddr;

    switch(DADONum)
    {
	case 0:
	    pStrADMABaseReg->TxDaSar = nAddr;
	    break;
	case 1:
	    pStrADMABaseReg->TxDaSar1 = nAddr;
	    break;
	case 2:
	    pStrADMABaseReg->TxDaSar2 = nAddr;
	    break;
	case 3:
	    pStrADMABaseReg->TxDaSar3 = nAddr;
	    break;
	default:
	    break;
    }

    return 0/*SUCCESS*/;
}

/*****************************************************************************
 * Function Name : tca_dma_setdestaddr()
 ******************************************************************************
 * Desription	: set dest address
 * Parameter 	: dma number, dest address
 * Return		: SUCCESS 
 ******************************************************************************/
unsigned int tca_dma_setdestaddr(void *pADMABaseAddr, unsigned int DADINum, unsigned int nDmanum, unsigned int nAddr)	
{
    ADMA *pStrADMABaseReg = (ADMA *)pADMABaseAddr;	

    switch(DADINum)
    {
	case 0:
	    pStrADMABaseReg->RxDaDar = nAddr;
	    break;
	case 1:
	    pStrADMABaseReg->RxDaDar1 = nAddr;
	    break;
	case 2:
	    pStrADMABaseReg->RxDaDar2 = nAddr;
	    break;
	case 3:
	    pStrADMABaseReg->RxDaDar3 = nAddr;
	    break;
	default:
	    break;
    }
    return 0/*SUCCESS*/;
}

/*****************************************************************************
 * Function Name : tca_dma_control()
 ******************************************************************************
 * Desription    : dma input control
 * Parameter     : mode 1: on, 0:off
 * Return        : success(SUCCESS) 
 ******************************************************************************/
unsigned int tca_dma_control(void *pADMABaseAddr, void *pADMADAIBaseAddr, unsigned int nMode, unsigned int nDmanum, unsigned int nInMode)				
{  
    ADMA *pStrADMABaseReg = (ADMA *)pADMABaseAddr;

    if(nMode) 
    { //run
	tca_i2s_start(pADMADAIBaseAddr, nInMode);
#if defined(_WM8581_INCLUDE_)		// CONFIG_SND_SOC_TCC_MULTICHANNEL
	pStrADMABaseReg->RptCtrl = Hw26|Hw25|Hw24;//DAI Buffer Threshod is set Multi-Port
#else
	pStrADMABaseReg->RptCtrl = 0;//Initialize repeat control register
#endif
	if(nInMode)//input
	{
	    pStrADMABaseReg->ChCtrl |= 0
		| Hw30 //DMA Channel enable of DAI Rx
		| Hw14 //Width of Audio Data of DAI Rx 16bites
		| Hw2; //Interrupt enable of DAI Rx
	}
	else//output
	{
	    pStrADMABaseReg->ChCtrl |= 0
		| Hw28 //DMA Channel enable of DAI Tx
#if defined(_WM8581_INCLUDE_) // CONFIG_SND_SOC_TCC_MULTICHANNEL
		| Hw21 | Hw20//DMA Multi channel select : DAI0, DAI1, DAI2, DAI3
#endif
		| Hw12	//Width of Audio Data of DAI Tx 16bites
		| Hw0; //Interrupt enable of DAI Tx
	}
    }
    else 
    {
	tca_i2s_stop(pADMADAIBaseAddr, nInMode);
	if(nInMode)//input
	{
	    pStrADMABaseReg->ChCtrl &= ~Hw30; //DMA Channel disable of DAI Rx
	}
	else//output
	{
	    pStrADMABaseReg->ChCtrl &= ~Hw28; //DMA Channel disable of DAI Tx
	}
    }
    return 0/*SUCCESS*/;
}

/*****************************************************************************
 * Function Name : tca_i2s_setregister()
 ******************************************************************************
 * Desription    : set dai register
 * Parameter     : 
 * Return        : success(SUCCESS) 
 ******************************************************************************/
unsigned int tca_i2s_setregister(void *pADMADAIBaseAddr, unsigned int nRegval)
{
    ADMADAI*pStrADMADAIBaseReg = (ADMADAI *)pADMADAIBaseAddr;
    pStrADMADAIBaseReg->DAMR = nRegval;
    return 0;
}

/*****************************************************************************
 * Function Name : tca_i2s_getregister()
 ******************************************************************************
 * Desription    : get dai register
 * Parameter     : 
 * Return        : success(SUCCESS) 
 ******************************************************************************/
unsigned int tca_i2s_getregister(void *pADMADAIBaseAddr)
{
    ADMADAI *pStrADMADAIBaseReg = (ADMADAI *)pADMADAIBaseAddr;
    return (pStrADMADAIBaseReg->DAMR);
}

/*****************************************************************************
 * Function Name : tca_i2s_dai_init()
 ******************************************************************************
 * Desription    : i2s init
 * Parameter     : 
 * Return        : success(SUCCESS) 
 ******************************************************************************/
unsigned int tca_i2s_dai_init(void *pADMADAIBaseAddr)
{
    ADMADAI *pStrADMADAIBaseReg = (ADMADAI *)pADMADAIBaseAddr;

    //Not used volume control
    pStrADMADAIBaseReg->DAVC = 0;

    pStrADMADAIBaseReg->DAMR = 0
	| Hw31 	//BCLK direct at master mode
	| Hw30 	//LRCLK direct at master mode	
#if defined(_WM8581_INCLUDE_) /* CONFIG_SND_SOC_TCC_MULTICHANNEL */
	| Hw29	//DAI Buffer Threshold Enable
	| Hw28	//Multi-Port Enable
#endif

#ifdef I2S_24BIT_MODE
		| Hw21	//DAI Rx shift Bit-pack LSB and 24bit mode
		| Hw19	//DAI Tx shift Bit-pack LSB and 24bit mode
#else
		| Hw21| Hw20 //DAI Rx shift Bit-pack LSB and 16bit mode
		| Hw19| Hw18 //DAI Tx shift Bit-pack LSB and 16bit mode
#endif

#if defined(CONFIG_SND_SOC_DAUDIO_CLOCK_CONTROL)
		//| Hw15	//DAI Enable
#else
		| Hw15	//DAI Enable
#endif
		| Hw11 	//DAI System Clock master select
		| Hw10 	//DAI Bit Clock master select
		| Hw9	//DAI Frame clock(LRCLK) master selcet 
		| Hw5	//DAI LRCLK(64fs->fs)
		;

    return(0);
}

/*****************************************************************************
 * Function Name : tca_i2s_initoutput()
 ******************************************************************************
 * Desription    : i2s init output
 * Parameter     : 
 * Return        : success(SUCCESS) 
 ******************************************************************************/
void tca_i2s_initoutput(void *pADMABaseAddr, unsigned int nOutputDma, unsigned int AUDIO_DMA_PAGE_SIZE, unsigned int SAMPLERATE )
{
    ADMA *pStrADMABaseReg = (ADMA *)pADMABaseAddr;
    unsigned int bitSet=0x0;
    //unsigned uSize;
    unsigned int DMABuffer = 0;

    //DMA Channel enable of DAI Tx
    pStrADMABaseReg->ChCtrl &= ~Hw28;

    DMABuffer = 0xFFFFFF00 / (((AUDIO_DMA_PAGE_SIZE*2) >> 4)<<8);
    DMABuffer = DMABuffer * (((AUDIO_DMA_PAGE_SIZE*2) >> 4)<<8);

    //uSize = AUDIO_DMA_PAGE_SIZE*2;

    // DAI Tx (Right) Source Address
    pStrADMABaseReg->TxDaParam = DMABuffer | 4; //(4|0xFFFFFE00);	

    // ADMA Count Reginster setting
    pStrADMABaseReg->TxDaTCnt = (((AUDIO_DMA_PAGE_SIZE)>>0x2)>>0x2) - 1;

    // Tx Transfer Control Refister Setting	
    bitSet = ( HwZERO
	    //Tx Trigger Type is edge-trigger
	    |Hw28 //DMA transfer begins from current source/destination address
	    //|Hw24 //Issue Locked Transfer of DAI Tx DMA
	    |Hw16 //Enable repeat mode control of DAI Tx DMA
	    |Hw9/*|Hw8*/ //Burst size of DAI Tx DMA is 4 cycle
	    |Hw1|Hw0 //Word size of DAI Tx DMA is 32bit
	    );

    pStrADMABaseReg->TransCtrl |= bitSet;
#if defined(_WM8581_INCLUDE_)		// CONFIG_SND_SOC_TCC_MULTICHANNEL
    pStrADMABaseReg->RptCtrl = Hw26|Hw25|Hw24;//DAI Buffer Threshod is set Multi-Port
#else
    pStrADMABaseReg->RptCtrl = 0;
#endif

    /* ADMA Repeat register setting */
    //	BITCLR(HwRptCtrl,HwRptCtrl_DRI_EN);	//DMA Interrupt is occurred when the end of each Repeated DMAoperation.

    // Tx channel Control Register Setting	
    //Start DMA
    bitSet = ( HwZERO
	    |Hw0 //Interrupt Enable of DAI Tx	
	    );

    pStrADMABaseReg->ChCtrl |= bitSet;
}

/*****************************************************************************
 * Function Name : tca_i2s_deinit()
 ******************************************************************************
 * Desription    : i2s deinit
 * Parameter     : 
 * Return        : success(SUCCESS) 
 ******************************************************************************/

unsigned int tca_i2s_deinit(void *pADMADAIBaseAddr)
{
    //----- 1. Stop the I2S clocks -----
    tca_i2s_stop(pADMADAIBaseAddr, 0);
    tca_i2s_stop(pADMADAIBaseAddr, 1);

    return 0/*SUCCESS*/;
}


/*****************************************************************************
 * Function Name : tca_i2s_start()
 ******************************************************************************
 * Desription    :
 * Parameter     : 
 * Return        : success(SUCCESS) 
 ******************************************************************************/
void tca_i2s_start(void *pADMADAIBaseAddr, unsigned int nMode)
{
    ADMADAI *pStrADMADAIBaseReg = (ADMADAI *)pADMADAIBaseAddr;

    if(nMode) //input
    {
	//ADMADAI Receiver enable
	pStrADMADAIBaseReg->DAMR |= Hw13;
    }
    else
    {
	//ADMADAI Transmitter enable
	pStrADMADAIBaseReg->DAMR |= Hw14;
    }


}

/*****************************************************************************
 * Function Name : tca_i2s_stop()
 ******************************************************************************
 * Desription    :
 * Parameter     : 
 * Return        : success(SUCCESS) 
 ******************************************************************************/
void tca_i2s_stop(void *pADMADAIBaseAddr, unsigned int nMode)
{
    ADMADAI *pStrADMADAIBaseReg = (ADMADAI *)pADMADAIBaseAddr;

    if(nMode)
    {
	//DAI Receiver disable
	pStrADMADAIBaseReg->DAMR &= ~Hw13;
    }
    else
    {
	//DAI Transmitter disable
	pStrADMADAIBaseReg->DAMR &= ~Hw14;
    }

}

/*****************************************************************************
 * Function Name : tca_i2s_initinput()
 ******************************************************************************
 * Desription    :
 * Parameter     : 
 * Return        : success(SUCCESS) 
 ******************************************************************************/
void tca_i2s_initinput(void *pADMABaseAddr, unsigned int nInputDma, unsigned int AUDIO_DMA_IN_PAGE_SIZE, unsigned int SAMPLERATE)
{
    ADMA *pStrADMABaseReg = (ADMA *)pADMABaseAddr;	
    unsigned int bitSet=0x0;
    //unsigned uSize;
    unsigned int DMABuffer = 0;

    //DMA Channel enable of DAI Rx
    pStrADMABaseReg->ChCtrl &= ~Hw30;//

    DMABuffer = 0xFFFFFF00 / (((AUDIO_DMA_IN_PAGE_SIZE*2) >> 4)<<8);
    DMABuffer = DMABuffer * (((AUDIO_DMA_IN_PAGE_SIZE*2) >> 4)<<8);

    //uSize = AUDIO_DMA_IN_PAGE_SIZE*2;

    // DAI Rx (Right) Source Address
    pStrADMABaseReg->RxDaParam = DMABuffer | 4; //(4|0xFFFFFE00);	

    // ADMA Count Reginster setting
    pStrADMABaseReg->RxDaTCnt = (((AUDIO_DMA_IN_PAGE_SIZE)>>0x02)>>0x02) - 1;

    //Rx Transfer Control Refister Setting 
    bitSet = ( HwZERO
	    //Rx Trigger Type is edge-trigger
	    |Hw29 //DMA transfer begins from current source/destination address
	    //|Hw26 //Issue Locked Transfer of DAI Rx DMA
	    |Hw18 //Enable repeat mode control of DAI Rx DMA
	    |Hw13 //Burst size of DAI Rx DMA is 4 cycle
	    |(Hw5|Hw4) //Word size of DAI Rx DMA is 32bit
	    );

    pStrADMABaseReg->TransCtrl |= bitSet;
#if defined(_WM8581_INCLUDE_)		// CONFIG_SND_SOC_TCC_MULTICHANNEL
    pStrADMABaseReg->RptCtrl = Hw26|Hw25|Hw24;//DAI Buffer Threshod is set Multi-Port
#else
    pStrADMABaseReg->RptCtrl = 0;
#endif

    /* ADMA Repeat register setting */
    //BITCLR(HwRptCtrl,HwRptCtrl_DRI_EN); //DMA Interrupt is occurred when the end of each Repeated DMAoperation.

    // Rx channel Control Register Setting	
    //Start DMA
    bitSet = ( HwZERO
	    |Hw2//Interrupt Enable of DAI Rx	
	    );

    pStrADMABaseReg->ChCtrl |= bitSet; 
}

#if 0 //for debug
void tca_adma_dump(void *pADMABaseAddr)
{
    ADMA *pADMAReg = (ADMA *)pADMABaseAddr;
#if 0
	printk("RxDaDar[0x%08x]\n", pADMAReg->RxDaDar);
	printk("RxDaParam[0x%08x]\n", pADMAReg->RxDaParam);
	printk("RxDaTCnt[0x%08x]\n", pADMAReg->RxDaTCnt);
	printk("RxDaCdar[0x%08x]\n", pADMAReg->RxDaCdar);
#endif
	printk("------I2S TX register------\n");
	printk("TxDaSar[0x%08x]\n", pADMAReg->TxDaSar);
	printk("TxDaParam[0x%08x]\n", pADMAReg->TxDaParam);
	printk("TxDaTCnt[0x%08x]\n", pADMAReg->TxDaTCnt);
	printk("TxDaCsar[0x%08x]\n", pADMAReg->TxDaCsar);
	printk("------SPDIF TX register------\n");
	printk("TxSpSar[0x%08x]\n", pADMAReg->TxSpSar);
	printk("TxSpParam[0x%08x]\n", pADMAReg->TxSpParam);
	printk("TxSpTCnt[0x%08x]\n", pADMAReg->TxSpTCnt);
	printk("TxSpCsar[0x%08x]\n", pADMAReg->TxSpCsar);
	printk("------ADMA CTL register------\n");
	printk("RptCtrl[0x%08x]\n", pADMAReg->RptCtrl);
	printk("TransCtrl[0x%08x]\n", pADMAReg->TransCtrl);
	printk("ChCtrl[0x%08x]\n", pADMAReg->ChCtrl);
	printk("IntStatus[0x%08x]\n", pADMAReg->IntStatus);
}

void tca_dai_dump(void *pDAIBaseAddr)
{
    ADMADAI *pDAIReg = (ADMADAI *)pDAIBaseAddr;
	printk("DAMR[0x%08x]\n", pDAIReg->DAMR);
	printk("DAVC[0x%08x]\n", pDAIReg->DAVC);
	printk("MCCR0[0x%08x]\n", pDAIReg->MCCR0);
	printk("MCCR1[0x%08x]\n", pDAIReg->MCCR1);
}

void tca_spdif_dump(void *pSPDIFBaseAddr)
{
    ADMASPDIFTX *pSPDIFReg = (ADMASPDIFTX *)pSPDIFBaseAddr;
	printk("TxVersion[0x%08x]\n", pSPDIFReg->TxVersion);
	printk("TxConfig[0x%08x]\n", pSPDIFReg->TxConfig);
	printk("TxChStat[0x%08x]\n", pSPDIFReg->TxChStat);
	printk("TxIntMask[0x%08x]\n", pSPDIFReg->TxIntMask);
	printk("TxIntStat[0x%08x]\n", pSPDIFReg->TxIntStat);
	printk("DMACFG[0x%08x]\n", pSPDIFReg->DMACFG);
}
#endif
