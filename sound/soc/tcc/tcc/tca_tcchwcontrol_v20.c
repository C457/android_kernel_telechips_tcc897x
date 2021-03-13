
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

#include <asm/mach-types.h>

#include "tca_audio_hw.h"
#include "tca_tcchwcontrol.h"


/* System revision Info */
extern unsigned int system_rev; 

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

		| Hw15	//DAI Enable
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
    unsigned uSize;
    unsigned int DMABuffer = 0;

    //DMA Channel enable of DAI Tx
    pStrADMABaseReg->ChCtrl &= ~Hw28;

    DMABuffer = 0xFFFFFF00 / (((AUDIO_DMA_PAGE_SIZE*2) >> 4)<<8);
    DMABuffer = DMABuffer * (((AUDIO_DMA_PAGE_SIZE*2) >> 4)<<8);

    uSize = AUDIO_DMA_PAGE_SIZE*2;

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
    unsigned uSize;
    unsigned int DMABuffer = 0;

    //DMA Channel enable of DAI Rx
    pStrADMABaseReg->ChCtrl &= ~Hw30;//

    DMABuffer = 0xFFFFFF00 / (((AUDIO_DMA_IN_PAGE_SIZE*2) >> 4)<<8);
    DMABuffer = DMABuffer * (((AUDIO_DMA_IN_PAGE_SIZE*2) >> 4)<<8);

    uSize = AUDIO_DMA_IN_PAGE_SIZE*2;

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

#if defined(CONFIG_ARCH_TCC898X)
/*****************************************************************************
 * Function Name : tca_dai_port_mux()
 ******************************************************************************
 * Desription    : DAI GPIO port mux for TCC898x
 * Parameter     : 3b'xx1-CH0 / 3b'x10-CH1 / 3b'100-CH2 / 3b'000-CH3 /
 * Return        : success(SUCCESS) 
 ******************************************************************************/
void tca_i2s_port_mux(int id, int port)
{
	PAIFCHSEL pAIFCHSEL = (PAIFCHSEL)tcc_p2v(BASE_ADDR_CHSEL);
	unsigned int ret = 0;
	ret = 0x1 << port;
	ret = ret & (0x7);
//	printk("i2s id: %d, port: %d, ret:0x%04x\n", id, port, ret);	
	if(id == 0){
		pAIFCHSEL->DAI0CH_SEL = ret;
	}else{
		pAIFCHSEL->DAI1CH_SEL = ret;
	}
}

void tca_spdif_port_mux(int id, int port)
{
	PAIFCHSEL pAIFCHSEL = (PAIFCHSEL)tcc_p2v(BASE_ADDR_CHSEL);
	unsigned int ret = 0;
	ret = 0x1 << port;
	ret = ret & (0x7);
//	printk("spdif id: %d, port: %d, ret:0x%04x\n", id, port, ret);	
	if(id == 0){
		pAIFCHSEL->SPDIF0CH_SEL = ret;
	}else{
		pAIFCHSEL->SPDIF1CH_SEL = ret;
	}
}
#elif defined(CONFIG_ARCH_TCC802X)
enum{
	DAI_I2S_TYPE=0,
	DAI_SPDIF_TYPE,
	DAI_CDIF_TYPE
};
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

void tca_gpio_alphabet_find(char port_num)
{
	static int gpio_alphabet[]={0, 17, 47, 77, 99, 131, 163, 183, 195, 206, 238};
	unsigned int base_addr = BASE_ADDR_GPIOA;
	int i=0;
	if((port_num > 237)&&(port_num != 255))
        printk("[%s] ERROR! Invalid input [pin_num:%d]\n", __func__, port_num);
	else{
		for(i=0; i < (sizeof(gpio_alphabet)-1); i++){
			if((port_num >= gpio_alphabet[i])&&(port_num < gpio_alphabet[i+1])){
				tca_gpio_func(base_addr, port_num-gpio_alphabet[i], 0xf);
#if 0	//for debug
				if((base_addr&0xFFF) <= 0x180)
					printk("GPIO_%c_%d[pin_num:%d]\n", ('A'+i), port_num-gpio_alphabet[i], port_num);
				else if((base_addr&0xFFF) == 0x240)
					printk("GPIO_SD0_%d[pin_num:%d]\n", port_num-gpio_alphabet[i], port_num);
				else if((base_addr&0xFFF) == 0x400)
					printk("GPIO_H_%d[pin_num:%d]\n", port_num-gpio_alphabet[i], port_num);
				else if((base_addr&0xFFF) == 0x440)
					printk("GPIO_K_%d[pin_num:%d]\n", port_num-gpio_alphabet[i], port_num);
#endif
				break;
			}
			base_addr=base_addr+0x40;
			if(((base_addr&0xFFF) > 0x180)&&((base_addr&0xFFF) < 0x280))base_addr=BASE_ADDR_GPIOSD0;
			else if(((base_addr&0xFFF) >= 0x280)&&((base_addr&0xFFF) < 0x400))base_addr=BASE_ADDR_GPIOH;
		}
	}
}
void tca_audio_gpio_port_set(int type, void *port)	
{
	struct audio_i2s_port *i2s_port = NULL;
	char *no_i2s_port = NULL; //for SPDIF, CDIF
    int i=0, port_num=0;
	if(type == DAI_I2S_TYPE){
		i2s_port = (struct audio_i2s_port *)port;
	}else if(type == DAI_SPDIF_TYPE){
		no_i2s_port = (char *)port;
		port_num = 2;
	}else if(type == DAI_CDIF_TYPE){
		no_i2s_port = (char *)port;
		port_num = 3;
	}

	if(type == DAI_I2S_TYPE){
		for(i=0; i<3; i++){
			tca_gpio_alphabet_find(i2s_port->clk[i]);
		}
		for(i=0; i<4; i++){
			tca_gpio_alphabet_find(i2s_port->daout[i]);
			tca_gpio_alphabet_find(i2s_port->dain[i]);
		}
	}else{
		for(i=0; i<port_num; i++){
			tca_gpio_alphabet_find(no_i2s_port[i]);
		}
	}
}
/*****************************************************************************
 * Function Name : tca_dai_port_mux()
 ******************************************************************************
 * Desription    : DAI GPIO port mux for TCC802x
 * Parameter     : Audio ID, GPIO port Number
 * Return        :  
 ******************************************************************************/
void tca_i2s_port_mux(int id, struct audio_i2s_port *port)
{
	PAPCFG pAPCFG = NULL;
	struct audio_i2s_port *i2s_port = port;
	if(id == 0) pAPCFG = (PAPCFG)tcc_p2v(BASE_ADDR_A0_PCFG);
	else if(id == 1) pAPCFG = (PAPCFG)tcc_p2v(BASE_ADDR_A1_PCFG);
	else if(id == 2) pAPCFG = (PAPCFG)tcc_p2v(BASE_ADDR_A2_PCFG);
	else if(id == 3) pAPCFG = (PAPCFG)tcc_p2v(BASE_ADDR_A3_PCFG);
	else
		printk("ERR! Invalid input id=%d!!\n", id);

	tca_audio_gpio_port_set(DAI_I2S_TYPE, i2s_port);	
	pAPCFG->PCFG0 = (i2s_port->dain[0]|(i2s_port->dain[1]<<8)|(i2s_port->dain[2]<<16)|(i2s_port->dain[3]<<24));
					/* DAI_DI0 | DAI_DI1 | DAI_DI2 | DAI_DI3 */
	pAPCFG->PCFG1 = (i2s_port->clk[1]|(i2s_port->clk[2]<<8)|(i2s_port->daout[0]<<16)|(i2s_port->daout[1]<<24));
					/* DAI_BCLK | DAI_LRCK | DAI_DO0 | DAI_DO1 */

	pAPCFG->PCFG2 = pAPCFG->PCFG2 & 0xFF000000; //FOR CDDI

	pAPCFG->PCFG2 = i2s_port->daout[2]|(i2s_port->daout[3]<<8)|(i2s_port->clk[0]<<16)|pAPCFG->PCFG2;
					/* DAI_DO2 | DAI_DO3 | DAI_MCLK */
/* For Debug
	printk("%dth PCFG0=0x%08x\n", id, (unsigned int)pAPCFG->PCFG0);
	printk("%dth PCFG1=0x%08x\n", id, (unsigned int)pAPCFG->PCFG1);
	printk("%dth PCFG2=0x%08x\n", id, (unsigned int)pAPCFG->PCFG2);
*/
}

void tca_spdif_port_mux(int id, char *port)
{
	PAPCFG pAPCFG = NULL;
	char *spdif_port = NULL;
	spdif_port=port;
	if(id == 0) pAPCFG = (PAPCFG)tcc_p2v(BASE_ADDR_A0_PCFG);
	else if(id == 1) pAPCFG = (PAPCFG)tcc_p2v(BASE_ADDR_A1_PCFG);
	else if(id == 2) pAPCFG = (PAPCFG)tcc_p2v(BASE_ADDR_A2_PCFG);
	else if(id == 3) pAPCFG = (PAPCFG)tcc_p2v(BASE_ADDR_A3_PCFG);
	else
		printk("ERR! Invalid input id=%d!!\n", id);
	tca_audio_gpio_port_set(DAI_SPDIF_TYPE, spdif_port);	
	pAPCFG->PCFG3=pAPCFG->PCFG3 & 0xFFFF; //FOR CD_BCLK, CD_LRCK

	pAPCFG->PCFG3 = (spdif_port[0] << 24) | (spdif_port[1] << 16) | pAPCFG->PCFG3;
					/* SPDIF_TX | SPDIF_RX | CD_LRCK | CD_BCLK */
/* For Debug
	printk("%dth PCFG3=0x%08x\n", id, (unsigned int)pAPCFG->PCFG3);
*/
}
#endif
#if 1 //for debug
typedef struct iobus_config {
	volatile unsigned int HCLKEN0;
	volatile unsigned int HCLKEN1;
	volatile unsigned int HCLKEN2;
	volatile unsigned int HRSTEN0;
	volatile unsigned int HRSTEN1;
	volatile unsigned int HRSTEN2;
} IO_CFG, *PIO_CFG;
#define BASE_ADDR_IOCFG		(0x16051000)

void tca_audio_reset(int index)
{
	PIO_CFG pIO_CONFIG = (PIO_CFG)tcc_p2v(BASE_ADDR_IOCFG);
	if(index == 0) {
		BITCLR(pIO_CONFIG->HRSTEN0, Hw22|Hw21); 
		mdelay(10);	
		BITSET(pIO_CONFIG->HRSTEN0, Hw22|Hw21);	
	}else if(index == 1) {
		BITCLR(pIO_CONFIG->HRSTEN0, Hw26|Hw25);	
		mdelay(10);	
		BITSET(pIO_CONFIG->HRSTEN0, Hw26|Hw25);	
	}else printk("invalid input[%d]\n", index);
}

void tca_iobus_dump(void)
{
	PIO_CFG pIO_CONFIG = (PIO_CFG)tcc_p2v(BASE_ADDR_IOCFG);
	printk("HCLKEN0[0x%08x]\n", pIO_CONFIG->HCLKEN0);
	printk("HCLKEN1[0x%08x]\n", pIO_CONFIG->HCLKEN1);
	printk("HCLKEN2[0x%08x]\n", pIO_CONFIG->HCLKEN2);
	printk("HRSTEN0[0x%08x]\n", pIO_CONFIG->HRSTEN0);
	printk("HRSTEN1[0x%08x]\n", pIO_CONFIG->HRSTEN1);
	printk("HRSTEN2[0x%08x]\n", pIO_CONFIG->HRSTEN2);
}

void tca_adma_dump(void *pADMABaseAddr)
{
    ADMA *pADMAReg = (ADMA *)pADMABaseAddr;

	printk("------I2S RX register------\n");
	printk("RxDaDar[0x%08x]\n", pADMAReg->RxDaDar);
	printk("RxDaParam[0x%08x]\n", pADMAReg->RxDaParam);
	printk("RxDaTCnt[0x%08x]\n", pADMAReg->RxDaTCnt);
	printk("RxDaCdar[0x%08x]\n", pADMAReg->RxDaCdar);
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
	printk("------DAI register------\n");
	printk("DAMR[0x%08x]\n", pDAIReg->DAMR);
	printk("DAVC[0x%08x]\n", pDAIReg->DAVC);
	printk("MCCR0[0x%08x]\n", pDAIReg->MCCR0);
	printk("MCCR1[0x%08x]\n", pDAIReg->MCCR1);
}

void tca_spdif_dump(void *pSPDIFBaseAddr)
{
    ADMASPDIFTX *pSPDIFReg = (ADMASPDIFTX *)pSPDIFBaseAddr;
	printk("------SPDIF register------\n");
	printk("TxVersion[0x%08x]\n", pSPDIFReg->TxVersion);
	printk("TxConfig[0x%08x]\n", pSPDIFReg->TxConfig);
	printk("TxChStat[0x%08x]\n", pSPDIFReg->TxChStat);
	printk("TxIntMask[0x%08x]\n", pSPDIFReg->TxIntMask);
	printk("TxIntStat[0x%08x]\n", pSPDIFReg->TxIntStat);
	printk("DMACFG[0x%08x]\n", pSPDIFReg->DMACFG);
}
#endif
