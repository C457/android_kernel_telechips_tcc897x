/****************************************************************************
 *   FileName    : tca_sc.c
 *   Description :
 ****************************************************************************
 *
 *   TCC Version 1.0
 *   Copyright (c) Telechips, Inc.
 *   ALL RIGHTS RESERVED
 *
 ****************************************************************************/

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

#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>

#include <linux/timer.h>
#include <linux/delay.h>

#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/div64.h>
#include <asm/mach/map.h>

#include <mach/bsp.h>
#include <mach/gpio.h>
#include <mach/iomap.h>
#include <mach/tcc_sc_ioctl.h>
#include <mach/tca_sc.h>

/****************************************************************************
  DEFINITiON
 ****************************************************************************/
#ifdef CONFIG_SUPPORT_TCC_NSK
#define SUPPORT_FLOW_CONTROL
#define SUPPORT_PWM_CLK
#endif

#ifdef STATE_H
#undef STATE_H
#endif
#define STATE_H					1

#ifdef STATE_L
#undef STATE_L
#endif
#define STATE_L					0

#ifdef ENABLE_TX
#undef ENABLE_TX
#endif
#define ENABLE_TX				1
	
#ifdef ENABLE_RX
#undef ENABLE_RX
#endif
#define ENABLE_RX				0

//#define DEBUG_TCA_SC
#ifdef DEBUG_TCA_SC
#undef dprintk
#define dprintk(msg...)			printk("tca_sc: " msg);
#undef eprintk
#define eprintk(msg...)			printk("tca_sc: err: " msg);
#else
#undef dprintk
#define dprintk(msg...)
#undef eprintk
#define eprintk(msg...)			//printk("tca_sc: err: " msg);
#endif

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
 	DRV_UART_SCCLK_FI_372,
	DRV_UART_SCCLK_FI_372,
	DRV_UART_SCCLK_FI_558, 
	DRV_UART_SCCLK_FI_744,
	DRV_UART_SCCLK_FI_1116,
	DRV_UART_SCCLK_FI_1488,
	DRV_UART_SCCLK_FI_1860,
	DRV_UART_SCCLK_FACTOR_RFU,
	DRV_UART_SCCLK_FACTOR_RFU,
	DRV_UART_SCCLK_FI_512,
	DRV_UART_SCCLK_FI_768,
	DRV_UART_SCCLK_FI_1024,
	DRV_UART_SCCLK_FI_1536,
	DRV_UART_SCCLK_FI_2048, 
	DRV_UART_SCCLK_FACTOR_RFU,
	DRV_UART_SCCLK_FACTOR_RFU
};

static unsigned gSCClk[16] =
{
	DRV_UART_SCCLK_FMAX_4MHz,
	DRV_UART_SCCLK_FMAX_5MHz,
	DRV_UART_SCCLK_FMAX_6MHz,
	DRV_UART_SCCLK_FMAX_8MHz,
	DRV_UART_SCCLK_FMAX_12MHz,
	DRV_UART_SCCLK_FMAX_16MHz,
	DRV_UART_SCCLK_FMAX_20MHz,
	DRV_UART_SCCLK_FMAX_RFU,
	DRV_UART_SCCLK_FMAX_RFU,
	DRV_UART_SCCLK_FMAX_5MHz,
	DRV_UART_SCCLK_FMAX_7MHz,
	DRV_UART_SCCLK_FMAX_10MHz,
	DRV_UART_SCCLK_FMAX_15MHz,
	DRV_UART_SCCLK_FMAX_20MHz,
	DRV_UART_SCCLK_FMAX_RFU,
	DRV_UART_SCCLK_FMAX_RFU
};

static sDRV_UART_SC_REG gDRV_UART_SC_REG = 
{
	NULL
#ifdef CONFIG_SUPPORT_TCC_NSK
	, DRV_UART_CH7
#else
	, DRV_UART_CH4
#endif
	, 0									// RX Interrupt Enable
	| DRV_UART_IER_EnableRXI
	, 0									// FCR Options
	| DRV_UART_FCR_TxWindowLvl_01
	| DRV_UART_FCR_TxFifoReset 
	| DRV_UART_FCR_RxFifoReset	
	| DRV_UART_FCR_EnableFifo
	, 0									// LCR Options
#ifdef CONFIG_SUPPORT_TCC_NSK
	| DRV_UART_LCR_EnableOddParity		// PASS
//	| DRV_UART_LCR_EnableEvenParity		// FAIL
	| DRV_UART_LCR_EnableParity	
	| DRV_UART_LCR_WordLength_8Bit
#else
	| DRV_UART_LCR_EnableEvenParity		
	| DRV_UART_LCR_EnableParity	
	| DRV_UART_LCR_OneStopBit 
	| DRV_UART_LCR_WordLength_8Bit
#endif
	, 0 									// SCR Options
//	| DRV_UART_SCR_EnableACK
	, 0 									// AFT Options
	, 0									// Smart Card Control
};

static sDRV_UART_SC_PORT gDRV_UART_SC_PORT = {NULL, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 1};

static sDRV_UART_SC_CLK gDRV_UART_SC_CLK = {NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

static sDRV_UART_SC_BUF gDRV_UART_SC_BUF = {0, 0, {0, }};

static sDRV_UART_SC_ATR gDRV_UART_SC_ATR = {0, TCC_SC_DIRECT_CONVENTION, 0, 0, {0, }, {0, }, {0, }, {0,}, {0, }, 0};

#ifdef SUPPORT_PWM_CLK
static sDRV_UART_PWM_REG gDRV_UART_PWM_REG = 
{
	NULL
	, 0
	, 0
	, 0
	, 0xCCCCCCCC
};

static sDRV_UART_PWM_PORT gDRV_UART_PWM_PORT = {NULL, -1, -1};

static sDRV_UART_PWM_CLK gDRV_UART_PWM_CLK = {NULL, 0};
#endif
	
static int giSCIrq, giSCIrqData;

//---------------------------------------------------------------------------
// DEFINITION OF BASIC GPIO PORT FUNCTIONS
//---------------------------------------------------------------------------
static void tca_sc_init_port(void)
{
	sDRV_UART_SC_PORT *pstPort = &gDRV_UART_SC_PORT;

	if (pstPort->PwrEn != -1) {
		gpio_request (pstPort->PwrEn, NULL);
	}
	if (pstPort->VccSel != -1) {
		gpio_request (pstPort->VccSel, NULL);
	}
	if (pstPort->Rst != -1) {
		gpio_request (pstPort->Rst, NULL);
	}
	if (pstPort->Detect != -1) {
		gpio_request (pstPort->Detect, NULL);
	}
	if (pstPort->Clk != -1) {
		gpio_request (pstPort->Clk, NULL);	
	}
	if (pstPort->DataC7 != -1) {
		gpio_request (pstPort->DataC7, NULL);
	}
#ifdef CONFIG_SUPPORT_TCC_NSK
	if (pstPort->DataC4 != -1) {
		gpio_request (pstPort->DataC4, NULL);
	}
	if (pstPort->DataC8 != -1) {
		gpio_request (pstPort->DataC8, NULL);
	}
#endif
	
	if (pstPort->PwrEn != -1) {
		tcc_gpio_config(pstPort->PwrEn, GPIO_FN(0)|GPIO_OUTPUT|GPIO_HIGH);
		if(pstPort->TDA8024) {
			gpio_direction_output(pstPort->PwrEn, 0);
		}
		else {
			gpio_direction_output(pstPort->PwrEn, 1);
		}
	}	
	if (pstPort->VccSel != -1) {
		tcc_gpio_config(pstPort->VccSel, GPIO_FN(0)|GPIO_OUTPUT|GPIO_HIGH);
		gpio_direction_output(pstPort->VccSel, 1);
	}	
	if (pstPort->Rst != -1) {
		tcc_gpio_config(pstPort->Rst, GPIO_FN(0)|GPIO_INPUT|GPIO_PULLUP);
		gpio_direction_input (pstPort->Rst);
	}
	if (pstPort->Detect != -1) {
		tcc_gpio_config(pstPort->Detect, GPIO_FN(0)|GPIO_INPUT); 
	}
	if (pstPort->Clk != -1) {
		tcc_gpio_config(pstPort->Clk, GPIO_FN(0)|GPIO_INPUT|GPIO_PULLUP);
	}
	if (pstPort->DataC7 != -1) {
		tcc_gpio_config(pstPort->DataC7, GPIO_FN(0)|GPIO_INPUT|GPIO_PULLUP);
	}
#ifdef CONFIG_SUPPORT_TCC_NSK
	if (pstPort->DataC4 != -1) {
		tcc_gpio_config(pstPort->DataC4, GPIO_FN(0)|GPIO_INPUT|GPIO_PULLUP);
	}
	if (pstPort->DataC8 != -1) {
		tcc_gpio_config(pstPort->DataC8, GPIO_FN(0)|GPIO_INPUT|GPIO_PULLUP);
	}
#endif

	return;
}

static void tca_sc_set_port_config(unsigned uEnable)
{
#ifdef SUPPORT_PWM_CLK
	sDRV_UART_PWM_PORT *pstPWMPort = &gDRV_UART_PWM_PORT;
#endif
	sDRV_UART_SC_PORT *pstPort = &gDRV_UART_SC_PORT;
	sDRV_UART_SC_REG *pstReg = &gDRV_UART_SC_REG;
	unsigned uData;

	dprintk("%s(%d, %d, %d)\n", __func__, pstReg->CH, pstPort->PortNum, pstPort->PinMask);

	if(uEnable) {	
		if (pstReg->CH < 4) {
			uData = sc_readl(pstPort->pBaseAddr+OFFSET_PCFG0);
			uData &= ~(0xFF << (pstReg->CH * 8));
			uData = pstPort->PortNum << (pstReg->CH * 8);
			sc_writel(uData, pstPort->pBaseAddr+OFFSET_PCFG0);
		}
		else if (pstReg->CH < 8) {
		        uData = sc_readl(pstPort->pBaseAddr+OFFSET_PCFG1);
		        uData &= ~(0xFF << ((pstReg->CH - 4) * 8));
		        uData = pstPort->PortNum << ((pstReg->CH - 4) * 8);
		        sc_writel(uData, pstPort->pBaseAddr+OFFSET_PCFG1);
		}

#ifdef SUPPORT_PWM_CLK
		tcc_gpio_config(pstPWMPort->Clk, GPIO_FN(pstPWMPort->Fn));
#else
		tcc_gpio_config(pstPort->Clk, GPIO_FN(pstPort->Fn));		// TX
#endif

		if(pstPort->PinMask & TCC_SC_PIN_MASK_IO_C7_ON) {
			tcc_gpio_config(pstPort->DataC7, GPIO_FN(pstPort->Fn));	 // RX
		}
		else if(pstPort->PinMask & TCC_SC_PIN_MASK_C7_ON) {
			tcc_gpio_config(pstPort->DataC7, GPIO_FN(0)|GPIO_OUTPUT|GPIO_HIGH);
		}
		else{
			tcc_gpio_config(pstPort->DataC7, GPIO_FN(0)|GPIO_INPUT|GPIO_PULLUP);
		}

#ifdef CONFIG_SUPPORT_TCC_NSK
		if(pstPort->PinMask & TCC_SC_PIN_MASK_IO_C4_ON) {
			tcc_gpio_config(pstPort->DataC4, GPIO_FN(pstPort->FnC4));	 // RX
		}
		else if(pstPort->PinMask & TCC_SC_PIN_MASK_C4_ON) {
			tcc_gpio_config(pstPort->DataC4, GPIO_FN(0)|GPIO_OUTPUT|GPIO_HIGH);
		}
		else {
			tcc_gpio_config(pstPort->DataC4, GPIO_FN(0)|GPIO_INPUT|GPIO_PULLUP);
		}

		if(pstPort->PinMask & TCC_SC_PIN_MASK_IO_C8_ON) {
			tcc_gpio_config(pstPort->DataC8, GPIO_FN(pstPort->FnC8));	 // RX
		}
		else if(pstPort->PinMask & TCC_SC_PIN_MASK_C8_ON) {
			tcc_gpio_config(pstPort->DataC8, GPIO_FN(0)|GPIO_OUTPUT|GPIO_HIGH);
		}
		else {
			tcc_gpio_config(pstPort->DataC8, GPIO_FN(0)|GPIO_INPUT|GPIO_PULLUP);
		}
#endif
	}
	else {
		if (pstPort->Clk != -1) {
			tcc_gpio_config(pstPort->Clk, GPIO_FN(0)|GPIO_INPUT|GPIO_PULLUP);
		}
		if (pstPort->Rst != -1) {
			tcc_gpio_config(pstPort->Rst, GPIO_FN(0)|GPIO_INPUT|GPIO_PULLUP);
		}
		if (pstPort->DataC7 != -1) {
			tcc_gpio_config(pstPort->DataC7, GPIO_FN(0)|GPIO_INPUT|GPIO_PULLUP);
		}
#ifdef CONFIG_SUPPORT_TCC_NSK
		if (pstPort->DataC4 != -1) {
			tcc_gpio_config(pstPort->DataC4, GPIO_FN(0)|GPIO_INPUT|GPIO_PULLUP);
		}
		if (pstPort->DataC8 != -1) {
			tcc_gpio_config(pstPort->DataC8, GPIO_FN(0)|GPIO_INPUT|GPIO_PULLUP);
		}
#endif
	}

	return;
}

#ifdef CONFIG_SUPPORT_TCC_NSK
static void tca_sc_change_io_port_config(unsigned uPinMask)
{
	sDRV_UART_SC_PORT *pstPort = &gDRV_UART_SC_PORT;

	if(uPinMask & TCC_SC_PIN_MASK_IO_C7_ON) {
		tcc_gpio_config(pstPort->DataC7, GPIO_FN(pstPort->Fn));	 // RX
	}
	else if(uPinMask & TCC_SC_PIN_MASK_C7_ON) {
		tcc_gpio_config(pstPort->DataC7, GPIO_FN(0)|GPIO_OUTPUT|GPIO_HIGH);
	}
	else{
		tcc_gpio_config(pstPort->DataC7, GPIO_FN(0)|GPIO_OUTPUT|GPIO_LOW);
	}

	if(uPinMask & TCC_SC_PIN_MASK_IO_C4_ON) {
		tcc_gpio_config(pstPort->DataC4, GPIO_FN(pstPort->FnC4));	 // RX
	}
	else if(uPinMask & TCC_SC_PIN_MASK_C4_ON) {
		tcc_gpio_config(pstPort->DataC4, GPIO_FN(0)|GPIO_OUTPUT|GPIO_HIGH);
	}
	else {
		tcc_gpio_config(pstPort->DataC4, GPIO_FN(0)|GPIO_OUTPUT|GPIO_LOW);
	}

	if(uPinMask & TCC_SC_PIN_MASK_IO_C8_ON) {
		tcc_gpio_config(pstPort->DataC8, GPIO_FN(pstPort->FnC8));	 // RX
	}
	else if(uPinMask & TCC_SC_PIN_MASK_C8_ON) {
		tcc_gpio_config(pstPort->DataC8, GPIO_FN(0)|GPIO_OUTPUT|GPIO_HIGH);
	}
	else {
		tcc_gpio_config(pstPort->DataC8, GPIO_FN(0)|GPIO_OUTPUT|GPIO_LOW);
	}

	pstPort->PinMask = uPinMask;

	return;
}
#endif

//---------------------------------------------------------------------------
// DEFINITION OF BASIC CLOCK FUNCTIONS
//---------------------------------------------------------------------------

static void tca_sc_set_peripheral_clock(unsigned uEnable)
{
#ifdef SUPPORT_PWM_CLK
	sDRV_UART_PWM_CLK *pstPWMClk = &gDRV_UART_PWM_CLK;
#endif
	sDRV_UART_SC_CLK *pstClk = &gDRV_UART_SC_CLK;

	if(uEnable) {
#ifdef SUPPORT_PWM_CLK
		clk_set_rate(pstPWMClk->pPClk, pstPWMClk->Fpwm);

		clk_prepare_enable(pstPWMClk->pPClk);
#endif

		clk_set_rate(pstClk->pHClk, pstClk->Fuart);

		clk_prepare_enable(pstClk->pPClk);
		clk_prepare_enable(pstClk->pHClk);

	}
	else {
#ifdef SUPPORT_PWM_CLK
		clk_disable_unprepare(pstPWMClk->pPClk);
#endif

		clk_disable_unprepare(pstClk->pPClk);
		clk_disable_unprepare(pstClk->pHClk);
	}

	return;
}

#ifdef SUPPORT_PWM_CLK
static void tca_sc_change_peripheral_clock(void)
{
	sDRV_UART_PWM_CLK *pstPWMClk = &gDRV_UART_PWM_CLK;
	sDRV_UART_SC_CLK *pstClk = &gDRV_UART_SC_CLK;

	if(pstClk->Fout == DRV_UART_SCCLK_FMAX_5MHz) {
		pstPWMClk->Fpwm = DRV_UART_PWMCLK_FPWM_36MHz;
	}
	else if(pstClk->Fout == DRV_UART_SCCLK_FMAX_8MHz) {
		pstPWMClk->Fpwm = DRV_UART_PWMCLK_FPWM_54MHz;
	}
	else {
		pstPWMClk->Fpwm = DRV_UART_PWMCLK_FPWM_109MHz;
	}

	// Change PWM clock
	clk_set_rate(pstPWMClk->pPClk, pstPWMClk->Fpwm);
	
	pstClk->Baud = (pstClk->Fout * pstClk->Di) / pstClk->Fi;
	pstClk->Fuart = pstClk->Baud * 16;

	// Change UART clock
	clk_disable(pstClk->pPClk);
	clk_disable(pstClk->pHClk);

	clk_set_rate(pstClk->pHClk, pstClk->Fuart);

	clk_enable(pstClk->pPClk);
	clk_enable(pstClk->pHClk);
}
#endif

//---------------------------------------------------------------------------
// DEFINITION OF BASIC SIGNAL FUNCTIONS
//---------------------------------------------------------------------------	
static void tca_sc_set_vcc(unsigned uState)
{
	int iTDA8024 = gDRV_UART_SC_PORT.TDA8024;
	int iPwrEn = gDRV_UART_SC_PORT.PwrEn;
	int iState = 0;

	dprintk("%s(%d)\n", __func__, uState);
	
	if (iPwrEn == -1) {
		eprintk("%s(-1)\n", __func__);
		return;
	}

	if(iTDA8024) {
		iState = (uState == STATE_H) ? STATE_L : STATE_H;
	}
	else {
		iState = uState;
	}

	gpio_direction_output (iPwrEn, iState);
	gpio_set_value_cansleep (iPwrEn, iState);

	return;
}

static void tca_sc_set_reset(unsigned uState)
{
	int iRst = gDRV_UART_SC_PORT.Rst;

	dprintk("%s(%d, %d)\n", __func__, iRst, uState);	

	if (iRst == -1) {
		eprintk("%s(-1)\n", __func__);
		return;
	}
	
	gpio_direction_output (iRst, uState);
	gpio_set_value_cansleep (iRst, uState);

	return;
}

static void tca_sc_set_io_interface(unsigned uEnable)
{
	sDRV_UART_SC_REG *pstReg = &gDRV_UART_SC_REG;
	unsigned uData;

	dprintk("%s(%d)\n", __func__, uEnable);	

	
	if(uEnable) {
		uData = sc_readl(pstReg->pBaseAddr+OFFSET_SCCR);
		uData |=HwUART_SCCR_SEN_EN;
		sc_writel(uData, pstReg->pBaseAddr+OFFSET_SCCR);
	}
	else {
		uData = sc_readl(pstReg->pBaseAddr+OFFSET_SCCR);
		uData &=~HwUART_SCCR_SEN_EN;
		sc_writel(uData, pstReg->pBaseAddr+OFFSET_SCCR);
	}
	
	return;
}

static void tca_sc_set_io_mode(unsigned uMode)
{
	sDRV_UART_SC_REG *pstReg = &gDRV_UART_SC_REG;
	unsigned uData;

	//dprintk("%s(%d)\n", __func__, uMode);		

	//Direction of IO Port
	uData = sc_readl(pstReg->pBaseAddr+OFFSET_SCCR);
	if(uMode == ENABLE_TX) { 
		if((uData & HwUART_SCCR_STEN_EN) != HwUART_SCCR_STEN_EN) {
			uData |=HwUART_SCCR_STEN_EN; //Tx Enable
			sc_writel(uData, pstReg->pBaseAddr+OFFSET_SCCR);
		}
	}
	else {
		if((uData & HwUART_SCCR_STEN_EN) == HwUART_SCCR_STEN_EN) {
			uData &=~HwUART_SCCR_STEN_EN; //Rx Enable
			sc_writel(uData, pstReg->pBaseAddr+OFFSET_SCCR);
		}
	}

	return;
}

//---------------------------------------------------------------------------
// DEFINITION OF BASIC IO FUNCTIONS
//---------------------------------------------------------------------------
void tca_sc_init_rx_buf(void)
{
	sDRV_UART_SC_BUF *pstBuf = &gDRV_UART_SC_BUF;

	pstBuf->usRxHead = pstBuf->usRxTail = 0;
	memset(pstBuf->pRxBuf, 0x00, DRV_UART_SC_RX_BUF_SIZE);

	return;
}

unsigned char tca_sc_ts_pattern_check(char cData)
{
	sDRV_UART_SC_ATR *pstATR = &gDRV_UART_SC_ATR;

	if(pstATR->uiConvention == TCC_SC_INVERSE_CONVENTION) {
		return gSCInvertTable[(int)cData];
	}

	return cData;
}

void tca_sc_rx_push_char(unsigned char ucChar)
{
	sDRV_UART_SC_BUF *pstBuf = &gDRV_UART_SC_BUF;

	pstBuf->pRxBuf[pstBuf->usRxHead++] = ucChar;
	pstBuf->usRxHead %= DRV_UART_SC_RX_BUF_SIZE;

	return;
}

unsigned tca_sc_rx_pop_char(void)
{
	sDRV_UART_SC_BUF *pstBuf = &gDRV_UART_SC_BUF;
	unsigned char ucData;

	if (pstBuf->usRxHead != pstBuf->usRxTail) {
		ucData = pstBuf->pRxBuf[pstBuf->usRxTail++];
		pstBuf->usRxTail %= DRV_UART_SC_RX_BUF_SIZE;
		
		return tca_sc_ts_pattern_check(ucData);
	}

	return 0x100;
}

void tca_sc_rx_print(void)
{
	sDRV_UART_SC_BUF *pstBuf = &gDRV_UART_SC_BUF;
	unsigned char *p = pstBuf->pRxBuf;
	int i;

	dprintk("Head:%d, Tail:%d\n", pstBuf->usRxHead, pstBuf->usRxTail);

	for(i=0; i<100; i+=10)
	{
		dprintk("%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X\n", \
				tca_sc_ts_pattern_check(p[i]), tca_sc_ts_pattern_check(p[i+1]), tca_sc_ts_pattern_check(p[i+2]), tca_sc_ts_pattern_check(p[i+3]), 
				tca_sc_ts_pattern_check(p[i+4]), tca_sc_ts_pattern_check(p[i+5]), tca_sc_ts_pattern_check(p[i+6]), tca_sc_ts_pattern_check(p[i+7]),  
				tca_sc_ts_pattern_check(p[i+8]), tca_sc_ts_pattern_check(p[i+9]));
	}
}

int tca_sc_send_byte(unsigned uChar, unsigned uLen)
{ 
	sDRV_UART_SC_REG *pstReg = &gDRV_UART_SC_REG;
	sDRV_UART_SC_CLK *pstClk = &gDRV_UART_SC_CLK;
	sDRV_UART_SC_ATR *pstATR = &gDRV_UART_SC_ATR;
	unsigned uData;

	//Wait util THR Empty
	do{
		uData = sc_readl(pstReg->pBaseAddr+OFFSET_LSR); 	
	} while(ISZERO(uData, HwUART_LSR_TEMT_ON));
	
	//Send a Byte
	uData = tca_sc_ts_pattern_check(uChar);
	sc_writel(uData, pstReg->pBaseAddr+OFFSET_THR);

	if(pstATR->uiProtocol == 0) {
		if(uLen > 1)
			udelay(pstClk->GT);
		else
			udelay((12 * pstClk->ETU) / 1000);

#ifdef SUPPORT_FLOW_CONTROL
		uData = sc_readl(pstReg->pBaseAddr+OFFSET_SCR);
		if(ISZERO(uData, HwUART_SCR_ACK)) {
			//eprintk("ACK faile... Resend[%d]\n", uData);
			return TCC_SC_ERROR_NACK;
		}
		else {
			while(ISZERO(uData, HwUART_SCR_RXD_ST_OFF))
			{
				udelay(pstClk->ETU/1000);
				uData = sc_readl(pstReg->pBaseAddr+OFFSET_SCR);
			}
		}
#endif
	}

	return TCC_SC_SUCCESS;
}

//---------------------------------------------------------------------------
// DEFINITION OF DETECT FUNCTION OF  SMART CARD 
//---------------------------------------------------------------------------
unsigned tca_sc_detect_card(void)
{
	sDRV_UART_SC_PORT *pstPort = &gDRV_UART_SC_PORT;
	int iPresence = 0;

//	dprintk("%s\n", __func__);

	gpio_set_value_cansleep (pstPort->Detect, 1);
	iPresence = gpio_get_value_cansleep(pstPort->Detect);

//	dprintk("%s(%d)\n", __func__, iPresence);

	return iPresence;
}

irqreturn_t tca_sc_interrupt_handler(int irq, void *dev_id)
{
	sDRV_UART_SC_REG *pstReg = &gDRV_UART_SC_REG;
	sDRV_UART_SC_PORT *pstPort = &gDRV_UART_SC_PORT;
	unsigned uData;

	uData = sc_readl(pstPort->pBaseAddr+OFFSET_ISTS);
	if(ISZERO(uData, (1<<pstReg->CH))) {
		return IRQ_NONE;
	}

	uData = sc_readl(pstReg->pBaseAddr+OFFSET_SCCR);
	if((uData & HwUART_SCCR_STEN_EN) == HwUART_SCCR_STEN_EN) {
		eprintk("Invalid Interrupt(%08X)\n", uData);
		return IRQ_NONE;
	}

	//Wait until Data is Ready
	do {
		uData = sc_readl(pstReg->pBaseAddr+OFFSET_LSR);
	} while(ISZERO(uData, HwUART_LSR_DR_ON));
	
	//Pop Data from 16byte FIFO
	uData = sc_readl(pstReg->pBaseAddr+OFFSET_RBR);

	//Push Data to UART Buffer
	tca_sc_rx_push_char((unsigned char)uData);

	return IRQ_HANDLED;
}

//---------------------------------------------------------------------------
// DEFINITION OF CONFIGURATION FUNCTION
//---------------------------------------------------------------------------
#ifdef SUPPORT_PWM_CLK
static void tca_sc_config_sc(void)
{
	sDRV_UART_SC_REG *pstReg = &gDRV_UART_SC_REG;
	unsigned uData;

	dprintk("%s \n", __func__);
	
	//Disable SmartCard Interface
	uData = sc_readl(pstReg->pBaseAddr+OFFSET_SCCR);
	uData &= ~HwUART_SCCR_SEN_EN;
	sc_writel(uData, pstReg->pBaseAddr+OFFSET_SCCR);
	
	//Set SmartCard Interface Configuration
	uData = sc_readl(pstReg->pBaseAddr+OFFSET_SCCR);
	uData &= ~(HwUART_SCCR_STEN_EN | HwUART_SCCR_STE_EN);
	uData |= HwUART_SCCR_DEN_EN;
	sc_writel(uData, pstReg->pBaseAddr+OFFSET_SCCR);

	//set SmartCard Clock Divider
	uData = sc_readl(pstReg->pBaseAddr+OFFSET_SCCR);
	uData &= ~HwUART_SCCR_DIV_MASK;
	sc_writel(uData, pstReg->pBaseAddr+OFFSET_SCCR);

	//Set Start TX Count
	uData = sc_readl(pstReg->pBaseAddr+OFFSET_STC);
	uData &= ~HwUART_STC_SCNT_MASK;
	uData |= HwUART_STC_SCNT(0);
	sc_writel(uData, pstReg->pBaseAddr+OFFSET_STC);

	uData = HwUART_LCR_DLAB_ON;                              // DLAB = 1
	sc_writel(uData, pstReg->pBaseAddr+OFFSET_LCR);

	uData = 1;
	sc_writel(uData, pstReg->pBaseAddr+OFFSET_DLL);

	uData = 0;
	sc_writel(uData, pstReg->pBaseAddr+OFFSET_DLM);

	uData =  pstReg->LCR;                                  // DLAB = 0
	sc_writel(uData, pstReg->pBaseAddr+OFFSET_LCR);

	uData = pstReg->FCR;
	sc_writel(uData, pstReg->pBaseAddr+OFFSET_FCR);
	
	uData = pstReg->SCR;
	sc_writel(uData, pstReg->pBaseAddr+OFFSET_SCR);

	if (pstReg->AFT) {
		uData = (HwUART_MCR_AFE_EN | HwUART_MCR_RTS);
		sc_writel(uData, pstReg->pBaseAddr+OFFSET_MCR);

		uData = pstReg->AFT;
		sc_writel(uData, pstReg->pBaseAddr+OFFSET_AFT);
	}
	else {
		uData = 0;
		sc_writel(uData, pstReg->pBaseAddr+OFFSET_MCR);
		sc_writel(uData, pstReg->pBaseAddr+OFFSET_AFT);
	}

	uData = pstReg->IER;
	sc_writel(uData, pstReg->pBaseAddr+OFFSET_IER);

	//Set the Interrupt Handler	
	//request_irq(INT_UART0+pstReg->CH, tca_sc_interrupt_handler, IRQF_SHARED, "tca_sc", NULl);

	//Enable SmartCard Interface
	uData = sc_readl(pstReg->pBaseAddr+OFFSET_SCCR);
	uData |=HwUART_SCCR_SEN_EN;
	sc_writel(uData, pstReg->pBaseAddr+OFFSET_SCCR);

	msleep(5);

	return;
}
#else
static void tca_sc_config_sc(void)
{
	sDRV_UART_SC_REG *pstReg = &gDRV_UART_SC_REG;
	sDRV_UART_SC_CLK *pstClk = &gDRV_UART_SC_CLK;
	unsigned uSCClk, uSCClkDiv, uSCClkPDiv, uDiv;
	unsigned uData;

	dprintk("%s \n", __func__);
	
	//Get Divder Value
	uData = sc_readl(pstReg->pBaseAddr+OFFSET_SCCR);
 	uSCClkPDiv  = (uData & HwUART_SCCR_P_EN)? 2:1;
	
	//Get SmartCard Clock Divisor
	uSCClkDiv = (pstClk->Fuart / (pstClk->Fout * uSCClkPDiv)) - 1;

	//Set SmartCard Clock Divisor
	uData = sc_readl(pstReg->pBaseAddr+OFFSET_SCCR);
	uData &= ~HwUART_SCCR_DIV_MASK;
	uData |= HwUART_SCCR_DIV(uSCClkDiv);
	sc_writel(uData, pstReg->pBaseAddr+OFFSET_SCCR);
	
	//Set Start TX Count
	uData = sc_readl(pstReg->pBaseAddr+OFFSET_STC);
	uData &= ~HwUART_STC_SCNT_MASK;
	uData |=  HwUART_STC_SCNT(0);
	sc_writel(uData, pstReg->pBaseAddr+OFFSET_STC);

	//Calc SmartCard  Actual Clock (Hz)
	uSCClk = (pstClk->Fuart) / ((uSCClkDiv + 1) * uSCClkPDiv);
	
	//Calc Baud Rate
	pstClk->Baud = (pstClk->Fout * pstClk->Di) / pstClk->Fi;
	
	//Get the Divisor Latch Value
	uDiv = pstClk->Fuart / (16 * pstClk->Baud);
	
	//Set divisor register to generate desired baud rate clock
	uData = sc_readl(pstReg->pBaseAddr+OFFSET_LCR);
	uData |= HwUART_LCR_DLAB_ON;                              // DLAB = 1 to enable access DLL/DLM
	sc_writel(uData, pstReg->pBaseAddr+OFFSET_LCR);

	uData = sc_readl(pstReg->pBaseAddr+OFFSET_DLL);
	uData =uDiv;
	sc_writel(uData, pstReg->pBaseAddr+OFFSET_DLL);

	uData = sc_readl(pstReg->pBaseAddr+OFFSET_DLM);
	uData =uDiv >> 8;
	sc_writel(uData, pstReg->pBaseAddr+OFFSET_DLM);

	uData = sc_readl(pstReg->pBaseAddr+OFFSET_LCR);
	uData &= HwUART_LCR_DLAB_OFF;                            // DLAB = 0 to disable access DLL/DLM
	sc_writel(uData, pstReg->pBaseAddr+OFFSET_LCR);

	dprintk("%s(%d, %d, %d, %d)\n", __func__, uSCClk, uSCClkDiv, uSCClkPDiv, uDiv);

	msleep(5);

	return;
}
#endif

//---------------------------------------------------------------------------
// DEFINITION OF ATR PARGSING FUNCTION
//---------------------------------------------------------------------------
static void tca_sc_calc_factor_T0(unsigned short usTA1, unsigned short usTC1, unsigned short usTC2)
{
	sDRV_UART_SC_CLK *pstClk = &gDRV_UART_SC_CLK;
	unsigned uHighNibble, uLowNibble;
	
	dprintk("%s \n", __func__);

	//Calculate Fout, Fi, Di

	if(usTA1 != 0xFFFF) {
		uHighNibble = (usTA1 & 0xF0) >> 4;
		uLowNibble = (usTA1 & 0x0F);

		pstClk->Fout = gSCClk[uHighNibble];
		pstClk->Fi = gSCClkFactor[uHighNibble];

		if(uLowNibble > 0 && uLowNibble < 8) {
			pstClk->Di = 1<<(uLowNibble-1);
		}
		else if(uLowNibble == 8) {
			pstClk->Di = 12;
		}
		else if(uLowNibble == 9) {
			pstClk->Di = 20;
		}
		else if(uLowNibble == 15) {
			pstClk->Di = 93;
		}
		else {
			pstClk->Di = 4;
		}
	}
	
	//Calculate N
	if(usTC1 != 0xFFFF) {
		pstClk->N = usTC1;
	}
	else {
		pstClk->N = 0;
	}

	//Calculate WI
	if(usTC2 != 0xFFFF) {
		pstClk->WI = usTC2;
	}
	else {
		pstClk->WI = 10;
	}

	//Calculate Baud	
	pstClk->Baud = (pstClk->Fout * pstClk->Di) / pstClk->Fi;

	//Calculate GT, WT
	pstClk->ETU = (pstClk->Fi * 1000000) / (pstClk->Di * (pstClk->Fout / 1000));	//nsec
	pstClk->GT = ((12 * pstClk->ETU) + (pstClk->N * pstClk->ETU)) / 1000;			//usec
	pstClk->WT = (pstClk->WI * 960 * pstClk->Fi) / (pstClk->Fout / 1000);				//msec

#if 1
	dprintk("%s(Fi:%d, Di:%d, N:%d)\n", __func__, pstClk->Fi, pstClk->Di, pstClk->N);
	dprintk("%s(Fuart:%d, Fout:%d, Baud:%d)\n", __func__, pstClk->Fuart, pstClk->Fout, pstClk->Baud);
	dprintk("%s(ETU:%d, GT:%d, WT:%d)\n", __func__, pstClk->ETU, pstClk->GT, pstClk->WT);
#endif

	return;
}

static void tca_sc_calc_factor_T1(unsigned short usTA1, unsigned short usTC1, unsigned short usTB3)
{
	sDRV_UART_SC_CLK *pstClk = &gDRV_UART_SC_CLK;
	unsigned uHighNibble, uLowNibble;
	int iCWI=1, iBWI=1;
	int i;
	
	dprintk("%s \n", __func__);

	//Calculate Fout, Fi, Di
	if(usTA1 != 0xFFFF) {
		uHighNibble = (usTA1 & 0xF0) >> 4;
		uLowNibble = (usTA1 & 0x0F);

		pstClk->Fout = gSCClk[uHighNibble];
		pstClk->Fi = gSCClkFactor[uHighNibble];

		if(uLowNibble > 0 && uLowNibble < 8) {
			pstClk->Di = 1<<(uLowNibble-1);
		}
		else if(uLowNibble == 8) {
			pstClk->Di = 12;
		}
		else if(uLowNibble == 9) {
			pstClk->Di = 20;
		}
		else {
			pstClk->Di = 4;
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

	pstClk->Baud = pstClk->Fout / (pstClk->Fi / pstClk->Di);
	pstClk->ETU = (pstClk->Fi * 1000000) / (pstClk->Di * (pstClk->Fout / 1000));	// nsec

	for(i=0; i<uLowNibble; i++)
	{
		iCWI *= 2;
	}
	
	for(i=0; i<uHighNibble; i++)
	{
		iBWI *= 2;
	}	
	
	pstClk->CWT = ((iCWI * pstClk->ETU) + (11 * pstClk->ETU)) / 1000;		// usec
	pstClk->BWT = (((iBWI * 960 * 372 * 10000) / (pstClk->Fout / 100000)) + (11 * pstClk->ETU)) / 1000;	// usec

#if 1
	dprintk("%s(Fi:%d, Di:%d, N:%d, iCWI:%d, iBWI:%d)\n", __func__, pstClk->Fi, pstClk->Di, pstClk->N, iCWI, iBWI);
	dprintk("%s(Fuart:%d, Fout:%d, Baud:%d)\n", __func__, pstClk->Fuart, pstClk->Fout, pstClk->Baud);
	dprintk("%s(ETU:%d, GT:%d, WT:%d)\n", __func__, pstClk->ETU, pstClk->GT, pstClk->WT);
	dprintk("%s(CWT:%d, BWT:%d)\n", __func__, pstClk->CWT, pstClk->BWT);
#endif

	return;
}

static void tca_sc_init_factor(void)
{
#ifdef SUPPORT_PWM_CLK
	sDRV_UART_PWM_CLK *pstPWMClk = &gDRV_UART_PWM_CLK;
#endif
	sDRV_UART_SC_CLK *pstClk = &gDRV_UART_SC_CLK;

#ifdef SUPPORT_PWM_CLK
	//Change Default Peripheral Clock
	pstClk->Fuart = DRV_UART_PWMCLK_FUART_193KHz;
	pstPWMClk->Fpwm = DRV_UART_PWMCLK_FPWM_36MHz;
#endif

	//372 clock cycle
	pstClk->Fi = DRV_UART_SCCLK_FI_372;
	pstClk->Di = 1;
	pstClk->N = 0;
	pstClk->WI = 10;
	
	pstClk->Fout = DRV_UART_SCCLK_FMAX_5MHz;
	pstClk->Baud = pstClk->Fout / (pstClk->Fi / pstClk->Di);

	pstClk->GT = 0;
	pstClk->WT = 0;
	pstClk->CWT = 0;
	pstClk->BWT = 0;

	return;
}

//---------------------------------------------------------------------------
// DEFINITION OF ATR FUNCTION
//---------------------------------------------------------------------------
static void tca_sc_receive_atr(unsigned char *pATR, unsigned *puATRLen)
{
	unsigned char  *pBuf = pATR;
	unsigned uBufLen = 0;
	unsigned uTimeOut = 10; //30;
	unsigned uTimeCount = 10; //30;
	unsigned uData = 0;

	dprintk("%s\n", __func__);
	
	//Enable Rx Mode
	tca_sc_set_io_mode(ENABLE_RX);

	while (uTimeOut > 0)
	{
		uData = tca_sc_rx_pop_char();
		if(uData < 0x100) {
			*pBuf++ = uData;
			uBufLen++;
			uTimeOut = uTimeCount;
		}
		else {
			msleep(5);
			uTimeOut--;
		}
	}
	pATR[uBufLen] = 0;

	//Set the ATR Length
	*puATRLen = uBufLen;

	dprintk("ATR: %d Bytes (0x%02X:0x%02X:0x%02X:0x%02X:0x%02X:0x%02X:0x%02X ...)\n", *puATRLen, pATR[0], pATR[1], pATR[2], pATR[3], pATR[4], pATR[5], pATR[6]);

	return;
}

static int tca_sc_parse_atr(unsigned char *pATR, unsigned uATRLen)
{
	sDRV_UART_SC_ATR *pstATR = &gDRV_UART_SC_ATR;
	unsigned uHCharNum = 0;
	unsigned uBufLen = 0;
	int i;
	
	dprintk("%s\n", __func__);

	//Check ATR Direction 
	pstATR->usTS = *(pATR + uBufLen++);
	if(pstATR->usTS == TCC_SC_DIRECT_CONVENTION) {
		pstATR->uiConvention = TCC_SC_DIRECT_CONVENTION;
	}
	else if(pstATR->usTS == 0x03) {
		pstATR->uiConvention = TCC_SC_INVERSE_CONVENTION;

		for(i=0; i<uATRLen; i++)
		{
			pATR[i] =  tca_sc_ts_pattern_check(pATR[i]);
		}
		pstATR->usTS = *pATR;
	}
	else  {
		eprintk("%s(0x%04X)\n", __func__, pstATR->usTS);
		return TCC_SC_ERROR_INVALID_ATR;
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
		eprintk("%s(%d, %d)\n", __func__, uATRLen, uBufLen);
		memset(pstATR, 0xFF, sizeof(sDRV_UART_SC_ATR));
		
		return TCC_SC_ERROR_INVALID_ATR;
	}

#if 0
	dprintk("TS:0x%04x\n", pstATR->usTS);
	dprintk("TO:0x%04x\n", pstATR->usT0);
	dprintk("TA1:0x%04x, TB1:0x%04x, TC1:0x%04x, TD1:0x%04x\n", pstATR->usTA[0], pstATR->usTB[0], pstATR->usTC[0], pstATR->usTD[0]);
	dprintk("TA2:0x%04x, TB2:0x%04x, TC2:0x%04x, TD2:0x%04x\n", pstATR->usTA[1], pstATR->usTB[1], pstATR->usTC[1], pstATR->usTD[1]);
	dprintk("TA3:0x%04x, TB3:0x%04x, TC3:0x%04x, TD3:0x%04x\n", pstATR->usTA[2], pstATR->usTB[2], pstATR->usTC[2], pstATR->usTD[2]);
	dprintk("TA4:0x%04x, TB4:0x%04x, TC4:0x%04x, TD4:0x%04x\n", pstATR->usTA[3], pstATR->usTB[3], pstATR->usTC[3], pstATR->usTD[3]);
	dprintk("Historical Charaters : %s\n", pstATR->ucHC);
	dprintk("TCK:0x%02x\n", pstATR->ucTCK);
#endif		

	if(pstATR->uiProtocol == 0) {
		tca_sc_calc_factor_T0(pstATR->usTA[0], pstATR->usTC[0], pstATR->usTC[1]);
	}
	else if(pstATR->uiProtocol == 1) {
		tca_sc_calc_factor_T1(pstATR->usTA[0], pstATR->usTC[0], pstATR->usTB[2]);
	}
	else {
		eprintk("%s(%d)\n", __func__, pstATR->uiProtocol);
		return TCC_SC_ERROR_INVALID_ATR;
	}

	return TCC_SC_SUCCESS;
}

//---------------------------------------------------------------------------
// DEFINITION OF IOCTL OPERATING FUNCTIONS
//---------------------------------------------------------------------------
int tca_sc_enable(void)
{
#ifdef SUPPORT_PWM_CLK
	sDRV_UART_PWM_REG *pstPWMReg = &gDRV_UART_PWM_REG;
	unsigned long flags;
#else
	sDRV_UART_SC_REG *pstReg = &gDRV_UART_SC_REG;
#endif
	unsigned uData;

	dprintk("%s\n", __func__);	

	//Set Default ATR Factor
	tca_sc_init_factor();

	//Enable UART Peripheral Clock
	tca_sc_set_peripheral_clock(ENABLE);
	
	//Set UART Port Configuration
	tca_sc_set_port_config(ENABLE);
	
#ifdef SUPPORT_PWM_CLK
	//Set PWM Block Configuration
	local_irq_save(flags);

	//uData = sc_readl(pstPWMReg->pBaseAddr + OFFSET_PWMEN);
	uData = 0;
	sc_writel(uData, pstPWMReg->pBaseAddr + OFFSET_PWMEN);

	uData = sc_readl(pstPWMReg->pBaseAddr + OFFSET_PWMMODE);
	uData &= DRV_PWM_MODE_CLR(pstPWMReg->CH);
	uData |= DRV_PWM_MODE_SET(pstPWMReg->CH, HwPWM_MODE_REG_OUT_MODE);
	sc_writel(uData, pstPWMReg->pBaseAddr + OFFSET_PWMMODE);

	sc_writel(pstPWMReg->PWMOUTN1, pstPWMReg->pBaseAddr + OFFSET_PWMOUTN1);
	//sc_writel(0x0, pstPWMReg->pBaseAddr + OFFSET_PWMOUTN2);
	//sc_writel(0x0, pstPWMReg->pBaseAddr + OFFSET_PWMOUTN3);
	//sc_writel(0x0, pstPWMReg->pBaseAddr + OFFSET_PWMOUTN4);

	uData = sc_readl(pstPWMReg->pBaseAddr + OFFSET_PWMEN);
	uData |= (DRV_PWM_EN_Trigger(pstPWMReg->CH) | DRV_PWM_EN_Enable(pstPWMReg->CH));
	sc_writel(uData, pstPWMReg->pBaseAddr + OFFSET_PWMEN);

	local_irq_restore(flags);

	//Sert SmartCard Configuration
	tca_sc_config_sc();
#else
	//Disable SmartCard Interface
	uData = sc_readl(pstReg->pBaseAddr+OFFSET_SCCR);
	uData &= ~HwUART_SCCR_SEN_EN;
	sc_writel(uData, pstReg->pBaseAddr+OFFSET_SCCR);	

	//Sert SmartCard Configuration
	uData = sc_readl(pstReg->pBaseAddr+OFFSET_SCCR);
	uData &= ~(HwUART_SCCR_STEN_EN | HwUART_SCCR_STE_EN);
	uData |= (HwUART_SCCR_DEN_EN|HwUART_SCCR_P_EN);
	sc_writel(uData, pstReg->pBaseAddr+OFFSET_SCCR);

	//Sert SmartCard Configuration
	tca_sc_config_sc();
	
	uData =  pstReg->LCR;
	sc_writel(uData, pstReg->pBaseAddr+OFFSET_LCR);

	uData = pstReg->FCR;
	sc_writel(uData, pstReg->pBaseAddr+OFFSET_FCR);
	
	uData = pstReg->SCR;
	sc_writel(uData, pstReg->pBaseAddr+OFFSET_SCR);

	if (pstReg->AFT) {
		uData = HwUART_MCR_AFE_EN | HwUART_MCR_RTS;
		sc_writel(uData, pstReg->pBaseAddr+OFFSET_MCR);

		uData = pstReg->AFT;
		sc_writel(uData, pstReg->pBaseAddr+OFFSET_AFT);
	}
	else {
		uData = 0;
		sc_writel(uData, pstReg->pBaseAddr+OFFSET_MCR);
		sc_writel(uData, pstReg->pBaseAddr+OFFSET_AFT);
	}

	uData = pstReg->IER;
	sc_writel(uData, pstReg->pBaseAddr+OFFSET_IER);

	//Set the Interrupt Handler	
	//request_irq(INT_UART0+pstReg->CH, tca_sc_interrupt_handler, IRQF_SHARED, "tca_sc", NULl);
	
	//Enable SmartCard Interface
	uData = sc_readl(pstReg->pBaseAddr+OFFSET_SCCR);
	uData |=HwUART_SCCR_SEN_EN;
	sc_writel(uData, pstReg->pBaseAddr+OFFSET_SCCR);
#endif 
	
	return TCC_SC_SUCCESS;
}

int tca_sc_disable(void)
{
	sDRV_UART_SC_REG *pstReg = &gDRV_UART_SC_REG;
        unsigned uData;

	dprintk("%s\n", __func__);

	//Disable SmartCard Interface
	uData = sc_readl(pstReg->pBaseAddr+OFFSET_SCCR);
	uData &=~HwUART_SCCR_SEN_EN;
	sc_writel(uData, pstReg->pBaseAddr+OFFSET_SCCR);

	//DRV_UART_FUNC_SC_CLOSE
	//free_irq(INT_UART0+pstReg->CH, NULL);

	tca_sc_set_port_config(DISABLE);

	//Disable UART Peripheral Clock
	tca_sc_set_peripheral_clock(DISABLE);
	
	return TCC_SC_SUCCESS;
}

void tca_sc_activate(void)
{
	dprintk("%s\n", __func__);

	tca_sc_set_reset(STATE_L);

	//Wait 400/f
	//usleep_range(200, 250);
	msleep(60);
	
	tca_sc_set_vcc(STATE_H);
	tca_sc_set_io_interface(ENABLE);
	tca_sc_set_io_mode(ENABLE_RX);
		
	tca_sc_set_reset(STATE_H);

	msleep(60);

	return;
}

void tca_sc_deactivate(void)
{
	dprintk("%s\n", __func__);
	
	tca_sc_set_reset(STATE_L);
	tca_sc_set_io_interface(DISABLE);
	tca_sc_set_vcc(STATE_L);

	return;
}

int tca_sc_select_vcc_level(unsigned iLevel)
{
	sDRV_UART_SC_PORT *pstPort = &gDRV_UART_SC_PORT;

	dprintk("%s(%d)\n", __func__, iLevel);

	if(iLevel == TCC_SC_VOLTAGE_LEVEL_3V){
		gpio_direction_output(pstPort->VccSel, 0);
		gpio_set_value(pstPort->VccSel, 0);
	}
	else {
		gpio_direction_output(pstPort->VccSel, 1);
		gpio_set_value(pstPort->VccSel, 1);
	}
	
	return TCC_SC_SUCCESS;
}

int tca_sc_warm_reset(unsigned char *pATR, unsigned *puATRLen)
{
	sDRV_UART_SC_ATR *pstATR = &gDRV_UART_SC_ATR;
	int iRetry;
	int iRet;

	//Set Default ATR Factor
	tca_sc_init_factor();

#ifdef SUPPORT_PWM_CLK
	tca_sc_change_peripheral_clock();
#endif

	//Set Configuration
	tca_sc_config_sc();
	
	//Init Rx Buffer
	tca_sc_init_rx_buf();

	//Set Default Value
	memset(pstATR, 0xFF, sizeof(sDRV_UART_SC_ATR));
	pstATR->uiConvention = TCC_SC_DIRECT_CONVENTION;

	for(iRetry=3; iRetry > 0; iRetry--)
	{
		tca_sc_set_reset(STATE_L);
	
		//Wait 400/f
		msleep(10);
	
		tca_sc_set_reset(STATE_H);

		tca_sc_receive_atr(pATR, puATRLen);
		if(*puATRLen > 0) {
			iRet = tca_sc_parse_atr(pATR, *puATRLen);
			if(iRet != TCC_SC_SUCCESS) {
				eprintk("%s(%d)\n", __func__, iRet);
				return TCC_SC_ERROR_INVALID_ATR;
			}

#ifdef SUPPORT_PWM_CLK
			tca_sc_change_peripheral_clock();
#endif

			tca_sc_config_sc();

			return TCC_SC_SUCCESS;
		}
	}

	eprintk("%s(ATRLen:0)\n", __func__);

	return TCC_SC_ERROR_INVALID_ATR;
}

void tca_sc_set_config(stTCC_SC_CONFIG stSCConfig)
{
	sDRV_UART_SC_CLK *pstClk = &gDRV_UART_SC_CLK;

	dprintk("%s(GT:%d, WT:%d)\n", __func__, stSCConfig.uiGuardTime, stSCConfig.uiWaitTime);

	pstClk->N = stSCConfig.uiGuardTime;
	pstClk->WT = stSCConfig.uiWaitTime; //msec

	//Calculate GT
	pstClk->ETU = (pstClk->Fi * 1000000) / (pstClk->Di * (pstClk->Fout / 1000)); //nsec
	pstClk->GT = ((12 * pstClk->ETU) + (pstClk->N * pstClk->ETU)) / 1000; //usec

	dprintk("%s: GT:%d, WT:%d\n", __func__, pstClk->GT, pstClk->WT);

	return;
}

void tca_sc_get_config(stTCC_SC_CONFIG *pstSCConfig)
{
	sDRV_UART_SC_CLK *pstClk = &gDRV_UART_SC_CLK;
	sDRV_UART_SC_ATR *pstATR = &gDRV_UART_SC_ATR;

	pstSCConfig->uiProtocol = pstATR->uiProtocol;

	pstSCConfig->uiClock = pstClk->Fout;
	pstSCConfig->uiBaudrate = pstClk->Fi/pstClk->Di;
	pstSCConfig->uiGuardTime = pstClk->N;
	pstSCConfig->uiWaitTime = pstClk->WT;

	return;
}

void tca_sc_send_data(unsigned char *pData, unsigned uDataLen)
{
	sDRV_UART_SC_REG *pstReg = &gDRV_UART_SC_REG;
	unsigned char *pBuf = pData;
	unsigned uBufLen = uDataLen;
	unsigned uData;
	int iRet;

	//dprintk("%s(%d)\n", __func__, uDataLen);
	
	//Check the Length of Data
	if(uBufLen > 0) {
		//Set SmartCard TX Count
		uData = sc_readl(pstReg->pBaseAddr+OFFSET_SCR);
		uData &=~Hw7;
		sc_writel(uData, pstReg->pBaseAddr+OFFSET_SCR);

		uData = sc_readl(pstReg->pBaseAddr+OFFSET_STC);
		uData &=~HwUART_STC_SCNT_MASK;
		uData |= HwUART_STC_SCNT(uDataLen);
		sc_writel(uData, pstReg->pBaseAddr+OFFSET_STC);

		//Set SmartCard TX
		tca_sc_set_io_mode(ENABLE_TX);

		while(uBufLen)
		{
			iRet = tca_sc_send_byte(*pBuf, uBufLen);			
			if(iRet == TCC_SC_SUCCESS) {
				pBuf++;
				uBufLen--;
			}
		}
	}

	return;
}

int tca_sc_receive_variable_data(char *pData, unsigned *puDataLen)
{
	sDRV_UART_SC_CLK *pstClk = &gDRV_UART_SC_CLK;
	unsigned uCWTmsec = (pstClk->CWT/1000) + 1;
	unsigned char *pBuf = pData;
	unsigned uBufLen = 0;
	unsigned uTimeOut 	= 200;
	unsigned uTimeCount	= 200;
	unsigned uData = 0;

	//dprintk("%s(%d)\n", __func__, uTimeOut);
	
	uTimeOut = uCWTmsec;
	uTimeCount = uCWTmsec;

	//Set SmartCard RX
	tca_sc_set_io_mode(ENABLE_RX);

	//Receive Data and uDataLen from SmartCard 
	while (uTimeOut > 0)
	{
		uData = tca_sc_rx_pop_char();
		if(uData < 0x100) {
			*pBuf++ = uData;
			uBufLen++;
			
			uTimeOut = uTimeCount;
		}
		else {
			msleep(uCWTmsec);
			uTimeOut--;
		}
	}

	*puDataLen = uBufLen;

	if(*puDataLen <= 0) {
		eprintk("%s: timeout\n", __func__);
		return TCC_SC_ERROR_TIMEOUT;
	}

	return TCC_SC_SUCCESS;
}

int tca_sc_send_receive_len(char *pTxBuf, unsigned uTxBufLen, char *pRxBuf, unsigned *puRxBufLen)
{
	sDRV_UART_SC_CLK *pstClk = &gDRV_UART_SC_CLK;
	int iBWTmsec = (pstClk->BWT/1000) + 1;
	int iRet;
	
	dprintk("%s(%d, %d)\n", __func__, uTxBufLen, *puRxBufLen);

	//Init Rx Buffer
	tca_sc_init_rx_buf();

	// Send Command
	if(pTxBuf && uTxBufLen) {
		tca_sc_send_data(pTxBuf, uTxBufLen);
	}

	msleep(iBWTmsec);
					
	// Receive Response
	if(pRxBuf && puRxBufLen) {
		iRet = tca_sc_receive_variable_data(pRxBuf, puRxBufLen);
		if(iRet) {
			eprintk("%s: timeout\n", __func__);
			return TCC_SC_ERROR_TIMEOUT;
		}
	}

	//tca_sc_rx_print();

	return TCC_SC_SUCCESS;
}

//---------------------------------------------------------------------------
// DEFINITION OF IOCTL OPERATING FUNCTIONS FOR NDS
//---------------------------------------------------------------------------
#define NDS_HEADER_LENGTH		5
#define NDS_STATUS_LENGTH 		2

int tca_sc_send_data_for_nds(unsigned char *pData, unsigned uDataLen, struct timeval stTime)
{
	sDRV_UART_SC_REG *pstReg = &gDRV_UART_SC_REG;
	unsigned char *pBuf = pData;
	unsigned uBufLen = uDataLen;
	struct timeval stCurTime;
	long lDiff;
 	unsigned uData;
	int iRet = TCC_SC_SUCCESS;

	//dprintk("%s(%d)\n", __func__, uDataLen);
	
	//Check the Length of Data
	if(uBufLen > 0) {
		uData = sc_readl(pstReg->pBaseAddr+OFFSET_IER);
		uData &= ~HwUART_IER_ERXI_EN;
		sc_writel(uData, pstReg->pBaseAddr+OFFSET_IER);

		//Set SmartCard TX Count
		uData = sc_readl(pstReg->pBaseAddr+OFFSET_SCR);
		uData &=~Hw7;
#ifdef SUPPORT_FLOW_CONTROL
		uData |=HwUART_SCR_ACK_EN;
#endif
		sc_writel(uData, pstReg->pBaseAddr+OFFSET_SCR);

		uData = sc_readl(pstReg->pBaseAddr+OFFSET_STC);
		uData &=~HwUART_STC_SCNT_MASK;
		uData |= HwUART_STC_SCNT(uDataLen);
		sc_writel(uData, pstReg->pBaseAddr+OFFSET_STC);

		//Set SmartCard TX
		tca_sc_set_io_mode(ENABLE_TX);

		while(uBufLen)
		{
/*
			uData = sc_readl(pstReg->pBaseAddr+OFFSET_STC);
			uData &=~HwUART_STC_SCNT_MASK;
			uData |= HwUART_STC_SCNT(1);
			sc_writel(uData, pstReg->pBaseAddr+OFFSET_STC);

			//Set SmartCard TX
			tca_sc_set_io_mode(ENABLE_TX);
*/
			iRet = tca_sc_send_byte(*pBuf, uBufLen);			
			if(iRet == TCC_SC_SUCCESS) {
				pBuf++;
				uBufLen--;
			}

			do_gettimeofday(&stCurTime);
			lDiff = ((stTime.tv_sec * 1000) + (stTime.tv_usec / 1000)) - ((stCurTime.tv_sec * 1000) + (stCurTime.tv_usec / 1000));
			if(lDiff < 0) {
/*
				uData = sc_readl(pstReg->pBaseAddr+OFFSET_STC);
				uData &=~HwUART_STC_SCNT_MASK;
				uData |= HwUART_STC_SCNT(0);
				sc_writel(uData, pstReg->pBaseAddr+OFFSET_STC);

				tca_sc_set_io_mode(ENABLE_RX);
*/
				iRet = TCC_SC_ERROR_TIMEOUT;
				break;
			}
		}

		uData = sc_readl(pstReg->pBaseAddr+OFFSET_SCR);
		uData &=HwUART_SCR_ACK_DIS;
		sc_writel(uData, pstReg->pBaseAddr+OFFSET_SCR);

		uData = pstReg->FCR;
		sc_writel(uData, pstReg->pBaseAddr+OFFSET_FCR);

		uData = pstReg->IER;
		sc_writel(uData, pstReg->pBaseAddr+OFFSET_IER);
	}

	return iRet;
}

int tca_sc_receive_procedure_byte(unsigned uINS, struct timeval stTime)
{
	sDRV_UART_SC_CLK *pstClk = &gDRV_UART_SC_CLK;
	unsigned uData = 0;
	unsigned uINSComp;
	struct timeval stStartTime, stCurTime;
	long lWaitTime = pstClk->WT;
	long lDiff;

	//dprintk("%s(%02X)\n", __func__, ucINS);
	
	uINSComp = ((~uINS) & 0xFF);

	do_gettimeofday(&stStartTime);

	//Set SmartCard RX
	tca_sc_set_io_mode(ENABLE_RX);

	//Receive Data from SmartCard 
	do {
		uData = tca_sc_rx_pop_char();
		if(uData < 0x100 && uData != 0x60) {
			if(uData == uINS) {
				return TCC_SC_SUCCESS;
			}
			else if(uData == uINSComp) {
				return TCC_SC_ERROR_COMPLEMENT;
			}
			else {
				dprintk("<%02X>\n", uData);
				return TCC_SC_ERROR_IO;
			}
		}
		else {
			usleep_range(pstClk->GT, pstClk->GT);
		}

		do_gettimeofday(&stCurTime);

		//Check WaitTime
		lDiff = ((stCurTime.tv_sec * 1000) + (stCurTime.tv_usec / 1000)) - ((stStartTime.tv_sec * 1000) + (stStartTime.tv_usec / 1000));
		if(lDiff > lWaitTime) {
			break;
		}

		//Check TimeOut
		lDiff = ((stTime.tv_sec * 1000) + (stTime.tv_usec / 1000)) - ((stCurTime.tv_sec * 1000) + (stCurTime.tv_usec / 1000));
	} while(lDiff > 0);

	//eprintk("%s: timeout(%d, %ld)\n", __func__, i, lDiff);

	return TCC_SC_ERROR_TIMEOUT;
}

int tca_sc_receive_status_bytes(char *pData, unsigned *puDataLen, struct timeval stTime)
{
	sDRV_UART_SC_CLK *pstClk = &gDRV_UART_SC_CLK;
	unsigned char *pBuf = pData;
	unsigned uBufLen = 0;
	unsigned uData = 0;
	struct timeval stStartTime, stCurTime;
	long lWaitTime = pstClk->WT;
	long lDiff;

	//dprintk("%s(%d, %ld)\n", __func__, *puDataLen, lTimeOut);
	
	do_gettimeofday(&stStartTime);

	//Set SmartCard RX
	tca_sc_set_io_mode(ENABLE_RX);

	//Receive Data from SmartCard 
	do {
		uData = tca_sc_rx_pop_char();
		if(uData < 0x100 && uData != 0x60) {
			*pBuf++ = uData;
			uBufLen++;

			if((pData[0] != 0x90) && (pData[0] < 0x61 || pData[0] > 0x6F)) {
				eprintk("%s: invalid SW1(%02X)\n", __func__, pData[0]);
				*puDataLen += uBufLen;
				return TCC_SC_ERROR_IO;
			}

			if(uBufLen >= NDS_STATUS_LENGTH) {
				*puDataLen += uBufLen;
				return TCC_SC_SUCCESS;
 			}

			do_gettimeofday(&stStartTime);
		}
		else {
			usleep_range(pstClk->GT, pstClk->GT);
		}

		do_gettimeofday(&stCurTime);

		//Check WaitTime
		lDiff = ((stCurTime.tv_sec * 1000) + (stCurTime.tv_usec / 1000)) - ((stStartTime.tv_sec * 1000) + (stStartTime.tv_usec / 1000));
		if(lDiff > lWaitTime) {
			break;
		}

		//Check TimeOut
		lDiff = ((stTime.tv_sec * 1000) + (stTime.tv_usec / 1000)) - ((stCurTime.tv_sec * 1000) + (stCurTime.tv_usec / 1000));
	} while(lDiff > 0);

	*puDataLen += uBufLen;
	//eprintk("%s: timeout(%d)\n", __func__, *puDataLen);

	return TCC_SC_ERROR_TIMEOUT;

}

int tca_sc_receive_fixed_data_for_nds(char *pData, unsigned *puDataLen, struct timeval stTime)
{
	sDRV_UART_SC_CLK *pstClk = &gDRV_UART_SC_CLK;
	unsigned char *pBuf = pData;
	unsigned uBufLen = 0;
	unsigned uData = 0;
	struct timeval stStartTime, stCurTime;
	long lWaitTime = pstClk->WT;
	long lDiff;

	//dprintk("%s(%d, %ld)\n", __func__, *puDataLen, lTimeOut);
	
	do_gettimeofday(&stStartTime);
	
	//Set SmartCard RX
	tca_sc_set_io_mode(ENABLE_RX);

	//Receive Data from SmartCard 
	do {
		uData = tca_sc_rx_pop_char();
		if(uData < 0x100) {
			*pBuf++ = uData;
			if(++uBufLen >= *puDataLen) {
				*puDataLen = uBufLen;
				return TCC_SC_SUCCESS;
 			}

			do_gettimeofday(&stStartTime);
		}
		else {
			usleep_range(pstClk->GT, pstClk->GT);
		}

		do_gettimeofday(&stCurTime);

		//Check WaitTime
		lDiff = ((stCurTime.tv_sec * 1000) + (stCurTime.tv_usec / 1000)) - ((stStartTime.tv_sec * 1000) + (stStartTime.tv_usec / 1000));
		if(lDiff > lWaitTime) {
			break;
		}

		//Check TimeOut
		lDiff = ((stTime.tv_sec * 1000) + (stTime.tv_usec / 1000)) - ((stCurTime.tv_sec * 1000) + (stCurTime.tv_usec / 1000));
	} while(lDiff > 0);

	*puDataLen = uBufLen;
	//eprintk("%s: timeout(%d)\n", __func__, *puDataLen);

	return TCC_SC_ERROR_TIMEOUT;
}

int tca_sc_send_receive_for_nds(char *pTxBuf, unsigned uTxBufLen, char *pRxBuf, unsigned *puRxBufLen, unsigned uDirection, unsigned uTimeOut)
{
	//sDRV_UART_SC_CLK *pstClk = &gDRV_UART_SC_CLK;
	struct timeval stTime;
	long lTimeOut;
	int iDataLen;
	int i;
	int iRet;

	*puRxBufLen = 0;
	
	if(pTxBuf == NULL || pRxBuf == NULL) {
		eprintk("%s: invalid param\n", __func__);
		return TCC_SC_ERROR_INVALID_PARAM;
	}
	
	if(uTxBufLen < NDS_HEADER_LENGTH) {
		eprintk("%s: invalid param\n", __func__);
		return TCC_SC_ERROR_INVALID_PARAM;
	}

	do_gettimeofday(&stTime);

	lTimeOut = stTime.tv_usec + ((uTimeOut * 1000) - (uTimeOut * 10));
	stTime.tv_sec += lTimeOut / 1000000;
	stTime.tv_usec = lTimeOut % 1000000;

	//Init Rx Buffer
	tca_sc_init_rx_buf();

	//Send Header 
	iRet = tca_sc_send_data_for_nds(pTxBuf, NDS_HEADER_LENGTH, stTime);
	if(iRet) {
		//eprintk("%s: timeout cmd(%d, %d)\n", __func__, pstClk->WT, uTimeOut);
		return TCC_SC_ERROR_TIMEOUT;
	}	
	
	//Receive ACK
	iRet = tca_sc_receive_procedure_byte(pTxBuf[1], stTime);
	if(iRet) {
		if(iRet != TCC_SC_ERROR_COMPLEMENT) {
			//eprintk("%s: timeout PB(%d, %d)\n", __func__, pstClk->WT, uTimeOut);
			return TCC_SC_ERROR_TIMEOUT;
		}

		//dprintk("%s: compliment PB\n", __func__);
	}	

	//Send or Receive Data
	if(uDirection == TCC_SC_DIRECTION_TO_CARD) {
		if(uTxBufLen < NDS_HEADER_LENGTH) {
			eprintk("%s: invalid param\n", __func__);
			return TCC_SC_ERROR_INVALID_PARAM;
		}
		else if(uTxBufLen == NDS_HEADER_LENGTH) {
			iRet = tca_sc_receive_status_bytes(pRxBuf, puRxBufLen, stTime);
			if(iRet) {
				//eprintk("%s: timeout SB(%d, %d, %d)\n", __func__, pstClk->WT, uTimeOut, *puRxBufLen);
				return TCC_SC_ERROR_TIMEOUT;
			}

			return TCC_SC_SUCCESS;
		}

		iDataLen = uTxBufLen-NDS_HEADER_LENGTH;
		if(iRet == TCC_SC_ERROR_COMPLEMENT) {
			for(i=0; i<iDataLen; i++)
			{
				iRet = tca_sc_send_data_for_nds(&pTxBuf[NDS_HEADER_LENGTH+i], 1, stTime);
				if(iRet) {
					//eprintk("%s: timeout Send(%d, %d, %d)\n", __func__, pstClk->WT, uTimeOut, *puRxBufLen);
					return TCC_SC_ERROR_TIMEOUT;
				}

				if(i < iDataLen-1) {
					iRet = tca_sc_receive_procedure_byte(pTxBuf[1], stTime);
					if(iRet) {
						if(iRet != TCC_SC_ERROR_COMPLEMENT) {
								//eprintk("%s: timeout PB(%d, %d)\n", __func__, pstClk->WT, uTimeOut);
							return TCC_SC_ERROR_TIMEOUT;
						}

						//dprintk("%s: compliment PB\n", __func__);
					}
				}
			}
		}
		else {
			iRet = tca_sc_send_data_for_nds(&pTxBuf[NDS_HEADER_LENGTH], iDataLen, stTime);
			if(iRet) {
				//eprintk("%s: timeout Send(%d, %d, %d)\n", __func__, pstClk->WT, uTimeOut, *puRxBufLen);
				return TCC_SC_ERROR_TIMEOUT;
			}
		}

		iRet = tca_sc_receive_status_bytes(pRxBuf, puRxBufLen, stTime);
		if(iRet) {
			//eprintk("%s: timeout SB(%d, %d, %d)\n", __func__, pstClk->WT, uTimeOut, *puRxBufLen);
			return TCC_SC_ERROR_TIMEOUT;
		}
	}
	else {
		*puRxBufLen = pTxBuf[NDS_HEADER_LENGTH-1];
		iRet = tca_sc_receive_fixed_data_for_nds(pRxBuf, puRxBufLen, stTime);
		if(iRet) {
			//eprintk("%s: timeout Rcv(%d, %d, %d)\n", __func__, pstClk->WT, uTimeOut, *puRxBufLen);
			return TCC_SC_ERROR_TIMEOUT;
		}

		iRet = tca_sc_receive_status_bytes(&pRxBuf[*puRxBufLen], puRxBufLen, stTime);
		if(iRet) {
			//eprintk("%s: timeout SB(%d, %d, %d)\n", __func__, pstClk->WT, uTimeOut, *puRxBufLen);
			return TCC_SC_ERROR_TIMEOUT;
		}
	}

	dprintk("%s(%d)\n", __func__, *puRxBufLen);

	return TCC_SC_SUCCESS;
}

int tca_sc_set_io_pin_config(unsigned uPinMask)
{
	dprintk("%s(%d)\n", __func__, uPinMask);

#ifdef CONFIG_SUPPORT_TCC_NSK
	tca_sc_change_io_port_config(uPinMask);
#endif

	return TCC_SC_SUCCESS;
}

int tca_sc_get_io_pin_config(unsigned *puPinMask)
{
	sDRV_UART_SC_PORT *pstPort = &gDRV_UART_SC_PORT;
	
	dprintk("%s(%d)\n", __func__, pstPort->PinMask);

	*puPinMask = pstPort->PinMask;

	return TCC_SC_SUCCESS;
}

int tca_sc_open(struct inode *inode, struct file *filp)
{
	unsigned int iRet = 0;
	
	dprintk("%s\n", __func__);

	tca_sc_init_port();

#if 1
	//set the interrupt handler
	iRet = request_irq(INT_UART, tca_sc_interrupt_handler, IRQF_SHARED, "tca_sc", &giSCIrqData);
	//iRet = request_irq(giSCIrq, tca_sc_interrupt_handler, IRQF_SHARED, "tca_sc", &giSCIrqData);
	if(iRet) { 
		eprintk("%s: request_irq(%d, %d)\n", __func__, iRet, giSCIrq);
		free_irq(INT_UART, &giSCIrqData);
		//free_irq(giSCIrq, &giSCIrqData);
		return TCC_SC_ERROR_UNKNOWN;	
	}
#endif

	return 0;
}

void tca_sc_close(struct inode *inode, struct file *file)
{
	dprintk("%s\n", __func__);

	free_irq(INT_UART, &giSCIrqData);
//	free_irq(giSCIrq, &giSCIrqData);

	return;
}

int tca_sc_probe(struct platform_device *pdev)
{
#ifdef SUPPORT_PWM_CLK
	sDRV_UART_PWM_REG *pstPWMReg = &gDRV_UART_PWM_REG;
	sDRV_UART_PWM_PORT *pstPWMPort = &gDRV_UART_PWM_PORT;
	sDRV_UART_PWM_CLK *pstPWMClk = &gDRV_UART_PWM_CLK;
#endif
	sDRV_UART_SC_REG *pstReg = &gDRV_UART_SC_REG;
	sDRV_UART_SC_PORT *pstPort = &gDRV_UART_SC_PORT;
	sDRV_UART_SC_CLK *pstClk = &gDRV_UART_SC_CLK;
	struct device_node *pstUartChNode;

	dprintk("%s\n", __func__);

	of_property_read_u32(pdev->dev.of_node, "clock-frequency", &pstClk->Fuart);
	of_property_read_u32(pdev->dev.of_node, "uart_chnum", &pstReg->CH);
	of_property_read_u32(pdev->dev.of_node, "uart_port", &pstPort->PortNum);
	of_property_read_u32(pdev->dev.of_node, "gpio_fun", &pstPort->Fn);
	of_property_read_u32(pdev->dev.of_node, "tda8024_used", &pstPort->TDA8024);

	pstPort->Rst = of_get_named_gpio(pdev->dev.of_node, "rst_gpios",0);
	pstPort->Detect = of_get_named_gpio(pdev->dev.of_node, "detect_gpios",0);
	pstPort->PwrEn = of_get_named_gpio(pdev->dev.of_node, "pwren_gpios",0);
	pstPort->Clk = of_get_named_gpio(pdev->dev.of_node, "clk_gpios",0);
	pstPort->DataC7 = of_get_named_gpio(pdev->dev.of_node, "data_c7_gpios",0);

#ifdef CONFIG_SUPPORT_TCC_NSK
	of_property_read_u32(pdev->dev.of_node, "gpio_fun_c4", &pstPort->FnC4);
	of_property_read_u32(pdev->dev.of_node, "gpio_fun_c8", &pstPort->FnC8);
	pstPort->VccSel = of_get_named_gpio(pdev->dev.of_node, "vccsel_gpios",0);
	pstPort->DataC4 = of_get_named_gpio(pdev->dev.of_node, "data_c4_gpios",0);
	pstPort->DataC8 = of_get_named_gpio(pdev->dev.of_node, "data_c8_gpios",0);
#endif
	pstPort->PinMask = TCC_SC_PIN_MASK_IO_C7_ON;

	pstReg->pBaseAddr = (void __iomem *)io_p2v(TCC_PA_UART0 + (0x10000 * pstReg->CH));
	pstPort->pBaseAddr = (void __iomem *)io_p2v(TCC_PA_UARTPORTCFG);

	dprintk("%s uart base addr = %08x\n", __func__, (unsigned int)pstReg->pBaseAddr);
	dprintk("%s uart portcfg base addr = %08x\n", __func__, (unsigned int)pstPort->pBaseAddr);

	pstUartChNode = of_parse_phandle(pdev->dev.of_node, "uart_ch", 0);
	giSCIrq = irq_of_parse_and_map(pstUartChNode, 0);
	
	pstClk->pPClk = of_clk_get(pstUartChNode, 0);
	if(pstClk->pPClk == NULL){
		eprintk("%s: get pclk\n", __func__);

		return TCC_SC_ERROR_UNSUPPORT;
	}
	
	pstClk->pHClk = of_clk_get(pstUartChNode, 1);
	if(pstClk->pHClk == NULL){
		eprintk("%s: get hclk\n", __func__);

		return TCC_SC_ERROR_UNSUPPORT;
	}

#ifdef SUPPORT_PWM_CLK
	of_property_read_u32(pdev->dev.of_node, "pwm_chnum", &pstPWMReg->CH);
	of_property_read_u32(pdev->dev.of_node, "pwm_gpio_fun", &pstPWMPort->Fn);
	
	pstPWMPort->Clk = of_get_named_gpio(pdev->dev.of_node, "pwm_clk_gpios", 0);

	pstPWMReg->pBaseAddr = (void __iomem *)io_p2v(TCC_PA_PWM);

	dprintk("%s uart pwm base addr = %08x\n", __func__, (unsigned int)pstPWMReg->pBaseAddr);
	
	pstPWMClk->pPClk = clk_get(NULL, "pdm");
	if(IS_ERR(pstPWMClk->pPClk)) {
		eprintk("%s: get pwm pclk\n", __func__);

		return TCC_SC_ERROR_UNSUPPORT;
	}

#endif

#if 1
	dprintk("%s(%d)\n", __func__, giSCIrq);
	dprintk("%s(%d, %d, %d, %d, %d)\n", __func__, pstClk->Fuart, pstReg->CH, pstPort->PortNum, pstPort->Fn, pstPort->TDA8024);
	dprintk("%s(%d, %d, %d, %d, %d)\n", __func__, pstPort->Rst, pstPort->Detect, pstPort->PwrEn, pstPort->VccSel, pstPort->Clk);
	dprintk("%s(%d, %d, %d, %d)\n", __func__, pstPort->DataC7, pstPort->DataC4, pstPort->DataC8, pstPort->PinMask);
#endif

	return 0;;
}

void tca_sc_remove(void)
{
#ifdef SUPPORT_PWM_CLK
	sDRV_UART_PWM_CLK *pstPWMClk = &gDRV_UART_PWM_CLK;
#endif
	sDRV_UART_SC_CLK *pstClk = &gDRV_UART_SC_CLK;

	dprintk("%s\n", __func__);

	if (pstClk->pHClk) {
		clk_disable_unprepare(pstClk->pHClk);
		clk_put(pstClk->pHClk);
		pstClk->pHClk = NULL;
	}
 
	if (pstClk->pPClk) {
		clk_disable_unprepare(pstClk->pPClk);
		clk_put(pstClk->pPClk);
		pstClk->pPClk = NULL;
	}

#ifdef SUPPORT_PWM_CLK
	if (pstPWMClk->pPClk) {
		clk_disable_unprepare(pstPWMClk->pPClk);
		clk_put(pstPWMClk->pPClk);
		pstPWMClk->pPClk = NULL;
	}
#endif

	return;
}
