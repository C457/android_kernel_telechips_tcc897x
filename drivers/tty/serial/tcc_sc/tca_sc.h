/****************************************************************************
 *   FileName    : tca_sc.h
 *   Description : 
 ****************************************************************************
 *
 *   TCC Version 1.0
 *   Copyright (c) Telechips, Inc.
 *   ALL RIGHTS RESERVED
 *
 ****************************************************************************/
#ifndef _TCA_SC_H_
#define _TCA_SC_H_

#ifdef __cplusplus
extern "C" {
#endif

/*****************************************************************************
* UART DEFINES
******************************************************************************/

// UART Register Address Definition
#define OFFSET_THR				0x00	// Transmit Holding register
#define OFFSET_RBR				0x00	// Receive Buffer register
#define OFFSET_DLL				0x00	// Divisor Latch (Low-Byte)
#define OFFSET_IER				0x04	// Interrupt Enable Register
#define OFFSET_DLM				0x04	// Divisor Latch (High-Byte)
#define OFFSET_IIR				0x08	// Interrupt Identification Register
#define OFFSET_FCR				0x08	// FIFO Control Register
#define OFFSET_LCR				0x0C	// Line Control Register
#define OFFSET_MCR				0x10	// Modem Control Register
#define OFFSET_LSR				0x14	// Line Status Register
#define OFFSET_MSR				0x18	// Modem Status Register
#define OFFSET_SCR				0x1C	// SCR Scratch Register
#define OFFSET_AFT				0x20	// AFC Trigger Level Register
#define OFFSET_UCR				0x24	// UART Control Register
#define OFFSET_SCCR				0x60	// Smart Card Control Register
#define OFFSET_STC				0x64	// Smart Card TX Count Register

#define OFFSET_PCFG0			0x00	// Port Configuration Register 0
#define OFFSET_PCFG1			0x04	// Port Configuration Register 1
#define OFFSET_ISTS				0x0C	// IRQ Status

// UART Register Setting Value Definition
#define HwUART_LCR_DLAB_ON					Hw7 			// Access the divisor latches of the baud generator
#define HwUART_LCR_DLAB_OFF					~Hw7			// Access the receiver buff, the transmitter holding register, or the interrupt enable register
#define HwUART_LCR_SB_ON						Hw6 			// The serial output is forced to the spacing (logic 0) state
#define HwUART_LCR_SB_OFF						~Hw6			// Disable the break
#define HwUART_LCR_SP_ON						Hw5 			// When bits 3, 4 and 5 are logic 1 the parity bits is transmitted and checked as a logic 0. If bits 3 and 5 are 1 and bit 4 is a logic 0 then the parity bit is transmitted and checked as a logic 1
#define HwUART_LCR_SP_OFF						~Hw5			// Disable stick parity
#define HwUART_LCR_EPS_EVEN 					Hw4 			// Generate or check even parity
#define HwUART_LCR_EPS_ODD					~Hw4			// Generate or check odd parity
#define HwUART_LCR_PEN_ON						Hw3 			// A parity bit is generated (TX) or Checked (RX)
#define HwUART_LCR_PEN_OFF					~Hw3			// Disabled
#define HwUART_LCR_STB_ONE					Hw2 			// One stop bit is generated in the transmitted data
#define HwUART_LCR_STB_WLS					~Hw2			// When 5bit word length is selected, one and a half stop bits are generated. When either a 6-, 7-, or 8-bit word length is selected, two stop bits are generated.
#define HwUART_LCR_WLS_5						~(Hw1+Hw0)		// 5 bits word length
#define HwUART_LCR_WLS_6						Hw0 			// 6 bits word length
#define HwUART_LCR_WLS_7						Hw1 			// 7 bits word length
#define HwUART_LCR_WLS_8						(Hw1+Hw0)		// 8 bits word length

#define HwUART_MCR_RS_START 					Hw6 			// nRTS is deasserted at the RX Start Condition
#define HwUART_MCR_RS_STOP					HwZERO			// nRTS is deasserted at the RX Stop Condition
#define HwUART_MCR_AFE_EN						Hw5 			// Auto Flow Control Enabled
#define HwUART_MCR_LOOP_ON					Hw4 			// Loop Back Enable
#define HwUART_MCR_LOOP_OFF 					HwZERO			// Loop Back Disable
#define HwUART_MCR_RTS							Hw1 			// This bit informs the external modem that the uart is ready to exchange data

#define HwUART_LSR_ERF_FIFO 					Hw7 			// In the FIFO mode this bit is set when there is at least one parity error, framing error or break indication in the FIFO
#define HwUART_LSR_ERF_16450					~Hw7			// In the 16450 mode
#define HwUART_LSR_TEMT_ON					Hw6 			// Transmitter holding register and the transmitter shift register are both empty
#define HwUART_LSR_TEMT_OFF 					~Hw6			//
#define HwUART_LSR_THRE_ON					Hw5 			// UART is ready to accept a new char for transmission
#define HwUART_LSR_THRE_OFF 					~Hw5			//
#define HwUART_LSR_BI_ON						Hw4 			// The received data input is held in the spacing (logic 0) state for longer than a full word transmission time
#define HwUART_LSR_BI_OFF						~Hw4			//
#define HwUART_LSR_FE_ON						Hw3 			// The received character did not have a valid stop bit
#define HwUART_LSR_FE_OFF						~Hw3			//
#define HwUART_LSR_PE_ON						Hw2 			// The received data character does not have the correct even or odd parity
#define HwUART_LSR_PE_OFF						~Hw2			//
#define HwUART_LSR_OE_ON						Hw1 			// The receiver buffer register was not read by the CPU before the next character was transferred into the receiver buffer register
#define HwUART_LSR_OE_OFF						~Hw1			//
#define HwUART_LSR_DR_ON						Hw0 			// The receiver data ready
#define HwUART_LSR_DR_OFF						~Hw0			//


#define HwUART_SCR_RXD_ST_OFF					Hw2				// RXD line status is released
#define HwUART_SCR_RXD_ST_ON					~Hw2			// RXD line status is busy
#define HwUART_SCR_ACK							Hw1				// Received ACK
#define HwUART_SCR_NAK							~Hw1			// Received NAK
#define HwUART_SCR_ACK_EN						Hw0				// RX ACK Signal Enable
#define HwUART_SCR_ACK_DIS						~Hw0			// RX ACK Signal Disable

#define HwUART_MSR_CTS							Hw4 			// This bit is the complement of the Claer to Send input
#define HwUART_MSR_DCTS 						Hw0 			// This bit is the Delta Clear to Send indicator

#define HwUART_SCCR_SEN_EN					Hw31			// SmartCard Interface Enable
#define HwUART_SCCR_SEN_DIS 					~Hw31			// SmartCard Interface Disable
#define HwUART_SCCR_STEN_EN 					Hw30			// SmartCard TX Enable
#define HwUART_SCCR_STEN_DIS					~Hw30			// SmartCard TX Disable
#define HwUART_SCCR_DEN_EN					Hw29			// SmartCard Clock Divider Enable
#define HwUART_SCCR_DEN_DIS					~Hw29			// SmartCard Clock Divider Disable
#define HwUART_SCCR_P_EN						Hw28			// SmartCard Clock Post Divider Enable
#define HwUART_SCCR_P_DIS						~Hw28			// SmartCard Clock Post Divider Disable
#define HwUART_SCCR_STF 						Hw27			// SmartCard Clock TX Done Flag
#define HwUART_SCCR_STE_EN					Hw26			// SmartCard Clock TX Done Interrupt Enable
#define HwUART_SCCR_STE_DIS 					~Hw26			// SmartCard Clock TX Done Interrupt Disable
#define HwUART_SCCR_DIV(X)						((X)*Hw0)		// SmartCard Clock Divisor
#define HwUART_SCCR_DIV_MASK					(0xFFFF)			

#define HwUART_STC_SCNT(X)						((X)*Hw0)		// SmartCard Start TX Count
#define HwUART_STC_SCNT_MASK					(0xFFFF)						

#define HwUART_IER_EMSI_EN					Hw3 			// Enable Modem status interrupt
#define HwUART_IER_EMSI_DIS 					~Hw3			// Disable Modem status interrupt
#define HwUART_IER_ELSI_EN						Hw2 			// Enable receiver line status interrupt
#define HwUART_IER_ELSI_DIS 					~Hw2			// Disable receiver line status interrupt
#define HwUART_IER_ETXI_EN						Hw1 			// Enable transmitter holding register empty interrupt
#define HwUART_IER_ETXI_DIS 					~Hw1			// Disable transmitter holding register empty interrupt
#define HwUART_IER_ERXI_EN						Hw0 			// Enable received data available interrupt
#define HwUART_IER_ERXI_DIS 					~Hw0			// Disable received data available interrupt

// UART Driver Channel Configuration Definition
#define	DRV_UART_LCR_AccessDivisor			HwUART_LCR_DLAB_ON
#define	DRV_UART_LCR_NotAccessDivisor			0
#define	DRV_UART_LCR_EnableBreak				HwUART_LCR_SB_ON
#define	DRV_UART_LCR_DisableBreak				0
#define	DRV_UART_LCR_EnableStickParity			HwUART_LCR_SP_ON
#define	DRV_UART_LCR_DisableStickParity		0
#define	DRV_UART_LCR_EnableEvenParity			HwUART_LCR_EPS_EVEN
#define	DRV_UART_LCR_EnableOddParity			0
#define	DRV_UART_LCR_EnableParity				HwUART_LCR_PEN_ON
#define	DRV_UART_LCR_DisableParity				0
#define	DRV_UART_LCR_OneStopBit				HwUART_LCR_STB_ONE
#define	DRV_UART_LCR_TwoStopBit				0
#define	DRV_UART_LCR_WordLength_SHIFT		0
#define	DRV_UART_LCR_WordLength(X)			((X)<<DRV_UART_LCR_WordLength_SHIFT)
#define	DRV_UART_LCR_WordLength_5Bit			DRV_UART_LCR_WordLength(0)
#define	DRV_UART_LCR_WordLength_6Bit			DRV_UART_LCR_WordLength(1)
#define	DRV_UART_LCR_WordLength_7Bit			DRV_UART_LCR_WordLength(2)
#define	DRV_UART_LCR_WordLength_8Bit			DRV_UART_LCR_WordLength(3)

#define	DRV_UART_IER_EnableMSI				HwUART_IER_EMSI_EN
#define	DRV_UART_IER_DisableMSI				0
#define	DRV_UART_IER_EnableRcvLSI				HwUART_IER_ELSI_EN
#define	DRV_UART_IER_DisableRcvLSI			0
#define	DRV_UART_IER_EnableTXI				HwUART_IER_ETXI_EN
#define	DRV_UART_IER_DisableTXI				0
#define	DRV_UART_IER_EnableRXI				HwUART_IER_ERXI_EN
#define	DRV_UART_IER_DisableRXI				0

#define	DRV_UART_FCR_RxTriggerLvl_SHIFT		6
#define	DRV_UART_FCR_RxTriggerLvl(X)			((X)<<DRV_UART_FCR_RxTriggerLvl_SHIFT)
#define	DRV_UART_FCR_RxTriggerLvl_01			DRV_UART_FCR_RxTriggerLvl(0)
#define	DRV_UART_FCR_RxTriggerLvl_04			DRV_UART_FCR_RxTriggerLvl(1)
#define	DRV_UART_FCR_RxTriggerLvl_08			DRV_UART_FCR_RxTriggerLvl(2)
#define	DRV_UART_FCR_RxTriggerLvl_14			DRV_UART_FCR_RxTriggerLvl(3)
#define	DRV_UART_FCR_TxWindowLvl_SHIFT		4
#define	DRV_UART_FCR_TxWindowLvl(X)			((X)<<DRV_UART_FCR_TxWindowLvl_SHIFT)
#define	DRV_UART_FCR_TxWindowLvl_16			DRV_UART_FCR_TxWindowLvl(0)
#define	DRV_UART_FCR_TxWindowLvl_08			DRV_UART_FCR_TxWindowLvl(1)
#define	DRV_UART_FCR_TxWindowLvl_04			DRV_UART_FCR_TxWindowLvl(2)
#define	DRV_UART_FCR_TxWindowLvl_01			DRV_UART_FCR_TxWindowLvl(3)
#define	DRV_UART_FCR_EnableDMA				Hw3
#define	DRV_UART_FCR_DisableDMA				0
#define	DRV_UART_FCR_TxFifoReset				Hw2
#define	DRV_UART_FCR_RxFifoReset				Hw1
#define	DRV_UART_FCR_EnableFifo				Hw0
#define	DRV_UART_FCR_DisableFifo				0

#define DRV_UART_SCR_EnableACK				Hw0
#define DRV_UART_SCR_DisableACK				0

#define DRV_UART_SCCR_Enable				Hw31
#define DRV_UART_SCCR_EnableTx				Hw30
#define DRV_UART_SCCR_EnableClockDiv		Hw29
#define DRV_UART_SCCR_EnableClcokPostDiv	Hw28
#define DRV_UART_SCCR_EnableTxDoneInt		Hw26

// UART SmartCard Base Address
#define DRV_UART_SC_CH_BASE(X)					(HwUART0_BASE+(X)*0x10000)

// UART SmartCard Clock Conversion Factor Definition
#define DRV_UART_SCCLK_FI_372				372 //Internal Clock
#define DRV_UART_SCCLK_FI_558				558
#define DRV_UART_SCCLK_FI_744				744
#define DRV_UART_SCCLK_FI_1116			1116
#define DRV_UART_SCCLK_FI_1488			1488
#define DRV_UART_SCCLK_FI_1860			1860
#define DRV_UART_SCCLK_FI_512				512
#define DRV_UART_SCCLK_FI_768				768
#define DRV_UART_SCCLK_FI_1024			1024		
#define DRV_UART_SCCLK_FI_1536			1536		
#define DRV_UART_SCCLK_FI_2048			2048
#define DRV_UART_SCCLK_FACTOR_RFU            372 // for NDS

// UART SmartCard Default Clock Definition
#define DRV_UART_SCCLK_FMAX_4MHz			4000*1000 
#define DRV_UART_SCCLK_FMAX_5MHz			4500*1000 // 4.5 MHz for NDS
#define DRV_UART_SCCLK_FMAX_6MHz			6000*1000 
#define DRV_UART_SCCLK_FMAX_8MHz			6750*1000 // 6.75 MHz for NDS
#define DRV_UART_SCCLK_FMAX_12MHz		12000*1000
#define DRV_UART_SCCLK_FMAX_16MHz		13500*1000 // 13.5 MHz for NDS
#define DRV_UART_SCCLK_FMAX_20MHz		13500*1000 // 13.5 MHz for NDS
#define DRV_UART_SCCLK_FMAX_7MHz			7500*1000
#define DRV_UART_SCCLK_FMAX_10MHz		10000*1000
#define DRV_UART_SCCLK_FMAX_15MHz		15000*1000
#define DRV_UART_SCCLK_FMAX_RFU			4000*1000 //Reserved

// UART SmartCard Time Out Definition
#define DRV_UART_SC_TIME_OUT					2000 // 2.0 ms

#define DRV_UART_SC_MAX_CH					1

// UART Channel Number Definition
enum
{
	DRV_UART_CH0 = 0,
	DRV_UART_CH1,
	DRV_UART_CH2,
	DRV_UART_CH3,
	DRV_UART_CH4,
	DRV_UART_CH5,
	DRV_UART_CH6,
	DRV_UART_CH7,
	DRV_UART_CHMAX
};

// UART Smart Card Control Definition
enum
{
	DRV_UART_SCCTRL_DISABLE = 0,
	DRV_UART_SCCTRL_ENABLE,
	DRV_UART_SCCTRL_MAX
};

// UART Interrupt Definition for Smart Card
enum
{
	DRV_UART_SCINT_DISABLE = 0,
	DRV_UART_SCINT_ENABLE,
	DRV_UART_SCINT_NOCHANGE,
	DRV_UART_SCINT_MAX
};

// UART Register Data Structure
typedef struct
{
	void __iomem 	*pBaseAddr;

	unsigned int		CH;		// Channel Number

	unsigned int		IER;		// Interrupt Enable Register
	unsigned int		FCR;		// FIFO Control Register
	unsigned int		LCR;		// Line Control Register
	unsigned int		SCR;		// Scratch Register		
	unsigned int		AFT;		// Auto Flow Control Trigger Level Register

	unsigned int		SCCTRL;		// UART Smart Card Control
} sDRV_UART_SC_REG;

// UART Port Data Structure
typedef struct
{
	void __iomem    *pBaseAddr;

	int	PortNum;
	
	int	Rst;
	int	Detect;
	int	PwrEn;
	int	VccSel;
	int	Clk;
	int	DataC7;
	int	DataC4;
	int	DataC8;
	int	PinMask;

	int	Fn;
	int	FnC4;
	int	FnC8;

	int	TDA8024;
} sDRV_UART_SC_PORT;

// UART Clock Structure
typedef struct
{
	struct clk* 		pPClk;		// UART Clock
	struct clk* 		pHClk;		// UART Clock
	
	unsigned int		Fi;
	unsigned int		Di;
	unsigned int		N;
	unsigned int		WI;
	
	unsigned int		Fuart;		// UART Peripherial Clock 
	unsigned int 		Fout;		// UART Clock
	unsigned int		Baud;		// UART Baud Rate Clock

	unsigned int		ETU;
	unsigned int		GT;
	unsigned int		WT;
	unsigned int		CWT;
	unsigned int		BWT;
} sDRV_UART_SC_CLK;

// UART Internal Buffer for Smart Card
#define DRV_UART_SC_RX_BUF_SIZE		512

typedef struct
{
	unsigned short	usRxHead;
	unsigned short	usRxTail;
	unsigned char		pRxBuf[DRV_UART_SC_RX_BUF_SIZE];
} sDRV_UART_SC_BUF;

// ATR(Answer To Reset) Information
#define TA_CHECK_BIT		0x10
#define TB_CHECK_BIT		0x20
#define TC_CHECK_BIT		0x40
#define TD_CHECK_BIT		0x80

typedef struct
{
	unsigned int	uiProtocol;
	unsigned int	uiConvention;
	
	unsigned short 	usTS;
	unsigned short 	usT0;
	unsigned short 	usTA[4];
	unsigned short	usTB[4];
	unsigned short	usTC[4];
	unsigned short 	usTD[4];
	unsigned char 	ucHC[15]; // Historical Characters - Max 15 Bytes
	unsigned char	ucTCK;
} sDRV_UART_SC_ATR;


/*****************************************************************************
* PWM DEFINES
******************************************************************************/
// PWM Register Address Definition
#define OFFSET_PWMST			0x00	// PWM Status Register
#define OFFSET_PWMEN			0x04	// PWM Enable Register
#define OFFSET_PWMMODE		0x08	// PWM Mode Register
#define OFFSET_PWMLOOP		0x0C	// PWM Loop Register
#define OFFSET_PWMPSTN12		0x10	// PWM Pstn12 Register
#define OFFSET_PWMPSTN32		0x14	// PWM pstn34 Register
#define OFFSET_PWMOUTN1		0x30	//PWM Out n1 Register
#define OFFSET_PWMOUTN2		0x34	//PWM Out n1 Register
#define OFFSET_PWMOUTN3		0x38	//PWM Out n1 Register
#define OFFSET_PWMOUTN4		0x3C	//PWM Out n1 Register

// PWM Register Setting Value Definition
#define HwPWM_EN_TRIG_D						Hw19
#define HwPWM_EN_TRIG_C						Hw18
#define HwPWM_EN_TRIG_B						Hw17
#define HwPWM_EN_TRIG_A						Hw16
#define HwPWM_EN_EN_D							Hw3
#define HwPWM_EN_DIS_D						~Hw3
#define HwPWM_EN_EN_C							Hw2
#define HwPWM_EN_DIS_C						~Hw2
#define HwPWM_EN_EN_B							Hw1
#define HwPWM_EN_DIS_B						~Hw1
#define HwPWM_EN_EN_A							Hw0
#define HwPWM_EN_DIS_A						~Hw0

#define HwPWM_MODE_PHASE_MODE				Hw0
#define HwPWM_MODE_REG_OUT_MODE			Hw1
#define HwPWM_MODE_REG_OUT_MODE2			Hw2
#define HwPWM_MODE_PHASE_MODE2				(Hw3|Hw0)

// UART Driver Channel Configuration Definition
#define	DRV_UART_LCR_WordLength_SHIFT		0
#define	DRV_UART_LCR_WordLength(X)			((X)<<DRV_UART_LCR_WordLength_SHIFT)
#define	DRV_UART_LCR_WordLength_5Bit			DRV_UART_LCR_WordLength(0)

#define DRV_PWM_EN_Trigger_BASE				HwPWM_EN_TRIG_A
#define DRV_PWM_EN_Trigger(CH)					(DRV_PWM_EN_Trigger_BASE<<(CH))
#define DRV_PWM_EN_Enable_BASE				HwPWM_EN_EN_A
#define DRV_PWM_EN_Enable(CH)					(DRV_PWM_EN_Enable_BASE<<(CH))

#define DRV_PWM_MODE_CLR(CH)				~(0xF<<(CH*4))
#define DRV_PWM_MODE_SET(CH, X)				((X)<<(CH*4))	

// UART Default Peripheral Clock Definition
#define DRV_UART_PWMCLK_FUART_193KHz		193536

// PWM Peripheral Clock Definition
#define DRV_UART_PWMCLK_FPWM_36MHz			36571428
#define DRV_UART_PWMCLK_FPWM_54MHz			54857142
#define DRV_UART_PWMCLK_FPWM_109MHz		109714285

// PWM Register Data Structure
typedef struct
{
	void __iomem 	*pBaseAddr;

	unsigned int		CH;				// Channel Number

	unsigned int		EN;				// Enable Register
	unsigned int		MODE;			// Mode Register
	unsigned int		PWMOUTN1;		// Out n1 Register
} sDRV_UART_PWM_REG;

// PWM Port Data Structure
typedef struct
{
	void __iomem    *pBaseAddr;

	int	Clk;
	
	int	Fn;
} sDRV_UART_PWM_PORT;

// PWM Clock Data Structure
typedef struct
{
	struct clk		*pPClk;
	
	unsigned int		Fpwm;		// PWM Peripherial Clock 
} sDRV_UART_PWM_CLK;

/*****************************************************************************
* APIs
******************************************************************************/
extern unsigned tca_sc_detect_card(void);
extern int tca_sc_enable(void);
extern int tca_sc_disable(void);
extern void tca_sc_activate(void);
extern void tca_sc_deactivate(void);
extern int tca_sc_select_vcc_level(unsigned iLevel);
extern int tca_sc_cold_reset(unsigned char *pATR, unsigned *puATRLen);
extern int tca_sc_warm_reset(unsigned char *pATR, unsigned *puATRLen);
extern void tca_sc_set_config(stTCC_SC_CONFIG stSCConfig);
extern void tca_sc_get_config(stTCC_SC_CONFIG *pstSCConfig);
extern int tca_sc_send_receive(char *pTxBuf, unsigned uTxBufLen, char *pRxBuf, unsigned *puRxBufLen);
extern int tca_sc_send_receive_len(char *pTxBuf, unsigned uTxBufLen, char *pRxBuf, unsigned *puRxBufLen);
extern int tca_sc_send_receive_for_nds(char *pTxBuf, unsigned uTxBufLen, char *pRxBuf, unsigned *puRxBufLen, unsigned uDirection, unsigned uTimeOut);
extern int tca_sc_set_io_pin_config(unsigned uPinMask);
extern int tca_sc_get_io_pin_config(unsigned *puPinMask);
extern int tca_sc_open(struct inode *inode, struct file *filp);
extern void tca_sc_close(struct inode *inode, struct file *file);
extern int tca_sc_probe(struct platform_device *pdev);
extern void tca_sc_remove(void);

#ifdef __cplusplus
}
#endif

#endif  /* #ifndef _TCA_SC_H_  */

