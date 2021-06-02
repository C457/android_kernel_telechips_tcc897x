/****************************************************************************
 *   FileName    : tca_cipher.h
 *   Description : 
 ****************************************************************************
 *
 *   TCC Version 1.0
 *   Copyright (c) Telechips, Inc.
 *   ALL RIGHTS RESERVED
 *
 ****************************************************************************/
#ifndef _TCA_CIPHER_H_
#define _TCA_CIPHER_H_


#ifdef __cplusplus
extern "C" {
#endif


/*****************************************************************************
*
* Defines
*
******************************************************************************/
#define TCC897x_VPICREG(x) (x)	//Vectored Priority Interrupt Controller

#define TCC897x_VPIC_IEN1	   		TCC897x_VPICREG(0x04)
#define TCC897x_VPIC_CLR1	   		TCC897x_VPICREG(0x0c)
#define TCC897x_VPIC_SEL1	   		TCC897x_VPICREG(0x1c)
#define TCC897x_VPIC_POL1	   		TCC897x_VPICREG(0x3c)
#define TCC897x_VPIC_MODE1	   		TCC897x_VPICREG(0x64)
#define TCC897x_VPIC_INTMSK1		TCC897x_VPICREG(0x104)

#define TCC897x_CIPHERREG(x) (x)

#define TCC897x_CIPHER_DCTRL	   	TCC897x_CIPHERREG(0x00)
#define TCC897x_CIPHER_SBASE	   	TCC897x_CIPHERREG(0x04)
#define TCC897x_CIPHER_DBASE	   	TCC897x_CIPHERREG(0x08)
#define TCC897x_CIPHER_SIZE	   	TCC897x_CIPHERREG(0x0C)
#define TCC897x_CIPHER_FIFO0	   	TCC897x_CIPHERREG(0x10)
#define TCC897x_CIPHER_FIFO1	   	TCC897x_CIPHERREG(0x14)
#define TCC897x_CIPHER_FIFO2	   	TCC897x_CIPHERREG(0x18)
#define TCC897x_CIPHER_FIFO3	   	TCC897x_CIPHERREG(0x1C)
#define TCC897x_CIPHER_INTMASK	TCC897x_CIPHERREG(0x20)
#define TCC897x_CIPHER_INTSTS	   	TCC897x_CIPHERREG(0x24)
#define TCC897x_CIPHER_CCTRL		TCC897x_CIPHERREG(0x100)
#define TCC897x_CIPHER_ADM			TCC897x_CIPHERREG(0x104)
#define TCC897x_CIPHER_HASH		TCC897x_CIPHERREG(0x110)
#define TCC897x_CIPHER_CSA3		TCC897x_CIPHERREG(0x114)
#define TCC897x_CIPHER_ROUND		TCC897x_CIPHERREG(0x120)
#define TCC897x_CIPHER_KEY0		TCC897x_CIPHERREG(0x180)
#define TCC897x_CIPHER_KEY1		TCC897x_CIPHERREG(0x184)
#define TCC897x_CIPHER_KEY2		TCC897x_CIPHERREG(0x188)
#define TCC897x_CIPHER_KEY3		TCC897x_CIPHERREG(0x18C)
#define TCC897x_CIPHER_KEY4		TCC897x_CIPHERREG(0x190)
#define TCC897x_CIPHER_KEY5		TCC897x_CIPHERREG(0x194)
#define TCC897x_CIPHER_KEY6		TCC897x_CIPHERREG(0x198)
#define TCC897x_CIPHER_KEY7		TCC897x_CIPHERREG(0x19C)
#define TCC897x_CIPHER_KEY8		TCC897x_CIPHERREG(0x1A0)
#define TCC897x_CIPHER_KEY9		TCC897x_CIPHERREG(0x1A4)
#define TCC897x_CIPHER_IV0			TCC897x_CIPHERREG(0x1C0)
#define TCC897x_CIPHER_IV1			TCC897x_CIPHERREG(0x1C4)
#define TCC897x_CIPHER_IV2			TCC897x_CIPHERREG(0x1C8)
#define TCC897x_CIPHER_IV3			TCC897x_CIPHERREG(0x1CC)

/* CIPHER DMA Control Register Bits */
#define TCC897x_CIPHERDCTRL_CH(x)			(((x)&0x1F)<<24)
#define TCC897x_CIPHERDCTRL_IEN			(1<<23)
#define TCC897x_CIPHERDCTRL_RWIT(x)		(((x)&0x7)<<20)
#define TCC897x_CIPHERDCTRL_RPAUSE(x)		(((x)&0x7)<<17)
#define TCC897x_CIPHERDCTRL_REN			(1<<16)
#define TCC897x_CIPHERDCTRL_WB			(1<<15)
#define TCC897x_CIPHERDCTRL_EN			(1<<0)


/* CIPHER Source Base Address Register Bits */
/* TCC897x_CIPHER_SBASE :: (Field :31-0) (Rest : 0x0)				*/

/* CIPHER Destination Base Address Register Bits */
/* TCC897x_CIPHER_DBASE :: (Field :31-0) (Rest : 0x0)				*/

/* CIPHER Transfer Size Register Bits */
/* TCC897x_CIPHER_SIZE :: (Field :23-0) (Rest : 0x0)				*/

/* CIPHER FIFO Status 0 Register Bits */
#define TCC897x_CIPHER_FIFO0_FULL			(1<<31)
#define TCC897x_CIPHER_FIFO0_EMP			(1<<30)
#define TCC897x_CIPHER_FIFO0_IDLE			(1<<29)
#define TCC897x_CIPHER_FIFO0_NEMPTY		((x)&0xff)

/* CIPHER FIFO Status 1,2,3 Register Bits */
/* TCC897x_CIPHER_FIFO1,2,3 :: (Field :31-0) (Rest : 0x0)				*/

/* CIPHER Interrupt Mask Register Bits */
/* TCC897x_CIPHER_INTMASK :: (Field :31-0) (Rest : 0x0)				*/

/* CIPHER Interrupt Status Register Bits */
/* TCC897x_CIPHER_INTSTS :: (Field :31-0) (Rest : 0x0)				*/

/* CIPHER Control Register Bits */
#define TCC897x_CIPHER_CCTRL_IEN			(1<<15)
#define TCC897x_CIPHER_CCTRL_KEY			(1<<8)
#define TCC897x_CIPHER_CCTRL_ENDIAN(x)	(((x)&0xF)<<4)
#define TCC897x_CIPHER_CCTRL_SELECT(x)		(((x)&0x7)<<0)

/* CIPHER AES, DEs, Multi2(ADM) Register Bits */
#define TCC897x_CIPHER_ADM_KEYLD			(1<<31)
#define TCC897x_CIPHER_ADM_IVLD			(1<<30)
#define TCC897x_CIPHER_ADM_ENC			(1<<27)
#define TCC897x_CIPHER_ADM_OPMODE(x)		(((x)&0x7)<<24)
#define TCC897x_CIPHER_ADM_PRT			(1<<6)
#define TCC897x_CIPHER_ADM_DESMODE(x)	(((x)&0x3)<<4)
#define TCC897x_CIPHER_ADM_AESMODE(x)	(((x)&0x3)<<0)

/* CIPHER Hash Register Bits */
#define TCC897x_CIPHER_HASH_MODE(x)		(((x)&0x7)<<0)

/* CIPHER Round Register Bits */
/* TCC897x_CIPHER_ROUND :: (Field :31-0) (Rest : 0x0)				*/

/* CIPHER Key 0~7 Register Bits */
/* TCC897x_CIPHER_KEY0~7 :: (Field :31-0) (Rest : 0x0)				*/

/* CIPHER IV 0~3 Register Bits */
/* TCC897x_CIPHER_IV0~3 :: (Field :31-0) (Rest : 0x0)				*/


/*****************************************************************************
*
* APIs
*
******************************************************************************/
extern void tca_cipher_dma_enable(unsigned int uEnable, unsigned  int uEndian, unsigned  int uAddrModeTx, unsigned  int uAddrModeRx);
extern void tca_cipher_dma_enable_request(unsigned  int uEnable);
extern void tca_cipher_interrupt_config(unsigned  int uTxSel, unsigned  int uDoneIrq, unsigned  int uPacketIrq);
extern void tca_cipher_interrupt_enable(unsigned  int uEnable);
extern irqreturn_t tca_cipher_interrupt_handler(int irq, void *dev_id);
extern void tca_cipher_set_opmode(unsigned  int uOpMode, unsigned  int uDmaMode);
extern void tca_cipher_set_algorithm(unsigned  int uAlgorithm, unsigned  int uArg1, unsigned  int uArg2);
extern void tca_cipher_set_baseaddr(unsigned  int uTxRx, unsigned char *pBaseAddr);
extern void tca_cipher_set_packet(unsigned  int uCount, unsigned  int uSize);
extern void tca_cipher_set_key(unsigned char *pucData, unsigned  int uLength, unsigned  int uOption);
extern void tca_cipher_set_vector(unsigned char *pucData, unsigned  int uLength);
extern int tca_cipher_get_packetcount(unsigned  int uTxRx);
extern int tca_cipher_get_blocknum(void);
extern void tca_cipher_clear_counter(unsigned  int uIndex);
extern void tca_cipher_wait_done(void);
extern int tca_cipher_encrypt(unsigned char *pucSrcAddr, unsigned char *pucDstAddr, unsigned  int uLength);
extern int tca_cipher_decrypt(unsigned char *pucSrcAddr, unsigned char *pucDstAddr, unsigned  int uLength);
//extern int tca_cipher_open(struct inode *inode, struct file *filp);
extern int tca_cipher_release(struct inode *inode, struct file *file);

#ifdef __cplusplus
}
#endif

#endif  /* #ifndef _TCA_CIPHER_H_  */
