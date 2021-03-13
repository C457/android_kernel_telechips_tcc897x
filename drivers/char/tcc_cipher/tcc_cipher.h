/* linux/arch/arm/mach-tcc92xx/include/mach/tcc_cipher_ioctl.h
 *
 * Copyright (C) 2009, 2010 Telechips, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _TCC_CIPHER_H_
#define _TCC_CIPHER_H_

/* The Key Length in AES */
enum
{
	TCC_CIPHER_KEYLEN_128 = 0,
	TCC_CIPHER_KEYLEN_192,
	TCC_CIPHER_KEYLEN_256,
	TCC_CIPHER_KEYLEN_256_1,
	TCC_CIPHER_KEYLEN_MAX
};

/* The Mode in DES */
enum
{
	TCC_CIPHER_DESMODE_SINGLE = 0,
	TCC_CIPHER_DESMODE_DOUBLE,
	TCC_CIPHER_DESMODE_TRIPLE2,
	TCC_CIPHER_DESMODE_TRIPLE3,
	TCC_CIPHER_DESMODE_MAX
};

/* The Operation Mode */
enum
{
	TCC_CIPHER_OPMODE_ECB = 0,
	TCC_CIPHER_OPMODE_CBC,
	TCC_CIPHER_OPMODE_CFB,
	TCC_CIPHER_OPMODE_OFB,
	TCC_CIPHER_OPMODE_CTR,
	TCC_CIPHER_OPMODE_CTR_1,
	TCC_CIPHER_OPMODE_CTR_2,
	TCC_CIPHER_OPMODE_CTR_3,
	TCC_CIPHER_OPMODE_MAX
};

/* The Algorithm of the Encryption */
enum
{
	TCC_CIPHER_ALGORITM_BYPASS = 0,	//Use Like DMA
	TCC_CIPHER_ALGORITM_AES = 1,
	TCC_CIPHER_ALGORITM_DES,
	TCC_CIPHER_ALGORITM_MULTI2,
	TCC_CIPHER_ALGORITM_CSA2,
	TCC_CIPHER_ALGORITM_CSA3,
	TCC_CIPHER_ALGORITM_HASH,
	TCC_CIPHER_ALGORITM_MAX
};

/* The Key Select Mode */
enum
{
	TCC_CIPHER_KEY_NORMAL = 0,
	TCC_CIPHER_KEY_FROM_OTP,
	TCC_CIPHER_KEY_MAX
};

/* The Base Address */
enum
{
	TCC_CIPHER_BASEADDR_TX = 0,
	TCC_CIPHER_BASEADDR_RX,
	TCC_CIPHER_BASEADDR_MAX
};


/* The Key Option for Multi2 */
enum
{
	TCC_CIPHER_KEY_MULTI2_DATA = 0,
	TCC_CIPHER_KEY_MULTI2_SYSTEM,
	TCC_CIPHER_KEY_MULTI2_MAX
};

/* CIPHER IOCTL Command */
enum
{
	TCC_CIPHER_IOCTL_SET_ALGORITHM = 0x100,
	TCC_CIPHER_IOCTL_SET_KEY,
	TCC_CIPHER_IOCTL_SET_VECTOR,
	TCC_CIPHER_IOCTL_SET,
	TCC_CIPHER_IOCTL_RUN,
	TCC_CIPHER_IOCTL_MAX,
};

/*DMA Control Channel */
typedef enum
{
	CH0 	= 0x00000,
	CH1 	= 0x00001,
	CH2 	= 0x00010,
	CH3 	= 0x00011,
	CH4 	= 0x00100,
	CH5 	= 0x00101,
	CH6 	= 0x00110,
	CH7 	= 0x00111,
	CH8 	= 0x01000,
	CH9 	= 0x01001,
	CH10 	= 0x01010,
	CH11 	= 0x01011,
	CH12	= 0x01100,
	CH13	= 0x01101,
	CH14	= 0x01110,
	CH15	= 0x01111,
}CIPHER_DCTRL_CH_TYPE;

/*HASH Mode */
enum
{
	MD5 = 0,
	SHA_1 ,
	SHA_224,
	SHA_256,
	SHA_384,
	SHA_512,
	SHA_512_224,
	SHA_512_256
};

/************************************************************************************************
uOperationMode	: ECB, CBC, ... 		refer to OPERATION_MODE_TYPE
uAlgorithm		: AES, DES, Multi2 	refer to ALGORITHM_TYPE
uArgument1		: AES	-> refer to AES_MODE_TYPE(key lenght)
DES	-> refer to DES_MODE_TYPE (single, double,..)
Multi2-> not used
uArgument2		: Multi2	-> round_count (Variable)
AES, DES  -> not  used
 *************************************************************************************************/
typedef struct
{
	unsigned int	uOperationMode;
	unsigned int	uAlgorithm;
	unsigned int	uArgument1;
	unsigned int	uArgument2;
} stCIPHER_ALGORITHM;

typedef struct
{
	unsigned char	pucData[40];
	unsigned int	uLength;
	unsigned int	uOption;
} stCIPHER_KEY;

typedef struct
{
	unsigned char	pucData[16];
	unsigned int	uLength;
} stCIPHER_VECTOR;

typedef struct {
	unsigned char   keyLoad;        /**< Load the key or not*/
	unsigned char   ivLoad;         /**< Load the initial vector or not*/
	unsigned char   enc;            /**< Decide encrypt or decrypt*/
} stCIPHER_SET;

typedef struct
{
	unsigned char	*pucSrcAddr;
	unsigned char	*pucDstAddr;
	unsigned int	uLength;
	unsigned int	uPKCS7Pad;	/**< Add or Remove PKCS7 Pad only for Mobis*/
} stCIPHER_RUN;

#ifdef __KERNEL__
int tcc_cipher_open(struct inode *inode, struct file *filp);
int tcc_cipher_release(struct inode *inode, struct file *file);
int tcc_cipher_encrypt(unsigned char *pucSrcAddr, unsigned char *pucDstAddr, unsigned  int uLength);
int tcc_cipher_decrypt(unsigned char *pucSrcAddr, unsigned char *pucDstAddr, unsigned  int uLength);
void tcc_cipher_set_vector(unsigned char *pucData, unsigned  int uLength);
void tcc_cipher_set_key(unsigned char *pucData, unsigned int uLength, unsigned int uOption);
void tcc_cipher_set_algorithm(unsigned  int uAlgorithm, unsigned  int uArg1, unsigned  int uArg2);
void tcc_cipher_set_opmode(unsigned  int uOpMode);
#endif

#endif
