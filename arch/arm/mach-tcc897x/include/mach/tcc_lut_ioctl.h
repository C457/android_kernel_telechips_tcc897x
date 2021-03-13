/* linux/arch/arm/mach-tcc893x/tcc_lut_ioctl.h
 *
 * Copyright (C) 2008, 2009 Telechips, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef _TCC_VIOC_LUT_IOCTL_H_
#define _TCC_VIOC_LUT_IOCTL_H_

#include <linux/ioctl.h>
#include <mach/vioc_lut.h>
#include <mach/vioc_global.h>

 #define LUT_IOC_MAGIC		'l'


enum{
	LUT_DEV0 = 0,
	LUT_DEV1 = 1,
	LUT_COMP0 = 3,
	LUT_COMP1 = 4,
	LUT_COMP2 = 5,
	LUT_COMP3 = 6
};

struct VIOC_LUT_VALUE_SET
{
	unsigned int Gamma[256];	 //vioc componet : RGB ,  disp component : BGR
	unsigned int lut_number; 	//enum VIOC_LUT_NUM 
};

struct VIOC_LUT_PLUG_IN_SET
{
	unsigned int enable;
	unsigned int lut_number;  		//enum VIOC_LUT_NUM 
	unsigned int lut_plug_in_ch; 	//ex :VIOC_LUT_RDMA_00 
};


struct VIOC_LUT_SET_Type
{
	unsigned int lut_value;
	int sin_value;
	int cos_value;

	int saturation;
};




#define TCC_LUT_SET			_IOW(LUT_IOC_MAGIC, 2, struct VIOC_LUT_SET_Type) //hw vioc lut set
#define TCC_LUT_PLUG_IN		_IOW(LUT_IOC_MAGIC, 3, struct VIOC_LUT_SET_Type) //hw vioc lut plug ing


#ifndef ADDRESS_ALIGNED
#define ADDRESS_ALIGNED
#define ALIGN_BIT 							(0x8-1)
#define BIT_0 								3
#define GET_ADDR_YUV42X_spY(Base_addr) 	(((((unsigned int)Base_addr) + ALIGN_BIT)>> BIT_0)<<BIT_0)
#define GET_ADDR_YUV42X_spU(Yaddr, x, y) 	(((((unsigned int)Yaddr+(x*y)) + ALIGN_BIT)>> BIT_0)<<BIT_0)
#define GET_ADDR_YUV422_spV(Uaddr, x, y) 	(((((unsigned int)Uaddr+(x*y/2)) + ALIGN_BIT) >> BIT_0)<<BIT_0)
#define GET_ADDR_YUV420_spV(Uaddr, x, y) 	(((((unsigned int)Uaddr+(x*y/4)) + ALIGN_BIT) >> BIT_0)<<BIT_0)
#endif

#endif//

