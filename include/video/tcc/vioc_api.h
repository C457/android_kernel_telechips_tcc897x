/*
 * linux/arch/arm/mach-tcc893x/include/mach/vioc_api.h
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

#ifndef __VIOC_API_H__
#define	__VIOC_API_H__

#include "tcc_video_regs.h"
#include "tccfb_ioctrl.h"

/* Scaler Related*/

typedef enum
{
	VIOC_SCALER0 = 0,
	VIOC_SCALER1, 
	VIOC_SCALER2, 
	VIOC_SCALER3,
	VIOC_SCALER4,
	VIOC_SCALER_MAX
} VIOC_SCALER_Type;

typedef struct
{
	unsigned short SRC_WIDTH; 	 	/* Scaler Input Source Width*/
	unsigned short SRC_HEIGHT; 		/* Scaler Input Source Height*/
	unsigned short DST_WIDTH; 		/* Scaler Output Image Dest Width in Output Window(Scaler out image real size) */
	unsigned short DST_HEIGHT; 		/* Scaler Output Image Dest Width in Output Window(Scaler out image real size) */
	unsigned short OUTPUT_POS_X; 	/* Scaler Output Image Positon x in output Window */
	unsigned short OUTPUT_POS_Y; 	/* Scaler Output Image Positon y in output Window*/
	unsigned short OUTPUT_WIDTH; 	/* Scaler Output Window Size*/
	unsigned short OUTPUT_HEIGHT; 	/* Scaler Output Window Size*/
	unsigned short BYPASS;  		/*Scaler Bypass Option*/
} VIOC_SCALER_INFO_Type;


/* 
	Interface APIs 
*/

/* Scaler Component */
extern void VIOC_API_SCALER_SetConfig(unsigned int scaler, VIOC_SCALER_INFO_Type * Info);
extern void VIOC_API_SCALER_SetUpdate(unsigned int scaler);
extern int VIOC_API_SCALER_SetInterruptEnable(unsigned int scaler, unsigned int interrupt);
extern int VIOC_API_SCALER_SetInterruptDiable(unsigned int scaler, unsigned int interrupt);
extern int VIOC_API_SCALER_SetPlugIn(unsigned int scaler, unsigned int path);
extern int VIOC_API_SCALER_SetPlugOut(unsigned int scaler);
extern void VIOC_API_IREQ_Init(void);

#endif //__VIOC_API_H__

