/*
 * linux/arch/arm/mach-tcc893x/vioc_api.c
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
#include <linux/slab.h>

#include <mach/bsp.h>
#include <mach/io.h>

#include <mach/vioc_scaler.h>
#include <mach/vioc_rdma.h>
#include <mach/vioc_wdma.h>
#include <mach/vioc_wmix.h>
#include <mach/vioc_disp.h>
#include <mach/vioc_ireq.h>
#include <mach/vioc_config.h>
#include <mach/vioc_global.h>
#include <mach/vioc_outcfg.h>
#include <mach/vioc_lut.h>
#include <mach/vioc_api.h>

/* Scaler Set */
/*
	API : VIOC_API_SCALER_Set/GetXXX()
*/
void VIOC_API_SCALER_SetConfig(unsigned int scaler, VIOC_SCALER_INFO_Type * Info)
{
	volatile PVIOC_SC pScaler;
	pScaler = (volatile PVIOC_SC)VIOC_SC_GetAddress(scaler);

	VIOC_SC_SetBypass(pScaler, Info->BYPASS);
	VIOC_SC_SetDstSize(pScaler, Info->DST_WIDTH, Info->DST_HEIGHT);
	VIOC_SC_SetOutPosition(pScaler, Info->OUTPUT_POS_X, Info->OUTPUT_POS_Y);
	VIOC_SC_SetOutSize(pScaler, Info->OUTPUT_WIDTH, Info->OUTPUT_HEIGHT);
}

void VIOC_API_SCALER_SetUpdate(unsigned int scaler)
{
	volatile PVIOC_SC pScaler;
	pScaler = (volatile PVIOC_SC)VIOC_SC_GetAddress(scaler);

	VIOC_SC_SetUpdate(pScaler);
}

int VIOC_API_SCALER_SetInterruptEnable(unsigned int scaler, unsigned int interrupt)
{
	int ret = 0; // VIOC_DRIVER_NOERR;
	volatile PVIOC_SC pScaler;
	pScaler = (volatile PVIOC_SC)VIOC_SC_GetAddress(scaler);

	if(scaler >= VIOC_SCALER_MAX)
	{
		ret = -3; // VIOC_DRIVER_ERR_INVALID;
	}
	else
	{
		VIOC_SC_SetInterruptMask(pScaler, interrupt, FALSE);
	}

	return ret;
}

int VIOC_API_SCALER_SetInterruptDiable(unsigned int scaler, unsigned int interrupt)
{
	int ret = 0; // VIOC_DRIVER_NOERR;
	volatile PVIOC_SC pScaler;
	pScaler = (volatile PVIOC_SC)VIOC_SC_GetAddress(scaler);

	if(scaler >= VIOC_SCALER_MAX)
	{
		ret = -3; // VIOC_DRIVER_ERR_INVALID;
	}
	else
	{
		VIOC_SC_SetInterruptMask(pScaler, interrupt, TRUE);
	}

	return ret;
}

int VIOC_API_SCALER_SetPlugIn(unsigned int scaler, unsigned int path)
{
	int ret;

	ret = VIOC_CONFIG_PlugIn(scaler, path);
	if(ret == VIOC_DEVICE_CONNECTED)
		ret = VIOC_DRIVER_NOERR;
	else
		ret = VIOC_DRIVER_ERR;

	return ret;
}

int VIOC_API_SCALER_SetPlugOut(unsigned int scaler)
{
	int ret;

	ret = VIOC_CONFIG_PlugOut(scaler);
	if(ret == VIOC_PATH_DISCONNECTED)
		ret = VIOC_DRIVER_NOERR;
	else
		ret = VIOC_DRIVER_ERR;

	return ret;
}

void VIOC_API_IREQ_Init(void)
{
	VIOC_IREQ_RegisterHandle();
}



