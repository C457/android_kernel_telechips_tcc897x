/*
 * linux/drivers/spi/tsdemux/TSDEMUX_sys.c
 *
 * Copyright (C) 2010 Telechips, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

/****************************************************************************

  Revision History

 ****************************************************************************/
#include <linux/mm.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include "TSDEMUX_sys.h"
#include <linux/time.h>

#define DTV_DEBUG(n, args...) printk(args)
#define __ABS__(a) ((a) >= 0 ? (a) : (-(a)))

#define	MAX_PCR_CNT 4

typedef struct MpegPCR
{
	unsigned int uiPCR;
	unsigned int uiSTCBase;
	int	fUse;
}MpegPCR;
static MpegPCR gstPCR[MAX_PCR_CNT];

//extern int TCCREFTIME_Open(void);
//extern int TCCREFTIME_Close(void);
//unsigned int TCCREFTIME_GetTime(void); //It return msec unint.

static unsigned int TCCREFTIME_GetTime(void)
{
	struct timespec tspec;
	unsigned int curTime;
	do_posix_clock_monotonic_gettime(&tspec);
	curTime = (tspec.tv_sec) * 1000 + (tspec.tv_nsec) / 1000000;

	return curTime;
}

static int parseAdaptationField (unsigned char *p, MpegTsAdaptation * adap)
{

	adap->length = *p++;

	if (adap->length == 0)
		return 0;

	*(unsigned char *) &adap->flag = *p++;

	if (adap->flag.PCR)
	{
		adap->PCR = (long long) * p++ << 24;
		adap->PCR |= *p++ << 16;
		adap->PCR |= *p++ << 8;
		adap->PCR |= *p++;
		adap->PCR <<= 1;
		adap->PCR |= (*p & 0x80) >> 7;
		/* Just need 90KHz tick count */
		p += 2;
	}

#if 0
	if (adap->flag.OPCR)
	{
		adap->OPCR.clock = (long long) *p++ << 24;
		adap->OPCR.clock |= *p++ << 16;
		adap->OPCR.clock |= *p++ << 8;
		adap->OPCR.clock |= *p++;
		adap->OPCR.clock <<= 1;
		adap->OPCR.clock |= (*p & 0x80) >> 7;
		/* Just need 90KHz tick count */
		p += 2;
	}

	if (adap->flag.splicing_point)
	{
		adap->splice.splice_countdown = *p++;
	}

	if (adap->flag.private_data)
	{
		adap->private_data.length = *p++;
		memcpy (adap->private_data.data, p, adap->private_data.length);
		p += adap->private_data.length;
	}

	if (adap->flag.field_ext)
	{
		adap->ext.length = *p++;
		*(unsigned char *) &adap->ext.flag = *p++;
		if (adap->ext.flag.ltw)
		{
			adap->ext.ltw.valid_flag = (*p & 0x80) >> 7;
			adap->ext.ltw.offset = (*p++ & 0x7f) << 8;
			adap->ext.ltw.offset |= *p++;
		}

		if (adap->ext.flag.piecewise_rate)
		{
			adap->ext.piecewise.rate = (*p++ & 0x3f) << 16;
			adap->ext.piecewise.rate |= *p++ << 8;
			adap->ext.piecewise.rate |= *p++;
		}

		if (adap->ext.flag.seamless_splice)
		{
			long long temp;
			adap->ext.seamless_splice.type = (*p & 0xf0) >> 4;
			temp = ((*p++ & 0x0e) >> 1) << 30;
			temp |= *p++ << 22;
			temp |= ((*p++ & 0xfe) >> 1) << 15;
			temp |= *p++ << 7;
			temp |= ((*p++ & 0xfe) >> 1);
			adap->ext.seamless_splice.DTS_next_AU = temp;
		}
	}
#endif

	return 0;
}

/**
 * Parse MPEG TS packet header
 *
 */
int MpegSys_ParseTs (unsigned char *p, MpegTsHeader * ts, unsigned int uiCheckPID)
{
	unsigned char *base;
	int       ret = 0;

	base = p;

	/* check MPEG syncbyte */
	if (p == NULL || *p++ != MPEGSYS_TS_PACKETSYNC)
	{
		return -1;
	}

	if (ts == NULL)
	{
		return -1;
	}

	ts->error_indicator = (*p & 0x80) >> 7;
	if (ts->error_indicator)
	{
		return -1;
	}

	ts->pusi = (*p & 0x40) >> 6;
	ts->priority = (*p & 0x20) >> 5;
	ts->pid = (*p++ & 0x1f) << 8;
	ts->pid |= *p++;

	if(ts->pid != uiCheckPID)
		return -1;

	ts->scrambling_control = (*p & 0xc0) >> 6;
	ts->adaptation_control = (MpegTsAdaptationCtrl) (*p & 0x30) >> 4;
	ts->cc = *p++ & 0x0f;

	switch (ts->adaptation_control)
	{
	case TS_ADAPTATION_RESERVED:
		//PKV_DTV_DEBUG(PKV_DEBUG_ERROR, "TS_ADAPTATION_RESERVED\n");
		/* discard this packet */
		ts->payload_size = 0;
		ts->payload = NULL;
		break;

	case TS_ADAPTATION_ONLY:
		if (parseAdaptationField (p, &ts->adap) < 0)
		{
			DTV_DEBUG (DEBUG_ERROR, "failed to parse adaptation\n");
			ret = -1;
		}
		ts->payload_size = 0;
		ts->payload = NULL;
		break;

	case TS_ADAPTATION_AND_PAYLOAD:
		if (parseAdaptationField (p, &ts->adap) < 0)
		{
			DTV_DEBUG (DEBUG_ERROR, "failed to parse adaptation\n");
			ret = -1;
		}
		else
		{
			ts->payload_size = MPEGSYS_TS_PACKETSIZE - (((int) p - (int) base) + ts->adap.length + 1);
			if (ts->payload_size < 0)
			{
				DTV_DEBUG (DEBUG_ERROR, "(p-b): %d, adaptation_length: %d, 0x%x, 0x%x\n", (int) p - (int) base,
						   ts->adap.length, base[0], base[1]);
				ret = -1;
			}
			ts->payload = p + ts->adap.length + 1;
		}
		break;

	case TS_ADAPTATION_PAYLOAD_ONLY:
		ts->payload_size = MPEGSYS_TS_PACKETSIZE - 4;
		ts->payload = p;
		break;
	}

	if (ts->payload_size < 0)
	{
		ret = -1;
	}

	return ret;
}

unsigned int TSDEMUX_GetSTC(int index)
{
	unsigned int uiSTC, uiCurTime;
	if(gstPCR[index].uiPCR == 0)
		return 0;

	uiCurTime = TCCREFTIME_GetTime();
	uiSTC = gstPCR[index].uiPCR + uiCurTime - gstPCR[index].uiSTCBase;
	return uiSTC;
}

int TSDEMUX_UpdatePCR (unsigned int uiPCR, int index)
{
	unsigned int uiSTC;
	if (gstPCR[index].uiPCR)
	{
		uiSTC = TSDEMUX_GetSTC(index);
		if( __ABS__((int)uiPCR - (int)uiSTC) < 200 ) //200ms
			return 0;
		printk("%s PCR %d, STC %d, DIFF %d\n", __func__, uiPCR, uiSTC, (int)uiPCR - (int)uiSTC);
	}
	gstPCR[index].uiSTCBase = TCCREFTIME_GetTime();
	gstPCR[index].uiPCR = uiPCR;
	return 0;
}

int TSDEMUX_MakeSTC (unsigned char *pucTS, unsigned int uiTSSize, unsigned int uiPCR, int index)
{
	int i;
	unsigned int uiPcr;
	MpegTsHeader tsHeader;

	if (index >= MAX_PCR_CNT) return 0;

	for(i=0; i < uiTSSize; i += MPEGSYS_TS_PACKETSIZE)
	{
		memset (&tsHeader, 0, sizeof (tsHeader));
		if (MpegSys_ParseTs (pucTS+i, &tsHeader, uiPCR) == 0)
		{
			if (tsHeader.adap.flag.PCR)
			{
				uiPcr = tsHeader.adap.PCR >> 1;
				TSDEMUX_UpdatePCR(uiPcr/45, index);
				break;
			}
		}
	}
	return 0;
}

int TSDEMUX_Open(int index)
{
	int	i, fFirst;

	if(index >= MAX_PCR_CNT) {
		printk("In %s, PCR index=%d is out of bound\n", __func__, index);
		return 0;
	}
	gstPCR[index].uiSTCBase = 0;
	gstPCR[index].uiPCR = 0;
	gstPCR[index].fUse = 1;

	fFirst = 0;
	for (i=0; i < MAX_PCR_CNT; i++)
		fFirst += gstPCR[index].fUse ? 1 : 0;
//	if (fFirst == 1)
//	TCCREFTIME_Open();
	return 0;
}

void TSDEMUX_Close(void)
{
	int	i;

	for(i=0; i < MAX_PCR_CNT; i++)
		gstPCR[i].fUse = 0;

//	TCCREFTIME_Close();
}

EXPORT_SYMBOL(TSDEMUX_UpdatePCR);
EXPORT_SYMBOL(TSDEMUX_GetSTC);
EXPORT_SYMBOL(TSDEMUX_Open);
EXPORT_SYMBOL(TSDEMUX_Close);

