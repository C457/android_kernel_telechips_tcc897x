
/****************************************************************************
 *   FileName    : tca_alarm.C
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
* Header Files Include
*
******************************************************************************/
#include <linux/kernel.h>
#include <asm/io.h>
#include "tca_alarm.h"
#include <asm/mach-types.h>


/*****************************************************************************
* Function Name : tca_rtcgettime()
******************************************************************************/
volatile int tca_alarm_gettime(unsigned int rtcbaseaddress, rtctime *pTime)
{
	unsigned uCON;
	PRTC	pRTC = (PRTC)rtcbaseaddress;

#if 0
	BITSET(pRTC->RTCCON, Hw1);	// RTC Register write enabled
	BITSET(pRTC->INTCON, Hw0);	// Interrupt Block Write Enable
#endif

	pTime->wSecond			= pRTC->ALMSEC;
	pTime->wMinute			= pRTC->ALMMIN;
	pTime->wHour			= pRTC->ALMHOUR;
	pTime->wDayOfWeek 		= pRTC->ALMDAY;
	pTime->wDay				= pRTC->ALMDATE;
	pTime->wMonth			= pRTC->ALMMON;
	pTime->wYear			= pRTC->ALMYEAR;
	uCON = pRTC->RTCALM;

	BITCLR(pRTC->INTCON, Hw0);	// Interrupt Block Write Disable
	BITCLR(pRTC->RTCCON, Hw1);	// RTC Register write Disable

	/* Second */
	if (ISZERO(uCON, Hw0))
		pTime->wSecond	= (unsigned char) 0;
	else
		pTime->wSecond	= FROM_BCD( pTime->wSecond );

	/* Minute */
	if (ISZERO(uCON, Hw1))
		pTime->wMinute	= (unsigned char) 0;
	else
		pTime->wMinute	= FROM_BCD( pTime->wMinute );

	/* Hour */
	if (ISZERO(uCON, Hw2))
		pTime->wHour	= (unsigned char) 0;
	else
		pTime->wHour	= FROM_BCD( pTime->wHour );

	/* date */
	if (ISZERO(uCON, Hw3))
		pTime->wDay	= (unsigned char) 0;
	else
		pTime->wDay	= FROM_BCD( pTime->wDay );

	/* month */
	if (ISZERO(uCON, Hw5))
		pTime->wMonth	= (unsigned char) 0;
	else
		pTime->wMonth	= FROM_BCD( pTime->wMonth );

	/* year */
	if (ISZERO(uCON, Hw6))
		pTime->wYear	= (unsigned short) 0;
	else
//		pTime->wYear	= FROM_BCD( pTime->wYear );
		pTime->wYear	= (FROM_BCD( pTime->wYear>>8 )*100) + FROM_BCD( pTime->wYear&0x00FF );	// YearFix : hjbae

	/* weekdays */
	if (ISZERO(uCON, Hw4))
		pTime->wDayOfWeek		= 1;
	else
		pTime->wDayOfWeek = FROM_BCD( pTime->wDayOfWeek );

	pRTC->RTCALM  = uCON;

	return 0;
}

/*****************************************************************************
* Function Name : tca_rtcsettime()
******************************************************************************/
volatile int tca_alarm_settime(unsigned int rtcbaseaddress, rtctime *pTime)
{
	unsigned	uCON;
	PRTC	pRTC = (PRTC)rtcbaseaddress;

	BITSET(pRTC->RTCCON, Hw1);				// RTC Register write enabled
	BITSET(pRTC->INTCON, Hw0);				// Interrupt Block & Write Enable

	uCON	= 0xEF; // Not wDayOfWeek
	/* Second */
//	if (pTime->wSecond == 0)
//		pTime->wSecond = 1;

	if ( pTime->wSecond > 59 )
		BITCLR(uCON, Hw0);//HwRTCALM_SECEN_EN
	else
		pRTC->ALMSEC	= TO_BCD( pTime->wSecond );

	/* Minute */
	if ( pTime->wMinute > 59 )
		BITCLR(uCON, Hw1);//HwRTCALM_MINEN_EN
	else
		pRTC->ALMMIN	= TO_BCD( pTime->wMinute );

	/* Hour */
	if ( pTime->wHour > 23 )
		BITCLR(uCON, Hw2);//HwRTCALM_HOUREN_EN
	else
		pRTC->ALMHOUR	= TO_BCD( pTime->wHour );

	/* Date */
	if ( pTime->wDay > 31 || pTime->wDay < 1 )
		BITCLR(uCON, Hw3);//HwRTCALM_DATEEN_EN
	else
		pRTC->ALMDATE	= TO_BCD( pTime->wDay );

	/* month */
	if (pTime->wMonth > 12 || pTime->wMonth < 1)
		BITCLR(uCON, Hw5);//HwRTCALM_MONEN_EN
	else
		pRTC->ALMMON	= TO_BCD( pTime->wMonth );

	/* year */
	if (pTime->wYear > 2099 || pTime->wYear < 1900)
		BITCLR(uCON, Hw6);//HwRTCALM_YEAREN_EN
	else
//		pRTC->ALMYEAR	= TO_BCD( pTime->wYear );
		pRTC->ALMYEAR	= (TO_BCD( pTime->wYear/100 ) << 8) | TO_BCD( pTime->wYear%100 );	// YearFix : hjbae

	/* Day */
	if ( pTime->wDayOfWeek > 6)
		BITCLR(uCON, Hw4);//HwRTCALM_DAYEN_EN
	else
		pRTC->ALMDAY	= pTime->wDayOfWeek+1;


	// Enable ALARM
	pRTC->RTCALM	= uCON;

	BITSET(pRTC->RTCIM, Hw2|Hw1|Hw0);		// Active High / Level Trigger / Enable Alarm Interrupt

	BITCLR(pRTC->RTCPEND, 0xFFFFFFFF);		// Clear Alarm Wake-Up Pin Output.
	BITCLR(pRTC->RTCSTR, Hw3|Hw2|Hw1|Hw0);	// Clear Alarm Wake-Up Count value.	
	BITSET(pRTC->RTCSTR, Hw7|Hw6);			// Clear Alarm, wake-up interrupt pending

	BITSET(pRTC->RTCCON, Hw7|Hw6);  		// Alarm Interrupt Output Enable (Hw6) Wake-up Output Enable (Hw7)

	if( !(pRTC->INTCON & Hw15) )			// If protection enable bit is Disabled,
	{
		BITSET(pRTC->RTCCON,Hw0);			// RTC Start bit - Halt
		BITSET(pRTC->INTCON,Hw15);			// protect bit - enable
		BITCLR(pRTC->RTCCON,Hw0);			// RTC Start bit - Run
	}

	//printk("RTC[%02X] INT[%04X] ALM[%02X] IM[%01X] PEND[%01X] STR[%02X]\n", pRTC->RTCCON, pRTC->INTCON, pRTC->RTCALM, pRTC->RTCIM, pRTC->RTCPEND, pRTC->RTCSTR);
	//printk("\n%04x.%02x.%02x-%02x:%02x:%02x", pRTC->ALMYEAR, pRTC->ALMMON, pRTC->ALMDATE, pRTC->ALMHOUR, pRTC->ALMMIN, pRTC->ALMSEC);


	BITCLR(pRTC->INTCON, Hw0);				// Interrupt Block Write Enable bit - disable
	BITCLR(pRTC->RTCCON, Hw1);				// RTC write enable bit - disable

	return 0;
}

/************************************************************************************************
* FUNCTION		:  tca_alarm_setint
*
* DESCRIPTION	:
*
************************************************************************************************/
volatile int tca_alarm_setint(unsigned int rtcbaseaddress)
{
	volatile rtctime lpTime;

	//Set Alarm
	tca_rtc_gettime(rtcbaseaddress, (rtctime *)&lpTime);

	if(lpTime.wSecond < 55)
		lpTime.wSecond += 5;

	tca_alarm_settime(rtcbaseaddress, (rtctime *)&lpTime);

	return 0;
}

/************************************************************************************************
* FUNCTION		:  tca_alarm_disable
*
* DESCRIPTION	:  Disable the RTC Alarm during the power off state
*
************************************************************************************************/
volatile int tca_alarm_disable(unsigned int rtcbaseaddress)
{
	PRTC pRTC = (PRTC)rtcbaseaddress;

	if (pRTC == NULL) {
		printk("failed RTC ioremap()\n");
	}
	else {
		BITCLR(pRTC->RTCCON, Hw7|Hw6);	// Disable - Wake Up Interrupt Output(Hw7), Alarm Interrupt Output(Hw6)

		BITSET(pRTC->RTCCON, Hw1);	// Enable - RTC Write
		BITSET(pRTC->INTCON, Hw0);	// Enable - Interrupt Block Write

		BITCLR(pRTC->RTCALM, Hw7|Hw6|Hw5|Hw4|Hw3|Hw2|Hw1|Hw0);	// Disable - Alarm Control

		BITCSET(pRTC->RTCIM, Hw3|Hw2|Hw1|Hw0, Hw2);	// Power down mode, Active HIGH, Disable alarm interrupt

		BITCLR(pRTC->INTCON, Hw0);	// Disable - Interrupt Block Write
		BITCLR(pRTC->RTCCON, Hw1);	// Disable - RTC Write
	}

	return 0;
}
