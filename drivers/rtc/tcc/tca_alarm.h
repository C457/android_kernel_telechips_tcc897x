
/****************************************************************************
 *   FileName    : tca_alarm.h
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
* Defines
*
******************************************************************************/
#ifndef __TCA_ALARAM_H__
#define __TCA_ALARAM_H__

#include "tca_rtc.h"

/*****************************************************************************
*
* Enum
*
******************************************************************************/


/*****************************************************************************
*
* Type Defines
*
******************************************************************************/

/*****************************************************************************
*
* Structures
*
******************************************************************************/


/*****************************************************************************
*
* External Variables
*
******************************************************************************/

/*****************************************************************************
*
* External Functions
*
******************************************************************************/
#ifdef __cplusplus
extern 
"C" { 
#endif

volatile int tca_alarm_gettime(unsigned int rtcbaseaddress, rtctime *pTime);
volatile int tca_alarm_settime(unsigned int rtcbaseaddress, rtctime *pTime);
volatile int tca_alarm_setint(unsigned int rtcbaseaddress);
volatile int tca_alarm_disable(unsigned int rtcbaseaddress);

#ifdef __cplusplus
 } 
#endif

#endif //__TCA_ALARAM_H__

