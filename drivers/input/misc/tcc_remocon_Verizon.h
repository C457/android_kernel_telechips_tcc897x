/*
 * IR driver for remote controller : tcc_remocon_CS_2000_IR_X_CANVAS.h
 *
 * Copyright (C) 2010 Telechips, Inc.
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
#include <linux/input.h>

#ifndef __TCC_REMOCON_CS_2000_H__
#define __TCC_REMOCON_CS_2000_H__

//*******************************************
//	Remote controller define
//*******************************************

#define IR_SIGNAL_MAXIMUN_BIT_COUNT	32
#define IR_FIFO_READ_COUNT			17
#define IR_BUFFER_BIT_SHIFT			16
#define IR_ID_MASK				0xFFFF0000
#define IR_CODE_MASK			0x0000FFFF

#define LOW_MIN_TCC892X		0x0100	
#define LOW_MAX_TCC892X		0x6000 
#define HIGH_MIN_TCC892X	0x6000
#define HIGH_MAX_TCC892X	0xd000
#define START_MIN_TCC892X	0x8800
#define START_MAX_TCC892X	0x9600
#define REPEAT_MIN_TCC892X	0x4900
#define REPEAT_MAX_TCC892X	0x4f00


/*****************************************************************************
*
* IR remote controller scancode
*
******************************************************************************/
#define REM_ID_ERROR		0xff

#define REMOCON_ID			0xf123	//vendor ID, prodcut ID
#define REMOCON_REPEAT		0x00000004

#define SCAN_POWER			0xf708

//#define SCAN_NUM_0			0x0000
#define SCAN_NUM_1			0xf001
#define SCAN_NUM_2			0xe002
#define SCAN_NUM_3			0xd003
#define SCAN_NUM_4			0xc004
#define SCAN_NUM_5			0xb005
#define SCAN_NUM_6			0xa006
#define SCAN_NUM_7			0x9007
#define SCAN_NUM_8			0x8008
#define SCAN_NUM_9			0x7009
#define SCAN_KPASTERISK		0x8044
#define SCAN_SLASH			0xc040

#define SCAN_UP				0x9034
#define SCAN_DOWN			0x8035
#define SCAN_LEFT			0x7036
#define SCAN_RIGHT			0x6037
#define SCAN_ENTER			0xe011	//center, enter

#define SCAN_STOP			0x301c
#define SCAN_PLAY			0x401b
#define SCAN_PAUSE			0x001f
#define SCAN_FB				0x101e
#define SCAN_FF				0x201d
#define SCAN_LAST			0xc013
#define SCAN_NEXT			0xffff	//no mapping
#define SCAN_REC			0xc031
#define SCAN_CURRENT		0x619e//no mapping

#define SCAN_VOLUP			0x300d
#define SCAN_VOLDN			0x200e

#define SCAN_MENU			0x6019
#define SCAN_SLEEP			0xf10e//no mapping
#define SCAN_CANCEL			0xd012
#define SCAN_GUIDE			0x56a9//no mapping

#define SCAN_NUM_MINUS		0xb34c//no mapping
#define SCAN_NUM_PREV		0xe51a//no mapping

#define SCAN_MUTE			0x100f
#define SCAN_INPUT			0xf40b

#define SCAN_CH_UP			0x500b
#define SCAN_CH_DN			0x400c

#define SCAN_AUTO_CHANNEL	0xab54 //no mapping
#define SCAN_ADD_DELETE		0xaa55//no mapping
#define SCAN_SIZE			0x8877//no mapping
#define SCAN_MULTI_SOUND	0xf50a//no mapping

/*****************************************************************************
*
* External Variables
*
******************************************************************************/
static SCANCODE_MAPPING key_mapping[] = 
{
#if 0 // android
	{SCAN_POWER, 			REM_POWER},

	{SCAN_NUM_1,		REM_1},
	{SCAN_NUM_2,		REM_2},
	{SCAN_NUM_3,		REM_3},
	{SCAN_NUM_4,		REM_4},
	{SCAN_NUM_5,		REM_5},
	{SCAN_NUM_6,		REM_6},
	{SCAN_NUM_7,		REM_7},
	{SCAN_NUM_8,		REM_8},
	{SCAN_NUM_9,		REM_9},
	{SCAN_NUM_0,		REM_0},

	{SCAN_UP,			REM_DPAD_UP},
	{SCAN_DOWN,			REM_DPAD_DOWN},	
	{SCAN_LEFT,			REM_DPAD_LEFT},
	{SCAN_RIGHT,		REM_DPAD_RIGHT},
#if 0
	{SCAN_ENTER,		REM_DPAD_CENTER},
#else
	{SCAN_ENTER,		REM_ENTER},
#endif

	{SCAN_STOP, 		REM_MEDIA_STOP},
	{SCAN_PLAY,			REM_MEDIA_PLAY},
	{SCAN_PAUSE,		REM_MEDIA_PAUSE},
	{SCAN_REC,			REM_MEDIA_RECORD},
#if 1
	{SCAN_FB,			REM_MEDIA_REWIND},
	{SCAN_FF,			REM_MEDIA_FAST_FORWARD},
#else
	{SCAN_FB,			REM_MEDIA_PREVIOUS},
	{SCAN_FF,			REM_MEDIA_NEXT},
#endif

	{SCAN_VOLUP,		REM_VOLUME_UP},
	{SCAN_VOLDN,		REM_VOLUME_DOWN},

	{SCAN_NUM_MINUS,	REM_TV},
	{SCAN_NUM_PREV,		REM_PERIOD},

	{SCAN_MUTE,			REM_VOLUME_MUTE},
	
	{SCAN_CH_UP,		REM_PAGE_UP},
	{SCAN_CH_DN,		REM_PAGE_DOWN},

	{SCAN_CANCEL,		REM_BACK},
	{SCAN_MENU,			REM_MENU},
	{SCAN_GUIDE,		REM_HOME},
	{SCAN_ADD_DELETE,	REM_DEL},

	{SCAN_INPUT,		REM_FUNCTION_F3},
	{SCAN_AUTO_CHANNEL,	REM_FUNCTION_F2},
	{SCAN_SLEEP,		REM_BOOKMARK},
	{SCAN_SIZE,			REM_FUNCTION},
	{SCAN_MULTI_SOUND,	REM_LANGUAGE_SWITCH},

	{SCAN_CURRENT,		REM_GUIDE},
#endif
	{SCAN_POWER,        KEY_POWER},

    {SCAN_NUM_1,        KEY_1},
    {SCAN_NUM_2,        KEY_2},
    {SCAN_NUM_3,        KEY_3},
    {SCAN_NUM_4,        KEY_4},
    {SCAN_NUM_5,        KEY_5},
    {SCAN_NUM_6,        KEY_6},
    {SCAN_NUM_7,        KEY_7},
    {SCAN_NUM_8,        KEY_8},
    {SCAN_NUM_9,        KEY_9},
//    {SCAN_NUM_0,        KEY_0},
	{SCAN_KPASTERISK,		KEY_KPASTERISK},
	{SCAN_SLASH,		KEY_SLASH},	

    {SCAN_UP,           KEY_UP},
    {SCAN_DOWN,         KEY_DOWN},
    {SCAN_LEFT,         KEY_LEFT},
    {SCAN_RIGHT,        KEY_RIGHT},

    {SCAN_ENTER,        KEY_ENTER},
 
    {SCAN_STOP,         KEY_S},
    {SCAN_PLAY,         KEY_P},
    {SCAN_PAUSE,        KEY_U},
    {SCAN_REC,          KEY_R},

    {SCAN_FB,           KEY_W},
    {SCAN_FF,           KEY_F},

    {SCAN_VOLUP,        KEY_VOLUMEUP},
    {SCAN_VOLDN,        KEY_VOLUMEDOWN},

    {SCAN_NUM_MINUS,    KEY_MINUS},
    {SCAN_NUM_PREV,     KEY_BACKSPACE},

    {SCAN_MUTE,         KEY_MUTE},
 
    {SCAN_CH_UP,        KEY_KPPLUS},
    {SCAN_CH_DN,        KEY_KPMINUS},
    {SCAN_CANCEL,       KEY_ESC},
    {SCAN_MENU,         KEY_MENU},
    {SCAN_GUIDE,        KEY_F5},
    {SCAN_ADD_DELETE,   KEY_F7},
 
    {SCAN_INPUT,        KEY_F2},
    {SCAN_AUTO_CHANNEL, KEY_F6},
    {SCAN_SLEEP,        KEY_F3},
    {SCAN_SIZE,         KEY_F8},
    {SCAN_MULTI_SOUND,  KEY_F9},
 
    {SCAN_CURRENT,      KEY_F10},
};

#endif	// __TCC_REMOCON_CS_2000_H__

