/****************************************************************************
One line to give the program's name and a brief idea of what it does.
Copyright (C) 2013 Telechips Inc.

This program is free software; you can redistribute it and/or modify it under the terms
of the GNU General Public License as published by the Free Software Foundation;
either version 2 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
this program; if not, write to the Free Software Foundation, Inc., 59 Temple Place,
Suite 330, Boston, MA 02111-1307 USA
****************************************************************************/

#include <linux/delay.h>

//#include <asm/system.h>
#include <asm/io.h>

#include "../sensor_if.h"
#include "../cam_reg.h"
#include "../tcc_cam_i2c.h"
#include "../tcc_cam.h"
#include "../tcc_camera_device.h"


#ifdef CONFIG_VIDEO_CAMERA_SENSOR_MT9M113

/* Array of image sizes supported by MT9D111.  These must be ordered from 
 * smallest image size to largest.
 */

struct capture_size sensor_sizes_mt9m113[] = {
	//{ 1600, 1200 },	/* UXGA */
	{ 1280,  1024 },	/* SXGA */
	{ 1024,  768 },	/* XGA */
	{  800,  600 },	/* SVGA */
	{  640,  480 },	/* VGA */
	{  320,  240 },	/* QVGA */
	{  176,  144 },	/* QCIF */
};


/* register initialization tables for sensor */
/* common sensor register initialization for all image sizes, pixel formats, 
 * and frame rates
 */
static struct sensor_reg sensor_initialize_mt9m113[] = {

// Initialize_Begin
{ 0x0016, 0x00FF 	}, // CLOCKS_CONTROL
{ 0x0018, 0x0028 	}, // STANDBY_CONTROL
{ MT9M113_REG_TERM, 0x0032},	//DELAY= 50	// 50ms
{ 0x098C, 0x222D 	}, // MCU_ADDRESS
{ 0x0990, 0x0067 	}, // MCU_DATA_0
{ 0x098C, 0xA408 	}, // MCU_ADDRESS
{ 0x0990, 0x0013 	}, // MCU_DATA_0
{ 0x098C, 0xA409 	}, // MCU_ADDRESS
{ 0x0990, 0x0015 	}, // MCU_DATA_0
{ 0x098C, 0xA40A 	}, // MCU_ADDRESS
{ 0x0990, 0x0017 	}, // MCU_DATA_0
{ 0x098C, 0xA40B 	}, // MCU_ADDRESS
{ 0x0990, 0x0019 	}, // MCU_DATA_0
{ 0x098C, 0x2411 	}, // MCU_ADDRESS
{ 0x0990, 0x0067 	}, // MCU_DATA_0
{ 0x098C, 0x2413 	}, // MCU_ADDRESS
{ 0x0990, 0x007C 	}, // MCU_DATA_0
{ 0x098C, 0x2415 	}, // MCU_ADDRESS
{ 0x0990, 0x0067 	}, // MCU_DATA_0
{ 0x098C, 0x2417 	}, // MCU_ADDRESS
{ 0x0990, 0x007C 	}, // MCU_DATA_0
{ 0x0016, 0x00FF 	}, // CLOCKS_CONTROL
{ 0x0018, 0x0008 	}, // STANDBY_CONTROL
{ 0x001A, 0x0210 	}, // RESET_AND_MISC_CONTROL
{ MT9M113_REG_TERM, 0x000A},	//DELAY= 10	// 10ms
{ 0x0014, 0xB047 	}, // PLL_CONTROL
{ 0x0014, 0xB045 	}, // PLL_CONTROL
{ 0x0014, 0x2145 	}, // PLL_CONTROL
{ 0x0010, 0x0212 	}, // PLL_DIVIDERS
{ 0x0012, 0x0031 	}, // PLL_P_DIVIDERS
{ 0x0014, 0x2545 	}, // PLL_CONTROL
{ 0x0014, 0x2547 	}, // PLL_CONTROL
{ 0x0014, 0x3447 	}, // PLL_CONTROL
{ MT9M113_REG_TERM, 0x000A},	//DELAY= 10	}, // 10ms
{ 0x0014, 0x3047 	}, // PLL_CONTROL
{ 0x0014, 0x3046 	}, // PLL_CONTROL
{ 0x001E, 0x0400 	}, // PAD_SLEW		// before 0x001E, 0x0400
{ 0x098C, 0x2703 	}, // MCU_ADDRESS
{ 0x0990, 0x0280 	}, // MCU_DATA_0
{ 0x098C, 0x2705 	}, // MCU_ADDRESS
{ 0x0990, 0x0200 	}, // MCU_DATA_0
{ 0x098C, 0x2707 	}, // MCU_ADDRESS
{ 0x0990, 0x0500 	}, // MCU_DATA_0
{ 0x098C, 0x2709 	}, // MCU_ADDRESS
{ 0x0990, 0x0400 	}, // MCU_DATA_0
{ 0x098C, 0x270D 	}, // MCU_ADDRESS
{ 0x0990, 0x0000 	}, // MCU_DATA_0
{ 0x098C, 0x270F 	}, // MCU_ADDRESS
{ 0x0990, 0x0000 	}, // MCU_DATA_0
{ 0x098C, 0x2711 	}, // MCU_ADDRESS
{ 0x0990, 0x040D 	}, // MCU_DATA_0
{ 0x098C, 0x2713 	}, // MCU_ADDRESS
{ 0x0990, 0x050D 	}, // MCU_DATA_0
{ 0x098C, 0x2715 	}, // MCU_ADDRESS
{ 0x0990, 0x2111 	}, // MCU_DATA_0
{ 0x098C, 0x2717 	}, // MCU_ADDRESS
{ 0x0990, 0x046C 	}, // MCU_DATA_0
{ 0x098C, 0x2719 	}, // MCU_ADDRESS
{ 0x0990, 0x00AC 	}, // MCU_DATA_0
{ 0x098C, 0x271B 	}, // MCU_ADDRESS
{ 0x0990, 0x01F1 	}, // MCU_DATA_0
{ 0x098C, 0x271D 	}, // MCU_ADDRESS
{ 0x0990, 0x013F 	}, // MCU_DATA_0
{ 0x098C, 0x271F 	}, // MCU_ADDRESS
{ 0x0990, 0x0291 	}, // MCU_DATA_0
{ 0x098C, 0x2721 	}, // MCU_ADDRESS
{ 0x0990, 0x0722 	}, // MCU_DATA_0
{ 0x098C, 0x2723 	}, // MCU_ADDRESS
{ 0x0990, 0x0004 	}, // MCU_DATA_0
{ 0x098C, 0x2725 	}, // MCU_ADDRESS
{ 0x0990, 0x0004 	}, // MCU_DATA_0
{ 0x098C, 0x2727 	}, // MCU_ADDRESS
{ 0x0990, 0x040B 	}, // MCU_DATA_0
{ 0x098C, 0x2729 	}, // MCU_ADDRESS
{ 0x0990, 0x050B 	}, // MCU_DATA_0
{ 0x098C, 0x272B 	}, // MCU_ADDRESS
{ 0x0990, 0x2111 	}, // MCU_DATA_0
{ 0x098C, 0x272D 	}, // MCU_ADDRESS
{ 0x0990, 0x0024 	}, // MCU_DATA_0
{ 0x098C, 0x272F 	}, // MCU_ADDRESS
{ 0x0990, 0x004C 	}, // MCU_DATA_0
{ 0x098C, 0x2731 	}, // MCU_ADDRESS
{ 0x0990, 0x00F9 	}, // MCU_DATA_0
{ 0x098C, 0x2733 	}, // MCU_ADDRESS
{ 0x0990, 0x00A7 	}, // MCU_DATA_0
{ 0x098C, 0x2735 	}, // MCU_ADDRESS
{ 0x0990, 0x048E 	}, // MCU_DATA_0
{ 0x098C, 0x2737 	}, // MCU_ADDRESS
{ 0x0990, 0x0722 	}, // MCU_DATA_0
{ 0x098C, 0x2739 	}, // MCU_ADDRESS
{ 0x0990, 0x0000 	}, // MCU_DATA_0
{ 0x098C, 0x273B 	}, // MCU_ADDRESS
{ 0x0990, 0x027F 	}, // MCU_DATA_0
{ 0x098C, 0x273D 	}, // MCU_ADDRESS
{ 0x0990, 0x0000 	}, // MCU_DATA_0
{ 0x098C, 0x273F 	}, // MCU_ADDRESS
{ 0x0990, 0x01FF 	}, // MCU_DATA_0
{ 0x098C, 0x2747 	}, // MCU_ADDRESS
{ 0x0990, 0x0000 	}, // MCU_DATA_0
{ 0x098C, 0x2749 	}, // MCU_ADDRESS
{ 0x0990, 0x04FF 	}, // MCU_DATA_0
{ 0x098C, 0x274B 	}, // MCU_ADDRESS
{ 0x0990, 0x0000 	}, // MCU_DATA_0
{ 0x098C, 0x274D 	}, // MCU_ADDRESS
{ 0x0990, 0x03FF 	}, // MCU_DATA_0
{ 0x098C, 0x222D 	}, // MCU_ADDRESS
{ 0x0990, 0x0052 	}, // MCU_DATA_0
{ 0x098C, 0xA408 	}, // MCU_ADDRESS
{ 0x0990, 0x0013 	}, // MCU_DATA_0
{ 0x098C, 0xA409 	}, // MCU_ADDRESS
{ 0x0990, 0x0016 	}, // MCU_DATA_0
{ 0x098C, 0xA40A 	}, // MCU_ADDRESS
{ 0x0990, 0x0017 	}, // MCU_DATA_0
{ 0x098C, 0xA40B 	}, // MCU_ADDRESS
{ 0x0990, 0x001A 	}, // MCU_DATA_0
{ 0x098C, 0x2411 	}, // MCU_ADDRESS
{ 0x0990, 0x0052 	}, // MCU_DATA_0
{ 0x098C, 0x2413 	}, // MCU_ADDRESS
{ 0x0990, 0x0063 	}, // MCU_DATA_0
{ 0x098C, 0x2415 	}, // MCU_ADDRESS
{ 0x0990, 0x0052 	}, // MCU_DATA_0
{ 0x098C, 0x2417 	}, // MCU_ADDRESS
{ 0x0990, 0x0063 	}, // MCU_DATA_0
{ 0x098C, 0xA404 	}, // MCU_ADDRESS
{ 0x0990, 0x0010 	}, // MCU_DATA_0
{ 0x098C, 0xA40D 	}, // MCU_ADDRESS
{ 0x0990, 0x0002 	}, // MCU_DATA_0
{ 0x098C, 0xA40E 	}, // MCU_ADDRESS
{ 0x0990, 0x0003 	}, // MCU_DATA_0
{ 0x098C, 0xA410 	}, // MCU_ADDRESS
{ 0x0990, 0x000A 	}, // MCU_DATA_0
{ 0x098C, 0xA103 	}, // MCU_ADDRESS
{ 0x0990, 0x0006 	}, // MCU_DATA_0
{ 0x098C, 0xA103 	}, // MCU_ADDRESS
{ 0x0990, 0x0005 	}, // MCU_DATA_0
{ 0x098C, 0xA11E 	}, // MCU_ADDRESS
{ 0x0990, 0x0001 	}, // MCU_DATA_0
{ 0x098C, 0xA404 	}, // MCU_ADDRESS
{ 0x0990, 0x0000 	}, // MCU_DATA_0
{ 0x098C, 0xA109 	}, // MCU_ADDRESS
{ 0x0990, 0x0004 	}, // MCU_DATA_0
{ 0x098C, 0xA10A 	}, // MCU_ADDRESS
{ 0x0990, 0x0001 	}, // MCU_DATA_0
{ 0x098C, 0xA112 	}, // MCU_ADDRESS
{ 0x0990, 0x000A 	}, // MCU_DATA_0
{ 0x098C, 0xAB20 	}, // MCU_ADDRESS
{ 0x0990, 0x0043 	}, // MCU_DATA_0
{ 0x098C, 0xAB21 	}, // MCU_ADDRESS
{ 0x0990, 0x0020 	}, // MCU_DATA_0
{ 0x098C, 0xAB22 	}, // MCU_ADDRESS
{ 0x0990, 0x0007 	}, // MCU_DATA_0
{ 0x098C, 0xAB23 	}, // MCU_ADDRESS
{ 0x0990, 0x0000 	}, // MCU_DATA_0
{ 0x098C, 0xAB24 	}, // MCU_ADDRESS
{ 0x0990, 0x000C 	}, // MCU_DATA_0
{ 0x098C, 0xAB25 	}, // MCU_ADDRESS
{ 0x0990, 0x002A 	}, // MCU_DATA_0
{ 0x098C, 0xAB26 	}, // MCU_ADDRESS
{ 0x0990, 0x0000 	}, // MCU_DATA_0
{ 0x098C, 0xAB27 	}, // MCU_ADDRESS
{ 0x0990, 0x0005 	}, // MCU_DATA_0
{ 0x098C, 0xAB2D 	}, // MCU_ADDRESS
{ 0x0990, 0x002A 	}, // MCU_DATA_0
{ 0x098C, 0xAB2F 	}, // MCU_ADDRESS
{ 0x0990, 0x002E 	}, // MCU_DATA_0
{ 0x098C, 0x2B28 	}, // MCU_ADDRESS
{ 0x0990, 0x157C 	}, // MCU_DATA_0
{ 0x098C, 0x2B2A 	}, // MCU_ADDRESS
{ 0x0990, 0x32C8 	}, // MCU_DATA_0
{ 0x098C, 0x2B38 	}, // MCU_ADDRESS
{ 0x0990, 0x1F40 	}, // MCU_DATA_0
{ 0x098C, 0x2B3A 	}, // MCU_ADDRESS
{ 0x0990, 0x3A98 	}, // MCU_DATA_0
{ 0x098C, 0x2257 	}, // MCU_ADDRESS
{ 0x0990, 0x2710 	}, // MCU_DATA_0
{ 0x098C, 0x2250 	}, // MCU_ADDRESS
{ 0x0990, 0x1B58 	}, // MCU_DATA_0
{ 0x098C, 0x2252 	}, // MCU_ADDRESS
{ 0x0990, 0x32C8 	}, // MCU_DATA_0
{ 0x3658, 0x0310 	}, // P_RD_P0Q0
{ 0x365A, 0xB9AD 	}, // P_RD_P0Q1
{ 0x365C, 0x5090 	}, // P_RD_P0Q2
{ 0x365E, 0x16CD 	}, // P_RD_P0Q3
{ 0x3660, 0xCA8F 	}, // P_RD_P0Q4
{ 0x3680, 0x56EA 	}, // P_RD_P1Q0
{ 0x3682, 0xB0AA 	}, // P_RD_P1Q1
{ 0x3684, 0x2FEC 	}, // P_RD_P1Q2
{ 0x3686, 0x19ED 	}, // P_RD_P1Q3
{ 0x3688, 0xB86E 	}, // P_RD_P1Q4
{ 0x36A8, 0x20B0 	}, // P_RD_P2Q0
{ 0x36AA, 0xB74E 	}, // P_RD_P2Q1
{ 0x36AC, 0xCFD1 	}, // P_RD_P2Q2
{ 0x36AE, 0xDF2E 	}, // P_RD_P2Q3
{ 0x36B0, 0x14B3 	}, // P_RD_P2Q4
{ 0x36D0, 0x39AD 	}, // P_RD_P3Q0
{ 0x36D2, 0xE3ED 	}, // P_RD_P3Q1
{ 0x36D4, 0xE2ED 	}, // P_RD_P3Q2
{ 0x36D6, 0xCBEB 	}, // P_RD_P3Q3
{ 0x36D8, 0x7B0E 	}, // P_RD_P3Q4
{ 0x36F8, 0x980F 	}, // P_RD_P4Q0
{ 0x36FA, 0x34AF 	}, // P_RD_P4Q1
{ 0x36FC, 0x42B3 	}, // P_RD_P4Q2
{ 0x36FE, 0x2B91 	}, // P_RD_P4Q3
{ 0x3700, 0x81B5 	}, // P_RD_P4Q4
{ 0x364E, 0x0370 	}, // P_GR_P0Q0
{ 0x3650, 0x2B2E 	}, // P_GR_P0Q1
{ 0x3652, 0x6B70 	}, // P_GR_P0Q2
{ 0x3654, 0xFF4A 	}, // P_GR_P0Q3
{ 0x3656, 0x9CF0 	}, // P_GR_P0Q4
{ 0x3676, 0x87AB 	}, // P_GR_P1Q0
{ 0x3678, 0x03AD 	}, // P_GR_P1Q1
{ 0x367A, 0x474A 	}, // P_GR_P1Q2
{ 0x367C, 0xBF4B 	}, // P_GR_P1Q3
{ 0x367E, 0x8D2B 	}, // P_GR_P1Q4
{ 0x369E, 0x0DF0 	}, // P_GR_P2Q0
{ 0x36A0, 0x930E 	}, // P_GR_P2Q1
{ 0x36A2, 0x8FD2 	}, // P_GR_P2Q2
{ 0x36A4, 0xDB0E 	}, // P_GR_P2Q3
{ 0x36A6, 0x21D3 	}, // P_GR_P2Q4
{ 0x36C6, 0x370E 	}, // P_GR_P3Q0
{ 0x36C8, 0x984E 	}, // P_GR_P3Q1
{ 0x36CA, 0xF60E 	}, // P_GR_P3Q2
{ 0x36CC, 0x438D 	}, // P_GR_P3Q3
{ 0x36CE, 0x3B8E 	}, // P_GR_P3Q4
{ 0x36EE, 0x81F0 	}, // P_GR_P4Q0
{ 0x36F0, 0x7B30 	}, // P_GR_P4Q1
{ 0x36F2, 0x5773 	}, // P_GR_P4Q2
{ 0x36F4, 0x5CAE 	}, // P_GR_P4Q3
{ 0x36F6, 0xF474 	}, // P_GR_P4Q4
{ 0x3662, 0x0410 	}, // P_BL_P0Q0
{ 0x3664, 0x2FEE 	}, // P_BL_P0Q1
{ 0x3666, 0x3730 	}, // P_BL_P0Q2
{ 0x3668, 0xFE8D 	}, // P_BL_P0Q3
{ 0x366A, 0xE56F 	}, // P_BL_P0Q4
{ 0x368A, 0x99EC 	}, // P_BL_P1Q0
{ 0x368C, 0xD68B 	}, // P_BL_P1Q1
{ 0x368E, 0xC6AD 	}, // P_BL_P1Q2
{ 0x3690, 0x608B 	}, // P_BL_P1Q3
{ 0x3692, 0x44ED 	}, // P_BL_P1Q4
{ 0x36B2, 0x542F 	}, // P_BL_P2Q0
{ 0x36B4, 0xC80D 	}, // P_BL_P2Q1
{ 0x36B6, 0xA151 	}, // P_BL_P2Q2
{ 0x36B8, 0xE9CE 	}, // P_BL_P2Q3
{ 0x36BA, 0x6B72 	}, // P_BL_P2Q4
{ 0x36DA, 0x4069 	}, // P_BL_P3Q0
{ 0x36DC, 0xBE4E 	}, // P_BL_P3Q1
{ 0x36DE, 0x6ECE 	}, // P_BL_P3Q2
{ 0x36E0, 0x3D0E 	}, // P_BL_P3Q3
{ 0x36E2, 0xA410 	}, // P_BL_P3Q4
{ 0x3702, 0xEC8A 	}, // P_BL_P4Q0
{ 0x3704, 0x2E50 	}, // P_BL_P4Q1
{ 0x3706, 0x68D2 	}, // P_BL_P4Q2
{ 0x3708, 0x43F1 	}, // P_BL_P4Q3
{ 0x370A, 0x81B4 	}, // P_BL_P4Q4
{ 0x366C, 0x0210 	}, // P_GB_P0Q0
{ 0x366E, 0xD0ED 	}, // P_GB_P0Q1
{ 0x3670, 0x4FD0 	}, // P_GB_P0Q2
{ 0x3672, 0x52EB 	}, // P_GB_P0Q3
{ 0x3674, 0x8BB0 	}, // P_GB_P0Q4
{ 0x3694, 0xA86C 	}, // P_GB_P1Q0
{ 0x3696, 0x494D 	}, // P_GB_P1Q1
{ 0x3698, 0xE86B 	}, // P_GB_P1Q2
{ 0x369A, 0xADAC 	}, // P_GB_P1Q3
{ 0x369C, 0xF02A 	}, // P_GB_P1Q4
{ 0x36BC, 0x12B0 	}, // P_GB_P2Q0
{ 0x36BE, 0xCE8E 	}, // P_GB_P2Q1
{ 0x36C0, 0x83B2 	}, // P_GB_P2Q2
{ 0x36C2, 0x946E 	}, // P_GB_P2Q3
{ 0x36C4, 0x1F73 	}, // P_GB_P2Q4
{ 0x36E4, 0x544D 	}, // P_GB_P3Q0
{ 0x36E6, 0xFA4E 	}, // P_GB_P3Q1
{ 0x36E8, 0x3F8B 	}, // P_GB_P3Q2
{ 0x36EA, 0x630E 	}, // P_GB_P3Q3
{ 0x36EC, 0xAA4E 	}, // P_GB_P3Q4
{ 0x370C, 0x8B90 	}, // P_GB_P4Q0
{ 0x370E, 0x4330 	}, // P_GB_P4Q1
{ 0x3710, 0x4573 	}, // P_GB_P4Q2
{ 0x3712, 0x6B90 	}, // P_GB_P4Q3
{ 0x3714, 0xECF4 	}, // P_GB_P4Q4
{ 0x3644, 0x0284 	}, // POLY_ORIGIN_C
{ 0x3642, 0x01F4 	}, // POLY_ORIGIN_R
{ 0x3210, 0x01B8 	}, // COLOR_PIPELINE_CONTROL
{ 0x098C, 0xAB04 	}, // MCU_ADDRESS
{ 0x0990, 0x0028 	}, // MCU_DATA_0
{ 0x098C, 0xAB06 	}, // MCU_ADDRESS
{ 0x0990, 0x0003 	}, // MCU_DATA_0
{ 0x098C, 0xAB3C 	}, // MCU_ADDRESS
{ 0x0990, 0x0000 	}, // MCU_DATA_0
{ 0x098C, 0xAB3D 	}, // MCU_ADDRESS
{ 0x0990, 0x000A 	}, // MCU_DATA_0
{ 0x098C, 0xAB3E 	}, // MCU_ADDRESS
{ 0x0990, 0x001D 	}, // MCU_DATA_0
{ 0x098C, 0xAB3F 	}, // MCU_ADDRESS
{ 0x0990, 0x0037 	}, // MCU_DATA_0
{ 0x098C, 0xAB40 	}, // MCU_ADDRESS
{ 0x0990, 0x0058 	}, // MCU_DATA_0
{ 0x098C, 0xAB41 	}, // MCU_ADDRESS
{ 0x0990, 0x0071 	}, // MCU_DATA_0
{ 0x098C, 0xAB42 	}, // MCU_ADDRESS
{ 0x0990, 0x0086 	}, // MCU_DATA_0
{ 0x098C, 0xAB43 	}, // MCU_ADDRESS
{ 0x0990, 0x0098 	}, // MCU_DATA_0
{ 0x098C, 0xAB44 	}, // MCU_ADDRESS
{ 0x0990, 0x00A7 	}, // MCU_DATA_0
{ 0x098C, 0xAB45 	}, // MCU_ADDRESS
{ 0x0990, 0x00B5 	}, // MCU_DATA_0
{ 0x098C, 0xAB46 	}, // MCU_ADDRESS
{ 0x0990, 0x00C0 	}, // MCU_DATA_0
{ 0x098C, 0xAB47 	}, // MCU_ADDRESS
{ 0x0990, 0x00CB 	}, // MCU_DATA_0
{ 0x098C, 0xAB48 	}, // MCU_ADDRESS
{ 0x0990, 0x00D4 	}, // MCU_DATA_0
{ 0x098C, 0xAB49 	}, // MCU_ADDRESS
{ 0x0990, 0x00DD 	}, // MCU_DATA_0
{ 0x098C, 0xAB4A 	}, // MCU_ADDRESS
{ 0x0990, 0x00E4 	}, // MCU_DATA_0
{ 0x098C, 0xAB4B 	}, // MCU_ADDRESS
{ 0x0990, 0x00EC 	}, // MCU_DATA_0
{ 0x098C, 0xAB4C 	}, // MCU_ADDRESS
{ 0x0990, 0x00F3 	}, // MCU_DATA_0
{ 0x098C, 0xAB4D 	}, // MCU_ADDRESS
{ 0x0990, 0x00F9 	}, // MCU_DATA_0
{ 0x098C, 0xAB4E 	}, // MCU_ADDRESS
{ 0x0990, 0x00FF 	}, // MCU_DATA_0
{ 0x098C, 0xAB4F 	}, // MCU_ADDRESS
{ 0x0990, 0x0000 	}, // MCU_DATA_0
{ 0x098C, 0xAB50 	}, // MCU_ADDRESS
{ 0x0990, 0x0008 	}, // MCU_DATA_0
{ 0x098C, 0xAB51 	}, // MCU_ADDRESS
{ 0x0990, 0x0015 	}, // MCU_DATA_0
{ 0x098C, 0xAB52 	}, // MCU_ADDRESS
{ 0x0990, 0x0029 	}, // MCU_DATA_0
{ 0x098C, 0xAB53 	}, // MCU_ADDRESS
{ 0x0990, 0x0044 	}, // MCU_DATA_0
{ 0x098C, 0xAB54 	}, // MCU_ADDRESS
{ 0x0990, 0x0059 	}, // MCU_DATA_0
{ 0x098C, 0xAB55 	}, // MCU_ADDRESS
{ 0x0990, 0x006C 	}, // MCU_DATA_0
{ 0x098C, 0xAB56 	}, // MCU_ADDRESS
{ 0x0990, 0x007C 	}, // MCU_DATA_0
{ 0x098C, 0xAB57 	}, // MCU_ADDRESS
{ 0x0990, 0x008B 	}, // MCU_DATA_0
{ 0x098C, 0xAB58 	}, // MCU_ADDRESS
{ 0x0990, 0x0099 	}, // MCU_DATA_0
{ 0x098C, 0xAB59 	}, // MCU_ADDRESS
{ 0x0990, 0x00A7 	}, // MCU_DATA_0
{ 0x098C, 0xAB5A 	}, // MCU_ADDRESS
{ 0x0990, 0x00B3 	}, // MCU_DATA_0
{ 0x098C, 0xAB5B 	}, // MCU_ADDRESS
{ 0x0990, 0x00BF 	}, // MCU_DATA_0
{ 0x098C, 0xAB5C 	}, // MCU_ADDRESS
{ 0x0990, 0x00CB 	}, // MCU_DATA_0
{ 0x098C, 0xAB5D 	}, // MCU_ADDRESS
{ 0x0990, 0x00D6 	}, // MCU_DATA_0
{ 0x098C, 0xAB5E 	}, // MCU_ADDRESS
{ 0x0990, 0x00E1 	}, // MCU_DATA_0
{ 0x098C, 0xAB5F 	}, // MCU_ADDRESS
{ 0x0990, 0x00EB 	}, // MCU_DATA_0
{ 0x098C, 0xAB60 	}, // MCU_ADDRESS
{ 0x0990, 0x00F5 	}, // MCU_DATA_0
{ 0x098C, 0xAB61 	}, // MCU_ADDRESS
{ 0x0990, 0x00FF 	}, // MCU_DATA_0
{ 0x098C, 0xA24F 	}, // MCU_ADDRESS
{ 0x0990, 0x004B 	}, // MCU_DATA_0
{ 0x098C, 0xA207 	}, // MCU_ADDRESS
{ 0x0990, 0x000A 	}, // MCU_DATA_0
{ 0x098C, 0x2306 	}, // MCU_ADDRESS
{ 0x0990, 0x00FC 	}, // MCU_DATA_0
{ 0x098C, 0x2308 	}, // MCU_ADDRESS
{ 0x0990, 0xFFAB 	}, // MCU_DATA_0
{ 0x098C, 0x230A 	}, // MCU_ADDRESS
{ 0x0990, 0x005D 	}, // MCU_DATA_0
{ 0x098C, 0x230C 	}, // MCU_ADDRESS
{ 0x0990, 0xFFC3 	}, // MCU_DATA_0
{ 0x098C, 0x230E 	}, // MCU_ADDRESS
{ 0x0990, 0x0151 	}, // MCU_DATA_0
{ 0x098C, 0x2310 	}, // MCU_ADDRESS
{ 0x0990, 0xFFF7 	}, // MCU_DATA_0
{ 0x098C, 0x2312 	}, // MCU_ADDRESS
{ 0x0990, 0x0014 	}, // MCU_DATA_0
{ 0x098C, 0x2314 	}, // MCU_ADDRESS
{ 0x0990, 0xFEF7 	}, // MCU_DATA_0
{ 0x098C, 0x2316 	}, // MCU_ADDRESS
{ 0x0990, 0x0205 	}, // MCU_DATA_0
{ 0x098C, 0x2318 	}, // MCU_ADDRESS
{ 0x0990, 0x0026 	}, // MCU_DATA_0
{ 0x098C, 0x231A 	}, // MCU_ADDRESS
{ 0x0990, 0x003E 	}, // MCU_DATA_0
{ 0x098C, 0x231C 	}, // MCU_ADDRESS
{ 0x0990, 0x0083 	}, // MCU_DATA_0
{ 0x098C, 0x231E 	}, // MCU_ADDRESS
{ 0x0990, 0xFFCA 	}, // MCU_DATA_0
{ 0x098C, 0x2320 	}, // MCU_ADDRESS
{ 0x0990, 0xFFB1 	}, // MCU_DATA_0
{ 0x098C, 0x2322 	}, // MCU_ADDRESS
{ 0x0990, 0x002F 	}, // MCU_DATA_0
{ 0x098C, 0x2324 	}, // MCU_ADDRESS
{ 0x0990, 0x003B 	}, // MCU_DATA_0
{ 0x098C, 0x2326 	}, // MCU_ADDRESS
{ 0x0990, 0xFF8E 	}, // MCU_DATA_0
{ 0x098C, 0x2328 	}, // MCU_ADDRESS
{ 0x0990, 0x002C 	}, // MCU_DATA_0
{ 0x098C, 0x232A 	}, // MCU_ADDRESS
{ 0x0990, 0x0092 	}, // MCU_DATA_0
{ 0x098C, 0x232C 	}, // MCU_ADDRESS
{ 0x0990, 0xFF32 	}, // MCU_DATA_0
{ 0x098C, 0x232E 	}, // MCU_ADDRESS
{ 0x0990, 0x000C 	}, // MCU_DATA_0
{ 0x098C, 0x2330 	}, // MCU_ADDRESS
{ 0x0990, 0xFFE9 	}, // MCU_DATA_0
{ 0x098C, 0xA348 	}, // MCU_ADDRESS
{ 0x0990, 0x0008 	}, // MCU_DATA_0
{ 0x098C, 0xA349 	}, // MCU_ADDRESS
{ 0x0990, 0x0002 	}, // MCU_DATA_0
{ 0x098C, 0xA34A 	}, // MCU_ADDRESS
{ 0x0990, 0x0059 	}, // MCU_DATA_0
{ 0x098C, 0xA34B 	}, // MCU_ADDRESS
{ 0x0990, 0x00C6 	}, // MCU_DATA_0
{ 0x098C, 0xA351 	}, // MCU_ADDRESS
{ 0x0990, 0x0000 	}, // MCU_DATA_0
{ 0x098C, 0xA352 	}, // MCU_ADDRESS
{ 0x0990, 0x007F 	}, // MCU_DATA_0
{ 0x098C, 0xA354 	}, // MCU_ADDRESS
{ 0x0990, 0x0043 	}, // MCU_DATA_0
{ 0x098C, 0xA355 	}, // MCU_ADDRESS
{ 0x0990, 0x0002 	}, // MCU_DATA_0
{ 0x098C, 0xA35D 	}, // MCU_ADDRESS
{ 0x0990, 0x0078 	}, // MCU_DATA_0
{ 0x098C, 0xA35E 	}, // MCU_ADDRESS
{ 0x0990, 0x0086 	}, // MCU_DATA_0
{ 0x098C, 0xA35F 	}, // MCU_ADDRESS
{ 0x0990, 0x007E 	}, // MCU_DATA_0
{ 0x098C, 0xA360 	}, // MCU_ADDRESS
{ 0x0990, 0x0082 	}, // MCU_DATA_0
{ 0x098C, 0xA302 	}, // MCU_ADDRESS
{ 0x0990, 0x0000 	}, // MCU_DATA_0
{ 0x098C, 0xA303 	}, // MCU_ADDRESS
{ 0x0990, 0x00EF 	}, // MCU_DATA_0
{ 0x098C, 0xA363 	}, // MCU_ADDRESS
{ 0x0990, 0x00CA 	}, // MCU_DATA_0
{ 0x098C, 0xA364 	}, // MCU_ADDRESS
{ 0x0990, 0x00F6 	}, // MCU_DATA_0
{ 0x098C, 0xA366 	}, // MCU_ADDRESS
{ 0x0990, 0x009A 	}, // MCU_DATA_0
{ 0x098C, 0xA367 	}, // MCU_ADDRESS
{ 0x0990, 0x0096 	}, // MCU_DATA_0
{ 0x098C, 0xA368 	}, // MCU_ADDRESS
{ 0x0990, 0x0098 	}, // MCU_DATA_0
{ 0x098C, 0xA369 	}, // MCU_ADDRESS
{ 0x0990, 0x0080 	}, // MCU_DATA_0
{ 0x098C, 0xA36A 	}, // MCU_ADDRESS
{ 0x0990, 0x007E 	}, // MCU_DATA_0
{ 0x098C, 0xA36B 	}, // MCU_ADDRESS
{ 0x0990, 0x0082 	}, // MCU_DATA_0
{ 0x098C, 0xA36D 	}, // MCU_ADDRESS
{ 0x0990, 0x0000 	}, // MCU_DATA_0
{ 0x098C, 0xA36E 	}, // MCU_ADDRESS
{ 0x0990, 0x0018 	}, // MCU_DATA_0
{ 0x098C, 0xA103 	}, // MCU_ADDRESS
{ 0x0990, 0x0005 	}, // MCU_DATA_0
{ 0x098C, 0xAB2C 	}, // MCU_ADDRESS
{ 0x0990, 0x0006 	}, // MCU_DATA_0
{ 0x098C, 0xAB2D 	}, // MCU_ADDRESS
{ 0x0990, 0x000E 	}, // MCU_DATA_0
{ 0x098C, 0xAB2E 	}, // MCU_ADDRESS
{ 0x0990, 0x0006 	}, // MCU_DATA_0
{ 0x098C, 0xAB2F 	}, // MCU_ADDRESS
{ 0x0990, 0x0006 	}, // MCU_DATA_0
{ 0x098C, 0xAB30 	}, // MCU_ADDRESS
{ 0x0990, 0x001E 	}, // MCU_DATA_0
{ 0x098C, 0xAB31 	}, // MCU_ADDRESS
{ 0x0990, 0x0015 	}, // MCU_DATA_0
{ 0x098C, 0xAB32 	}, // MCU_ADDRESS
{ 0x0990, 0x001E 	}, // MCU_DATA_0
{ 0x098C, 0xAB33 	}, // MCU_ADDRESS
{ 0x0990, 0x001E 	}, // MCU_DATA_0
{ 0x098C, 0xAB34 	}, // MCU_ADDRESS
{ 0x0990, 0x0030 	}, // MCU_DATA_0
{ 0x098C, 0xAB35 	}, // MCU_ADDRESS
{ 0x0990, 0x004D 	}, // MCU_DATA_0
{ 0x098C, 0xA20E 	}, // MCU_ADDRESS
{ 0x0990, 0x0080 	}, // MCU_DATA_0
{ 0x098C, 0xA20D 	}, // MCU_ADDRESS
{ 0x0990, 0x001A 	}, // MCU_DATA_0
{ 0x098C, 0xA11D 	}, // MCU_ADDRESS
{ 0x0990, 0x0002 	}, // MCU_DATA_0
{ 0x098C, 0xA24A 	}, // MCU_ADDRESS
{ 0x0990, 0x0040 	}, // MCU_DATA_0
{ 0x098C, 0x275F 	}, // MCU_ADDRESS
{ 0x0990, 0x0596 	}, // MCU_DATA_0
{ 0x098C, 0x2761 	}, // MCU_ADDRESS
{ 0x0990, 0x0094 	}, // MCU_DATA_0
{ 0x33F4, 0x011B 	}, // KERNEL_CONFIG
{ 0x098C, 0xA11E 	}, // MCU_ADDRESS
{ 0x0990, 0x0002 	}, // MCU_DATA_0
{ 0x098C, 0xA404 	}, // MCU_ADDRESS
{ 0x0990, 0x0080 	}, // MCU_DATA_0
{ 0x098C, 0xA202 	}, // MCU_ADDRESS
{ 0x0990, 0x0022 	}, // MCU_DATA_0
{ 0x098C, 0xA203 	}, // MCU_ADDRESS
{ 0x0990, 0x00BB 	}, // MCU_DATA_0
{ 0x098C, 0xA20C 	}, // MCU_ADDRESS
{ 0x0990, 0x0018 	}, // MCU_DATA_0
{ 0x098C, 0xA214 	}, // MCU_ADDRESS
{ 0x0990, 0x001E 	}, // MCU_DATA_0
{ 0x098C, 0xA103 	}, // MCU_ADDRESS
{ 0x0990, 0x0005 	}, // MCU_DATA_0
{ MT9M113_REG_TERM, 0x012C},	//DELAY= 300	}, // 300ms
//{ MT9M113_REG_TERM, 0x0064}, // 300ms
{ 0x098C, 0xA103 	}, // MCU_ADDRESS
{ 0x0990, 0x0006 	}, // MCU_DATA_0
{ MT9M113_REG_TERM, 0x012C},	//DELAY= 300	}, // 300ms
//{ MT9M113_REG_TERM, 0x0064}, // 300ms
// Intialize_End

    {MT9M113_REG_TERM, MT9M113_VAL_TERM}
};


static struct sensor_reg sensor_set_preview_mt9m113[] = {
	{ 0x098C, 0xA115 },	// MCU_ADDRESS [SEQ_CAP_MODE]
	{ 0x0990, 0x0000 },	// MCU_DATA_0
	{ 0x098C, 0xA103 },	// MCU_ADDRESS [SEQ_CMD]
	{ 0x0990, 0x0001 },	// MCU_DATA_0

    {MT9M113_REG_TERM, MT9M113_VAL_TERM}
};

static struct sensor_reg sensor_set_capture_mt9m113[] = {
	{ 0x098C, 0xA115 }, 	// MCU_ADDRESS [SEQ_CAP_MODE]
	{ 0x0990, 0x0002 },	// MCU_DATA_0
	{ 0x098C, 0xA103 },	// MCU_ADDRESS [SEQ_CMD]
	{ 0x0990, 0x0002 },	// MCU_DATA_0

    {MT9M113_REG_TERM, MT9M113_VAL_TERM}
};

 struct sensor_reg* sensor_reg_common_mt9m113[3] =
{
	//sensor_initialize_mt9m113_max27fps,
	sensor_initialize_mt9m113,
	sensor_set_preview_mt9m113,
	sensor_set_capture_mt9m113
};

static struct sensor_reg sensor_brightness_0_mt9m113[] = {
    {MT9M113_REG_TERM, MT9M113_VAL_TERM}
};

static struct sensor_reg sensor_brightness_1_mt9m113[] = {
    {MT9M113_REG_TERM, MT9M113_VAL_TERM}
};

static struct sensor_reg sensor_brightness_2_mt9m113[] = {
    {MT9M113_REG_TERM, MT9M113_VAL_TERM}
};

static struct sensor_reg sensor_brightness_3_mt9m113[] = {
    {MT9M113_REG_TERM, MT9M113_VAL_TERM}
};

static struct sensor_reg sensor_brightness_4_mt9m113[] = {
    {MT9M113_REG_TERM, MT9M113_VAL_TERM}
};

struct sensor_reg* sensor_reg_brightness_mt9m113[5] =
{
	sensor_brightness_0_mt9m113,
	sensor_brightness_1_mt9m113,
	sensor_brightness_2_mt9m113,
	sensor_brightness_3_mt9m113,
	sensor_brightness_4_mt9m113
};


static struct sensor_reg sensor_awb_auto_mt9m113[] = {
	{ 0x098C, 0xA355 },
	{ 0x0990, 0x0002 },
	{ 0x098C, 0xA369 },	// red
	{ 0x0990, 0x0080 },
	{ 0x098C, 0xA36B },	// blue
	{ 0x0990, 0x0082 },
	{ 0x098C, 0xA36D },
	{ 0x0990, 0x0000 },
	{ 0x098C, 0xA36E },
	{ 0x0990, 0x0018 },
    {MT9M113_REG_TERM, MT9M113_VAL_TERM}
};

static struct sensor_reg sensor_awb_daylight_mt9m113[] = {
	{ 0x098C, 0xA355 },
	{ 0x0990, 0x0002 },
	{ 0x098C, 0xA369 },	// red
	{ 0x0990, 0x0048 },
	{ 0x098C, 0xA36B },	// blue
	{ 0x0990, 0x004B },
	{ 0x098C, 0xA36D },
	{ 0x0990, 0x0000 },
	{ 0x098C, 0xA36E },
	{ 0x0990, 0x0018 },
    {MT9M113_REG_TERM, MT9M113_VAL_TERM}
};

static struct sensor_reg sensor_awb_incandescent_mt9m113[] = {
	{ 0x098C, 0xA355 },
	{ 0x0990, 0x0002 },
	{ 0x098C, 0xA369 },	// red
	{ 0x0990, 0x0037 },
	{ 0x098C, 0xA36B },	// blue
	{ 0x0990, 0x007B },
	{ 0x098C, 0xA36D },
	{ 0x0990, 0x0000 },
	{ 0x098C, 0xA36E },
	{ 0x0990, 0x0018 },
    {MT9M113_REG_TERM, MT9M113_VAL_TERM}
};

static struct sensor_reg sensor_awb_fluorescent_mt9m113[] = {
	{ 0x098C, 0xA355 },
	{ 0x0990, 0x0002 },
	{ 0x098C, 0xA369 },	// red
	{ 0x0990, 0x0037 },
	{ 0x098C, 0xA36B },	// blue
	{ 0x0990, 0x006B },
	{ 0x098C, 0xA36D },
	{ 0x0990, 0x0000 },
	{ 0x098C, 0xA36E },
	{ 0x0990, 0x0018 },
    {MT9M113_REG_TERM, MT9M113_VAL_TERM}
};

static struct sensor_reg sensor_awb_cloudy_mt9m113[] = {
	{ 0x098C, 0xA355 },
	{ 0x0990, 0x0002 },
	{ 0x098C, 0xA369 },	// red
	{ 0x0990, 0x005A },
	{ 0x098C, 0xA36B },	// blue
	{ 0x0990, 0x003D },
	{ 0x098C, 0xA36D },
	{ 0x0990, 0x0000 },
	{ 0x098C, 0xA36E },
	{ 0x0990, 0x0018 },
    {MT9M113_REG_TERM, MT9M113_VAL_TERM}
};

static struct sensor_reg sensor_awb_sunset_mt9m113[] = {
    {MT9M113_REG_TERM, MT9M113_VAL_TERM}
};

struct sensor_reg* sensor_reg_awb_mt9m113[6] =
{
	sensor_awb_auto_mt9m113,
	sensor_awb_daylight_mt9m113,
	sensor_awb_incandescent_mt9m113,
	sensor_awb_fluorescent_mt9m113,
	sensor_awb_cloudy_mt9m113,
	sensor_awb_sunset_mt9m113
	
};


static struct sensor_reg sensor_iso_auto_mt9m113[] = {
    {MT9M113_REG_TERM, MT9M113_VAL_TERM}
};

static struct sensor_reg sensor_iso_100_mt9m113[] = {
    {MT9M113_REG_TERM, MT9M113_VAL_TERM}
};

static struct sensor_reg sensor_iso_200_mt9m113[] = {
    {MT9M113_REG_TERM, MT9M113_VAL_TERM}
};

static struct sensor_reg sensor_iso_400_mt9m113[] = {
	{MT9M113_REG_TERM, MT9M113_VAL_TERM}
};

struct sensor_reg* sensor_reg_iso_mt9m113[4] =
{
	sensor_iso_auto_mt9m113,
	sensor_iso_100_mt9m113,
	sensor_iso_200_mt9m113,
	sensor_iso_400_mt9m113
};


static struct sensor_reg sensor_effect_normal_mt9m113[] = {
    {MT9M113_REG_TERM, MT9M113_VAL_TERM}
};

static struct sensor_reg sensor_effect_gray_mt9m113[] = {
    {MT9M113_REG_TERM, MT9M113_VAL_TERM}
};

static struct sensor_reg sensor_effect_negative_mt9m113[] = {
    {MT9M113_REG_TERM, MT9M113_VAL_TERM}
};

static struct sensor_reg sensor_effect_sepia_mt9m113[] = {
    {MT9M113_REG_TERM, MT9M113_VAL_TERM}
};

static struct sensor_reg sensor_effect_sharpness_mt9m113[] = {
    {MT9M113_REG_TERM, MT9M113_VAL_TERM}
};

static struct sensor_reg sensor_effect_sketch_mt9m113[] = {
    {MT9M113_REG_TERM, MT9M113_VAL_TERM}
};

struct sensor_reg* sensor_reg_effect_mt9m113[6] =
{
	sensor_effect_normal_mt9m113,
	sensor_effect_gray_mt9m113,
	sensor_effect_negative_mt9m113,
	sensor_effect_sepia_mt9m113,
	sensor_effect_sharpness_mt9m113,
	sensor_effect_sketch_mt9m113,
};


static struct sensor_reg sensor_reg_flipnone_mt9m113[] = {
    {MT9M113_REG_TERM, MT9M113_VAL_TERM}
};

static struct sensor_reg sensor_reg_hflip_mt9m113[] = {
    {MT9M113_REG_TERM, MT9M113_VAL_TERM}
};

static struct sensor_reg sensor_reg_vflip_mt9m113[] = {
    {MT9M113_REG_TERM, MT9M113_VAL_TERM}
};

static struct sensor_reg sensor_reg_hvflip_mt9m113[] = {
    {MT9M113_REG_TERM, MT9M113_VAL_TERM}
};

struct sensor_reg* sensor_reg_flip_mt9m113[4] =
{
	sensor_reg_flipnone_mt9m113,
	sensor_reg_hflip_mt9m113,
	sensor_reg_vflip_mt9m113,
	sensor_reg_hvflip_mt9m113,
};


static struct sensor_reg sensor_secne_auto_mt9m113[] = {
    {MT9M113_REG_TERM, MT9M113_VAL_TERM}
};

static struct sensor_reg sensor_secne_night_mt9m113[] = {
    {MT9M113_REG_TERM, MT9M113_VAL_TERM}
};

static struct sensor_reg sensor_secne_landscape_mt9m113[] = {
    {MT9M113_REG_TERM, MT9M113_VAL_TERM}
};

static struct sensor_reg sensor_secne_portrait_mt9m113[] = {
	{MT9M113_REG_TERM, MT9M113_VAL_TERM}
};

static struct sensor_reg sensor_secne_sport_mt9m113[] = {
	{MT9M113_REG_TERM, MT9M113_VAL_TERM}
};

struct sensor_reg* sensor_reg_scene_mt9m113[5] =
{
	sensor_secne_auto_mt9m113,
	sensor_secne_night_mt9m113,
	sensor_secne_landscape_mt9m113,
	sensor_secne_portrait_mt9m113,
	sensor_secne_sport_mt9m113
};

static struct sensor_reg sensor_me_mtrix_mt9m113[] = {
    {MT9M113_REG_TERM, MT9M113_VAL_TERM}
};

static struct sensor_reg sensor_me_center_weighted_mt9m113[] = {
    {MT9M113_REG_TERM, MT9M113_VAL_TERM}
};

static struct sensor_reg sensor_me_spot_mt9m113[] = {
    {MT9M113_REG_TERM, MT9M113_VAL_TERM}
};

struct sensor_reg* sensor_reg_metering_exposure_mt9m113[3] =
{
	sensor_me_mtrix_mt9m113,
	sensor_me_center_weighted_mt9m113,
	sensor_me_spot_mt9m113
};

static struct sensor_reg sensor_exposure_plus_2_mt9m113[] = {
    {0x098C, 0xA24F},
    {0x0990, 0x005F},
    {MT9M113_REG_TERM, MT9M113_VAL_TERM}
};

static struct sensor_reg sensor_exposure_plus_1_mt9m113[] = {
    {0x098C, 0xA24F},
    {0x0990, 0x0055},
    {MT9M113_REG_TERM, MT9M113_VAL_TERM}
};

static struct sensor_reg sensor_exposure_0_mt9m113[] = {
    {0x098C, 0xA24F},
    {0x0990, 0x004B},
    {MT9M113_REG_TERM, MT9M113_VAL_TERM}
};

static struct sensor_reg sensor_exposure_minus_1_mt9m113[] = {
    {0x098C, 0xA24F},
    {0x0990, 0x0041},
    {MT9M113_REG_TERM, MT9M113_VAL_TERM}
};

static struct sensor_reg sensor_exposure_minus_2_mt9m113[] = {
    {0x098C, 0xA24F},
    {0x0990, 0x0037},
    {MT9M113_REG_TERM, MT9M113_VAL_TERM}
};

struct sensor_reg* sensor_reg_exposure_mt9m113[5] =
{
	sensor_exposure_plus_2_mt9m113,
	sensor_exposure_plus_1_mt9m113,
	sensor_exposure_0_mt9m113,
	sensor_exposure_minus_1_mt9m113,
	sensor_exposure_minus_2_mt9m113
};

static struct sensor_reg sensor_af_single_mt9m113[] = {
    {MT9M113_REG_TERM, MT9M113_VAL_TERM}
};

static struct sensor_reg sensor_af_manual_mt9m113[] = {
    {MT9M113_REG_TERM, MT9M113_VAL_TERM}
};

struct sensor_reg* sensor_reg_af_mt9m113[2] =
{
	sensor_af_single_mt9m113,
	sensor_af_manual_mt9m113
};

static int write_regs_mt9m113(const struct sensor_reg reglist[], struct tcc_camera_device * vdev)
{
	int err;
	int err_cnt = 0;	
	int sleep_cnt = 100;
	unsigned char data[132];
	unsigned char bytes;
	const struct sensor_reg *next = reglist;
	
	while (!((next->reg == MT9M113_REG_TERM) && (next->val == MT9M113_VAL_TERM)))
	{
		
		if(next->reg == MT9M113_REG_TERM && next->val != MT9M113_VAL_TERM)
		{
			//mdelay(next->val);
			sensor_delay(next->val);
			sleep_cnt = 100;
			//printk("Sensor init Delay[%d]!!!! \n", next->val);
			next++;
		}
		else
		{
			sleep_cnt--;
			if(sleep_cnt == 0)
			{
				sensor_delay(10);
				sleep_cnt = 100;
			}
			
			bytes = 0;
			data[bytes]= next->reg>>8;		bytes++;		
			data[bytes]= (u8)next->reg&0xff; 	bytes++;

			data[bytes]= next->val>>8;		bytes++;		
			data[bytes]= (u8)next->val&0xff; 	bytes++;

			err = DDI_I2C_Write(data, 2, bytes-2, vdev);
			if (err)
			{
				err_cnt++;
				if(err_cnt >= 3)
				{
					printk("ERROR: Sensor I2C !!!! \n"); 
					return err;
				}
			}
			else
			{
				err_cnt = 0;
				next++;
			}
		}
	}

	return 0;
}

static int sensor_open_mt9m113(struct tcc_camera_device * vdev, bool bChangeCamera)
{
	if(bChangeCamera){
		sensor_cifmc_enable(vdev);
		sensor_delay(40);
		
		sensor_port_disable(vdev->gpio_data.rst_port);
		sensor_port_disable(vdev->gpio_data.pwd_port);
		sensor_delay(10);

		sensor_port_enable(vdev->gpio_data.rst_port);
		sensor_delay(10);
	}
	else{
		sensor_port_disable(vdev->gpio_data.pwr_port);
		sensor_delay(10);
		
		sensor_port_enable(vdev->gpio_data.pwr_port);
		sensor_delay(10);

		sensor_port_disable(vdev->gpio_data.pwd_port);
		sensor_delay(10);

		sensor_port_disable(vdev->gpio_data.rst_port);
		sensor_delay(10);

		sensor_cifmc_enable(vdev);
		sensor_delay(40);

		sensor_port_enable(vdev->gpio_data.rst_port);
		sensor_delay(15);
	}

	return write_regs_mt9m113(sensor_reg_common_mt9m113[0], vdev);
}

static int sensor_close_mt9m113(struct tcc_camera_device * vdev)
{
	sensor_port_disable(vdev->gpio_data.rst_port);
	sensor_port_disable(vdev->gpio_data.pwr_port);
	sensor_port_disable(vdev->gpio_data.pwd_port);

	sensor_cifmc_disable(vdev);
	sensor_delay(5);

	//printk("mt9m113 close!\n");

	return 0;
}

static int sensor_powerdown_mt9m113(struct tcc_camera_device * vdev, bool bChangeCamera)
{		

	if(bChangeCamera){
		sensor_cifmc_enable(vdev);
		sensor_delay(10);
		sensor_port_disable(vdev->gpio_data.rst_port);
		sensor_delay(10);
		sensor_port_enable(vdev->gpio_data.pwd_port);
		sensor_delay(10);

		//printk("mt9m113 change camera powerdown! \n");
	}
	else{
		sensor_port_disable(vdev->gpio_data.rst_port);
		sensor_port_disable(vdev->gpio_data.pwr_port);
		sensor_port_enable(vdev->gpio_data.pwd_port);

		//printk("mt9m113 first/last camera powerdown! \n");
	}

	return 0;
}

static int sensor_preview_mt9m113(struct tcc_camera_device * vdev)
{
	return write_regs_mt9m113(sensor_reg_common_mt9m113[1], vdev);
}

static int sensor_capture_mt9m113(struct tcc_camera_device * vdev)
{
	return write_regs_mt9m113(sensor_reg_common_mt9m113[2], vdev);
}

static int sensor_capturecfg_mt9m113(int width, int height, struct tcc_camera_device * vdev)
{
	return 0;
}

static int sensor_zoom_mt9m113(int val, struct tcc_camera_device * vdev)
{
	return 0;
}

static int sensor_autofocus_mt9m113(int val, struct tcc_camera_device * vdev)
{
	return write_regs_mt9m113(sensor_reg_af_mt9m113[val], vdev);
}

static int sensor_effect_mt9m113(int val, struct tcc_camera_device * vdev)
{
	return write_regs_mt9m113(sensor_reg_effect_mt9m113[val], vdev);
}

static int sensor_flip_mt9m113(int val, struct tcc_camera_device * vdev)
{
	return write_regs_mt9m113(sensor_reg_flip_mt9m113[val], vdev);
}

static int sensor_iso_mt9m113(int val, struct tcc_camera_device * vdev)
{
	return write_regs_mt9m113(sensor_reg_iso_mt9m113[val], vdev);
}

static int sensor_me_mt9m113(int val, struct tcc_camera_device * vdev)
{
	return write_regs_mt9m113(sensor_reg_metering_exposure_mt9m113[val], vdev);
}

static int sensor_wb_mt9m113(int val, struct tcc_camera_device * vdev)
{
	return write_regs_mt9m113(sensor_reg_awb_mt9m113[val], vdev);
}

static int sensor_bright_mt9m113(int val, struct tcc_camera_device * vdev)
{
	return write_regs_mt9m113(sensor_reg_brightness_mt9m113[val], vdev);
}

static int sensor_scene_mt9m113(int val, struct tcc_camera_device * vdev)
{
	return write_regs_mt9m113(sensor_reg_scene_mt9m113[val], vdev);
}

static int sensor_exposure_mt9m113(int val, struct tcc_camera_device * vdev)
{
	int val2;
	int val3;
	if((val-20)>= 0)
	{
		val3 = 10;
	}
	else
	{
		val3 = -10;
	}
	val2 = (val-20)/val3;
	return write_regs_mt9m113(sensor_reg_exposure_mt9m113[val2], vdev);
}

static int sensor_check_esd_mt9m113(int val, struct tcc_camera_device * vdev)
{
	return 0;
}

static int sensor_check_luma_mt9m113(int val, struct tcc_camera_device * vdev)
{
	return 0;
}

/**********************************************************
*    Function  
**********************************************************/

void sensor_info_init_mt9m113(TCC_SENSOR_INFO_TYPE *sensor_info)
{
	sensor_info->preview_w 				= 640;
	sensor_info->preview_h 				= 512;
	sensor_info->preview_zoom_offset_x 	= 10;
	sensor_info->preview_zoom_offset_y 	= 8;
	sensor_info->capture_w 				= 1280;
	sensor_info->capture_h 				= 1024;
	sensor_info->capture_zoom_offset_x 	= 20;
	sensor_info->capture_zoom_offset_y 	= 16;
	sensor_info->max_zoom_step 			= 15;
	sensor_info->cam_capchg_width 		= 640;
	sensor_info->framerate				= 15;
	sensor_info->p_clock_pol 				= NEGATIVE_EDGE;//POSITIVE_EDGE;//
	sensor_info->v_sync_pol 				= ACT_HIGH;
	sensor_info->h_sync_pol                 = ACT_HIGH;
	sensor_info->de_pol                 = ACT_LOW;
	sensor_info->field_bfield_low			= OFF;
	sensor_info->gen_field_en				= OFF;
	sensor_info->conv_en 					= OFF;
	sensor_info->hsde_connect_en 			= OFF;
	sensor_info->vs_mask 					= OFF;
	sensor_info->input_fmt 				= FMT_YUV422_8BIT;
	sensor_info->data_order 				= ORDER_RGB;
	sensor_info->intl_en					= OFF;
	sensor_info->intpl_en	 				= OFF;	
	sensor_info->format 					= M420_ZERO;
	sensor_info->capture_skip_frame 		= 1;
	sensor_info->sensor_sizes 			= sensor_sizes_mt9m113;
}

void sensor_init_fnc_mt9m113(SENSOR_FUNC_TYPE *sensor_func)
{
	sensor_func->Open 					= sensor_open_mt9m113;
	sensor_func->Close 					= sensor_close_mt9m113;
	sensor_func->PowerDown				= sensor_powerdown_mt9m113;

	sensor_func->Set_Preview 			= sensor_preview_mt9m113;
	sensor_func->Set_Capture 			= sensor_capture_mt9m113;
	sensor_func->Set_CaptureCfg 			= sensor_capturecfg_mt9m113;

	sensor_func->Set_Zoom 				= sensor_zoom_mt9m113;
	sensor_func->Set_AF 				= sensor_autofocus_mt9m113;
	sensor_func->Set_Effect 				= sensor_effect_mt9m113;
	sensor_func->Set_Flip 				= sensor_flip_mt9m113;
	sensor_func->Set_ISO 				= sensor_iso_mt9m113;
	sensor_func->Set_ME 				= sensor_me_mt9m113;
	sensor_func->Set_WB 				= sensor_wb_mt9m113;
	sensor_func->Set_Bright 				= sensor_bright_mt9m113;
	sensor_func->Set_Scene 				= sensor_scene_mt9m113;
	sensor_func->Set_Exposure			= sensor_exposure_mt9m113;

	sensor_func->Check_ESD 				= sensor_check_esd_mt9m113;
	sensor_func->Check_Luma 			= sensor_check_luma_mt9m113;
}

void sensor_info_init_front(TCC_SENSOR_INFO_TYPE *sensor_info)
{
	sensor_info_init_mt9m113(sensor_info);
}

void sensor_init_fnc_front(SENSOR_FUNC_TYPE *sensor_func)
{
	sensor_init_fnc_mt9m113(sensor_func);
}
#ifndef CONFIG_VIDEO_DUAL_CAMERA_SUPPORT
void sensor_info_init_back(TCC_SENSOR_INFO_TYPE *sensor_info)
{
	sensor_info_init_mt9m113(sensor_info);
}

void sensor_init_fnc_back(SENSOR_FUNC_TYPE *sensor_func)
{
	sensor_init_fnc_mt9m113(sensor_func);
}
#endif

#endif

