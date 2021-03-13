/****************************************************************************
 * FileName    : kernel/drivers/char/hdmi_v1_3/hdmi/hdmi.c
 * Description : hdmi driver
 *
 * Copyright (C) 2013 Telechips Inc.
 *
 * This program is free software; you can redistribute it and/or modify it under the terms
 * of the GNU General Public License as published by the Free Software Foundation;
 * either version 2 of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc., 59 Temple Place,
 * Suite 330, Boston, MA 02111-1307 USA
 * ****************************************************************************/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/poll.h>
#include <linux/wait.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/gpio.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/consumer.h>

#ifdef CONFIG_PM_RUNTIME
#include <linux/pm_runtime.h>
#endif

#ifdef CONFIG_PM
#include <linux/pm.h>
#endif

#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_device.h>
#include <linux/of_dma.h>

#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/mach-types.h>
#include "regs-hdmi.h"

#include <mach/hdmi_1_4_audio.h>
#include <mach/hdmi_1_4_hdmi.h>
#include <mach/tcc_board_hdmi.h>
#include <mach/vioc_outcfg.h>
#include <mach/vioc_disp.h>
#include <mach/tccfb.h>
#include <mach/gpio.h>
#include <mach/tca_display_config.h>

#ifdef HDMI_TX13_REV_05
#include "vwrapper.h"
#include "regs-vwrapper.h"
#endif

#define VERSION 		"1.2.1" /* Driver version number */
#define HDMI_MINOR 	240 /* Major 10, Minor 240, /dev/hdmi */


/**
 * If 'SIMPLAYHD' is 1, check Ri of 127th and 128th frame -@n
 * on 3rd authentication. And also check if Ri of 127th frame is -@n
 * different from that of 128th frame. if 'SIMPLAYHD' is 0, check only Ri -@n
 * of 128th frame.
 */
#define HDMI_DEBUG 	0
#define HDMI_DEBUG_TIME 0

#if HDMI_DEBUG
#define dprintk(args...)   { printk( "hdmi1.4: " args); }
#else
#define dprintk(args...)
#endif


#if HDMI_DEBUG_TIME
unsigned long jstart, jend;
unsigned long ji2cstart, ji2cend;
#endif

extern void tca_vioc_displayblock_powerOff(struct tcc_dp_device *pDisplayInfo);
extern void tca_vioc_displayblock_disable(struct tcc_dp_device *pDisplayInfo);

static inline u8 hdmi_readb (unsigned offset){
    return readb (IOMEM(offset));
}
static inline void hdmi_writeb(u8 b, unsigned offset){
    writeb(b, IOMEM(offset));
}
static inline u32 hdmi_readl(unsigned offset){
    return readl(IOMEM(offset));
}
static inline void hdmi_writel(u32 b, unsigned offset){
    writel(b, IOMEM(offset));
}


static struct clk *hdmi_pclk = NULL;
static struct clk *hdmi_hclk = NULL;
#if defined(CONFIG_ARCH_TCC896X) || defined(CONFIG_ARCH_TCC897X) || defined(CONFIG_ARCH_TCC570X)
static struct clk *hdmi_ipclk = NULL;
#endif

static int hdmi_lcdc_num = -1;
static int hdmi_gpio_manual_control = 0;
struct tcc_hdmi_platform_data *phdmi_data;

//! Structure for video timing parameters
static const struct device_video_params HDMIVideoParams[] =
{
	{ 800 , 160 , 525 , 45, 16  , 96 , 1, 10, 2, 1, 1 , 1 , 0, 0, },	// v640x480p_60Hz
	{ 858 , 138 , 525 , 45, 16  , 62 , 1, 9 , 6, 1, 2 , 3 , 0, 0, },	// v720x480p_60Hz
	//MVC_PROCESS
	{ 1650, 370 , 750 , 30, 110 , 40 , 0, 5 , 5, 0, 4 , 4 , 0, 0, },	// v1280x720p_60Hz_3D
	{ 1650, 370 , 750 , 30, 110 , 40 , 0, 5 , 5, 0, 4 , 4 , 0, 0, },	// v1280x720p_60Hz
	{ 2200, 280 , 1125, 22, 88  , 44 , 0, 2 , 5, 0, 5 , 5 , 1, 0, },	// v1920x1080i_60H
	{ 1716, 276 , 525 , 22, 38  , 124, 1, 4 , 3, 1, 6 , 7 , 1, 1, },	// v720x480i_60Hz
	{ 1716, 276 , 262 , 22, 38  , 124, 1, 4 , 3, 1, 8 , 9 , 0, 1, },	// v720x240p_60Hz
	//{ 1716, 276 , 263 , 23, 38  , 124, 1, 5 , 3, 1, 8 , 9 , 0, 1, },	// v720x240p_60Hz(mode 2)
	{ 3432, 552 , 525 , 22, 76  , 248, 1, 4 , 3, 1, 10, 11, 1, 1, },	// v2880x480i_60Hz
	{ 3432, 552 , 262 , 22, 76  , 248, 1, 4 , 3, 1, 12, 13, 0, 1, },	// v2880x240p_60Hz
	//{ 3432, 552 , 263 , 23, 76  , 248, 1, 5 , 3, 1, 12, 13, 0, 1, },	// v2880x240p_60Hz(mode 2)
	{ 1716, 276 , 525 , 45, 32  , 124, 1, 9 , 6, 1, 14, 15, 0, 1, },	// v1440x480p_60Hz
	{ 2200, 280 , 1125, 45, 88  , 44 , 0, 4 , 5, 0, 16, 16, 0, 0, },	// v1920x1080p_60H
	{ 864 , 144 , 625 , 49, 12  , 64 , 1, 5 , 5, 1, 17, 18, 0, 0, },	// v720x576p_50Hz
	{ 1980, 700 , 750 , 30, 440 , 40 , 0, 5 , 5, 0, 19, 19, 0, 0, },	// v1280x720p_50Hz
	{ 2640, 720 , 1125, 22, 528 , 44 , 0, 2 , 5, 0, 20, 20, 1, 0, },	// v1920x1080i_50H
	{ 1728, 288 , 625 , 24, 24  , 126, 1, 2 , 3, 1, 21, 22, 1, 1, },	// v720x576i_50Hz
	{ 1728, 288 , 312 , 24, 24  , 126, 1, 2 , 3, 1, 23, 24, 0, 1, },	// v720x288p_50Hz
	//{ 1728, 288 , 313 , 25, 24  , 126, 1, 3 , 3, 1, 23, 24, 0, 1, },	// v720x288p_50Hz(mode 2)
	//{ 1728, 288 , 314 , 26, 24  , 126, 1, 4 , 3, 1, 23, 24, 0, 1, },	// v720x288p_50Hz(mode 3)
	{ 3456, 576 , 625 , 24, 48  , 252, 1, 2 , 3, 1, 25, 26, 1, 1, },	// v2880x576i_50Hz
	{ 3456, 576 , 312 , 24, 48  , 252, 1, 2 , 3, 1, 27, 28, 0, 1, },	// v2880x288p_50Hz
	//{ 3456, 576 , 313 , 25, 48  , 252, 1, 3 , 3, 1, 27, 28, 0, 1, },	// v2880x288p_50Hz(mode 2)
	//{ 3456, 576 , 314 , 26, 48  , 252, 1, 4 , 3, 1, 27, 28, 0, 1, },	// v2880x288p_50Hz(mode 3)
	{ 1728, 288 , 625 , 49, 24  , 128, 1, 5 , 5, 1, 29, 30, 0, 1, },	// v1440x576p_50Hz
	{ 2640, 720 , 1125, 45, 528 , 44 , 0, 4 , 5, 0, 31, 31, 0, 0, },	// v1920x1080p_50Hz
	{ 2750, 830 , 1125, 45, 638 , 44 , 0, 4 , 5, 0, 32, 32, 0, 0, },	// v1920x1080p_23.976Hz
	//MVC_PROCESS
	{ 2750, 830 , 1125, 45, 638 , 44 , 0, 4 , 5, 0, 32, 32, 0, 0, },	// v1920x1080p_24Hz_3D
	{ 2750, 830 , 1125, 45, 638 , 44 , 0, 4 , 5, 0, 32, 32, 0, 0, },	// v1920x1080p_24Hz
	{ 2640, 720 , 1125, 45, 528 , 44 , 0, 4 , 5, 0, 33, 33, 0, 0, },	// v1920x1080p_25Hz
	{ 2200, 280 , 1125, 45, 88  , 44 , 0, 4 , 5, 0, 34, 34, 0, 0, },	// v1920x1080p_30Hz

	{ 3432, 552 , 525 , 45, 64  , 248, 1, 9 , 6, 1, 35, 36, 0, 1, },	// v2880x480p_60Hz
	{ 3456, 576 , 625 , 49, 48  , 256, 1, 5 , 5, 1, 37, 38, 0, 1, },	// v2880x576p_50Hz
	{ 2304, 384 , 1250, 85, 32  , 168, 0, 23, 5, 1, 39, 39, 1, 0, },	// v1920x1080i_50Hz(1250)
	{ 2640, 720 , 1125, 22, 528 , 44 , 0, 2 , 5, 0, 40, 40, 1, 0, },	// v1920x1080i_100Hz
	{ 1980, 700 , 750 , 30, 440 , 40 , 0, 5 , 5, 0, 41, 41, 0, 0, },	// v1280x720p_100Hz
	{ 864 , 144 , 625 , 49, 12  , 64 , 1, 5 , 5, 1, 42, 43, 0, 0, },	// v720x576p_100Hz
	{ 1728, 288 , 625 , 24, 24  , 126, 1, 2 , 3, 1, 44, 45, 1, 1, },	// v720x576i_100Hz
	{ 2200, 280 , 1125, 22, 88  , 44 , 0, 2 , 5, 0, 46, 46, 1, 0, },	// v1920x1080i_120Hz
	{ 1650, 370 , 750 , 30, 110 , 40 , 0, 5 , 5, 0, 47, 47, 0, 0, },	// v1280x720p_120Hz
	{ 858 , 138 , 525 , 54, 16  , 62 , 1, 9 , 6, 1, 48, 49, 0, 0, },	// v720x480p_120Hz
	{ 1716, 276 , 525 , 22, 38  , 124, 1, 4 , 3, 1, 50, 51, 1, 1, },	// v720x480i_120Hz
	{ 864 , 144 , 625 , 49, 12  , 64 , 1, 5 , 5, 1, 52, 53, 0, 0, },	// v720x576p_200Hz
	{ 1728, 288 , 625 , 24, 24  , 126, 1, 2 , 3, 1, 54, 55, 1, 1, },	// v720x576i_200Hz
	{ 858 , 138 , 525 , 45, 16  , 62 , 1, 9 , 6, 1, 56, 57, 0, 0, },	// v720x480p_240Hz
	{ 1716, 276 , 525 , 22, 38  , 124, 1, 4 , 3, 1, 58, 59, 1, 1, },	// v720x480i_240Hz
/*	
	{ 3300, 2020, 750 , 30, 1760, 40 , 0, 5 , 5, 0, 60, 60, 0, 0, },	// v1280x720p24Hz
	{ 3960, 2680, 750 , 30, 2420, 40 , 0, 5 , 5, 0, 61, 61, 0, 0, },        // v1280x720p25Hz
	{ 3300, 2020, 750 , 30, 1760, 40 , 0, 5 , 5, 0, 62, 62, 0, 0, },	// v1280x720p30Hz
	{ 2200, 280 , 1125, 45, 88  , 44 , 0, 4 , 5, 0, 63, 63, 0, 0, },	// v1920x1080p120Hz
	{ 2640, 720 , 1125, 45, 528 , 44 , 0, 4 , 5, 0, 64, 64, 0, 0, },	// v1920x1080p100Hz
	{ 4400, 560 , 2250, 90, 176 , 88 , 0, 8 , 10, 0, 1, 1, 0, 0, },	    	// v4Kx2K_30Hz
*/
	{ 2200, 280 , 1125, 45, 88  , 44 , 0, 4 , 5, 0, 34, 34, 0, 0, },	// v1920x1080p_29.976Hz	
       { 2080, 160 , 780,   60, 60  , 20 , 0, 2 , 1, 0,   0, 0,  0, 0, }        // v1920x720p_60H

};


/**
 * N value of ACR packet.@n
 * 4096  is the N value for 32 KHz sampling frequency @n
 * 6272  is the N value for 44.1 KHz sampling frequency @n
 * 12544 is the N value for 88.2 KHz sampling frequency @n
 * 25088 is the N value for 176.4 KHz sampling frequency @n
 * 6144  is the N value for 48 KHz sampling frequency @n
 * 12288 is the N value for 96 KHz sampling frequency @n
 * 24576 is the N value for 192 KHz sampling frequency @n
 */
static const unsigned int ACR_N_params[] =
{
    4096,
    6272,
    12544,
    25088,
    6144,
    12288,
    24576
};

#if defined(CONFIG_ARCH_TCC896X) || defined(CONFIG_ARCH_TCC897X) || defined(CONFIG_ARCH_TCC570X)
static unsigned int gHDMIFreqIndex = 0;

/**
 * PHY register setting values for each Pixel clock and Bit depth (8, 10, 12 bit).
 * @see  Setting values are came from LN28LPP_HDMI_v1p4_TX_PHY_DataSheet_REV1.2.pdf document.
 */

// TCC896X & TCC897x HDMI PHY Setting
static const unsigned char hdmi_phy_config[][3][32] =
{
	//25.200Mhz
	{
	   // TMDS Data Amplitude[4:0] : (Reg40 Bit<3:0>, REG3C Bit<7>), TMDS Clock Amplitude[3:0] : (Reg5C Bit<7:3>), TMDS Pre-emphasis Control : (Reg40 Bit<7:4>)
	   //0x04,0x08,0x0C,0x10,0x14.0x18,0x1C,0x20,0x24,0x28,0x2C,0x30,0x34,0x38,0x3C,0x40,0x44,0x48,0x4C,0x50,0x54,0x58,0x5C,0x60,0x64,0x68,0x6C,0x70,0x74,0x78,0x7C,0x80
		{0x52,0x3F,0x55,0x40,0x01,0x00,0xC8,0x82,0xC8,0xBD,0xD8,0x45,0xA0,0xAC,0x30,0x08,0x80,0x01,0x84,0x05,0x02,0x24,0x86,0x54,0xF4,0x24,0x00,0x00,0x00,0x01,0x80,0x90},
		{0x52,0x69,0x75,0x57,0x01,0x00,0xC8,0x82,0xC8,0x3B,0xD9,0x45,0xA0,0xAC,0x30,0x08,0x80,0x01,0x84,0x05,0x02,0x24,0x86,0x54,0xC3,0x24,0x00,0x00,0x00,0x01,0x80,0x90},
		{0x52,0x3F,0x35,0x63,0x01,0x00,0xC8,0x82,0xC8,0xBD,0xD8,0x45,0xA0,0xAC,0x30,0x08,0x80,0x09,0x84,0x05,0x02,0x24,0x86,0x54,0xA3,0x24,0x00,0x00,0x00,0x01,0x80,0x90},
	},
	
	//25.175Mhz
	{
	   // TMDS Data Amplitude[4:0] : (Reg40 Bit<3:0>, REG3C Bit<7>), TMDS Clock Amplitude[3:0] : (Reg5C Bit<7:3>), TMDS Pre-emphasis Control : (Reg40 Bit<7:4>)
	   //0x04,0x08,0x0C,0x10,0x14.0x18,0x1C,0x20,0x24,0x28,0x2C,0x30,0x34,0x38,0x3C,0x40,0x44,0x48,0x4C,0x50,0x54,0x58,0x5C,0x60,0x64,0x68,0x6C,0x70,0x74,0x78,0x7C,0x80
		{0xD1,0x1F,0x50,0x40,0x20,0x1E,0xC8,0x81,0xE8,0xBD,0xD8,0x45,0xA0,0xAC,0x30,0x08,0x80,0x09,0x84,0x05,0x02,0x24,0x86,0x54,0xF4,0x24,0x00,0x00,0x00,0x01,0x80,0x90},
		{0xD1,0x27,0x51,0x55,0x40,0x2B,0xC8,0x81,0xE8,0xEC,0xD8,0x45,0xA0,0xAC,0x30,0x08,0x80,0x09,0x84,0x05,0x02,0x24,0x86,0x54,0xC3,0x24,0x00,0x00,0x00,0x01,0x80,0x90},
		{0xD1,0x1F,0x30,0x63,0x20,0x1E,0xC8,0x81,0xE8,0xBD,0xD8,0x45,0xA0,0xAC,0x30,0x08,0x80,0x09,0x84,0x05,0x02,0x24,0x86,0x54,0xA3,0x24,0x00,0x00,0x00,0x01,0x80,0x90},
	},
	
	//27.000Mhz
	{
	   // TMDS Data Amplitude[4:0] : (Reg40 Bit<3:0>, REG3C Bit<7>), TMDS Clock Amplitude[3:0] : (Reg5C Bit<7:3>), TMDS Pre-emphasis Control : (Reg40 Bit<7:4>)
	   //0x04,0x08,0x0C,0x10,0x14.0x18,0x1C,0x20,0x24,0x28,0x2C,0x30,0x34,0x38,0x3C,0x40,0x44,0x48,0x4C,0x50,0x54,0x58,0x5C,0x60,0x64,0x68,0x6C,0x70,0x74,0x78,0x7C,0x80
		{0xD1,0x22,0x51,0x40,0x08,0xFC,0xE0,0x98,0xE8,0xCB,0xD8,0x45,0xA0,0xAC,0x30,0x08,0x80,0x09,0x84,0x05,0x02,0x24,0x86,0x54,0xE4,0x24,0x00,0x00,0x00,0x01,0x80,0x90},
		{0xD1,0x2A,0x52,0x55,0x08,0x03,0xC8,0x86,0xE8,0xFD,0xD8,0x45,0xA0,0xAC,0x30,0x08,0x80,0x09,0x84,0x05,0x02,0x24,0x86,0x54,0xB6,0x24,0x00,0x00,0x00,0x01,0x80,0x90},
		{0xD1,0x33,0x54,0x65,0x08,0xFA,0xC8,0x85,0xE8,0x30,0xD9,0x45,0xA0,0xAC,0x30,0x08,0x80,0x09,0x84,0x05,0x02,0x24,0x86,0x54,0x98,0x24,0x00,0x00,0x00,0x01,0x80,0x90},
	},
	
	//27.027Mhz
	{
	   // TMDS Data Amplitude[4:0] : (Reg40 Bit<3:0>, REG3C Bit<7>), TMDS Clock Amplitude[3:0] : (Reg5C Bit<7:3>), TMDS Pre-emphasis Control : (Reg40 Bit<7:4>)
	   //0x04,0x08,0x0C,0x10,0x14.0x18,0x1C,0x20,0x24,0x28,0x2C,0x30,0x34,0x38,0x3C,0x40,0x44,0x48,0x4C,0x50,0x54,0x58,0x5C,0x60,0x64,0x68,0x6C,0x70,0x74,0x78,0x7C,0x80
		{0xD1,0x2D,0x72,0x40,0x64,0x12,0xC8,0x43,0xE8,0x0E,0xD9,0x45,0xA0,0xAC,0x30,0x08,0x80,0x09,0x84,0x05,0x02,0x24,0x86,0x54,0xE3,0x24,0x00,0x00,0x00,0x01,0x80,0x90},
		{0xD1,0x38,0x74,0x57,0x50,0x31,0xC1,0x80,0xC8,0x52,0xD9,0x45,0xA0,0xAC,0x30,0x08,0x80,0x09,0x84,0x05,0x02,0x24,0x86,0x54,0xB6,0x24,0x00,0x00,0x00,0x01,0x80,0x90},
		{0xD4,0x87,0x31,0x63,0x64,0x1B,0xE0,0x19,0xE8,0xCB,0xD8,0x45,0xA0,0xAC,0x30,0x08,0x80,0x09,0x84,0x05,0x02,0x24,0x86,0x54,0x98,0x24,0x00,0x00,0x00,0x01,0x80,0x90},
	},
	
	//54.000Mhz
	{
	   // TMDS Data Amplitude[4:0] : (Reg40 Bit<3:0>, REG3C Bit<7>), TMDS Clock Amplitude[3:0] : (Reg5C Bit<7:3>), TMDS Pre-emphasis Control : (Reg40 Bit<7:4>)
	   //0x04,0x08,0x0C,0x10,0x14.0x18,0x1C,0x20,0x24,0x28,0x2C,0x30,0x34,0x38,0x3C,0x40,0x44,0x48,0x4C,0x50,0x54,0x58,0x5C,0x60,0x64,0x68,0x6C,0x70,0x74,0x78,0x7C,0x80
		{0x51,0x2D,0x35,0x40,0x01,0x00,0xC8,0x82,0xC8,0x0E,0xD9,0x45,0xA0,0xAC,0x30,0x08,0x80,0x09,0x84,0x05,0x02,0x24,0x86,0x54,0xE4,0x24,0x01,0x00,0x00,0x01,0x80,0x90},
		{0xD1,0x38,0x35,0x53,0x08,0x04,0xC8,0x88,0xE8,0x52,0xD9,0x45,0xA0,0xAC,0x30,0x08,0x80,0x09,0x84,0x05,0x02,0x24,0x86,0x54,0xB6,0x24,0x01,0x00,0x00,0x01,0x80,0x90},
		{0xD1,0x22,0x15,0x61,0x08,0xFC,0xC8,0x82,0xC8,0xCB,0xD8,0x45,0xA0,0xAC,0x30,0x08,0x80,0x09,0x84,0x05,0x02,0x24,0x86,0x54,0x98,0x24,0x01,0x00,0x00,0x01,0x80,0x90},
	},
	
	//54.054Mhz
	{
	   // TMDS Data Amplitude[4:0] : (Reg40 Bit<3:0>, REG3C Bit<7>), TMDS Clock Amplitude[3:0] : (Reg5C Bit<7:3>), TMDS Pre-emphasis Control : (Reg40 Bit<7:4>)
	   //0x04,0x08,0x0C,0x10,0x14.0x18,0x1C,0x20,0x24,0x28,0x2C,0x30,0x34,0x38,0x3C,0x40,0x44,0x48,0x4C,0x50,0x54,0x58,0x5C,0x60,0x64,0x68,0x6C,0x70,0x74,0x78,0x7C,0x80
		{0xD1,0x2D,0x32,0x40,0x64,0x12,0xC8,0x43,0xE8,0x0E,0xD9,0x45,0xA0,0xAC,0x30,0x08,0x80,0x09,0x84,0x05,0x02,0x24,0x86,0x54,0xE3,0x24,0x01,0x00,0x00,0x01,0x80,0x90},
		{0xD2,0x70,0x34,0x53,0x50,0x31,0xC8,0x80,0xC8,0x52,0xD9,0x45,0xA0,0xAC,0x30,0x08,0x80,0x09,0x84,0x05,0x02,0x24,0x86,0x54,0xB6,0x24,0x01,0x00,0x00,0x01,0x80,0x90},
		{0xD4,0x87,0x11,0x61,0x64,0x1B,0xE0,0x19,0xE8,0xCB,0xD8,0x45,0xA0,0xAC,0x30,0x08,0x80,0x09,0x84,0x05,0x02,0x24,0x86,0x54,0x98,0x24,0x01,0x00,0x00,0x01,0x80,0x90},
	},
	
	//74.250Mhz
	{
	   // TMDS Data Amplitude[4:0] : (Reg40 Bit<3:0>, REG3C Bit<7>), TMDS Clock Amplitude[3:0] : (Reg5C Bit<7:3>), TMDS Pre-emphasis Control : (Reg40 Bit<7:4>)
	   //0x04,0x08,0x0C,0x10,0x14.0x18,0x1C,0x20,0x24,0x28,0x2C,0x30,0x34,0x38,0x3C,0x40,0x44,0x48,0x4C,0x50,0x54,0x58,0x5C,0x60,0x64,0x68,0x6C,0x70,0x74,0x78,0x7C,0x80
		{0xD1,0x1F,0x10,0x40,0x40,0xF8,0xC8,0x81,0xE8,0xBA,0xD8,0x45,0xA0,0xAC,0x30,0x08,0x80,0x09,0x84,0x05,0x02,0x24,0x86,0x54,0xA5,0x24,0x01,0x00,0x00,0x01,0x80,0x90},
		{0xD1,0x27,0x11,0x51,0x40,0xD6,0xC8,0x81,0xE8,0xE8,0xD8,0x45,0xA0,0xAC,0x30,0x08,0x80,0x09,0x84,0x05,0x02,0x24,0x86,0x54,0x84,0x24,0x01,0x00,0x00,0x01,0x80,0x90},
		{0xD1,0x2E,0x12,0x61,0x40,0x34,0xC8,0x82,0xE8,0x16,0xD9,0x45,0xA0,0xAC,0x30,0x08,0x80,0x09,0x84,0x05,0x02,0x24,0x86,0x54,0xB9,0x25,0x03,0x00,0x00,0x01,0x80,0x90},
	},
	
	//74.176Mhz
	{
	   // TMDS Data Amplitude[4:0] : (Reg40 Bit<3:0>, REG3C Bit<7>), TMDS Clock Amplitude[3:0] : (Reg5C Bit<7:3>), TMDS Pre-emphasis Control : (Reg40 Bit<7:4>)
	   //0x04,0x08,0x0C,0x10,0x14.0x18,0x1C,0x20,0x24,0x28,0x2C,0x30,0x34,0x38,0x3C,0x40,0x44,0x48,0x4C,0x50,0x54,0x58,0x5C,0x60,0x64,0x68,0x6C,0x70,0x74,0x78,0x7C,0x80
		{0xD1,0x1F,0x10,0x40,0x5B,0xEF,0xC8,0x81,0xE8,0xB9,0xD8,0x45,0xA0,0xAC,0x30,0x08,0x80,0x09,0x84,0x05,0x02,0x24,0x86,0x54,0xA6,0x24,0x01,0x00,0x00,0x01,0x80,0x90},
		{0xD1,0x27,0x14,0x51,0x5B,0xA7,0xC8,0x84,0xE8,0xE8,0xD8,0x45,0xA0,0xAC,0x30,0x08,0x80,0x09,0x84,0x05,0x02,0x24,0x86,0x54,0x85,0x24,0x01,0x00,0x00,0x01,0x80,0x90},
		{0xD2,0x5D,0x12,0x61,0x5B,0xCD,0xD0,0x43,0xE8,0x16,0xD9,0x45,0xA0,0xAC,0x30,0x08,0x80,0x09,0x84,0x05,0x02,0x24,0x86,0x54,0xBA,0x25,0x03,0x00,0x00,0x01,0x80,0x90},
	},
	
	//148.500Mhz
	{
	   // TMDS Data Amplitude[4:0] : (Reg40 Bit<3:0>, REG3C Bit<7>), TMDS Clock Amplitude[3:0] : (Reg5C Bit<7:3>), TMDS Pre-emphasis Control : (Reg40 Bit<7:4>)
	   //0x04,0x08,0x0C,0x10,0x14.0x18,0x1C,0x20,0x24,0x28,0x2C,0x30,0x34,0x38,0x3C,0x40,0x44,0x48,0x4C,0x50,0x54,0x58,0x5C,0x60,0x64,0x68,0x6C,0x70,0x74,0x78,0x7C,0x80
		{0xD1,0x1F,0x00,0x40,0x40,0xF8,0xC8,0x81,0xE8,0xBA,0xD8,0x45,0xA0,0xAC,0x30,0x0e,0x80,0x09,0x84,0x05,0x02,0x24,0xe4,0x54,0x4B,0x25,0x03,0x00,0x00,0x01,0x80,0x90},
		{0xD1,0x27,0x01,0x50,0x40,0xD6,0xC8,0x81,0xE8,0xE8,0xD8,0x45,0xA0,0xAC,0x30,0x0e,0x80,0x09,0x84,0x05,0x02,0x24,0xe4,0x54,0x09,0x25,0x03,0x00,0x00,0x01,0x80,0x90},
		{0xD1,0x2E,0x02,0x60,0x40,0x34,0xC8,0x82,0xE8,0x16,0xD9,0x45,0xA0,0xAC,0x30,0x0e,0x80,0x09,0x84,0x05,0x02,0x24,0xe4,0x54,0xDD,0x24,0x03,0x00,0x00,0x01,0x80,0x90},
	},
	
	//148.352Mhz
	{
	   // TMDS Data Amplitude[4:0] : (Reg40 Bit<3:0>, REG3C Bit<7>), TMDS Clock Amplitude[3:0] : (Reg5C Bit<7:3>), TMDS Pre-emphasis Control : (Reg40 Bit<7:4>)
	   //0x04,0x08,0x0C,0x10,0x14.0x18,0x1C,0x20,0x24,0x28,0x2C,0x30,0x34,0x38,0x3C,0x40,0x44,0x48,0x4C,0x50,0x54,0x58,0x5C,0x60,0x64,0x68,0x6C,0x70,0x74,0x78,0x7C,0x80
		{0xD1,0x1F,0x00,0x40,0x5B,0xEF,0xC8,0x81,0xE8,0xB9,0xD8,0x45,0xA0,0xAC,0x30,0x0e,0x80,0x09,0x84,0x05,0x02,0x24,0xe4,0x54,0x4B,0x25,0x03,0x00,0x00,0x01,0x80,0x90},
		{0xD1,0x27,0x04,0x50,0x5B,0xA7,0xC8,0x84,0xE8,0xE8,0xD8,0x45,0xA0,0xAC,0x30,0x0e,0x80,0x09,0x84,0x05,0x02,0x24,0xe4,0x54,0x09,0x25,0x03,0x00,0x00,0x01,0x80,0x90},
		{0xD2,0x5D,0x02,0x60,0x5B,0xCD,0xD0,0x43,0xE8,0x16,0xD9,0x45,0xA0,0xAC,0x30,0x0e,0x80,0x09,0x84,0x05,0x02,0x24,0xe4,0x54,0xDD,0x24,0x03,0x00,0x00,0x01,0x80,0x90},
	},
	
	//108.108Mhz
	{
	   // TMDS Data Amplitude[4:0] : (Reg40 Bit<3:0>, REG3C Bit<7>), TMDS Clock Amplitude[3:0] : (Reg5C Bit<7:3>), TMDS Pre-emphasis Control : (Reg40 Bit<7:4>)
	   //0x04,0x08,0x0C,0x10,0x14.0x18,0x1C,0x20,0x24,0x28,0x2C,0x30,0x34,0x38,0x3C,0x40,0x44,0x48,0x4C,0x50,0x54,0x58,0x5C,0x60,0x64,0x68,0x6C,0x70,0x74,0x78,0x7C,0x80
		{0xD1,0x2D,0x12,0x40,0x64,0x12,0xC8,0x43,0xE8,0x0E,0xD9,0x45,0xA0,0xAC,0x30,0x08,0x80,0x09,0x84,0x05,0x02,0x24,0x86,0x54,0xC7,0x25,0x03,0x00,0x00,0x01,0x80,0x90},
		{0xD2,0x70,0x14,0x51,0x50,0x31,0xC8,0x80,0xC8,0x52,0xD9,0x45,0xA0,0xAC,0x30,0x08,0x80,0x09,0x84,0x05,0x02,0x24,0x86,0x54,0x6C,0x25,0x03,0x00,0x00,0x01,0x80,0x90},
		{0xD4,0x87,0x01,0x60,0x64,0x1B,0xE0,0x19,0xE8,0xCB,0xD8,0x45,0xA0,0xAC,0x30,0x08,0x80,0x09,0x84,0x05,0x02,0x24,0x86,0x54,0x2F,0x25,0x03,0x00,0x00,0x01,0x80,0x90},
	},
	
	//72.000Mhz
	{
	   // TMDS Data Amplitude[4:0] : (Reg40 Bit<3:0>, REG3C Bit<7>), TMDS Clock Amplitude[3:0] : (Reg5C Bit<7:3>), TMDS Pre-emphasis Control : (Reg40 Bit<7:4>)
	   //0x04,0x08,0x0C,0x10,0x14.0x18,0x1C,0x20,0x24,0x28,0x2C,0x30,0x34,0x38,0x3C,0x40,0x44,0x48,0x4C,0x50,0x54,0x58,0x5C,0x60,0x64,0x68,0x6C,0x70,0x74,0x78,0x7C,0x80
		{0x51,0x1E,0x15,0x40,0x01,0x00,0xC8,0x82,0xC8,0xB4,0xD8,0x45,0xA0,0xAC,0x30,0x08,0x80,0x09,0x84,0x05,0x02,0x24,0x86,0x54,0xAB,0x24,0x01,0x00,0x00,0x01,0x80,0x90},
		{0x52,0x4B,0x15,0x51,0x01,0x00,0xC8,0x82,0xC8,0xE1,0xD8,0x45,0xA0,0xAC,0x30,0x08,0x80,0x09,0x84,0x05,0x02,0x24,0x86,0x54,0x89,0x24,0x01,0x00,0x00,0x01,0x80,0x90},
		{0x51,0x2D,0x15,0x61,0x01,0x00,0xC8,0x82,0xC8,0x0E,0xD9,0x45,0xA0,0xAC,0x30,0x08,0x80,0x09,0x84,0x05,0x02,0x24,0x86,0x54,0xC7,0x25,0x03,0x00,0x00,0x01,0x80,0x90},
	},
	
	//25.000Mhz
	{
	   // TMDS Data Amplitude[4:0] : (Reg40 Bit<3:0>, REG3C Bit<7>), TMDS Clock Amplitude[3:0] : (Reg5C Bit<7:3>), TMDS Pre-emphasis Control : (Reg40 Bit<7:4>)
	   //0x04,0x08,0x0C,0x10,0x14.0x18,0x1C,0x20,0x24,0x28,0x2C,0x30,0x34,0x38,0x3C,0x40,0x44,0x48,0x4C,0x50,0x54,0x58,0x5C,0x60,0x64,0x68,0x6C,0x70,0x74,0x78,0x7C,0x80
		{0xD1,0x2A,0x72,0x40,0x3C,0xD8,0xC8,0x86,0xE8,0xFA,0xD8,0x45,0xA0,0xAC,0x30,0x08,0x80,0x09,0x84,0x05,0x02,0x24,0x86,0x54,0xF6,0x24,0x00,0x00,0x00,0x01,0x80,0x90},
		{0xD1,0x27,0x51,0x55,0x40,0x08,0xC8,0x81,0xE8,0xEA,0xD8,0x45,0xA0,0xAC,0x30,0x08,0x80,0x09,0x84,0x05,0x02,0x24,0x86,0x54,0xC5,0x24,0x00,0x00,0x00,0x01,0x80,0x90},
		{0xD1,0x1F,0x30,0x63,0x40,0x20,0xC8,0x81,0xE8,0xBC,0xD8,0x45,0xA0,0xAC,0x30,0x08,0x80,0x09,0x84,0x05,0x02,0x24,0x86,0x54,0xA4,0x24,0x00,0x00,0x00,0x01,0x80,0x90},
	},
	
	//65.000Mhz
	{
	   // TMDS Data Amplitude[4:0] : (Reg40 Bit<3:0>, REG3C Bit<7>), TMDS Clock Amplitude[3:0] : (Reg5C Bit<7:3>), TMDS Pre-emphasis Control : (Reg40 Bit<7:4>)
	   //0x04,0x08,0x0C,0x10,0x14.0x18,0x1C,0x20,0x24,0x28,0x2C,0x30,0x34,0x38,0x3C,0x40,0x44,0x48,0x4C,0x50,0x54,0x58,0x5C,0x60,0x64,0x68,0x6C,0x70,0x74,0x78,0x7C,0x80
		{0xD1,0x36,0x34,0x40,0x0C,0x04,0xC8,0x82,0xE8,0x45,0xD9,0x45,0xA0,0xAC,0x30,0x08,0x80,0x09,0x84,0x05,0x02,0x24,0x86,0x54,0xBD,0x24,0x01,0x00,0x00,0x01,0x80,0x90},
		{0xD1,0x22,0x11,0x51,0x30,0xF2,0xC8,0x86,0xE8,0xCB,0xD8,0x45,0xA0,0xAC,0x30,0x08,0x80,0x09,0x84,0x05,0x02,0x24,0x86,0x54,0x97,0x24,0x01,0x00,0x00,0x01,0x80,0x90},
		{0xD1,0x29,0x12,0x61,0x40,0xD0,0xC8,0x87,0xE8,0xF4,0xD8,0x45,0xA0,0xAC,0x30,0x08,0x80,0x09,0x84,0x05,0x02,0x24,0x86,0x54,0x7E,0x24,0x01,0x00,0x00,0x01,0x80,0x90},
	},
	
	//108.000Mhz
	{
	   // TMDS Data Amplitude[4:0] : (Reg40 Bit<3:0>, REG3C Bit<7>), TMDS Clock Amplitude[3:0] : (Reg5C Bit<7:3>), TMDS Pre-emphasis Control : (Reg40 Bit<7:4>)
	   //0x04,0x08,0x0C,0x10,0x14.0x18,0x1C,0x20,0x24,0x28,0x2C,0x30,0x34,0x38,0x3C,0x40,0x44,0x48,0x4C,0x50,0x54,0x58,0x5C,0x60,0x64,0x68,0x6C,0x70,0x74,0x78,0x7C,0x80
		{0x51,0x2D,0x15,0x40,0x01,0x00,0xC8,0x82,0xC8,0x0E,0xD9,0x45,0xA0,0xAC,0x30,0x08,0x80,0x09,0x84,0x05,0x02,0x24,0x86,0x54,0xC7,0x25,0x03,0x00,0x00,0x01,0x80,0x90},
		{0xD1,0x38,0x14,0x51,0x08,0x04,0xC8,0x80,0xC8,0x52,0xD9,0x45,0xA0,0xAC,0x30,0x08,0x80,0x09,0x84,0x05,0x02,0x24,0x86,0x54,0x6C,0x25,0x03,0x00,0x00,0x01,0x80,0x90},
		{0xD1,0x22,0x01,0x60,0x08,0xFC,0xC8,0x86,0xE8,0xCB,0xD8,0x45,0xA0,0xAC,0x30,0x08,0x80,0x09,0x84,0x05,0x02,0x24,0x86,0x54,0x2F,0x25,0x03,0x00,0x00,0x01,0x80,0x90},
	},
	
	//162.000Mhz
	{
	   // TMDS Data Amplitude[4:0] : (Reg40 Bit<3:0>, REG3C Bit<7>), TMDS Clock Amplitude[3:0] : (Reg5C Bit<7:3>), TMDS Pre-emphasis Control : (Reg40 Bit<7:4>)
	   //0x04,0x08,0x0C,0x10,0x14.0x18,0x1C,0x20,0x24,0x28,0x2C,0x30,0x34,0x38,0x3C,0x40,0x44,0x48,0x4C,0x50,0x54,0x58,0x5C,0x60,0x64,0x68,0x6C,0x70,0x74,0x78,0x7C,0x80
		{0x54,0x87,0x05,0x40,0x01,0x00,0xC8,0x82,0xC8,0xCB,0xD8,0x45,0xA0,0xAC,0x30,0x0e,0x80,0x09,0x84,0x05,0x02,0x24,0xe4,0x54,0x2F,0x25,0x03,0x00,0x00,0x01,0x80,0x90},
		{0xD1,0x2A,0x02,0x50,0x40,0x18,0xC8,0x86,0xE8,0xFD,0xD8,0x45,0xA0,0xAC,0x30,0x0e,0x80,0x09,0x84,0x05,0x02,0x24,0xe4,0x54,0xF3,0x24,0x03,0x00,0x00,0x01,0x80,0x90},
		{0xD1,0x33,0x04,0x60,0x40,0xD0,0xC8,0x85,0xE8,0x30,0xD9,0x45,0xA0,0xAC,0x30,0x0e,0x80,0x09,0x84,0x05,0x02,0x24,0xe4,0x54,0xCA,0x24,0x03,0x00,0x00,0x01,0x80,0x90},
	},
	
	//45.000Mhz
	{
	   // TMDS Data Amplitude[4:0] : (Reg40 Bit<3:0>, REG3C Bit<7>), TMDS Clock Amplitude[3:0] : (Reg5C Bit<7:3>), TMDS Pre-emphasis Control : (Reg40 Bit<7:4>)
	   //0x04,0x08,0x0C,0x10,0x14.0x18,0x1C,0x20,0x24,0x28,0x2C,0x30,0x34,0x38,0x3C,0x40,0x44,0x48,0x4C,0x50,0x54,0x58,0x5C,0x60,0x64,0x68,0x6C,0x70,0x74,0x78,0x7C,0x80
		{0x52,0x4B,0x35,0x40,0x01,0x00,0xC8,0x82,0xC8,0xE1,0xD8,0x45,0xA0,0xAC,0x30,0x08,0x80,0x09,0x84,0x05,0x02,0x24,0x86,0x54,0x11,0x25,0x01,0x00,0x00,0x01,0x80,0x90},
		{0xD1,0x2F,0x32,0x53,0x40,0xF0,0xC8,0x81,0xE8,0x19,0xD9,0x45,0xA0,0xAC,0x30,0x08,0x80,0x09,0x84,0x05,0x02,0x24,0x86,0x54,0xDA,0x24,0x01,0x00,0x00,0x01,0x80,0x90},
		{0xD1,0x38,0x34,0x63,0x40,0x20,0xC8,0x80,0xC8,0x52,0xD9,0x45,0xA0,0xAC,0x30,0x08,0x80,0x09,0x84,0x05,0x02,0x24,0x86,0x54,0xB6,0x24,0x01,0x00,0x00,0x01,0x80,0x90},
	},
	
	//44.955Mhz
	{
	   // TMDS Data Amplitude[4:0] : (Reg40 Bit<3:0>, REG3C Bit<7>), TMDS Clock Amplitude[3:0] : (Reg5C Bit<7:3>), TMDS Pre-emphasis Control : (Reg40 Bit<7:4>)
	   //0x04,0x08,0x0C,0x10,0x14.0x18,0x1C,0x20,0x24,0x28,0x2C,0x30,0x34,0x38,0x3C,0x40,0x44,0x48,0x4C,0x50,0x54,0x58,0x5C,0x60,0x64,0x68,0x6C,0x70,0x74,0x78,0x7C,0x80
		{0xD1,0x25,0x31,0x40,0x5B,0x54,0xC8,0x83,0xE8,0xE1,0xD8,0x45,0xA0,0xAC,0x30,0x08,0x80,0x09,0x84,0x05,0x02,0x24,0x86,0x54,0x11,0x25,0x01,0x00,0x00,0x01,0x80,0x90},
		{0xD2,0x5D,0x32,0x53,0x5B,0x3B,0xD0,0x83,0xE8,0x19,0xD9,0x45,0xA0,0xAC,0x30,0x08,0x80,0x09,0x84,0x05,0x02,0x24,0x86,0x54,0xDB,0x24,0x01,0x00,0x00,0x01,0x80,0x90},
		{0xD2,0x70,0x35,0x63,0x5B,0x23,0xC8,0x88,0xE8,0x51,0xD9,0x45,0xA0,0xAC,0x30,0x08,0x80,0x09,0x84,0x05,0x02,0x24,0x86,0x54,0xB6,0x24,0x01,0x00,0x00,0x01,0x80,0x90},
	},
	
	//297.000Mhz
	//We only support 8bit mode, 10bit and 12bit is same setting with 8bit. If you set 10bit and 12bit, there is no meaning.
	{
	   // TMDS Data Amplitude[4:0] : (Reg40 Bit<3:0>, REG3C Bit<7>), TMDS Clock Amplitude[3:0] : (Reg5C Bit<7:3>), TMDS Pre-emphasis Control : (Reg40 Bit<7:4>)
	   //0x04,0x08,0x0C,0x10,0x14.0x18,0x1C,0x20,0x24,0x28,0x2C,0x30,0x34,0x38,0x3C,0x40,0x44,0x48,0x4C,0x50,0x54,0x58,0x5C,0x60,0x64,0x68,0x6C,0x70,0x74,0x78,0x7C,0x80
		{0xD1,0x3E,0x05,0x40,0x40,0xE0,0xC8,0x42,0xE8,0x73,0xD9,0x45,0xA0,0xAC,0x30,0x0e,0x80,0x09,0x84,0x05,0x02,0x24,0xe4,0x54,0xA5,0x24,0x03,0x00,0x00,0x01,0x80,0x90},
		{0xD1,0x3E,0x05,0x40,0x40,0xE0,0xC8,0x42,0xE8,0x73,0xD9,0x45,0xA0,0xAC,0x30,0x0e,0x80,0x09,0x84,0x05,0x02,0x24,0xe4,0x54,0xA5,0x24,0x03,0x00,0x00,0x01,0x80,0x90},
		{0xD1,0x3E,0x05,0x40,0x40,0xE0,0xC8,0x42,0xE8,0x73,0xD9,0x45,0xA0,0xAC,0x30,0x0e,0x80,0x09,0x84,0x05,0x02,0x24,0xe4,0x54,0xA5,0x24,0x03,0x00,0x00,0x01,0x80,0x90},
	},
	
	//296.703Mhz
	//We only support 8bit mode, 10bit and 12bit is same setting with 8bit. If you set 10bit and 12bit, there is no meaning.
	{
	   // TMDS Data Amplitude[4:0] : (Reg40 Bit<3:0>, REG3C Bit<7>), TMDS Clock Amplitude[3:0] : (Reg5C Bit<7:3>), TMDS Pre-emphasis Control : (Reg40 Bit<7:4>)
	   //0x04,0x08,0x0C,0x10,0x14.0x18,0x1C,0x20,0x24,0x28,0x2C,0x30,0x34,0x38,0x3C,0x40,0x44,0x48,0x4C,0x50,0x54,0x58,0x5C,0x60,0x64,0x68,0x6C,0x70,0x74,0x78,0x7C,0x80
		{0xD1,0x3E,0x05,0x40,0x5B,0xDE,0xC8,0x82,0xE8,0x73,0xD9,0x45,0xA0,0xAC,0x30,0x0e,0x80,0x09,0x84,0x05,0x02,0x24,0xe4,0x54,0xA6,0x24,0x03,0x00,0x00,0x01,0x80,0x90},
		{0xD1,0x3E,0x05,0x40,0x5B,0xDE,0xC8,0x82,0xE8,0x73,0xD9,0x45,0xA0,0xAC,0x30,0x0e,0x80,0x09,0x84,0x05,0x02,0x24,0xe4,0x54,0xA6,0x24,0x03,0x00,0x00,0x01,0x80,0x90},
		{0xD1,0x3E,0x05,0x40,0x5B,0xDE,0xC8,0x82,0xE8,0x73,0xD9,0x45,0xA0,0xAC,0x30,0x0e,0x80,0x09,0x84,0x05,0x02,0x24,0xe4,0x54,0xA6,0x24,0x03,0x00,0x00,0x01,0x80,0x90},
	},
	
	//59.400Mhz
	{
	   // TMDS Data Amplitude[4:0] : (Reg40 Bit<3:0>, REG3C Bit<7>), TMDS Clock Amplitude[3:0] : (Reg5C Bit<7:3>), TMDS Pre-emphasis Control : (Reg40 Bit<7:4>)
	   //0x04,0x08,0x0C,0x10,0x14.0x18,0x1C,0x20,0x24,0x28,0x2C,0x30,0x34,0x38,0x3C,0x40,0x44,0x48,0x4C,0x50,0x54,0x58,0x5C,0x60,0x64,0x68,0x6C,0x70,0x74,0x78,0x7C,0x80
		{0x52,0x63,0x35,0x40,0x01,0x00,0xC8,0x82,0xC8,0x29,0xD9,0x45,0xA0,0xAC,0x30,0x08,0x80,0x09,0x84,0x05,0x02,0x24,0x86,0x54,0xCF,0x24,0x01,0x00,0x00,0x01,0x80,0x90},
		{0xD1,0x1F,0x10,0x51,0x40,0xF8,0xC8,0x81,0xE8,0xBA,0xD8,0x45,0xA0,0xAC,0x30,0x08,0x80,0x09,0x84,0x05,0x02,0x24,0x86,0x54,0xA5,0x24,0x01,0x00,0x00,0x01,0x80,0x90},
		{0xD1,0x25,0x11,0x61,0x40,0x10,0xC8,0x83,0xE8,0xDF,0xD8,0x45,0xA0,0xAC,0x30,0x08,0x80,0x09,0x84,0x05,0x02,0x24,0x86,0x54,0x8A,0x24,0x01,0x00,0x00,0x01,0x80,0x90},
	},
	
	//36.000Mhz
	//We only support 8bit mode, 10bit and 12bit is same setting with 8bit. If you set 10bit and 12bit, there is no meaning.
	{
	   // TMDS Data Amplitude[4:0] : (Reg40 Bit<3:0>, REG3C Bit<7>), TMDS Clock Amplitude[3:0] : (Reg5C Bit<7:3>), TMDS Pre-emphasis Control : (Reg40 Bit<7:4>)
	   //0x04,0x08,0x0C,0x10,0x14.0x18,0x1C,0x20,0x24,0x28,0x2C,0x30,0x34,0x38,0x3C,0x40,0x44,0x48,0x4C,0x50,0x54,0x58,0x5C,0x60,0x64,0x68,0x6C,0x70,0x74,0x78,0x7C,0x80
		{0x51,0x2D,0x55,0x40,0x40,0x00,0xC8,0x02,0xC8,0x0E,0xD9,0x45,0xA0,0xAC,0x30,0x08,0x80,0x09,0x84,0x05,0x02,0x24,0x86,0x54,0xAB,0x24,0x00,0x00,0x00,0x01,0x80,0x90},
		{0x51,0x2D,0x55,0x40,0x40,0x00,0xC8,0x02,0xC8,0x0E,0xD9,0x45,0xA0,0xAC,0x30,0x08,0x80,0x09,0x84,0x05,0x02,0x24,0x86,0x54,0xAB,0x24,0x00,0x00,0x00,0x01,0x80,0x90},
		{0x51,0x2D,0x55,0x40,0x40,0x00,0xC8,0x02,0xC8,0x0E,0xD9,0x45,0xA0,0xAC,0x30,0x08,0x80,0x09,0x84,0x05,0x02,0x24,0x86,0x54,0xAB,0x24,0x00,0x00,0x00,0x01,0x80,0x90},
	},
	
	//40.000Mhz
	//We only support 8bit mode, 10bit and 12bit is same setting with 8bit. If you set 10bit and 12bit, there is no meaning.
	{
	   // TMDS Data Amplitude[4:0] : (Reg40 Bit<3:0>, REG3C Bit<7>), TMDS Clock Amplitude[3:0] : (Reg5C Bit<7:3>), TMDS Pre-emphasis Control : (Reg40 Bit<7:4>)
	   //0x04,0x08,0x0C,0x10,0x14.0x18,0x1C,0x20,0x24,0x28,0x2C,0x30,0x34,0x38,0x3C,0x40,0x44,0x48,0x4C,0x50,0x54,0x58,0x5C,0x60,0x64,0x68,0x6C,0x70,0x74,0x78,0x7C,0x80
		{0xD1,0x21,0x31,0x40,0x3C,0x28,0xC8,0x87,0xE8,0xC8,0xD8,0x45,0xA0,0xAC,0x30,0x08,0x80,0x09,0x84,0x05,0x02,0x24,0x86,0x54,0x9A,0x24,0x00,0x00,0x00,0x01,0x80,0x90},
		{0xD1,0x21,0x31,0x40,0x3C,0x28,0xC8,0x87,0xE8,0xC8,0xD8,0x45,0xA0,0xAC,0x30,0x08,0x80,0x09,0x84,0x05,0x02,0x24,0x86,0x54,0x9A,0x24,0x00,0x00,0x00,0x01,0x80,0x90},
		{0xD1,0x21,0x31,0x40,0x3C,0x28,0xC8,0x87,0xE8,0xC8,0xD8,0x45,0xA0,0xAC,0x30,0x08,0x80,0x09,0x84,0x05,0x02,0x24,0x86,0x54,0x9A,0x24,0x00,0x00,0x00,0x01,0x80,0x90},
	},
	
	//71.000Mhz
	//We only support 8bit mode, 10bit and 12bit is same setting with 8bit. If you set 10bit and 12bit, there is no meaning.
	{
	   // TMDS Data Amplitude[4:0] : (Reg40 Bit<3:0>, REG3C Bit<7>), TMDS Clock Amplitude[3:0] : (Reg5C Bit<7:3>), TMDS Pre-emphasis Control : (Reg40 Bit<7:4>)
	   //0x04,0x08,0x0C,0x10,0x14.0x18,0x1C,0x20,0x24,0x28,0x2C,0x30,0x34,0x38,0x3C,0x40,0x44,0x48,0x4C,0x50,0x54,0x58,0x5C,0x60,0x64,0x68,0x6C,0x70,0x74,0x78,0x7C,0x80
		{0xD1,0x3B,0x35,0x40,0x0C,0x04,0xC8,0x85,0xE8,0x63,0xD9,0x45,0xA0,0xAC,0x30,0x08,0x80,0x09,0x84,0x05,0x02,0x24,0x86,0x54,0x57,0x24,0x00,0x00,0x00,0x01,0x80,0x90},
		{0xD1,0x3B,0x35,0x40,0x0C,0x04,0xC8,0x85,0xE8,0x63,0xD9,0x45,0xA0,0xAC,0x30,0x08,0x80,0x09,0x84,0x05,0x02,0x24,0x86,0x54,0x57,0x24,0x00,0x00,0x00,0x01,0x80,0x90},
		{0xD1,0x3B,0x35,0x40,0x0C,0x04,0xC8,0x85,0xE8,0x63,0xD9,0x45,0xA0,0xAC,0x30,0x08,0x80,0x09,0x84,0x05,0x02,0x24,0x86,0x54,0x57,0x24,0x00,0x00,0x00,0x01,0x80,0x90},
	},
	
	//83.500Mhz
	//We only support 8bit mode, 10bit and 12bit is same setting with 8bit. If you set 10bit and 12bit, there is no meaning.
	{
	   // TMDS Data Amplitude[4:0] : (Reg40 Bit<3:0>, REG3C Bit<7>), TMDS Clock Amplitude[3:0] : (Reg5C Bit<7:3>), TMDS Pre-emphasis Control : (Reg40 Bit<7:4>)
	   //0x04,0x08,0x0C,0x10,0x14.0x18,0x1C,0x20,0x24,0x28,0x2C,0x30,0x34,0x38,0x3C,0x40,0x44,0x48,0x4C,0x50,0x54,0x58,0x5C,0x60,0x64,0x68,0x6C,0x70,0x74,0x78,0x7C,0x80
		{0xD1,0x23,0x11,0x40,0x0C,0xFB,0xC8,0x85,0xE8,0xD1,0xD8,0x45,0xA0,0xAC,0x30,0x08,0x80,0x09,0x84,0x05,0x02,0x24,0x86,0x54,0x4A,0x24,0x00,0x00,0x00,0x01,0x80,0x90},
		{0xD1,0x23,0x11,0x40,0x0C,0xFB,0xC8,0x85,0xE8,0xD1,0xD8,0x45,0xA0,0xAC,0x30,0x08,0x80,0x09,0x84,0x05,0x02,0x24,0x86,0x54,0x4A,0x24,0x00,0x00,0x00,0x01,0x80,0x90},
		{0xD1,0x23,0x11,0x40,0x0C,0xFB,0xC8,0x85,0xE8,0xD1,0xD8,0x45,0xA0,0xAC,0x30,0x08,0x80,0x09,0x84,0x05,0x02,0x24,0x86,0x54,0x4A,0x24,0x00,0x00,0x00,0x01,0x80,0x90},
	},
	
	//106.500Mhz
	//We only support 8bit mode, 10bit and 12bit is same setting with 8bit. If you set 10bit and 12bit, there is no meaning.
	{
	   // TMDS Data Amplitude[4:0] : (Reg40 Bit<3:0>, REG3C Bit<7>), TMDS Clock Amplitude[3:0] : (Reg5C Bit<7:3>), TMDS Pre-emphasis Control : (Reg40 Bit<7:4>)
	   //0x04,0x08,0x0C,0x10,0x14.0x18,0x1C,0x20,0x24,0x28,0x2C,0x30,0x34,0x38,0x3C,0x40,0x44,0x48,0x4C,0x50,0x54,0x58,0x5C,0x60,0x64,0x68,0x6C,0x70,0x74,0x78,0x7C,0x80
		{0xD1,0x2C,0x12,0x40,0x0C,0x09,0xC8,0x84,0xE8,0x0A,0xD9,0x45,0xA0,0xAC,0x30,0x08,0x80,0x09,0x84,0x05,0x02,0x24,0x86,0x54,0x73,0x24,0x01,0x00,0x00,0x01,0x80,0x90},
		{0xD1,0x2C,0x12,0x40,0x0C,0x09,0xC8,0x84,0xE8,0x0A,0xD9,0x45,0xA0,0xAC,0x30,0x08,0x80,0x09,0x84,0x05,0x02,0x24,0x86,0x54,0x73,0x24,0x01,0x00,0x00,0x01,0x80,0x90},
		{0xD1,0x2C,0x12,0x40,0x0C,0x09,0xC8,0x84,0xE8,0x0A,0xD9,0x45,0xA0,0xAC,0x30,0x08,0x80,0x09,0x84,0x05,0x02,0x24,0x86,0x54,0x73,0x24,0x01,0x00,0x00,0x01,0x80,0x90},
	},
	
	//122.500Mhz
	//We only support 8bit mode, 10bit and 12bit is same setting with 8bit. If you set 10bit and 12bit, there is no meaning.
	{
	   // TMDS Data Amplitude[4:0] : (Reg40 Bit<3:0>, REG3C Bit<7>), TMDS Clock Amplitude[3:0] : (Reg5C Bit<7:3>), TMDS Pre-emphasis Control : (Reg40 Bit<7:4>)
	   //0x04,0x08,0x0C,0x10,0x14.0x18,0x1C,0x20,0x24,0x28,0x2C,0x30,0x34,0x38,0x3C,0x40,0x44,0x48,0x4C,0x50,0x54,0x58,0x5C,0x60,0x64,0x68,0x6C,0x70,0x74,0x78,0x7C,0x80
		{0xD1,0x33,0x14,0x40,0x0C,0x01,0xC8,0x85,0xE8,0x32,0xD9,0x45,0xA0,0xAC,0x30,0x08,0x80,0x09,0x84,0x05,0x02,0x24,0x86,0x54,0x64,0x24,0x01,0x00,0x00,0x01,0x80,0x90},
		{0xD1,0x33,0x14,0x40,0x0C,0x01,0xC8,0x85,0xE8,0x32,0xD9,0x45,0xA0,0xAC,0x30,0x08,0x80,0x09,0x84,0x05,0x02,0x24,0x86,0x54,0x64,0x24,0x01,0x00,0x00,0x01,0x80,0x90},
		{0xD1,0x33,0x14,0x40,0x0C,0x01,0xC8,0x85,0xE8,0x32,0xD9,0x45,0xA0,0xAC,0x30,0x08,0x80,0x09,0x84,0x05,0x02,0x24,0x86,0x54,0x64,0x24,0x01,0x00,0x00,0x01,0x80,0x90},
	},
	
	//146.250Mhz
	//We only support 8bit mode, 10bit and 12bit is same setting with 8bit. If you set 10bit and 12bit, there is no meaning.
	{
	   // TMDS Data Amplitude[4:0] : (Reg40 Bit<3:0>, REG3C Bit<7>), TMDS Clock Amplitude[3:0] : (Reg5C Bit<7:3>), TMDS Pre-emphasis Control : (Reg40 Bit<7:4>)
	   //0x04,0x08,0x0C,0x10,0x14.0x18,0x1C,0x20,0x24,0x28,0x2C,0x30,0x34,0x38,0x3C,0x40,0x44,0x48,0x4C,0x50,0x54,0x58,0x5C,0x60,0x64,0x68,0x6C,0x70,0x74,0x78,0x7C,0x80
		{0xD1,0x3D,0x15,0x40,0x18,0xFD,0xC8,0x83,0xE8,0x6E,0xD9,0x45,0xA0,0xAC,0x30,0x0e,0x80,0x09,0x84,0x05,0x02,0x24,0xe4,0x54,0x54,0x24,0x01,0x00,0x00,0x01,0x80,0x90},
		{0xD1,0x3D,0x15,0x40,0x18,0xFD,0xC8,0x83,0xE8,0x6E,0xD9,0x45,0xA0,0xAC,0x30,0x0e,0x80,0x09,0x84,0x05,0x02,0x24,0xe4,0x54,0x54,0x24,0x01,0x00,0x00,0x01,0x80,0x90},
		{0xD1,0x3D,0x15,0x40,0x18,0xFD,0xC8,0x83,0xE8,0x6E,0xD9,0x45,0xA0,0xAC,0x30,0x0e,0x80,0x09,0x84,0x05,0x02,0x24,0xe4,0x54,0x54,0x24,0x01,0x00,0x00,0x01,0x80,0x90},
	},

#if 0
        //97.34Mhz 
		//  change to lcdc clock        nPCLKCTRL.sel = PCLKCTRL_SEL_HDMITMDS;
       //We only support 8bit mode, 10bit and 12bit is same setting with 8bit. If you set 10bit and 12bit, there is no meaning.
    {
        {0xD1,0x29,0x12,0x61,0x40,0xD0,0xC8,0x87,0xE8,0xF4,0xD8,0x45,0xA0,0xAC,0x30,0x08,0x80,0x09,0x84,0x05,0x02,0x24,0x86,0x54,0x7E,0x24,0x01,0x00,0x00,0x01,0x80,0x90},
        {0xD1,0x29,0x12,0x61,0x40,0xD0,0xC8,0x87,0xE8,0xF4,0xD8,0x45,0xA0,0xAC,0x30,0x08,0x80,0x09,0x84,0x05,0x02,0x24,0x86,0x54,0x7E,0x24,0x01,0x00,0x00,0x01,0x80,0x90},
        {0xD1,0x29,0x12,0x61,0x40,0xD0,0xC8,0x87,0xE8,0xF4,0xD8,0x45,0xA0,0xAC,0x30,0x08,0x80,0x09,0x84,0x05,0x02,0x24,0x86,0x54,0x7E,0x24,0x01,0x00,0x00,0x01,0x80,0x90},
    }
#else
	{
	    {0xD1,0x29,0x12,0x40,0x3C,0xCB,0xC8,0x87,0xE8,0xF3,0xD8,0x45,0xA0,0xAC,0x80,0x08,0x80,0x09,0x84,0x05,0x02,0x24,0x86,0x54,0x7E,0x24,0x01,0x00,0x00,0x01,0x80,0x90},
	    {0xD1,0x29,0x12,0x40,0x3C,0xCB,0xC8,0x87,0xE8,0xF3,0xD8,0x45,0xA0,0xAC,0x80,0x08,0x80,0x09,0x84,0x05,0x02,0x24,0x86,0x54,0x7E,0x24,0x01,0x00,0x00,0x01,0x80,0x90},
	    {0xD1,0x29,0x12,0x40,0x3C,0xCB,0xC8,0x87,0xE8,0xF3,0xD8,0x45,0xA0,0xAC,0x80,0x08,0x80,0x09,0x84,0x05,0x02,0x24,0x86,0x54,0x7E,0x24,0x01,0x00,0x00,0x01,0x80,0x90},	        
	}
#endif//
};
#endif

static int /*__init*/ hdmi_init(void);
static void /*__init*/ hdmi_exit(void);
static int hdmi_open(struct inode *inode, struct file *file);
static int hdmi_release(struct inode *inode, struct file *file);
//static irqreturn_t hdmi_handler(int irq, void *dev_id);
static ssize_t hdmi_read(struct file *file, char __user *buffer, size_t count, loff_t *ppos);
static ssize_t hdmi_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos);
static unsigned int hdmi_poll(struct file *file, poll_table *wait);
static long hdmi_ioctl(struct file *file, unsigned int cmd, unsigned long arg);

static void hdmi_spd_update_checksum(void);
static int hdmi_set_spd_infoframe(struct HDMIVideoFormatCtrl VideoFormatCtrl);

static void hdmi_avi_update_checksum(void);
static void hdmi_aui_update_checksum(void);
static int hdmi_set_hdmimode(int mode);
static int hdmi_set_colorimetry(enum HDMIColorimetry);
static int hdmi_set_audio_sample_freq(enum SamplingFreq);
static int hdmi_set_audio_packet_type(enum HDMIASPType);
static int hdmi_set_audio_number_of_channels(enum ChannelNum);
static void hdmi_enable_bluescreen(unsigned char enable);
static int hdmi_set_aux_data(struct HDMIVideoParameter* pVideo);
static void hdmi_set_default_value(void);
static int hdmi_set_2D_video(enum VideoFormat format);
static int hdmi_set_3D_FP_video(enum VideoFormat format);
static int hdmi_set_3D_SSH_video(enum VideoFormat format);
static int hdmi_set_3D_TB_video(enum VideoFormat format);
static int hdmi_set_3D_FA_video(enum VideoFormat format);
static int hdmi_set_3D_LA_video(enum VideoFormat format);
static int hdmi_set_3D_SSF_video(enum VideoFormat format);
static int hdmi_set_3D_LD_video(enum VideoFormat format);
static int hdmi_set_3D_LDGFX_video(enum VideoFormat format);
static int hdmi_fill_one_subpacket(unsigned char enable);

#if defined(CONFIG_ARCH_TCC893X)
static void tcc_ddi_reg_acess_hdmi(void);
#endif

#if defined(CONFIG_ARCH_TCC896X) || defined(CONFIG_ARCH_TCC897X) || defined(CONFIG_ARCH_TCC570X)
static int hdmi_set_phy_freq(enum PHYFreq freq);
static int hdmi_set_phy_pwdn(unsigned char enable);
#endif

// for debugging
static void hdmi_core_debug(void);
static void hdmi_core_video_debug(void);

#if defined(CONFIG_TCC_OUTPUT_COLOR_SPACE_YUV)
int hdmi_set_yuv420_color_space(void);
#endif
int hdmi_set_color_space(enum ColorSpace);
int hdmi_set_color_depth(enum ColorDepth);
int hdmi_set_video_mode(struct HDMIVideoParameter* pVideo);
int hdmi_set_pixel_limit(enum PixelLimit);
int hdmi_set_pixel_aspect_ratio(enum PixelAspectRatio);

void hdmi_start(void);
void hdmi_stop(void);
void hdmi_audio_start(void);
void hdmi_audio_stop(void);
void hdmi_phy_reset(void);

void tcc_hdmi_power_on(void);
void tcc_hdmi_power_off(void);
void tcc_hdmi_v5_power_on(void);
void tcc_hdmi_v5_power_off(void);

static unsigned int gHDMISuspendStatus = 0;
static unsigned int gLinuxHDMISuspendStatus = 0;

struct HDMIVideoParameter gHdmiVideoParms;
static unsigned int gHdmiStartFlag=0;
static unsigned int gPixelLimit=0;
static unsigned int gAudioChNum=0;
static unsigned int gSampleFreq=0;
static unsigned int gOutPacket=0;

#ifdef CONFIG_REGULATOR
static struct regulator         *vdd_v5p0 = NULL;
static struct regulator         *vdd_hdmi = NULL;
#endif

#if defined(CONFIG_SWITCH_GPIO_HDMI)
static struct platform_device tcc_hdmi_detect_device = {
	.name   = "switch-gpio-hdmi-detect",
	.id             = -1,
	.dev = {
		.platform_data = NULL,
	}, 
};
#endif

#define	PWR_STATUS_OFF		0
#define	PWR_STATUS_ON		1
typedef struct {
	int status;
}stpwrinfo;
static stpwrinfo gHdmiPwrInfo = {PWR_STATUS_OFF};

static struct device *pdev_hdmi;

static const struct file_operations hdmi_fops =
{
    .owner          = THIS_MODULE,
    .open           = hdmi_open,
    .release        = hdmi_release,
    .read           = hdmi_read,
    .write          = hdmi_write,
    .unlocked_ioctl = hdmi_ioctl,
	.poll			= hdmi_poll,
};

static struct miscdevice hdmi_misc_device =
{
    HDMI_MINOR,
    "hdmi",  //"HDMI",
    &hdmi_fops,
};

static struct device *pdev_hdmi;

static int hdmi_open(struct inode *inode, struct file *file)
{
    	
	if(phdmi_data != NULL) {
		phdmi_data->open_cnt++;
		dprintk(KERN_INFO "%s open_num:%d\n", __FUNCTION__, phdmi_data->open_cnt);
	}
	else
		pr_err("%s phdmi_data is NULL \n", __FUNCTION__);

    	return 0;
}

static int hdmi_release(struct inode *inode, struct file *file)
{
	if(phdmi_data != NULL) {
		phdmi_data->open_cnt--;
		dprintk(KERN_INFO "%s open_num:%d\n", __FUNCTION__, phdmi_data->open_cnt);
	}
	else
		pr_err("%s phdmi_data is NULL \n", __FUNCTION__);		
	return 0;
}

ssize_t hdmi_read(struct file *file, char __user *buffer, size_t count, loff_t *ppos)
{
    return 0;
}

ssize_t hdmi_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
    return 0;
}

unsigned int hdmi_poll(struct file *file, poll_table *wait)
{
	unsigned int mask = 0;
	return mask;
}

/**
 *      tccfb_blank
 *	@blank_mode: the blank mode we want.
 *	@info: frame buffer structure that represents a single frame buffer
 *
 *	Blank the screen if blank_mode != 0, else unblank. Return 0 if
 *	blanking succeeded, != 0 if un-/blanking failed due to e.g. a
 *	video mode which doesn't support it. Implements VESA suspend
 *	and powerdown modes on hardware that supports disabling hsync/vsync:
 *	blank_mode == 2: suspend vsync
 *	blank_mode == 3: suspend hsync
 *	blank_mode == 4: powerdown
 *
 *	Returns negative errno on error, or zero on success.
 *
 */
#ifdef CONFIG_PM_RUNTIME

static int hdmi_enable(void)
{		
	printk("%s\n", __func__);

	pm_runtime_get_sync(pdev_hdmi);

	return 0;
}

static int hdmi_disable(void)
{
	printk("%s\n", __func__);

	pm_runtime_put_sync(pdev_hdmi);

	return 0;
}

#endif

static int hdmi_blank(int blank_mode)
{
	int ret = 0;

	printk("%s : blank(mode=%d)\n",__func__, blank_mode);

#ifdef CONFIG_PM_RUNTIME
	if( (pdev_hdmi->power.usage_count.counter==1) && (blank_mode == 0))
	{
		// usage_count = 1 ( resume ), blank_mode = 0 ( FB_BLANK_UNBLANK ) is stable state when booting
		// don't call runtime_suspend or resume state 
	      //printk("%s ### state = [%d] count =[%d] power_cnt=[%d] \n",__func__,blank_mode, pdev_hdmi->power.usage_count, pdev_hdmi->power.usage_count.counter);		  
		return 0;
	}

	switch(blank_mode)
	{
		case FB_BLANK_POWERDOWN:
		case FB_BLANK_NORMAL:
			hdmi_disable();
			break;
		case FB_BLANK_UNBLANK:
			hdmi_enable();
			break;
		case FB_BLANK_HSYNC_SUSPEND:
		case FB_BLANK_VSYNC_SUSPEND:
		default:
			ret = -EINVAL;
	}
#endif

	return ret;
}

long hdmi_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{

    switch (cmd)
    {
		case HDMI_IOC_GET_PWR_STATUS:
		{
			dprintk(KERN_INFO "HDMI: ioctl(HDMI_IOC_GET_PWR_STATUS : %d )\n", gHdmiPwrInfo.status);

			put_user(gHdmiPwrInfo.status,(unsigned int __user*)arg);

			break;
		}
		case HDMI_IOC_SET_PWR_CONTROL:
		{
			unsigned int cmd;

			if (get_user(cmd, (unsigned int __user *) arg))
				return -EFAULT;

			dprintk(KERN_INFO "HDMI: ioctl(HDMI_IOC_SET_PWR_CONTROL :  %d )\n", cmd);
			if (cmd == 0) {
				//For Phy Pwdn
				gHdmiPwrInfo.status = PWR_STATUS_OFF;
				//tcc_hdmi_power_off();
			} else {
				tcc_hdmi_power_on();
			}
			break;
		}
		case HDMI_IOC_SET_PWR_V5_CONTROL:
		{
			unsigned int cmd;

			if (get_user(cmd, (unsigned int __user *) arg))
				return -EFAULT;

			dprintk(KERN_INFO "HDMI: ioctl(HDMI_IOC_SET_PWR_V5_CONTROL :  %d )\n", cmd);
			if (cmd == 0)
				tcc_hdmi_v5_power_off();
			else
				tcc_hdmi_v5_power_on();
			break;
		}
		case HDMI_IOC_GET_SUSPEND_STATUS:
		{
			//dprintk(KERN_INFO "HDMI: ioctl(HDMI_IOC_GET_SUSPEND_STATUS : %d )\n", gHDMISuspendStatus);

			if (gLinuxHDMISuspendStatus) {
				put_user(gLinuxHDMISuspendStatus,(unsigned int __user*)arg);
				gLinuxHDMISuspendStatus = 0;
			} else {
				put_user(gHDMISuspendStatus,(unsigned int __user*)arg);
			}

			break;
		}
        case HDMI_IOC_SET_COLORSPACE:
        {
            int space;

            dprintk(KERN_INFO "HDMI: ioctl(HDMI_IOC_SET_COLORSPACE)\n");

            // get arg
            if (get_user(space, (int __user *) arg))
                return -EFAULT;

            if ( !hdmi_set_color_space(space) )
            {
                dprintk(KERN_INFO "HDMI: ioctl(HDMI_IOC_SET_COLORSPACE) : Not Correct Arg = %d\n", space);
                return -EFAULT;
            }

			gHdmiVideoParms.colorSpace = space;

            break;
        }
        case HDMI_IOC_SET_COLORDEPTH:
        {
            int depth;

            dprintk(KERN_INFO "HDMI: ioctl(HDMI_IOC_SET_COLORDEPTH)\n");

            // get arg
            if (get_user(depth, (int __user *) arg))
                return -EFAULT;

            if ( !hdmi_set_color_depth(depth) )
            {
                dprintk(KERN_INFO "HDMI: ioctl(HDMI_IOC_SET_COLORDEPTH) : Not Correct Arg = %d\n", depth);
                return -EFAULT;
            }

			gHdmiVideoParms.colorDepth = depth;

            break;
        }
        case HDMI_IOC_SET_HDMIMODE:
        {
            int mode;

            dprintk(KERN_INFO "HDMI: ioctl(HDMI_IOC_SET_HDMIMODE)\n");

            // get arg
            if (get_user(mode, (int __user *) arg))
	            return -EFAULT;

			hdmi_set_hdmimode(mode);
			
			gHdmiVideoParms.mode = mode;

            break;
        }
        case HDMI_IOC_SET_VIDEOMODE:
        {
            unsigned int ret;
			struct HDMIVideoParameter video_mode;

            dprintk(KERN_INFO "HDMI: ioctl(HDMI_IOC_SET_VIDEOMODE)\n");

            // get arg
			if ( (ret = copy_from_user((void*)&video_mode,(const void*)arg,sizeof(struct HDMIVideoParameter))) < 0)
				return -EFAULT;

			if (!hdmi_set_video_mode(&video_mode))
                return -EFAULT;

#ifdef HDMI_TX13_REV_05
           video_wrapper_set_mode(video_mode.hdmi_3d_format, HDMIVideoParams[video_mode.resolution]);
#endif

            break;
        }
		
        case HDMI_IOC_SET_VIDEOFORMAT_INFO:
        {
            enum VideoFormat video_format;
            unsigned int ret;

            dprintk(KERN_INFO "HDMI: ioctl(HDMI_IOC_SET_VIDEOFORMAT_INFO)\n");

            // get arg
            if ( (ret = copy_from_user((void*)&video_format,(const void*)arg,sizeof(enum VideoFormat))) < 0)
            {
                return -EFAULT;
            }

			gHdmiVideoParms.resolution = video_format;

            break;
        }

        case HDMI_IOC_GET_VIDEOCONFIG:
        {
            int ret;

            dprintk(KERN_INFO "HDMI: ioctl(HDMI_IOC_GET_VIDEOCONFIG)\n");

            // copy to user
            if ( (ret = copy_to_user((void*)arg,(const void*)&gHdmiVideoParms,sizeof(struct HDMIVideoParameter))) < 0)
                return -EFAULT;

            break;
        }

        case HDMI_IOC_GET_HDMISTART_STATUS:
        {
            dprintk(KERN_INFO "HDMI: ioctl(HDMI_IOC_GET_HDMISTART_STATUS)\n");

            put_user(gHdmiStartFlag,(unsigned int __user*)arg);

            break;
        }

		case HDMI_IOC_SET_LCDC_TIMING:
		{

			struct device_lcdc_timing_params lcdc_mode;
            unsigned int ret;

            dprintk(KERN_INFO "HDMI: ioctl(HDMI_IOC_SET_LCDC_TIMING)\n");

            // get arg
			if ( (ret = copy_from_user((void*)&lcdc_mode,(const void*)arg,sizeof(struct device_lcdc_timing_params))) < 0)	{
			    return -EFAULT;
			}

			#if 0	// Not use
			#if 1
            hdmi_set_lcdc_timing(lcdc_mode);
			#else
			memcpy(&lcdc_timing_params, &lcdc_mode, sizeof(struct device_lcdc_timing_params));
			#endif
			#endif//
            break;
		}

        case HDMI_IOC_SET_BLUESCREEN:
        {
            unsigned char val;

            dprintk(KERN_INFO "HDMI: ioctl(HDMI_IOC_SET_BLUESCREEN)\n");

            // get arg
            if (get_user(val, (unsigned char __user *) arg))
                return -EFAULT;

            if (val) // if on
            {
				dprintk(" ON\n");
				hdmi_enable_bluescreen(1);
            }
            else // if off
            {
				dprintk(" OFF\n");
				hdmi_enable_bluescreen(0);
            }

            break;
        }
        case HDMI_IOC_SET_PIXEL_LIMIT:
        {
            int val;

            dprintk(KERN_INFO "HDMI: ioctl(HDMI_IOC_SET_PIXEL_LIMIT)\n");

            // get arg
            if (get_user(val, (int __user *) arg))
                return -EFAULT;

            if (!hdmi_set_pixel_limit(val))
            {
                dprintk(KERN_INFO "Not available Arg\n");
                return -EFAULT;
            }

			gPixelLimit = val;

            break;
        }
        case HDMI_IOC_GET_PIXEL_LIMIT:
        {
            dprintk(KERN_INFO "HDMI: ioctl(HDMI_IOC_GET_PIXEL_LIMIT)\n");

            // put to user
            if (put_user(gPixelLimit, (int __user *) arg))
                return -EFAULT;

            break;
        }

        case HDMI_IOC_SET_PIXEL_ASPECT_RATIO:
        {
            int val;
            dprintk(KERN_INFO "HDMI: ioctl(HDMI_IOC_SET_PIXEL_ASPECT_RATIO)\n");
            if (get_user(val, (int __user *) arg))
                return -EFAULT;

            if (!hdmi_set_pixel_aspect_ratio(val))
            {
                dprintk(KERN_INFO "Not available Arg\n");
                return -EFAULT;
            }

			gHdmiVideoParms.pixelAspectRatio = val;
			
            break;
        }
	case HDMI_IOC_SET_COLORIMETRY:
	{
		int val;

		dprintk(KERN_INFO "HDMI: ioctl(HDMI_IOC_SET_COLORIMETRY)\n");

		if (get_user(val, (int __user *) arg))
			return -EFAULT;

		if (!hdmi_set_colorimetry(val))
		{
			dprintk(KERN_INFO "invalid argument!\n");
			return -EFAULT;
		}
		break;
	}
        case HDMI_IOC_SET_AVMUTE:
        {
            unsigned char val,reg;

            dprintk(KERN_INFO "HDMI: ioctl(HDMI_IOC_SET_AVMUTE)\n");

            // get arg
            if (get_user(val, (unsigned int __user *) arg))
                return -EFAULT;

            reg = hdmi_readb(HDMI_MODE_SEL) & HDMI_MODE_SEL_HDMI;
            if (reg)
            {
                if (val)
                {
                    // set AV Mute
                    hdmi_writeb(GCP_AVMUTE_ON,HDMI_GCP_BYTE1);
                    hdmi_writeb(GCP_TRANSMIT_EVERY_VSYNC,HDMI_GCP_CON);
                }
                else
                {
                    // clear AV Mute
                    hdmi_writeb(GCP_AVMUTE_OFF, HDMI_GCP_BYTE1);
                    hdmi_writeb(GCP_TRANSMIT_EVERY_VSYNC,HDMI_GCP_CON);
                }
            }

            break;
        }
        case HDMI_IOC_SET_AUDIOPACKETTYPE:
        {
            int val;

            dprintk(KERN_INFO "HDMI: ioctl(HDMI_IOC_SET_AUDIOPACKETTYPE)\n");

            // get arg
            if (get_user(val, (int __user *) arg))
                return -EFAULT;

            if (!hdmi_set_audio_packet_type(val))
            {
                dprintk(KERN_INFO "Not available Arg\n");
                return -EFAULT;
            }

			gOutPacket = val;

            break;
        }
        case HDMI_IOC_GET_AUDIOPACKETTYPE:
        {
            dprintk(KERN_INFO "HDMI: ioctl(HDMI_IOC_GET_AUDIOPACKETTYPE)\n");

			put_user(gOutPacket,(unsigned int __user*)arg);

            break;
        }

        case HDMI_IOC_SET_AUDIOSAMPLEFREQ:
        {
            int val;
//            unsigned char reg = hdmi_readb(HDMI_CON_0);
            dprintk(KERN_INFO "HDMI: ioctl(HDMI_IOC_SET_AUDIOSAMPLEFREQ)\n");

            // get arg
            if (get_user(val, (int __user *) arg))
                return -EFAULT;

            if ( !hdmi_set_audio_sample_freq(val) )
            {
                dprintk(KERN_INFO "Not available Arg\n");
                return -EFAULT;
            }
#if 0
            // set audio enable
            hdmi_writeb(reg|HDMI_ASP_ENABLE ,HDMI_CON_0);
#endif /* 0 */

			gSampleFreq = val;
			
            break;
        }
        case HDMI_IOC_GET_AUDIOSAMPLEFREQ:
        {
            dprintk(KERN_INFO "HDMI: ioctl(HDMI_IOC_GET_AUDIOSAMPLEFREQ)\n");

			put_user(gSampleFreq,(unsigned int __user*)arg);
			
            break;
        }

        case HDMI_IOC_SET_AUDIOCHANNEL:
        {
            int val;

            dprintk(KERN_INFO "HDMI: ioctl(HDMI_IOC_SET_AUDIOCHANNEL)\n");

            // get arg
            if (get_user(val, (int __user *) arg))
                return -EFAULT;

            if (!hdmi_set_audio_number_of_channels(val))
            {
                dprintk(KERN_INFO "Not available Arg\n");
                return -EFAULT;
            }

			gAudioChNum = val;

            break;
        }
        case HDMI_IOC_GET_AUDIOCHANNEL:
        {
            dprintk(KERN_INFO "HDMI: ioctl(HDMI_IOC_GET_AUDIOCHANNEL)\n");

			put_user(gAudioChNum,(unsigned int __user*)arg);			

            break;
        }

        case HDMI_IOC_SET_SPEAKER_ALLOCATION:
        {
            unsigned int val;

            dprintk(KERN_INFO "HDMI: ioctl(HDMI_IOC_SET_SPEAKER_ALLOCATION)\n");

            // get arg
            if (get_user(val, (unsigned int __user *) arg))
                return -EFAULT;

            hdmi_writeb(val,HDMI_AUI_BYTE4);

            break;
        }
        case HDMI_IOC_GET_SPEAKER_ALLOCATION:
        {
            unsigned int val;

            dprintk(KERN_INFO "HDMI: ioctl(HDMI_IOC_GET_SPEAKER_ALLOCATION)\n");

            val = hdmi_readb(HDMI_AUI_BYTE4);

            put_user(val,(unsigned int __user*)arg);

            break;
        }
		
        case HDMI_IOC_GET_PHYREADY:
        {
            unsigned char phy_status;

            //dprintk(KERN_INFO "HDMI: ioctl(HDMI_IOC_GET_PHYREADY)\n");

            phy_status = hdmi_readb(HDMI_SS_PHY_STATUS_0);

            put_user(phy_status,(unsigned char __user*)arg);

            break;
        }

		case HDMI_IOC_SET_PHYRESET:
		{
			dprintk(KERN_INFO "HDMI: ioctl(HDMI_IOC_SET_PHYRESET)\n");
			hdmi_phy_reset();
			break;
		}
		
		case HDMI_IOC_SET_HPD_SWITCH_STATUS	:
		{
			dprintk(KERN_INFO "HDMI: ioctl(HDMI_IOC_SET_HPD_SWITCH_STATUS)\n");
			#if defined(CONFIG_SWITCH_GPIO_HDMI)
			if(tcc_hdmi_detect_device.dev.platform_data != NULL)
			{
				struct hdmi_gpio_switch_data	*switch_data = tcc_hdmi_detect_device.dev.platform_data;
				switch_data->send_hdp_event(switch_data, TCC_HDMI_ON);
			}
			#endif//
			break;
		}

       	case HDMI_IOC_START_HDMI:
		{
			dprintk(KERN_INFO "HDMI: ioctl(HDMI_IOC_START_HDMI)\n");

			#ifdef HDMI_TX13_REV_05
			video_wrapper_enable(1);
			#endif

			#if defined(CONFIG_TCC_OUTPUT_COLOR_SPACE_YUV)
				if(gHdmiVideoParms.mode == HDMI)
					hdmi_set_yuv420_color_space();
			#endif

			hdmi_start();
          		break;
		}
        case HDMI_IOC_STOP_HDMI:
        {
            dprintk(KERN_INFO "HDMI: ioctl(HDMI_IOC_STOP_HDMI)\n");

		#if defined(CONFIG_SWITCH_GPIO_HDMI)
		if(tcc_hdmi_detect_device.dev.platform_data != NULL)
		{
			struct hdmi_gpio_switch_data	*switch_data = tcc_hdmi_detect_device.dev.platform_data;
			switch_data->send_hdp_event(switch_data, TCC_HDMI_OFF);
		}
		#endif//
		hdmi_stop();
		break;

        }
        case HDMI_IOC_SET_AUDIO_ENABLE:
        {
            unsigned char enable;
            unsigned char reg, mode;

            dprintk(KERN_INFO "HDMI: ioctl(HDMI_IOC_SET_AUDIO_ENABLE)\n");

            // get arg
            if (get_user(enable, (int __user *) arg))
                return -EFAULT;

			// check HDMI mode
			mode = hdmi_readb(HDMI_MODE_SEL) & HDMI_MODE_SEL_HDMI;

            reg = hdmi_readb(HDMI_CON_0);
            // enable audio output
            if ( enable && mode )
            {
                hdmi_aui_update_checksum();
                hdmi_writeb(TRANSMIT_EVERY_VSYNC,HDMI_AUI_CON);
	        //  hdmi_writeb(TRANSMIT_ONCE,HDMI_AUI_CON);
				hdmi_writeb(ACR_MEASURED_CTS_MODE,HDMI_ACR_CON);
                hdmi_writeb(reg|HDMI_ASP_ENABLE,HDMI_CON_0);

				dprintk(KERN_INFO "HDMI: ioctl(HDMI_IOC_SET_AUDIO_ENABLE) : enable\n");
            }
            else // disable encryption
            {
	            hdmi_writeb(reg& ~HDMI_ASP_ENABLE,HDMI_CON_0);
                hdmi_writeb(DO_NOT_TRANSMIT,HDMI_AUI_CON);
                hdmi_writeb(DO_NOT_TRANSMIT,HDMI_ACR_CON);
				dprintk(KERN_INFO "HDMI: ioctl(HDMI_IOC_SET_AUDIO_ENABLE) : disable\n");
            }

            break;
        }
        case HDMI_IOC_RESET_AUISAMPLEFREQ:
        {
            unsigned char reg = hdmi_readb(HDMI_AUI_BYTE2) & ~HDMI_AUI_SF_MASK;
            hdmi_writeb(reg, HDMI_AUI_BYTE2);
            break;
        }
		case HDMI_IOC_VIDEO_FORMAT_CONTROL:
		{
            struct HDMIVideoFormatCtrl video_format_ctrl;
            int ret;

            //dprintk(KERN_INFO "HDMI: ioctl(HDMI_IOC_VIDEO_FORMAT_CONTROL)\n");

            // get size arg;
            if ( (ret = copy_from_user((void*)&video_format_ctrl,(const void*)arg,sizeof(video_format_ctrl))) < 0)
                return -EFAULT;

			hdmi_set_spd_infoframe( video_format_ctrl );

			hdmi_writeb(TRANSMIT_EVERY_VSYNC,HDMI_SPD_CON);

			break;
		}

		case HDMI_IOC_SET_VIDEO_SOURCE:
		{
			int mode;

			dprintk(KERN_INFO "HDMI: ioctl(HDMI_IOC_SET_VIDEO_SOURCE)\n");

			// get arg
			if (get_user(mode, (int __user *) arg))
				return -EFAULT;

			if (mode == HDMI_SOURCE_EXTERNAL)
			{
				hdmi_writeb(HDMI_EXTERNAL_VIDEO,HDMI_VIDEO_PATTERN_GEN);

			}
			else
			{
				hdmi_writeb(HDMI_INTERNAL_VIDEO,HDMI_VIDEO_PATTERN_GEN);
			}
			break;
		}
		case HDMI_IOC_DEBUG_HDMI_CORE:
		{
			hdmi_core_debug();
			break;
		}
		case HDMI_IOC_DEBUG_HDMI_CORE_VIDEO:
		{
			hdmi_core_video_debug();
			break;
		}
		case HDMI_IOC_FILL_ONE_SUBPACKET:
		{
			unsigned char enable;

			dprintk(KERN_INFO "HDMI: ioctl(HDMI_IOC_FILL_ONE_SUBPACKET)");

			// get arg
			if (get_user(enable, (unsigned char __user *) arg))
				return -EFAULT;

			hdmi_fill_one_subpacket(enable);
			break;
		}

		#if defined(CONFIG_ARCH_TCC896X) || defined(CONFIG_ARCH_TCC897X) || defined(CONFIG_ARCH_TCC570X)
		case HDMI_IOC_SET_PHY_FREQ:
		{
            int phy_freq;
			dprintk(KERN_INFO "HDMI: ioctl(HDMI_IOC_SET_PHY_FREQ)\n");

            // get arg
            if (get_user(phy_freq, (int __user *) arg))
                return -EFAULT;

            if ( !hdmi_set_phy_freq(phy_freq) )
            {
                dprintk(KERN_INFO "HDMI: ioctl(HDMI_IOC_SET_PHY_FREQ) : Not Correct Arg = %d\n", phy_freq);
                return -EFAULT;
            }

			break;
		}

		case HDMI_IOC_SET_PHY_INDEX:
		{
			dprintk(KERN_INFO "HDMI: ioctl(HDMI_IOC_SET_PHY_INDEX)\n");

			// get arg
			if (get_user(gHDMIFreqIndex, (unsigned char __user *) arg))
				return -EFAULT;

			break;
		}
		#endif

		case HDMI_IOC_SET_DEBUG:
		{
			struct hdmi_dbg dbg;
			int ret;
			printk(KERN_ERR "HDMI_IOC_SET_DEBUG\n");

			// get size arg;
			if ( (ret = copy_from_user((void*)&dbg,(const void*)arg,sizeof(struct hdmi_dbg))) < 0)
				return -EFAULT;

			printk(KERN_ERR "write value to reg\n");
			//hdmi_writeb(dbg.value, regs_core + dbg.offset);
			break;
		}

		case HDMI_IOC_BLANK:
		{
			unsigned int cmd;

			if (get_user(cmd, (unsigned int __user *) arg))
				return -EFAULT;

			printk(KERN_INFO "HDMI: ioctl(HDMI_IOC_BLANK :  %d )\n", cmd);

			hdmi_blank(cmd);

			break;

		}

        default:
            return -EINVAL;
    }

    return 0;
}

#if defined(CONFIG_ARCH_TCC893X)
void tcc_ddi_reg_acess_hdmi(void)
{
	unsigned int regl;

	regl = hdmi_readl(DDICFG_PWDN);
	hdmi_writel(regl & ~(0xf << 8), DDICFG_PWDN);

 }
#endif

// onoff : 1 - power down
void tcc_ddi_pwdn_hdmi(char onoff)
{
	unsigned int  regl;

	// HDMI Power-on
	regl = hdmi_readl(DDICFG_PWDN);

	if(onoff)
		hdmi_writel(regl & ~PWDN_HDMI, DDICFG_PWDN);	
	else
		hdmi_writel(regl | PWDN_HDMI, DDICFG_PWDN);	

}

// onoff : 1 -reset
void tcc_ddi_swreset_hdmi(char onoff)
{
	unsigned int  regl;

	regl = hdmi_readl(DDICFG_SWRESET);

	if(onoff)
		hdmi_writel(regl & ~SWRESET_HDMI, DDICFG_SWRESET);
	else
		hdmi_writel(regl | SWRESET_HDMI, DDICFG_SWRESET);	

}

// onoff : 1 -power on
void tcc_ddi_hdmi_ctrl(unsigned int index, char onoff)
{
	unsigned int regl;

	regl = hdmi_readl(DDICFG_HDMICTRL);
	
	switch(index)
	{
		case HDMICTRL_RESET_HDMI:
		{
			if(onoff)
				hdmi_writel(regl & ~HDMICTRL_RESET_HDMI, DDICFG_HDMICTRL);
			else
				hdmi_writel(regl | HDMICTRL_RESET_HDMI, DDICFG_HDMICTRL);

		}
		break;

		case HDMICTRL_RESET_SPDIF:
		{
			if(onoff)
				hdmi_writel(regl & ~HDMICTRL_RESET_SPDIF, DDICFG_HDMICTRL);
			else
				hdmi_writel(regl | HDMICTRL_RESET_SPDIF, DDICFG_HDMICTRL);
		}
		break;
		
		case HDMICTRL_RESET_TMDS:
		{
			if(onoff)
				hdmi_writel(regl & ~HDMICTRL_RESET_TMDS, DDICFG_HDMICTRL);
			else
				hdmi_writel(regl | HDMICTRL_RESET_TMDS, DDICFG_HDMICTRL);
		}
		break;
		
		case HDMICTRL_HDMI_ENABLE:
		{
			if(onoff)
			{
				hdmi_writel(regl | HDMICTRL_HDMI_ENABLE, DDICFG_HDMICTRL);
				
			}		
			else
			{
				hdmi_writel(regl & ~HDMICTRL_HDMI_ENABLE, DDICFG_HDMICTRL);
			}


		}
		break;
	}
}

void tcc_hdmi_power_on(void)
{
	unsigned int  regl;
#if defined(CONFIG_REGULATOR)
	int ret;
#endif

	dprintk(KERN_INFO "%s Start\n", __FUNCTION__);

#if defined(CONFIG_REGULATOR)
	if(vdd_hdmi) {
		ret = regulator_enable(vdd_hdmi);
		if (ret)
			pr_err("failed to enable hdmi regulator: %d\n", ret);
	}
#endif

	if (hdmi_pclk && hdmi_hclk && (gHdmiPwrInfo.status == PWR_STATUS_OFF)) {
		clk_prepare_enable(hdmi_pclk);
		clk_prepare_enable(hdmi_hclk);
	}

	udelay(100);

	if (hdmi_pclk)
		clk_set_rate(hdmi_pclk, XIN_CLK_RATE);
	
#if defined(CONFIG_ARCH_TCC896X) || defined(CONFIG_ARCH_TCC897X) || defined(CONFIG_ARCH_TCC570X)
	if(hdmi_ipclk) {
		clk_prepare_enable(hdmi_ipclk);
		clk_set_rate(hdmi_ipclk, 50*1000*1000);
	}
#endif

	// HDMI Power-on
	tcc_ddi_pwdn_hdmi(0);

	regl = hdmi_readl(DDICFG_PWDN);
	hdmi_writel(regl & ~(0xf << 8), DDICFG_PWDN);

	hdmi_phy_reset();

	if(hdmi_lcdc_num)
		VIOC_OUTCFG_SetOutConfig(VIOC_OUTCFG_HDMI, 1);
	else
		VIOC_OUTCFG_SetOutConfig(VIOC_OUTCFG_HDMI, 0);

	tcc_ddi_hdmi_ctrl(HDMICTRL_HDMI_ENABLE, 1);

#if defined(CONFIG_ARCH_TCC893X)
	memset(&gHdmiVideoParms, 0, sizeof(struct HDMIVideoParameter));

	// default video mode setting.
	memset(&gHdmiVideoParms, 0, sizeof(struct HDMIVideoParameter));

	gHdmiVideoParms.mode = DVI;
	gHdmiVideoParms.resolution = max_video_formats;
	gHdmiVideoParms.colorSpace = HDMI_CS_RGB;
	gHdmiVideoParms.colorDepth = HDMI_CD_24;
	gHdmiVideoParms.colorimetry = HDMI_COLORIMETRY_NO_DATA;
	gHdmiVideoParms.pixelAspectRatio = HDMI_PIXEL_RATIO_4_3;

	hdmi_set_hdmimode(gHdmiVideoParms.mode);
	hdmi_set_color_space(gHdmiVideoParms.colorSpace);
	hdmi_set_color_depth(gHdmiVideoParms.colorDepth);
	hdmi_set_pixel_aspect_ratio(gHdmiVideoParms.pixelAspectRatio);
#endif

	gHdmiPwrInfo.status = PWR_STATUS_ON;

	// disable HDCP INT
	regl = hdmi_readb(HDMI_SS_INTC_CON);
	hdmi_writeb(regl & ~(1<<HDMI_IRQ_HDCP), HDMI_SS_INTC_CON);

	// disable SPDIF INT
	regl = hdmi_readb(HDMI_SS_INTC_CON);
	hdmi_writeb(regl & ~(1<<HDMI_IRQ_SPDIF), HDMI_SS_INTC_CON);

	dprintk(KERN_INFO "%s End\n", __FUNCTION__);
}

void tcc_hdmi_power_off(void)
{
	dprintk(KERN_INFO "%s\n", __FUNCTION__);

	// HDMI PHY Reset
	tcc_ddi_hdmi_ctrl(HDMICTRL_RESET_HDMI, 0);
	udelay(1);
	tcc_ddi_hdmi_ctrl(HDMICTRL_RESET_HDMI, 1);

	// HDMI SPDIF Reset
	tcc_ddi_hdmi_ctrl(HDMICTRL_RESET_SPDIF, 0);
	udelay(1);
	tcc_ddi_hdmi_ctrl(HDMICTRL_RESET_SPDIF, 1);

	// HDMI TMDS Reset
	tcc_ddi_hdmi_ctrl(HDMICTRL_RESET_TMDS, 0);
	udelay(1);
	tcc_ddi_hdmi_ctrl(HDMICTRL_RESET_TMDS, 1);

	// swreset DDI_BUS HDMI
	tcc_ddi_swreset_hdmi(1);
	udelay(1);
	tcc_ddi_swreset_hdmi(0);

	// disable DDI_BUS HDMI CLK
	tcc_ddi_hdmi_ctrl(HDMICTRL_HDMI_ENABLE, 0);

	// ddi hdmi power down
	tcc_ddi_pwdn_hdmi(1);

	#if defined(CONFIG_ARCH_TCC896X) || defined(CONFIG_ARCH_TCC897X) || defined(CONFIG_ARCH_TCC570X)
	// enable HDMI PHY Power-off
	if(hdmi_ipclk)
		clk_disable_unprepare(hdmi_ipclk); // power down
	#endif
	
	// gpio power on
	udelay(100);

	// enable HDMI Power-down
	if (hdmi_pclk && hdmi_hclk && (gHdmiPwrInfo.status == PWR_STATUS_ON)) {
		clk_disable_unprepare(hdmi_pclk);
		clk_disable_unprepare(hdmi_hclk);
	}
	
#if defined(CONFIG_REGULATOR)
	if(vdd_hdmi) {
		regulator_disable(vdd_hdmi);
	}
#endif
	
	gHdmiPwrInfo.status = PWR_STATUS_OFF;
	memset(&gHdmiVideoParms, 0, sizeof(struct HDMIVideoParameter));
}

void tcc_hdmi_v5_power_on(void)
{
#if defined(CONFIG_REGULATOR)
	int ret;
#endif

	dprintk(KERN_INFO "%s\n", __FUNCTION__);

#if defined(CONFIG_REGULATOR)
	if(vdd_v5p0) {
		ret = regulator_enable(vdd_v5p0);
		if (ret)
			pr_err("failed to enable v5p0 regulator: %d\n", ret);
	}
    if(hdmi_gpio_manual_control) {
        tcc_gpio_config(TCC_GPHDMI(2), GPIO_FN1|GPIO_OUTPUT|GPIO_LOW);
        tcc_gpio_config(TCC_GPHDMI(3), GPIO_FN1|GPIO_OUTPUT|GPIO_LOW);
    }
#endif

}

void tcc_hdmi_v5_power_off(void)
{

	dprintk(KERN_INFO "%s\n", __FUNCTION__);

#if defined(CONFIG_REGULATOR)
    if(hdmi_gpio_manual_control) {
        tcc_gpio_config(TCC_GPHDMI(2), GPIO_FN0|GPIO_OUTPUT|GPIO_LOW);
        tcc_gpio_config(TCC_GPHDMI(3), GPIO_FN0|GPIO_OUTPUT|GPIO_LOW);
    }
	if(vdd_v5p0) {
		regulator_disable(vdd_v5p0);
	}
#endif

}

static int hdmi_remove(struct platform_device *pdev)
{
	dprintk(KERN_INFO "%s\n", __FUNCTION__);

    misc_deregister(&hdmi_misc_device);
#ifdef CONFIG_PM_RUNTIME
	pm_runtime_disable(pdev_hdmi);
#endif

	return 0;
	
}

/**
 * Set checksum in SPD InfoFrame Packet. @n
 * Calculate a checksum and set it in packet.
 */
void hdmi_spd_update_checksum(void)
{
    unsigned char index, checksum;

    checksum = SPD_HEADER;

    for (index = 0; index < SPD_PACKET_BYTE_LENGTH; index++)
    {
        checksum += hdmi_readb(HDMI_SPD_DATA1 + 4*index);
    }

    hdmi_writeb(~checksum+1,HDMI_SPD_CHECK_SUM);
}

/**
 * Set checksum in Audio InfoFrame Packet. @n
 * Calculate a checksum and set it in packet.
 */
void hdmi_aui_update_checksum(void)
{
    unsigned char index, checksum;

	hdmi_writeb(AUI_HEADER_BYTE0, HDMI_AUI_HEADER0);
	hdmi_writeb(AUI_HEADER_BYTE1, HDMI_AUI_HEADER1);
	hdmi_writeb(AUI_HEADER_BYTE2, HDMI_AUI_HEADER2);

    checksum = AUI_HEADER;
    for (index = 0; index < AUI_PACKET_BYTE_LENGTH; index++)
    {
        checksum += hdmi_readb(HDMI_AUI_BYTE1 + 4*index);
    }
    hdmi_writeb(~checksum+1,HDMI_AUI_CHECK_SUM);
}

/**
 * Set checksum in AVI InfoFrame Packet. @n
 * Calculate a checksum and set it in packet.
 */
void hdmi_avi_update_checksum(void)
{
    unsigned char index, checksum;

    checksum = AVI_HEADER;

	hdmi_writeb(AVI_HEADER_BYTE0, HDMI_AVI_HEADER0);
	hdmi_writeb(AVI_HEADER_BYTE1, HDMI_AVI_HEADER1);
	hdmi_writeb(AVI_HEADER_BYTE2, HDMI_AVI_HEADER2);

    for (index = 0; index < AVI_PACKET_BYTE_LENGTH; index++)
    {
        checksum += hdmi_readb(HDMI_AVI_BYTE1 + 4*index);
    }
    hdmi_writeb(~checksum+1,HDMI_AVI_CHECK_SUM);
}

/**
 * Set color space in HDMI H/W. @n
 * @param   space   [in] Color space
 * @return  If argument is invalid, return 0;Otherwise return 1.
 */
int hdmi_set_color_space(enum ColorSpace space)
{
	VIOC_DISP *pDispBase;
    unsigned char reg,aviYY;
    int ret = 1;
	unsigned int iPXWD=12, iR2YMD=0, iR2Y=0, iSWAP=0;

	if(hdmi_lcdc_num)
		pDispBase = (VIOC_DISP*)tcc_p2v(HwVIOC_DISP1);
	else
		pDispBase = (VIOC_DISP*)tcc_p2v(HwVIOC_DISP0);

    reg = hdmi_readb(HDMI_CON_0);
    aviYY = hdmi_readb(HDMI_AVI_BYTE1);
    // clear fields
    hdmi_writeb(aviYY & ~(AVI_CS_Y422|AVI_CS_Y444),HDMI_AVI_BYTE1);

    if (space == HDMI_CS_YCBCR422)
    {
        // set video input interface
        hdmi_writeb( reg | HDMI_YCBCR422_ENABLE, HDMI_CON_0);
        // set avi
        hdmi_writeb( aviYY | AVI_CS_Y422, HDMI_AVI_BYTE1);

		iPXWD = 8;
		iR2YMD = 3;
		iR2Y = 1;
		iSWAP =0;
    }
    else
    {
        // set video input interface
        hdmi_writeb( reg & ~HDMI_YCBCR422_ENABLE, HDMI_CON_0);
        if (space == HDMI_CS_YCBCR444)
        {
            // set AVI packet
            hdmi_writeb( aviYY | AVI_CS_Y444, HDMI_AVI_BYTE1);

			iPXWD = 12;
			iR2YMD = 3;
			iR2Y = 1;
			iSWAP = 4;
			
        }
        // aviYY for RGB = 0, nothing to set
        else if (space == HDMI_CS_RGB)
        {
			iPXWD = 12;
			iR2YMD = 0;
			iR2Y = 0;
			iSWAP = 0;
        }
    }

	VIOC_DISP_SetPXDW(pDispBase,  iPXWD);
	VIOC_DISP_SetR2YMD(pDispBase, iR2YMD);
	VIOC_DISP_SetR2Y(pDispBase, iR2Y);
	VIOC_DISP_SetSWAP(pDispBase, iSWAP);

    return ret;
}


#if defined(CONFIG_TCC_OUTPUT_COLOR_SPACE_YUV)
int hdmi_set_yuv420_color_space(void)
{
	unsigned char reg,aviYY;

    reg = hdmi_readb(HDMI_CON_0);
    aviYY = hdmi_readb(HDMI_AVI_BYTE1);
    // clear fields
    hdmi_writeb(aviYY & ~(AVI_CS_Y422|AVI_CS_Y444),HDMI_AVI_BYTE1);

    // set video input interface
    hdmi_writeb( reg | HDMI_YCBCR422_ENABLE, HDMI_CON_0);
    // set avi
    hdmi_writeb( aviYY | AVI_CS_Y422, HDMI_AVI_BYTE1);

	return 1;
}
#endif

/**
 * Set color depth.@n
 * @param   depth   [in] Color depth of input vieo stream
 * @return  If argument is invalid, return 0;Otherwise return 1.
 */
int hdmi_set_color_depth(enum ColorDepth depth)
{
	int ret = 1;

	switch (depth)
	{
		case HDMI_CD_36:
			// set GCP CD
			hdmi_writeb(GCP_CD_36BPP,HDMI_GCP_BYTE2);
			// set DC_CTRL
			hdmi_writeb(HDMI_DC_CTL_12,HDMI_DC_CONTROL);
			break;
		case HDMI_CD_30:
			// set GCP CD
			hdmi_writeb(GCP_CD_30BPP,HDMI_GCP_BYTE2);
			// set DC_CTRL
			hdmi_writeb(HDMI_DC_CTL_10,HDMI_DC_CONTROL);
			break;
		case HDMI_CD_24:
			// set GCP CD
			hdmi_writeb(GCP_CD_24BPP,HDMI_GCP_BYTE2);
			// set DC_CTRL
			hdmi_writeb(HDMI_DC_CTL_8,HDMI_DC_CONTROL);
			break;
		default:
			ret = 0;
			break;
	}
	return ret;
}

#if defined(CONFIG_ARCH_TCC896X) || defined(CONFIG_ARCH_TCC897X) || defined(CONFIG_ARCH_TCC570X)
/**
 * Set Phy Freq.@n
 * @param   freq   [in] freq of phy
 * @return  If argument is invalid, return 0;Otherwise return 1.
 */
int hdmi_set_phy_freq(enum PHYFreq freq)
{
    int index = 0;
    int size = 0;
    unsigned char *buffer;
	unsigned char reg = 0;
	unsigned int i = 0;

	index = gHDMIFreqIndex;
    size = sizeof(hdmi_phy_config[freq][index]) /
      sizeof(hdmi_phy_config[freq][index][0]);
    buffer = (unsigned char *) hdmi_phy_config[freq][index];

	#if (HDMI_DEBUG)
	{
		printk("%s : freq = %d, phy_index = %d, size = %d\n",__func__,(int)freq, index, size);

		for(i=0; i<size; i++)
		{
			printk("0x%02x ", buffer[i]);
		}
		
		printk("\n");
	}
	#endif

    // Clear Top:MODE_SET_DONE.
	// If you set the PHY setting, you must clear Mode_Set_Done.
    reg = 0x00;
	hdmi_writeb(reg, HDMIDP_PHYREG(0x7C));

	for(i=0;i<(size-1); i++)
	{
		hdmi_writeb(buffer[i], HDMIDP_PHYREG((i+1)*4));
	}

	hdmi_writeb(0x90, HDMIDP_PHYREG(0x8C));

	// Set Top:MODE_SET_DONE.
	//If you apply the PHY Setting, you must set MODE_SET_DONE>
	reg = 0x80;
	hdmi_writeb(reg, HDMIDP_PHYREG(0x7C));

	msleep(16);

	#if (HDMI_DEBUG)
	{
		for(i=0; i<(size-1); i++)
		{
			buffer[i] = hdmi_readb( HDMIDP_PHYREG((i+1)*4));
			printk("0x%02x ", buffer[i]);
		}
		
		printk("\n");
	}
	#endif

	return 1;
}

static int hdmi_set_phy_pwdn(unsigned char enable)
{
	unsigned char reg = 0;

	if(enable) {
	    // Clear Top:MODE_SET_DONE.
		// If you set the PHY setting, you must clear Mode_Set_Done.
	    reg = 0x00;
		hdmi_writeb(reg, HDMIDP_PHYREG(0x7C));

		//TX Power down(Reg74) Bit<6:4>,Bit<2:0> : refer LN28LPP_HDMI_v1p4_TX_PHY_DataSheet_REV1.2.pdf 
		// Bit<6> : PLL PD(PLL & Bias Block Power Down)
		// Bit<5> : TX_CLKSER_PD (Clock Serializer Power Down)
		// Bit<4> : TX_CLKDRV_PD (TMDS Clock Driver Power Down)
		// Bit<2> : TX_DRV_PD (TMDS Data Driver Power Down)
		// Bit<1> : TX_SER_PD (TMDS Data Serializer Power Down)
		// Bit<0> : TX_CLK_PD (TMDS Internal Clock Buffer / Divider Power Down)

		reg = 0x36;
		hdmi_writeb(reg, HDMIDP_PHYREG(0x74));	

		// Set Top:MODE_SET_DONE.
		//If you apply the PHY Setting, you must set MODE_SET_DONE>
		reg = 0x80;
		hdmi_writeb(reg, HDMIDP_PHYREG(0x7C));

		#if (HDMI_DEBUG)
		{
			reg = hdmi_readb( HDMIDP_PHYREG(0x74));
			printk("Reg74  = 0x%02x \n", reg);
		}
		#endif
	}

	return 1;
}

#endif

/**
 * Set video timing parameters.@n
 * @param   pVideo   [in] pointer to Video timing parameters
 * @return  If argument is invalid, return 0; Otherwise return 1.
 */
int hdmi_set_video_mode(struct HDMIVideoParameter* pVideo)
{
	dprintk("%s : pVideo->hdmi_3d_format = %d, pVideo->resolution = %d\n", __func__, pVideo->hdmi_3d_format, pVideo->resolution);

	// set default values first
	hdmi_set_default_value();

	// set video registers
	switch (pVideo->hdmi_3d_format)
	{
		case HDMI_2D_VIDEO_FORMAT:
		{
			if (!hdmi_set_2D_video(pVideo->resolution))
				return 0;
			break;
		}
		case HDMI_3D_FP_FORMAT:
		{
			if (!hdmi_set_3D_FP_video(pVideo->resolution))
				return 0;
			break;
		}
		case HDMI_3D_SSH_FORMAT:
		{
			if (!hdmi_set_3D_SSH_video(pVideo->resolution))
				return 0;
			break;
		}
		case HDMI_3D_TB_FORMAT:
		{
			if (!hdmi_set_3D_TB_video(pVideo->resolution))
				return 0;
			break;
		}
		case HDMI_3D_FA_FORMAT:
		{
			if (!hdmi_set_3D_FA_video(pVideo->resolution))
				return 0;
			break;
		}
		case HDMI_3D_LA_FORMAT:
		{
			if (!hdmi_set_3D_LA_video(pVideo->resolution))
				return 0;
			break;
		}
		case HDMI_3D_SSF_FORMAT:
		{
			if (!hdmi_set_3D_SSF_video(pVideo->resolution))
				return 0;
			break;
		}
		case HDMI_3D_LD_FORMAT:
		{
			if (!hdmi_set_3D_LD_video(pVideo->resolution))
				return 0;
			break;
		}
		case HDMI_3D_LDGFX_FORMAT:
		{
			if (!hdmi_set_3D_LDGFX_video(pVideo->resolution))
				return 0;
			break;
		}
		default:
			return 0;
	}
        // Set Video Auxilary packet
        if (!hdmi_set_aux_data(pVideo))
                return 0;

	return 1;
}

/**
 * Set pixel limitation.
 * @param   limit   [in] Pixel limitation.
* @return  If argument is invalid, return 0;Otherwise return 1.
 */
int hdmi_set_pixel_limit(enum PixelLimit limit)
{
	int ret = 1;
	unsigned char reg,aviQQ;

	// clear field
	reg = hdmi_readb(HDMI_CON_1);
	reg &= ~HDMICON1_LIMIT_MASK;

	aviQQ = hdmi_readb(HDMI_AVI_BYTE3);
	aviQQ &= ~AVI_QUANTIZATION_MASK;

	switch (limit) // full
	{
		case HDMI_FULL_RANGE:
			aviQQ |= AVI_QUANTIZATION_FULL;
			break;
		case HDMI_RGB_LIMIT_RANGE:
			reg |= HDMICON1_RGB_LIMIT;
			aviQQ |= AVI_QUANTIZATION_LIMITED;
			break;
		case HDMI_YCBCR_LIMIT_RANGE:
			reg |= HDMICON1_YCBCR_LIMIT;
			aviQQ |= AVI_QUANTIZATION_LIMITED;
			break;
		default:
			ret = 0;
			break;
	}

	// set pixel repetition
	hdmi_writeb(reg,HDMI_CON_1);
	// set avi packet body
	hdmi_writeb(aviQQ,HDMI_AVI_BYTE3);

	return ret;
}

/**
 * Set pixel aspect ratio information in AVI InfoFrame
 * @param   ratio   [in] Pixel Aspect Ratio
 * @return  If argument is invalid, return 0;Otherwise return 1.
 */
int hdmi_set_pixel_aspect_ratio(enum PixelAspectRatio ratio)
{
	int ret = 1;
	unsigned char reg = AVI_FORMAT_ASPECT_AS_PICTURE;

	switch (ratio)
	{
		case HDMI_PIXEL_RATIO_AS_PICTURE:
			break;
		case HDMI_PIXEL_RATIO_16_9:
			reg |= AVI_PICTURE_ASPECT_RATIO_16_9;
			break;
		case HDMI_PIXEL_RATIO_4_3:
			reg |= AVI_PICTURE_ASPECT_RATIO_4_3;
			break;
		default:
			ret = 0;
			break;
	}

	hdmi_writeb(reg,HDMI_AVI_BYTE2);
	return ret;
}

/**
 * Set colorimetry information in AVI InfoFrame
 * @param   colorimetry   [in] colorimetry
 * @return  If argument is invalid, return 0; Otherwise return 1.
 */
int hdmi_set_colorimetry(enum HDMIColorimetry colorimetry)
{
	int ret = 1;
	unsigned char avi2,avi3;
	avi2 = hdmi_readb(HDMI_AVI_BYTE2);
	avi3 = hdmi_readb(HDMI_AVI_BYTE3);

	avi2 &= ~AVI_COLORIMETRY_MASK;
	avi3 &= ~AVI_COLORIMETRY_EXT_MASK;

	switch (colorimetry)
	{
		case HDMI_COLORIMETRY_NO_DATA:
			break;

		case HDMI_COLORIMETRY_ITU601:
			avi2 |= AVI_COLORIMETRY_ITU601;
			break;

		case HDMI_COLORIMETRY_ITU709:
			avi2 |= AVI_COLORIMETRY_ITU709;
			break;

		case HDMI_COLORIMETRY_EXTENDED_xvYCC601:
			avi2 |= AVI_COLORIMETRY_EXTENDED;
			avi3 |= AVI_COLORIMETRY_EXT_xvYCC601;
			break;

		case HDMI_COLORIMETRY_EXTENDED_xvYCC709:
			avi2 |= AVI_COLORIMETRY_EXTENDED;
			avi3 |= AVI_COLORIMETRY_EXT_xvYCC709;
			break;

		default:
			ret = 0;
			break;
	}

	hdmi_writeb(avi2,HDMI_AVI_BYTE2);
	hdmi_writeb(avi3,HDMI_AVI_BYTE3);
	return ret;
}


/**
 * Set HDMI/DVI mode
 * @param   mode   [in] HDMI/DVI mode
 * @return  If argument is invalid, return 0;Otherwise return 1.
 */
int hdmi_set_hdmimode(int mode)
{
	int ret = 1;

	switch(mode)
	{
		case HDMI:
	        hdmi_writeb(HDMI_MODE_SEL_HDMI,HDMI_MODE_SEL);
	        hdmi_writeb(HDMICON2_HDMI,HDMI_CON_2);
			break;
		case DVI:
	        hdmi_writeb(HDMI_MODE_SEL_DVI,HDMI_MODE_SEL);
	        hdmi_writeb(HDMICON2_DVI,HDMI_CON_2);
			break;
		default:
			ret = 0;
			break;
	}

	return ret;
}

/**
 * Set Audio Clock Recovery and Audio Infoframe packet -@n
 * based on sampling frequency.
 * @param   freq   [in] Sampling frequency
 * @return  If argument is invalid, return 0;Otherwise return 1.
 */
int hdmi_set_audio_sample_freq(enum SamplingFreq freq)
{
	unsigned char reg;
	unsigned int n;
	int ret = 1;

	// check param
	if ( freq > sizeof(ACR_N_params)/sizeof(unsigned int) || freq < 0 )
		return 0;

	// set ACR packet
	// set N value
	n = ACR_N_params[freq];
	reg = n & 0xff;
	hdmi_writeb(0,HDMI_ACR_N0);
	hdmi_writeb(reg,HDMI_ACR_N0);
	reg = (n>>8) & 0xff;
	hdmi_writeb(reg,HDMI_ACR_N1);
	reg = (n>>16) & 0xff;
	hdmi_writeb(reg,HDMI_ACR_N2);

#if 0
	// set as measure cts mode
	hdmi_writeb(ACR_MEASURED_CTS_MODE,HDMI_ACR_CON);
#endif
	// set AUI packet
	reg = hdmi_readb(HDMI_AUI_BYTE2) & ~HDMI_AUI_SF_MASK;

	switch (freq)
	{
		case SF_32KHZ:
			reg |= HDMI_AUI_SF_SF_32KHZ;
			break;

		case SF_44KHZ:
			reg |= HDMI_AUI_SF_SF_44KHZ;
			break;

		case SF_88KHZ:
			reg |= HDMI_AUI_SF_SF_88KHZ;
			break;

		case SF_176KHZ:
			reg |= HDMI_AUI_SF_SF_176KHZ;
			break;

		case SF_48KHZ:
			reg |= HDMI_AUI_SF_SF_48KHZ;
			break;

		case SF_96KHZ:
			reg |= HDMI_AUI_SF_SF_96KHZ;
			break;

		case SF_192KHZ:
			reg |= HDMI_AUI_SF_SF_192KHZ;
			break;

		default:
			ret = 0;
			break;
	}

	hdmi_writeb(reg, HDMI_AUI_BYTE2);

	return ret;
}

/**
 * Set HDMI audio output packet type.
 * @param   packet   [in] Audio packet type
 * @return  If argument is invalid, return 0;Otherwise return 1.
 */
int hdmi_set_audio_packet_type(enum HDMIASPType packet)
{
    int ret = 1;
    unsigned char reg;

    reg = hdmi_readb(HDMI_ASP_CON);
    reg &= ~ASP_TYPE_MASK;

    switch (packet)
    {
        case HDMI_ASP:
        {
            reg |= ASP_LPCM_TYPE;
            break;
        }
        case HDMI_DSD:
        {
            reg |= ASP_DSD_TYPE;
            break;
        }
        case HDMI_HBR:
        {
            unsigned char regb = hdmi_readb(HDMI_SS_I2S_CH_ST_3) & ~I2S_CH_ST_3_SF_MASK;
            regb |= I2S_CH_ST_3_SF_768KHZ;
            hdmi_writeb(regb, HDMI_SS_I2S_CH_ST_3);
            reg |= ASP_HBR_TYPE;
            break;
        }
        case HDMI_DST:
        {
            reg |= ASP_DST_TYPE;
            break;
        }
        default:
        {
            ret = 0;
            break;
        }
    }

    hdmi_writeb(reg,HDMI_ASP_CON);
    return ret;
}

/**
 * Set layout and sample present fields in Audio Sample Packet -@n
 * and channel number field in Audio InfoFrame packet.
 * @param   channel   [in]  Number of channels
 * @return  If argument is invalid, return 0;Otherwise return 1.
 */
int hdmi_set_audio_number_of_channels(enum ChannelNum channel)
{
	int ret = 1;
	unsigned char reg, reg_byte4;

	reg = hdmi_readb(HDMI_ASP_CON);
	// clear field
	reg &= ~(ASP_MODE_MASK|ASP_SP_MASK);

	// clear field
	hdmi_writeb(0x00,HDMI_ASP_SP_FLAT);

	// celar field
	reg_byte4 = 0;

	// set layout & SP_PRESENT on ASP_CON
	// set AUI Packet
	switch (channel)
	{
		case CH_2:
			reg |= (ASP_LAYOUT_0|ASP_SP_0);
			hdmi_writeb(AUI_CC_2CH,HDMI_AUI_BYTE1);
			break;
		case CH_3:
			reg |= (ASP_LAYOUT_1|ASP_SP_0|ASP_SP_1);
			hdmi_writeb(AUI_CC_3CH,HDMI_AUI_BYTE1);
			break;
		case CH_4:
			reg |= (ASP_LAYOUT_1|ASP_SP_0|ASP_SP_1);
			hdmi_writeb(AUI_CC_4CH,HDMI_AUI_BYTE1);
			break;
		case CH_5:
			reg_byte4 = 0x0A;
			reg |= (ASP_LAYOUT_1|ASP_SP_0|ASP_SP_1|ASP_SP_2);
			hdmi_writeb(AUI_CC_5CH,HDMI_AUI_BYTE1);
			break;
		case CH_6:
			reg_byte4 = 0x0A;
			reg |= (ASP_LAYOUT_1|ASP_SP_0|ASP_SP_1|ASP_SP_2);
			hdmi_writeb(AUI_CC_6CH,HDMI_AUI_BYTE1);
			break;
		case CH_7:
			reg_byte4 = 0x12;
			reg |= (ASP_LAYOUT_1|ASP_SP_0|ASP_SP_1|ASP_SP_2|ASP_SP_3);
			hdmi_writeb(AUI_CC_7CH,HDMI_AUI_BYTE1);
			break;
		case CH_8:
			reg_byte4 = 0x12;
			reg |= (ASP_LAYOUT_1|ASP_SP_0|ASP_SP_1|ASP_SP_2|ASP_SP_3);
			hdmi_writeb(AUI_CC_8CH,HDMI_AUI_BYTE1);
			break;
		default:
			ret = 0;
	}

	hdmi_writeb(reg_byte4, HDMI_AUI_BYTE4);
	hdmi_writeb(reg,HDMI_ASP_CON);
	return ret;
}

int hdmi_set_spd_infoframe(struct HDMIVideoFormatCtrl VideoFormatCtrl)
{
	//SPD INFOFRAME PACKET HEADER
	hdmi_writeb(SPD_PACKET_TYPE,HDMI_SPD_HEADER0);
	hdmi_writeb(SPD_PACKET_VERSION,HDMI_SPD_HEADER1);
	hdmi_writeb(SPD_PACKET_BYTE_LENGTH,HDMI_SPD_HEADER2);

	//SPD INFOFRAME PACKET CONTENTS
	hdmi_writeb(SPD_PACKET_ID0,HDMI_SPD_DATA1);
	hdmi_writeb(SPD_PACKET_ID1,HDMI_SPD_DATA2);
	hdmi_writeb(SPD_PACKET_ID2,HDMI_SPD_DATA3);

	switch(VideoFormatCtrl.video_format)
	{
		case HDMI_2D:
			hdmi_writeb((SPD_HDMI_VIDEO_FORMAT_NONE << 5),HDMI_SPD_DATA4);
			break;
		case HDMI_VIC:
			hdmi_writeb((SPD_HDMI_VIDEO_FORMAT_VIC << 5),HDMI_SPD_DATA4);
			break;
		case HDMI_3D:
			hdmi_writeb((SPD_HDMI_VIDEO_FORMAT_3D << 5),HDMI_SPD_DATA4);
			break;
		default:
			break;
	}

	if(VideoFormatCtrl.video_format == HDMI_3D)
	{
		switch(VideoFormatCtrl.structure_3D)
		{
			case FRAME_PACKING:
				hdmi_writeb((SPD_3D_STRUCT_FRAME_PACKING << 4),HDMI_SPD_DATA5);
				break;
			case TOP_AND_BOTTOM:
				hdmi_writeb((SPD_3D_STRUCT_TOP_AND_BOTTOM << 4),HDMI_SPD_DATA5);
				break;
			case SIDE_BY_SIDE:
				hdmi_writeb((SPD_3D_STRUCT_SIDE_BY_SIDE << 4),HDMI_SPD_DATA5);
				break;
		}

		if(VideoFormatCtrl.ext_data_3D)
			hdmi_writeb(VideoFormatCtrl.ext_data_3D << 4,HDMI_SPD_DATA5);
	}
	else
	{
		hdmi_writeb(0,HDMI_SPD_DATA5);
		hdmi_writeb(0,HDMI_SPD_DATA6);
		hdmi_writeb(0,HDMI_SPD_DATA7);
	}
	
	hdmi_spd_update_checksum();

	return 1;
}

unsigned char hdmi_get_power_status(void)
{
	return gHdmiPwrInfo.status;
}


unsigned char hdmi_get_system_en(void)
{
	unsigned char hdmi_system_en = 0, reg;
	reg = hdmi_readb(HDMI_CON_0);
	hdmi_system_en = reg & 0x01;

	if(hdmi_system_en) 	{
		//dprintk(KERN_INFO "%s hdmi system is enabled : %d\n", __FUNCTION__);
	}
	else
	{
		printk(KERN_INFO "%s hdmi system is not enabled : %d\n", __FUNCTION__, hdmi_system_en);
	}

	return hdmi_system_en;
}

unsigned char hdmi_get_phy_status(void)
{
	unsigned char phy_status = 0;
	unsigned int phy_status_cmu;
	unsigned int phy_status_pll;

	phy_status = hdmi_readb(HDMI_SS_PHY_STATUS_0);
	phy_status_cmu = hdmi_readb(HDMI_SS_PHY_STATUS_CMU);
	phy_status_pll = hdmi_readb(HDMI_SS_PHY_STATUS_PLL);

	if(phy_status) 	{
		//dprintk(KERN_INFO "%s phy is ready \n", __FUNCTION__);
	}
	else
	{
		printk(KERN_INFO "%s phy is not ready \n", __FUNCTION__);
		printk(KERN_INFO "%s Phy status = %d, Phy stats CMU = %d, Phy status PLL = %d\n", __func__, phy_status, phy_status_cmu, phy_status_pll);
	}

	return phy_status;
}

unsigned char hdmi_get_hdmimode(void)
{
	return gHdmiVideoParms.mode;
}

/**
 * hdmi_phy_reset
 */
void hdmi_phy_reset(void)
{
	unsigned int  regl;

	// HDMI PHY Reset

	regl = hdmi_readl(DDICFG_HDMICTRL);

	regl |= HDMICTRL_HDMI_ENABLE;
	regl &= ~HDMICTRL_RESET_SPDIF;
	regl &= ~HDMICTRL_RESET_TMDS;
	regl |= HDMICTRL_RESET_HDMI;

	hdmi_writel(regl, DDICFG_HDMICTRL);

	tcc_ddi_swreset_hdmi(1);
	udelay(1);
	tcc_ddi_swreset_hdmi(0);

	udelay(100);        // min 25us wait is needed for resetting phy.		

	regl = hdmi_readl(DDICFG_HDMICTRL);

	regl |= HDMICTRL_HDMI_ENABLE;
	regl &= ~HDMICTRL_RESET_SPDIF;
	regl &= ~HDMICTRL_RESET_TMDS;
	regl &= ~HDMICTRL_RESET_HDMI;

	hdmi_writel(regl, DDICFG_HDMICTRL);

	#if defined(CONFIG_HDMI_CLK_USE_XIN_24MHZ)
	regl = hdmi_readl(DDICFG_HDMICTRL);
	regl |= (1 << 9);
	regl |= (1 << 8);
	#if defined(CONFIG_ARCH_TCC896X)
	//-------------------------------------------------------------------------------------------
	// This option must be enable at TCC8960
	// Internal Clock Setting
	regl |= (1 << 11);
	// In TCC8960 HDMI Link, Encoding option is must disabled.
	regl |= (0 << 14);
	//-------------------------------------------------------------------------------------------
	#elif defined(CONFIG_ARCH_TCC897X) || defined(CONFIG_ARCH_TCC570X)
	//-------------------------------------------------------------------------------------------
	// This option must be enable at TCC897x
	// Internal Clock Setting
	regl |= (1 << 11);
	// In TCC897x HDMI Link, Encoding option is must disabled.
	regl |= (0 << 14);
	//-------------------------------------------------------------------------------------------
	#endif
	hdmi_writel(regl, DDICFG_HDMICTRL);

	udelay(1000);
	#endif /* CONFIG_HDMI_CLK_USE_XIN_24MHZ */


}

/**
 * Enable HDMI output.
 */
void hdmi_start(void)
{
	unsigned char reg, mode;

	dprintk(KERN_INFO "%s \n", __func__);

	// check HDMI mode
	mode = hdmi_readb(HDMI_MODE_SEL) & HDMI_MODE_SEL_HDMI;
	reg = hdmi_readb(HDMI_CON_0);

	// enable external vido gen.
	hdmi_writeb(HDMI_EXTERNAL_VIDEO,HDMI_VIDEO_PATTERN_GEN);

	if (mode) // HDMI
	{
		// enable AVI packet: mandatory
		// update avi packet checksum
		hdmi_avi_update_checksum();
		// enable avi packet
		hdmi_writeb(TRANSMIT_EVERY_VSYNC,HDMI_AVI_CON);

		// check if audio is enable
		if (hdmi_readb(HDMI_ACR_CON))
		{
			// enable aui packet
			hdmi_aui_update_checksum();
			hdmi_writeb(TRANSMIT_EVERY_VSYNC,HDMI_AUI_CON);
			reg |= HDMI_ASP_ENABLE;
		}

		// check if it is deep color mode or not
		if (hdmi_readb(HDMI_DC_CONTROL))
		{
			hdmi_writeb(GCP_TRANSMIT_EVERY_VSYNC,HDMI_GCP_CON);
		}
		else
		{
			// disable GCP
			hdmi_writeb(DO_NOT_TRANSMIT,HDMI_GCP_CON);
		}

        	// for checking
		udelay(200);

		// enable hdmi
	#if defined(CONFIG_ARCH_TCC893X)
		hdmi_writeb(reg|HDMI_SYS_ENABLE,HDMI_CON_0);
	#elif defined(CONFIG_ARCH_TCC896X)
		//-------------------------------------------------------------------------------------------
		// In TCC8960 HDMI Link, Encoding option is must disabled.
		//-------------------------------------------------------------------------------------------
       	hdmi_writeb(reg|HDMI_SYS_ENABLE/*|HDMI_ENCODING_OPTION_ENABLE*/, HDMI_CON_0);
		//-------------------------------------------------------------------------------------------
	#elif defined(CONFIG_ARCH_TCC897X) || defined(CONFIG_ARCH_TCC570X)
		//-------------------------------------------------------------------------------------------
		// In TCC897x HDMI Link, Encoding option is must disabled.
		//-------------------------------------------------------------------------------------------
       	hdmi_writeb(reg|HDMI_SYS_ENABLE/*|HDMI_ENCODING_OPTION_ENABLE*/, HDMI_CON_0);
		//-------------------------------------------------------------------------------------------
	#endif
	}
	else // DVI
	{
		// disable all packet
		hdmi_writeb(DO_NOT_TRANSMIT,HDMI_AVI_CON);
		hdmi_writeb(DO_NOT_TRANSMIT,HDMI_AUI_CON);
		hdmi_writeb(DO_NOT_TRANSMIT,HDMI_GCP_CON);
		hdmi_writeb(DO_NOT_TRANSMIT,HDMI_GAMUT_CON);
		hdmi_writeb(DO_NOT_TRANSMIT,HDMI_ACP_CON);
		hdmi_writeb(DO_NOT_TRANSMIT,HDMI_ISRC_CON);
		hdmi_writeb(DO_NOT_TRANSMIT,HDMI_MPG_CON);
		hdmi_writeb(DO_NOT_TRANSMIT,HDMI_SPD_CON);
		hdmi_writeb(DO_NOT_TRANSMIT,HDMI_ACR_CON);

		// enable hdmi without audio
		reg &= ~HDMI_ASP_ENABLE;
	#if defined(CONFIG_ARCH_TCC893X)
		hdmi_writeb(reg|HDMI_SYS_ENABLE,HDMI_CON_0);
	#elif defined(CONFIG_ARCH_TCC896X)
		//-------------------------------------------------------------------------------------------
		// In TCC8960 HDMI Link, Encoding option is must disabled.
		//-------------------------------------------------------------------------------------------
       	hdmi_writeb(reg|HDMI_SYS_ENABLE/*|HDMI_ENCODING_OPTION_ENABLE*/, HDMI_CON_0);
		//-------------------------------------------------------------------------------------------
	#elif defined(CONFIG_ARCH_TCC897X) || defined(CONFIG_ARCH_TCC570X)
		//-------------------------------------------------------------------------------------------
		// In TCC897x HDMI Link, Encoding option is must disabled.
		//-------------------------------------------------------------------------------------------
       	hdmi_writeb(reg|HDMI_SYS_ENABLE/*|HDMI_ENCODING_OPTION_ENABLE*/, HDMI_CON_0);
		//-------------------------------------------------------------------------------------------
	#endif
	}

	hdmi_enable_bluescreen(0);

	gHdmiStartFlag = 1;

    return;
}

void hdmi_stop(void)
{
    unsigned char reg;

	dprintk(KERN_INFO "%s \n", __func__);

    reg = hdmi_readb(HDMI_CON_0);
    hdmi_writeb(reg & ~HDMI_SYS_ENABLE,HDMI_CON_0);
#ifdef HDMI_TX13_REV_05
    video_wrapper_enable(0);
#endif

	#if defined(CONFIG_ARCH_TCC896X) || defined(CONFIG_ARCH_TCC897X) || defined(CONFIG_ARCH_TCC570X)
	if(	!gHDMISuspendStatus )
		hdmi_set_phy_pwdn(1);
	#endif

	gHdmiStartFlag = 0;
}

/**
 * Enable/disable Blue-Screen.
 *
 * @param  enable	[in] 0 to stop sending, 1 to start sending.
 */
void hdmi_enable_bluescreen(unsigned char enable)
{
	unsigned char reg = hdmi_readb(HDMI_CON_0);

	dprintk(KERN_INFO "%s enable = %d\n", __func__, enable);
	
	if (enable)
	{
		reg |= HDMI_BLUE_SCR_ENABLE;
	}
	else
	{
		reg &= ~HDMI_BLUE_SCR_ENABLE;
	}
	hdmi_writeb(reg,HDMI_CON_0);
}

/**
 * Set VIC field in AVI Packet.@n
 * Set HDMI VSI packet in case of 3D video mode or HDMI VIC mode.
 *
 * @param  pVideo	[in] Video parameter
 * @return  If argument is invalid, return 0; Otherwise return 1.
 */
int hdmi_set_aux_data(struct HDMIVideoParameter* pVideo)
{
	enum HDMI3DVideoStructure mode = pVideo->hdmi_3d_format;
	unsigned char reg;
        dprintk(KERN_ERR "%s()\n",__FUNCTION__);

    // common for all except HDMI_VIC_FORMAT
	// set AVI packet with VIC
	if (pVideo->pixelAspectRatio == HDMI_PIXEL_RATIO_16_9)
		hdmi_writeb(HDMIVideoParams[pVideo->resolution].AVI_VIC_16_9,HDMI_AVI_BYTE4);
	else
		hdmi_writeb(HDMIVideoParams[pVideo->resolution].AVI_VIC,HDMI_AVI_BYTE4);

	if (mode == HDMI_2D_VIDEO_FORMAT)
	{
		hdmi_writeb(DO_NOT_TRANSMIT,HDMI_VSI_CON);

		// set pixel repetition
		reg = hdmi_readb(HDMI_CON_1);
		if ( HDMIVideoParams[pVideo->resolution].repetition )
		{
			// set pixel repetition
			hdmi_writeb(reg|HDMICON1_DOUBLE_PIXEL_REPETITION,HDMI_CON_1);
			// set avi packet
			hdmi_writeb(AVI_PIXEL_REPETITION_DOUBLE,HDMI_AVI_BYTE5);
		}
		else
		{
			// clear pixel repetition
			hdmi_writeb(reg & ~(1<<1|1<<0),HDMI_CON_1);
			// set avi packet
			hdmi_writeb(0x00,HDMI_AVI_BYTE5);
		}


		return 1;
	}
	else
	{
		// common for all 3D mode
		hdmi_writeb(0x81,HDMI_VSI_HEADER0);
		hdmi_writeb(0x01,HDMI_VSI_HEADER1);
		hdmi_writeb(0x05,HDMI_VSI_HEADER2);

		hdmi_writeb(0x03,HDMI_VSI_DATA01);
		hdmi_writeb(0x0C,HDMI_VSI_DATA02);
		hdmi_writeb(0x00,HDMI_VSI_DATA03);


		switch (mode)
		{
			case HDMI_3D_FP_FORMAT:
			{
				hdmi_writeb(0x2a,HDMI_VSI_DATA00);
				hdmi_writeb(0x40,HDMI_VSI_DATA04);
				hdmi_writeb(0x00,HDMI_VSI_DATA05);
				break;
			}
			case HDMI_3D_SSH_FORMAT:
			{
				hdmi_writeb(0x06,HDMI_VSI_HEADER2);
				hdmi_writeb(0x99,HDMI_VSI_DATA00);
				hdmi_writeb(0x40,HDMI_VSI_DATA04);
				hdmi_writeb(0x80,HDMI_VSI_DATA05);
				hdmi_writeb(0x10,HDMI_VSI_DATA06);

				break;
			}
			case HDMI_3D_TB_FORMAT:
			{
				hdmi_writeb(0xCA,HDMI_VSI_DATA00);
				hdmi_writeb(0x40,HDMI_VSI_DATA04);
				hdmi_writeb(0x60,HDMI_VSI_DATA05);
				break;
			}
			case HDMI_3D_FA_FORMAT:
			{
				hdmi_writeb(0x1A,HDMI_VSI_DATA00);
				hdmi_writeb(0x40,HDMI_VSI_DATA04);
				hdmi_writeb(0x10,HDMI_VSI_DATA05);
				break;
			}
			case HDMI_3D_LA_FORMAT:
			{
				hdmi_writeb(0x0A,HDMI_VSI_DATA00);
				hdmi_writeb(0x40,HDMI_VSI_DATA04);
				hdmi_writeb(0x20,HDMI_VSI_DATA05);
				break;
			}
			case HDMI_3D_SSF_FORMAT:
			{
				hdmi_writeb(0x09,HDMI_VSI_DATA00);
				hdmi_writeb(0x40,HDMI_VSI_DATA04);
				hdmi_writeb(0x03,HDMI_VSI_DATA05);
				break;
			}
			case HDMI_3D_LD_FORMAT:
			{
				hdmi_writeb(0xEA,HDMI_VSI_DATA00);
				hdmi_writeb(0x40,HDMI_VSI_DATA04);
				hdmi_writeb(0x40,HDMI_VSI_DATA05);
				break;
			}
			case HDMI_3D_LDGFX_FORMAT:
			{
				hdmi_writeb(0xE9,HDMI_VSI_DATA00);
				hdmi_writeb(0x40,HDMI_VSI_DATA04);
				hdmi_writeb(0x05,HDMI_VSI_DATA05);
				break;
			}
			case HDMI_VIC_FORMAT:
			{
				unsigned char vic = HDMIVideoParams[pVideo->resolution].AVI_VIC;
				hdmi_writeb(0x20,HDMI_VSI_DATA04);
				hdmi_writeb(vic, HDMI_VSI_DATA05);
				hdmi_writeb(0x4A - vic, HDMI_VSI_DATA00);
				// clear VIC in AVI
				hdmi_writeb(0x00,HDMI_AVI_BYTE4);
			}
			default:
				return 0;
		} // switch

		hdmi_writeb(TRANSMIT_EVERY_VSYNC,HDMI_VSI_CON);

		// set pixel repetition
		reg = hdmi_readb(HDMI_CON_1);
		if ( HDMIVideoParams[pVideo->resolution].repetition )
		{
			// set pixel repetition
			hdmi_writeb(reg|HDMICON1_DOUBLE_PIXEL_REPETITION,HDMI_CON_1);
			// set avi packet
			hdmi_writeb(AVI_PIXEL_REPETITION_DOUBLE,HDMI_AVI_BYTE5);
		}
		else
		{
			// clear pixel repetition
			hdmi_writeb(reg & ~(1<<1|1<<0),HDMI_CON_1);
			// set avi packet
			hdmi_writeb(0x00,HDMI_AVI_BYTE5);
		}
	}
	return 1;
}

/**
 * Print out HDMI H/W registers for debugging.
 */
void hdmi_core_debug(void)
{
#if 0
	// for video wrapper
	{
		unsigned int* addr = VIDEO_WRAPPER_SYNC_MODE;
		unsigned int* final_addr  = VIDEO_WRAPPER_3D_SEL;
                unsigned int* base_addr = addr;
		printk("\n\n\nfor video wrapper\n\n\n");
		for ( ; addr <= final_addr; addr++ )
		{
			if (hdmi_readl(addr) != 0)
				printk("[offset = 0x%02X] = 0x%04X\n",(unsigned int)addr-(unsigned int)base_addr, hdmi_readl(addr));
		}
	}

	// for core
	{
		unsigned int* addr = HDMI_CON_0;
//		unsigned int* addr = HDMI_HSYNC_TEST_START;
		unsigned int* final_addr  = HDCP_FRAME_COUNT;
//                unsigned int* final_addr = HDMI_HSYNC_DGUA_END_MATCH;
                unsigned int* base_addr = addr;
		printk("\n\n\nfor hdmi core\n\n\n\n");
		for ( ; addr <= final_addr; addr++ )
		{
			if (hdmi_readb(addr) != 0)
				printk("[offset = 0x%02X] = 0x%02X\n",(unsigned int)(addr)-(unsigned int)(base_addr), hdmi_readb(addr));
		}
	}
#endif /* 0 */
}

/**
 * Print out HDMI video registers for debugging.
 */
void hdmi_core_video_debug(void)
{
#if 0
	// for video wrapper
	{
		unsigned int* addr = VIDEO_WRAPPER_SYNC_MODE;
		unsigned int* final_addr  = VIDEO_WRAPPER_3D_SEL;
                unsigned int* base_addr = addr;
		printk("\n\n\nfor video wrapper\n\n\n");
		for ( ; addr <= final_addr; addr++ )
		{
			if (hdmi_readl(addr) != 0)
				printk("[offset = 0x%02X] = 0x%04X\n",(unsigned int)addr-(unsigned int)base_addr, hdmi_readl(addr));
		}
	}

	// for core
	{
		unsigned int* addr = HDMI_H_BLANK_0;
		unsigned int* final_addr  = HDMI_VACT_SPACE6_1;
                unsigned int* base_addr = addr;
		printk("\n\n\nfor hdmi core\n\n\n\n");
		for ( ; addr <= final_addr; addr++ )
		{
				printk("[offset = 0x%02X] = 0x%02X\n",(unsigned int)addr-(unsigned int)base_addr, hdmi_readb(addr));
		}
	}
#endif /* 0 */
}

int hdmi_get_VBlank(void)
{
	unsigned int VBlankValue = 0;

	VBlankValue = hdmi_readb(HDMI_V1_BLANK_0) & 0xFF;
	VBlankValue |= ((hdmi_readb(HDMI_V1_BLANK_1) & 0xFF) << 8);

	return VBlankValue;
}

/**
 * Initialize HDMI video registers. @n
 */
void hdmi_set_default_value(void)
{
	// set default value(0xff) from HDMI_V_BLANK_F0_0 to HDMI_VACT_SPACE6_1
	unsigned int* base_addr = (unsigned int *)HDMI_V_BLANK_F0_0;
	unsigned int* final_addr = (unsigned int *)HDMI_VACT_SPACE6_1;

	// HBLANK
	hdmi_writeb(0x00, HDMI_H_BLANK_0);
	hdmi_writeb(0x00, HDMI_H_BLANK_1);

	// VBLANK1,2
	hdmi_writeb(0x00, HDMI_V1_BLANK_0);
	hdmi_writeb(0x00, HDMI_V1_BLANK_1);

	hdmi_writeb(0x00, HDMI_V2_BLANK_0);
	hdmi_writeb(0x00, HDMI_V2_BLANK_1);

	// H_LINE
	hdmi_writeb(0x00, HDMI_H_LINE_0);
	hdmi_writeb(0x00, HDMI_H_LINE_1);

	// V_LINE
	hdmi_writeb(0x00, HDMI_V_LINE_0);
	hdmi_writeb(0x00, HDMI_V_LINE_1);

	for ( ; base_addr <= final_addr; base_addr++ )
		hdmi_writeb(0xFF, HDMI_V_BLANK_F0_0);
}

/**
 * Set video registers as 2D video structure
 *
 * @param  format	[in] Video format
 * @return  1
 */
int hdmi_set_2D_video(enum VideoFormat format)
{
	// basic video parametres
	unsigned int temp;

	dprintk("%s(): format = %d\n",__FUNCTION__, format);

	// HBlank
	hdmi_writeb( HDMIVideoParams[format].HBlank & 0xFF, HDMI_H_BLANK_0 );
	hdmi_writeb( (HDMIVideoParams[format].HBlank>>8) & 0xFF, HDMI_H_BLANK_1 );

	// V1 Blank
	hdmi_writeb( HDMIVideoParams[format].VBlank & 0xFF, HDMI_V1_BLANK_0 );
	hdmi_writeb( (HDMIVideoParams[format].VBlank>>8) & 0xFF, HDMI_V1_BLANK_1 );

	// HTotal
	hdmi_writeb( HDMIVideoParams[format].HTotal & 0xFF, HDMI_H_LINE_0 );
	hdmi_writeb( (HDMIVideoParams[format].HTotal>>8) & 0xFF, HDMI_H_LINE_1 );

	// VTotal
	hdmi_writeb( HDMIVideoParams[format].VTotal & 0xFF, HDMI_V_LINE_0 );
	hdmi_writeb( (HDMIVideoParams[format].VTotal>>8) & 0xFF, HDMI_V_LINE_1 );

	// H POL
	hdmi_writeb( HDMIVideoParams[format].HPol, HDMI_HSYNC_POL );

	// V POL
	hdmi_writeb( HDMIVideoParams[format].VPol, HDMI_VSYNC_POL );

	// HSYNC Front
	hdmi_writeb( (HDMIVideoParams[format].HFront-2) & 0xFF, HDMI_H_SYNC_START_0 );
	hdmi_writeb( ((HDMIVideoParams[format].HFront-2)>>8) & 0xFF, HDMI_H_SYNC_START_1 );

	// HSYNC End
	hdmi_writeb( ((HDMIVideoParams[format].HFront-2) + HDMIVideoParams[format].HSync) & 0xFF
								, HDMI_H_SYNC_END_0 );
	hdmi_writeb( (((HDMIVideoParams[format].HFront-2) + HDMIVideoParams[format].HSync)>>8) & 0xFF
								, HDMI_H_SYNC_END_1 );


	// VSYNC Front
	hdmi_writeb( HDMIVideoParams[format].VFront & 0xFF, HDMI_V_SYNC_LINE_BEF_1_0 );
	hdmi_writeb( (HDMIVideoParams[format].VFront>>8) & 0xFF, HDMI_V_SYNC_LINE_BEF_1_1 );

	// VSYNC End
	hdmi_writeb( (HDMIVideoParams[format].VFront + HDMIVideoParams[format].VSync) & 0xFF
								, HDMI_V_SYNC_LINE_BEF_2_0 );
	hdmi_writeb( ((HDMIVideoParams[format].VFront + HDMIVideoParams[format].VSync)>>8) & 0xFF
								, HDMI_V_SYNC_LINE_BEF_2_1 );


	if ( HDMIVideoParams[format].interlaced )
	{
		// for interlace
		hdmi_writeb(0x1, HDMI_INT_PRO_MODE );

		if ( format == v1920x1080i_50Hz_1250 ) // V TOP and V BOT are same
		{
			// V2 BLANK
			temp = HDMIVideoParams[format].VTotal/2;
			// V TOP and V BOT are same
			hdmi_writeb( temp & 0xFF, HDMI_V2_BLANK_0 );
			hdmi_writeb( (temp>>8) & 0xFF, HDMI_V2_BLANK_1 );

			// VBLANK_F0
			hdmi_writeb( (temp + HDMIVideoParams[format].VBlank)&0xFF, HDMI_V_BLANK_F0_0 );
			hdmi_writeb( ((temp + HDMIVideoParams[format].VBlank)>>8)&0xFF, HDMI_V_BLANK_F0_1 );

			// VSYNC_LINE_AFT1
			temp = temp + HDMIVideoParams[format].VFront - 1;
			hdmi_writeb( temp & 0xFF , HDMI_V_SYNC_LINE_AFT_1_0);
			hdmi_writeb( (temp>>8 & 0xFF), HDMI_V_SYNC_LINE_AFT_1_1);

			// VSYNC_LINE_AFT2
			temp = temp + HDMIVideoParams[format].VSync;
			hdmi_writeb( temp & 0xFF, HDMI_V_SYNC_LINE_AFT_2_0);
			hdmi_writeb( (temp>>8) & 0xFF, HDMI_V_SYNC_LINE_AFT_2_1);
		}
		else // V TOP and V BOT are not same
		{
			// V2 BLANK
			temp = (HDMIVideoParams[format].VTotal - HDMIVideoParams[format].VBlank*2 - 1)/2 
					+ HDMIVideoParams[format].VBlank;
			hdmi_writeb(temp&0xFF, HDMI_V2_BLANK_0 );
			hdmi_writeb((temp>>8)&0xFF, HDMI_V2_BLANK_1 );

			// VBLANK_F0
			hdmi_writeb( ((HDMIVideoParams[format].VTotal + HDMIVideoParams[format].VBlank*2 + 1)/2) 
								& 0xFF, HDMI_V_BLANK_F0_0);
			hdmi_writeb( (((HDMIVideoParams[format].VTotal + HDMIVideoParams[format].VBlank*2 + 1)/2)>>8) 
								& 0xFF, HDMI_V_BLANK_F0_1);

			// VSYNC_LINE_AFT1
			temp = temp + HDMIVideoParams[format].VFront;
			hdmi_writeb( temp & 0xFF , HDMI_V_SYNC_LINE_AFT_1_0);
			hdmi_writeb( (temp>>8 & 0xFF), HDMI_V_SYNC_LINE_AFT_1_1);

			// VSYNC_LINE_AFT2
			temp = temp + HDMIVideoParams[format].VSync;
			hdmi_writeb( temp & 0xFF, HDMI_V_SYNC_LINE_AFT_2_0);
			hdmi_writeb( (temp>>8) & 0xFF, HDMI_V_SYNC_LINE_AFT_2_1);
		}



		// VBLANK_F1
		hdmi_writeb( HDMIVideoParams[format].VTotal & 0xFF, HDMI_V_BLANK_F1_0);
		hdmi_writeb( (HDMIVideoParams[format].VTotal>>8) & 0xFF, HDMI_V_BLANK_F1_1);

		temp = HDMIVideoParams[format].HTotal/2 + HDMIVideoParams[format].HFront;

		// VSYNC_LINE_AFT_PXL_1
		hdmi_writeb( temp & 0xFF, HDMI_V_SYNC_LINE_AFT_PXL_1_0 );
		hdmi_writeb( (temp>>8) & 0xFF, HDMI_V_SYNC_LINE_AFT_PXL_1_1 );
		// VSYNC_LINE_AFT_PXL_2
		hdmi_writeb( temp & 0xFF, HDMI_V_SYNC_LINE_AFT_PXL_2_0 );
		hdmi_writeb( (temp>>8) & 0xFF, HDMI_V_SYNC_LINE_AFT_PXL_2_1 );
	}
	else
	{
		// for progressive
		hdmi_writeb(0x0, HDMI_INT_PRO_MODE );

		// V2 BLANK, same as V total
		hdmi_writeb( HDMIVideoParams[format].VTotal & 0xFF, HDMI_V2_BLANK_0 );
		hdmi_writeb( (HDMIVideoParams[format].VTotal>>8) & 0xFF, HDMI_V2_BLANK_1 );
	}

	return 1;
}

/**
 * Set video registers as 3D Frame-Packing video structure
 *
 * @param  format	[in] Video format
 * @return  1
 */
int hdmi_set_3D_FP_video(enum VideoFormat format)
{
	// basic video parametres
	unsigned int temp;

	dprintk("%s()\n",__FUNCTION__);

	// HBlank
	hdmi_writeb( HDMIVideoParams[format].HBlank & 0xFF, HDMI_H_BLANK_0 );
	hdmi_writeb( (HDMIVideoParams[format].HBlank>>8) & 0xFF, HDMI_H_BLANK_1 );

	// V1 Blank
	hdmi_writeb( HDMIVideoParams[format].VBlank & 0xFF, HDMI_V1_BLANK_0 );
	hdmi_writeb( (HDMIVideoParams[format].VBlank>>8) & 0xFF, HDMI_V1_BLANK_1 );

	// HTotal
	hdmi_writeb( HDMIVideoParams[format].HTotal & 0xFF, HDMI_H_LINE_0 );
	hdmi_writeb( (HDMIVideoParams[format].HTotal>>8) & 0xFF, HDMI_H_LINE_1 );

	temp = HDMIVideoParams[format].VTotal*2;

	// VTotal
	hdmi_writeb( temp & 0xFF, HDMI_V_LINE_0 );
	hdmi_writeb( (temp>>8) & 0xFF, HDMI_V_LINE_1 );

	// V2 BLANK
	hdmi_writeb( temp & 0xFF, HDMI_V2_BLANK_0 );
	hdmi_writeb( (temp>>8) & 0xFF, HDMI_V2_BLANK_1 );

	// H POL
	hdmi_writeb( HDMIVideoParams[format].HPol, HDMI_HSYNC_POL );

	// V POL
	hdmi_writeb( HDMIVideoParams[format].VPol, HDMI_VSYNC_POL );

	// HSYNC Front
	hdmi_writeb( (HDMIVideoParams[format].HFront-2) & 0xFF, HDMI_H_SYNC_START_0 );
	hdmi_writeb( ((HDMIVideoParams[format].HFront-2)>>8) & 0xFF, HDMI_H_SYNC_START_1 );

	// HSYNC End
	hdmi_writeb( ((HDMIVideoParams[format].HFront-2) + HDMIVideoParams[format].HSync) & 0xFF
								, HDMI_H_SYNC_END_0 );
	hdmi_writeb( (((HDMIVideoParams[format].HFront-2) + HDMIVideoParams[format].HSync)>>8) & 0xFF
								, HDMI_H_SYNC_END_1 );

	// VSYNC Front
	hdmi_writeb( HDMIVideoParams[format].VFront & 0xFF, HDMI_V_SYNC_LINE_BEF_1_0 );
	hdmi_writeb( (HDMIVideoParams[format].VFront>>8) & 0xFF, HDMI_V_SYNC_LINE_BEF_1_1 );

	// VSYNC End
	hdmi_writeb( (HDMIVideoParams[format].VFront + HDMIVideoParams[format].VSync) & 0xFF
								, HDMI_V_SYNC_LINE_BEF_2_0 );
	hdmi_writeb( ((HDMIVideoParams[format].VFront + HDMIVideoParams[format].VSync)>>8) & 0xFF
								, HDMI_V_SYNC_LINE_BEF_2_1 );

	if ( HDMIVideoParams[format].interlaced )
	{
		// for interlace
		hdmi_writeb(0x1, HDMI_INT_PRO_MODE );

		if ( format == v1920x1080i_50Hz_1250 ) // V TOP and V BOT are same
		{
			temp = HDMIVideoParams[format].VTotal/2;
			// VACT_SPACE1
			hdmi_writeb( (temp) & 0xFF, HDMI_VACT_SPACE1_0);
			hdmi_writeb( (temp>>8)  & 0xFF, HDMI_VACT_SPACE1_1);

			// VACT_SPACE2
			temp += HDMIVideoParams[format].VBlank;
			hdmi_writeb( temp & 0xFF, HDMI_VACT_SPACE2_0);
			hdmi_writeb( (temp>>8) & 0xFF, HDMI_VACT_SPACE2_1);

			// VACT_SPACE5
			temp = (HDMIVideoParams[format].VTotal*3)/2;
			hdmi_writeb( (temp) & 0xFF, HDMI_VACT_SPACE5_0);
			hdmi_writeb( (temp>>8)  & 0xFF, HDMI_VACT_SPACE5_1);

			// VACT_SPACE6
			temp = (HDMIVideoParams[format].VTotal*3 + HDMIVideoParams[format].VBlank*2)/2;
			hdmi_writeb( (temp) & 0xFF, HDMI_VACT_SPACE6_0);
			hdmi_writeb( (temp>>8)  & 0xFF, HDMI_VACT_SPACE6_1);

		}
		else // V TOP and V BOT are not same
		{
			temp = (HDMIVideoParams[format].VTotal - HDMIVideoParams[format].VBlank*2 - 1)/2 
					+ HDMIVideoParams[format].VBlank;

			// VACT_SPACE1
			hdmi_writeb( (temp) & 0xFF, HDMI_VACT_SPACE1_0);
			hdmi_writeb( (temp>>8)  & 0xFF, HDMI_VACT_SPACE1_1);

			// VACT_SPACE2
			temp = (HDMIVideoParams[format].VTotal + HDMIVideoParams[format].VBlank*2 + 1)/2;
			hdmi_writeb( (temp) & 0xFF, HDMI_VACT_SPACE2_0);
			hdmi_writeb( (temp>>8)  & 0xFF, HDMI_VACT_SPACE2_1);

			// VACT_SPACE5
			temp = (HDMIVideoParams[format].VTotal*3 - 1)/2;
			hdmi_writeb( (temp) & 0xFF, HDMI_VACT_SPACE5_0);
			hdmi_writeb( (temp>>8)  & 0xFF, HDMI_VACT_SPACE5_1);

			// VACT_SPACE6
			temp = (HDMIVideoParams[format].VTotal*3 + HDMIVideoParams[format].VBlank*2 + 1)/2;
			hdmi_writeb( (temp) & 0xFF, HDMI_VACT_SPACE6_0);
			hdmi_writeb( (temp>>8)  & 0xFF, HDMI_VACT_SPACE6_1);
		}

		// VACT_SPACE3
		temp = HDMIVideoParams[format].VTotal;
		hdmi_writeb( (temp) & 0xFF, HDMI_VACT_SPACE3_0);
		hdmi_writeb( (temp>>8)  & 0xFF, HDMI_VACT_SPACE3_1);

		// VACT_SPACE4
		temp = HDMIVideoParams[format].VTotal + HDMIVideoParams[format].VBlank;
		hdmi_writeb( (temp) & 0xFF, HDMI_VACT_SPACE4_0);
		hdmi_writeb( (temp>>8)  & 0xFF, HDMI_VACT_SPACE4_1);
	}
	else
	{
		// for progressive
		hdmi_writeb(0x0, HDMI_INT_PRO_MODE );

		temp = HDMIVideoParams[format].VTotal;
		// VACT_SPACE1
		hdmi_writeb( temp & 0xFF, HDMI_VACT_SPACE1_0);
		hdmi_writeb( (temp>>8) & 0xFF, HDMI_VACT_SPACE1_1);

		// VACT_SPACE2
		temp += HDMIVideoParams[format].VBlank;
		hdmi_writeb( temp & 0xFF, HDMI_VACT_SPACE2_0);
		hdmi_writeb( (temp>>8) & 0xFF, HDMI_VACT_SPACE2_1);
	}

	return 1;
}

/**
 * Set video registers as 3D Side-by-Side video structure
 *
 * @param  format	[in] Video format
 * @return 1
 */
int hdmi_set_3D_SSH_video(enum VideoFormat format)
{
	return hdmi_set_2D_video(format);
}

/**
 * Set video registers as 3D Top-And-Bottom video structure
 *
 * @param  format	[in] Video format
 * @return  1
 */
int hdmi_set_3D_TB_video(enum VideoFormat format)
{
	return hdmi_set_2D_video(format);
}

/**
 * Set video registers as 3D Field Alternative video structure
 *
 * @param  format	[in] Video format
 * @return  If format is progressive, return 0; Otherwise return 1.
 */
int hdmi_set_3D_FA_video(enum VideoFormat format)
{
	// basic video parametres
	unsigned int temp;

	// only for interlaced
	if ( HDMIVideoParams[format].interlaced )
	{
		// for interlaced
		hdmi_writeb(0x1, HDMI_INT_PRO_MODE );

		// HBlank
		hdmi_writeb( HDMIVideoParams[format].HBlank & 0xFF, HDMI_H_BLANK_0 );
		hdmi_writeb( (HDMIVideoParams[format].HBlank>>8) & 0xFF, HDMI_H_BLANK_1 );

		// V1 Blank
		hdmi_writeb( HDMIVideoParams[format].VBlank & 0xFF, HDMI_V1_BLANK_0 );
		hdmi_writeb( (HDMIVideoParams[format].VBlank>>8) & 0xFF, HDMI_V1_BLANK_1 );

		// HTotal
		hdmi_writeb( HDMIVideoParams[format].HTotal & 0xFF, HDMI_H_LINE_0 );
		hdmi_writeb( (HDMIVideoParams[format].HTotal>>8) & 0xFF, HDMI_H_LINE_1 );

		temp = HDMIVideoParams[format].VTotal*2;

		// VTotal
		hdmi_writeb( HDMIVideoParams[format].VTotal & 0xFF, HDMI_V_LINE_0 );
		hdmi_writeb( (HDMIVideoParams[format].VTotal>>8) & 0xFF, HDMI_V_LINE_1 );

		// H POL
		hdmi_writeb( HDMIVideoParams[format].HPol, HDMI_HSYNC_POL );

		// V POL
		hdmi_writeb( HDMIVideoParams[format].VPol, HDMI_VSYNC_POL );

		// HSYNC Front
		hdmi_writeb( (HDMIVideoParams[format].HFront-2) & 0xFF, HDMI_H_SYNC_START_0 );
		hdmi_writeb( ((HDMIVideoParams[format].HFront-2)>>8) & 0xFF, HDMI_H_SYNC_START_1 );

		// HSYNC End
		hdmi_writeb( ((HDMIVideoParams[format].HFront-2) + HDMIVideoParams[format].HSync) & 0xFF
								, HDMI_H_SYNC_END_0 );
		hdmi_writeb( (((HDMIVideoParams[format].HFront-2) + HDMIVideoParams[format].HSync)>>8) & 0xFF
								, HDMI_H_SYNC_END_1 );

		// VSYNC Front
		hdmi_writeb( HDMIVideoParams[format].VFront & 0xFF, HDMI_V_SYNC_LINE_BEF_1_0 );
		hdmi_writeb( (HDMIVideoParams[format].VFront>>8) & 0xFF, HDMI_V_SYNC_LINE_BEF_1_1 );

		// VSYNC End
		hdmi_writeb( (HDMIVideoParams[format].VFront + HDMIVideoParams[format].VSync) & 0xFF
								, HDMI_V_SYNC_LINE_BEF_2_0 );
		hdmi_writeb( ((HDMIVideoParams[format].VFront + HDMIVideoParams[format].VSync)>>8) & 0xFF
								, HDMI_V_SYNC_LINE_BEF_2_1 );

		if ( format == v1920x1080i_50Hz_1250 ) // V TOP and V BOT are same
		{
			temp = HDMIVideoParams[format].VTotal/2;
			//V BLANK2
			hdmi_writeb( (temp) & 0xFF, HDMI_V2_BLANK_0);
			hdmi_writeb( (temp>>8)  & 0xFF, HDMI_V2_BLANK_1);

			// VBLANK_F0
			temp += HDMIVideoParams[format].VBlank;
			hdmi_writeb( temp & 0xFF, HDMI_V_BLANK_F0_0);
			hdmi_writeb( (temp>>8) & 0xFF, HDMI_V_BLANK_F0_1);

			// VBLANK_F3 == VACT_SPACE5
			temp = HDMIVideoParams[format].VTotal - HDMIVideoParams[format].VBlank/2;
			hdmi_writeb( (temp) & 0xFF, HDMI_V_BLANK_F3_0);
			hdmi_writeb( (temp>>8)  & 0xFF, HDMI_V_BLANK_F3_1);

			// VBLANK_F4 = VACT_SPACE6
			temp = (HDMIVideoParams[format].VTotal*3 + HDMIVideoParams[format].VBlank*2)/2;
			hdmi_writeb( (temp) & 0xFF, HDMI_V_BLANK_F4_0);
			hdmi_writeb( (temp>>8)  & 0xFF, HDMI_V_BLANK_F4_1);

			// VSYNC_LINE_AFT1
			temp += HDMIVideoParams[format].VFront-1;
			hdmi_writeb( (temp) & 0xFF, HDMI_V_SYNC_LINE_AFT_1_0);
			hdmi_writeb( (temp>>8)  & 0xFF, HDMI_V_SYNC_LINE_AFT_1_1);

			// VSYNC_LINE_AFT2
			temp += HDMIVideoParams[format].VSync;
			hdmi_writeb( (temp) & 0xFF, HDMI_V_SYNC_LINE_AFT_2_0);
			hdmi_writeb( (temp>>8)  & 0xFF, HDMI_V_SYNC_LINE_AFT_2_1);

		}
		else // V TOP and V BOT are not same
		{
			temp = (HDMIVideoParams[format].VTotal - HDMIVideoParams[format].VBlank*2 - 1)/2 
				+ HDMIVideoParams[format].VBlank;

			//V BLANK2
			hdmi_writeb( (temp) & 0xFF, HDMI_V2_BLANK_0);
			hdmi_writeb( (temp>>8)  & 0xFF, HDMI_V2_BLANK_1);

			// VBLANK_F0
			temp = (HDMIVideoParams[format].VTotal + HDMIVideoParams[format].VBlank*2 + 1)/2;
			hdmi_writeb( (temp) & 0xFF, HDMI_V_BLANK_F0_0);
			hdmi_writeb( (temp>>8)  & 0xFF, HDMI_V_BLANK_F0_1);

			// VBLANK_F3 == VACT5
			temp = (HDMIVideoParams[format].VTotal - (HDMIVideoParams[format].VBlank+1))/2;
			hdmi_writeb( (temp) & 0xFF, HDMI_V_BLANK_F3_0);
			hdmi_writeb( (temp>>8)  & 0xFF, HDMI_V_BLANK_F3_1);

			// VBLANK_F4 = VACT_SPACE6
			temp = (HDMIVideoParams[format].VTotal*3 + HDMIVideoParams[format].VBlank*2 + 1)/2;
			hdmi_writeb( (temp) & 0xFF, HDMI_V_BLANK_F4_0);
			hdmi_writeb( (temp>>8)  & 0xFF, HDMI_V_BLANK_F4_1);

			// VSYNC_LINE_AFT1
			temp += HDMIVideoParams[format].VFront;
			hdmi_writeb( (temp) & 0xFF, HDMI_V_SYNC_LINE_AFT_1_0);
			hdmi_writeb( (temp>>8)  & 0xFF, HDMI_V_SYNC_LINE_AFT_1_1);

			// VSYNC_LINE_AFT2
			temp += HDMIVideoParams[format].VSync;
			hdmi_writeb( (temp) & 0xFF, HDMI_V_SYNC_LINE_AFT_2_0);
			hdmi_writeb( (temp>>8)  & 0xFF, HDMI_V_SYNC_LINE_AFT_2_1);
		}

		// VBLANK_F1
		temp = HDMIVideoParams[format].VTotal;
		hdmi_writeb( (temp) & 0xFF, HDMI_V_BLANK_F1_0);
		hdmi_writeb( (temp>>8)  & 0xFF, HDMI_V_BLANK_F1_1);


		temp = HDMIVideoParams[format].HTotal/2 + HDMIVideoParams[format].HFront;
		// VSYNC_LINE_AFT_PXL_1
		hdmi_writeb( temp & 0xFF, HDMI_V_SYNC_LINE_AFT_PXL_1_0 );
		hdmi_writeb( (temp>>8) & 0xFF, HDMI_V_SYNC_LINE_AFT_PXL_1_1 );
		// VSYNC_LINE_AFT_PXL_2
		hdmi_writeb( temp & 0xFF, HDMI_V_SYNC_LINE_AFT_PXL_1_0 );
		hdmi_writeb( (temp>>8) & 0xFF, HDMI_V_SYNC_LINE_AFT_PXL_1_1 );

		// VBLANK_F2 == VACT4
		temp = HDMIVideoParams[format].VTotal + HDMIVideoParams[format].VBlank;
		hdmi_writeb( (temp) & 0xFF, HDMI_V_BLANK_F2_0);
		hdmi_writeb( (temp>>8)  & 0xFF, HDMI_V_BLANK_F2_1);

		// VBLANK_F5
		temp = HDMIVideoParams[format].VTotal*2;
		hdmi_writeb( (temp) & 0xFF, HDMI_V_BLANK_F5_0);
		hdmi_writeb( (temp>>8)  & 0xFF, HDMI_V_BLANK_F5_1);
	}
	else // progressive mode
	{
		// not available
		return 0;
	}

	return 1;
}

/**
 * Set video registers as 3D Line Alternative video structure
 *
 * @param  format	[in] Video format
 * @return  If format is interlaced, return 0; Otherwise return 1.
 */
int hdmi_set_3D_LA_video(enum VideoFormat format)
{
	// only for progressive
	if ( HDMIVideoParams[format].interlaced )
	{
		// interlaced mode
		return 0;
	}
	else // progressive mode
	{
		unsigned int temp;

		// HBlank
		temp = HDMIVideoParams[format].HBlank;
		hdmi_writeb( temp & 0xFF, HDMI_H_BLANK_0 );
		hdmi_writeb( (temp>>8) & 0xFF, HDMI_H_BLANK_1 );

		// V1Blank
		temp = (unsigned int)HDMIVideoParams[format].VBlank*2;
		hdmi_writeb( temp & 0xFF, HDMI_V1_BLANK_0 );
		hdmi_writeb( (temp>>8) & 0xFF, HDMI_V1_BLANK_1 );

		// V2Blank
		temp = HDMIVideoParams[format].VTotal*2;
		hdmi_writeb( temp & 0xFF, HDMI_V2_BLANK_0 );
		hdmi_writeb( (temp>>8) & 0xFF, HDMI_V2_BLANK_1 );

		// VTotal
		hdmi_writeb( temp & 0xFF, HDMI_V_LINE_0 );
		hdmi_writeb( (temp>>8) & 0xFF, HDMI_V_LINE_1 );

		// Htotal
		hdmi_writeb( HDMIVideoParams[format].HTotal & 0xFF, HDMI_H_LINE_0 );
		hdmi_writeb( (HDMIVideoParams[format].HTotal>>8) & 0xFF, HDMI_H_LINE_1 );

		// H POL
		hdmi_writeb( HDMIVideoParams[format].HPol, HDMI_HSYNC_POL );

		// V POL
		hdmi_writeb( HDMIVideoParams[format].VPol, HDMI_VSYNC_POL );

		// HSYNC Front
		hdmi_writeb( (HDMIVideoParams[format].HFront-2) & 0xFF, HDMI_H_SYNC_START_0 );
		hdmi_writeb( ((HDMIVideoParams[format].HFront-2)>>8) & 0xFF, HDMI_H_SYNC_START_1 );

		// HSYNC End
		hdmi_writeb( ((HDMIVideoParams[format].HFront-2) + HDMIVideoParams[format].HSync) & 0xFF
								, HDMI_H_SYNC_END_0 );
		hdmi_writeb( (((HDMIVideoParams[format].HFront-2) + HDMIVideoParams[format].HSync)>>8) & 0xFF
								, HDMI_H_SYNC_END_1 );

		// VSYNC Front
		temp = HDMIVideoParams[format].VFront*2;
		hdmi_writeb( temp & 0xFF, HDMI_V_SYNC_LINE_BEF_1_0 );
		hdmi_writeb( (temp>>8) & 0xFF, HDMI_V_SYNC_LINE_BEF_1_1 );

		// VSYNC End
		temp += HDMIVideoParams[format].VSync*2;
		hdmi_writeb( temp & 0xFF, HDMI_V_SYNC_LINE_BEF_2_0 );
		hdmi_writeb( (temp>>8) & 0xFF, HDMI_V_SYNC_LINE_BEF_2_1 );
	}

	return 1;
}

/**
 * Set video registers as Side-by-Side Full video structure
 *
 * @param  format	[in] Video format
 * @return  1
 */
int hdmi_set_3D_SSF_video(enum VideoFormat format)
{
	unsigned int temp;

	// same with 2D but H is twice longer than 2D
	hdmi_set_2D_video(format);

	// H
	// HBlank
	temp = HDMIVideoParams[format].HBlank*2;
	hdmi_writeb( temp & 0xFF, HDMI_H_BLANK_0 );
	hdmi_writeb( (temp>>8) & 0xFF, HDMI_H_BLANK_1 );

	// Htotal
	temp = HDMIVideoParams[format].HTotal*2;
	hdmi_writeb( temp & 0xFF, HDMI_H_LINE_0 );
	hdmi_writeb( (temp>>8) & 0xFF, HDMI_H_LINE_1 );

	// HSync Start
	temp = HDMIVideoParams[format].HFront*2-2;
	hdmi_writeb( temp & 0xFF, HDMI_H_SYNC_START_0 );
	hdmi_writeb( (temp>>8) & 0xFF, HDMI_H_SYNC_START_1 );

	// HSYNC End
	temp += HDMIVideoParams[format].HSync*2;
	hdmi_writeb( temp & 0xFF, HDMI_H_SYNC_END_0 );
	hdmi_writeb( (temp>>8) & 0xFF, HDMI_H_SYNC_END_1 );

	temp = (HDMIVideoParams[format].HTotal/2 + HDMIVideoParams[format].HFront)*2;
	// VSYNC_LINE_AFT_PXL_1
	hdmi_writeb( temp & 0xFF, HDMI_V_SYNC_LINE_AFT_PXL_1_0 );
	hdmi_writeb( (temp>>8) & 0xFF, HDMI_V_SYNC_LINE_AFT_PXL_1_1 );

	// VSYNC_LINE_AFT_PXL_2
	hdmi_writeb( temp & 0xFF, HDMI_V_SYNC_LINE_AFT_PXL_1_0 );
	hdmi_writeb( (temp>>8) & 0xFF, HDMI_V_SYNC_LINE_AFT_PXL_1_1 );


	return 1;
}

/**
 * Set video registers as L + Depth video structure
 *
 * @param  format	[in] Video format
 * @return  If format is interlaced, return 0; Otherwise return 1.
 */
int hdmi_set_3D_LD_video(enum VideoFormat format)
{
	// same with 3D FP but only for prog
	// only for progressive
	if ( HDMIVideoParams[format].interlaced )
	{
		// interlaced mode
		return 0;
	}
	else // progressive mode
	{
		return hdmi_set_3D_FP_video(format);
	}
}

/**
 * Set video registers as L + Depth + Graphics + Graphics-Depth video structure
 *
 * @param  format	[in] Video format
 * @return  If format is interlaced, return 0; Otherwise return 1.
 */
int hdmi_set_3D_LDGFX_video(enum VideoFormat format)
{
	// similar to 3D LD but V is twice longer than LD

	// basic video parametres
	unsigned int temp;

	if ( HDMIVideoParams[format].interlaced )
	{
		return 0;
	}
	else
	{
		// HBlank
		hdmi_writeb( HDMIVideoParams[format].HBlank & 0xFF, HDMI_H_BLANK_0 );
		hdmi_writeb( (HDMIVideoParams[format].HBlank>>8) & 0xFF, HDMI_H_BLANK_1 );

		// V1 Blank
		hdmi_writeb( HDMIVideoParams[format].VBlank & 0xFF, HDMI_V1_BLANK_0 );
		hdmi_writeb( (HDMIVideoParams[format].VBlank>>8) & 0xFF, HDMI_V1_BLANK_1 );

		// HTotal
		hdmi_writeb( HDMIVideoParams[format].HTotal & 0xFF, HDMI_H_LINE_0 );
		hdmi_writeb( (HDMIVideoParams[format].HTotal>>8) & 0xFF, HDMI_H_LINE_1 );

		temp = HDMIVideoParams[format].VTotal*4;

		// VTotal
		hdmi_writeb( HDMIVideoParams[format].VTotal & 0xFF, HDMI_V_LINE_0 );
		hdmi_writeb( (HDMIVideoParams[format].VTotal>>8) & 0xFF, HDMI_V_LINE_1 );

		// V2 BLANK
		hdmi_writeb( temp & 0xFF, HDMI_V2_BLANK_0 );
		hdmi_writeb( (temp>>8) & 0xFF, HDMI_V2_BLANK_1 );

		// H POL
		hdmi_writeb( HDMIVideoParams[format].HPol, HDMI_HSYNC_POL );

		// V POL
		hdmi_writeb( HDMIVideoParams[format].VPol, HDMI_VSYNC_POL );

		// HSYNC Front
		hdmi_writeb( (HDMIVideoParams[format].HFront-2) & 0xFF, HDMI_H_SYNC_START_0 );
		hdmi_writeb( ((HDMIVideoParams[format].HFront-2)>>8) & 0xFF, HDMI_H_SYNC_START_1 );

		// HSYNC End
		hdmi_writeb( ((HDMIVideoParams[format].HFront-2) + HDMIVideoParams[format].HSync) & 0xFF
								, HDMI_H_SYNC_END_0 );
		hdmi_writeb( (((HDMIVideoParams[format].HFront-2) + HDMIVideoParams[format].HSync)>>8) & 0xFF
								, HDMI_H_SYNC_END_1 );

		// VSYNC Front
		hdmi_writeb( HDMIVideoParams[format].VFront & 0xFF, HDMI_V_SYNC_LINE_BEF_1_0 );
		hdmi_writeb( (HDMIVideoParams[format].VFront>>8) & 0xFF, HDMI_V_SYNC_LINE_BEF_1_1 );

		// VSYNC End
		hdmi_writeb( (HDMIVideoParams[format].VFront + HDMIVideoParams[format].VSync) & 0xFF
								, HDMI_V_SYNC_LINE_BEF_2_0 );
		hdmi_writeb( ((HDMIVideoParams[format].VFront + HDMIVideoParams[format].VSync)>>8) & 0xFF
								, HDMI_V_SYNC_LINE_BEF_2_1 );

		// for progressive
		hdmi_writeb(0x0, HDMI_INT_PRO_MODE );

		temp = HDMIVideoParams[format].VTotal;
		// VACT_SPACE1
		hdmi_writeb( temp & 0xFF, HDMI_VACT_SPACE1_0);
		hdmi_writeb( (temp>>8) & 0xFF, HDMI_VACT_SPACE1_1);

		// VACT_SPACE2
		temp += HDMIVideoParams[format].VBlank;
		hdmi_writeb( temp & 0xFF, HDMI_VACT_SPACE2_0);
		hdmi_writeb( (temp>>8) & 0xFF, HDMI_VACT_SPACE2_1);

		temp = HDMIVideoParams[format].VTotal*2;
		// VACT_SPACE3
		hdmi_writeb( temp & 0xFF, HDMI_VACT_SPACE3_0);
		hdmi_writeb( (temp>>8) & 0xFF, HDMI_VACT_SPACE3_1);

		// VACT_SPACE4
		temp += HDMIVideoParams[format].VBlank;
		hdmi_writeb( temp & 0xFF, HDMI_VACT_SPACE4_0);
		hdmi_writeb( (temp>>8) & 0xFF, HDMI_VACT_SPACE4_1);

		temp = HDMIVideoParams[format].VTotal*3;
		// VACT_SPACE5
		hdmi_writeb( temp & 0xFF, HDMI_VACT_SPACE5_0);
		hdmi_writeb( (temp>>8) & 0xFF, HDMI_VACT_SPACE5_1);

		// VACT_SPACE6
		temp += HDMIVideoParams[format].VBlank;
		hdmi_writeb( temp & 0xFF, HDMI_VACT_SPACE6_0);
		hdmi_writeb( (temp>>8) & 0xFF, HDMI_VACT_SPACE6_1);
	}

	return 1;
}

/**
 * If 2CH Audio Stream, fill only one Subpacket in Audio Sample Packet.
 *
 * @param  enable	[in] 0 to stop sending, 1 to start sending.
 * @return 1
 */
int hdmi_fill_one_subpacket(unsigned char enable)
{
	if (enable)
	{
		hdmi_writeb(HDMI_OLD_IP_VER,HDMI_IP_VER);
	}
	else
	{
		hdmi_writeb(HDMI_NEW_IP_VER,HDMI_IP_VER);
	}
	return 1;
}

#ifdef CONFIG_PM_RUNTIME
int hdmi_runtime_suspend(struct device *dev)
{
#if 0
	struct fb_info *info = registered_fb[0];
	struct tccfb_info *tccfb_info = NULL;
	struct tcc_dp_device *dp_device = NULL;
	
	if(gHdmiPwrInfo.status == PWR_STATUS_ON) {
		hdmi_stop();

		tccfb_info = info->par;

		if(tccfb_info->pdata.Mdp_data.DispNum == hdmi_lcdc_num)
			dp_device = &tccfb_info->pdata.Mdp_data;
		else if(tccfb_info->pdata.Sdp_data.DispNum == hdmi_lcdc_num)
			dp_device = &tccfb_info->pdata.Sdp_data;

		if(dp_device)
		{
			tca_vioc_displayblock_disable(dp_device);
			tca_vioc_displayblock_powerOff(dp_device);
			dp_device->DispDeviceType = TCC_OUTPUT_NONE;
		}
	}
#endif
	
	printk("%s:  \n", __FUNCTION__);

	gHDMISuspendStatus = 1;

	printk("%s: finish \n", __FUNCTION__);

	return 0;
}

int hdmi_runtime_resume(struct device *dev)
{
	printk("%s:  \n", __FUNCTION__);

	gHDMISuspendStatus = 0;

	return 0;
}

#endif

#ifdef CONFIG_PM
int hdmi_suspend(struct platform_device *dev, pm_message_t state)
{
	#ifndef CONFIG_ANDROID
	struct fb_info *info = registered_fb[0];
	struct tccfb_info *tccfb_info = NULL;
	struct tcc_dp_device *dp_device = NULL;
	tccfb_info = info->par;


	if(tccfb_info->pdata.Mdp_data.DispNum == hdmi_lcdc_num)
		dp_device = &tccfb_info->pdata.Mdp_data;
	else if(tccfb_info->pdata.Sdp_data.DispNum == hdmi_lcdc_num)
		dp_device = &tccfb_info->pdata.Sdp_data;

	if(dp_device)
	{
		tca_vioc_displayblock_disable(dp_device);
		tca_vioc_displayblock_powerOff(dp_device);
		dp_device->DispDeviceType = TCC_OUTPUT_NONE;
	}
	gLinuxHDMISuspendStatus = 1;
	#endif//
	printk(KERN_INFO "%s  state:%d \n", __FUNCTION__, gHdmiPwrInfo.status);

	return 0;
}

int hdmi_resume(struct platform_device *dev)
{
	printk(KERN_INFO "%s\n", __FUNCTION__);

/*
	if(gHdmiPwrInfo.status == PWR_STATUS_OFF) {

		tcc_hdmi_v5_power_on();
		tcc_hdmi_power_on();

		gHdmiPwrInfo.status = PWR_STATUS_OFF;

	}
*/

	return 0;
}
#else
#define hdmi_suspend NULL
#define hdmi_resume NULL
#endif

#ifdef CONFIG_PM
static const struct dev_pm_ops hdmi_pm_ops = {
#ifdef CONFIG_PM_RUNTIME
	.runtime_suspend      = hdmi_runtime_suspend,
	.runtime_resume       = hdmi_runtime_resume,
#else
	.suspend	= hdmi_suspend,
	.resume = hdmi_resume,
#endif
};
#endif

#ifdef CONFIG_OF
static void hdmi_parse_dt(struct device_node *np)
{
    int no_hdmi_gpio_manual_control = 0;
	of_property_read_u32(np, "vioc_lcdc_num", &hdmi_lcdc_num);
    of_property_read_u32(np, "no_gpio_manual_control", &no_hdmi_gpio_manual_control);
    hdmi_gpio_manual_control = no_hdmi_gpio_manual_control?0:1;
	dprintk("%s : hdmi_lcdc_num = %d, hdmi_gpio_manual_control = %d \n", __func__, hdmi_lcdc_num, hdmi_gpio_manual_control);
}
#else
static void hdmi_parse_dt(struct device_node *np)
{
}
#endif

#ifdef CONFIG_OF
static struct of_device_id hdmi_of_match[] = {
	{ .compatible = "telechips,tcc893x-hdmi" },
	{ .compatible = "telechips,tcc896x-hdmi" },
	{ .compatible = "telechips,tcc897x-hdmi" },
	{}
};
MODULE_DEVICE_TABLE(of, hdmi_of_match);
#endif


static int hdmi_probe(struct platform_device *pdev)
{
	int ret;
	
	if (!pdev->dev.of_node) {
		pdev_hdmi = pdev->dev.platform_data;;
		if (!pdev_hdmi) {
			dev_err(&pdev->dev, "no platform data\n");
			ret = -EINVAL;
			goto err_platform_data_failed;
		}
	}

	phdmi_data = kzalloc(sizeof(*phdmi_data), GFP_KERNEL);
	if (phdmi_data == NULL) {
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}
	
	phdmi_data->pdev = pdev_hdmi = &pdev->dev;

	hdmi_parse_dt(pdev->dev.of_node);
	
	printk("%s : hdmi_lcdc_num = %d\n", __func__,hdmi_lcdc_num);
	
	if (hdmi_pclk == NULL && hdmi_hclk == NULL ) {

		hdmi_pclk = of_clk_get(pdev->dev.of_node, 0);
		hdmi_hclk = of_clk_get(pdev->dev.of_node, 1);

		if (IS_ERR(hdmi_pclk)) {
			printk(KERN_WARNING "HDMI: failed to get hdmi pclock\n");
			hdmi_pclk = NULL;
			ret =  -ENODEV;
		}

		if (IS_ERR(hdmi_hclk)) {
			printk(KERN_WARNING "HDMI: failed to get hdmi hclock\n");
			hdmi_hclk = NULL;
			ret =  -ENODEV;
		}
	}

	if (hdmi_pclk)
		clk_prepare_enable(hdmi_pclk);

	if (hdmi_hclk)
		clk_prepare_enable(hdmi_hclk);

	#if defined(CONFIG_ARCH_TCC896X) || defined(CONFIG_ARCH_TCC897X) || defined(CONFIG_ARCH_TCC570X)
	{
		if(hdmi_ipclk == NULL) {
			hdmi_ipclk = of_clk_get(pdev->dev.of_node, 2);

		if (IS_ERR(hdmi_ipclk)) {
			printk(KERN_WARNING "HDMI: failed to get hdmi ipclock\n");
			hdmi_hclk = NULL;
			ret = -ENODEV;
		}
	}
	}
	#endif


    dprintk(KERN_INFO "%s\n", __FUNCTION__);

    pr_info(KERN_INFO "HDMI Driver ver. %s\n", VERSION);

    if (!machine_is_hdmidp())
        return -ENODEV;


    if (misc_register(&hdmi_misc_device))    {
        ret = -EBUSY;
	 	goto err_device_reg;
    }

	#if defined(CONFIG_ARCH_TCC893X)
	{
		tcc_ddi_reg_acess_hdmi();
	}
	#endif

	if (hdmi_pclk)
		clk_disable_unprepare(hdmi_pclk);

	if (hdmi_hclk)
		clk_disable_unprepare(hdmi_hclk);

#if defined(CONFIG_REGULATOR)
	vdd_v5p0 = regulator_get(pdev_hdmi, "vdd_v5p0");
    if (IS_ERR(vdd_v5p0)) {
        pr_warning("clock_table: failed to obtain vdd_v5p0\n");
        vdd_v5p0 = NULL;
	}

	vdd_hdmi = regulator_get(pdev_hdmi, "vdd_hdmi");
    if (IS_ERR(vdd_hdmi)) {
        pr_warning("clock_table: failed to obtain vdd_hdmi\n");
        vdd_hdmi = NULL;
	}
#endif

	#if defined(CONFIG_SWITCH_GPIO_HDMI)
	platform_device_register(&tcc_hdmi_detect_device);
	#endif//
	
#ifdef CONFIG_PM_RUNTIME
	pm_runtime_set_active(pdev_hdmi);	
	pm_runtime_enable(pdev_hdmi);  
	pm_runtime_get_noresume(pdev_hdmi);  //increase usage_count 
#endif

	
	return 0;
err_device_reg:
        printk(KERN_WARNING "HDMI: Couldn't register device , %d.\n", HDMI_MINOR);
	kfree(phdmi_data);
err_alloc_data_failed:
err_platform_data_failed:
	return ret;
}


static struct platform_driver tcc_hdmi = {
	.probe	= hdmi_probe,
	.remove	= hdmi_remove,
	.driver	= {
		.name	= "tcc_hdmi",
		.owner	= THIS_MODULE,
#ifdef CONFIG_PM_RUNTIME
		.pm		= &hdmi_pm_ops,
#endif
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(hdmi_of_match),
#endif
	},
};

static __init int hdmi_init(void)
{
	printk("%s \n", __FUNCTION__);
	
	return platform_driver_register(&tcc_hdmi);
}

static __exit void hdmi_exit(void)
{
	printk("%s \n", __FUNCTION__);

	platform_driver_unregister(&tcc_hdmi);
}

module_init(hdmi_init);
module_exit(hdmi_exit);

MODULE_AUTHOR("Telechips Inc. <linux@telechips.com>");
MODULE_DESCRIPTION("TCCxxx hdmi driver");
MODULE_LICENSE("GPL");
