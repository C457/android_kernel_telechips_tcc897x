/****************************************************************************
FileName    : kernel/drivers/video/tcc/vioc/tcc_composite.c
Description : 

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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/wait.h>
#include <linux/clk.h>
#include <linux/cpufreq.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/div64.h>
#include <asm/mach/map.h>
#include <asm/mach-types.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/fb.h>

#ifdef CONFIG_PM_RUNTIME
#include <linux/pm_runtime.h>
#endif

#ifdef CONFIG_PM
#include <linux/pm.h>
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#include <linux/of_address.h>
#include <linux/of_gpio.h>

#ifdef CONFIG_ARCH_TCC897X
#include <mach/bsp.h>
#include <mach/gpio.h>

#include <mach/tccfb.h>
#include <mach/tcc_fb.h>
#include <mach/tcc_composite_ioctl.h>
#include <mach/tccfb_ioctrl.h>
#include <mach/vioc_outcfg.h>
#include <mach/vioc_rdma.h>
#include <mach/vioc_wdma.h>
#include <mach/vioc_wmix.h>
#include <mach/vioc_disp.h>
#include <mach/vioc_global.h>
#include <mach/vioc_config.h>
#include <mach/vioc_scaler.h>
#include <mach/tca_display_config.h>
#include <mach/tcc_board_composite.h>
#include <mach/daudio_info.h>
#else
#include <video/tcc/tcc_types.h>
#include <video/tcc/tccfb.h>
#include <video/tcc/tcc_fb.h>
#include <video/tcc/tcc_composite_ioctl.h>
#include <video/tcc/tccfb_ioctrl.h>
#include <video/tcc/vioc_outcfg.h>
#include <video/tcc/vioc_rdma.h>
#include <video/tcc/vioc_wdma.h>
#include <video/tcc/vioc_wmix.h>
#include <video/tcc/vioc_disp.h>
#include <video/tcc/vioc_global.h>
#include <video/tcc/vioc_config.h>
#include <video/tcc/vioc_scaler.h>
#include <video/tcc/tca_display_config.h>
#include <video/tcc/tcc_board_composite.h>
#endif

#include "tcc_composite.h"
#include "tcc_composite_internal.h"

extern void tcc_video_post_process(struct tcc_lcdc_image_update *ImageInfo);

extern void tccxxx_GetAddress(unsigned char format, unsigned int base_Yaddr, unsigned int src_imgx, unsigned int  src_imgy,
									unsigned int start_x, unsigned int start_y, unsigned int* Y, unsigned int* U,unsigned int* V);

extern void tca_vioc_displayblock_powerOn(struct tcc_dp_device *pDisplayInfo);
extern void tca_vioc_displayblock_powerOff(struct tcc_dp_device *pDisplayInfo);
extern void tca_vioc_displayblock_disable(struct tcc_dp_device *pDisplayInfo);
extern void tca_vioc_displayblock_ctrl_set(unsigned int outDevice, struct tcc_dp_device *pDisplayInfo, stLTIMING *pstTiming, stLCDCTR *pstCtrl, unsigned int format);
extern void tca_fb_attach_start(struct tccfb_info *info);
extern void tca_fb_attach_stop(struct tccfb_info *info);

/*****************************************************************************

 VARIABLES

******************************************************************************/

extern char fb_power_state;

/* Debugging stuff */
static int debug = 0;
#define dprintk(msg...)	if (debug) { printk( "tcc_composite: " msg); }

#define TCC_LCDC1_USE

static int					Composite_LCDC_Num = -1;
static int					Composite_Disp_Num;
static PVIOC_DISP			pComposite_DISP;
static PVIOC_WMIX			pComposite_WMIX;
static PVIOC_RDMA			pComposite_RDMA_UI;
static PVIOC_RDMA			pComposite_RDMA_VIDEO;
static PVIOC_SC				pComposite_SCALER;
static PDDICONFIG 			pComposite_DDICFG;
static PVIOC_IREQ_CONFIG	pComposite_IREQ; 

static PVIOC_DISP			pComposite_Attach_DISP;
static PVIOC_WMIX			pComposite_Attach_WMIX;
static PVIOC_RDMA			pComposite_Attach_RDMA_UI;
static PVIOC_RDMA			pComposite_Attach_RDMA_VIDEO;

#define DEVICE_NAME			"composite"
#define COMPOSITE_MINOR		205

#define COMPOSITE_DETECT_GPIO		NULL
#define COMPOSITE_DETECT_EINTSEL	0
#define COMPOSITE_DETECT_EINTNUM	0
#define COMPOSITE_DETECT_EINT		NULL
 
static struct clk *composite_lcdc0_clk;
static struct clk *composite_lcdc1_clk;

static char tcc_composite_mode = COMPOSITE_MAX_M;
static char tcc_composite_started = 0;
static char tcc_composite_attached = 0;
static char tcc_composite_starter = 0;
static char tcc_composite_attached_cvbs = 0;

static char composite_plugout = 0;
static char composite_bypass_mode = 0;

static unsigned int gCompositeSuspendStatus = 0;
static unsigned int gLinuxCompositeSuspendStatus = 0;

static struct device *pdev_composite;

#define RDMA_UVI_MAX_WIDTH             3072

#if defined(CONFIG_TCC_DISPLAY_LCD_CVBS)
static char attach_block_operating = 0;
static char detach_block_operating = 0;
#else
static int hdmi_hpd_port = 0;
#endif

#if defined(CONFIG_SWITCH_GPIO_COMPOSITE)
static struct platform_device tcc_composite_detect_device = {
	.name   = "switch-gpio-composite-detect",
	.id             = -1,
	.dev = {
		.platform_data = NULL,
	}, 
};
#endif


/*****************************************************************************
 Function Name : tcc_composite_detect()
******************************************************************************/
int tcc_composite_detect(void)
{
	int detect = true;

	/* Check the HDMI detection */
	#if defined(CONFIG_TCC_DISPLAY_MODE_AUTO_DETECT) || defined(CONFIG_TCC_DISPLAY_MODE_DUAL_HDMI_CVBS)
		if(gpio_get_value(hdmi_hpd_port))
			detect = false;
	#elif defined(CONFIG_TCC_DISPLAY_MODE_DUAL_AUTO) || defined(CONFIG_TCC_DISPLAY_LCD_CVBS)
		detect = false;
	#endif
 		
	return detect;
}

/*****************************************************************************
 Function Name : tcc_composite_connect_lcdc()
******************************************************************************/
int tcc_composite_connect_lcdc(int lcdc_num, int enable)
{
	PDDICONFIG pHwDDICFG = pComposite_DDICFG;
	
	VIOC_OUTCFG_SetOutConfig(VIOC_OUTCFG_SDVENC, lcdc_num);

	if(enable)
		pHwDDICFG->NTSCPAL_EN.nREG |= Hw0;	// enable
	else
		pHwDDICFG->NTSCPAL_EN.nREG &= ~Hw0;	// disable

	return 0;
}

/*****************************************************************************
 Function Name : tcc_composite_get_started()
******************************************************************************/
int tcc_composite_get_started(void)
{
	int ret = 0;

	if(tcc_composite_started && !tcc_composite_attached)
		ret = 1;

	return ret;
}

/*****************************************************************************
 Function Name : tcc_composite_get_spec()
******************************************************************************/
void tcc_composite_get_spec(COMPOSITE_MODE_TYPE mode, COMPOSITE_SPEC_TYPE *spec)
{
	switch(mode)
	{
		case NTSC_M:
		case NTSC_M_J:
		case PAL_M:
		case NTSC_443:
		case PSEUDO_PAL:
			spec->composite_clk = 27*1000*1000;
			spec->composite_bus_width = 8;
			spec->composite_lcd_width = 720;
			spec->composite_lcd_height = 480;
		#ifdef TCC_COMPOSITE_CCIR656
			spec->composite_LPW = 224 - 1; 					// line pulse width
			spec->composite_LPC = 720 * 2 - 1; 				// line pulse count (active horizontal pixel - 1)
			spec->composite_LSWC = 20 - 1;					// line start wait clock (the number of dummy pixel clock - 1)
			spec->composite_LEWC = 32 - 1;					// line end wait clock (the number of dummy pixel clock - 1)
		#else
			spec->composite_LPW = 212 - 1; 					// line pulse width
			spec->composite_LPC = 720 * 2 - 1; 				// line pulse count (active horizontal pixel - 1)
			spec->composite_LSWC = 32 - 1;					// line start wait clock (the number of dummy pixel clock - 1)
			spec->composite_LEWC = 32 - 1;					// line end wait clock (the number of dummy pixel clock - 1)
		#endif

			spec->composite_VDB = 0; 						// Back porch Vsync delay
			spec->composite_VDF = 0; 						// front porch of Vsync delay

			spec->composite_FPW1 = 1 - 1;					// TFT/TV : Frame pulse width is the pulse width of frmae clock
			spec->composite_FLC1 = 480 - 1;					// frmae line count is the number of lines in each frmae on the screen
			spec->composite_FSWC1 = 37 - 1;					// frmae start wait cycle is the number of lines to insert at the end each frame
			spec->composite_FEWC1 = 7 - 1;					// frame start wait cycle is the number of lines to insert at the begining each frame
			spec->composite_FPW2 = 1 - 1;					// TFT/TV : Frame pulse width is the pulse width of frmae clock
			spec->composite_FLC2 = 480 - 1;					// frmae line count is the number of lines in each frmae on the screen
			spec->composite_FSWC2 = 38 - 1;					// frmae start wait cycle is the number of lines to insert at the end each frame
			spec->composite_FEWC2 = 6 - 1; 					// frame start wait cycle is the number of lines to insert at the begining each frame
			break;

		case NTSC_N:
		case NTSC_N_J:
		case PAL_N:
		case PAL_B:
		case PAL_G:
		case PAL_H:
		case PAL_I:
		case PSEUDO_NTSC:		
			spec->composite_clk = 27*1000*1000;
			spec->composite_bus_width = 8;
			spec->composite_lcd_width = 720;
			spec->composite_lcd_height = 576;
			spec->composite_LPW = 128 - 1; 					// line pulse width
			spec->composite_LPC = 720 * 2 - 1; 				// line pulse count (active horizontal pixel - 1)
			spec->composite_LSWC = 138 - 1;					// line start wait clock (the number of dummy pixel clock - 1)
			spec->composite_LEWC = 22 - 1;					// line end wait clock (the number of dummy pixel clock - 1)

			spec->composite_VDB = 0; 						// Back porch Vsync delay
			spec->composite_VDF = 0; 						// front porch of Vsync delay
				
			spec->composite_FPW1 = 1 - 1;					// TFT/TV : Frame pulse width is the pulse width of frmae clock
			spec->composite_FLC1 = 576 - 1;					// frmae line count is the number of lines in each frmae on the screen
			spec->composite_FSWC1 = 43-1;					// frmae start wait cycle is the number of lines to insert at the end each frame
			spec->composite_FEWC1 = 5-1;					// frame start wait cycle is the number of lines to insert at the begining each frame
			spec->composite_FPW2 = 1 - 1;					// TFT/TV : Frame pulse width is the pulse width of frmae clock
			spec->composite_FLC2 = 576 - 1;					// frmae line count is the number of lines in each frmae on the screen
			spec->composite_FSWC2 = 44-1;//4 					// frmae start wait cycle is the number of lines to insert at the end each frame
			spec->composite_FEWC2 = 4-1; 					// frame start wait cycle is the number of lines to insert at the begining each frame
			break;

		default:
			spec->composite_clk = 0;
			pr_err("%s: error get timing\n", __func__);
			break;
	}
}

/*****************************************************************************
 Function Name : tcc_composite_set_lcd2tv()
******************************************************************************/
void tcc_composite_set_lcd2tv(COMPOSITE_MODE_TYPE type)
{
	COMPOSITE_SPEC_TYPE 	spec;
	stLTIMING				CompositeTiming;
	stLCDCTR				LcdCtrlParam;

	struct fb_info *info = registered_fb[0];
	struct tccfb_info *tccfb_info = NULL;
	struct tcc_dp_device *dp_device = NULL;

	dprintk("%s, type=%d\n", __func__, type);
	
	tcc_composite_get_spec(type, &spec);
	if (!spec.composite_clk) {
		return;
	}

	CompositeTiming.lpw = spec.composite_LPW;
	CompositeTiming.lpc = spec.composite_LPC + 1;
	CompositeTiming.lswc = spec.composite_LSWC + 1;
	CompositeTiming.lewc = spec.composite_LEWC + 1;
	
	CompositeTiming.vdb = spec.composite_VDB;
	CompositeTiming.vdf = spec.composite_VDF;
	CompositeTiming.fpw = spec.composite_FPW1;
	CompositeTiming.flc = spec.composite_FLC1;
	CompositeTiming.fswc = spec.composite_FSWC1;
	CompositeTiming.fewc = spec.composite_FEWC1;
	CompositeTiming.fpw2 = spec.composite_FPW2;
	CompositeTiming.flc2 = spec.composite_FLC2;
	CompositeTiming.fswc2 = spec.composite_FSWC2;
	CompositeTiming.fewc2 = spec.composite_FEWC2;
	
	memset(&LcdCtrlParam, 0, sizeof(LcdCtrlParam));

	#if defined(CONFIG_TCC_OUTPUT_COLOR_SPACE_YUV) || defined(CONFIG_TCC_COMPOSITE_COLOR_SPACE_YUV)
		LcdCtrlParam.r2ymd = 0;
	#else
		LcdCtrlParam.r2ymd = 3;
	#endif

	LcdCtrlParam.ckg = 1;
	LcdCtrlParam.id= 0;
	LcdCtrlParam.iv = 0;
	LcdCtrlParam.ih = 1;
	LcdCtrlParam.ip = 1;
	LcdCtrlParam.clen = 1;
	LcdCtrlParam.r2y = 1;
	LcdCtrlParam.pxdw = 6;
	LcdCtrlParam.dp = 0;
	LcdCtrlParam.ni = 0;
	LcdCtrlParam.tv = 1;

	LcdCtrlParam.advi = 1;

	#if defined(CONFIG_TCC_M2M_USE_INTERLACE_OUTPUT)
		LcdCtrlParam.advi = 0;
	#endif

	#if defined(CONFIG_TCC_OUTPUT_COLOR_SPACE_YUV) || defined(CONFIG_TCC_COMPOSITE_COLOR_SPACE_YUV)
		if(tcc_composite_starter == 0)
			LcdCtrlParam.r2y = 0;
	#endif

	tccfb_info = info->par;

#if defined(CONFIG_HWCOMPOSER_OVER_1_1_FOR_STB)
		dp_device = &tccfb_info->pdata.Sdp_data;
#else
	if(tccfb_info->pdata.Mdp_data.DispNum == Composite_LCDC_Num) {
		dp_device = &tccfb_info->pdata.Mdp_data;
	}
	else if(tccfb_info->pdata.Sdp_data.DispNum == Composite_LCDC_Num) {
		dp_device = &tccfb_info->pdata.Sdp_data;
		dp_device->DispOrder = 1;
	}
#endif

	if(dp_device)
	{
		tca_vioc_displayblock_disable(dp_device);

		dp_device->DispDeviceType = TCC_OUTPUT_COMPOSITE;

		if(tcc_composite_attached) {
			dp_device->FbUpdateType = FB_ATTACH_UPDATE;
		}
		else {
			dp_device->FbUpdateType = FB_SC_RDMA_UPDATE;
			dp_device->sc_num0 = VIOC_SC2;
		}



		#if defined(CONFIG_HWCOMPOSER_OVER_1_1_FOR_STB)
			dp_device->DispOrder = 1;	//Set to Sub Display

			#ifdef CONFIG_PRESENTATION_SECONDAY_DISPLAY_RESIZE_STB
				dp_device->FbUpdateType = FB_SC_RDMA_UPDATE;
				dp_device->sc_num0 = VIOC_SC4;
			#else
				dp_device->FbUpdateType = FB_RDMA_UPDATE;
			#endif

			#ifdef CONFIG_PRESENTATION_SECONDAY_DISPLAY_RESIZE_STB
				if(dp_device->FbUpdateType == FB_SC_RDMA_UPDATE)	{
					if(VIOC_CONFIG_PlugOut(dp_device->sc_num0) == VIOC_PATH_DISCONNECTED)
					{
						VIOC_SC_SetSWReset(dp_device->sc_num0, dp_device->rdma_info[RDMA_FB].blk_num, 0xFF);
						VIOC_CONFIG_PlugIn(dp_device->sc_num0, dp_device->rdma_info[RDMA_FB].blk_num);
					}
				}
			#endif
		#endif//
		tca_vioc_displayblock_powerOn(dp_device);
		tca_vioc_displayblock_ctrl_set(VIOC_OUTCFG_SDVENC, dp_device, &CompositeTiming, &LcdCtrlParam, 0);
	}

	#if defined(CONFIG_TCC_OUTPUT_COLOR_SPACE_YUV) || defined(CONFIG_TCC_COMPOSITE_COLOR_SPACE_YUV)
		if(tcc_composite_starter == 0)
			VIOC_WMIX_SetBGColor(pComposite_WMIX, 0x00, 0x80, 0x80, 0x00);
		else
			VIOC_WMIX_SetBGColor(pComposite_WMIX, 0x00, 0x00, 0x00, 0xff);
	#else
		VIOC_WMIX_SetBGColor(pComposite_WMIX, 0x00, 0x00, 0x00, 0xff);
	#endif
}

/*****************************************************************************
 Function Name : tcc_composite_set_bypass()
******************************************************************************/
void tcc_composite_set_bypass(char bypass_mode)
{
	dprintk("%s, bypass_mode=%d\n", __func__, bypass_mode);

	composite_bypass_mode = bypass_mode;
}

static int onthefly_using;
/*****************************************************************************
 Function Name : tcc_composite_update()
******************************************************************************/
// 0 : 3 : layer enable/disable 
#ifdef CONFIG_HDMI_DISPLAY_LASTFRAME
extern int enabled_LastFrame;
void tcc_plugout_for_composite(int ch_layer)
{
	unsigned int iSCType;

	#if defined(CONFIG_ARCH_TCC893X)
		#if defined(CONFIG_CHIP_TCC8933S) || defined(CONFIG_CHIP_TCC8935S) || defined(CONFIG_CHIP_TCC8937S) || defined(CONFIG_MACH_TCC8930ST)
		iSCType = VIOC_SC1;
		#else
		iSCType = VIOC_SC3;
		#endif
	#else
		iSCType = VIOC_SC1;
	#endif /* CONFIG_ARCH_TCC893X */

	if(ISSET(onthefly_using, 1<<ch_layer))
	{
		VIOC_CONFIG_PlugOut(iSCType);
		BITCLR(onthefly_using, 1 << ch_layer);
	}

}
EXPORT_SYMBOL(tcc_plugout_for_composite);
extern unsigned int LastFrame;
#endif

void tcc_composite_update(struct tcc_lcdc_image_update *ImageInfo)
{
	VIOC_DISP *pDISPBase;
	VIOC_WMIX *pWMIXBase;
	VIOC_RDMA *pRDMABase;
#ifdef CONFIG_HDMI_DISPLAY_LASTFRAME
	VIOC_RDMA *pRDMABase_temp;
#endif		
	VIOC_SC *pSC;
	unsigned int lcd_width = 0, lcd_height = 0, lcd_h_pos = 0, lcd_w_pos = 0;
	unsigned int iSCType;
	unsigned int nRDMB;
	unsigned int interlace_output = 0;

	pSC = pComposite_SCALER;
	iSCType = VIOC_SC1;
	
	dprintk("%s enable:%d, layer:%d, fmt:%d, Fw:%d, Fh:%d, Iw:%d, Ih:%d, fmt:%d onthefly:%d\n", __func__, ImageInfo->enable, ImageInfo->Lcdc_layer,
			ImageInfo->fmt,ImageInfo->Frame_width, ImageInfo->Frame_height, ImageInfo->Image_width, ImageInfo->Image_height, ImageInfo->fmt, ImageInfo->on_the_fly);
	
	ImageInfo->Lcdc_layer = 3;

	pDISPBase = (VIOC_DISP*)pComposite_DISP;
	pWMIXBase = (VIOC_WMIX*)pComposite_WMIX;		
	pRDMABase = (VIOC_RDMA*)pComposite_RDMA_VIDEO;
#ifdef CONFIG_HDMI_DISPLAY_LASTFRAME
	pRDMABase_temp  = (VIOC_RDMA*)tcc_p2v(HwVIOC_RDMA05);
#endif	

	if(!ImageInfo->enable)	{
		volatile PVIOC_IREQ_CONFIG pIREQConfig;
		pIREQConfig = (volatile PVIOC_IREQ_CONFIG)pComposite_IREQ;

		VIOC_RDMA_SetImageDisable(pRDMABase);		

		nRDMB = Composite_LCDC_Num*4 + ImageInfo->Lcdc_layer;
		BITCSET(pIREQConfig->uSOFTRESET.nREG[0], (0x1<<nRDMB), (0x1<<nRDMB)); // rdma reset
		BITCSET(pIREQConfig->uSOFTRESET.nREG[0], (0x1<<nRDMB), (0x0<<nRDMB)); // rdma reset

#ifdef CONFIG_HDMI_DISPLAY_LASTFRAME
		if(enabled_LastFrame){
			VIOC_RDMA_SetImageDisable(pRDMABase_temp);	
		}
#endif	

		if(ImageInfo->MVCframeView != 1)
		{
			if(ISSET(onthefly_using, 1<<ImageInfo->Lcdc_layer))
			{
				dprintk("%s scaler_%d is plugged out!!\n", __func__, iSCType);
			
				VIOC_CONFIG_PlugOut(iSCType);
				BITCLR(onthefly_using, 1 << ImageInfo->Lcdc_layer);

				BITCSET(pIREQConfig->uSOFTRESET.nREG[0], (0x1<<(28+iSCType)), (0x1<<(28+iSCType))); // scaler reset
				BITCSET(pIREQConfig->uSOFTRESET.nREG[0], (0x1<<(28+iSCType)), (0x0<<(28+iSCType))); // scaler reset

			}		
		}		
#ifdef CONFIG_HDMI_DISPLAY_LASTFRAME
		LastFrame = 0;
#endif
		return;
	}	

	VIOC_DISP_GetSize(pDISPBase, &lcd_width, &lcd_height);
	
	if((!lcd_width) || (!lcd_height)){
		printk(" %s Error :: lcd_width %d, lcd_height %d, hdmi_lcdc=%d, enable=%d\n", __func__,lcd_width, lcd_height, Composite_LCDC_Num, ImageInfo->enable);
		return;
	}

	if(ImageInfo->fmt >TCC_LCDC_IMG_FMT_MAX)
		return;

	#if defined(CONFIG_TCC_VIDEO_DISPLAY_BY_VSYNC_INT)
		if(ImageInfo->outputMode != OUTPUT_COMPOSITE)
			return;
	#endif

	//dprintk("%s lcdc:%d, pRDMA:0x%08x, pWMIX:0x%08x, pDISP:0x%08x, addr0:0x%08x\n", __func__, hdmi_lcdc, pRDMABase, pWMIXBase, pDISPBase, ImageInfo->addr0);

#if defined(CONFIG_TCC_M2M_USE_INTERLACE_OUTPUT)
	if( !(pDISPBase->uCTRL.nREG & HwDISP_NI )) {//interlace mode
		interlace_output = 1;
	}
#endif

	if((ImageInfo->MVCframeView != 1) && ImageInfo->on_the_fly)
	{
		unsigned int RDMA_NUM;
		RDMA_NUM = Composite_LCDC_Num ? (ImageInfo->Lcdc_layer + VIOC_SC_RDMA_04) : ImageInfo->Lcdc_layer;

		VIOC_SC_SetSrcSize(pSC, ImageInfo->Frame_width, ImageInfo->Frame_height);
		VIOC_SC_SetDstSize (pSC, ImageInfo->Image_width, ImageInfo->Image_height);			// set destination size in scaler
		VIOC_SC_SetOutSize (pSC, ImageInfo->Image_width, ImageInfo->Image_height);			// set output size in scaer

		if(ImageInfo->Frame_width==ImageInfo->Image_width && ImageInfo->Frame_height==ImageInfo->Image_height) {
			VIOC_SC_SetBypass (pSC, true);
			dprintk("%s scaler_%d is plug in SetBypass ON \n", __func__, iSCType);
		}else {
			VIOC_SC_SetBypass (pSC, false);
			dprintk("%s scaler_%d is plug in SetBypass OFF \n", __func__, iSCType);
		}

		if(!onthefly_using)
		{ 
			VIOC_RDMA_SetImageDisable(pRDMABase);
			dprintk("%s scaler_%d is plug in RDMA %d \n", __func__, RDMA_NUM, iSCType);
			BITSET(onthefly_using, 1 << ImageInfo->Lcdc_layer);
			VIOC_CONFIG_PlugIn (iSCType, RDMA_NUM);	
		}
	}
	else
	{
		if(ISSET(onthefly_using, 1<<ImageInfo->Lcdc_layer))
		{
			dprintk("%s scaler_%d is plug out  \n", __func__, iSCType);
			VIOC_RDMA_SetImageDisable(pRDMABase);
			VIOC_CONFIG_PlugOut(iSCType);
			BITCLR(onthefly_using, 1 << ImageInfo->Lcdc_layer);
		}
	}

	if(onthefly_using)
		VIOC_SC_SetUpdate (pSC);
		
	// position
	VIOC_WMIX_SetPosition(pWMIXBase, ImageInfo->Lcdc_layer, ImageInfo->offset_x, ImageInfo->offset_y);
	VIOC_WMIX_SetUpdate(pWMIXBase);	
		
	#if defined(CONFIG_TCC_OUTPUT_COLOR_SPACE_YUV) || defined(CONFIG_TCC_COMPOSITE_COLOR_SPACE_YUV)
		VIOC_RDMA_SetImageY2REnable(pRDMABase, false);
	#else
		if(ImageInfo->fmt >= TCC_LCDC_IMG_FMT_UYVY && ImageInfo->fmt <= TCC_LCDC_IMG_FMT_YUV422ITL1)
		{
			VIOC_RDMA_SetImageY2REnable(pRDMABase, true);
			VIOC_RDMA_SetImageY2RMode(pRDMABase, 1);

			if( ImageInfo->Frame_width <= RDMA_UVI_MAX_WIDTH )
				VIOC_RDMA_SetImageUVIEnable(pRDMABase, true);
			else
				VIOC_RDMA_SetImageUVIEnable(pRDMABase, false);
		}
	#endif

	if(ImageInfo->one_field_only_interlace)
		VIOC_RDMA_SetImageOffset(pRDMABase, ImageInfo->fmt, ImageInfo->Frame_width*2);
	else
		VIOC_RDMA_SetImageOffset(pRDMABase, ImageInfo->fmt, ImageInfo->Frame_width);

	VIOC_RDMA_SetImageFormat(pRDMABase, ImageInfo->fmt);

	if(lcd_width > ImageInfo->Image_width)
		lcd_w_pos = (lcd_width - ImageInfo->Image_width)/2;
	
	if(lcd_height > ImageInfo->Image_height)
		lcd_h_pos = (lcd_height - ImageInfo->Image_height)/2;

	#if defined(CONFIG_TCC_OUTPUT_COLOR_SPACE_YUV) || defined(CONFIG_TCC_COMPOSITE_COLOR_SPACE_YUV)
		if(composite_bypass_mode)
		{
			ImageInfo->offset_x = lcd_w_pos;
			ImageInfo->offset_y = lcd_h_pos;
		}
	#endif
	
	if( !( ((ImageInfo->crop_left == 0) && (ImageInfo->crop_right == ImageInfo->Frame_width)) &&  ((ImageInfo->crop_top == 0) && (ImageInfo->crop_bottom == ImageInfo->Frame_height)))  )
	{
		unsigned int addr_Y = (unsigned int)ImageInfo->addr0;
		unsigned int addr_U = (unsigned int)ImageInfo->addr1;
		unsigned int addr_V = (unsigned int)ImageInfo->addr2;
		
		dprintk(" Image Crop left=[%d], right=[%d], top=[%d], bottom=[%d] \n", ImageInfo->crop_left, ImageInfo->crop_right, ImageInfo->crop_top, ImageInfo->crop_bottom);

		tccxxx_GetAddress(ImageInfo->fmt, (unsigned int)ImageInfo->addr0, ImageInfo->Frame_width, ImageInfo->Frame_height, 		
								ImageInfo->crop_left, ImageInfo->crop_top, &addr_Y, &addr_U, &addr_V);		

		if(ImageInfo->one_field_only_interlace)
			VIOC_RDMA_SetImageSize(pRDMABase, ImageInfo->crop_right - ImageInfo->crop_left, (ImageInfo->crop_bottom - ImageInfo->crop_top)/2);
		else
			VIOC_RDMA_SetImageSize(pRDMABase, ImageInfo->crop_right - ImageInfo->crop_left, ImageInfo->crop_bottom - ImageInfo->crop_top);
		VIOC_RDMA_SetImageBase(pRDMABase, addr_Y, addr_U, addr_V);
	}
	else
	{	
		dprintk(" don't Image Crop left=[%d], right=[%d], top=[%d], bottom=[%d] \n", ImageInfo->crop_left, ImageInfo->crop_right, ImageInfo->crop_top, ImageInfo->crop_bottom);
		if(ImageInfo->one_field_only_interlace)
			VIOC_RDMA_SetImageSize(pRDMABase, ImageInfo->Frame_width, ImageInfo->Frame_height/2);
		else
			VIOC_RDMA_SetImageSize(pRDMABase, ImageInfo->Frame_width, ImageInfo->Frame_height);
		VIOC_RDMA_SetImageBase(pRDMABase, ImageInfo->addr0, ImageInfo->addr1, ImageInfo->addr2);		
	}
		
	VIOC_RDMA_SetImageIntl(pRDMABase, interlace_output);

	VIOC_WMIX_SetUpdate(pWMIXBase);	

	VIOC_RDMA_SetImageEnable(pRDMABase);

	tcc_video_post_process(ImageInfo);
}

/*****************************************************************************
 Function Name : tcc_composite_get_mode()
******************************************************************************/
TCC_COMPOSITE_MODE_TYPE tcc_composite_get_mode(void)
{
	return tcc_composite_mode;
}

/*****************************************************************************
 Function Name : tcc_composite_enabled()
******************************************************************************/
int tcc_composite_enabled(void)
{
	volatile PNTSCPAL_ENCODER_CTRL pHwTVE_VEN = (PNTSCPAL_ENCODER_CTRL)HwNTSCPAL_ENC_CTRL_BASE;

	int iEnabled = 0;
	
	if(pHwTVE_VEN->VENCON.nREG & HwTVEVENCON_EN_EN)
	{
		iEnabled = 1;
	}

	return iEnabled;
}

/*****************************************************************************
 Function Name : tcc_composite_end()
******************************************************************************/
void tcc_composite_end(void)
{
	struct fb_info *info = registered_fb[0];
	struct tccfb_info *tccfb_info = NULL;
	struct tcc_dp_device *dp_device = NULL;
	
	dprintk("%s, LCDC_Num = %d \n", __func__, Composite_LCDC_Num);

	if(!tcc_composite_started)
		return;

	internal_tve_enable(0, 0);
	tcc_composite_connect_lcdc(Composite_LCDC_Num, 0);

	if(composite_plugout)
		composite_plugout = 0;

	tcc_composite_started = 0;

	tccfb_info = info->par;

	if(tccfb_info->pdata.Mdp_data.DispNum == Composite_LCDC_Num)
		dp_device = &tccfb_info->pdata.Mdp_data;
	else if(tccfb_info->pdata.Sdp_data.DispNum == Composite_LCDC_Num)
		dp_device = &tccfb_info->pdata.Sdp_data;
	else
		return;

	tca_vioc_displayblock_disable(dp_device);	
	tca_vioc_displayblock_powerOff(dp_device);	
	dp_device->DispDeviceType = TCC_OUTPUT_NONE;
	
}

/*****************************************************************************
 Function Name : tcc_composite_start()
******************************************************************************/
void tcc_composite_start(TCC_COMPOSITE_MODE_TYPE mode)
{
	COMPOSITE_MODE_TYPE composite_mode;

	printk("%s mode=%d, lcdc_num=%d\n", __func__, mode, Composite_LCDC_Num);

	tcc_composite_mode = mode;

	if(mode == COMPOSITE_NTST_M)
		composite_mode = NTSC_M;
	else
		composite_mode = PAL_B;

	tcc_composite_connect_lcdc(Composite_LCDC_Num, 1);
	tcc_composite_set_lcd2tv(composite_mode);

	internal_tve_enable(composite_mode, 1);

	tcc_composite_started = 1;
}

/*****************************************************************************
 Function Name : tcc_composite_clock_onoff()
******************************************************************************/
void tcc_composite_clock_onoff(char OnOff)
{
	dprintk("%s, OnOff = %d \n", __func__, OnOff);

	internal_tve_clock_onoff(OnOff);	
}

#ifdef CONFIG_PM_RUNTIME
static int composite_enable(void)
{		
	printk("%s\n", __func__);

	pm_runtime_get_sync(pdev_composite);

	return 0;
}

static int composite_disable(void)
{
	printk("%s\n", __func__);

	pm_runtime_put_sync(pdev_composite);

	return 0;
}
#endif

static int composite_blank(int blank_mode)
{
	int ret = 0;

	printk("%s : blank(mode=%d)\n", __func__, blank_mode);

#ifdef CONFIG_PM_RUNTIME
	if( (pdev_composite->power.usage_count.counter==1) && (blank_mode == 0))
	{
		// usage_count = 1 ( resume ), blank_mode = 0 ( FB_BLANK_UNBLANK ) is stable state when booting
		// don't call runtime_suspend or resume state 
		//printk("%s ### state = [%d] count =[%d] power_cnt=[%d] \n",__func__,blank_mode, pdev_composite->power.usage_count, pdev_composite->power.usage_count.counter);		  
		return 0;
	}

	switch(blank_mode)
	{
		case FB_BLANK_POWERDOWN:
		case FB_BLANK_NORMAL:
			composite_disable();
			break;
		case FB_BLANK_UNBLANK:
			composite_enable();
			break;
		case FB_BLANK_HSYNC_SUSPEND:
		case FB_BLANK_VSYNC_SUSPEND:
		default:
			ret = -EINVAL;
	}
#endif

	return ret;
}

/*****************************************************************************
 Function Name : tcc_composite_attach()
******************************************************************************/
void tcc_composite_attach(char lcdc_num, char mode, char starter_flag)
{
	char composite_mode;

	struct fb_info *info = registered_fb[0];
	struct tccfb_info *tccfb_info = NULL;

	dprintk("%s, lcdc_num=%d, mode=%d, starter=%d\n", __func__, lcdc_num, mode, starter_flag);
	
	/* reset previous display path */
	if(starter_flag)
	{
		VIOC_DISP_TurnOff(pComposite_Attach_DISP);
		VIOC_DISP_Wait_DisplayDone(pComposite_Attach_DISP);
		VIOC_RDMA_SetImageDisable(pComposite_Attach_RDMA_UI);
		VIOC_RDMA_SetImageDisable(pComposite_Attach_RDMA_VIDEO);
	}

	tccfb_info = info->par;
	
	tcc_composite_attached = 1;
	tcc_composite_starter = starter_flag;

#if defined(CONFIG_TCC_DISPLAY_LCD_CVBS)
	if(tccfb_info->pdata.lcdc_number ==1)
		Composite_LCDC_Num = 0;
	else
		Composite_LCDC_Num = 1;
#else
	Composite_LCDC_Num = lcdc_num;
#endif
	
	if(mode >= COMPOSITE_MAX_M)
		composite_mode = COMPOSITE_NTST_M;
	else
		composite_mode = mode;

	tcc_composite_clock_onoff(true);
	tcc_composite_start(composite_mode);
	tca_fb_attach_start(tccfb_info);

}

/*****************************************************************************
 Function Name : tcc_composite_detach()
******************************************************************************/
void tcc_composite_detach(void)
{
	struct fb_info *info = registered_fb[0];
	struct tccfb_info *tccfb_info = NULL;

	dprintk("%s\n", __func__);
	
	tccfb_info = info->par;

	tcc_composite_end();

	tca_fb_attach_stop(tccfb_info);

	tcc_composite_attached = 0;
	tcc_composite_starter = 0;

	Composite_LCDC_Num = Composite_Disp_Num;
}

/*****************************************************************************
 Function Name : tcc_composite_ioctl()
******************************************************************************/
static long tcc_composite_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	TCC_COMPOSITE_START_TYPE start;
	struct tcc_lcdc_image_update update;			
	
	dprintk("composite_ioctl IOCTRL[%d]\n", cmd);

	switch(cmd)
	{

		case TCC_COMPOSITE_IOCTL_HPD_SWITCH_STATUS	:
		{
			#if defined(CONFIG_SWITCH_GPIO_COMPOSITE)
			unsigned int enable = 0;
			//dprintk(KERN_INFO "ioctl(TCC_COMPOSITE_IOCTL_HPD_SWITCH_STATUS)\n");

			if(copy_from_user(&enable, (void *)arg, sizeof(enable)))
				return -EFAULT;

			
			if(tcc_composite_detect_device.dev.platform_data != NULL)
			{
				struct composite_gpio_switch_data	*switch_data = tcc_composite_detect_device.dev.platform_data;

				if(enable) {
				switch_data->send_composite_event(switch_data, TCC_COMPOSITE_ON);
					printk(KERN_INFO "ioctl(TCC_COMPOSITE_IOCTL_HPD_SWITCH_STATUS), enable = %d\n", enable);
				} else {
					switch_data->send_composite_event(switch_data, TCC_COMPOSITE_OFF);
					printk(KERN_INFO "ioctl(TCC_COMPOSITE_IOCTL_HPD_SWITCH_STATUS), enable = %d\n", enable);
				}
			}
			#endif//
			break;
		}

		case TCC_COMPOSITE_IOCTL_START:
			if(copy_from_user(&start, (void *)arg, sizeof(start)))
				return -EFAULT;

			if(tcc_composite_attached)
				tcc_composite_detach();

			tcc_composite_start(start.mode);
			break;

		case TCC_COMPOSITE_IOCTL_UPDATE:
			if(copy_from_user(&update, (void *)arg, sizeof(update)))
				return -EFAULT;

			tcc_composite_update(&update);									
			break;
			
		case TCC_COMPOSITE_IOCTL_END:
			#if defined(CONFIG_SWITCH_GPIO_COMPOSITE)
			if(tcc_composite_detect_device.dev.platform_data != NULL)
			{
				struct composite_gpio_switch_data	*switch_data = tcc_composite_detect_device.dev.platform_data;
				switch_data->send_composite_event(switch_data, TCC_COMPOSITE_OFF);
			}
			#endif//

			tcc_composite_end();
			break;

		case TCC_COMPOSITE_IOCTL_PROCESS:
			break;

		case TCC_COMPOSITE_IOCTL_ATTACH:
			#if defined(CONFIG_TCC_DISPLAY_MODE_USE)
				if(copy_from_user(&start, (void *)arg, sizeof(start)))
					return -EFAULT;
				
				#if defined(CONFIG_TCC_DISPLAY_LCD_CVBS)
					if(!attach_block_operating){
						/* start.lcdc : display number of source output */
						/* start.mode : composite mode (NTSC or PAL) */
						tcc_composite_mode = start.mode;

						if(tcc_composite_attached)
							tcc_composite_detach();

						if(start.lcdc)
							tcc_composite_attach(0, start.mode, 0);
						else
							tcc_composite_attach(1, start.mode, 0);

						attach_block_operating = 1;
						detach_block_operating = 0;
						tcc_composite_attached_cvbs = 1;
					}
				#else
					/* start.lcdc : display number of source output */
					/* start.mode : composite mode (NTSC or PAL) */
					tcc_composite_mode = start.mode;

					if(tcc_composite_attached)
						tcc_composite_detach();

					if(start.lcdc)
						tcc_composite_attach(0, start.mode, 0);
					else
						tcc_composite_attach(1, start.mode, 0);

					tcc_composite_attached_cvbs = 1;
				#endif
			#endif
			break;

		case TCC_COMPOSITE_IOCTL_DETACH:
			#if defined(CONFIG_TCC_DISPLAY_MODE_USE)
				#if defined(CONFIG_TCC_DISPLAY_LCD_CVBS)
					if(!detach_block_operating)
					{
						if(tcc_composite_started){
							tcc_composite_detach();
							detach_block_operating = 1;
							attach_block_operating = 0;
							tcc_composite_attached_cvbs = 0;
						}
					}
				#else
					if(tcc_composite_started){
						tcc_composite_detach();
						tcc_composite_attached_cvbs = 0;
					}
				#endif
			#endif
			break;
			
		case TCC_COPOSITE_IOCTL_BLANK:
		{
			unsigned int cmd;

			if (get_user(cmd, (unsigned int __user *)arg))
				return -EFAULT;

			dprintk(KERN_INFO "COMPOSITE: ioctl(TCC_COPOSITE_IOCTL_BLANK :  %d )\n", cmd);

			composite_blank(cmd);

			break;

		}

		case TCC_COPOSITE_IOCTL_GET_SUSPEND_STATUS:
		{
			dprintk(KERN_INFO "COMPOSITE: ioctl(TCC_COPOSITE_IOCTL_GET_SUSPEND_STATUS : %d )\n", gCompositeSuspendStatus);
			if (gLinuxCompositeSuspendStatus)
			{
				put_user(gLinuxCompositeSuspendStatus, (unsigned int __user*)arg);
				gLinuxCompositeSuspendStatus = 0;
			}
			else
			{
				put_user(gCompositeSuspendStatus, (unsigned int __user*)arg);
			}

			break;			
		}

		default:
			printk(" Unsupported IOCTL!!!\n");      
			break;			
	}

	return 0;
}

/*****************************************************************************
 Function Name : tcc_composite_open()
******************************************************************************/
static int tcc_composite_open(struct inode *inode, struct file *filp)
{	
	dprintk("%s  \n", __func__);

	tcc_composite_clock_onoff(1);

	return 0;
}

/*****************************************************************************
 Function Name : tcc_composite_release()
******************************************************************************/
static int tcc_composite_release(struct inode *inode, struct file *file)
{
	dprintk(" %s  \n", __func__);

	tcc_composite_clock_onoff(0);

	return 0;
}

#ifdef CONFIG_PM_RUNTIME
int composite_runtime_suspend(struct device *dev)
{
	printk("%s:  \n", __FUNCTION__);

	if(tcc_composite_started) {
		#if 1//defined(CONFIG_TCC_DISPLAY_LCD_CVBS)
			if(tcc_composite_attached_cvbs)
				tcc_composite_detach();
		#else
			tcc_composite_end();
		#endif
	}

	gCompositeSuspendStatus = 1;

	printk("%s: finish \n", __FUNCTION__);

	return 0;
}

int composite_runtime_resume(struct device *dev)
{
	printk("%s:  \n", __FUNCTION__);

	#if 1//defined(CONFIG_TCC_DISPLAY_LCD_CVBS)
		if(tcc_composite_attached_cvbs)
			tcc_composite_attach( Composite_LCDC_Num,tcc_composite_mode,0);
	#endif

	gCompositeSuspendStatus = 0;

	return 0;
}

#else
static int composite_suspend(struct device *dev)
{

#ifndef CONFIG_ANDROID //pjj
	/* Linux Platform */
	tcc_composite_end();
	gLinuxCompositeSuspendStatus = 1;
#endif//	
	printk("%s: gCompositeSuspendStatus = %d\n"
			, __FUNCTION__, gCompositeSuspendStatus);
	return 0;
}

static int composite_resume(struct device *dev)
{
	/* Linux Platform */

	printk("%s: gCompositeSuspendStatus = %d\n"
			, __FUNCTION__, gCompositeSuspendStatus);

	return 0;
}
#endif

static struct file_operations tcc_composite_fops = 
{
	.owner          = THIS_MODULE,
	.unlocked_ioctl = tcc_composite_ioctl,
	.open           = tcc_composite_open,
	.release        = tcc_composite_release,	
};

static struct miscdevice tcc_composite_misc_device =
{
    .minor = COMPOSITE_MINOR,
    .name  = "composite",
    .fops  = &tcc_composite_fops,
};


#ifdef CONFIG_OF
static int composite_parse_dt(struct device_node *np)
{
	struct device_node *np_fb_child, *np_fb, *np_fb_1st, *np_fb_2nd, *np_hpd;
	int index = 0, ret = 0;

	/* get the information of composite device node */
	composite_lcdc0_clk = of_clk_get(np, 0);
	BUG_ON(composite_lcdc0_clk == NULL);
	composite_lcdc1_clk = of_clk_get(np, 1);
	BUG_ON(composite_lcdc1_clk == NULL);

	np_fb_child = of_parse_phandle(np, "scaler", 0);
	if(np_fb_child) {
		of_property_read_u32_index(np, "scaler", 1, &index);
		pComposite_SCALER = (PVIOC_SC)of_iomap(np_fb_child, index);
	} else {
		printk("%s, could not find scaler node\n", __func__);
		ret = -ENODEV;
	}

	np_fb_child = of_parse_phandle(np, "config", 0);
	if(np_fb_child) {
		pComposite_IREQ = (PVIOC_IREQ_CONFIG)of_iomap(np_fb_child, 0);
	} else {
		printk("%s, could not find irq_config node\n", __func__);
		ret = -ENODEV;
	}

	np_fb_child = of_parse_phandle(np, "ddicfg", 0);
	if(np_fb_child) {
		pComposite_DDICFG = (PDDICONFIG)of_iomap(np_fb_child, 0);
	} else {
		printk("%s, could not find ddi_config node\n", __func__);
		ret = -ENODEV;
	}

	/* get the information of vioc-fb device node */
	np_fb = of_find_compatible_node(NULL, NULL, "telechips,vioc-fb");

	if(of_property_read_u32(np_fb, "telechips,fbdisplay_num", &Composite_Disp_Num)) {
		printk("%s, could not find fbdisplay_num\n", __func__);
		ret = -ENODEV;
	}

	Composite_Disp_Num = daudio_lcd_type_lvds_check();

	if(Composite_Disp_Num) {
		np_fb_1st = of_find_node_by_name(np_fb, "fbdisplay1");
		np_fb_2nd = of_find_node_by_name(np_fb, "fbdisplay0");
	}
	else {
		np_fb_1st = of_find_node_by_name(np_fb, "fbdisplay0");
		np_fb_2nd = of_find_node_by_name(np_fb, "fbdisplay1");
	}

	Composite_LCDC_Num = Composite_Disp_Num;

	/* get register address for main output */
	np_fb_child = of_parse_phandle(np_fb_1st,"telechips,disp", 0);
	if(np_fb_child) {
		pComposite_DISP = (VIOC_DISP *)of_iomap(np_fb_child, 0);
	} else {
		printk( "%s, could not find disp node\n", __func__);
		ret = -ENODEV;
	}

	np_fb_child = of_parse_phandle(np_fb_1st,"telechips,wmixer", 0);
	if(np_fb_child) {
		pComposite_WMIX = (VIOC_WMIX *)of_iomap(np_fb_child, 0);
	} else {
		printk( "%s, could not find wmixer node\n", __func__);
		ret = -ENODEV;
	}

	np_fb_child = of_parse_phandle(np_fb_1st,"telechips,rdma", 0);
	if(np_fb_child) {
		of_property_read_u32_index(np_fb_1st, "telechips,rdma", 1+0, &index);
		pComposite_RDMA_UI = (VIOC_RDMA *)of_iomap(np_fb_child, index);
		of_property_read_u32_index(np_fb_1st, "telechips,rdma", 1+3, &index);
		pComposite_RDMA_VIDEO = (VIOC_RDMA *)of_iomap(np_fb_child, index);
	} else {
		printk( "%s, could not find rdma node\n", __func__);
		ret = -ENODEV;
	}

	/* get register address for attached output */
	np_fb_child = of_parse_phandle(np_fb_2nd,"telechips,disp", 0);
	if(np_fb_child) {
		pComposite_Attach_DISP = (VIOC_DISP *)of_iomap(np_fb_child, 0);
	} else {
		printk( "%s, could not find disp node for attached output\n", __func__);
		ret = -ENODEV;
	}

	np_fb_child = of_parse_phandle(np_fb_2nd,"telechips,wmixer", 0);
	if(np_fb_child) {
		pComposite_Attach_WMIX = (VIOC_WMIX *)of_iomap(np_fb_child, 0);
	} else {
		printk( "%s, could not find wmixer node for attached output\n", __func__);
		ret = -ENODEV;
	}

	np_fb_child = of_parse_phandle(np_fb_2nd,"telechips,rdma", 0);
	if(np_fb_child) {
		of_property_read_u32_index(np_fb_2nd, "telechips,rdma", 1+0, &index);
		pComposite_Attach_RDMA_UI = (VIOC_RDMA *)of_iomap(np_fb_child, index);
		of_property_read_u32_index(np_fb_2nd, "telechips,rdma", 1+3, &index);
		pComposite_Attach_RDMA_VIDEO = (VIOC_RDMA *)of_iomap(np_fb_child, index);
	} else {
		printk( "%s, could not find rdma node for attached output\n", __func__);
		ret = -ENODEV;
	}

	/* get the information of hpd node */
#if !defined(CONFIG_TCC_DISPLAY_LCD_CVBS)
	#if defined(CONFIG_ARCH_TCC893X)
		np_hpd = of_find_compatible_node(NULL, NULL, "telechips,tcc893x-hdmi-hpd");
	#elif defined(CONFIG_ARCH_TCC896X)
		np_hpd = of_find_compatible_node(NULL, NULL, "telechips,tcc896x-hdmi-hpd");
	#elif defined(CONFIG_ARCH_TCC897X)
		np_hpd = of_find_compatible_node(NULL, NULL, "telechips,tcc897x-hdmi-hpd");
	#endif

	if(np_hpd) {
		hdmi_hpd_port = of_get_gpio(np_hpd, 0);
		if(gpio_is_valid(hdmi_hpd_port)) 
		{
			printk("%s, hpd port: 0x%02x\n", __func__, hdmi_hpd_port);
			gpio_request(hdmi_hpd_port, "composite-hdmi-hpd");
			gpio_direction_input(hdmi_hpd_port);
		}
		else
		{
			printk("%s, err to get hpd port\n", __func__);
			hdmi_hpd_port = -1;
		}
	} else {
		printk( "%s, could not find hpd node\n", __func__);
		ret = -ENODEV;
	}
#endif

	dprintk("%s, Composite_LCDC_Num = %d\n", __func__, Composite_Disp_Num);

	return ret;
}
#else
static int composite_parse_dt(struct device_node *np)
{
}
#endif

static int composite_probe(struct platform_device *pdev)
{
	struct tcc_hdmi_platform_data *composite_dev;

	printk("%s\n", __func__);

	pdev_composite = &pdev->dev;
	
	composite_dev = (struct tcc_hdmi_platform_data *)pdev_composite->platform_data;

	composite_parse_dt(pdev->dev.of_node);

	if (misc_register(&tcc_composite_misc_device))
	{
	    printk(KERN_WARNING "COMPOSITE: Couldn't register device 10, %d.\n", COMPOSITE_MINOR);
	    return -EBUSY;
	}

	internal_tve_init();

	#if defined(CONFIG_SWITCH_GPIO_COMPOSITE)
	platform_device_register(&tcc_composite_detect_device);
	#endif//

#ifdef CONFIG_PM_RUNTIME
	pm_runtime_set_active(pdev_composite);	
	pm_runtime_enable(pdev_composite);  
	pm_runtime_get_noresume(pdev_composite);  //increase usage_count 
#endif

	return 0;
}

static int composite_remove(struct platform_device *pdev)
{
	printk("%s LCDC:%d \n", __func__, Composite_LCDC_Num);

    misc_deregister(&tcc_composite_misc_device);

	if(composite_lcdc0_clk)
		clk_put(composite_lcdc0_clk);
	if(composite_lcdc1_clk)
		clk_put(composite_lcdc1_clk);

	return 0;
}

static const struct dev_pm_ops composite_pm_ops = {
#ifdef CONFIG_PM_RUNTIME
	.runtime_suspend = composite_runtime_suspend,
	.runtime_resume = composite_runtime_resume,
#else
	.suspend = composite_suspend,
	.resume = composite_resume,
#endif
};

#ifdef CONFIG_OF
static struct of_device_id composite_of_match[] = {
	{ .compatible = "telechips,tcc893x-composite" },
	{ .compatible = "telechips,tcc896x-composite" },
	{ .compatible = "telechips,tcc897x-composite" },
	{}
};
MODULE_DEVICE_TABLE(of, composite_of_match);
#endif

static struct platform_driver tcc_composite = {
	.probe	= composite_probe,
	.remove	= composite_remove,
	.driver	= {
		.name	= "tcc_composite",
		.owner	= THIS_MODULE,
#ifdef CONFIG_PM_RUNTIME
		.pm		= &composite_pm_ops,
#endif
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(composite_of_match),
#endif
	},
};

/*****************************************************************************
 Function Name : tcc_composite_init()
******************************************************************************/
int __init tcc_composite_init(void)
{
	return platform_driver_register(&tcc_composite);
}

/*****************************************************************************
 Function Name : tcc_composite_cleanup()
******************************************************************************/
void __exit tcc_composite_exit(void)
{
	platform_driver_unregister(&tcc_composite);
}

module_init(tcc_composite_init);
module_exit(tcc_composite_exit);

MODULE_AUTHOR("Telechps");
MODULE_DESCRIPTION("Telechips COMPOSITE Out driver");
MODULE_LICENSE("GPL");


