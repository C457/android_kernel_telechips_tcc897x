/*
 * linux/drivers/video/tcc/tcc_vsync.c
 *
 * Based on:    Based on s3c2410fb.c, sa1100fb.c and others
 * Author:  <linux@telechips.com>
 * Created: Aug  2014
 * Description: TCC Vsync Driver
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/mm_types.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/kthread.h>

#include <linux/fb.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/wait.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/cpufreq.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>

#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/mach-types.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>

#ifdef CONFIG_PM_RUNTIME
#include <linux/pm_runtime.h>
#endif

#ifdef CONFIG_PM
#include <linux/pm.h>
#endif

#include <soc/tcc/pmap.h>
#include <soc/tcc/timer.h>
#ifdef CONFIG_ARCH_TCC897X
#include <mach/bsp.h>
#include <mach/tccfb.h>
#include <mach/tccfb_ioctrl.h>
#include <mach/tcc_vsync_ioctl.h>
#include <mach/tcc_composite_ioctl.h>
#include <mach/tcc_component_ioctl.h>
#include <mach/tca_lcdc.h>
#include <mach/vioc_disp.h>
#include <mach/tcc_lut_ioctl.h>
#include <mach/vioc_blk.h>

#ifdef CONFIG_HDMI_DISPLAY_LASTFRAME
#include <mach/vioc_global.h>
#include <mach/vioc_config.h>
#include <mach/vioc_rdma.h>
#endif

#include <mach/tca_display_config.h>

#else	//CONFIG_ARCH_TCC897X
#include <video/tcc/tcc_types.h>

#include <video/tcc/tccfb.h>
#include <video/tcc/tccfb_ioctrl.h>
#include <video/tcc/tcc_vsync_ioctl.h>
#include <video/tcc/tcc_composite_ioctl.h>
#include <video/tcc/tcc_component_ioctl.h>
#include <video/tcc/tca_lcdc.h>
#include <video/tcc/vioc_disp.h>
#include <video/tcc/tcc_lut_ioctl.h>
#include <video/tcc/vioc_blk.h>

#ifdef CONFIG_HDMI_DISPLAY_LASTFRAME
#include <video/tcc/vioc_global.h>
#include <video/tcc/vioc_config.h>
#include <video/tcc/vioc_rdma.h>
#endif

#include <video/tcc/tca_display_config.h>

#endif	//CONFIG_ARCH_TCC897X

#ifdef TCC_VIDEO_DISPLAY_DEINTERLACE_MODE
#include "tcc_vioc_viqe_interface.h"
#include "viqe.h"
#endif

#include "tcc_vsync.h"

#define DEVICE_NAME "tcc_vsync"

#define ON_ENABLE 1
#define OFF_DISABLE 0


#if defined(CONFIG_HDMI_DISPLAY_LASTFRAME)

#ifdef CONFIG_ARCH_TCC897X
#include <mach/tcc_wmixer_ioctrl.h>
#include <mach/tcc_scaler_ioctrl.h>
#else
#include <video/tcc/tcc_wmixer_ioctrl.h>
#include <video/tcc/tcc_scaler_ioctrl.h>
#endif

#define NUM_OF_VSYNC_CH		(2)
tcc_video_disp tccvid_vsync[NUM_OF_VSYNC_CH];

struct tcc_lcdc_image_update 		CurrDisplayingImage;
struct tcc_lcdc_image_update		Last_ImageInfo;
unsigned int LastFrame = 0;
pmap_t fb_lastframe_pbuf;

VIOC_RDMA * pLastFrame_RDMABase = NULL;
VIOC_RDMA * pRDMA_LastFrame;

int enabled_LastFrame = 0;

unsigned int LastFrame_for_ResChanged = 0;
unsigned int LastFrame_for_CodecChanged = 0;
spinlock_t LastFrame_lockDisp;

#if defined(CONFIG_ARCH_TCC893X)
#define WMIXER_PATH	"/dev/wmixer1"
#else
#define WMIXER_PATH	"/dev/wmixer1"
#endif
//#define USE_SCALER2_FOR_LASTFRAME
#ifdef USE_SCALER2_FOR_LASTFRAME
#define SCALER_PATH	"/dev/scaler2"
#endif
extern void tcc_video_rdma_off(struct tcc_dp_device *pdp_data, struct tcc_lcdc_image_update *ImageInfo, int outputMode, int interlaced);

extern int tcc_ctrl_ext_frame(char enable);
#ifdef CONFIG_DISPLAY_EXT_FRAME
extern VIOC_RDMA* tcc_video_ext_display(struct tcc_dp_device *pdp_data, struct tcc_lcdc_image_update *ImageInfo);
static int bExt_Frame = 0;
static int bCamera_mode = 0;
static int bToggleBuffer = 0;
struct mutex ext_mutex;
VIOC_RDMA * pExtFrame_RDMABase = NULL;
static struct tcc_lcdc_image_update last_backup;
#define EXT_FRAME_OUT_MODE (TCC_OUTPUT_LCD)
#endif
#endif


/*******			define extern symbol	******/
#if defined(CONFIG_FB_TCC_COMPOSITE)
extern void tcc_composite_update(struct tcc_lcdc_image_update *update);
extern void tcc_composite_set_bypass(char bypass_mode);
#endif

#if defined(CONFIG_FB_TCC_COMPONENT)
extern void tcc_component_update(struct tcc_lcdc_image_update *update);
#endif

#if defined(CONFIG_TCC_DISPLAY_MODE_USE) && !defined(CONFIG_TCC_DISPLAY_MODE_AUTO_DETECT)
extern void tca_fb_attach_update_flag(void);
#endif

extern int tca_fb_divide_get_status(void);

extern void tca_lcdc_set_onthefly(struct tcc_dp_device *pdp_data, struct tcc_lcdc_image_update *ImageInfo);
extern void tca_scale_display_update(struct tcc_dp_device *pdp_data, struct tcc_lcdc_image_update *ImageInfo);
extern void tca_mvc_display_update(char hdmi_lcdc, struct tcc_lcdc_image_update *ImageInfo);

extern TCC_OUTPUT_TYPE Output_SelectMode;
extern unsigned int HDMI_video_hz;

#ifdef TCC_LCD_VIDEO_DISPLAY_BY_VSYNC_INT
static int vsync_started_device = LCD_START_VSYNC;
#endif

static unsigned int EX_OUT_LCDC;
static unsigned int LCD_LCDC_NUM;
static char vsync_power_state;
static int lcdc_interrupt_onoff = 0;
static int video_display_disable_check = 0;

static int debug_v = 0;
#define vprintk(msg...) if (debug_v) { printk( "tcc_vsync: " msg); }
static int debug = 0;
#define dprintk(msg...) if (debug) { printk( "tccfb_vsync: " msg); }
#define dprintk_drop(msg...) if (debug) { printk( "tccfb_vsync: " msg); }
#define lf_printk(msg...) //printk( "ext-frame: " msg);

#if 0
static int testToggleBit1=0;
static int testToggleBit=0;
#endif

inline static int tcc_vsync_get_time(tcc_video_disp *p);
inline static void tcc_vsync_set_time(tcc_video_disp *p, int time);
#define USE_VSYNC_TIMER
//#define USE_SOFT_IRQ_FOR_VSYNC

// if time gab of video frame over base time is bigger than this value, that frame is skipped.
#define VIDEO_SYNC_MARGIN_LATE		50
// if time gap of video frame over base time is less than this value, that frame is displayed immediately
#define VIDEO_SYNC_MARGIN_EARLY 	50

#define TIME_MARK_SKIP				(-1)

spinlock_t vsync_lock ;
spinlock_t vsync_lockDisp ;

int vsync_started = 0;
//static int vsync_output_mode = 0;

DECLARE_WAIT_QUEUE_HEAD( wq_consume ) ;

#ifdef USE_SOFT_IRQ_FOR_VSYNC
static struct work_struct vsync_work_q;
#endif

#ifdef USE_VSYNC_TIMER
static struct tcc_timer *vsync_timer = NULL;
static int g_is_vsync0_opened = 0;
static int g_is_vsync1_opened = 0;
#endif

struct tcc_vsync_display_info_t vsync_vioc1_disp;
struct tcc_vsync_display_info_t vsync_vioc0_disp;

static int tcc_vsync_mvc_status = 0;
static int tcc_vsync_mvc_overlay_priority = 0;

static int tcc_vsync_set_max_buffer(tcc_vsync_buffer_t * buffer_t, int buffer_count)
{
	buffer_t->max_buff_num = buffer_count;
	return buffer_count;
}

static int tcc_vsync_push_buffer(tcc_vsync_buffer_t * buffer_t, struct tcc_lcdc_image_update* inputData)
{

	if(atomic_read(&buffer_t->valid_buff_count) >= buffer_t->max_buff_num || atomic_read( &buffer_t->readable_buff_count) >= buffer_t->max_buff_num)
	{
		printk("error: buffer full %d, max %d %d ts %d \n", atomic_read(&buffer_t->valid_buff_count), buffer_t->max_buff_num,atomic_read( &buffer_t->readable_buff_count),inputData->time_stamp);
		return -1;
	}

	memcpy(&(buffer_t->stImage[buffer_t->writeIdx]),(void*)inputData, sizeof(struct tcc_lcdc_image_update) );
	
	if(++buffer_t->writeIdx >= buffer_t->max_buff_num)
		buffer_t->writeIdx = 0;
	

	atomic_inc( &buffer_t->valid_buff_count);
	atomic_inc( &buffer_t->readable_buff_count);
	
	return 0;
}

inline static int tcc_vsync_pop_buffer(tcc_vsync_buffer_t * buffer_t)
{
	if(atomic_read( &buffer_t->readable_buff_count) == 0)
	{
		printk("error: buffer empty \n");
		return -1;
	}

	if(++buffer_t->readIdx >= buffer_t->max_buff_num)
		buffer_t->readIdx = 0;

	atomic_dec(&buffer_t->readable_buff_count);

	return atomic_read( &buffer_t->readable_buff_count);
}

inline static void* tcc_vsync_get_buffer(tcc_vsync_buffer_t * buffer_t, int offset)
{
	int readIdx;

	if((atomic_read( &buffer_t->readable_buff_count)-offset) > 0)
	{
		readIdx = buffer_t->readIdx;

		while(offset)
		{
			if(++readIdx >= buffer_t->max_buff_num)
				readIdx = 0;

			offset--;
		}

		return (void*)&(buffer_t->stImage[readIdx]);
	}
	else
	{
		return NULL;
	}
}

inline static int tcc_vsync_clean_buffer(tcc_vsync_buffer_t * buffer_t)
{
	if(buffer_t->readIdx == buffer_t->clearIdx || atomic_read(&buffer_t->valid_buff_count) == 0)
	{
		vprintk("error: no clean buffer clearIdx(%d) valid_buff_count(%d) \n",	buffer_t->clearIdx,atomic_read(&buffer_t->valid_buff_count) );
		return -1;
	}

	vprintk("tcc_vsync_clean_buffer start clearIdx(%d) readIdx(%d) writeIdx(%d) valid_buff_count %d  \n", buffer_t->clearIdx,buffer_t->readIdx,buffer_t->writeIdx,atomic_read(&buffer_t->valid_buff_count));
	do
	{
		if(++buffer_t->clearIdx >= buffer_t->max_buff_num)
			buffer_t->clearIdx = 0;
		
		if(buffer_t->last_cleared_buff_id < buffer_t->stImage[buffer_t->clearIdx].buffer_unique_id)
			buffer_t->last_cleared_buff_id = buffer_t->stImage[buffer_t->clearIdx].buffer_unique_id;

		atomic_dec( &buffer_t->valid_buff_count);

	}while(buffer_t->readIdx != buffer_t->clearIdx);
	wake_up_interruptible( &wq_consume ) ;
	vprintk("tcc_vsync_clean_buffer valid_buff_count %d  \n", atomic_read(&buffer_t->valid_buff_count));
	
	return atomic_read(&buffer_t->valid_buff_count);
}

static int tcc_vsync_pop_all_buffer(tcc_vsync_buffer_t * buffer_t)
{

	if(atomic_read(&buffer_t->valid_buff_count) == 0)
	{
		vprintk("error: buffer empty \n");
		return -1;
	}

	while(atomic_read(&buffer_t->valid_buff_count))
	{
		if(buffer_t->last_cleared_buff_id < buffer_t->stImage[buffer_t->clearIdx].buffer_unique_id)
			buffer_t->last_cleared_buff_id = buffer_t->stImage[buffer_t->clearIdx].buffer_unique_id;

		if(++buffer_t->clearIdx >= buffer_t->max_buff_num)
			buffer_t->clearIdx = 0;

		atomic_dec( &buffer_t->valid_buff_count);
	}

	buffer_t->readIdx = buffer_t->clearIdx;
	atomic_set( &buffer_t->readable_buff_count,0);
	
	printk("pop_all_buffer readIdx %d writeIdx %d clearIdx %d valid %d \n", buffer_t->readIdx,buffer_t->writeIdx,buffer_t->clearIdx, atomic_read(&buffer_t->valid_buff_count));
	return 0;
}

static int tcc_vsync_is_full_buffer(tcc_vsync_buffer_t * buffer_t)
{
	if(atomic_read(&buffer_t->valid_buff_count) >= buffer_t->max_buff_num)
		return 1;
	else
		return 0;
}

inline static int tcc_vsync_is_empty_buffer(tcc_vsync_buffer_t * buffer_t)
{
	if(atomic_read(&buffer_t->valid_buff_count) > 0)
		return 0;
	else
		return 1;
}

static void tcc_vsync_check_interlace_output(tcc_video_disp *p, struct tcc_dp_device *pdp_data)
{
	VIOC_DISP * pDISPBase;
	
	pDISPBase = (VIOC_DISP*)pdp_data->ddc_info.virt_addr;
	
	if(pDISPBase->uCTRL.nREG & HwDISP_NI)
		p->interlace_output = 0;
	else
		p->interlace_output = 1;
}

static int tcc_vsync_bypass_frame(tcc_video_disp *p, struct tcc_dp_device *pdp_data, struct tcc_lcdc_image_update *pImage)
{
	VIOC_DISP * pDISPBase;
	unsigned int lstatus = 0;
	int ret = 0;
	
	pDISPBase = (VIOC_DISP*)pdp_data->ddc_info.virt_addr;
	
	lstatus = pDISPBase->uLSTATUS.nREG;
		
	if(pImage->odd_first_flag==0){
		if(p->nDeinterProcCount ==0){
			if(ISSET(lstatus, HwLSTATUS_EF))
			{
				ret = 1;
			}
		}
		else{
			if(!ISSET(lstatus, HwLSTATUS_EF))
			{
				ret = 1;
			}
		}
	}
	else{

		if(p->nDeinterProcCount ==1){
			if(ISSET(lstatus, HwLSTATUS_EF))
			{
				ret = 1;
			}
		}
		else{
			if(!ISSET(lstatus, HwLSTATUS_EF))
			{
				ret = 1;
			}
		}
	}
	
	if(ret == 1){
		//printk("nDeinterProcCount(%d), odd_first_flag(%d)\n", tccvid_vsync.nDeinterProcCount, pImage->odd_first_flag) ;
		return ret;
	}
	
	switch(Output_SelectMode)
	{
		case TCC_OUTPUT_NONE:
			break;
#ifdef TCC_LCD_VIDEO_DISPLAY_BY_VSYNC_INT
		case TCC_OUTPUT_LCD:
			tca_scale_display_update(pdp_data, pImage);
			break;
#endif
		case TCC_OUTPUT_HDMI:
			tca_scale_display_update(pdp_data, pImage);
			break;
		case TCC_OUTPUT_COMPONENT:
			#if defined(CONFIG_FB_TCC_COMPONENT)
				tcc_component_update(pImage);
			#endif
			break;
		case TCC_OUTPUT_COMPOSITE:
			#if defined(CONFIG_FB_TCC_COMPOSITE)
				tcc_composite_update(pImage);
			#endif
			break;
			
		default:
			break;
	}

	return ret;
}


static void tcc_vsync_display_update(tcc_video_disp *p)
{
	int current_time;
	int time_gap;
	int valid_buff_count;
	int i;
	struct tcc_lcdc_image_update *pNextImage;

	current_time = tcc_vsync_get_time(p);
	tcc_vsync_clean_buffer(&p->vsync_buffer);

	valid_buff_count = atomic_read( & p->vsync_buffer.readable_buff_count);
	valid_buff_count--;
	
	for(i=0; i < valid_buff_count; i++)
	{
		pNextImage = tcc_vsync_get_buffer(&p->vsync_buffer, 0);
		if (pNextImage == NULL) {
			pr_err("%s: we don't have vsync_buffer\n", __func__);
			return;
		}
		vprintk("pNextImage->time_stamp : %d %d\n", pNextImage->time_stamp,current_time) ;

		time_gap = (current_time+VIDEO_SYNC_MARGIN_EARLY) - pNextImage->time_stamp;
		if(time_gap >= 0)
		{
			if (tcc_vsync_pop_buffer(&p->vsync_buffer) < 0)
				return;

			if(p->perfect_vsync_flag == 1 && time_gap < 4 )
			{
				tcc_vsync_set_time(p, current_time + (p->vsync_interval>>1)*p->time_gap_sign);
				if(p->time_gap_sign > 0)
					p->time_gap_sign = -1;
				else
					p->time_gap_sign = 1;
			}
		}
		else
		{
			break;
		}
	}

	pNextImage = &p->vsync_buffer.stImage[p->vsync_buffer.readIdx];
	
#ifdef CONFIG_ANDROID
	if(p->outputMode == Output_SelectMode )
#else
	if(p->outputMode == Output_SelectMode && p->vsync_buffer.cur_lcdc_buff_addr != pNextImage->addr0)
#endif
	{
		struct fb_info *info = registered_fb[0];
		struct tccfb_info *tccfb_info = NULL;
		struct tcc_dp_device *dp_device = NULL;
		//printk("mt(%d), remain_buff(%d) readIdx(%d) write(%d) readaddr(%x) cleared_buff_id(%d) buffer_unique_id %d\n", pNextImage->time_stamp, p->vsync_buffer.readable_buff_count,
		//	p->vsync_buffer.readIdx,p->vsync_buffer.writeIdx,pNextImage->addr0,p->vsync_buffer.last_cleared_buff_id,p->vsync_buffer.stImage[p->vsync_buffer.readIdx].buffer_unique_id) ;
		p->vsync_buffer.cur_lcdc_buff_addr = pNextImage->addr0;

#if 0
		if(testToggleBit)
		{
			PGPION hwGPIOC;
			testToggleBit = 0; 

			hwGPIOC = (volatile PGPION)tcc_p2v(HwGPIOC_BASE);
			//hwGPIOC->GPEN.nREG|= (unsigned int)(0x00004000);
			//hwGPIOC->GPDAT.nREG |= (unsigned int)(0x00004000);
			hwGPIOC->GPFN1.bREG.GPFN14 = 0;
			hwGPIOC->GPEN.bREG.GP14 = 1;
			hwGPIOC->GPDAT.bREG.GP14 = 1;

			
		}
		else
		{
			PGPION hwGPIOC;
			testToggleBit = 1;

			hwGPIOC = (volatile PGPION)tcc_p2v(HwGPIOC_BASE);
			//hwGPIOC->GPEN.nREG |= (unsigned int)(0x00004000);
			//hwGPIOC->GPDAT.nREG &= (unsigned int)(~0x00004000);
			hwGPIOC->GPFN1.bREG.GPFN14 = 0;
			hwGPIOC->GPEN.bREG.GP14 = 1;
			hwGPIOC->GPDAT.bREG.GP14 = 0;

		}
#endif
		tccfb_info = info->par;
		if(tccfb_info->pdata.Mdp_data.DispDeviceType == Output_SelectMode)
			dp_device = &tccfb_info->pdata.Mdp_data;
		else if(tccfb_info->pdata.Sdp_data.DispDeviceType == Output_SelectMode)
			dp_device = &tccfb_info->pdata.Sdp_data;
		else {
			printk("%s: Main:%d, Sub :%d Select:%d \n", __func__,
				tccfb_info->pdata.Mdp_data.DispDeviceType,
				tccfb_info->pdata.Sdp_data.DispDeviceType,
				Output_SelectMode);
			return;
		}

		if (p->output_toMemory && p->m2m_mode) {
			TCC_VIQE_DI_Run60Hz_M2M(pNextImage, 0);
		} else {
			switch(Output_SelectMode)
			{
				case TCC_OUTPUT_NONE:
					break;

				#ifdef TCC_LCD_VIDEO_DISPLAY_BY_VSYNC_INT
				case TCC_OUTPUT_LCD:
					tca_scale_display_update(dp_device, pNextImage);
					break;
				#endif

				case TCC_OUTPUT_HDMI:
					{
						//if(tcc_vsync_mvc_status )
						if(tcc_vsync_mvc_status && pNextImage->MVCframeView == 1 && pNextImage->MVC_Base_addr0)
						{
							//printk("@@@@@tcc_vsync_mvc_status=%d pNextImage->MVCframeView = %d, pNextImage->MVC_Base_addr0 = 0x%08x\n",tcc_vsync_mvc_status, pNextImage->MVCframeView, pNextImage->MVC_Base_addr0);
							tca_mvc_display_update(EX_OUT_LCDC, pNextImage);
						}
						else
						{
							tca_scale_display_update(dp_device, pNextImage);
						}
					}

					break;
				case TCC_OUTPUT_COMPONENT:
					#if defined(CONFIG_FB_TCC_COMPONENT)
					tcc_component_update(pNextImage);
					#endif
					break;
				case TCC_OUTPUT_COMPOSITE:
					#if defined(CONFIG_FB_TCC_COMPOSITE)
					tcc_composite_update(pNextImage);
					#endif
					break;

				default:
					break;
			}
		}
	}
	return;
}

static void tcc_vsync_display_update_forDeinterlaced(tcc_video_disp *p)
{
	static int PrevImageWidth = 0, PrevImageHeight = 0;
	int reset_frmCnt = 0;
	int current_time;
	int time_gap, time_diff;
	int valid_buff_count;
	int i;
	int popped_count = 0;

	current_time = tcc_vsync_get_time(p);

	if(p->nDeinterProcCount == 0)
	{
		tcc_vsync_clean_buffer(&p->vsync_buffer);

		valid_buff_count = atomic_read( & p->vsync_buffer.readable_buff_count);
		valid_buff_count--;
		if( valid_buff_count < 0 ){
			dprintk_drop("%s: no available buffer(%d) \n", p->pIntlNextImage->Lcdc_layer == RDMA_VIDEO ? "M" : "S", valid_buff_count);
		}

		for(i=0; i < valid_buff_count; i++)
		{
			p->pIntlNextImage = tcc_vsync_get_buffer(&p->vsync_buffer, 0);
			if (p->pIntlNextImage == NULL) {
				pr_err("%s: we don't have vsync_buffer\n", __func__);
				return;
			}

			time_gap = (current_time+VIDEO_SYNC_MARGIN_EARLY) - p->pIntlNextImage->time_stamp;
			if(time_gap >= 0)
			{
				if (tcc_vsync_pop_buffer(&p->vsync_buffer) < 0){
					if(popped_count > 0){
						dprintk_drop("%s:0 skipped buffer(%d) and no display \n", p->pIntlNextImage->Lcdc_layer == RDMA_VIDEO ? "M" : "S", popped_count);
					}
					return;
				}

				popped_count++;

				if(p->perfect_vsync_flag == 1 && time_gap < 4 )
				{
					tcc_vsync_set_time(p, current_time + (p->vsync_interval>>1)*p->time_gap_sign);
					if(p->time_gap_sign > 0)
						p->time_gap_sign = -1;
					else
						p->time_gap_sign = 1;
				}
			}
			else
			{
				break;
			}
		}

		if(popped_count > 1){
			dprintk_drop("%s:1 skipped buffer(p:[%d]) r:[%d -> %d]) \n",
						p->pIntlNextImage->Lcdc_layer == RDMA_VIDEO ? "M" : "S", popped_count, valid_buff_count+1, tcc_video_get_valid_count(p));
		}

		p->pIntlNextImage = &p->vsync_buffer.stImage[p->vsync_buffer.readIdx];

		if(p->vsync_buffer.last_cleared_buff_id > p->pIntlNextImage->buffer_unique_id){
			dprintk_drop("%s:error wierd buffer id[%d >= %d]\n",
						p->pIntlNextImage->Lcdc_layer == RDMA_VIDEO ? "M" : "S", p->pIntlNextImage->buffer_unique_id, p->vsync_buffer.last_cleared_buff_id);
		}
//		printk("mt(%d), st(%d)\n", p->pIntlNextImage->time_stamp, current_time) ;
		if(p->pIntlNextImage->first_frame_after_seek){
			time_diff =  p->pIntlNextImage->time_stamp - (current_time+VIDEO_SYNC_MARGIN_EARLY);
			if(time_diff > 0 && abs(time_diff) > (p->nTimeGapToNextField*2) )
			{
				printk("frame need to wait (%d) (%d)\n",time_diff, p->nTimeGapToNextField*2);
				return;
			}

			p->pIntlNextImage->first_frame_after_seek = 0;
		}

		// resolution change support
		if((PrevImageWidth != 0) && (PrevImageHeight != 0)) {
			int image_width = p->pIntlNextImage->crop_right - ((p->pIntlNextImage->crop_left >> 3) << 3);
			int image_height = p->pIntlNextImage->crop_bottom - p->pIntlNextImage->crop_top;

			if((PrevImageWidth != image_width) || (PrevImageHeight != image_height)) {
				dprintk("Interlaced_Res_Changed ::  %d x %d -> %d x %d \n", PrevImageWidth, PrevImageHeight, image_width, image_height);
				PrevImageWidth = image_width;
				PrevImageHeight = image_height;

				reset_frmCnt = 1;	// SoC guide info.
			} else {
				reset_frmCnt = 0;
			}
		} else {
			PrevImageWidth = p->pIntlNextImage->crop_right - ((p->pIntlNextImage->crop_left >> 3) << 3);;
			PrevImageHeight = p->pIntlNextImage->crop_bottom - p->pIntlNextImage->crop_top;;
			reset_frmCnt = 0;
		}

		if(p->vsync_buffer.cur_lcdc_buff_addr != p->pIntlNextImage->addr0)
		{
			p->vsync_buffer.cur_lcdc_buff_addr = p->pIntlNextImage->addr0;

			if((Output_SelectMode != TCC_OUTPUT_NONE) && (p->outputMode == Output_SelectMode))
			{
				if(!p->interlace_bypass_lcdc)
				{
					if(p->output_toMemory && p->m2m_mode) {
						TCC_VIQE_DI_Run60Hz_M2M(p->pIntlNextImage, reset_frmCnt);
					} else {
						if(p->duplicateUseFlag) {
							viqe_render_field(current_time);
						} else {
							TCC_VIQE_DI_Run60Hz(p->pIntlNextImage,reset_frmCnt);
						}
					}
				}
				else
				{
					
					struct fb_info *info = registered_fb[0];
					struct tccfb_info *tccfb_info = NULL;
					struct tcc_dp_device *dp_device = NULL;
					tccfb_info = info->par;
					if(tccfb_info->pdata.Mdp_data.DispDeviceType == Output_SelectMode)
						dp_device = &tccfb_info->pdata.Mdp_data;
					else if(tccfb_info->pdata.Sdp_data.DispDeviceType == Output_SelectMode)
						dp_device = &tccfb_info->pdata.Sdp_data;
					else {
						printk("%s: Main:%d, Sub :%d Select:%d \n", __func__,
							tccfb_info->pdata.Mdp_data.DispDeviceType,
							tccfb_info->pdata.Sdp_data.DispDeviceType,
							Output_SelectMode);
						return;
					}

					if(tcc_vsync_bypass_frame(p, dp_device, p->pIntlNextImage) == 1){
						p->nDeinterProcCount =0;
						return;
					}
				}
			}

			if(p->output_toMemory && p->m2m_mode){
				if(p->deinterlace_mode){ // To prevent progressive contents from working twice on dual-vsync mode for PIP.
					p->nDeinterProcCount ++;
				}
			}
			else{
				p->nDeinterProcCount ++;
			}
		}
		else
		{
			if((Output_SelectMode != TCC_OUTPUT_NONE) && (p->outputMode == Output_SelectMode))
			{
				if (!p->output_toMemory && !p->m2m_mode) {
					if(!p->interlace_bypass_lcdc)
					{
						if(p->duplicateUseFlag)
							viqe_render_field(current_time);
					}
					else
					{

						struct fb_info *info = registered_fb[0];
						struct tccfb_info *tccfb_info = NULL;
						struct tcc_dp_device *dp_device = NULL;
						tccfb_info = info->par;
						if(tccfb_info->pdata.Mdp_data.DispDeviceType == Output_SelectMode)
							dp_device = &tccfb_info->pdata.Mdp_data;
						else if(tccfb_info->pdata.Sdp_data.DispDeviceType == Output_SelectMode)
							dp_device = &tccfb_info->pdata.Sdp_data;
						else {
							printk("%s: Main:%d, Sub :%d Select:%d \n", __func__,
								tccfb_info->pdata.Mdp_data.DispDeviceType,
								tccfb_info->pdata.Sdp_data.DispDeviceType,
								Output_SelectMode);
							return;
						}

						if(tcc_vsync_bypass_frame(p, dp_device, p->pIntlNextImage) == 1){
							p->nDeinterProcCount =0;
							return;
						}
					}
				}
			}
		}
	}
	else
	{
		if((Output_SelectMode != TCC_OUTPUT_NONE) && (p->outputMode == Output_SelectMode) )
		{
			if(!p->interlace_bypass_lcdc)
			{
				if(p->output_toMemory && p->m2m_mode) {
					p->pIntlNextImage->odd_first_flag = !p->pIntlNextImage->odd_first_flag;
					//p->pIntlNextImage->dst_addr0 = p->pIntlNextImage->dst_addr1;
					TCC_VIQE_DI_Run60Hz_M2M(p->pIntlNextImage, 0);
				} else {
					if(p->duplicateUseFlag){
						viqe_render_field(current_time);
					} else {
						p->pIntlNextImage->odd_first_flag = !p->pIntlNextImage->odd_first_flag;
						TCC_VIQE_DI_Run60Hz(p->pIntlNextImage,0);
					}
				}
			}
		}

		p->nDeinterProcCount = 0;
	}
	return;
}

int display_vsync(tcc_video_disp *p)
{
#ifdef USE_SOFT_IRQ_FOR_VSYNC
	if (schedule_work(&vsync_work_q) == 0 ) {
		printk("vsync error:cannot schedule work !!!\n");
	}
#else

#ifndef USE_VSYNC_TIMER
   if((++p->unVsyncCnt) &0x01)
		   p->baseTime += 16;
   else
		   p->baseTime += 17;

   if(!(p->unVsyncCnt%6))
		   p->baseTime++;
#endif

	if( tcc_vsync_is_empty_buffer(&p->vsync_buffer) == 0)
	{
	#ifdef TCC_VIDEO_DISPLAY_DEINTERLACE_MODE
		if(p->deinterlace_mode && !p->output_toMemory)
			tcc_vsync_display_update_forDeinterlaced(p);
		//else if(p->deinterlace_mode && p->output_toMemory && p->m2m_mode)
		else if(p->output_toMemory && p->m2m_mode)
			tcc_vsync_display_update_forDeinterlaced(p);
		else
	#endif
			tcc_vsync_display_update(p);

	#if defined(CONFIG_TCC_DISPLAY_MODE_USE) && !defined(CONFIG_TCC_DISPLAY_MODE_AUTO_DETECT)
		tca_fb_attach_update_flag();
	#endif
	}
	else
	{
		if( p->isVsyncRunning && (0 >= tcc_video_get_valid_count(p)))
			return -1;
	}

	return 0;
#endif	
}

static irqreturn_t tcc_vsync_handler_for_video(int irq, void *dev_id)
{
	struct tcc_vsync_display_info_t *vsync = (struct tcc_vsync_display_info_t *)dev_id;

	if (!is_vioc_intr_activatied(vsync->vioc_intr->id, vsync->vioc_intr->bits)) {
		return IRQ_NONE;
	}

	vioc_intr_clear(vsync->vioc_intr->id, vsync->vioc_intr->bits);

#if defined(CONFIG_HDMI_DISPLAY_LASTFRAME)
	if(LastFrame)
		return IRQ_HANDLED;
#endif

	display_vsync(&tccvid_vsync[0]);
	display_vsync(&tccvid_vsync[1]);

	return IRQ_HANDLED; 
}

inline static void tcc_vsync_set_time(tcc_video_disp *p, int time)
{
#ifdef USE_VSYNC_TIMER
	p->baseTime = tcc_get_timer_count(vsync_timer) - time;
#else
	p->baseTime = time;
#endif
}
	
inline static int tcc_vsync_get_time(tcc_video_disp *p)
{
#ifdef USE_VSYNC_TIMER
	return tcc_get_timer_count(vsync_timer) - p->baseTime;
#else
	return p->baseTime;
#endif

}

static void tcc_vsync_reset_syncTime(tcc_video_disp *p, int currentTime)
{
	memset(p->timeGap, 0x00, sizeof(p->timeGap));
	p->timeGapBufferFullFlag = 0;
	p->timeGapIdx = 0;
	p->timeGapTotal = 0;
	
	spin_lock_irq(&vsync_lock) ;
	tcc_vsync_set_time(p, currentTime);
	p->unVsyncCnt=0;
	spin_unlock_irq(&vsync_lock) ;

	printk("reset base time : %d\n",currentTime);
}

static int tcc_vsync_calculate_syncTime(tcc_video_disp *p, int currentTime)
{
	int diffTime;
	int avgTime;
	int base_time;

	if(p->lastUdateTime == currentTime)
		return 0;

	p->lastUdateTime= currentTime;

	spin_lock_irq(&vsync_lock) ;
	base_time = tcc_vsync_get_time(p);
	spin_unlock_irq(&vsync_lock) ;
	
	diffTime =	currentTime - base_time;

	p->timeGapTotal -= p->timeGap[p->timeGapIdx];
	p->timeGapTotal += diffTime;
	p->timeGap[p->timeGapIdx++] = diffTime;

	if(p->timeGapIdx >= TIME_BUFFER_COUNT)
		p->timeGapIdx = 0;

	if(p->timeGapIdx == 0)
		p->timeGapBufferFullFlag = 1;

	if(p->timeGapBufferFullFlag)
		avgTime = p->timeGapTotal / TIME_BUFFER_COUNT;
	else
		avgTime = p->timeGapTotal / (int)(p->timeGapIdx);


	vprintk("diffTime %d, avgTime %d, base : %d\n", diffTime, avgTime, base_time);

	#if 0//def USE_VSYNC_TIMER
	if( p->timeGapBufferFullFlag && (avgTime > p->updateGapTime || avgTime < -(p->updateGapTime)))
	#else
	if( (p->timeGapBufferFullFlag || p->unVsyncCnt < 100) 
		&& (avgTime > p->updateGapTime || avgTime < -(p->updateGapTime)))
	#endif
	{
		memset(p->timeGap, 0x00, sizeof(p->timeGap));
		p->timeGapBufferFullFlag = 0;
		p->timeGapIdx = 0;
		p->timeGapTotal = 0;
		
		//printk("changed time base time %d kernel time %d time %d \n",
		//	p->baseTime,tcc_get_timer_count(vsync_timer),currentTime);
		spin_lock_irq(&vsync_lock) ;
		tcc_vsync_set_time(p, base_time+avgTime);
		spin_unlock_irq(&vsync_lock) ;

		printk("changed base time : %d, add time: %d diffTime %d \n",base_time+avgTime, avgTime,diffTime);
	}
	
	return 0;
}


static int tcc_vsync_push_preprocess(tcc_video_disp *p, struct tcc_dp_device *pdp_data, struct tcc_lcdc_image_update *input_image_info)
{
	unsigned int lcd_width, lcd_height;
	VIOC_DISP * pDISPBase;
	int check_time;
	int display_hz=0;

	if(pdp_data == NULL)
		return -1;

	pDISPBase = (VIOC_DISP*)pdp_data->ddc_info.virt_addr;
	VIOC_DISP_GetSize(pDISPBase, &lcd_width, &lcd_height);

	if(!lcd_width || !lcd_height || !VIOC_DISP_Get_TurnOnOff(pDISPBase) || !p->isVsyncRunning)
	{
		spin_lock_irq(&vsync_lockDisp);
		p->vsync_buffer.last_cleared_buff_id = input_image_info->buffer_unique_id;
		//printk("%s : %d %d %d %d \n",__func__,lcd_width,lcd_height,VIOC_DISP_Get_TurnOnOff(pDISPBase) ,p->isVsyncRunning);
		spin_unlock_irq(&vsync_lockDisp);
		return -1;
	}

	if(tcc_video_check_framerate(p, input_image_info->private_data.optional_info[8]))
	{
		if(Output_SelectMode == TCC_OUTPUT_HDMI && HDMI_video_hz != 0)
			display_hz = HDMI_video_hz;
		else
			display_hz = 60;

		p->vsync_interval = (1000/display_hz);

		if( (p->video_frame_rate > 0) && (display_hz >= p->video_frame_rate)  && ((display_hz % p->video_frame_rate) == 0))
			p->perfect_vsync_flag = 1;
		else
			p->perfect_vsync_flag = 0;

		if( p->video_frame_rate < display_hz/2)
			p->duplicateUseFlag = 1;
		else p->duplicateUseFlag = 0;

		printk("### Change_Frame_Rate with ex_out(%d) ,interval(%d), perfect_flag(%d) duplicate(%d) \n", input_image_info->ex_output
			, p->vsync_interval, p->perfect_vsync_flag,p->duplicateUseFlag);
	}

	#if defined(CONFIG_TCC_M2M_USE_INTERLACE_OUTPUT)
	if( !(pDISPBase->uCTRL.nREG & HwDISP_NI )) {//interlace mode
		input_image_info->on_the_fly = 0;
	}
	#endif

#if defined(CONFIG_HDMI_DISPLAY_LASTFRAME)
	if(0 >= tcc_video_check_last_frame(input_image_info)){
		printk("----> skip this frame for last-frame \n");
		return 0;
	}
#endif

	spin_lock_irq(&vsync_lock) ;
	check_time = abs(tcc_vsync_get_time(p) - input_image_info->sync_time);
	spin_unlock_irq(&vsync_lock) ;

	if(check_time> 200)
	{
#ifdef USE_VSYNC_TIMER
		vprintk("reset time base time %d kernel time %ld time %d \n",
		p->baseTime,tcc_get_timer_count(vsync_timer),input_image_info->sync_time);
#endif
		tcc_vsync_reset_syncTime(p, 0);
	}
	
	tcc_vsync_calculate_syncTime(p, input_image_info->sync_time);

	return 0;
}

static void tcc_vsync_push_set_outputMode(tcc_video_disp *p, struct tcc_dp_device *pdp_data, struct tcc_lcdc_image_update *input_image_info)
	{

		switch(input_image_info->outputMode)
		{
			case OUTPUT_NONE:
				#ifdef TCC_LCD_VIDEO_DISPLAY_BY_VSYNC_INT
				p->outputMode = TCC_OUTPUT_LCD;
				#else
				p->outputMode = TCC_OUTPUT_NONE;			
				#endif
				break;
			case OUTPUT_HDMI:
				p->outputMode = TCC_OUTPUT_HDMI;
				tcc_vsync_check_interlace_output(p, pdp_data);
				break;
			case OUTPUT_COMPOSITE:
				p->outputMode = TCC_OUTPUT_COMPOSITE; 		
				tcc_vsync_check_interlace_output(p, pdp_data);
				break;
			case OUTPUT_COMPONENT:
				p->outputMode = TCC_OUTPUT_COMPONENT; 		
				tcc_vsync_check_interlace_output(p, pdp_data);
				break;
			default:
				p->outputMode = TCC_OUTPUT_NONE;									
		}
		
		printk("set_outputMode input->outputMode:%d SelectMode:%d \n", input_image_info->outputMode,Output_SelectMode);			

	return;
}

static int tcc_vsync_push_preprocess_deinterlacing(tcc_video_disp *p, struct tcc_dp_device *pdp_data, struct tcc_lcdc_image_update *input_image_info)
{
	unsigned int lcdCtrlNum, lcd_width, lcd_height;
	VIOC_DISP * pDISPBase;

	if(p->deinterlace_mode < 0)
	{
		
		pDISPBase = (VIOC_DISP*)pdp_data->ddc_info.virt_addr;
				
		VIOC_DISP_GetSize(pDISPBase, &lcd_width, &lcd_height);
		
		p->deinterlace_mode = input_image_info->deinterlace_mode;
		p->frameInfo_interlace = input_image_info->frameInfo_interlace;
		p->output_toMemory = input_image_info->output_toMemory;
		printk("### deinterlace_mode(%d), output_toMemory(%d)\n", p->deinterlace_mode, p->output_toMemory);
		
		p->interlace_bypass_lcdc = 0;
		p->mvcMode = input_image_info->MVCframeView;

		p->m2m_mode =  input_image_info->m2m_mode;
	
		if( (p->deinterlace_mode == 1) && 
			(p->interlace_output == 1) && 
			(p->output_toMemory == 0) &&
			(input_image_info->Frame_width == input_image_info->Image_width) && (input_image_info->Frame_height == input_image_info->Image_height) )
		{
			printk("### interlace_bypass_lcdc set !!\n");
			p->interlace_bypass_lcdc = 1;
		}

		#if defined(CONFIG_TCC_OUTPUT_COLOR_SPACE_YUV) || defined(CONFIG_TCC_COMPOSITE_COLOR_SPACE_YUV)
		if( (p->outputMode == TCC_OUTPUT_COMPOSITE) && 
			(p->deinterlace_mode == 1) && 
			(p->interlace_output == 1) && 
			(p->output_toMemory == 0) &&
			(input_image_info->Frame_width == input_image_info->Image_width) )
		{
			if(p->interlace_bypass_lcdc == 0)
			{
				printk("### interlace_bypass_lcdc set for testing composite signal\n");
				p->interlace_bypass_lcdc = 1;
			}
		}

		#if defined(CONFIG_FB_TCC_COMPOSITE)
		tcc_composite_set_bypass(p->interlace_bypass_lcdc);
		#endif
		#endif
	}


	
	#if defined(CONFIG_TCC_OUTPUT_COLOR_SPACE_YUV) || defined(CONFIG_TCC_COMPOSITE_COLOR_SPACE_YUV)
	if((p->outputMode == TCC_OUTPUT_COMPOSITE) && (p->interlace_bypass_lcdc))
	{
		if(input_image_info->Frame_width <= input_image_info->Image_width)
			input_image_info->Image_width = input_image_info->Frame_width;
	}
	#endif
	
	if(p->deinterlace_mode && p->firstFrameFlag &&!p->interlace_bypass_lcdc)
	{
		if(p->outputMode == TCC_OUTPUT_LCD ||p->outputMode == TCC_OUTPUT_NONE )
			lcdCtrlNum = LCD_LCDC_NUM;
		else
			lcdCtrlNum = EX_OUT_LCDC;	

		p->nDeinterProcCount = 0;

		if(!p->output_toMemory)
		{
			printk("first on-the-fly TCC_excuteVIQE_60Hz \n") ;
			if (input_image_info->Lcdc_layer == RDMA_VIDEO)
				TCC_VIQE_DI_Init60Hz(Output_SelectMode, lcdCtrlNum, input_image_info);
		}
	}

	if (p->firstFrameFlag &&!p->interlace_bypass_lcdc) {
		if(p->output_toMemory && p->m2m_mode)
		{
			printk("first m2m TCC_excuteVIQE_60Hz \n") ;
			TCC_VIQE_DI_Init60Hz_M2M(Output_SelectMode, input_image_info);
		}
	}

	p->firstFrameFlag = 0;

	return 0;
}

static int tcc_vsync_push_bypass_frame(tcc_video_disp *p, struct tcc_dp_device *pdp_data, struct tcc_lcdc_image_update *input_image_info)
{
	
	if(input_image_info->output_path)
	{
		//printk("### output_path buffer_unique_id %d \n",input_image_info->buffer_unique_id);
		spin_lock_irq(&vsync_lockDisp) ;
		tcc_vsync_pop_all_buffer(&p->vsync_buffer);
		p->vsync_buffer.last_cleared_buff_id = input_image_info->buffer_unique_id;
		spin_unlock_irq(&vsync_lockDisp) ;

		if((p->deinterlace_mode && !p->interlace_bypass_lcdc) || p->m2m_mode)
		{
			p->nDeinterProcCount = 0;
			
			if(Output_SelectMode != TCC_OUTPUT_NONE  && !p->duplicateUseFlag)
			{
				if(!p->output_toMemory) {
					if (input_image_info->Lcdc_layer == RDMA_VIDEO)
						TCC_VIQE_DI_Run60Hz(input_image_info,0);
				}
				else if(p->output_toMemory && p->m2m_mode)
					TCC_VIQE_DI_Run60Hz_M2M(input_image_info,0);
			}
		}
		else if(p->interlace_bypass_lcdc){
			p->nDeinterProcCount =0;
			if(tcc_vsync_bypass_frame(p, pdp_data,input_image_info) == 1){
				//p->nDeinterProcCount =0;
				//return;
			}
		}
		else
		{
			switch(Output_SelectMode)
			{
				case TCC_OUTPUT_NONE:
					break;
				#ifdef TCC_LCD_VIDEO_DISPLAY_BY_VSYNC_INT
				case TCC_OUTPUT_LCD:
					tca_scale_display_update(pdp_data, input_image_info);
					break;
				#endif
				case TCC_OUTPUT_HDMI:
					//if(tcc_vsync_mvc_status )
					//printk("#####tcc_vsync_mvc_status=%d input_image_info->MVCframeView = %d, input_image_info->MVC_Base_addr0 = 0x%08x\n",tcc_vsync_mvc_status, input_image_info->MVCframeView, input_image_info->MVC_Base_addr0);

					if(tcc_vsync_mvc_status && input_image_info->MVCframeView == 1 && input_image_info->MVC_Base_addr0)
						tca_mvc_display_update(EX_OUT_LCDC, input_image_info);
					else
						tca_scale_display_update(pdp_data, input_image_info);
					break;
					
				case TCC_OUTPUT_COMPONENT:
					#if defined(CONFIG_FB_TCC_COMPONENT)
					tcc_component_update(input_image_info);
					#endif
					break;
				case TCC_OUTPUT_COMPOSITE:
					#if defined(CONFIG_FB_TCC_COMPOSITE)
					tcc_composite_update(input_image_info);
					#endif
					break;
				default:
					break;
			}
		}

		input_image_info->time_stamp = 0;
		
		return 1;
	}

	return 0;
}

static int tcc_vsync_push_check_error(tcc_video_disp *p, struct tcc_dp_device *pdp_data, struct tcc_lcdc_image_update *input_image_info)
{
	
	int error_type = 0;
	
	if(p->skipFrameStatus)
	{
		error_type = 1;
	}
	else if(p->outputMode != Output_SelectMode)
	{
		vprintk("vsync push error : output mode different %d %d \n", p->outputMode ,Output_SelectMode);			
		error_type = 2;
	}
	else if(p->vsync_buffer.available_buffer_id_on_vpu > input_image_info->buffer_unique_id)
	{
		vprintk("vsync push error : buffer index sync fail omx_buf_id: %d, cur_buff_id: %d \n", 
				p->vsync_buffer.available_buffer_id_on_vpu, input_image_info->buffer_unique_id);
		error_type = 3;
	}
	else if(input_image_info->time_stamp < input_image_info->sync_time)
	{
		input_image_info->time_stamp = TIME_MARK_SKIP;
	}
	
	if(error_type > 0)
	{
		vprintk("vsync push error : %d buffer_unique_id %d\n", error_type,input_image_info->buffer_unique_id);
		spin_lock_irq(&vsync_lockDisp) ;
		p->vsync_buffer.last_cleared_buff_id = input_image_info->buffer_unique_id;
		spin_unlock_irq(&vsync_lockDisp) ;
		return -1;
	}

	return 0;
}

static int tcc_vsync_push_process(tcc_video_disp *p, struct tcc_dp_device *pdp_data, struct tcc_lcdc_image_update *input_image_info, int idx)
{
	if(tcc_vsync_is_full_buffer(&p->vsync_buffer))
	{
		int timeout = wait_event_interruptible_timeout( wq_consume,
										tcc_vsync_is_full_buffer(&p->vsync_buffer) == 0, 
										msecs_to_jiffies(500)) ;
		vprintk("mio push wait(%d ms) due to buffer-full \n", 500-jiffies_to_msecs(timeout));
	}
	
	spin_lock_irq(&vsync_lock) ;
	if(tcc_vsync_push_buffer(&p->vsync_buffer, input_image_info) < 0)
	{
		printk("critical error: vsync buffer full by fault buffer controll\n");
	}

	if((p->deinterlace_mode &&  !p->interlace_bypass_lcdc) && (!p->output_toMemory || p->m2m_mode))
	{
		int curTime = tcc_vsync_get_time(p);
	
		if(Output_SelectMode != TCC_OUTPUT_NONE) {
			if (input_image_info->Lcdc_layer == RDMA_VIDEO)
				viqe_render_frame(input_image_info, p->vsync_interval, curTime,0);
		}
		
	}
	
	spin_unlock_irq(&vsync_lock) ;
	//printk("OddFirst = %d, interlaced %d\n", input_image_info->odd_first_flag, input_image_info->deinterlace_mode);
	vprintk("vtime : %d, curtime : %d\n", input_image_info->time_stamp, input_image_info->sync_time) ;
	vprintk("%s: PUSH_%d buffer ID(%d) - TS(v[%d ms] / s[%d ms]) \n", input_image_info->Lcdc_layer == RDMA_VIDEO ? "M" : "S", idx,
					input_image_info->buffer_unique_id, input_image_info->time_stamp, input_image_info->sync_time);

	return 0;
}

static void tcc_vsync_start(tcc_video_disp *p, struct tcc_lcdc_image_update *input_image_info, int idx)
{
	int backup_time;
	int backup_frame_rate;
	int display_hz=0;
#ifdef TCC_LCD_VIDEO_DISPLAY_BY_VSYNC_INT
	if(!input_image_info->ex_output){
		vsync_started_device = LCD_START_VSYNC;
		Output_SelectMode = TCC_OUTPUT_LCD;
	}
	else{
		Output_SelectMode = TCC_OUTPUT_HDMI;
		vsync_started_device = HDMI_START_VSYNC;
	}
#endif
			
#if defined(CONFIG_HDMI_DISPLAY_LASTFRAME)
	spin_lock_irq(&LastFrame_lockDisp);
	if(LastFrame && LastFrame_for_ResChanged == 0 && LastFrame_for_CodecChanged == 0){
		LastFrame = 0;
		printk("----> LastFrame = 0 in TCC_LCDC_VIDEO_START_VSYNC \n");
	}
	spin_unlock_irq(&LastFrame_lockDisp);
#endif

	if(p->vsync_started == 0)
	{
		tcc_video_set_framerate(p, input_image_info->fps);
		
		backup_time = p->nTimeGapToNextField; 
		backup_frame_rate = p->video_frame_rate;
		memset( p, 0, sizeof( tcc_video_disp) ) ; 
		p->isVsyncRunning = 1;
		p->overlayUsedFlag = -1;
		p->outputMode = -1;
		p->firstFrameFlag = 1;
		p->deinterlace_mode= -1;
		p->m2m_mode = -1;
		p->output_toMemory = -1;
		p->mvcMode = 0;
		p->duplicateUseFlag =0;
		p->nTimeGapToNextField = backup_time;
		p->video_frame_rate = backup_frame_rate;
		p->time_gap_sign = 1;
		if (idx == 0)
			viqe_render_init();

		if(backup_time)
			p->updateGapTime = backup_time;
		else
			p->updateGapTime = 32;

		if(Output_SelectMode == TCC_OUTPUT_HDMI && HDMI_video_hz != 0)
			display_hz = HDMI_video_hz;
			else
			display_hz = 60;

		p->vsync_interval = (1000/display_hz);

		if( (p->video_frame_rate > 0) && (display_hz >= p->video_frame_rate)  && ((display_hz % p->video_frame_rate) == 0))
				p->perfect_vsync_flag = 1;
			else
				p->perfect_vsync_flag = 0;
			
		if( p->video_frame_rate < display_hz/2)
				p->duplicateUseFlag =1;

		
		//spin_lock_init(&vsync_lock);
		//spin_lock_init(&vsync_lockDisp);

		#if 0 //def USE_VSYNC_TIMER
		if (vsync_timer) {
			tcc_timer_enable(vsync_timer);
			msleep(0);
		}
		#endif

		spin_lock_irq(&vsync_lock) ;
		tcc_vsync_set_time(p, 0);
		spin_unlock_irq(&vsync_lock) ;
		printk("### START_VSYNC_%d with ex_out(%d) ,interval(%d), perfect_flag(%d) duplicate(%d) \n", idx, input_image_info->ex_output
			, p->vsync_interval, p->perfect_vsync_flag,p->duplicateUseFlag);

		tca_vsync_video_display_enable();
		tcc_vsync_set_max_buffer(&p->vsync_buffer, input_image_info->max_buffer);
		
		p->vsync_started = 1;
		tcc_vsync_set_output_mode(p, Output_SelectMode);
	} else {
		p->firstFrameFlag = 1;
	}
}

static void tcc_vsync_end(tcc_video_disp *p, int idx)
{
	printk("### END_VSYNC_%d started:%d SelectMode:%d\n", idx, p->vsync_started, Output_SelectMode);

	if(p->vsync_started == 1)
	{
		struct tcc_lcdc_image_update ImageInfo;
		struct fb_info *info = registered_fb[0];
		struct tccfb_info *tccfb_info = NULL;
		struct tcc_dp_device *dp_device = NULL;

		#if 0 //def USE_VSYNC_TIMER
		if (vsync_timer)
			tcc_timer_disable(vsync_timer);
		#endif

		spin_lock_irq(&vsync_lock) ;
		tcc_vsync_pop_all_buffer(&p->vsync_buffer);
		spin_unlock_irq(&vsync_lock) ;

		if(tcc_vsync_get_isVsyncRunning(idx) && !video_display_disable_check)
		{
			tca_vsync_video_display_disable();
			video_display_disable_check = 1;
		}

		p->skipFrameStatus = 1;
		p->nTimeGapToNextField = 0;
		p->isVsyncRunning = 0;

		
	#ifdef TCC_VIDEO_DISPLAY_DEINTERLACE_MODE
		if(p->deinterlace_mode &&!p->interlace_bypass_lcdc)
		{
			if(!p->output_toMemory) {
				if (idx == 0)
					TCC_VIQE_DI_DeInit60Hz();
			}
            else if(p->output_toMemory && p->m2m_mode) {
				if (idx == 0)
					TCC_VIQE_DI_DeInit60Hz_M2M(RDMA_VIDEO);
				else if (idx == 1)
					TCC_VIQE_DI_DeInit60Hz_M2M(RDMA_VIDEO_SUB);
			}
		}
		else if(p->output_toMemory && p->m2m_mode)
		{
			if (idx == 0)
				TCC_VIQE_DI_DeInit60Hz_M2M(RDMA_VIDEO);
			else if (idx == 1)
				TCC_VIQE_DI_DeInit60Hz_M2M(RDMA_VIDEO_SUB);
		}
	#endif

		memset(&ImageInfo, 0x00, sizeof(struct tcc_lcdc_image_update));
		ImageInfo.enable = 0;
		if (idx == 0)
			ImageInfo.Lcdc_layer = RDMA_VIDEO;
		else if (idx == 1)
			ImageInfo.Lcdc_layer = RDMA_VIDEO_SUB;

		if(Output_SelectMode != TCC_OUTPUT_NONE) {
			tccfb_info = info->par;
			if(tccfb_info->pdata.Mdp_data.DispDeviceType == Output_SelectMode)
				dp_device = &tccfb_info->pdata.Mdp_data;
			else if(tccfb_info->pdata.Sdp_data.DispDeviceType == Output_SelectMode)
				dp_device = &tccfb_info->pdata.Sdp_data;
			else {
				printk("%s: Main:%d, Sub :%d Select:%d \n", __func__,
					tccfb_info->pdata.Mdp_data.DispDeviceType,
					tccfb_info->pdata.Sdp_data.DispDeviceType,
					Output_SelectMode);
				goto err_exit;
			}

			if (dp_device != NULL && Output_SelectMode == TCC_OUTPUT_HDMI) {
				if(tcc_vsync_mvc_status && p->mvcMode==1) {
					tca_mvc_display_update(EX_OUT_LCDC, (struct tcc_lcdc_image_update *)&ImageInfo);
				} else {
					tca_lcdc_set_onthefly(dp_device, (struct tcc_lcdc_image_update *)&ImageInfo);
				}

				if(tcc_vsync_mvc_overlay_priority) {
					printk(" @@@@ %s: tcc_vsync_mvc_overlay_priority %d \n",__func__,tcc_vsync_mvc_overlay_priority);

					VIOC_WMIX_SetOverlayPriority(dp_device->wmixer_info.virt_addr, tcc_vsync_mvc_overlay_priority);		//restore
					VIOC_WMIX_SetUpdate(dp_device->wmixer_info.virt_addr);
					tcc_vsync_mvc_overlay_priority = 0;
				}
			}
			else if(Output_SelectMode == TCC_OUTPUT_COMPOSITE){
				#if defined(CONFIG_FB_TCC_COMPOSITE)
				tcc_composite_update((struct tcc_lcdc_image_update *)&ImageInfo);
				#endif
			}
			else if(Output_SelectMode == TCC_OUTPUT_COMPONENT){
				#if defined(CONFIG_FB_TCC_COMPONENT)
				tcc_component_update((struct tcc_lcdc_image_update *)&ImageInfo);
				#endif
			}
			#ifdef TCC_LCD_VIDEO_DISPLAY_BY_VSYNC_INT
			else if(Output_SelectMode == TCC_OUTPUT_LCD){
				tca_lcdc_set_onthefly(dp_device, (struct tcc_lcdc_image_update *)&ImageInfo);
			}
			#endif
		}

err_exit:
		p->deinterlace_mode = 0;
		p->vsync_started = 0;
	}

#if defined(CONFIG_HDMI_DISPLAY_LASTFRAME)
	if(!enabled_LastFrame)
		Last_ImageInfo.enable = 0;

	CurrDisplayingImage.enable = 0;
#endif
	tcc_ctrl_ext_frame(0);

}

#if  defined(CONFIG_HDMI_DISPLAY_LASTFRAME) || defined(CONFIG_VIDEO_DISPLAY_SWAP_VPU_FRAME)
int tcc_move_video_frame_simple( struct file *file, struct tcc_lcdc_image_update* inFframeInfo, WMIXER_INFO_TYPE* WmixerInfo, unsigned int target_addr, unsigned int target_format)
{
	int ret = 0;
	
	memset(WmixerInfo, 0x00, sizeof(WmixerInfo));

	WmixerInfo->rsp_type		= WMIXER_POLLING;

	//source info
	WmixerInfo->src_fmt			= inFframeInfo->fmt;	
	WmixerInfo->src_img_width 	= inFframeInfo->Frame_width;
	WmixerInfo->src_img_height	= inFframeInfo->Frame_height;
	WmixerInfo->src_win_left	= 0;
	WmixerInfo->src_win_top		= 0;
	WmixerInfo->src_win_right	= inFframeInfo->Frame_width;
	WmixerInfo->src_win_bottom	= inFframeInfo->Frame_height;
	WmixerInfo->src_y_addr		= (unsigned int)inFframeInfo->addr0;
	WmixerInfo->src_u_addr		= (unsigned int)inFframeInfo->addr1;
	WmixerInfo->src_v_addr		= (unsigned int)inFframeInfo->addr2;

	//destination info
	WmixerInfo->dst_fmt			= target_format;
    WmixerInfo->dst_img_width   = inFframeInfo->Frame_width;
	WmixerInfo->dst_img_height  = inFframeInfo->Frame_height;
	WmixerInfo->dst_win_left	= 0;
	WmixerInfo->dst_win_top		= 0;
	WmixerInfo->dst_win_right	= inFframeInfo->Frame_width;
	WmixerInfo->dst_win_bottom	= inFframeInfo->Frame_height;
	WmixerInfo->dst_y_addr		= target_addr;
	WmixerInfo->dst_u_addr		= (unsigned int)GET_ADDR_YUV42X_spU(WmixerInfo->dst_y_addr, WmixerInfo->dst_img_width, WmixerInfo->dst_img_height);
	WmixerInfo->dst_v_addr		= (unsigned int)GET_ADDR_YUV422_spV(WmixerInfo->dst_u_addr, WmixerInfo->dst_img_width, WmixerInfo->dst_img_height);

	ret = file->f_op->unlocked_ioctl(file, TCC_WMIXER_IOCTRL_KERNEL, (unsigned long)WmixerInfo);

	printk("### tcc_move_video_frame_simple pre-processing(%dx%d - 0x%x) \n", inFframeInfo->Frame_width, inFframeInfo->Frame_height, inFframeInfo->addr0);

	return ret;
}
#endif

#if defined(CONFIG_HDMI_DISPLAY_LASTFRAME)
int tcc_video_check_last_frame(struct tcc_lcdc_image_update *ImageInfo)
{
	spin_lock_irq(&LastFrame_lockDisp);
	if(LastFrame)
	{
		if( LastFrame_for_ResChanged && (Last_ImageInfo.Frame_width == ImageInfo->Frame_width && Last_ImageInfo.Frame_height == ImageInfo->Frame_height) )
		{
			spin_unlock_irq(&LastFrame_lockDisp);
			return 0;
		}
		if( LastFrame_for_CodecChanged && (Last_ImageInfo.codec_id == ImageInfo->codec_id) )
		{
			spin_unlock_irq(&LastFrame_lockDisp);
			return 0;
		}
		printk("----> PUSH_VSYNC :: last-frame cleared : %dx%d, 0x%x \n", ImageInfo->Frame_width, ImageInfo->Frame_height, ImageInfo->addr0);
		LastFrame = LastFrame_for_ResChanged = LastFrame_for_CodecChanged = 0;
	}
	spin_unlock_irq(&LastFrame_lockDisp);

	memcpy(&Last_ImageInfo, ImageInfo, sizeof(struct tcc_lcdc_image_update));

	return 1;
}

int tcc_move_video_last_frame(struct tcc_lcdc_image_update* lastUpdated, char bInterlaced, unsigned int dest_addr, int dest_fmt)
{
	struct file    *file;
	int ret = 0;

#ifdef USE_SCALER2_FOR_LASTFRAME
	{
		SCALER_TYPE LFScaler;

		memset(&LFScaler, 0x00, sizeof(LFScaler));

		LFScaler.responsetype	= SCALER_POLLING;

		LFScaler.src_ImgWidth   = lastUpdated->Frame_width;
		LFScaler.src_ImgHeight  = lastUpdated->Frame_height;

		LFScaler.src_winLeft    = lastUpdated->crop_left;
		LFScaler.src_winTop     = lastUpdated->crop_top;
		LFScaler.src_winRight   = lastUpdated->crop_right;
		LFScaler.src_winBottom 	= lastUpdated->crop_bottom;

		//source info
		LFScaler.src_Yaddr      = (char*)lastUpdated->addr0;
		LFScaler.src_Uaddr      = (char*)lastUpdated->addr1;
		LFScaler.src_Vaddr      = (char*)lastUpdated->addr2;
		LFScaler.src_fmt 	   	= lastUpdated->fmt;

		//destination info
		LFScaler.dest_ImgWidth   = lastUpdated->Image_width;
		LFScaler.dest_ImgHeight  = lastUpdated->Image_height;
		LFScaler.dest_winLeft    = 0;
		LFScaler.dest_winTop     = 0;
		LFScaler.dest_winRight   = lastUpdated->Image_width;
		LFScaler.dest_winBottom  = lastUpdated->Image_height;
		
		if(bInterlaced > 0)
			LFScaler.interlaced= true;
		else
			LFScaler.interlaced = false;

		LFScaler.dest_Yaddr	    = (char*)dest_addr;
		LFScaler.dest_fmt       = dest_fmt;

		file = filp_open(SCALER_PATH, O_RDWR, 0666);
		ret = file->f_op->unlocked_ioctl(file, TCC_SCALER_IOCTRL_KERENL, (unsigned long)&fbmixer);
		filp_close(file, 0);
		if(ret <= 0){
			printk(" Error :: wmixer ctrl \n");
			return -100;
		}
		lastUpdated->addr0	 	= (unsigned int)LFScaler.dest_Yaddr;
	}
#else
	{
		WMIXER_ALPHASCALERING_INFO_TYPE fbmixer;
		#if defined(CONFIG_EXT_FRAME_VIDEO_EFFECT)
		WMIXER_VIOC_INFO wmixer_id_info;
		unsigned int lut_plugin, lut_en;
		#endif

		memset(&fbmixer, 0x00, sizeof(fbmixer));
		
		fbmixer.rsp_type        = WMIXER_POLLING;
		
		fbmixer.src_img_width   = lastUpdated->Frame_width;
		fbmixer.src_img_height  = lastUpdated->Frame_height;

		fbmixer.src_win_left    = lastUpdated->crop_left;
		fbmixer.src_win_top     = lastUpdated->crop_top;
		fbmixer.src_win_right   = lastUpdated->crop_right;
		fbmixer.src_win_bottom  = lastUpdated->crop_bottom;

		fbmixer.src_y_addr      = (unsigned int)lastUpdated->addr0;
		fbmixer.src_u_addr      = (unsigned int)lastUpdated->addr1;
		fbmixer.src_v_addr      = (unsigned int)lastUpdated->addr2;
		fbmixer.src_fmt 		= lastUpdated->fmt;

		fbmixer.dst_img_width   = lastUpdated->Image_width;
		fbmixer.dst_img_height  = lastUpdated->Image_height;
		fbmixer.dst_win_left    = 0;
		fbmixer.dst_win_top     = 0;
		fbmixer.dst_win_right   = lastUpdated->Image_width;
		fbmixer.dst_win_bottom  = lastUpdated->Image_height;

		if(bInterlaced > 0)
			fbmixer.interlaced= true;
		else
			fbmixer.interlaced = false;

		fbmixer.dst_y_addr      = (unsigned int)dest_addr;
		fbmixer.dst_fmt         = dest_fmt;

		file = filp_open(WMIXER_PATH, O_RDWR, 0666);
		if(file)
		{
			/* lut set to ExtFrame */
			unsigned int lut_plugin0, lut_en0;
			#if defined(CONFIG_EXT_FRAME_VIDEO_EFFECT)
			ret = file->f_op->unlocked_ioctl(file, TCC_WMIXER_VIOC_INFO_KERNEL, (unsigned long)&wmixer_id_info);

			lut_plugin = tcc_get_lut_plugin(TVC_LUT(LUT_COMP1));
			lut_en = tcc_get_lut_enable(TVC_LUT(LUT_COMP1));
			
			lut_plugin0 = tcc_get_lut_plugin(TVC_LUT(LUT_COMP0));
			lut_en0 = tcc_get_lut_enable(TVC_LUT(LUT_COMP0));
			tcc_set_lut_enable(TVC_LUT(LUT_COMP0), false);

			tcc_set_lut_enable(TVC_LUT(LUT_COMP1), false);
			
			tcc_set_lut_plugin(TVC_LUT(LUT_COMP1), TVC_RDMA(wmixer_id_info.rdma[0]));
			tcc_set_lut_enable(TVC_LUT(LUT_COMP1), 1);
			#endif//
			ret = file->f_op->unlocked_ioctl(file, TCC_WMIXER_ALPHA_SCALING_KERNEL, (unsigned long)&fbmixer);

			#if defined(CONFIG_EXT_FRAME_VIDEO_EFFECT)
			// lut set to before state.
			tcc_set_lut_enable(TVC_LUT(LUT_COMP1), lut_en);
			tcc_set_lut_plugin(TVC_LUT(LUT_COMP1), lut_plugin);

			tcc_set_lut_enable(TVC_LUT(LUT_COMP0), lut_en0);
			tcc_set_lut_plugin(TVC_LUT(LUT_COMP0), lut_plugin0);
			#endif//

			filp_close(file, 0);
		}
		else{
			ret = -1;
		}

		if(ret <= 0){
			printk(" Error :: wmixer ctrl \n");
			return -100;
		}

		lastUpdated->addr0       = (unsigned int)fbmixer.dst_y_addr;				
	}
#endif

	return 0;
}

int tcc_video_last_frame(tcc_video_disp *p, struct stTcc_last_frame iLastFrame, struct tcc_lcdc_image_update *lastUpdated, int idx)
{
	int ret = 0;
	struct fb_info *info = registered_fb[0];
	struct tccfb_info *tccfb_info = NULL;
	struct tcc_dp_device *dp_device = NULL;	

	if(!p)
	{
		p = &tccvid_vsync[idx];
	}

	if(tca_fb_divide_get_status())
	{
		printk("%s, Skip last frame because 3D UI is displayed!!\n", __func__);
		return 0;
	}

	if(tcc_vsync_mvc_status == 1)
	{
		dprintk("%s is returned because of mvc frame view\n", __func__);
		return 0;
	}

	memcpy(lastUpdated, &Last_ImageInfo, sizeof(struct tcc_lcdc_image_update));

	if(lastUpdated->enable == 0){
		printk("----> return last-frame :: The channel has already disabled. info(%dx%d), resolution_changed(%d), codec_changed(%d)\n",
					lastUpdated->Frame_width, lastUpdated->Frame_height, LastFrame_for_ResChanged, LastFrame_for_CodecChanged);
		return -100;
	}

	if(lastUpdated->outputMode != p->outputMode-TCC_OUTPUT_LCD){
		printk("----> return last-frame :: mode is different between %d and %d \n", lastUpdated->outputMode, p->outputMode-TCC_OUTPUT_LCD);
		return -100;
	}

	if(lastUpdated->Frame_width == 0 || lastUpdated->Frame_height == 0)
	{
		printk("Error :: no setting for Last-Frame \n");
		return -1;
	}

	if( (iLastFrame.codec_id != 0) && (lastUpdated->codec_id != iLastFrame.codec_id) )
	{
		printk("Error :: codec mismatch \n");
		return -1;
	}


	printk("----> TCC_LCDC_VIDEO_KEEP_LASTFRAME(%d) called with reason(%d), (Already LastFrame %d), output(%d)!! \n", enabled_LastFrame, iLastFrame.reason, LastFrame, p->outputMode);

	if(LastFrame){
		if(iLastFrame.reason&LASTFRAME_FOR_RESOLUTION_CHANGE)
			LastFrame_for_ResChanged = 1;
		else if(iLastFrame.reason&LASTFRAME_FOR_CODEC_CHANGE)
			LastFrame_for_CodecChanged = 1;
		printk("----> TCC_LCDC_VIDEO_KEEP_LASTFRAME returned due to (LastFrame %d)!! resolution_changed(%d), codec_changed(%d) \n", LastFrame, LastFrame_for_ResChanged, LastFrame_for_CodecChanged);
		return 0;
	}

	tccfb_info = info->par;
	if(tccfb_info->pdata.Mdp_data.DispDeviceType == p->outputMode)
		dp_device = &tccfb_info->pdata.Mdp_data;
	else if(tccfb_info->pdata.Sdp_data.DispDeviceType == p->outputMode)
		dp_device = &tccfb_info->pdata.Sdp_data;

	if(!dp_device)
	{
		printk("Error :: Display device is null.\n");
		return -1;
	}

	if(enabled_LastFrame)
	{
		if( p->outputMode == TCC_OUTPUT_HDMI || p->outputMode== TCC_OUTPUT_COMPOSITE || p->outputMode == TCC_OUTPUT_COMPONENT
#ifdef TCC_LCD_VIDEO_DISPLAY_BY_VSYNC_INT
				|| p->outputMode == TCC_OUTPUT_LCD 
#endif
		)
		{
			LastFrame = 1;
			LastFrame_for_ResChanged = iLastFrame.reason&0x1;
			LastFrame_for_CodecChanged = iLastFrame.reason&0x2;

			if(p->vsync_started == 1){
				spin_lock_irq(&vsync_lock);
				tcc_vsync_pop_all_buffer(&p->vsync_buffer);
				spin_unlock_irq(&vsync_lock);
			}

			if(CurrDisplayingImage.addr0 == fb_lastframe_pbuf.base){
				printk(" TCC_LCDC_VIDEO_KEEP_LASTFRAME :: called just after swap-frame \n");
				return 0;
			}

	#ifdef TCC_VIDEO_DISPLAY_BY_VSYNC_INT
				// this code prevent a error of transparent on layer0 after stopping v
			if(lastUpdated->Lcdc_layer == 0 && lastUpdated->enable == 0)
			{
				printk("Last-frame for a error prevention \n");
				spin_lock_irq(&vsync_lock);
				p->outputMode = -1;
				spin_unlock_irq(&vsync_lock);
			}
	#endif

	#if 1  // for PIP mode with dual-vsync!! // No need to display on M2M mode.
			if(p->deinterlace_mode)
			{
				if(!p->duplicateUseFlag)
				{
					if(p->output_toMemory && p->m2m_mode) {
						return 0;
					}
				}
			}
			else if(p->output_toMemory && p->m2m_mode)
			{
				return 0;
			}
	#endif

			if( (ret = tcc_move_video_last_frame(lastUpdated, p->deinterlace_mode, fb_lastframe_pbuf.base, TCC_LCDC_IMG_FMT_YUYV)) < 0 )
				return ret;

			lastUpdated->Frame_width  	= lastUpdated->Image_width;
			lastUpdated->Frame_height 	= lastUpdated->Image_height;	
			lastUpdated->fmt	 		= TCC_LCDC_IMG_FMT_YUYV; // W x H * 2 = Max 4Mb

			printk("### TCC_LCDC_VIDEO_KEEP_LASTFRAME RDMA(0x%p) :: Start info(%dx%d), Reason(0x%x) \n", dp_device->rdma_info[RDMA_LASTFRM].virt_addr,
						lastUpdated->Frame_width, lastUpdated->Frame_height, iLastFrame.reason);

			{
				lastUpdated->enable 		= 1;
				lastUpdated->on_the_fly		= 0;
				lastUpdated->one_field_only_interlace = 0;
				lastUpdated->crop_left 		= lastUpdated->crop_top = 0;
				lastUpdated->crop_right 	= lastUpdated->Frame_width;
				lastUpdated->crop_bottom 	= lastUpdated->Frame_height;			
				lastUpdated->Lcdc_layer 	= RDMA_LASTFRM;
				pLastFrame_RDMABase = dp_device->rdma_info[RDMA_LASTFRM].virt_addr;

				
				spin_lock_irq(&LastFrame_lockDisp);
				tca_scale_display_update(dp_device, lastUpdated);
				spin_unlock_irq(&LastFrame_lockDisp);
			}

	#ifdef TCC_VIDEO_DISPLAY_DEINTERLACE_MODE
			if(p->deinterlace_mode && !p->interlace_bypass_lcdc)
			{
				if(!p->output_toMemory)
				{
					if (idx==0)
						TCC_VIQE_DI_DeInit60Hz();
				}
			}
			else
	#endif
			{
				lastUpdated->enable 		= 0;
				if (idx == 0) {
					lastUpdated->Lcdc_layer 	= RDMA_VIDEO;
				} else {
					lastUpdated->Lcdc_layer 	= RDMA_VIDEO_SUB;
				}
				tcc_video_rdma_off(dp_device, lastUpdated, p->outputMode, 0);
			}

			printk("### TCC_LCDC_VIDEO_KEEP_LASTFRAME End \n");
		}
		else
		{
			printk("TCC_LCDC_VIDEO_KEEP_LASTFRAME skip :: reason(p->outputMode == %d)  \n", p->outputMode);
		}

		return 0;
	}
	else
	{
		if(p->outputMode == TCC_OUTPUT_HDMI ||p->outputMode == TCC_OUTPUT_COMPOSITE || p->outputMode == TCC_OUTPUT_COMPONENT 
	#ifdef TCC_LCD_VIDEO_DISPLAY_BY_VSYNC_INT
			|| p->outputMode ==  TCC_OUTPUT_LCD
	#endif
		)
		{
			WMIXER_INFO_TYPE WmixerInfo;

			LastFrame = 1;
			LastFrame_for_ResChanged = iLastFrame.reason&LASTFRAME_FOR_RESOLUTION_CHANGE;
			LastFrame_for_CodecChanged = iLastFrame.reason&LASTFRAME_FOR_CODEC_CHANGE ? 1:0;

			if(p->deinterlace_mode > 0 && p->vsync_started){
				if(!LastFrame_for_CodecChanged){
					printk("----> return last-frame :: deinterlace_mode %d. info(%dx%d), resolution_changed(%d), codec_changed(%d) \n", p->deinterlace_mode,
								Last_ImageInfo.Frame_width, Last_ImageInfo.Frame_height, LastFrame_for_ResChanged, LastFrame_for_CodecChanged);
					return -100;
				}
			}

			if(p->vsync_started == 1){
				spin_lock_irq(&vsync_lock);
				tcc_vsync_pop_all_buffer(&p->vsync_buffer);
				spin_unlock_irq(&vsync_lock);
			}

			if(LastFrame_for_CodecChanged
				|| (p->output_toMemory && p->m2m_mode) // for PIP mode with dual-vsync!!
			)
			{
				int bInterlaced = (p->deinterlace_mode && !p->output_toMemory && !p->interlace_bypass_lcdc);

				spin_lock_irq(&LastFrame_lockDisp);
				if (idx == 0) {
					lastUpdated->Lcdc_layer 	= RDMA_VIDEO;
				} else {
					lastUpdated->Lcdc_layer 	= RDMA_VIDEO_SUB;
				}
				tcc_video_rdma_off(dp_device, lastUpdated, p->outputMode, bInterlaced);
				spin_unlock_irq(&LastFrame_lockDisp);

				printk("---->  fake TCC_LCDC_VIDEO_KEEP_LASTFRAME to diable only RDMA channel. info(%dx%d), resolution_changed(%d), codec_changed(%d) \n",
								lastUpdated->Frame_width, lastUpdated->Frame_height, LastFrame_for_ResChanged, LastFrame_for_CodecChanged);
				return -100;
			}
			else
			{
				struct file *file;
				file = filp_open(WMIXER_PATH, O_RDWR, 0666);
				ret = tcc_move_video_frame_simple(file, lastUpdated, &WmixerInfo, fb_lastframe_pbuf.base, TCC_LCDC_IMG_FMT_YUYV);
				filp_close(file, 0);
				if(ret <= 0){
					printk(" Error :: wmixer ctrl \n");
					return -100;
				}

				lastUpdated->fmt = WmixerInfo.dst_fmt;
				lastUpdated->addr0 = WmixerInfo.dst_y_addr;
				lastUpdated->addr1 = WmixerInfo.dst_u_addr;
				lastUpdated->addr2 = WmixerInfo.dst_v_addr;
			}

			spin_lock_irq(&LastFrame_lockDisp);
			printk("----> fake TCC_LCDC_VIDEO_KEEP_LASTFRAME Start info(%dx%d), resolution_changed(%d), codec_changed(%d) \n", lastUpdated->Frame_width, lastUpdated->Frame_height, LastFrame_for_ResChanged, LastFrame_for_CodecChanged);

			switch(p->outputMode)
			{
				case TCC_OUTPUT_NONE:
					break;
			#ifdef TCC_LCD_VIDEO_DISPLAY_BY_VSYNC_INT
				case TCC_OUTPUT_LCD:
					tca_scale_display_update(dp_device, lastUpdated);
					break;
			#endif
				case TCC_OUTPUT_HDMI:
					tca_scale_display_update(dp_device, lastUpdated);
					break;
				case TCC_OUTPUT_COMPONENT:
			#if defined(CONFIG_FB_TCC_COMPONENT)
					tcc_component_update(lastUpdated);
			#endif
					break;
				case TCC_OUTPUT_COMPOSITE:
			#if defined(CONFIG_FB_TCC_COMPOSITE)
					tcc_composite_update(lastUpdated);
			#endif
					break;
				default:
					break;
			}

			printk("----> fake TCC_LCDC_VIDEO_KEEP_LASTFRAME End \n");
			spin_unlock_irq(&LastFrame_lockDisp);

			return -0x100;
		}
		else
		{
			printk("fake TCC_LCDC_VIDEO_KEEP_LASTFRAME skip :: reason(p->outputMode == %d)  \n", p->outputMode);
		}
	}

	return -1;
}
EXPORT_SYMBOL(tcc_video_last_frame);

void tcc_video_clear_last_frame(VIOC_RDMA *rdma, bool reset)
{
	if(tca_fb_divide_get_status())
	{
		//printk("%s, 3D UI(%s) is displayed!!\n", __func__);
		return;
	}

    if(enabled_LastFrame)
    {
		if(LastFrame)
			printk("----> lastframe cleared!! 0x%p \n", rdma);
        if( rdma != NULL )
            VIOC_RDMA_SetImageDisable(rdma);
    }

	if(reset){
		LastFrame = LastFrame_for_ResChanged = LastFrame_for_CodecChanged = 0;
		Last_ImageInfo.enable = 0;
	}
	
}

int tcc_video_ctrl_last_frame(int enable)
{	
	#if defined(CONFIG_HDMI_DISPLAY_LASTFRAME_STB_ENABLE)
	enabled_LastFrame = 1;
	#else
	enabled_LastFrame = enable;
	#endif

	return 0;
}

#ifdef CONFIG_DISPLAY_EXT_FRAME
int tcc_display_ext_frame(struct tcc_lcdc_image_update *lastUpdated, char bInterlaced, int output_mode)
{
	struct fb_info *info = registered_fb[0];
	struct tccfb_info *tccfb_info = NULL;
	struct tcc_dp_device *dp_device = NULL;
	int ret = 0;

	mutex_lock(&ext_mutex);

	if(lastUpdated->enable)
	{
		unsigned int dest_addr = 0x00;
		struct tcc_lcdc_image_update iImage;

		memcpy(&iImage, lastUpdated, sizeof(struct tcc_lcdc_image_update));

		if(iImage.Frame_width == 0 || iImage.Frame_height == 0)
		{
			printk("----> TCC_LCDC_VIDEO_CTRL_EXT_FRAME Error :: no setting for Ext-Frame \n");
			ret = -1;
			goto Error;
		}

		if(bCamera_mode)
		{
			printk("----> TCC_LCDC_VIDEO_CTRL_EXT_FRAME Info Cam(%d):: already enabled. \n", bCamera_mode);
			bExt_Frame = 1;
			ret = -1;
			goto Error;
		}


		tccfb_info = info->par;
		if(tccfb_info->pdata.Mdp_data.DispDeviceType == output_mode)
			dp_device = &tccfb_info->pdata.Mdp_data;
		else if(tccfb_info->pdata.Sdp_data.DispDeviceType == output_mode)
			dp_device = &tccfb_info->pdata.Sdp_data;

		if(!dp_device)
		{
			printk("Error :: Display device is null.\n");
			ret = -1;
			goto Error;
		}

		if(bToggleBuffer)
			dest_addr = fb_lastframe_pbuf.base;
		else
			dest_addr = fb_lastframe_pbuf.base + (fb_lastframe_pbuf.size/2);
		bToggleBuffer = ~bToggleBuffer;

		memcpy(&last_backup, &iImage, sizeof(struct tcc_lcdc_image_update));

		if( (ret = tcc_move_video_last_frame(&iImage, bInterlaced, dest_addr, TCC_LCDC_IMG_FMT_UYVY)) < 0 ){
			ret = -1;
			goto Error;
		}

		iImage.Frame_width  	= iImage.Image_width;
		iImage.Frame_height 	= iImage.Image_height;
		iImage.fmt	 		= TCC_LCDC_IMG_FMT_UYVY; // W x H * 2 = Max 4Mb

		printk("### TCC_LCDC_VIDEO_CTRL_EXT_FRAME Phy(0x%x) :: Start info(%dx%d, %d,%d ~ %dx%d), Display(%d,%d ~ %dx%d)\n", dest_addr,
					last_backup.Frame_width, last_backup.Frame_height,
					last_backup.crop_left, last_backup.crop_top, last_backup.crop_right, last_backup.crop_bottom,
					iImage.offset_x, iImage.offset_y, iImage.Image_width, iImage.Image_height);

		{
			iImage.enable 		= 1;
			iImage.on_the_fly	= 0;
			iImage.one_field_only_interlace = 0;
			iImage.crop_left 	= iImage.crop_top = 0;
			iImage.crop_right 	= iImage.Frame_width;
			iImage.crop_bottom 	= iImage.Frame_height;
			iImage.Lcdc_layer 	= RDMA_LASTFRM;
			pExtFrame_RDMABase  = dp_device->rdma_info[RDMA_LASTFRM].virt_addr;

			spin_lock_irq(&LastFrame_lockDisp);
			tca_scale_display_update(dp_device, &iImage);
			spin_unlock_irq(&LastFrame_lockDisp);

			bExt_Frame = 1;
		}
	}
	else
	{
		int alive = bExt_Frame;
		if(bExt_Frame)
		{
			if(!bCamera_mode)
			{
		        if( pExtFrame_RDMABase != NULL )
		            VIOC_RDMA_SetImageDisable(pExtFrame_RDMABase);
				printk("----> ext frame cleared!! 0x%p \n", pExtFrame_RDMABase);
			}
			last_backup.enable = bExt_Frame = 0;
		}

		ret = alive;
    }

Error:
	mutex_unlock(&ext_mutex);

	return ret;
}
#endif

void tcc_ext_mutex(char lock, char bCamera)
{
#ifdef CONFIG_DISPLAY_EXT_FRAME
	if(lock){
		mutex_lock(&ext_mutex);
	}
	else{
		mutex_unlock(&ext_mutex);
	}
	bCamera_mode = bCamera;
#endif
}
EXPORT_SYMBOL(tcc_ext_mutex);

int tcc_ctrl_ext_frame(char enable)
{
#ifdef CONFIG_DISPLAY_EXT_FRAME
	struct tcc_lcdc_image_update iImage;

	if(!enable)
	{
		iImage.enable = 0;
		return tcc_display_ext_frame((void*)&iImage, 0, EXT_FRAME_OUT_MODE);
	}
	else
	{
		memcpy(&iImage, &last_backup, sizeof(struct tcc_lcdc_image_update));
		iImage.enable = enable;
		tcc_display_ext_frame((void*)&iImage, iImage.deinterlace_mode, EXT_FRAME_OUT_MODE);
	}
#endif
	return 0;
}
EXPORT_SYMBOL(tcc_ctrl_ext_frame);

#endif

#ifdef CONFIG_VIDEO_DISPLAY_SWAP_VPU_FRAME
void tcc_video_frame_backup(struct tcc_lcdc_image_update *Image)
{
	if(Image->enable){
		memcpy(&CurrDisplayingImage, Image, sizeof(struct tcc_lcdc_image_update));
	}
}
unsigned int tcc_video_swap_vpu_frame(tcc_video_disp *p, int idx, WMIXER_INFO_TYPE *WmixerInfo, struct tcc_lcdc_image_update *TempImage)
{
	int ret = 0;
	
	if(!p->vsync_started || CurrDisplayingImage.enable == 0 || LastFrame){
		printk("----> return tcc_video_swap_vpu_frame :: reason %d/%d/%d \n", p->vsync_started, CurrDisplayingImage.enable, LastFrame);
		return -1;
	}

	if( CurrDisplayingImage.addr0 == fb_lastframe_pbuf.base ){
		printk("----> already swapped frame!!");
		return -1;
	}

	if( p->outputMode == Output_SelectMode )
	{
		struct file *file;

		file = filp_open(WMIXER_PATH, O_RDWR, 0666);
		msleep(16); // wait until previous interrupt is completed!!

		memcpy(TempImage, &CurrDisplayingImage, sizeof(struct tcc_lcdc_image_update));

		ret = tcc_move_video_frame_simple(file, TempImage, WmixerInfo, fb_lastframe_pbuf.base, TempImage->fmt);
		if( ret > 0 )
		{
			printk("----> start tcc_video_swap_vpu_frame :: %d \n", CurrDisplayingImage.buffer_unique_id);
			TempImage->addr0 = WmixerInfo->dst_y_addr;
			TempImage->addr1 = WmixerInfo->dst_u_addr;
			TempImage->addr2 = WmixerInfo->dst_v_addr;
			TempImage->buffer_unique_id = p->vsync_buffer.available_buffer_id_on_vpu;

			if(p->deinterlace_mode)
			{
				if(p->duplicateUseFlag)
				{
					int current_time;
					
					viqe_render_init();

					current_time = tcc_vsync_get_time(p);
					viqe_render_frame(TempImage, p->vsync_interval, current_time, 0);
					viqe_render_field(current_time);
				}
				else{
					if(p->output_toMemory && p->m2m_mode) {
						//TCC_VIQE_DI_Run60Hz_M2M(TempImage,0); // No need to display on M2M mode.
					} else {
						if (TempImage->Lcdc_layer == RDMA_VIDEO)
							TCC_VIQE_DI_Run60Hz(TempImage, 0);
					}
				}
			}
			else if(p->output_toMemory && p->m2m_mode) // for PIP mode with dual-vsync!!
			{
				//TCC_VIQE_DI_Run60Hz_M2M(TempImage,0); // No need to display on M2M mode.
			}
			else
			{
				struct fb_info *info = registered_fb[0];
				struct tccfb_info *tccfb_info = NULL;
				struct tcc_dp_device *dp_device = NULL;
				tccfb_info = info->par;
				if(tccfb_info->pdata.Mdp_data.DispDeviceType == p->outputMode)
					dp_device = &tccfb_info->pdata.Mdp_data;
				else if(tccfb_info->pdata.Sdp_data.DispDeviceType == p->outputMode)
					dp_device = &tccfb_info->pdata.Sdp_data;

				switch(p->outputMode)
				{
					case TCC_OUTPUT_NONE:
						break;
				#ifdef TCC_LCD_VIDEO_DISPLAY_BY_VSYNC_INT
					case TCC_OUTPUT_LCD:
						tca_scale_display_update(dp_device, TempImage);
						break;
				#endif
					case TCC_OUTPUT_HDMI:
						tca_scale_display_update(dp_device, TempImage);
						break;
					case TCC_OUTPUT_COMPONENT:
				#if defined(CONFIG_FB_TCC_COMPONENT)
						tcc_component_update(TempImage);
				#endif
						break;
					case TCC_OUTPUT_COMPOSITE:
				#if defined(CONFIG_FB_TCC_COMPOSITE)
						tcc_composite_update(TempImage);
				#endif
						break;
					default:
						break;
				}			
			}

			memcpy(&CurrDisplayingImage, TempImage, sizeof(struct tcc_lcdc_image_update));
			
			dprintk("----> video seek %d, %d :: backup %d \n", idx, CurrDisplayingImage.buffer_unique_id, CurrDisplayingImage.buffer_unique_id) ;
			msleep(16*2); // wait until current setting is applyed completely!!
		}
		else
		{
			printk(" Error :: wmixer ctrl \n");
		}
		filp_close(file, 0);
	}

	return CurrDisplayingImage.buffer_unique_id;
}
#endif

struct mutex vsync_io_mutex;
static long tcc_vsync_do_ioctl(unsigned int cmd, unsigned long arg, int idx)
{
	int ret = 0;

	tcc_video_disp *p = &tccvid_vsync[idx];
	
	mutex_lock(&vsync_io_mutex);
	{
		switch (cmd) {
		
			case TCC_LCDC_VIDEO_START_VSYNC:
				{
					struct tcc_lcdc_image_update input_image;
		
					#if 0//defined(CONFIG_TCC_DISPLAY_MODE_USE)
					if(Output_SelectMode == TCC_OUTPUT_NONE)
					{
						//printk("### Error: there is no valid output on STB\n");
						ret = -EFAULT;
						goto Error;
					}
					#endif
		
					if (copy_from_user((void *)&input_image , (const void *)arg, sizeof(struct tcc_lcdc_image_update)))
					{
						printk("fatal error") ;
						ret = -EFAULT;
						goto Error;
					}

					if(!vsync_power_state)
					{
						printk("##### Error ### vsync_power_state \n"); 			
						ret = -EFAULT;
						goto Error;
					}
					
					tcc_vsync_start(p, &input_image, idx);
				}
				break ;
		
			case TCC_LCDC_VIDEO_END_VSYNC:
				tcc_vsync_end(p, idx);
				break ;
		
			case TCC_LCDC_VIDEO_SET_SIZE_CHANGE:
				printk("### SET_SIZE_CHANGE, firstFrame(%d) deint_mode(%d) \n", p->firstFrameFlag, p->deinterlace_mode);

				#if 0
				spin_lock_irq(&vsync_lock) ;
				tcc_vsync_pop_all_buffer(&p->vsync_buffer);
				spin_unlock_irq(&vsync_lock) ;
				#endif
				break ;
		
			case TCC_LCDC_VIDEO_SET_FRAMERATE:
				tcc_video_set_framerate(p, (int)arg);
				break ;
		
			case TCC_LCDC_VIDEO_SET_MVC_STATUS:
				{
					int status;

					if(copy_from_user(&status, (int*)arg, sizeof(int))){
						ret = -EFAULT;
						goto Error;
					}

					if(status)
					{
						tcc_vsync_mvc_status = 1;
						printk("$$$ tcc_vsync_mvc_status= %d\n", tcc_vsync_mvc_status);
					}
					else
					{
						tcc_vsync_mvc_status = 0;
						printk("$$$ tcc_vsync_mvc_status= %d\n", tcc_vsync_mvc_status);
					}
				}
				break ;

		
			case TCC_LCDC_VIDEO_PUSH_VSYNC:
				{
					struct tcc_lcdc_image_update input_image;
					struct fb_info *info = registered_fb[0];
					struct tccfb_info *tccfb_info = NULL;
					struct tcc_dp_device *dp_device = NULL;

					if (copy_from_user((void *)&input_image , (const void *)arg, sizeof(struct tcc_lcdc_image_update)))
					{
						printk("fatal error :: copy err");
						ret = 0;
						goto Error;
					}

					if(!p->vsync_started){
						printk("fatal error :: vsync is not started");
						ret = 0;
						goto Error;
					}

					vprintk("%s: PUSH_%d ioctl ID(%d) - TS(v[%d ms] / s[%d ms]) \n", input_image.Lcdc_layer == RDMA_VIDEO ? "M" : "S", idx,
								input_image.buffer_unique_id, input_image.time_stamp, input_image.sync_time);

					tccfb_info = info->par;

					if(tccfb_info->pdata.Mdp_data.DispDeviceType == Output_SelectMode)
						dp_device = &tccfb_info->pdata.Mdp_data;
					else if(tccfb_info->pdata.Sdp_data.DispDeviceType == Output_SelectMode)
						dp_device = &tccfb_info->pdata.Sdp_data;
					else {
						printk("%s: Main:%d, Sub :%d Select:%d \n", __func__,
							tccfb_info->pdata.Mdp_data.DispDeviceType,
							tccfb_info->pdata.Sdp_data.DispDeviceType,
							Output_SelectMode);
						goto Error;
					}

					if(tcc_vsync_push_preprocess(p, dp_device,&input_image) < 0)
						goto TCC_VSYNC_PUSH_ERROR;

					#ifdef CONFIG_HDMI_DISPLAY_LASTFRAME
					memcpy(&Last_ImageInfo, &input_image, sizeof(struct tcc_lcdc_image_update));
					#endif

					if(p->outputMode < 0)
					{
						tcc_vsync_push_set_outputMode(p, dp_device,&input_image);
					}

					tcc_vsync_push_preprocess_deinterlacing(p, dp_device,&input_image);
		
					// This is for Display underrun issue.
					if(!p->deinterlace_mode && !p->output_toMemory )
					{
						#if 0
						if(p->outputMode == Output_SelectMode && 
							(p->outputMode == TCC_OUTPUT_HDMI || p->outputMode == TCC_OUTPUT_LCD))
							tca_set_onthefly(pdp_data,&input_image);
						#else
						
						if(p->outputMode == TCC_OUTPUT_HDMI || p->outputMode == TCC_OUTPUT_COMPOSITE || p->outputMode == TCC_OUTPUT_COMPONENT)
							tca_lcdc_set_onthefly(dp_device, &input_image);
						else if(p->outputMode == TCC_OUTPUT_LCD)
							tca_lcdc_set_onthefly(dp_device, &input_image);
						
						#endif
					}
					
					if(tcc_vsync_push_bypass_frame(p, dp_device,&input_image) > 0)
						goto PUSH_VIDEO_FORCE;
					
					if(tcc_vsync_push_check_error(p, dp_device,&input_image) < 0)
						goto TCC_VSYNC_PUSH_ERROR;
		
				PUSH_VIDEO_FORCE : 
					tcc_vsync_push_process(p, dp_device,&input_image, idx);
				}
		
				TCC_VSYNC_PUSH_ERROR:
					ret = p->vsync_buffer.writeIdx;
				break ;
				
			case TCC_LCDC_VIDEO_GET_DISPLAYED :
				ret = tcc_video_get_displayed(p); 
				break ;
				
			case TCC_LCDC_VIDEO_GET_VALID_COUNT:
				ret = tcc_video_get_valid_count(p);
				break ;
				
			case TCC_LCDC_VIDEO_CLEAR_FRAME:
				tcc_video_clear_frame(p, (int)arg);
				break ;
		
			case TCC_LCDC_VIDEO_SKIP_FRAME_START:
				tcc_video_skip_frame_start(p);
				break ;
		
			case TCC_LCDC_VIDEO_SKIP_FRAME_END:
				tcc_video_skip_frame_end(p);
				break ;
				
			case TCC_LCDC_VIDEO_SKIP_ONE_FRAME:
				tcc_video_skip_one_frame(p, (int)arg);
				break ;

			case TCC_LCDC_VIDEO_SWAP_VPU_FRAME:
#ifdef CONFIG_VIDEO_DISPLAY_SWAP_VPU_FRAME
			{
				WMIXER_INFO_TYPE WmixerInfo;
				struct tcc_lcdc_image_update TempImage;

				ret = tcc_video_swap_vpu_frame(p, (int)arg, &WmixerInfo, &TempImage);
			}
#endif
				break;

			case TCC_LCDC_VIDEO_KEEP_LASTFRAME:
#if defined(CONFIG_HDMI_DISPLAY_LASTFRAME)
				if (!g_is_vsync1_opened) 
				{
					struct stTcc_last_frame lastframe_info;
					
					if (copy_from_user((void*)&lastframe_info, (const void*)arg, sizeof(struct stTcc_last_frame)))
						ret = -EFAULT;
					else
					{
						struct tcc_lcdc_image_update lastUpdated;

						if((ret = tcc_video_last_frame(p, lastframe_info, &lastUpdated, idx)) > 0)
							p->deinterlace_mode = 0;

					}
				}
#endif
				break;

			case TCC_LCDC_VIDEO_GET_LASTFRAME_STATUS:
#if defined(CONFIG_HDMI_DISPLAY_LASTFRAME)
				if(enabled_LastFrame)
					ret = 0;
				else
#endif
					ret = -EFAULT;
				break;
				
			case TCC_LCDC_VIDEO_CTRL_LAST_FRAME:
				{
#if defined(CONFIG_HDMI_DISPLAY_LASTFRAME)
					tcc_video_ctrl_last_frame((int)arg);
#endif
				}
				break;
		  
			case TCC_LCDC_SET_M2M_LASTFRAME:
				{
#if defined(CONFIG_HDMI_DISPLAY_LASTFRAME)
					LastFrame = 0;
					Output_SelectMode = TCC_OUTPUT_NONE;

					dprintk("TCC_LCDC_SET_M2M_LASTFRAME \n");
					dprintk(" LastFrame=%d Output_SelectMode=%d\n",LastFrame, Output_SelectMode);

					//if(Output_SelectMode == TCC_OUTPUT_NONE){
						if (LCD_LCDC_NUM)
							pRDMA_LastFrame = (VIOC_RDMA*)tcc_p2v(HwVIOC_RDMA05);
						else
							pRDMA_LastFrame = (VIOC_RDMA*)tcc_p2v(HwVIOC_RDMA01);

						if(pRDMA_LastFrame != NULL){
							VIOC_RDMA_SetImageDisable(pRDMA_LastFrame);
						}
					//}
#endif
				}
				break;

#ifdef CONFIG_DISPLAY_EXT_FRAME
			case TCC_LCDC_VIDEO_CONFIGURE_EXT_FRAME:
				{
					struct tcc_lcdc_image_update iImage;

					if (copy_from_user((void*)&iImage, (const void*)arg, sizeof(struct tcc_lcdc_image_update)))
						return -EFAULT;
					else
					{
						lf_printk("%s-%d :: TCC_LCDC_VIDEO_CONFIGURE_EXT_FRAME %d x %d / %d x %d \n", __func__, __LINE__,
								iImage.private_data.optional_info[0], iImage.private_data.optional_info[1],
								iImage.private_data.optional_info[2], iImage.private_data.optional_info[3]);					
						memcpy(&last_backup, &iImage, sizeof(struct tcc_lcdc_image_update));
					}
				}
				break;

			// change the region displaying!!
			case TCC_LCDC_VIDEO_CHANGE_REGION_OF_EXT_FRAME:
				{
					struct tcc_ext_dispregion disp_region;

					if (copy_from_user((void*)&disp_region, (const void*)arg, sizeof(struct tcc_ext_dispregion)))
						return -EFAULT;
					else
					{
						last_backup.offset_x 	 = disp_region.start_x;
						last_backup.offset_y 	 = disp_region.start_y;
						last_backup.Image_width  = disp_region.width;
						last_backup.Image_height = disp_region.height;
						lf_printk("%s-%d :: TCC_LCDC_VIDEO_CHANGE_REGION_OF_EXT_FRAME bExt_Frame(%d), %d,%d ~ %d x %d\n", __func__, __LINE__, bExt_Frame,
									last_backup.offset_x, last_backup.offset_y, last_backup.Image_width, last_backup.Image_height);						
						if(bExt_Frame)
							tcc_display_ext_frame((void*)&last_backup, last_backup.deinterlace_mode, EXT_FRAME_OUT_MODE);
					}
				}
				break;

			// refresh current frame with LUT...
			case TCC_LCDC_VIDEO_REFRESH_EXT_FRAME:
				{
					lf_printk("%s-%d :: TCC_LCDC_VIDEO_REFRESH_EXT_FRAME bExt_Frame(%d)\n", __func__, __LINE__, bExt_Frame);
					if(bExt_Frame)
						tcc_display_ext_frame((void*)&last_backup, last_backup.deinterlace_mode, EXT_FRAME_OUT_MODE);
				}
				break;

			// control on/off!!
			case TCC_LCDC_VIDEO_CTRL_EXT_FRAME:
				{
					unsigned int enable;

					if (copy_from_user((void*)&enable, (const void*)arg, sizeof(unsigned int)))
						return -EFAULT;
					else
					{
						last_backup.enable = enable;
						lf_printk("%s-%d :: TCC_LCDC_VIDEO_CTRL_EXT_FRAME enable(%d)\n", __func__, __LINE__, enable);						
						tcc_display_ext_frame((void*)&last_backup, last_backup.deinterlace_mode, EXT_FRAME_OUT_MODE);
					}
				}
				break;
#endif

			default:
				printk("vsync: unrecognized ioctl (0x%x)\n", cmd); 
				ret = -EINVAL;
				break;
		}
	}

Error:
	mutex_unlock(&vsync_io_mutex);
	
	return ret;
}

void tca_vsync_video_display_enable(void)
{
	int ret=-1;
	char lcdc_num = EX_OUT_LCDC;

	#ifdef TCC_LCD_VIDEO_DISPLAY_BY_VSYNC_INT
	if(vsync_started_device == LCD_START_VSYNC)
		lcdc_num = LCD_LCDC_NUM;
	#endif

	
	if(lcdc_interrupt_onoff == 0)
	{
		if(lcdc_num){
			
			#ifdef TCC_LCD_VIDEO_DISPLAY_BY_VSYNC_INT
			if(vsync_started_device == LCD_START_VSYNC)
			{
				vioc_intr_disable(lcdc_num, vsync_vioc1_disp.vioc_intr->bits);
				ret = request_irq(vsync_vioc1_disp.irq_num, tcc_vsync_handler_for_video,IRQF_SHARED,"vioc_vsync1", &vsync_vioc1_disp);
				if (ret)
					printk("\n\n @@@%s: vioc1 ret=%d @@@\n\n\n", __func__, ret);
				vioc_intr_enable(lcdc_num, vsync_vioc1_disp.vioc_intr->bits);
			}
			else
			#endif
			{
				// TODO:
				BUG();
				
			}
		}
		else{
			#ifdef TCC_LCD_VIDEO_DISPLAY_BY_VSYNC_INT
			if(vsync_started_device == LCD_START_VSYNC)
			{
				vioc_intr_disable(lcdc_num, vsync_vioc0_disp.vioc_intr->bits);
				ret= request_irq(vsync_vioc0_disp.irq_num, tcc_vsync_handler_for_video,IRQF_SHARED, "vioc_vsync0", &vsync_vioc0_disp);
				vioc_intr_enable(lcdc_num, vsync_vioc0_disp.vioc_intr->bits);
			}
			else
			#endif
			{
				vioc_intr_disable(lcdc_num, vsync_vioc0_disp.vioc_intr->bits);
				ret= request_irq(vsync_vioc0_disp.irq_num, tcc_vsync_handler_for_video,IRQF_SHARED, "vioc_vsync0", &vsync_vioc0_disp);
				vioc_intr_enable(lcdc_num, vsync_vioc0_disp.vioc_intr->bits);
			}
		}
		
		printk("%s: onoff(%d) request_irq(%d) ret(%d) \n",__func__,lcdc_interrupt_onoff,lcdc_num,ret);
		
#ifdef USE_SOFT_IRQ_FOR_VSYNC
		INIT_WORK(&vsync_work_q, tcc_video_display_update_isr);
#endif
		lcdc_interrupt_onoff = 1;
		video_display_disable_check = 0;
	}
	else
	{
		printk("tccfb error: lcdc interrupt have been enabled already\n");
	}
}

void tca_vsync_video_display_disable(void)
{
	char lcdc_num = EX_OUT_LCDC;

	#ifdef TCC_LCD_VIDEO_DISPLAY_BY_VSYNC_INT
	if(vsync_started_device == LCD_START_VSYNC)
		lcdc_num = LCD_LCDC_NUM;

	printk("%s: onoff(%d) started_device(%d) \n",__func__,lcdc_interrupt_onoff,vsync_started_device);
	#else
	printk("%s: onoff(%d) \n",__func__,lcdc_interrupt_onoff);
	#endif
	
	if(lcdc_interrupt_onoff == 1)
	{
		if(lcdc_num){
			#ifdef TCC_LCD_VIDEO_DISPLAY_BY_VSYNC_INT
			if(vsync_started_device == LCD_START_VSYNC)
			{
				vioc_intr_disable(lcdc_num, vsync_vioc1_disp.vioc_intr->bits);
				free_irq(vsync_vioc1_disp.irq_num, &vsync_vioc1_disp);
			}
			else
			#endif
			{
				// TODO:
				BUG();
			}
		}
		else {
			#ifdef TCC_LCD_VIDEO_DISPLAY_BY_VSYNC_INT
			if(vsync_started_device == LCD_START_VSYNC)
			{
				vioc_intr_disable(lcdc_num, vsync_vioc0_disp.vioc_intr->bits);
				free_irq(vsync_vioc0_disp.irq_num, &vsync_vioc0_disp);
			}
			else
			#endif
			{
				vioc_intr_disable(lcdc_num, vsync_vioc0_disp.vioc_intr->bits);
				free_irq(vsync_vioc0_disp.irq_num, &vsync_vioc0_disp);
			}
		}
		lcdc_interrupt_onoff = 0;
	}
	else
	{
		printk("tccfb error: lcdc interrupt have been disabled already\n");
	}
}

int tcc_video_get_displayed(tcc_video_disp *p)
{

	if(p->skipFrameStatus || !p->vsync_started)
	{
		if(p->skipFrameStatus){
			vprintk("frame skip mode return\n");
			return -2;
		}
		return -1;
	}

	//printk("# last disp num(%d) \n", p->vsync_buffer.last_cleared_buff_id ) ;
	if(p->mvcMode == 1)
		return p->vsync_buffer.last_cleared_buff_id > 1 ? (p->vsync_buffer.last_cleared_buff_id-2):0 ; 
	else
		return p->vsync_buffer.last_cleared_buff_id ? (p->vsync_buffer.last_cleared_buff_id-1):0 ; 

}

void tcc_vsync_set_firstFrameFlag(tcc_video_disp *p, int firstFrameFlag)
{
	p->firstFrameFlag = firstFrameFlag;
}

void tcc_vsync_set_firstFrameFlag_all(int firstFrameFlag)
{
	tcc_vsync_set_firstFrameFlag(&tccvid_vsync[0], firstFrameFlag);
	tcc_vsync_set_firstFrameFlag(&tccvid_vsync[1], firstFrameFlag);
}

void tcc_video_clear_frame(tcc_video_disp *p, int idx)
{
	int syncBufferIdx;
	
	syncBufferIdx = idx;

	printk("video frame clear %d\n", syncBufferIdx) ;

	spin_lock_irq(&vsync_lock) ;
	tcc_vsync_pop_all_buffer(&p->vsync_buffer);
	spin_unlock_irq(&vsync_lock) ;
	//printk("valid video fram count %d\n", p->vsync_buffer.valid_buff_count) ;

	p->vsync_buffer.available_buffer_id_on_vpu = syncBufferIdx;//+1;
}

void tcc_video_set_framerate(tcc_video_disp *p, int fps)
{
	int media_time_gap;
	printk("### tcc_video_set_framerate %d\n", fps);

	if(fps < 1000)
	{
		if(fps == 29)
		{
			fps = 30;
		}
		else if(fps == 23)
		{
			fps = 24;
		}
	}
	else
	{
		fps += 500;
		fps /= 1000;
	}

	p->video_frame_rate = fps;

	if(p->video_frame_rate > 0)
		media_time_gap = 1000/fps;
	else
		media_time_gap = 32;
	
	p->nTimeGapToNextField= media_time_gap;

	printk("### video_frame_rate(%d), media_time_gap(%d)\n",p->video_frame_rate, media_time_gap);
}

int tcc_video_check_framerate(tcc_video_disp *p, int fps)
{
	int media_time_gap;

	if(fps < 1000)
	{
		if(fps == 29)
		{
			fps = 30;
		}
		else if(fps == 23)
		{
			fps = 24;
		}
	}
	else
	{
		fps += 500;
		fps /= 1000;
	}

	if(p->video_frame_rate != fps )
	{

		p->video_frame_rate = fps;

		if(p->video_frame_rate > 0)
			media_time_gap = 1000/fps;
		else
			media_time_gap = 32;

		p->nTimeGapToNextField= media_time_gap;

		printk("### video_frame_rate(%d), media_time_gap(%d)\n",p->video_frame_rate, media_time_gap);

		return 1;
	}

	return 0;
}

void tcc_video_skip_frame_start(tcc_video_disp *p)
{
	vprintk("video fram skip start\n") ;
	printk("TCC_LCDC_VIDEO_SKIP_FRAME_START\n") ;
	spin_lock_irq(&vsync_lock) ;
	tcc_vsync_pop_all_buffer(&p->vsync_buffer);
	spin_unlock_irq(&vsync_lock);
	p->skipFrameStatus = 1;
}

void tcc_video_skip_frame_end(tcc_video_disp *p)
{
	if(p->skipFrameStatus)
	{
		vprintk("video fram skip end\n") ;
		spin_lock_irq(&vsync_lock) ;
		tcc_vsync_pop_all_buffer(&p->vsync_buffer);
		spin_unlock_irq(&vsync_lock);
		printk("TCC_LCDC_VIDEO_SKIP_FRAME_END\n") ;

		p->skipFrameStatus = 0;
		tcc_vsync_reset_syncTime(p, 0);
	}

}

void tcc_video_skip_one_frame(tcc_video_disp *p, int frame_id)
{
	spin_lock_irq(&vsync_lockDisp) ;
	//printk("TCC_LCDC_VIDEO_SKIP_ONE_FRAME frame_id %d\n",frame_id) ;
	p->vsync_buffer.last_cleared_buff_id = frame_id;
	spin_unlock_irq(&vsync_lockDisp) ;
}

int tcc_video_get_valid_count(tcc_video_disp *p)
{
	return atomic_read( & p->vsync_buffer.readable_buff_count); 
}

void tcc_vsync_set_output_mode(tcc_video_disp *p, int mode)
{
	spin_lock_irq(&vsync_lockDisp) ;
	p->outputMode = mode;
	spin_unlock_irq(&vsync_lockDisp) ;
}

void tcc_vsync_set_output_mode_all(int mode)
{
	tcc_vsync_set_output_mode(&tccvid_vsync[0], mode);
	tcc_vsync_set_output_mode(&tccvid_vsync[1], mode);
}

void tcc_vsync_hdmi_start(struct tcc_dp_device *pdp_data,int* lcd_video_started)
{

#ifdef TCC_LCD_VIDEO_DISPLAY_BY_VSYNC_INT
	printk("%s : SelectMode(%d) started_device(%d) \n",__func__,Output_SelectMode,vsync_started_device);

	Output_SelectMode = TCC_OUTPUT_HDMI;
	if(tccvid_vsync[0].isVsyncRunning)
	{
		printk("VSYNC CH0 HDMI TIMING, by the way LCD and vsync enable \n");
		// interlace video on LCD -> HDMI plug in -> garbage screen debugging
		if(tccvid_vsync[0].deinterlace_mode && !tccvid_vsync[0].output_toMemory &&!tccvid_vsync[0].interlace_bypass_lcdc)
		{
			printk("and Interlaced \n");
		}
		else
		{
			struct tcc_lcdc_image_update ImageInfo;
			memset(&ImageInfo, 0x00, sizeof(struct tcc_lcdc_image_update));
			ImageInfo.enable = 0;
			ImageInfo.Lcdc_layer = RDMA_VIDEO;

			tca_scale_display_update(pdp_data, &ImageInfo);
		}
	}

	if(tccvid_vsync[1].isVsyncRunning)
	{
		printk("VSYNC CH1 HDMI TIMING, by the way LCD and vsync enable \n");
		// interlace video on LCD -> HDMI plug in -> garbage screen debugging
		if(tccvid_vsync[1].deinterlace_mode && !tccvid_vsync[1].output_toMemory &&!tccvid_vsync[1].interlace_bypass_lcdc)
		{
			printk("and Interlaced \n");
		}
		else
		{
			struct tcc_lcdc_image_update ImageInfo;
			memset(&ImageInfo, 0x00, sizeof(struct tcc_lcdc_image_update));
			ImageInfo.enable = 0;
			ImageInfo.Lcdc_layer = RDMA_VIDEO_SUB;

			tca_scale_display_update(pdp_data, &ImageInfo);
		}
	}
				
#else
	printk(" %s: SelectMode(%d) lcd_started(%d)\n",__func__,Output_SelectMode,*lcd_video_started);

	if(*lcd_video_started)
	{
		struct tcc_lcdc_image_update ImageInfo;
		
		printk(" TCC_LCDC_HDMI_TIMING => TCC_LCDC_DISPLAY_END \n");

		memset(&ImageInfo, 0x00, sizeof(struct tcc_lcdc_image_update));
		ImageInfo.enable = 0;
		ImageInfo.Lcdc_layer = RDMA_VIDEO;

		tca_scale_display_update(pdp_data, (struct tcc_lcdc_image_update *)&ImageInfo);

		ImageInfo.enable = 0;
		ImageInfo.Lcdc_layer = RDMA_VIDEO_SUB;

		tca_scale_display_update(pdp_data, (struct tcc_lcdc_image_update *)&ImageInfo);

		*lcd_video_started = 0;
	}

	Output_SelectMode = TCC_OUTPUT_HDMI;

	if(video_display_disable_check) {
		tca_vsync_video_display_enable();
		video_display_disable_check = 0;
	}

	#if defined(CONFIG_TCC_OUTPUT_AUTO_HDMI_CVBS) || defined(CONFIG_TCC_OUTPUT_ATTACH)
	tcc_vsync_set_firstFrameFlag_all(1);
	#endif

	if(tcc_vsync_mvc_status && tccvid_vsync[0].mvcMode==1) {
		VIOC_WMIX_GetOverlayPriority(pdp_data->wmixer_info.virt_addr, &tcc_vsync_mvc_overlay_priority);	//save
		VIOC_WMIX_SetOverlayPriority(pdp_data->wmixer_info.virt_addr, 16);									//Image1 - Image0 - Image2 - Image3
		VIOC_WMIX_SetUpdate(pdp_data->wmixer_info.virt_addr);
	}

	spin_lock_irq(&vsync_lock);
	tcc_vsync_set_output_mode_all(-1);
	spin_unlock_irq(&vsync_lock);
#endif

}

void tcc_vsync_hdmi_end(struct tcc_dp_device *pdp_data)
{
#ifdef TCC_LCD_VIDEO_DISPLAY_BY_VSYNC_INT
	printk(" %s: out %d start %d \n",__func__,Output_SelectMode,vsync_started_device);
	if(Output_SelectMode == TCC_OUTPUT_HDMI) 
	{
		struct tcc_lcdc_image_update lcdc_image;

		//this check is for the case LCD video - HDMI cable plug out - plug in after short time(before HDMI video) - LCD video
		if(vsync_started_device == LCD_START_VSYNC)
			Output_SelectMode = TCC_OUTPUT_LCD;
		else
			Output_SelectMode = TCC_OUTPUT_NONE;
		
		memset(&lcdc_image, 0x00, sizeof(struct tcc_lcdc_image_update));
		lcdc_image.enable = 0;
		lcdc_image.Lcdc_layer = 3;

		tca_scale_display_update(pdp_data, &lcdc_image);
	}

	tcc_vsync_set_firstFrameFlag_all(1);
	
#else
	printk(" %s: SelectMode %d \n",__func__,Output_SelectMode);

	if(Output_SelectMode == TCC_OUTPUT_HDMI) 
	{				
		struct tcc_lcdc_image_update lcdc_image;

		Output_SelectMode = TCC_OUTPUT_NONE;
		memset(&lcdc_image, 0x00, sizeof(struct tcc_lcdc_image_update));
		lcdc_image.enable = 0;
		lcdc_image.Lcdc_layer = RDMA_VIDEO;

		tca_scale_display_update(pdp_data, &lcdc_image);

		lcdc_image.enable = 0;
		lcdc_image.Lcdc_layer = RDMA_VIDEO_SUB;

		tca_scale_display_update(pdp_data, &lcdc_image);

		if(!video_display_disable_check)
		{
			tca_vsync_video_display_disable();
			video_display_disable_check = 1;
		}

	#if !defined(CONFIG_TCC_OUTPUT_AUTO_HDMI_CVBS) && !defined(CONFIG_TCC_OUTPUT_ATTACH)
		tcc_vsync_set_firstFrameFlag_all(1);
	#endif
	}

	#ifdef TCC_VIDEO_DISPLAY_DEINTERLACE_MODE
	if (tccvid_vsync[0].vsync_started) {
		if(tccvid_vsync[0].output_toMemory && tccvid_vsync[0].m2m_mode) {
			TCC_VIQE_DI_DeInit60Hz_M2M(RDMA_VIDEO);
		} else if(tccvid_vsync[0].deinterlace_mode &&!tccvid_vsync[0].interlace_bypass_lcdc) {
			if(!tccvid_vsync[0].output_toMemory)
				TCC_VIQE_DI_DeInit60Hz();
		}
	}

	if (tccvid_vsync[1].vsync_started) {
		if(tccvid_vsync[1].output_toMemory && tccvid_vsync[1].m2m_mode) {
			TCC_VIQE_DI_DeInit60Hz_M2M(RDMA_VIDEO_SUB);
		} else if(tccvid_vsync[1].deinterlace_mode &&!tccvid_vsync[1].interlace_bypass_lcdc) {
			if(!tccvid_vsync[1].output_toMemory)
				TCC_VIQE_DI_DeInit60Hz();
		}
	}
	#endif
	//tcc_vsync_pop_all_buffer(&tccvid_vsync[0].vsync_buffer);
	//tcc_vsync_pop_all_buffer(&tccvid_vsync[1].vsync_buffer);
#endif

#if defined(CONFIG_HDMI_DISPLAY_LASTFRAME) 
	tcc_video_clear_last_frame(pLastFrame_RDMABase, false);
#endif
}

void tcc_vsync_reset_all(void)
{
	int i;
	
	for (i=0; i<NUM_OF_VSYNC_CH; i++) {
		memset( &tccvid_vsync[i], 0, sizeof( tcc_video_disp) );
		tccvid_vsync[i].overlayUsedFlag = -1;
		tccvid_vsync[i].outputMode = -1;
		tccvid_vsync[i].firstFrameFlag = 1;
		tccvid_vsync[i].deinterlace_mode= -1;
		tccvid_vsync[i].m2m_mode = -1;
		tccvid_vsync[i].output_toMemory = -1;
	}
}

void tcc_vsync_if_pop_all_buffer(tcc_video_disp *p)
{
	spin_lock_irq(&vsync_lock);
	tcc_vsync_pop_all_buffer(&p->vsync_buffer);
	spin_unlock_irq(&vsync_lock);
}

void tcc_vsync_set_deinterlace_mode(tcc_video_disp *p, int mode)
{
	p->deinterlace_mode = mode;
}
EXPORT_SYMBOL(tcc_vsync_set_deinterlace_mode);

int tcc_vsync_isVsyncRunning(int idx)
{
	return tccvid_vsync[idx].isVsyncRunning;
}
EXPORT_SYMBOL(tcc_vsync_isVsyncRunning);

int tcc_vsync_get_isVsyncRunning(int idx)
{
	if(idx == 0 && !tccvid_vsync[1].isVsyncRunning)
		return tccvid_vsync[0].isVsyncRunning;
	else if(idx == 1 && !tccvid_vsync[0].isVsyncRunning)
		return tccvid_vsync[1].isVsyncRunning;

	return 0;
}

int tcc_vsync_get_output_toMemory(tcc_video_disp *p)
{
	return p->output_toMemory;
}

int tcc_vsync_get_interlace_bypass_lcdc(tcc_video_disp *p)
{
	return p->interlace_bypass_lcdc;
}

int tcc_vsync_get_deinterlace_mode(tcc_video_disp *p)
{
	return p->deinterlace_mode;
}

int is_deinterlace_enabled(int idx)
{
	if (tcc_vsync_get_deinterlace_mode(&tccvid_vsync[idx]) 
	&& !tcc_vsync_get_output_toMemory(&tccvid_vsync[idx]) 
	&& !tcc_vsync_get_interlace_bypass_lcdc(&tccvid_vsync[idx]))
		return 1;

	return 0;
}
static long tcc_vsync1_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	return tcc_vsync_do_ioctl(cmd, arg, 1);
}

static int tcc_vsync1_release(struct inode *inode, struct file *filp)
{
	dprintk("%s(%d)\n", __func__, g_is_vsync1_opened);

	if (--g_is_vsync1_opened)
		return 0;

	#ifdef USE_VSYNC_TIMER
	if (vsync_timer && !g_is_vsync0_opened)
		tcc_timer_disable(vsync_timer);
	#endif

	return 0;
}

static int tcc_vsync1_open(struct inode *inode, struct file *filp)
{
	dprintk("%s(%d)\n", __func__, g_is_vsync1_opened);

	if (g_is_vsync1_opened++)
		return 0;

	#ifdef USE_VSYNC_TIMER
	if (vsync_timer && !g_is_vsync0_opened) {
		tcc_timer_enable(vsync_timer);
		msleep(0);
	}
	#endif

	return 0;
}

static long tcc_vsync0_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	return tcc_vsync_do_ioctl(cmd, arg, 0);
}

static int tcc_vsync0_release(struct inode *inode, struct file *filp)
{
	dprintk("%s(%d)\n", __func__, g_is_vsync0_opened);

	if (--g_is_vsync0_opened)
		return 0;

	#ifdef USE_VSYNC_TIMER
	if (vsync_timer && !g_is_vsync1_opened)
		tcc_timer_disable(vsync_timer);
	#endif

	return 0;
}

static int tcc_vsync0_open(struct inode *inode, struct file *filp)
{
	dprintk("%s(%d)\n", __func__, g_is_vsync0_opened);

	if (g_is_vsync0_opened++)
		return 0;

	#ifdef USE_VSYNC_TIMER
	if (vsync_timer && !g_is_vsync1_opened) {
		tcc_timer_enable(vsync_timer);
		msleep(0);
	}
	#endif

	return 0;
}


static int tcc_vsync_parse_dt(struct device_node *np)
{
	struct device_node *vioc0_node,*vioc1_node;
	int ret;
	
	if (np) {
		memset(&vsync_vioc1_disp,0x00,sizeof(struct tcc_vsync_display_info_t));

		vioc1_node = of_parse_phandle(np,"display-vioc1", 0);
		if (vioc1_node) {
			vsync_vioc1_disp.virt_addr = of_iomap(vioc1_node, 0);
			if (of_property_read_u32(vioc1_node, "disp_number", &vsync_vioc1_disp.lcdc_num)){
				pr_err( "could not find ddc nubmer\n");
				ret = -ENODEV;
			}
			vsync_vioc1_disp.irq_num = irq_of_parse_and_map(vioc1_node, 0);

			printk("%s: vioc1 addr(%x) lcdc_num(%d) irq_num(%d) \n ",__func__,(unsigned int)vsync_vioc1_disp.virt_addr,vsync_vioc1_disp.lcdc_num,vsync_vioc1_disp.irq_num);
				
		}
		
		memset(&vsync_vioc0_disp,0x00,sizeof(struct tcc_vsync_display_info_t));
		vioc0_node = of_parse_phandle(np,"display-vioc0", 0);
		if (!vioc0_node) {
			pr_err( "could not find telechips,ddc node\n");
			ret = -ENODEV;
		}
		vsync_vioc0_disp.virt_addr = of_iomap(vioc0_node, 0);
		if (of_property_read_u32(vioc0_node, "disp_number", &vsync_vioc0_disp.lcdc_num)){
			pr_err( "could not find ddc nubmer\n");
			ret = -ENODEV;
		}
		vsync_vioc0_disp.irq_num = irq_of_parse_and_map(vioc0_node, 0);

		printk("%s: vioc0 addr(%x) lcdc_num(%d) irq_num(%d) \n ",__func__,(unsigned int)vsync_vioc0_disp.virt_addr,vsync_vioc0_disp.lcdc_num,vsync_vioc0_disp.irq_num);
	}
	else{
		return -ENODEV;
	}
	
	return 0;
}

#ifdef CONFIG_PM
static int tcc_vsync_suspend(struct device *dev)
{
	printk("%s \n", __FUNCTION__);
	
	return 0;
}

static int tcc_vsync_resume(struct device *dev)
{
	printk("%s \n", __FUNCTION__);

	return 0;
}
#else
#define tcc_vsync_suspend NULL
#define tcc_vsync_resume NULL
#endif

#ifdef CONFIG_PM_RUNTIME
static int tcc_vsync_runtime_suspend(struct device *dev)
{
	printk("%s:  \n", __FUNCTION__);
	vsync_power_state = 0;

	return 0;
}

static int tcc_vsync_runtime_resume(struct device *dev)
{
	printk("%s:  \n", __FUNCTION__);
	vsync_power_state = 1;

	return 0;
}

static const struct dev_pm_ops tcc_vsync_pm_ops = {
	.runtime_suspend	  = tcc_vsync_runtime_suspend,
	.runtime_resume 	  = tcc_vsync_runtime_resume,
	.suspend	= tcc_vsync_suspend,
	.resume =tcc_vsync_resume,
};
#endif

static const struct file_operations tcc_vsync0_fops =
{
	.owner			= THIS_MODULE,
	.open			= tcc_vsync0_open,
	.release		= tcc_vsync0_release,
	.unlocked_ioctl = tcc_vsync0_ioctl,
};

static const struct file_operations tcc_vsync1_fops =
{
	.owner			= THIS_MODULE,
	.open			= tcc_vsync1_open,
	.release		= tcc_vsync1_release,
	.unlocked_ioctl = tcc_vsync1_ioctl,
};

static struct miscdevice vsync0_misc_device =
{
	MISC_DYNAMIC_MINOR,
	"tcc_vsync0",
	&tcc_vsync0_fops,
};

static struct miscdevice vsync1_misc_device =
{
	MISC_DYNAMIC_MINOR,
	"tcc_vsync1",
	&tcc_vsync1_fops,
};

static int tcc_vsync_probe(struct platform_device *pdev)
{
	int ret=0;
	
	spin_lock_init(&vsync_lock) ;
	spin_lock_init(&vsync_lockDisp ) ;
	
	vsync_power_state = 1;
	
	if(tcc_vsync_parse_dt(pdev->dev.of_node) < 0){
		return -ENODEV;
	}

	EX_OUT_LCDC = vsync_vioc0_disp.lcdc_num;
	LCD_LCDC_NUM = vsync_vioc1_disp.lcdc_num;

	vsync_vioc0_disp.vioc_intr = kzalloc(sizeof(struct vioc_intr_type), GFP_KERNEL);
	if (!vsync_vioc0_disp.vioc_intr)
		return -ENOMEM;

	vsync_vioc0_disp.vioc_intr->id = vsync_vioc0_disp.lcdc_num;
	vsync_vioc0_disp.vioc_intr->bits = (1<<VIOC_DISP_INTR_VSF);

	vsync_vioc1_disp.vioc_intr = kzalloc(sizeof(struct vioc_intr_type), GFP_KERNEL);
	if (!vsync_vioc1_disp.vioc_intr)
		return -ENOMEM;

	vsync_vioc1_disp.vioc_intr->id = vsync_vioc1_disp.lcdc_num;
	vsync_vioc1_disp.vioc_intr->bits = (1<<VIOC_DISP_INTR_VSF);
	
#ifdef USE_VSYNC_TIMER
	vsync_timer = tcc_register_timer(NULL, 1000/*msec*/, NULL);
	if (IS_ERR(vsync_timer)) {
		printk("%s: cannot register tcc timer. ret:0x%x\n", __func__, (unsigned)vsync_timer);
		vsync_timer = NULL;
		BUG();
	}
#endif

	if (misc_register(&vsync0_misc_device))
	{
		dprintk(KERN_WARNING "Couldn't register device .\n");
		return -EBUSY;
	}
	
	if (misc_register(&vsync1_misc_device))
	{
		dprintk(KERN_WARNING "Couldn't register device .\n");
		return -EBUSY;
	}
	
	return ret;
}

static int tcc_vsync_remove(struct platform_device *pdev)
{
	int ret=0;
	
	misc_deregister(&vsync0_misc_device);
	misc_deregister(&vsync1_misc_device);

	return ret;
}

#ifdef CONFIG_OF
static struct of_device_id vsync_of_match[] = {
	{ .compatible = "telechips,tcc_vsync" },
	{}
};
MODULE_DEVICE_TABLE(of, vsync_of_match);
#endif

static struct platform_driver tcc_vsync_driver = {
	.probe		= tcc_vsync_probe,
	.remove		= tcc_vsync_remove,
	.driver		= {
		.name	= "tcc_vsync",
		.owner	= THIS_MODULE,
#ifdef CONFIG_PM_RUNTIME
		.pm 	= &tcc_vsync_pm_ops,
#endif
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(vsync_of_match),
#endif

	},
};

static int __init tcc_vsync_init(void)
{
#if defined(CONFIG_HDMI_DISPLAY_LASTFRAME)
	pmap_get_info("fb_wmixer", &fb_lastframe_pbuf);
	printk(" %s wmixer base:0x%08x size:%d\n",__func__,fb_lastframe_pbuf.base, fb_lastframe_pbuf.size );

	spin_lock_init(&LastFrame_lockDisp);

	tcc_video_ctrl_last_frame(0);
#endif
	mutex_init(&vsync_io_mutex);

#ifdef CONFIG_DISPLAY_EXT_FRAME
	mutex_init(&ext_mutex);
#endif
	/*
	DevMajorNum = register_chrdev(0, DEVICE_NAME, &tcc_vsync_fops);
	if (DevMajorNum < 0) {
		printk("%s: device failed widh %d\n", __func__, DevMajorNum);
		err = -ENODEV;
	}
	
	vsync_class = class_create(THIS_MODULE, DEVICE_NAME);
	if (IS_ERR(vsync_class)) {
		err = PTR_ERR(vsync_class);
	}
	
	printk("%s Major(%d)\n", __FUNCTION__,DevMajorNum);
	
	device_create(vsync_class,NULL,MKDEV(DevMajorNum, 0),NULL,DEVICE_NAME);
	*/
	return platform_driver_register(&tcc_vsync_driver);
}

static void __exit tcc_vsync_exit(void)
{
	
	printk("%s\n", __FUNCTION__);
	platform_driver_unregister(&tcc_vsync_driver);
}

module_init(tcc_vsync_init);
module_exit(tcc_vsync_exit);

MODULE_AUTHOR("linux <linux@telechips.com>");
MODULE_DESCRIPTION("Telechips TCC Vsync driver");
MODULE_LICENSE("GPL");
