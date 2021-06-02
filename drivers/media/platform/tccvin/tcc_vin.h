/*
 *  drivers/media/video/tccvin/camera_core.h
 *
 * Copyright (C) 2008 Telechips, Inc. 
 * 
 * This package is free software; you can redistribute it and/or modify 
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation. 
 * 
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR 
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED 
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE. 
 */

#ifndef __TCC_VIN_H__
#define __TCC_VIN_H__

#include <media/videobuf-dma-sg.h>
#include <asm/scatterlist.h>
#include <linux/wakelock.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#include <soc/tcc/pmap.h>
#include <mach/bsp.h>
#include <mach/tcc_cif.h>

#include <mach/vioc_vin.h>
#include <mach/vioc_wmix.h>
#include <mach/vioc_wdma.h>
#include <mach/vioc_scaler.h>
#include <mach/vioc_ireq.h>
#include <mach/vioc_viqe.h>
#include <mach/vioc_config.h>

#include <linux/videodev2.h>
#include <media/v4l2-common.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>

#define NUM_FREQS 3

struct vioc_component_info {
	unsigned int id;
	unsigned int idx;
	unsigned int phys_addr;
	unsigned int swrst_bit;
	unsigned int irq;
	unsigned int irq_mask;
};


struct VIOC_dt_parse{
	unsigned int  index;	
};

struct VIOC_Index {
	struct VIOC_dt_parse	wmixer;
	struct VIOC_dt_parse	vin;
	struct VIOC_dt_parse 	vin_demux;
	struct VIOC_dt_parse	wdma;
	struct VIOC_dt_parse	scaler;
	struct VIOC_dt_parse	viqe;
	struct VIOC_dt_parse	config;
	struct VIOC_dt_parse	deintls;
};


struct tcc_vioc {
	VIOC_VIN_DEMUX   *vindemux_addr;
	VIOC_WMIX        *wmix_addr;
	VIOC_WDMA        *wdma_addr;
	VIOC_VIN         *vin_addr;
	VIOC_SC          *sc_addr;
	VIOC_IREQ_CONFIG *config_addr;
	VIQE *viqe_addr;
	DDICONFIG *ddiconfig_addr;
	unsigned int*		pLUTBase;

	struct VIOC_Index vioc_num;

	struct vioc_component_info vin;
	struct vioc_component_info vin_demux;
	struct vioc_component_info wmix;
	struct vioc_component_info wdma;
	struct vioc_component_info sc;
	struct vioc_component_info viqe;
	struct vioc_component_info deintls;

	/* plugin info */
	unsigned int sc_plugin;
	unsigned int viqe_plugin;
	unsigned int deintls_plugin;

	/* for viqe de-interlacer 3d */
	unsigned int di_field_cnt;
	unsigned int di_field_bottom;

	/* wdma transfer mode */
	unsigned int wdma_continuous_mode;

	/* VSync Error Correction Code */
	unsigned int vsync_ecc;
	atomic_t vs_count;
};

struct tcc_vin_core {
	pmap_t pmap_preview;

	struct tccxxx_cif_buffer *prev_buf;
	unsigned int prev_num;

	struct tccxxx_cif_buffer *next_buf;
	unsigned int next_num;

	/* for s/w zoom */
	unsigned int old_zoom_step;

	/* buffer offset */
	unsigned int offset_total;
	unsigned int offset_y;
	unsigned int offset_uv;
};

/*
 * Main sturct of vin driver.
 */
struct tcc_video_device {
//	char name[16];
	char name[32];

	struct device *dev;

	int cif_id;
	int enabled;

	/* video device minor 
	 * -1 ==> auto assign, X ==> /dev/videoX 
	 */
	int video_nr;

	/* clock
	 */
	struct clk *vioc_clk;
	struct clk *cifmc_clk;

	struct video_device *vfd;
	
	struct v4l2_device v4l2_dev;
	
	struct v4l2_framebuffer fbuf;
	struct v4l2_pix_format pix;
	unsigned long xclk;

	/* for tcc_vin_core.c
	 */
	struct tcc_vin_core core;
	unsigned int bfield;
	unsigned int frm_cnt;
	unsigned int field_cnt;
	unsigned int frm_sq;
#define FRM_SQ_ODD	0
#define FRM_SQ_EVEN	1
	unsigned int skip_buf;

	/* vioc components
	 */
	struct tcc_vioc vioc;

	/* handling device module
	 */
	struct TCCxxxCIF *h;

	/* sensor module information
	 */
	struct sensor_info *sinfo;
	enum sensor_module_type sensor_type;
	
	/* i2c_client handle */
	struct i2c_client *cif_i2c_client;

#if defined(CONFIG_SENSOR_PLUG_INOUT_DETECT)
	/* detect sensor plug-in/out
	 */
	unsigned int det_status;
	struct work_struct det_wq;
#endif
};

#endif /*__TCC_VIN_H__*/
