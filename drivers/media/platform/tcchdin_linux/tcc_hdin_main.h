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

#include <asm/scatterlist.h>
#include <soc/tcc/pmap.h>

#include <linux/wakelock.h>
#include <media/videobuf-dma-sg.h>
#include <media/v4l2-common.h>

#include <mach/tcc_hdin_ioctrl.h>
#include <mach/tcc_hdmi_in_parameters.h>
#include <mach/bsp.h>
#include <mach/vioc_ireq.h>
#include <mach/vioc_config.h>
#include <mach/vioc_rdma.h>
#include <mach/vioc_wdma.h>
#include <mach/vioc_wmix.h>
#include <mach/vioc_scaler.h>
#include <mach/vioc_vin.h>
#include <mach/vioc_viqe.h>

#ifndef TCC_HDIN_MAIN__H
#define TCC_HDIN_MAIN__H

#define TCC_HDIN_MAX_BUFNBRS		8
#define ALIGNED_BUFF(buf, mul) ( ( (unsigned int)buf + (mul-1) ) & ~(mul-1) )

#define ON								1
#define OFF 							0

#ifdef HDIN_DRV_BYPASS_EN
#if defined(CONFIG_ARCH_TCC893X)  || defined(CONFIG_ARCH_TCC897X)
#define HDMI_I2SREG(x)				(tcc_p2v(0x72340000) + (x))
#elif defined(CONFIG_ARCH_TCC896X)
#define HDMI_I2SREG(x)				(tcc_p2v(0x12340000) + (x))
#endif
#endif

typedef struct
{
	int (*Open)(void);
	int (*Close)(void);
	int (*GetVideoSize)(unsigned int *uINTL);
	int (*GetAudioSR)(void);
	int (*GetAudioType)(void);
}MODULE_FUNC_TYPE;

typedef struct {
	unsigned int			p_Y;
	unsigned int			p_Cb;
	unsigned int			p_Cr;
}img_buf_t;

typedef struct {
	unsigned short			  target_x;
	unsigned short			  target_y;
}hdin_main_set;

enum hdin_out_fmt {
	YUV420SEP = 0,
	RGB565 = 1,
	ARGB8888 = 2,
};

enum hdin_source_state {
	NORMAL = 0,
	WIFI_DISPLAY = 1,
};

enum hdin_stream_state {
	STREAM_OFF,
	STREAM_INTERRUPT,
	STREAM_ON,
};

struct hdin_gpio {
	int pwr_port;
	int key_port;
	int rst_port;
	int int_port;
	int scan_port;
};

struct tcc_vioc {
	VIOC_RDMA			*rdma_addr;
	VIOC_WMIX			*wmix_addr;
	VIOC_WDMA			*wdma_addr;
	VIOC_VIN			*vin_addr;
	VIOC_SC				*sc_addr;
	VIOC_IREQ_CONFIG	*config_addr;
	VIQE				*viqe_addr;
	DDICONFIG			*ddiconfig_addr;
	unsigned int 		*vin_lut_addr;

	/* plugin info */
	unsigned int sc_plugin;
	unsigned int viqe_plugin;
	unsigned int deintls_plugin;

	/* interrupt */
	unsigned int wdma_irq_mask;
};

struct hdin_dma_buffer {
	unsigned char *area;	/* virtual pointer */
	dma_addr_t addr;	/* physical address */
	size_t bytes;		/* buffer size in bytes */
	void *private_data; /* private for allocator; don't touch */
};

typedef struct hdin_c_t {
	hdin_main_set			main_set;
	dma_addr_t				base_buf;
	int 					bpp;
	unsigned char			pp_num; 	   /* used pingpong memory number */
	img_buf_t				preview_buf[TCC_HDIN_MAX_BUFNBRS];
	enum hdin_out_fmt	   fmt;
	volatile unsigned char	now_frame_num; // current buffer_index
} hdin_cfg_t;

struct tcc_hdin_buffer {
	struct list_head buf_list;
	struct v4l2_buffer v4lbuf;

	int mapcount;
	struct TCC_HDIN *device;
};

struct TCC_HDIN{
	hdin_cfg_t hdin_cfg;
	struct list_head list;
	struct list_head done_list;
	struct tcc_hdin_buffer static_buf[TCC_HDIN_MAX_BUFNBRS];
	struct tcc_hdin_buffer *buf;
	struct hdin_dma_buffer hdin_buf;
	unsigned int n_sbufs;		/* How many we have */

	unsigned int wakeup_int;
	wait_queue_head_t frame_wait;	/* Waiting on frame data */
	spinlock_t dev_lock;  /* Access to device */
	struct mutex	lock;
	enum hdin_stream_state stream_state;
};

struct tcc_vin_core {
	pmap_t pmap_preview;
	pmap_t pmap_viqe;

	struct tcc_hdin_buffer *prev_buf;
	unsigned int prev_num;

	struct tcc_hdin_buffer *next_buf;
	unsigned int next_num;

	/* for s/w zoom */
	unsigned int old_zoom_step;

	/* buffer offset */
	unsigned int offset_total;
	unsigned int offset_y;
	unsigned int offset_uv;
};

struct tcc_hdin_device {
	char name[16];
	struct device *dev;
	struct device_node	*hdin_np;

	int enabled;

	/* video device minor
	 * -1 ==> auto assign, X ==> /dev/videoX
	 */
	int video_nr;
	//struct v4l2_framebuffer fbuf;
	struct v4l2_pix_format pix;

	/* for tcc_vin_core.c
	 */
	struct tcc_vin_core core;

	/* clock
	 */
	struct clk *hdin_clk;

	/* gpio
	 */
	struct hdin_gpio gpio;

	struct video_device *vfd;

	struct TCC_HDIN *h;
	struct tcc_vioc vioc;

	/* mode
	 */
	int bypass;

	/* module
	 */
	unsigned int cur_resolution;
	unsigned int hdmi_in_Interlaced;
	MODULE_FUNC_TYPE module_func;
};

#define DRIVER_NAME		"tcc-hdmi-in"
#define DRIVER_VERSION	"v1.0"
#define DRIVER_AUTHOR	"Telechips.Co.Ltd"
#define DRIVER_DESC		"TCC HDMI Receiver driver"

#endif
