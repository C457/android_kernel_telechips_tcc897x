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

#ifndef TCCXX_CIF_H
#define TCCXX_CIF_H

#include <media/v4l2-common.h>
#if defined(CONFIG_ARCH_TCC898X)
#include <video/tcc/tcc_jpeg_ioctl.h>
#include <video/tcc/vioc_intr.h>
#else
#include <mach/tcc_jpeg_ioctl.h>
#include <mach/vioc_intr.h>
#endif
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

/* Maximum number of buffers */
#define TCC_CAMERA_MAX_BUFNBRS 		14
/* Minumum number of buffers */
#define TCC_CAMERA_MIN_BUFNBRS	8

#define ALIGNED_BUFF(buf, mul) ( ( (unsigned int)buf + (mul-1) ) & ~(mul-1) )

#ifdef CONFIG_VIDEO_ATV_SENSOR_DAUDIO
//#define JPEG_ENCODE_WITH_CAPTURE
#ifdef JPEG_ENCODE_WITH_CAPTURE
#define JPEG_TESTCODE
#endif

#define DAUDIO_CAMERA_REAR		0
#define DAUDIO_CAMERA_CMMB		1
#define DAUDIO_CAMERA_AUX		2
#define DAUDIO_CAMERA_LVDS		3

#define DAUDIO_CMMB_CROP_X             20
#define DAUDIO_CMMB_CROP_Y             20

//#define DMA_ELEM_SIZE   4
//#define FIFO_TRIGGER_LVL (32)
#endif

/*
 * ---------------------------------------------------------------------------
 *  tccxxx Camera Interface
 * ---------------------------------------------------------------------------
 */

enum cif_order422 {
	CIF_YCBYCR = 0,
	CIF_YCRYCB,
	CIF_CBYCRY,
	CIF_CRYCBY,
};

enum cifout_fmt {
	CIF_YUV422 = 0,
	CIF_YUV420_ODD,
	CIF_YUV420_EVEN,
};

enum cifoper_mode {
	OPER_PREVIEW = 0,
	OPER_CAPTURE,
};

enum cif_caminfo_tobeopen{
    CIF_FACING_BACK = 0,
    CIF_FACING_FRONT = 1 /* The camera faces to the user */
};

enum jpeg_resolution{
	CAM_UXGA = 0,     // 1600 X 1200,
	CAM_SXGA,         // 1280 X 1024, //1280 X 960
	CAM_XGA,          // 1024 X 768, 
	CAM_SVGA,         // 800 X 600,   
	CAM_VGA,          // 640 X 480,   
	CAM_QVGA,         // 320 X 240, 
	CAM_QCIF,		  // 176 X 144,
	CAM_MAX
};

enum tcc_effect_type{
	TCC_EFFECT_NORMAL,
	TCC_EFFECT_GRAY,                    
	TCC_EFFECT_NEGATIVE,
	TCC_EFFECT_SEPHIA,
	TCC_EFFECT_AQUA,
	TCC_EFFECT_GREEN,
	TCC_EFFECT_SKETCH,
	TCC_EFFECT_UNDEFINED 
};

typedef struct _ZOOM_DATA
{
	unsigned short XOffset;
	unsigned short YOffset;
	unsigned short Src_HSize;
	unsigned short Src_VSize;    
} ZOOM_DATA;

typedef struct _FRAME_DATA
{
	unsigned short framex;
	unsigned short framey;
	unsigned short cap_x;
	unsigned short cap_y;    
} FRAME_DATA;

typedef struct {
    unsigned int            p_Y;  
    unsigned int            p_Cb;
    unsigned int            p_Cr;
}img_buf_t;

typedef struct {
    unsigned short            source_x; 		/* CIF Input-Selection width/height */
    unsigned short            source_y;
    unsigned short            win_hor_ofst;	/* CIF Input-Selection Hori/Vert Offset (ex. For Zoom)*/
    unsigned short            win_ver_ofst;		

    unsigned short            scaler_x;		/* CIF Scaler out */
    unsigned short            scaler_y;
    unsigned short            target_x;		/* CIF Crop out :: real out */
    unsigned short            target_y;
}cif_main_set;

typedef struct
{
	unsigned short 			chromakey;
	
	unsigned char			mask_r;
	unsigned char			mask_g;
	unsigned char			mask_b;
	
	unsigned char			key_y;
	unsigned char			key_u;
	unsigned char			key_v;
	
}si_chromakey_info;

typedef struct
{
	unsigned short 			start_x;
	unsigned short 			start_y;
	unsigned short 			width;
	unsigned short 			height;
	
	unsigned int 			buff_offset;

	si_chromakey_info		chromakey_info;			
}cif_SuperImpose;

enum capture_status{
	CAPTURE_DONE = 0,
	CAPTURE_NO_INT,
	CAPTURE_OVERFLOW,
	CAPTURE_NONE
};

typedef struct
{
	unsigned int start_phy_addr; // start address of bitstream + thumb + header!!

 	unsigned int header_offset;
	unsigned int thumb_offset;
	unsigned int bitstream_offset;	
	unsigned int header_size;
	unsigned int thumb_size;
	unsigned int bitstream_size;
}jpeg_info;

typedef struct cif_c_t {
	enum cif_order422      	order422;
			
	unsigned char           polarity_pclk;
	unsigned char           polarity_vsync;
	unsigned char           polarity_href;
	unsigned char           polarity_de;
	unsigned int            field_bfield_low;
	unsigned int            gen_field_en;
	unsigned int            conv_en;
	unsigned int            hsde_connect_en;
	unsigned int            vs_mask;
	unsigned int            input_fmt;
	unsigned int            data_order;
	unsigned int            intl_en;
	unsigned int            intpl_en;
	
	enum cifoper_mode		oper_mode;
	unsigned char 			zoom_step; // for capture!!
	cif_main_set			main_set;
		
	/* 2 pingpong Frame memory */
	dma_addr_t            	base_buf;
	unsigned int            pp_totalsize;
	unsigned char           pp_num;        /* used pingpong memory number */
	img_buf_t               preview_buf[TCC_CAMERA_MAX_BUFNBRS];
	img_buf_t				capture_buf;

	cif_SuperImpose			si_overlay;

	jpeg_info				jpg_info;
	enum cifout_fmt         fmt;

	volatile unsigned char	now_frame_num; 

	volatile unsigned char			esd_restart;

	//Retry Capture
	volatile enum capture_status	cap_status;
	volatile unsigned char			retry_cnt;

 //http://gerrit.daudio/#/c/22079/ 2016.6.07 mhjung merge
	int						encode;	// 0: NTSC, 1: PAL  

} cif_cfg_t;

/* NUM_CAMDMA_CHANNELS is the number of logical channels used for
 * DMA data transfer.
 */
#define NUM_CAMDMA_CHANNELS 2
/*
 * info for buffer allocation
 */
struct cif_dma_buffer {
	unsigned char *area;	/* virtual pointer */
	dma_addr_t addr;	/* physical address */
	size_t bytes;		/* buffer size in bytes */
	void *private_data; /* private for allocator; don't touch */
};

enum tcc_cif_buffer_state {
	TCC_CIF_BUF_UNUSED,  /* not used */
	TCC_CIF_BUF_QUEUED,
	TCC_CIF_BUF_USING,    /* currently grabbing / playing */
	TCC_CIF_BUF_GRABBING,
	TCC_CIF_BUF_DONE,     /* done */	
	TCC_CIF_BUF_ERROR,
};


enum tcc_stream_state {
	STREAM_OFF,
	STREAM_INTERRUPT,
	STREAM_ON,
};


/*
 * Tracking of streaming I/O buffers.
 */
struct tccxxx_cif_buffer {
	struct list_head buf_list;
	struct v4l2_buffer v4lbuf;
	enum tcc_cif_buffer_state state;	
	int mapcount;
	unsigned long vma_use_count;		
	struct TCCxxxCIF *cam;	
};

struct VIOC_dt_parse{
	unsigned int  index;	
	unsigned int  *address;		
};

struct TCC_VIOC {
	struct VIOC_dt_parse 	wmixer;
	struct VIOC_dt_parse 	pgl;
	struct VIOC_dt_parse 	vin;
	struct VIOC_dt_parse 	lut;	
	struct VIOC_dt_parse 	wdma;
	struct VIOC_dt_parse 	scaler;
	struct VIOC_dt_parse 	viqe;
	struct VIOC_dt_parse 	deintls;
	struct VIOC_dt_parse 	config;
	struct VIOC_dt_parse 	rdma;
	struct VIOC_dt_parse 	fifo;		
	struct VIOC_dt_parse 	wmixer_out;
	struct VIOC_dt_parse 	disp;
	struct VIOC_dt_parse	disp_rdma;

	unsigned int	sc_channel_num;
	unsigned int	sc_plugin_pos;
};

struct TCCxxxCIF{
	struct vioc_intr_type		vioc_intr;
	enum cif_caminfo_tobeopen	cam_info;
	enum tcc_stream_state 		stream_state;	
	struct mutex				lock;	
	struct timer_list 			cam_timer;
	struct list_head 			list;
	struct list_head 			done_list;
	struct tccxxx_cif_buffer	static_buf[TCC_CAMERA_MAX_BUFNBRS]; 			
	struct tccxxx_cif_buffer	*buf; 			
	struct cif_dma_buffer		cif_buf;		// Memory size 
	wait_queue_head_t 			frame_wait;	/* Waiting on frame data */	
	spinlock_t 					dev_lock;  /* Access to device */	
	cif_cfg_t					cif_cfg;
	unsigned int 				n_sbufs;		/* How many we have */	
	unsigned int 				wakeup_int;
};

struct tcc_camera_device;
extern int 	tccxxx_cif_buffer_set(struct tcc_camera_device * vdev, struct v4l2_requestbuffers *req);
extern int 	tccxxx_cif_start_stream(struct tcc_camera_device * vdev);
extern int  tccxxx_cif_stop_stream(struct tcc_camera_device * vdev);
extern int 	tccxxx_cif_capture(int quality, struct tcc_camera_device * vdev);
extern int 	tccxxx_cif_set_zoom(unsigned char arg, struct tcc_camera_device * vdev);
extern int 	tccxxx_cif_set_resolution(struct tcc_camera_device * vdev, unsigned int pixel_fmt, unsigned short width, unsigned short height);
extern int 	tccxxx_cif_open(struct tcc_camera_device * vdev);
extern int 	tccxxx_cif_close(struct tcc_camera_device * vdev);
extern int 	tccxxx_cif_init(struct tcc_camera_device * vdev);
extern int 	tcc_get_sensor_info(struct tcc_camera_device * vdev, int index);
extern void tccxxx_set_camera_addr(struct tcc_camera_device * vdev, int index, unsigned int addr, unsigned int cameraStatus);
extern void tccxxx_parse_vioc_dt_data(struct tcc_camera_device * vdev);
extern int tccxxx_cif_irq_request(struct tcc_camera_device * vdev);
extern void tccxxx_cif_irq_free(struct tcc_camera_device * vdev);

#endif /* TCCXX_CIF_H */
