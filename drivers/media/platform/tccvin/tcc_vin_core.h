#ifndef __TCC_VIN_CORE_H__
#define __TCC_VIN_CORE_H__

/* Maximum/minumum number of buffers */
#define TCC_V4L2_MAX_BUFNBRS	32
#define TCC_V4L2_MIN_BUFNBRS	8

#define fourcc2char(fourcc) \
        ((char) ((fourcc)     &0xff)), \
        ((char) (((fourcc)>>8 )&0xff)), \
        ((char) (((fourcc)>>16)&0xff)), \
        ((char) (((fourcc)>>24)&0xff))

enum cif_order422 {
	CIF_YCBYCR = 0,
	CIF_YCRYCB,
	CIF_CBYCRY,
	CIF_CRYCBY,
};

enum pix_fmt {
	TCC_PFMT_YUV420,
	TCC_PFMT_YUV422,
	TCC_PFMT_RGB,
};

enum cifoper_mode {
	OPER_PREVIEW = 0,
	OPER_CAPTURE,
};

enum capture_status{
	CAPTURE_DONE = 0,
	CAPTURE_NO_INT,
	CAPTURE_OVERFLOW,
	CAPTURE_NONE
};

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
} cif_main_set;

typedef struct cif_c_t {
	enum cif_order422      	order422;
			
	unsigned char           polarity_pclk;
	unsigned char           polarity_vsync;
	unsigned char           polarity_hsync;

	enum cifoper_mode		oper_mode;
	unsigned char 			zoom_step; // for capture!!
	cif_main_set			main_set;

	/* 2 pingpong Frame memory */
	dma_addr_t            	base_buf;
	unsigned int            pp_totalsize;
	unsigned char           pp_num;        /* used pingpong memory number */
	img_buf_t               preview_buf[TCC_V4L2_MAX_BUFNBRS];
	img_buf_t				capture_buf;

	enum pix_fmt			pfmt;
	int						bpp;

	volatile unsigned char	now_frame_num; // current buffer_index

	volatile unsigned char			esd_restart;

	/* retry capture */
	volatile enum capture_status	cap_status;
	volatile unsigned char			retry_cnt;
} cif_cfg_t;


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

struct TCCxxxCIF{
	cif_cfg_t cif_cfg;

	unsigned int cam_path_wdma_fmt;

	struct timer_list cam_timer;
	unsigned int register_timer;

	struct list_head list;
	struct list_head done_list;

	struct tccxxx_cif_buffer static_buf[TCC_V4L2_MAX_BUFNBRS]; 			
	struct tccxxx_cif_buffer *buf; 			
	struct cif_dma_buffer cif_buf;
	unsigned int n_sbufs;		/* How many we have */	

	unsigned int wakeup_int;
	wait_queue_head_t frame_wait;	/* Waiting on frame data */

	spinlock_t dev_lock;  /* Access to device */	
	unsigned long dev_lock_flag;

	enum tcc_stream_state stream_state;

	unsigned int frame_skip, frame_skipped;
};

extern int	vincore_set_bufs(struct tcc_video_device *vdev, struct v4l2_requestbuffers *req);
extern int  vincore_restart(struct tcc_video_device *vdev);
extern int  vincore_start_stream(struct tcc_video_device *vdev);
extern int  vincore_stop_stream(struct tcc_video_device *vdev);
extern int 	vincore_set_zoom(struct tcc_video_device *vdev, int arg);
extern int 	vincore_set_resolution(struct tcc_video_device *vdev, unsigned int pixel_fmt, unsigned short width, unsigned short height);
extern int 	vincore_open(struct tcc_video_device *vdev);
extern int 	vincore_close(struct tcc_video_device *vdev);
extern int 	vincore_init(struct tcc_video_device *vdev);
extern void vioc_vin_path_mapping(struct tcc_video_device *vdev);

#endif /*__TCC_VIN_CORE_H__*/
