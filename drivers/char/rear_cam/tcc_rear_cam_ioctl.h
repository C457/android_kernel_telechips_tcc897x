#ifndef _RCAM_IOCTL_H_
#define _RCAM_IOCTL_H_
 

typedef struct RCAM_SCALE_OVERSCAN_INFO_ {
	unsigned long overscan_position_x;
	unsigned long overscan_position_y;
	unsigned long overscan_width_factor;
	unsigned long overscan_height_factor;
}RCAM_SCALE_OVERSCAN_INFO, *PRCAM_SCALE_OVERSCAN_INFO;

typedef struct RCAM_LINE_POSITION_INFO_ {
	unsigned long line_x;
	unsigned long line_y;
}RCAM_LINE_POSITION_INFO, *PRCAM_LINE_POSITION_INFO;

typedef struct RCAM_LINE_SIZE_INFO_ {
	unsigned long width;
	unsigned long height;
}RCAM_LINE_SIZE_INFO, *PRCAM_LINE_SIZE_INFO;

typedef struct RCAM_LINE_BUFFER_INFO_ {
	unsigned long index;
	unsigned long rear_linebuf_addr;
	unsigned long buffer_size;
}RCAM_LINE_BUFFER_INFO, *PRCAM_LINE_BUFFER_INFO;

typedef struct RCAM_PREVIEW_POSITION_INFO_ {
	unsigned int preview_position_x;
	unsigned int preview_position_y;
}RCAM_PREVIEW_POSITION_INFO, *PRCAM_PREVIEW_POSITION_INFO;

typedef struct RCAM_PREVIEW_SIZE_INFO_ {
	unsigned int preview_width;
	unsigned int preview_height;
}RCAM_PREVIEW_SIZE_INFO, *PRCAM_PREVIEW_SIZE_INFO;

typedef struct PREVIEW_LINE_POSITION_SIZE_INFO_ {
	RCAM_LINE_POSITION_INFO mLine_position_info;
	RCAM_LINE_SIZE_INFO mLine_size_info;
	RCAM_PREVIEW_POSITION_INFO mPreview_position_info;
	RCAM_PREVIEW_SIZE_INFO mPreview_size_info;
}RCAM_PREVIEW_LINE_POSITION_SIZE_INFO, *PRCAM_PREVIEW_LINE_POSITION_SIZE_INFO;

typedef struct RCAM_LINE_BUFFER_UPDATE_INFO_ {
	unsigned long mBufnum;

	RCAM_LINE_SIZE_INFO mSize;
	RCAM_LINE_POSITION_INFO mPos;
}RCAM_LINE_BUFFER_UPDATE_INFO, *PRCAM_LINE_UFFER_UPDATE_INFO;

typedef struct {
	unsigned int captureAddr;
	int frameCnt;
	unsigned int status;			// 0 : las frame
} TCC_REAR_CAMERA_CAPTURE_INFO;

typedef struct {
	unsigned int width;
	unsigned int height;
	unsigned int fmt;
	unsigned int bpp;
	unsigned long pg_offset;
	void * saveAddr;
} TCC_REAR_CAMERA_PREVIEW_INFO;

typedef struct {
	TCC_REAR_CAMERA_PREVIEW_INFO previewInfo;
	unsigned int maxIdx;
	unsigned int firstIdx;
	int status;

} TCC_REAR_CAMERA_CONTINUOUS_PREVIEW_CAPTURE_INFO;


#define             IOCTL_RCAM_MAGIC         'R'
#define             RCAM_DIRECT_ON              		_IO( IOCTL_RCAM_MAGIC, 0)
#define             RCAM_DIRECT_OFF            		_IO( IOCTL_RCAM_MAGIC, 1)
#define             RCAM_LINE_ON              			_IO( IOCTL_RCAM_MAGIC, 2)
#define             RCAM_LINE_OFF            			_IO( IOCTL_RCAM_MAGIC, 3)
#define             RCAM_GET_REAR_STATUS      	_IOR( IOCTL_RCAM_MAGIC, 4, unsigned long)
#define             RCAM_RELEASE_VIDEO	       	_IO( IOCTL_RCAM_MAGIC, 5)
#define             RCAM_LINEBUF_ADDR	       	_IOR( IOCTL_RCAM_MAGIC, 6, unsigned long)
#define             RCAM_WAIT_REAR_EVENT	      	_IOR( IOCTL_RCAM_MAGIC, 7, unsigned long)
#define             RCAM_REAR_EVENT_SET            	_IO( IOCTL_RCAM_MAGIC, 8)
#define		        RCAM_REAR_EVENT_UNSET		_IO( IOCTL_RCAM_MAGIC, 9)
#define             RCAM_LINE_UPDATE			_IOW( IOCTL_RCAM_MAGIC, 12, RCAM_LINE_BUFFER_UPDATE_INFO)
#if 0
#define		   RCAM_SET_PREVIEW_CONFIG	_IOW( IOCTL_RCAM_MAGIC, 13, unsigned long)
#define		   RCAM_PREVIEW_UPDATE     		_IOW( IOCTL_RCAM_MAGIC, 14, unsigned long)
#endif
#define             RCAM_SCALE_OVERSCAN         _IOW( IOCTL_RCAM_MAGIC, 16, RCAM_SCALE_OVERSCAN_INFO)

#define		   RCAM_LINE_SET_POSITION    _IOW(IOCTL_RCAM_MAGIC, 17, RCAM_LINE_POSITION_INFO)
#define		   RCAM_LINEBUF_ADDR_WITH_INDEX _IOWR(IOCTL_RCAM_MAGIC, 18, RCAM_LINE_BUFFER_INFO)
#define		   RCAM_LINE_SET_SIZE				_IOW(IOCTL_RCAM_MAGIC, 19, RCAM_LINE_SIZE_INFO)

#define		   RCAM_PREVIEW_SET_POSITION		_IOW(IOCTL_RCAM_MAGIC, 20, RCAM_PREVIEW_POSITION_INFO)
#define		   RCAM_PREVIEW_SET_SIZE			_IOW(IOCTL_RCAM_MAGIC, 21, RCAM_PREVIEW_SIZE_INFO)

#define		RCAM_HANDOVER				_IOW (IOCTL_RCAM_MAGIC, 28, int)
#endif  // _RCAM_IOCTL_H_



