
#ifndef __SENSOR_IF_H__
#define __SENSOR_IF_H__

#include <linux/videodev2.h>
#include "tcc_vin.h"

#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>


#define ACT_HIGH			1
#define ACT_LOW				0
#define NEGATIVE_EDGE		1
#define POSITIVE_EDGE		0

#define DE_HS_NORMAL		0
#define DE_FROM_HS_CONNECT	1

#define ZOOM_NOT_USE		0
#define ZOOM_USE_SENSOR		1
#define ZOOM_USE_SW			2
#define ZOOM_USE_SW_MAX_VAL 24

/*---------------------------------------------------------------
 * VINDEMUX channel & clock info
 *-------------------------------
 */
/* # of channel */
#define VINDEMUX_CH_MASK				0xf0
#define VINDEMUX_CH_EXT1				0x00
#define VINDEMUX_CH_MUL2				0x10
#define VINDEMUX_CH_MUL2_EXT1			0x20
#define VINDEMUX_CH_MUL4				0x30
/* clock mode (27mhz reference) */
#define VINDEMUX_CLK_MASK				0x0f
#define VINDEMUX_CLK_DOUBLE_EDGE		0x00	// 27mhz double edge
#define VINDEMUX_CLK_DOUBLE_FREQ		0x01	// 27mhz * 2 = 57mhz
#define VINDEMUX_CLK_DOUBLE_EDGE_FREQ	0x02	// 27mhz * 2 = 57mhz double edge
#define VINDEMUX_CLK_4TIMES_FREQ		0x03	// 27mhz * 4 = 108mhz

/* VIN DEMUX TYPE 
 */
#define VINDEMUX_MODE_1CH_D1_27MHZ		(VINDEMUX_CH_EXT1 | VINDEMUX_CLK_DOUBLE_EDGE)	// single direct channel
#define VINDEMUX_MODE_2CH_D1_27MHZ		(VINDEMUX_CH_MUL2 | VINDEMUX_CLK_DOUBLE_EDGE)	// two time-multiplex channel
#define VINDEMUX_MODE_4CH_D1_108MHZ		(VINDEMUX_CH_MUL4 | VINDEMUX_CLK_4TIMES_FREQ)	// four time-multiplex channel

/* 2ch time-multiplex channel (27mhz double edge) + 1ch single direct channel */
#define VINDEMUX_MODE_2CH_1CH_D1_27MHZ	(VINDEMUX_CH_MUL2_EXT1 | VINDEMUX_CLK_DOUBLE_EDGE)
/* extra type for test */
#define VINDEMUX_MODE_2CH_D1_54MHZ		(VINDEMUX_CH_MUL2 | VINDEMUX_CLK_DOUBLE_FREQ)
#define VINDEMUX_MODE_4CH_D1_54MHZ		(VINDEMUX_CH_MUL4 | VINDEMUX_CLK_DOUBLE_EDGE_FREQ)
#define VINDEMUX_MODE_4CH_CIF_54MHZ		(VINDEMUX_CH_MUL4 | VINDEMUX_CLK_DOUBLE_FREQ)
/*---------------------------------------------------------------*/

#define VIN_Y2R_NOT_USE		0x00
#define VIN_Y2R_MODE0		0x10
#define VIN_Y2R_MODE1		0x11
#define VIN_Y2R_MODE2		0x12
#define VIN_Y2R_MODE3		0x13
#define WDMA_R2Y_NOT_USE	0x00
#define WDMA_R2Y_MODE0		0x10
#define WDMA_R2Y_MODE1		0x11
#define WDMA_R2Y_MODE2		0x12
#define WDMA_R2Y_MODE3		0x13

enum image_size {
	QQXGA,
	QXGA,
	UXGA,
	SXGA,
	XGA,
	SVGA,
	VGA,
	QVGA,
	QCIF
};

enum vin_interface_type {
	BT601 = 0,
	BT656 = 1
};

enum vin_scan_type {
	SCAN_PROGRESSIVE	= 0,
	SCAN_INTERLACE		= 1
};

enum vin_de_interlace {
	DEINTL_NOT_USE		= 0,
	DEINTL_ONE_FIELD	= 1,
	DEINTL_SIMPLE		= 2,
	DEINTL_VIQE_2D		= 3,
	DEINTL_VIQE_3D		= 4,
	DEINTL_WDMA_AUTO	= 5,
	DEINTL_WDMA_MANUAL	= 6
};

enum vin_frame_skip {
	FRAME_NOT_SKIP	= 0,
	FRAME_SKIP		= 1		// 1 frame skip
};

enum sensor_init_mode {
	SINIT_NORMAL		= 0,
	SINIT_ZOMBI			= 1,
	SINIT_ZOMBI_OPEN	= 2,
};

#if 0
extern static VIOC_WMIX*			pWMIXBase;
extern static VIOC_WDMA*			pWDMABase;
extern static VIOC_VIN*			pVINBase;
extern static VIOC_SC* 			pSCBase;
extern static VIQE*				pVIQEBase;
extern static VIOC_IREQ_CONFIG*	pVIOCConfig;
extern static unsigned int*		pLUTBase;
#endif

struct capture_size {
	unsigned long width;
	unsigned long height;
};

struct v4l2_ctrl_t {
	struct v4l2_queryctrl qc;
	int current_value;
	int need_set;
};

/*-------------------------------------------------------------------
 * struct sensor_info
 * -----------------------------------------
 *      Configuration of sensor module
 * -----------------------------------------
 *               name - Sensor name
 *       nr_data_bits - # of input data bits
 *                      You can set this vlaue in the tcc_cif_port_setup() (port->pd_bits)
 *           i2c_addr - i2c slave address (7bit addr)
 *       sensor_sizes - Support sensor size
 *              prv_w - Preview width
 *              prv_h - Preview height
 *          framerate - Default framerate
 *               mclk - Master clock
 *     input_data_fmt - Input data format
 *                      FMT_YUV422_16BIT
 *                      FMT_YUV422_8BIT
 *                      FMT_YUVK4444_16BIT
 *                      FMT_YUVK4224_24BIT
 *                      FMT_RGBK4444_16BIT
 *                      FMT_RGB444_24BIT
 *   input_data_order - Input data order
 *                      ORDER_RGB: EP[23:00] == IP[23:00] 
 *                      ORDER_RBG: EP[23:00] == {IP[23:16], IP[07:00], IP[15:08]}
 *                      ORDER_GRB: EP[23:00] == {IP[15:08], IP[23:16], IP[07:00]}
 *                      ORDER_GBR: EP[23:00] == {IP[15:08], IP[07:00], IP[23:16]}
 *                      ORDER_BRG: EP[23:00] == {IP[07:00], IP[23:16], IP[15:08]}
 *                      ORDER_BGR: EP[23:00] == {IP[07:00], IP[15:08], IP[23:16]}
 *                      * EP: External Port, IP: Input Port
 *                      * Base port mapping (EP[23:00] == RGB or YUV)
 *       vin_y2r_mode - [VIN] YCbCr to RGB format conversion option
 *                      VIN_Y2R_NOT_USE: disable Y2R conversion
 *                      VIN_Y2R_MODE0: Studio Color - Normally SDTV
 *                      VIN_Y2R_MODE1: Computer System Color - Normally SDTV
 *                      VIN_Y2R_MODE2: Studio Color - Normally HDTV
 *                      VIN_Y2R_MODE3: Computer System COlor - Normally HDTV
 *      wdma_r2y_mode - [WDMA] RGB to YCbCr format comversion option
 *                      WDMA_R2Y_NOT_USE: disable Y2R conversion
 *                      WDMA_R2Y_MODE0: Studio Color - Normally SDTV
 *                      WDMA_R2Y_MODE1: Computer System Color - Normally SDTV
 *                      WDMA_R2Y_MODE2: Studio Color - Normally HDTV
 *                      WDMA_R2Y_MODE3: Computer System COlor - Normally HDTV
 *      polarity_pclk - Pclk (pixel clock) polarity
 *     polarity_vsync - Vsync polarity
 *     polarity_hsync - Hsync polarity
 *           de2hsync - Use DE signal (connect DE signal from HS signal)
 *                      DE_HS_NORMAL: DE-DE, HS-HS normal connection
 *                      DE_FROM_HS_CONNECT: connect DE signal from HS signal (Sensor HS -> VIN DE)
 *            intplen - Converting from format 4:2:2 to 4:4:4 module interplates the chrominace channel (default disable)
 *     interface_type - ITU-R BT. type (BT601 or BT656)
 *          scan_type - Scan type
 *                      SCAN_PROGRESSIVE: progressive video input
 *                      SCAN_INTERLACE: interlace video input. 
 *                                      wdma write even field and odd field to the buffer.
 *       de-interlace - De-interlace mode (converts an imputted image from interlace to progressive)
 *                      DEINTL_ONE_FIELD: simply one-field (even or odd) scaling.
 *                      DEINTL_SIMPLE: using VIOC simple de-interlacer
 *                      DEINTL_VIQE_2D: using VIQE de-interlacer 2D style 
 *                      DEINTL_VIQE_3D: using VIQE de-interlacer 3D style
 *                      DEINTL_WDMA_AUTO: using WDMA INTL register (only test)
 *                      DEINTL_WDMA_MANUAL: manually processing such as DEINTL_WDMA_AUTO 
 *     frame_skip_vin - Frame skip using VIN control register (VIN.CTRL.skip)
 *                      FRAME_NOT_SKIP: not use this option.
 *                      FRAME_SKIP: skip one frame in the VIN component.
 *     frame_skip_irq - Frame skip in the vin_irq_handler()
 *                      FRAME_NOT_SKIP: not use this option.
 *                      FRAME_SKIP: skip one frame in the vin_irq_handler().
 *          zoom_type - Select zoom type:
 *                      ZOOM_NOT_USE: not use zoom
 *                      ZOOM_USE_SENSOR: sensor h/w zoom (if sensor is supported)
 *                      ZOOM_USE_SW: s/w zoom
 *        prv_zoffx,y - If zoom_type is s/w, zoom x,y offset
 *            private - Private value
 *-------------------------------------------------------------------
 */
struct sensor_info {
	/* basic info */
	char *name;
	unsigned int nr_data_bits;	// # of input data bits
	unsigned int i2c_addr;
	struct capture_size *sensor_sizes;
	unsigned int prv_w;
	unsigned int prv_h;
	unsigned int framerate;
	unsigned int mclk;
	unsigned int input_data_fmt;
	unsigned int input_data_order;
	unsigned int vin_y2r_mode;
	unsigned int wdma_r2y_mode;
	unsigned char polarity_pclk;
	unsigned char polarity_vsync;
	unsigned char polarity_hsync;
	unsigned char de2hsync;
	unsigned int intplen;
	enum vin_interface_type interface_type;
	enum vin_scan_type scan_type;
	enum vin_de_interlace de_interlace;
	enum vin_frame_skip frame_skip_vin;
	enum vin_frame_skip frame_skip_irq;

	/* zoom */
	unsigned int zoom_type;
	unsigned int prv_zoffx;
	unsigned int prv_zoffy;

	/* callback func */
	int (*open)(struct tcc_video_device *vdev);
	int (*close)(struct tcc_video_device *vdev);
	int (*powerdown)(struct tcc_video_device *vdev);
	int (*set_preview)(struct tcc_video_device *vdev);

	/* sensor tunning */
	int (* v4l2_set_ctrl)(struct tcc_video_device *vdev, __u32 id, int val);
	unsigned int need_new_set;
	struct v4l2_ctrl_t *v4l2_ctrl;
	unsigned int n_v4l2_ctrl;
	unsigned int *v4l2_re_ctrl;
	unsigned int n_v4l2_re_ctrl;

	/* when you close the driver, 
	 * dose not turn off the power of sensor.
	 */
	enum sensor_init_mode init_mode;

	/* private */
	unsigned int private;
};

struct sensor_gpio {
	int pwr_port;
	int pwd_port;
	int rst_port;
};


/*---------------------------------------------------------------------------------
 * sensor interface functions
 *---------------------------------------------------------------------------------
 */
extern int sensor_if_change_mode(struct tcc_video_device *vdev, unsigned char mode);
extern int sensor_if_query_control(struct tcc_video_device *vdev, struct v4l2_queryctrl *qc);
extern int sensor_if_get_control(struct tcc_video_device *vdev, struct v4l2_control *vc);
extern int sensor_if_set_control(struct tcc_video_device *vdev, struct v4l2_control *vc, unsigned char init);
extern int sensor_if_check_control(struct tcc_video_device *vdev);
extern int sensor_if_set_current_control(struct tcc_video_device *vdev);
extern int sensor_if_enum_pixformat(struct v4l2_fmtdesc *fmt);
extern int sensor_if_mapping(struct sensor_info *sinfo, unsigned int sensor_type);
extern int sensor_if_init(struct tcc_video_device *vdev);
extern int sensor_if_deinit(struct tcc_video_device *vdev);
extern int sensor_if_get_sensor_framerate(struct tcc_video_device *vdev, int *nFrameRate);
extern int sensor_if_restart(struct tcc_video_device *vdev);

/*---------------------------------------------------------------------------------
 * wrapper functions for cif h/w contrl
 *---------------------------------------------------------------------------------
 */
extern void cif_open(struct tcc_video_device *vdev);
extern void cif_close(struct tcc_video_device *vdev);
extern void cif_power_enable(struct tcc_video_device *vdev);
extern void cif_power_disable(struct tcc_video_device *vdev);
extern void cif_powerdown_enable(struct tcc_video_device *vdev);
extern void cif_powerdown_disable(struct tcc_video_device *vdev);
extern void cif_reset_high(struct tcc_video_device *vdev);
extern void cif_reset_low(struct tcc_video_device *vdev);
extern void cif_set_port(struct tcc_video_device *vdev);

extern void sensor_power_enable(void);
extern void sensor_power_disable(void);
extern int sensor_get_powerdown(void);
extern void sensor_powerdown_enable(void);
extern void sensor_powerdown_disable(void);
extern void sensor_reset_high(void);
extern void sensor_reset_low(void);

#if defined(CONFIG_SENSOR_PLUG_INOUT_DETECT)
/*---------------------------------------------------------------------------------
 * detect sensor plug-in/out
 *---------------------------------------------------------------------------------
 */
extern int detect_sensor_init(struct tcc_video_device *vdev);
extern void detect_sensor_deinit(struct tcc_video_device *vdev);
#endif

extern void tccxxx_vioc_dt_parse_data(struct device_node *dev_node, struct tcc_video_device *vdev);

#endif /*__SENSOR_IF_H__*/
