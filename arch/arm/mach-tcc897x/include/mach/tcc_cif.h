#ifndef __TCC_CIF_H__
#define __TCC_CIF_H__

#include <linux/device.h>
#include <linux/i2c.h>
#include <soc/tcc/pmap.h>


/* 
 */
#define CAM_INIT_MT9D111			(1 << 0)
#define CAM_INIT_THCV220			(1 << 1)
#define is_cam_init_by_lk(x)		(tcc_cam_init_by_lk & x)
#define clear_cam_init_by_lk(x)		(tcc_cam_init_by_lk &= ~x)

/* Blackbox 4ch Senario Type
 * ===========================
 * TYPE1: video0,1,2,3 - 4ch tw2867 time-multiplex VD1 (108Mhz)
 * TYPE2: video0,1     - 2ch tw2867 time-multiplex VD1
 *        video2       - 1ch tw2867 single VD2
 *        video4       - 1ch mt9d111
 * TYPE3: video0,1,2,3 - 4ch tw2867 time-multiplex VD1 (54Mhz double-edge)
 * TYPE4: video0,1,2,3 - 4ch tw2867 time-multiplex VD1 (54Mhz)
 * --------------------------
 * TYPE5: video0,1,2,3 - 4ch tvp5158 time-multiplex VD1 (108Mhz)
 * TYPE6: video0       - 1ch tvp5158 single VD1 (27Mhz)
 * TYPE7: video0,1     - 2ch tvp5158 single VD1 (54Mhz)
 * TYPE8: video0,1     - 2ch tvp5158 single VD1 (27Mhz)
 */
//#define BB_4CH_TYPE1	/* tw2867 */
//#define BB_4CH_TYPE2
//#define BB_4CH_TYPE3
//#define BB_4CH_TYPE4
//#define BB_4CH_TYPE5	/* tvp5158 */
//#define BB_4CH_TYPE6
//#define BB_4CH_TYPE7
//#define BB_4CH_TYPE8

/* If you use XTAL
 */
#define USE_MCLK_XTAL

#define CIF_PORT_NULL		0x0FFFFFFF
#define CIF_CTRL_DISABLE	0
#define CIF_CTRL_ENABLE		1
#define CIF_RESET_LOW		0
#define CIF_RESET_HIGH		1
#define CIF_CLK_GET			0x1000
#define CIF_CLK_PUT			0x1001
#define CIF_CLK_SET			0x1002
#define CIF_CLK_ENABLE		0x1003
#define CIF_CLK_DISABLE		0x1004


enum sensor_module_type {
	SENSOR_NONE,
	SENSOR_MT9D112,
	SENSOR_MT9D111,
	SENSOR_OV2643,
	SENSOR_OV5640,
	SENSOR_NT99141,
	SENSOR_MC1080A,
	SENSOR_TVP5150,
	SENSOR_TW2867,
	SENSOR_TVP5158,
	SENSOR_THCV220,
	SENSOR_TW9900,
	SENSOR_ADV7182,
};

enum vioc_component_id {
	/* Video INput */
	VIOC_VIN_00 = 0,
	VIOC_VIN_01 = 1,
	VIOC_VIN_02 = 2,
	VIOC_VIN_03 = 3,

	/* VIN_DEMUX */
	VIOC_VD = 4,
	VIOC_VD_EP_00 = VIOC_VD,	/* Vin_Demux External Port */
	VIOC_VD_EP_01 = 5,
	VIOC_VD_EP_02 = 6,
	VIOC_VD_EP_03 = 7,
	VIOC_VD_MP_00 = 8,			/* Vin_Demux Multiplex Port */
	VIOC_VD_MP_01 = 9,
	VIOC_VD_MP_02 = 10,
	VIOC_VD_MP_03 = 11,

	/* Window (overlay) Mixer */
	VIOC_WMIX_00 = 12,
	VIOC_WMIX_01 = 13,
	VIOC_WMIX_02 = 14,	/* N/A */
	VIOC_WMIX_03 = 15,
	VIOC_WMIX_04 = 16,
	VIOC_WMIX_05 = 17,
	VIOC_WMIX_06 = 18,

	/* WDMA */
	VIOC_WDMA_00 = 19,
	VIOC_WDMA_01 = 20,
	VIOC_WDMA_02 = 21,
	VIOC_WDMA_03 = 22,
	VIOC_WDMA_04 = 23,
	VIOC_WDMA_05 = 24,
	VIOC_WDMA_06 = 25,
	VIOC_WDMA_07 = 26,
	VIOC_WDMA_08 = 27,

	/* SCaler */
	VIOC_SC_00 = 28,
	VIOC_SC_01 = 29,
	VIOC_SC_02 = 30,

	/* VIQE */
	VIOC_VIQE_00 = 31,

	/* DEINTLS */
	VIOC_DEINTLS_00 = 32,

	VIOC_NULL = 0xFF
};

enum tcc_cif_type {
	TCC_CIF0 = 0,
	TCC_CIF1 = 1,
	TCC_CIF2 = 2,
	TCC_CIF3 = 3,
	TCC_CIF4 = 4
};

enum video_dev_nr {
	VIDEO0 = 0,
	VIDEO1 = 1,
	VIDEO2 = 2,
	VIDEO3 = 3,
	VIDEO4 = 4
};

/* WDMA transfer mode:
 * WDMACTRL.CONT register
 */
enum wdma_tansfer_mode {
	WDMA_TRANS_FRAME_BY_FRAME = 0,	/* Frame-by-Frame mode */
	WDMA_TRANS_CONTINUOUS = 1		/* Continuous mode */ 
};

enum tcc_i2c_core_nr {
	/*  i2c core              tcc8925   tcc8920  tcc8923  0xA004  0xA005/6 | tcc8930 */
	I2C_MASTER_CORE0 = 0,	/* I2C_16             I2C_22  I2C_22  I2C_16   |  I2C_08 */
	I2C_MASTER_CORE1 = 1,	/* I2C_28                                      |  I2C_21 */
	I2C_MASTER_CORE2 = 2,	/* I2C_18    I2C_28           I2C_18  I2C_18   |  I2C_15 */
	I2C_MASTER_CORE3 = 3,
	I2C_NONE = 4
};

/* structures for the management of devices that share a one sensor.
 * ex. tw2867
 */
enum shared_sensor_ctrl {
	SHARED_SENSOR_INFO_I2C,
	SHARED_SENSOR_INFO_PORT,
	SHARED_SENSOR_INFO_OPEN,
	SHARED_SENSOR_INFO_VINDEMUX,
	SHARED_SENSOR_SET_I2C,
	SHARED_SENSOR_GET_I2C
};

struct shared_sensor_info {
	unsigned long i2c_status;
	unsigned long port_status;
	unsigned long open_status;
	unsigned long vd_status;
	struct i2c_client *shared_i2c_client;
};

/* structure for the management of pmap reserved memory 
 */
#define MAX_VIDEO_NR	5
struct pmap_used_list {
	int used;
	unsigned int base;
	unsigned int size;
};

struct pmap_v4l2_info {
	pmap_t pmap_ump_reserved;
	unsigned int end_addr;
	unsigned int cur_addr;
	struct pmap_used_list pmap_list[MAX_VIDEO_NR];
};

/*
 */
struct cif_port_conf {
	/* gpio function selection 
	 */
	unsigned int cif_func;	// func select #
	unsigned int pd_bits;	// # of data line bits (default 8bits) 
	unsigned int pd[24];	// cif_data bits
	unsigned int clki;		// clock in
	unsigned int hsync;		// hsync
	unsigned int vsync;		// vsync
	unsigned int dataen;	// data enable
	unsigned int field;		// filed

	/* normal gpio 
	 */
	unsigned int pwr;		// sensor power
	unsigned int pwdn;		// sensor power-down
	unsigned int rst;		// sensor reset
	unsigned int standby;	// sensor stand-by
	unsigned int fl_en;		// sensor flash enable

	/* CLKOUT function selection 
	 */
	unsigned int clko;		// clock out - mclk
	unsigned int clko_func;	// clock out func select #
	const char *clko_name;	// clock out source name

	/* ??? VD_IRQ - TW2867
	 */
	unsigned int irq;

	/* # of i2c_core for sensor control 
	 * refer to tca_i2c_setgpio() of tca_i2c.c
	 */
	unsigned int i2c_core;

#if defined(CONFIG_SENSOR_PLUG_INOUT_DETECT)
	/* detect sensor plug-in/out
	 */
	unsigned int det_irq;
	unsigned int det_gpio;
	unsigned int det_eint;
#endif
};

struct vioc_vin_conf {
	/* id of vioc component 
	 */
	enum vioc_component_id vin;			// video in
	enum vioc_component_id vin_demux;	// vin_demux
	enum vioc_component_id wmix;		// overlay mixer
	enum vioc_component_id wdma;		// wdma
	enum vioc_component_id sc;			// scaler
	enum vioc_component_id viqe;		// viqe
	enum vioc_component_id deintls;		// deintls
	unsigned int vin_demux_idx;			// index of vin_demux

	/* plugin info 
	 */
	unsigned int sc_plugin;
	unsigned int viqe_plugin;
	unsigned int deintls_plugin;

	/* physical address of each component 
	 */
	unsigned int vindemux;
	unsigned int config;
	unsigned int ddi;

	/* wdma transfer mode
	 */
	enum wdma_tansfer_mode wdma_mode;

	/* VSync Error Correction Code
	 */
	unsigned int vsync_ecc;
};

struct tcc_cif_platform_data {
	/* cif port number 
	 */
	enum tcc_cif_type port_nr;
	
	/* cif port config 
	 */
	struct cif_port_conf *port;

	/* vioc-vin path config 
	 */
	struct vioc_vin_conf *vioc;

	/* sensor module type 
	 */
	enum sensor_module_type sensor_type;

	/* name of i2c driver for cif control 
	 */
	const char *i2c_drv_name;

	/* cif hw callbalck functions 
	 */
	void (* cif_port_init)(struct device *dev, int flag, int video_nr);
	void (* cif_port_deinit)(struct device *dev, int flag, int video_nr);
	void (* cif_rst_ctrl)(struct device *dev, int ctrl);
	void (* cif_pwr_ctrl)(struct device *dev, int ctrl);
	void (* cif_pwdn_ctrl)(struct device *dev, int ctrl);
	void (* cif_standby_ctrl)(struct device *dev, int ctrl);
	void (* cif_flen_ctrl)(struct device *dev, int ctrl);
	int (* cif_shared_sensor_ctrl)(int video_nr, enum shared_sensor_ctrl ctrl_id, int ctrl, struct i2c_client *client);
	int (* cif_pmap_ctrl)(int video_nr, pmap_t *pmap, unsigned int width, unsigned int height, int total_buf);
};

#endif /*__TCC_CIF_H__*/
