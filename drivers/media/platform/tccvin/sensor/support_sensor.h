#ifndef __SUPPORT_SENSOR_H__
#define __SUPPORT_SENSOR_H__

#include <linux/errno.h>

#define ERR_MSG(x) \
	{ \
		printk("[vin-hw] error! %s was not selected in the Kconfig\n", x); \
		return -ENODEV; \
	}

/*---------------------------------------------------------------------------------
 * supported sensor module init functions
 *---------------------------------------------------------------------------------
 */

/* support camera modues
 */
#if defined(CONFIG_VIDEO_CAMERA_MT9D111)
extern void sensor_init_mt9d111(struct sensor_info *sinfo);
#else
#define sensor_init_mt9d111(x) ERR_MSG("mt9d111")
#endif

#if defined(CONFIG_VIDEO_CAMERA_MT9D112)
extern void sensor_init_mt9d112(struct sensor_info *sinfo);
#else
#define sensor_init_mt9d112(x) ERR_MSG("mt9d112")
#endif

#if defined(CONFIG_VIDEO_CAMERA_OV2643)
extern void sensor_init_ov2643(struct sensor_info *sinfo);
#else
#define sensor_init_ov2643(x) ERR_MSG("ov2643")
#endif

#if defined(CONFIG_VIDEO_CAMERA_OV5640)
extern void sensor_init_ov5640(struct sensor_info *sinfo);
#else
#define sensor_init_ov5640(x) ERR_MSG("ov5640")
#endif

#if defined(CONFIG_VIDEO_CAMERA_NT99141)
extern void sensor_init_nt99141(struct sensor_info *sinfo);
#else
#define sensor_init_nt99141(x) ERR_MSG("nt99141")
#endif

/* support decoder modules
 */
#if defined(CONFIG_VIDEO_DECODER_TVP5150)
extern void sensor_init_tvp5150(struct sensor_info *sinfo);
//#define CAM_I2C_NAME "tcc_camera,0xB8"
#define	MODULE_NODE	"tvp5150_atv"
#else
#define sensor_init_tvp5150(x) ERR_MSG("tvp5150")
#endif

#if defined(CONFIG_VIDEO_DECODER_TW2867)
extern void sensor_init_tw2867(struct sensor_info *sinfo);
#else
#define sensor_init_tw2867(x) ERR_MSG("tw2867")
#endif

#if defined(CONFIG_VIDEO_DECODER_TVP5158)
extern void sensor_init_tvp5158(struct sensor_info *sinfo);
#else
#define sensor_init_tvp5158(x) ERR_MSG("tvp5158")
#endif

#if defined(CONFIG_VIDEO_DECODER_THCV220)
extern void sensor_init_thcv220(struct sensor_info *sinfo);
#else
#define sensor_init_thcv220(x) ERR_MSG("thcv220")
#endif

#if defined(CONFIG_VIDEO_DECODER_TW9900)
extern void sensor_init_tw9900(struct sensor_info *sinfo);
#else
#define sensor_init_tw9900(x) ERR_MSG("tw9900")
#endif

#if defined(CONFIG_VIDEO_DECODER_ADV7182)
extern void sensor_init_adv7182(struct sensor_info *sinfo);
#define MODULE_NODE "adv7182_atv"
#else
#define sensor_init_adv7182(x) ERR_MSG("adv7182")
#endif

#endif /*__SUPPORT_SENSOR_H__*/
