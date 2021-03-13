
#ifndef _STRUCTURES_VIDEO_H_
#define _STRUCTURES_VIDEO_H_

#include "vpu_type.h"

#if defined(CONFIG_ARCH_TCC893X) || defined(CONFIG_ARCH_TCC897X)
#define HWCLK_BASE        						(0x74000000)
#define HwVBUS_BASE                             (0x75000000)
#if defined(CONFIG_ARCH_TCC893X) 
#define HwVIDEOBUSCONFIG_BASE                   (0x75200000)
#else //897x
#define HwVIDEOBUSCONFIG_BASE                   (0x75100000)
#endif
#elif defined(CONFIG_ARCH_TCC896X)
#define VBUS_QOS_MATRIX_CTL
#define HWCLK_BASE        						(0x14000000)
#define HwVBUS_BASE                             (0x15000000)
#define HwVIDEOBUSCONFIG_BASE                   (0x15200000)
#elif defined(CONFIG_ARCH_TCC898X)
#define HWCLK_BASE        						(0x14000000)
#define HwVBUS_BASE                             (0x15000000)
#define HwVIDEOBUSCONFIG_BASE                   (0x15100000)
#else
#error
#endif


#if defined(CONFIG_ARCH_TCC897X)
#define VBUS 	0x18
#define VCODEC 	0x1C
#define HEVC_C 	0x30
#define HEVC_V 	0x34
#define HEVC_B 	0x38
#elif defined(CONFIG_ARCH_TCC896X)
#define VBUS 	0x14
#define VCODEC 	0x18
#define HEVC_C 	0x34
#define HEVC_V 	0x38
#define HEVC_B 	0x3C
#elif defined(CONFIG_ARCH_TCC898X)
#define VBUS 	0x14
#define VCODEC 	0x30
#define HEVC_C 	0x34
#define HEVC_V 	0x38
#define HEVC_B 	0x3C
#else
#define VBUS 	0x14
#define VCODEC 	0x18
#endif

#endif /* _STRUCTURES_VIDEO_H_ */

