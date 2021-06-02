
#ifndef _VPU_TYPE_H_
#define _VPU_TYPE_H_

#if defined(CONFIG_CHIP_TCC8935S)
#define VPU_C5
#elif defined(CONFIG_ARCH_TCC897X)
//#define VBUS_CLK_ALWAYS_ON
#define VPU_D6
#else
#define VPU_C7
#endif

#if defined(CONFIG_ARCH_TCC893X)
#define JPU_C5
#else
#define JPU_C6
#endif

#endif /* _VPU_TYPE_H_ */

