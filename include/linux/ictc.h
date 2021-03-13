#ifndef __ICTC_H__
#define __ICTC_H__

#include <linux/device.h>
#include <linux/types.h>

struct ictc_platform_data {
	unsigned int irq_num;
	unsigned int f_in_sel;
};
#endif
