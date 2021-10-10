#ifndef __DAUDIO_DEBUG___
#define __DAUDIO_DEBUG___


#include <linux/device.h>
#include <linux/platform_device.h>


/**
 * DEBUG_XXXX rule
 * 0: Disable, 1: Enable
 */
#define DEBUG_DAUDIO_PINCTL	0			
#define TAG_DAUDIO_PINCTL			"[gpio_pinctl] "

#define DEBUG_DAUDIO_LVDS			0
#define TAG_DAUDIO_LVDS				"[daudio lvds] "

#define DEBUG_DAUDIO_ATV	0	
#define TAG_DAUDIO_ATV				"[daudio_atv] "
#define TAG_DAUDIO_TW9912			"[tw9912] "
#define TAG_DAUDIO_TW8836			"[tw8836] "
#define TAG_DAUDIO_TW9921			"[tw9921] "
#define TAG_DAUDIO_TW9990			"[tw9990] "

#define DEBUG_DAUDIO_CMMB			1
#define TAG_DAUDIO_CMMB				"[cmmb] "

#define CLL_RESERVED0	0
#define CLL_RESERVED1	1
#define CLL_CRITICAL	2
#define CLL_ERROR		3
#define CLL_WARNING		4
#define CLL_INFO		5
#define CLL_DEBUG		6
#define CLL_TRACE		7

#define CLOG_INIT(TagOrganization, TagDevice, Level)\
static int sCLogLevel=Level;\
static char *spCLogTagOrganization=TagOrganization;\
static char *spCLogTagDevice=TagDevice;\
\
static ssize_t show_clog(struct device *dev, struct device_attribute *attr, char *buf)\
{\
	return sprintf(buf, "\n[%s] clog level=%d\n", dev_name(dev), sCLogLevel);\
}\
\
static ssize_t store_clog(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)\
{\
	int cll;\
	if( 1==sscanf(buf, "%d", &cll) ){\
		if( 0<=cll && cll<=CLL_TRACE ){\
			sCLogLevel=cll;\
			return count;\
		}\
	}\
\
	return -EINVAL;\
}\
\
static DEVICE_ATTR(clog, S_IRUSR|S_IWUSR, show_clog, store_clog);\
\
static inline int device_create_file_clog(struct device *dev)\
{\
	return device_create_file(dev, &dev_attr_clog);\
}\
\
static inline void device_remove_file_clog(struct device *dev)\
{\
	device_remove_file(dev, &dev_attr_clog);\
}\


#define CLOG(level, format, arg...) do{ if( level<=sCLogLevel ) printk(KERN_INFO"[%s %s]"format, spCLogTagOrganization, spCLogTagDevice, ##arg); }while(0)


#define CLOG_TAG_ORGANIZATION	"tcc"
#define CLOG_TAG_DAUDIO		"daudio"

#define CLOG_TAG_UART			"uart"
#define CLOG_LEVEL_UART			CLL_ERROR

#define CLOG_TAG_CM3 			"cm3"
#define CLOG_LEVEL_CM3 			CLL_ERROR

#define CLOG_TAG_MMC			"mmc"
#define CLOG_LEVEL_MMC			CLL_ERROR

#define CLOG_TAG_EYEPATTERN		"eyepattern"
#define CLOG_LEVEL_EYEPATTERN	CLL_ERROR

#define CLOG_TAG_IE				"ie"
#define CLOG_LEVEL_IE			CLL_ERROR

#define CLOG_TAG_IE_TCC			"ie_tcc"
#define CLOG_LEVEL_IE_TCC		CLL_ERROR

#define CLOG_TAG_IE_TWXXXX		"ie_twxxxx"
#define CLOG_LEVEL_IE_TWXXXX	CLL_ERROR

#endif // __DAUDIO_DEBUG__

