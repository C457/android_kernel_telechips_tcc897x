#include <linux/module.h>

#include <asm/io.h>
#include <linux/earlysuspend.h>
#include <linux/fs.h>
#include <linux/major.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include <mach/daudio.h>
#include <mach/daudio_debug.h>
#include <mach/daudio_ie.h>
#include <mach/daudio_ie_lut.h>
#include <mach/daudio_settings.h>

#include "daudio_ie_twxxxx.h"
#include "daudio_ie_tcc.h"

#define DRIVER_NAME            "display_image_enhancement"
#define SYS_DRIVER_NAME        "daudio_ie"

CLOG_INIT(CLOG_TAG_DAUDIO, CLOG_TAG_IE, CLOG_LEVEL_IE);

#define CHECK_SETTING_VALUE        1

//for debug
typedef struct
{
    int mode;
    char *descrption;
} IE_mode_name;

#define MAX_MODE_NAME_SIZE    GET_TCC_USB_SATURATION    // GT system 48
 
static IE_mode_name mode_name_list[] =
{
    {SET_BRIGHTNESS                , "SET_BRIGHTNESS"},
    {SET_CONTRAST                , "SET_CONTRAST"},
    {SET_HUE                    , "SET_HUE"},
    {SET_SATURATION                , "SET_SATURATION"},
    {SET_CAM_BRIGHTNESS            , "SET_CAM_BRIGHTNESS"},
    {SET_CAM_CONTRAST            , "SET_CAM_CONTRAST"},
    {SET_CAM_HUE                , "SET_CAM_HUE"},
/*
    {SET_CAM_SHARPNESS            , "SET_CAM_SHARPNESS"},
    {SET_CAM_U_GAIN             , "SET_CAM_U_GAIN"},
    {SET_CAM_V_GAIN             , "SET_CAM_V_GAIN"},
*/
    {SET_CAM_SATURATION            , "SET_CAM_SATURATION"},

    {SET_AUX_BRIGHTNESS         , "SET_TCC_AUX_BRIGHTNESS"},
    {SET_AUX_CONTRAST            , "SET_TCC_AUX_CONTRAST"},    
    {SET_AUX_GAMMA                , "SET_TCC_AUX_GAMMA"},
    {SET_AUX_SATURATION         , "SET_TCC_AUX_SATURATION"},
    
    {SET_TCC_BRIGHTNESS            , "SET_TCC_BRIGHTNESS"},
    {SET_TCC_CONTRAST            , "SET_TCC_CONTRAST"},
    {SET_TCC_HUE                , "SET_TCC_HUE"},
    {SET_TCC_SATURATION            , "SET_TCC_SATURATION"},
    {SET_TCC_VIDEO_BRIGHTNESS    , "SET_TCC_VIDEO_BRIGHTNESS"},
    {SET_TCC_VIDEO_CONTRAST        , "SET_TCC_VIDEO_CONTRAST"},
    {SET_TCC_VIDEO_GAMMA        , "SET_TCC_VIDEO_GAMMA"},
    {SET_TCC_VIDEO_SATURATION    , "SET_TCC_VIDEO_SATURATION"},
    {SET_TCC_VIDEO_BRIGHTNESS2    , "SET_TCC_VIDEO_BRIGHTNESS2"},
    {SET_TCC_VIDEO_CONTRAST2    , "SET_TCC_VIDEO_CONTRAST2"},
    {SET_TCC_VIDEO_GAMMA2        , "SET_TCC_VIDEO_GAMMA2"},
    {SET_TCC_VIDEO_SATURATION2    , "SET_TCC_VIDEO_SATURATION2"},

    {SET_TCC_DMB_BRIGHTNESS        , "SET_TCC_DMB_BRIGHTNESS"},
    {SET_TCC_DMB_CONTRAST        , "SET_TCC_DMB_CONTRAST"},
    {SET_TCC_DMB_GAMMA            , "SET_TCC_DMB_GAMMA"},
    {SET_TCC_DMB_SATURATION        , "SET_TCC_DMB_SATURATION"},

    {SET_TCC_CMMB_BRIGHTNESS    , "SET_TCC_CMMB_BRIGHTNESS"},
    {SET_TCC_CMMB_CONTRAST        , "SET_TCC_CMMB_CONTRAST"},
    {SET_TCC_CMMB_GAMMA            , "SET_TCC_CMMB_GAMMA"},
    {SET_TCC_CMMB_SATURATION    , "SET_TCC_CMMB_SATURATION"},

    {SET_TCC_USB_BRIGHTNESS        , "SET_TCC_USB_BRIGHTNESS"},
    {SET_TCC_USB_CONTRAST        , "SET_TCC_USB_CONTRAST"},
    {SET_TCC_USB_GAMMA            , "SET_TCC_USB_GAMMA"},
    {SET_TCC_USB_SATURATION        , "SET_TCC_USB_SATURATION"},

    {SET_TCC_ISDB_BRIGHTNESS        , "SET_TCC_ISDB_BRIGHTNESS"},
    {SET_TCC_ISDB_CONTRAST        , "SET_TCC_ISDB_CONTRAST"},
    {SET_TCC_ISDB_GAMMA            , "SET_TCC_ISDB_GAMMA"},
    {SET_TCC_ISDB_SATURATION        , "SET_TCC_ISDB_SATURATION"},

    {GET_BRIGHTNESS                , "GET_BRIGHTNESS"},
    {GET_CONTRAST                , "GET_CONTRAST"},
    {GET_HUE                    , "GET_HUE"},
    {GET_SATURATION                , "GET_SATURATION"},
    {GET_CAM_BRIGHTNESS            , "GET_CAM_BRIGHTNESS"},
    {GET_CAM_CONTRAST            , "GET_CAM_CONTRAST"},
    {GET_CAM_HUE                , "GET_CAM_HUE"},
    /*{GET_CAM_SHARPNESS            , "GET_CAM_SHARPNESS"},
    {GET_CAM_U_GAIN             , "GET_CAM_U_GAIN"},
    {GET_CAM_V_GAIN                , "GET_CAM_V_GAIN"},
    */
    {GET_CAM_SATURATION         , "GET_CAM_SATURATION"},
    
    {GET_AUX_BRIGHTNESS     , "GET_TCC_AUX_BRIGHTNESS"},
    {GET_AUX_CONTRAST        , "SET_TCC_AUX_CONTRAST"},    
    {GET_AUX_GAMMA            , "GET_TCC_AUX_GAMMA"},
    {GET_AUX_SATURATION     , "GET_TCC_AUX_SATURATION"},
    
    {GET_TCC_BRIGHTNESS            , "GET_TCC_BRIGHTNESS"},
    {GET_TCC_CONTRAST            , "GET_TCC_CONTRAST"},
    {GET_TCC_HUE                , "GET_TCC_HUE"},
    {GET_TCC_SATURATION            , "GET_TCC_SATURATION"},
    {GET_TCC_VIDEO_BRIGHTNESS    , "GET_TCC_VIDEO_BRIGHTNESS"},
    {GET_TCC_VIDEO_CONTRAST        , "GET_TCC_VIDEO_CONTRAST"},
    {GET_TCC_VIDEO_GAMMA        , "GET_TCC_VIDEO_GAMMA"},
    {GET_TCC_VIDEO_SATURATION    , "GET_TCC_VIDEO_SATURATION"},
    {GET_TCC_VIDEO_BRIGHTNESS2    , "GET_TCC_VIDEO_BRIGHTNESS2"},
    {GET_TCC_VIDEO_CONTRAST2    , "GET_TCC_VIDEO_CONTRAST2"},
    {GET_TCC_VIDEO_GAMMA2        , "GET_TCC_VIDEO_GAMMA2"},
    {GET_TCC_VIDEO_SATURATION2    , "GET_TCC_VIDEO_SATURATION2"},

    {GET_TCC_DMB_BRIGHTNESS     , "GET_TCC_DMB_BRIGHTNESS"},
    {GET_TCC_DMB_CONTRAST        , "GET_TCC_DMB_CONTRAST"},
    {GET_TCC_DMB_GAMMA            , "GET_TCC_DMB_GAMMA"},
    {GET_TCC_DMB_SATURATION     , "GET_TCC_DMB_SATURATION"},
    
    {GET_TCC_CMMB_BRIGHTNESS     , "GET_TCC_CMMB_BRIGHTNESS"},
    {GET_TCC_CMMB_CONTRAST        , "GET_TCC_CMMB_CONTRAST"},
    {GET_TCC_CMMB_GAMMA            , "GET_TCC_CMMB_GAMMA"},
    {GET_TCC_CMMB_SATURATION     , "GET_TCC_CMMB_SATURATION"},
    
    {GET_TCC_USB_BRIGHTNESS     , "GET_TCC_USB_BRIGHTNESS"},
    {GET_TCC_USB_CONTRAST        , "GET_TCC_USB_CONTRAST"},
    {GET_TCC_USB_GAMMA            , "GET_TCC_USB_GAMMA"},
    {GET_TCC_USB_SATURATION     , "GET_TCC_USB_SATURATION"},    

    {GET_TCC_ISDB_BRIGHTNESS     , "GET_TCC_ISDB_BRIGHTNESS"},
    {GET_TCC_ISDB_CONTRAST        , "GET_TCC_ISDB_CONTRAST"},
    {GET_TCC_ISDB_GAMMA            , "GET_TCC_ISDB_GAMMA"},
    {GET_TCC_ISDB_SATURATION     , "GET_TCC_ISDB_SATURATION"},
};

static ie_setting_info ie_info;
static struct class *daudio_ie_class;
static struct device *pdev;
static struct mutex ie_lock;

struct early_suspend daudio_ie_early_suspend;

static int init_ie_settings(void);
static int daudio_set_settings(int mode, unsigned char level);
static int daudio_set_ie_level(int mode, unsigned char level);
static int daudio_get_ie_level(int mode, unsigned char *level);
static void print_setting_value(void);

static DEFINE_MUTEX(daudio_tw9990_setting_lock);
/**
  *   @author legolamp@cleinsoft.com
  *   @date 2014/11/13
  *   return ie_debug_levlel to daudio_ie_tcc, daudio_ie_twxxxx
**/

int get_ie_debug_level(void)
{
    return sCLogLevel;
}

static int atoi(const char *name)
{
    int val = 0;
    for (;; name++)
    {
        switch (*name)
        {
            case '0'...'9':
                val = 10 * val + (*name - '0');
                break;
            default:
                return val;
        }
    }
    return val;
}

static int get_mode_name(int mode, char *name)
{
    int i = 0;
    int size = sizeof(mode_name_list) / sizeof(IE_mode_name);
    int count = 0;

    if (name == NULL)
    {
        CLOG(CLL_ERROR, "%s failed (arg is NULL)\n", __func__);
        return -1;
    }

    for (i = 0; i < size; i++)
    {
        if (mode_name_list[i].mode == mode)
        {
            while (count < MAX_MODE_NAME_SIZE)
            {
                if (mode_name_list[i].descrption[count] == '\0')
                    break;
                else
                    count++;
            }

            if (count > 0)
                memcpy(name, mode_name_list[i].descrption, count);
            else
                count = 0;

            break;
        }

        count = 0;
    }

    return count;
}

static bool isOpenRun = true;
static bool initDefautSet = true;

int init_default_ie_info(void)
{

    if (initDefautSet)
    {
		int loop;
		int total_cnt = sizeof(ie_setting_info)/sizeof(int);
		int *tmp = (int*)&ie_info;
		
        CLOG(CLL_INFO, "[GT]%s \n", __func__);
        initDefautSet= false;
        for(loop =2; loop < total_cnt ; loop++)
        {
            *tmp = 127;
            tmp++;
        }
		ie_info.id      = DAUDIO_IE_SETTINGS_ID;
		ie_info.version = DAUDIO_IE_SETTINGS_VERSION;
    }
	return 0;
}

int init_video_ie_check(void)
{
	init_default_ie_info();

    if (isOpenRun)
    {
        CLOG(CLL_INFO, "[GT]%s \n", __func__);
        isOpenRun = false;
        init_tcc_video_ie(ie_info.tcc_usb_brightness,
                       ie_info.tcc_usb_contrast,
                       ie_info.tcc_usb_gamma,
                       ie_info.tcc_usb_saturation);
    }
	return 0;
}


static int init_ie_settings(void)
{
	init_video_ie_check();

    if (ie_info.id == (unsigned int)DAUDIO_IE_SETTINGS_ID)
    {
        return SUCCESS;
    }
    print_ie_info(&ie_info);

    return SUCCESS;
}

static int daudio_set_settings(int mode, unsigned char level)
{
    int ret = init_ie_settings();
    if (ret == SUCCESS)
    {
        int count = mode - SET_BRIGHTNESS;
        int size_id = sizeof(ie_info.id) / 4;
        int size_version = sizeof(ie_info.version) / 4;
        int *plevel = (int *)(&ie_info);
        plevel[size_id + size_version + count] = level;

    }

    return ret;
}

static int daudio_set_ie_level(int mode, unsigned char level)
{
    int ret = FAIL;

    //if (mode >= SET_BRIGHTNESS && mode <= SET_CAM_SATURATION) // GT system
    if (mode >= SET_BRIGHTNESS && mode <= SET_AUX_SATURATION) 
    {
        if (is_twxxxx_ie_invalid_arg(mode, level))
            return FAIL_OVERFLOW_LIMIT_ARG;

        mutex_lock(&ie_lock);
        CLOG(CLL_INFO, "%s, twxxxx_set_ie mode =%d , level = %d \n ", __func__,mode,level );
        ret = twxxxx_set_ie(mode, level);
        mutex_unlock(&ie_lock);
    }
    //else if (mode >= SET_TCC_BRIGHTNESS && mode <= SET_TCC_VIDEO_SATURATION2) // GT system
    else if (mode >= SET_TCC_BRIGHTNESS && mode <= SET_TCC_USB_SATURATION)
    {
        CLOG(CLL_INFO, "%s, TCC  mod =%d , level =%d   \n ", __func__ ,mode,level);
        if (is_tcc_ie_invalid_arg(mode, level))
            return FAIL_OVERFLOW_LIMIT_ARG;

        mutex_lock(&ie_lock);
        ret = set_tcc_ie(mode, level);
        mutex_unlock(&ie_lock);
    }
    else
    {    
        CLOG(CLL_ERROR, "%s, fail mode = %d , level = %d--------------------->> input error  \n", __func__, mode , level );
    }
    return ret;
}

static int daudio_get_ie_level(int mode, unsigned char *level)
{
    int ret = FAIL;
    ie_setting_info t_ie_info;

    mutex_lock(&ie_lock);

    ret = init_ie_settings();
    int *plevel = (int *)(&ie_info.tw8836_brightness + (mode - GET_BRIGHTNESS));

    CLOG(CLL_INFO, "%s, mode = %d \n ", __func__,mode );
    if (ret == SUCCESS)
    {
        //if (mode >= GET_BRIGHTNESS && mode <= GET_CAM_SATURATION)
        if (mode >= GET_BRIGHTNESS && mode <= GET_AUX_SATURATION ) //   GT system
            ret = twxxxx_get_ie(mode, level);

#if defined(CONFIG_TW8836) 
        if (mode == GET_HUE && ret >= 0)
        {
            char temp = *level;
            *level = parse_tw8836_hue_to_value(*level);
            CLOG(CLL_INFO, "%s get hue tw8836: %d, setting.hue: %d, parse: %d\n", __func__,
                    temp, ie_info.tw8836_hue, *level);
        }
#endif        
        CLOG(CLL_INFO, "%s, plevel =%d--------------------->>  \n", __func__, *plevel );

        /*if( mode >= GET_TCC_VIDEO_BRIGHTNESS && mode <= GET_TCC_VIDEO_SATURATION )
            {
                init_tcc_video_ie(ie_info.tcc_video_brightness,
                    ie_info.tcc_video_contrast,
                    ie_info.tcc_video_gamma,
                    ie_info.tcc_video_saturation);
            }
            else if ( mode >= GET_TCC_DMB_BRIGHTNESS && mode <= GET_TCC_DMB_SATURATION )
            {
                init_tcc_video_ie(ie_info.tcc_dmb_brightness,
                    ie_info.tcc_dmb_contrast,
                    ie_info.tcc_dmb_gamma,
                    ie_info.tcc_dmb_saturation);
            }
            else if ( mode >= GET_TCC_CMMB_BRIGHTNESS && mode <= GET_TCC_CMMB_SATURATION )
            {
                init_tcc_video_ie(ie_info.tcc_cmmb_brightness,
                    ie_info.tcc_cmmb_contrast,
                    ie_info.tcc_cmmb_gamma,
                    ie_info.tcc_cmmb_saturation);
            }
            else if ( mode >= GET_TCC_USB_BRIGHTNESS && mode <= GET_TCC_USB_SATURATION )
            {
                init_tcc_video_ie(ie_info.tcc_usb_brightness,
                    ie_info.tcc_usb_contrast,
                    ie_info.tcc_usb_gamma,
                    ie_info.tcc_usb_saturation);
            }
            else if(  mode >= GET_TCC_BRIGHTNESS  && mode <= GET_TCC_SATURATION )
            {
                init_tcc_ie(ie_info.tcc_brightness,
                    ie_info.tcc_contrast,
                    ie_info.tcc_hue,
                    ie_info.tcc_saturation);
            }*/

        //TCC Image Enhancement
        //if (mode >= GET_TCC_BRIGHTNESS && mode <= GET_TCC_VIDEO_SATURATION2)
        if (mode >= GET_TCC_BRIGHTNESS && mode <= GET_TCC_USB_SATURATION) // GT system
        {
            if ( *plevel > 0 && *plevel < 256 )
                *level = *plevel;
            else
            {
                //not init
                if(get_tcc_ie(mode, level) == FAIL)
                {
                    *level = 127;    //default
                    *plevel = 127;
                }
                else
                {
                    if ( *plevel <= 0 && *plevel >= 256 )
                    {    
                        *level = 127;
                    }

                    *plevel = *level;
                }
            }

            if (ret < FAIL)
            {
                #if 0//defined(CHECK_SETTING_VALUE)
                //compare setting data.
                if (*level > 0 && *level < 256)
                {
                    if (*level == *plevel)
                    {
                        //success
                        ret = SUCCESS;
                    }
                    else if (*plevel == 0)
                    {
                        //no setting
                        ret = FAIL_NO_SETTING;
                    }
                    else
                    {
                        //unmatch setting
                        if (mode != GET_HUE)
                        {
                            if (*plevel > 0 && *plevel < 256)
                                *level = *plevel;

                            ret = FAIL_UNMATCH_SETTING;
                        }
                        else    //case TW8836 HUE
                        {
                            ret = SUCCESS;
                        }    
                    }
                }

                #endif
            }
        }// if (mode >= ~
        CLOG(CLL_INFO, "%s level: %d, ret: %d\n", __func__, (int)(*level), ret);
    }

    mutex_unlock(&ie_lock);

    return ret;
}

int daudio_set_ie(int mode, unsigned char level)
{
    int ret = 0;
    unsigned char hlevel = level;
    
#if defined(CONFIG_TW8836) 

    if (mode == SET_HUE && level != 0)
    {
        hlevel = parse_value_to_tw8836_hue(level);
    }
#endif

    ret = daudio_set_ie_level(mode, hlevel);
    if (ret == SUCCESS_I2C || ret == SUCCESS)
    {
        ret = daudio_set_settings(mode, level);
    }
    return ret;
}

int daudio_get_ie(int mode, unsigned char *level)
{
    int ret = FAIL;
    ret = daudio_get_ie_level(mode, level);
    return ret;
}

static long daudio_ie_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    unsigned char level = 0;
    unsigned char level1 = 0;
    int ret = FAIL;
    ie_setting_info t_ie_info;

    CLOG(CLL_INFO, "%s cmd: %d arg: 0x%lx\n", __func__, cmd, arg);

    copy_from_user(&t_ie_info, (ie_setting_info*)arg, sizeof(ie_setting_info));

    if (cmd < SET_BRIGHTNESS || cmd > GET_TW9990_STRAGE)
    {
        return FAIL_NO_COMMNAD;
    }

    //set
    if (cmd < GET_BRIGHTNESS)
    {
        //set level
        int count = cmd - SET_BRIGHTNESS;
        int size_id = sizeof(t_ie_info.id) / sizeof(int);
        int size_version = sizeof(t_ie_info.version) /sizeof(int);
        int *plevel = (int *)&t_ie_info;
        level = (unsigned char)(plevel[size_id + size_version + count]);

        ret = daudio_set_ie(cmd, level);
    }
    //get
    else if(cmd >= GET_BRIGHTNESS && cmd < SET_TW9990_STRAGE )
    {
        int count = cmd - GET_BRIGHTNESS;
        int size_id =  sizeof(t_ie_info.id) / sizeof(int) ;
        int size_version = sizeof(t_ie_info.version) /sizeof(int) ;
        int *plevel = (int *)&t_ie_info;
        /*    
        if(cmd == GET_CAM_SATURATION)
        {
            ret = daudio_get_ie(GET_CAM_U_GAIN, &level);
            ret = daudio_get_ie(GET_CAM_V_GAIN, &level1);            
            CLOG(CLL_CRITICAL,"%s cmd: %d ret: %d V_GAIN level:%d\n", __func__, cmd, ret, level1);
        }
        else*/
	{
            ret = daudio_get_ie(cmd, &level);
	}

        if (ret < FAIL || ret == FAIL_NO_SETTING || ret == FAIL_UNMATCH_SETTING)
        {
            plevel[size_id + size_version + count] = (int)level;
        }
    }
    else if(cmd == SET_TW9990_STRAGE)
    {
	CLOG(CLL_ERROR,"TW9990_IOC_STORAGE SETTING_SET - empty\n");

    }
    else if(cmd == GET_TW9990_STRAGE)
    {
	CLOG(CLL_ERROR,"TW9990_IOC_STORAGE  SETTING_GET\n");

	mutex_lock(&daudio_tw9990_setting_lock);
	ret = read_ie_setting(&t_ie_info);

	 if (!ret) {
		ret = -EINVAL;
		printk(KERN_ERR "[GT system] %s , read_ie_setting FAIL~~!!\n", __func__ );
	}
	copy_to_user((ie_setting_info*)arg, &t_ie_info, sizeof(ie_setting_info));

	mutex_unlock(&daudio_tw9990_setting_lock);
    }
    else
    {
	CLOG(CLL_ERROR,"daudio_ie_ioctl------------ERROR\n");
    }
    
    if (ret == 0)
        ret = 1;

    copy_to_user((ie_setting_info*)arg, &t_ie_info, sizeof(ie_setting_info));

    CLOG(CLL_CRITICAL, "[GT]%s cmd: %d ret: %d level:%d\n", __func__, cmd, ret, level);
    return ret;
}

static int daudio_ie_open(struct inode *inode, struct file *file)
{
    static bool isrun = false;
    CLOG(CLL_INFO, "[GT]%s  start ========>>>\n", __func__);
    if(isrun)
        return 0;
    isrun = true;
    int ret = init_ie_settings();
    CLOG(CLL_INFO, "[GT]%s , ret = %d\n", __func__, ret);
    if (ret == SUCCESS)
    {
    init_tcc_video_ie(ie_info.tcc_usb_brightness,
                       ie_info.tcc_usb_contrast,
                       ie_info.tcc_usb_gamma,
                       ie_info.tcc_usb_saturation);
    }    
    return 0;
}

static int daudio_ie_release(struct inode *inode, struct file *file)
{
    return 0;
}

static struct file_operations daudio_ie_fops =
{
    .owner          = THIS_MODULE,
    .unlocked_ioctl = daudio_ie_ioctl,
    .open           = daudio_ie_open,
    .release        = daudio_ie_release,
};

static void ie_early_suspend(struct early_suspend *h)
{
    //CLOG(CLL_TRACE, "%s\n", __func__);
}

static void ie_late_resume(struct early_suspend *h)
{
    char level;

    //CLOG(CLL_TRACE, "%s\n", __func__);
    // Restore ie_setting_info.
    daudio_get_ie(GET_BRIGHTNESS, &level);
}

static void print_setting_value(void)
{
    int i = 0, max = 0;
    unsigned char level = 0, *mode_name = 0;

    CLOG(CLL_INFO, "%s\n", __func__);

    i = GET_BRIGHTNESS;
    max = GET_TCC_USB_SATURATION+1 ; //GET_TCC_VIDEO_SATURATION2 + 1; // GT system

    CLOG(CLL_CRITICAL,"======== D-Audio Image Enhancement =======\n");
    for (; i < max; i++)
    {
        daudio_get_ie(i, &level);
        mode_name = kmalloc(MAX_MODE_NAME_SIZE, GFP_KERNEL);
        memset(mode_name, 0, MAX_MODE_NAME_SIZE);
        
        /*if(i == GET_CAM_SATURATION)
        {
            get_mode_name(GET_CAM_U_GAIN, mode_name);
            CLOG(CLL_CRITICAL,"MODE: %s(%d) level: %d\n", mode_name, i, level);
            get_mode_name(GET_CAM_V_GAIN, mode_name);
        }
        else*/
        get_mode_name(i, mode_name);
        CLOG(CLL_CRITICAL,"MODE: %s(%d) level: %d\n", mode_name, i, level);
        kfree(mode_name);
    }
    CLOG(CLL_CRITICAL, "==========================================\n");
}

static ssize_t daudio_store_settings(const char *buf, size_t count)
{
    unsigned char *bmode;
    unsigned char *bvalue;
    char *mode_name = 0;
    int mode_name_count = 0, i = 0;
    int mode = -1, value = -1, seperator = -1, ret = -1;

    while (1)
    {
        if (buf[i] == ' ' || buf[i] == '\0')
        {
            seperator = i;
            break;
        }
        else
            i++;
    }

    if (seperator >= count || seperator < 0)
    {
        mutex_unlock(&ie_lock);
        return count;
    }

    bmode = kmalloc(seperator, GFP_KERNEL);
    bvalue = kmalloc(count - (seperator + 1) - 1, GFP_KERNEL);
    mode_name = (char *)kmalloc(MAX_MODE_NAME_SIZE, GFP_KERNEL);
    memset(mode_name, 0, MAX_MODE_NAME_SIZE);

    memcpy(bmode, buf, seperator);
    memcpy(bvalue, buf + seperator + 1, count - (seperator + 1) - 1);

    mode = atoi(bmode);
    value = atoi(bvalue);

    mode_name_count = get_mode_name(mode, mode_name);

    kfree(bmode);
    kfree(bvalue);

#if defined(CONFIG_DAUDIO_IE) //20160725 mhjung sysfs ie test 
    if(value > 254)
	    value = 254;
#endif
    ret = daudio_set_ie(mode, value);

    kfree(mode_name);

    print_ie_info(&ie_info);

    return count;
}

static ssize_t show_settings(struct device *dev, struct device_attribute *attr, char *buf)
{
#ifdef REAR_CAM_SETTIGS_EMMC_TEST
	ie_setting_info setting_info;

	read_ie_setting(&setting_info);

	printk(KERN_ERR "1@#---- %s() - twxxxx_cam_brightness=%d,twxxxx_cam_contrast=%d,twxxxx_cam_saturationu=%d\n", __func__,setting_info.twxxxx_cam_brightness ,setting_info.twxxxx_cam_contrast ,setting_info.twxxxx_cam_saturation );
#endif
	print_setting_value();

    return 0;
}

static ssize_t store_settings(struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{

#ifdef REAR_CAM_SETTIGS_EMMC_TEST
	ie_setting_info setting_info;

	setting_info.twxxxx_cam_brightness = 99;
	setting_info.twxxxx_cam_contrast= 99;
	setting_info.twxxxx_cam_saturation= 99;

	write_ie_setting(&setting_info);
#endif

	return daudio_store_settings(buf, count);
}

static DEVICE_ATTR(settings, 0664, show_settings, store_settings);


static __init int daudio_ie_init(void)
{
    int ret = register_chrdev(DAUDIO_IE_MAJOR, SYS_DRIVER_NAME, &daudio_ie_fops);

    CLOG(CLL_INFO ,"%s %d\n", __func__, ret);

    if (ret < 0)
    {
        CLOG(CLL_ERROR, "%s failed to register_chrdev\n", __func__);
        ret = -EIO;
        goto out;
    }

    daudio_ie_class = class_create(THIS_MODULE, SYS_DRIVER_NAME);
    if (IS_ERR(daudio_ie_class))
    {
        ret = PTR_ERR(daudio_ie_class);
        CLOG(CLL_ERROR, "failed to create class: %d\n", ret);
        goto fail_class_create;
    }

    pdev = device_create(daudio_ie_class, NULL, MKDEV(DAUDIO_IE_MAJOR, 0), NULL, DRIVER_NAME);
    if (IS_ERR(pdev))
    {
        ret = PTR_ERR(pdev);
        CLOG(CLL_ERROR, "failed to create pdev : %d\n", ret);
        goto fail_device_create_file;
    }

    ret = device_create_file(pdev, &dev_attr_settings);
    if (ret)
    {
        CLOG(CLL_ERROR, "failed to create daudio image enhancement settings: %d\n", ret);
        goto fail_device_create_setting_file;
    }

    ret = device_create_file_clog(pdev);
    if (ret)
    {
        CLOG(CLL_ERROR, "failed to create daudio image enhancement clog: %d\n", ret);
        goto fail_device_create_clog_file;
    }

#ifdef CONFIG_HAS_EARLYSUSPEND
    daudio_ie_early_suspend.suspend = ie_early_suspend;
    daudio_ie_early_suspend.resume = ie_late_resume;
    daudio_ie_early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB;
    register_early_suspend(&daudio_ie_early_suspend);
#endif

    mutex_init(&ie_lock);
    goto out;

fail_device_create_clog_file:
    device_remove_file_clog(pdev);
fail_device_create_setting_file:
    device_remove_file(pdev, &dev_attr_settings);
fail_device_create_file:
    device_destroy(daudio_ie_class, MKDEV(DAUDIO_IE_MAJOR, 0));
fail_class_create:
    unregister_chrdev(DAUDIO_IE_MAJOR, DRIVER_NAME);
out:
    return ret;
}

static __exit void daudio_ie_exit(void)
{
    device_remove_file_clog(pdev);
    device_remove_file(pdev, &dev_attr_settings);
    device_destroy(daudio_ie_class, MKDEV(DAUDIO_IE_MAJOR, 0));
    class_destroy(daudio_ie_class);
    unregister_chrdev(DAUDIO_IE_MAJOR, DRIVER_NAME);
}

subsys_initcall(daudio_ie_init);
module_exit(daudio_ie_exit);

MODULE_AUTHOR("Cleinsoft");
MODULE_DESCRIPTION("D-Audio Image Enhancement Driver");
MODULE_LICENSE("GPL");
