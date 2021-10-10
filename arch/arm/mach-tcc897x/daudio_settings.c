#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/syscalls.h>

#include <asm/uaccess.h>
#include <mach/daudio_settings.h>
//#include <mach/daudio_eng.h> 2016.0623 mhjung delete
#include <mach/daudio.h>

static int debug_settings_info = 0;
static int debug_settings_detail =0 ;

#define VPRINTK(fmt, args...) if (debug_settings_info) printk(KERN_CRIT "[daudio settings] " fmt, ##args)
#define DPRINTK(fmt, args...) if (debug_settings_detail) printk(KERN_CRIT "[daudio settings] " fmt, ##args)

const char *ie_setting_field_name[] = {
	"id",
	"version",
	"tw8836_brightness",
	"tw8836_contrast",
	"w8836_hue",
	"tw8836_saturation",
	"tw8836_cam_brightness",
	"tw8836_cam_contrast",
	"tw8836_cam_hue",
	/*"tw8836_cam_sharpness", //GT start
	"tw8836_cam_u_gain",
	"tw8836_cam_v_gain",   //GT end*/
	"tw8836_cam_saturation",

	"tcc_aux_brightness",
	"tcc_aux_contrast",
	"tcc_aux_hue",
	"tcc_aux_saturation",

	"tcc_brightness",
	"tcc_contrast",
	"tcc_hue",
	"tcc_saturation",
	"tcc_video_brightness",
	"tcc_video_contrast",
	"tcc_video_gamma",
	"tcc_video_saturation",
	"tcc_video_brightness2",
	"tcc_video_contrast2",
	"tcc_video_gamma2",
	"tcc_video_saturation2",
	/*GT system*/
	"tcc_dmb_brightness",
	"tcc_dmb_contrast",
	"tcc_dmb_gamma",
	"tcc_dmb_saturation",

	"tcc_cmmb_brightness",
	"tcc_cmmb_contrast",
	"tcc_cmmb_gamma",
	"tcc_cmmb_saturation",

	"tcc_usb_brightness",
	"tcc_usb_contrast",
	"tcc_usb_gamma",
	"tcc_usb_saturation",

	"temp_1",
	"temp_2",
	"temp_3",
	"temp_4",
};

static int read_data(char *filename, char *data, int read_len, unsigned int offset)
{
	int ret = FAIL_READ_SETTING, fd = 0;

	mm_segment_t old_fs = get_fs();
	set_fs(KERNEL_DS);

	fd = sys_open(filename, O_RDONLY, 0);
	if (fd >= 0)
	{
		sys_lseek(fd, offset, SEEK_SET);
		ret = sys_read(fd, data, read_len);
		sys_close(fd);
		if (ret >= 0)
			ret = SUCCESS;
	}else
	{
		VPRINTK("%s read_data fd : %d\n", __func__, fd );
	}

	set_fs(old_fs);

	return ret;
}

static int write_data(char *filename, char *data, int write_len, unsigned int offset)
{
	int ret = FAIL_WRITE_SETTING, fd = 0;

	mm_segment_t old_fs = get_fs();
	set_fs(KERNEL_DS);

	fd = sys_open(filename, O_RDWR, 0);
	if (fd >= 0)
	{
		sys_lseek(fd, offset, SEEK_SET);
		ret = sys_write(fd, data, write_len);
		VPRINTK("%s sys_write ret: %d\n", __func__, ret);
		sys_close(fd);
		if (ret >= 0)
			ret = SUCCESS;
	}else
	{
		VPRINTK("%s sys_write_error fd : %d\n", __func__, fd );
	}
	set_fs(old_fs);

	return ret;
}

/**
 * Print Image Enhancement settings.
 * @param	info Pointer to print the result.
 */
void print_ie_info(ie_setting_info *info)
{
	int i;
	unsigned int temp = 0;
	const int size = sizeof(ie_setting_info) / sizeof(int);

	if (info == NULL)
	{
		VPRINTK("%s IE info is NULL\n", __func__);
		return;
	}

	for (i = 0; i < size; i++)
	{
		temp = ((unsigned int *)info)[i];
		if (temp)
			VPRINTK("%s %s: %d\n", __func__, ie_setting_field_name[i], temp);
	}
}

/**
 * Read Engineer mode settings from settings partition.
 * @param	info Pointer to receive the result.
 * @return	1 - success, 0 - read fail.
 */
int read_em_setting(em_setting_info *info)
{
	int ret = FAIL_READ_SETTING;
	const unsigned int size = sizeof(em_setting_info);
	char *buf;

	if (info == NULL)
	{
		VPRINTK("%s info is NULL!\n", __func__);
		return ret;
	}
	DPRINTK("%s\n", __func__);

	buf = (char *)kmalloc(size, GFP_KERNEL);
	if (buf == NULL)
	{
		VPRINTK("%s failed to kmalloc\n", __func__);
		return ret;
	}

#if defined(CONFIG_HIBERNATION) && defined(CONFIG_SNAPSHOT_BOOT)	//if quickboot mode
	ret = read_data(SETTINGS_BLOCK, buf, size, OFFSET_EM_SETTINGS);
#endif
	if (ret == SUCCESS)
	{
		memcpy((char *)info, buf, size);
		if (info->id != EM_SETTING_ID)
			info->id = EM_SETTING_ID;
	}

	DPRINTK("%s id: 0x%llx mode: %d size: %d\n", __func__, info->id, info->mode, size);

	kfree(buf);

	return ret;
}

/**
 * Write Engineer mode settings to settings partition.
 * @param	info Pointer to save the mode.
 * @return	1 - success, 0 - read fail.
 */
int write_em_setting(em_setting_info *info)
{
	int ret = FAIL_READ_SETTING;
	const int size = sizeof(em_setting_info);

	if (info == NULL)
	{
		VPRINTK("%s info is NULL!\n", __func__);
		return ret;
	}

	if (info->id != EM_SETTING_ID)
		info->id = EM_SETTING_ID;

	DPRINTK("%s id: 0x%llx mode: %d size: %d\n", __func__, info->id, info->mode, size);

#if defined(CONFIG_HIBERNATION) && defined(CONFIG_SNAPSHOT_BOOT)	//if quickboot mode
	ret = write_data(SETTINGS_BLOCK, (char *)info, size, OFFSET_EM_SETTINGS);
#endif

	return ret;
}

/**
 * Read Engineer mode settings from settings partition.
 * @param	info Pointer to receive the result.
 * @return	1 - success, 0 - read fail.
 */
int read_ie_setting(ie_setting_info *info)
{
	int ret = FAIL_READ_SETTING;
	const unsigned int size = sizeof(ie_setting_info);
	char *buf;

	if (info == NULL)
	{
		VPRINTK("%s info is NULL!\n", __func__);
		return ret;
	}
	DPRINTK("%s\n", __func__);

	buf = (char *)kmalloc(size, GFP_KERNEL);
	if (buf == NULL)
	{
		VPRINTK("%s failed to kmalloc\n", __func__);
		return ret;
	}

#if defined(CONFIG_HIBERNATION) && defined(CONFIG_SNAPSHOT_BOOT)	//if quickboot mode
	ret = read_data(SETTINGS_NAME, buf, size, OFFSET_IE_SETTINGS);
#endif

	if (ret == SUCCESS)
	{
		memcpy((char *)info, buf, size);
	}
	
	if(ret == 0)
	{
		VPRINTK("%s IE setting read fail !!\n", __func__);
	}
		
	kfree(buf);

	return ret;
}

/**
 * Write Engineer mode settings to settings partition.
 * @param	info Pointer to save the mode.
 * @return	1 - success, 0 - read fail.
 */

 int write_ie_setting(ie_setting_info *info)
{
	int ret = FAIL_WRITE_SETTING;
	const int size = sizeof(ie_setting_info);

	if (info == NULL)
	{
		VPRINTK("%s info is NULL!\n", __func__);
		return ret;
	}

#if defined(CONFIG_HIBERNATION) && defined(CONFIG_SNAPSHOT_BOOT)	//if quickboot mode
	ret = write_data(SETTINGS_NAME, (char *)info, size, OFFSET_IE_SETTINGS);
#endif

	if(ret == 0)
	{
		VPRINTK("%s IE setting write fail !!\n", __func__);
	}

	return ret;
}

/**
 * Print ENGINEER MODE settings.
 * @param	info Pointer to print the result.
 */
void print_em_settings(em_setting_info *info)
{
	if (info == NULL)
	{
		VPRINTK("%s EM Settings is NULL\n", __func__);
		return;
	}

	VPRINTK("%s id: 0x%llx\n", __func__, info->id);
	VPRINTK("%s mode: %d\n", __func__, info->mode);
}

/**
 * Sync cache and eMMC data.
 */
void update_settings(void)
{
	sys_sync();
}

