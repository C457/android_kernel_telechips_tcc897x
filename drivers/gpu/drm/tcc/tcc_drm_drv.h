/* tcc_drm_drv.h
 *
 * Copyright (c) 2016 Telechips Inc.
 *
 * Author:  Telechips Inc.
 * Created: Jan 07, 2016
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 *---------------------------------------------------------------------------
 * Copyright (c) 2011 Samsung Electronics Co., Ltd.
 * Authors:
 *	Inki Dae <inki.dae@samsung.com>
 *	Joonyoung Shim <jy0922.shim@samsung.com>
 *	Seung-Woo Kim <sw0312.kim@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef _TCC_DRM_DRV_H_
#define _TCC_DRM_DRV_H_

#include <linux/module.h>

#define MAX_CRTC	3
#define MAX_PLANE	4
#define MAX_FB_BUFFER	3
#define DEFAULT_ZPOS	0

#define _wait_for(COND, MS) ({ \
	unsigned long timeout__ = jiffies + msecs_to_jiffies(MS);	\
	int ret__ = 0;							\
	while (!(COND)) {						\
		if (time_after(jiffies, timeout__)) {			\
			ret__ = -ETIMEDOUT;				\
			break;						\
		}							\
	}								\
	ret__;								\
})

#define wait_for(COND, MS) _wait_for(COND, MS)

struct drm_device;
struct tcc_drm_overlay;
struct drm_connector;

/* This enumerates device type. */
enum tcc_drm_device_type {
	TCC_DEVICE_TYPE_NONE,
	TCC_DEVICE_TYPE_CRTC,
	TCC_DEVICE_TYPE_CONNECTOR,
};

/* this enumerates display type. */
enum tcc_drm_output_type {
	TCC_DISPLAY_TYPE_NONE,
	/* RGB or CPU Interface. */
	TCC_DISPLAY_TYPE_LCD,
	/* HDMI Interface. */
	TCC_DISPLAY_TYPE_HDMI,
	/* Virtual Display Interface. */
	TCC_DISPLAY_TYPE_VIDI,
};

/*
 * Telechips drm common overlay structure.
 *
 * @fb_x: offset x on a framebuffer to be displayed.
 *	- the unit is screen coordinates.
 * @fb_y: offset y on a framebuffer to be displayed.
 *	- the unit is screen coordinates.
 * @fb_width: width of a framebuffer.
 * @fb_height: height of a framebuffer.
 * @src_width: width of a partial image to be displayed from framebuffer.
 * @src_height: height of a partial image to be displayed from framebuffer.
 * @crtc_x: offset x on hardware screen.
 * @crtc_y: offset y on hardware screen.
 * @crtc_width: window width to be displayed (hardware screen).
 * @crtc_height: window height to be displayed (hardware screen).
 * @mode_width: width of screen mode.
 * @mode_height: height of screen mode.
 * @refresh: refresh rate.
 * @scan_flag: interlace or progressive way.
 *	(it could be DRM_MODE_FLAG_*)
 * @bpp: pixel size.(in bit)
 * @pixel_format: fourcc pixel format of this overlay
 * @dma_addr: array of bus(accessed by dma) address to the memory region
 *	      allocated for a overlay.
 * @zpos: order of overlay layer(z position).
 * @default_win: a window to be enabled.
 * @color_key: color key on or off.
 * @index_color: if using color key feature then this value would be used
 *			as index color.
 * @local_path: in case of lcd type, local path mode on or off.
 * @transparency: transparency on or off.
 * @activated: activated or not.
 *
 * this structure is common to tcc SoC and its contents would be copied
 * to hardware specific overlay info.
 */
struct tcc_drm_overlay {
	unsigned int fb_x;
	unsigned int fb_y;
	unsigned int fb_width;
	unsigned int fb_height;
	unsigned int src_width;
	unsigned int src_height;
	unsigned int crtc_x;
	unsigned int crtc_y;
	unsigned int crtc_width;
	unsigned int crtc_height;
	unsigned int mode_width;
	unsigned int mode_height;
	unsigned int refresh;
	unsigned int scan_flag;
	unsigned int bpp;
	unsigned int pitch;
	uint32_t pixel_format;
	dma_addr_t dma_addr[MAX_FB_BUFFER];
	int zpos;

	bool default_win;
	bool color_key;
	unsigned int index_color;
	bool local_path;
	bool transparency;
	bool activated;
};

/*
 * Telechips DRM Display Structure.
 *	- this structure is common to analog tv, digital tv and lcd panel.
 *
 * @remove: cleans up the display for removal
 * @mode_fixup: fix mode data comparing to hw specific display mode.
 * @mode_set: convert drm_display_mode to hw specific display mode and
 *	      would be called by encoder->mode_set().
 * @check_mode: check if mode is valid or not.
 * @dpms: display device on or off.
 * @commit: apply changes to hw
 */
struct tcc_drm_display;
struct tcc_drm_display_ops {
	int (*create_connector)(struct tcc_drm_display *display,
				struct drm_encoder *encoder);
	void (*remove)(struct tcc_drm_display *display);
	void (*mode_fixup)(struct tcc_drm_display *display,
				struct drm_connector *connector,
				const struct drm_display_mode *mode,
				struct drm_display_mode *adjusted_mode);
	void (*mode_set)(struct tcc_drm_display *display,
				struct drm_display_mode *mode);
	int (*check_mode)(struct tcc_drm_display *display,
				struct drm_display_mode *mode);
	void (*dpms)(struct tcc_drm_display *display, int mode);
	void (*commit)(struct tcc_drm_display *display);
};

/*
 * Telechips drm display structure, maps 1:1 with an encoder/connector
 *
 * @list: the list entry for this manager
 * @type: one of TCC_DISPLAY_TYPE_LCD and HDMI.
 * @encoder: encoder object this display maps to
 * @connector: connector object this display maps to
 * @ops: pointer to callbacks for tcc drm specific functionality
 * @ctx: A pointer to the display's implementation specific context
 */
struct tcc_drm_display {
	struct list_head list;
	enum tcc_drm_output_type type;
	struct drm_encoder *encoder;
	struct drm_connector *connector;
	struct tcc_drm_display_ops *ops;
	void *ctx;
};

/*
 * Telechips drm manager ops
 *
 * @dpms: control device power.
 * @mode_fixup: fix mode data before applying it
 * @mode_set: set the given mode to the manager
 * @commit: set current hw specific display mode to hw.
 * @enable_vblank: specific driver callback for enabling vblank interrupt.
 * @disable_vblank: specific driver callback for disabling vblank interrupt.
 * @wait_for_vblank: wait for vblank interrupt to make sure that
 *	hardware overlay is updated.
 * @win_mode_set: copy drm overlay info to hw specific overlay info.
 * @win_commit: apply hardware specific overlay data to registers.
 * @win_enable: enable hardware specific overlay.
 * @win_disable: disable hardware specific overlay.
 * @te_handler: trigger to transfer video image at the tearing effect
 *	synchronization signal if there is a page flip request.
 */
struct tcc_drm_manager;
struct tcc_drm_manager_ops {
	void (*dpms)(struct tcc_drm_manager *mgr, int mode);
	bool (*mode_fixup)(struct tcc_drm_manager *mgr,
				const struct drm_display_mode *mode,
				struct drm_display_mode *adjusted_mode);
	void (*mode_set)(struct tcc_drm_manager *mgr,
				const struct drm_display_mode *mode);
	void (*commit)(struct tcc_drm_manager *mgr);
	int (*enable_vblank)(struct tcc_drm_manager *mgr);
	void (*disable_vblank)(struct tcc_drm_manager *mgr);
	void (*wait_for_vblank)(struct tcc_drm_manager *mgr);
	void (*win_mode_set)(struct tcc_drm_manager *mgr,
				struct tcc_drm_overlay *overlay);
	void (*win_commit)(struct tcc_drm_manager *mgr, int zpos);
	void (*win_enable)(struct tcc_drm_manager *mgr, int zpos);
	void (*win_disable)(struct tcc_drm_manager *mgr, int zpos);
	void (*te_handler)(struct tcc_drm_manager *mgr);
};

/*
 * Telechips drm common manager structure, maps 1:1 with a crtc
 *
 * @list: the list entry for this manager
 * @type: one of TCC_DISPLAY_TYPE_LCD and HDMI.
 * @drm_dev: pointer to the drm device
 * @crtc: crtc object.
 * @pipe: the pipe number for this crtc/manager
 * @ops: pointer to callbacks for tcc drm specific functionality
 * @ctx: A pointer to the manager's implementation specific context
 */
struct tcc_drm_manager {
	struct list_head list;
	enum tcc_drm_output_type type;
	struct drm_device *drm_dev;
	struct drm_crtc *crtc;
	int pipe;
	struct tcc_drm_manager_ops *ops;
	void *ctx;
};

struct tcc_drm_g2d_private {
	struct device		*dev;
	struct list_head	inuse_cmdlist;
	struct list_head	event_list;
	struct list_head	userptr_list;
};

struct drm_tcc_file_private {
	struct tcc_drm_g2d_private	*g2d_priv;
	struct device			*ipp_dev;
};

/*
 * Telechips drm private structure.
 *
 * @da_start: start address to device address space.
 *	with iommu, device address space starts from this address
 *	otherwise default one.
 * @da_space_size: size of device address space.
 *	if 0 then default value is used for it.
 * @pipe: the pipe number for this crtc/manager.
 */
struct tcc_drm_private {
	struct drm_fb_helper *fb_helper;

	/* list head for new event to be added. */
	struct list_head pageflip_event_list;

	/*
	 * created crtc object would be contained at this array and
	 * this array is used to be aware of which crtc did it request vblank.
	 */
	struct drm_crtc *crtc[MAX_CRTC];
	struct drm_property *plane_zpos_property;
	struct drm_property *crtc_mode_property;

	unsigned long da_start;
	unsigned long da_space_size;

	unsigned int pipe;
};

/*
 * Telechips drm sub driver structure.
 *
 * @list: sub driver has its own list object to register to tcc drm driver.
 * @dev: pointer to device object for subdrv device driver.
 * @drm_dev: pointer to drm_device and this pointer would be set
 *	when sub driver calls tcc_drm_subdrv_register().
 * @manager: subdrv has its own manager to control a hardware appropriately
 *     and we can access a hardware drawing on this manager.
 * @probe: this callback would be called by tcc drm driver after
 *     subdrv is registered to it.
 * @remove: this callback is used to release resources created
 *     by probe callback.
 * @open: this would be called with drm device file open.
 * @close: this would be called with drm device file close.
 */
struct tcc_drm_subdrv {
	struct list_head list;
	struct device *dev;
	struct drm_device *drm_dev;

	int (*probe)(struct drm_device *drm_dev, struct device *dev);
	void (*remove)(struct drm_device *drm_dev, struct device *dev);
	int (*open)(struct drm_device *drm_dev, struct device *dev,
			struct drm_file *file);
	void (*close)(struct drm_device *drm_dev, struct device *dev,
			struct drm_file *file);
};

 /* This function would be called by non kms drivers such as g2d and ipp. */
int tcc_drm_subdrv_register(struct tcc_drm_subdrv *drm_subdrv);

/* this function removes subdrv list from tcc drm driver */
int tcc_drm_subdrv_unregister(struct tcc_drm_subdrv *drm_subdrv);

int tcc_drm_device_subdrv_probe(struct drm_device *dev);
int tcc_drm_device_subdrv_remove(struct drm_device *dev);
int tcc_drm_subdrv_open(struct drm_device *dev, struct drm_file *file);
void tcc_drm_subdrv_close(struct drm_device *dev, struct drm_file *file);

struct tcc_drm_display * tcc_dpi_probe(struct device *dev);
int tcc_dpi_remove(struct device *dev);

/*
 * this function registers tcc drm lcd platform device/driver.
 */
int tcc_drm_probe_lcd(void);

/*
 * this function unregister tcc drm lcd platform device/driver.
 */

void tcc_drm_remove_lcd(void);

/* This function creates a encoder and a connector, and initializes them. */
int tcc_drm_create_enc_conn(struct drm_device *dev,
				struct tcc_drm_display *display);

int tcc_drm_component_add(struct device *dev,
				enum tcc_drm_device_type dev_type,
				enum tcc_drm_output_type out_type);

void tcc_drm_component_del(struct device *dev,
				enum tcc_drm_device_type dev_type);

extern struct platform_driver lcd_driver;

#endif
