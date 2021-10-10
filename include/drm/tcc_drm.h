/* tcc_drm.h
 *
 * Copyright (C) 2015-2016 Telechips
 *
 * Author: Telechips Inc.
 * Based on Samsung Exynos code
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
#ifndef _TCC_DRM_H_
#define _TCC_DRM_H_

#include <uapi/drm/tcc_drm.h>
#include <video/videomode.h>


/**
 * A structure for lcd panel information.
 *
 * @timing: default video mode for initializing
 * @width_mm: physical size of lcd width.
 * @height_mm: physical size of lcd height.
 */
struct tcc_drm_panel_info {
	struct videomode vm;
	u32 width_mm;
	u32 height_mm;
};

/**
 * Platform Specific Structure for DRM based LCD.
 *
 * @panel: default panel info for initializing
 * @default_win: default window layer number to be used for UI.
 * @bpp: default bit per pixel.
 * @nr_disp: TCC specific vioc display componet id
 */
struct tcc_drm_lcd_pdata {
	struct tcc_drm_panel_info panel;
	u32 vidcon0;
	u32 vidcon1;
	unsigned int default_win;
	unsigned int bpp;
	unsigned int nr_disp;	// vioc display component id
};

#endif	/* _TCC_DRM_H_ */
