/*
 *  Advanced Linux Sound Architecture - ALSA - Driver
 *  Copyright (c) 1994-2003 by Jaroslav Kysela <perex@perex.cz>,
 *                             Abramo Bagnara <abramo@alsa-project.org>
 *
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */
#ifndef __SOUND_ASOUND_H
#define __SOUND_ASOUND_H

#include <linux/ioctl.h>
#include <linux/time.h>
#include <asm/byteorder.h>

#ifdef  __LITTLE_ENDIAN
#define SNDRV_LITTLE_ENDIAN
#else
#ifdef __BIG_ENDIAN
#define SNDRV_BIG_ENDIAN
#else
#error "Unsupported endian..."
#endif
#endif

#include <uapi/sound/asound.h>

/** 
* @author sjpark@cleinsoft
* @date 2014/05/20
* Support PCM & IIS IF switching
**/
#if defined(CONFIG_DAUDIO)
#define SNDRV_PCM_HW_PARAM_EFLAG0 0 /* use reserved area index 0 to pass the extra flags */
#if defined(CONFIG_SND_SOC_DAUDIO_SUPPORT_PCMIF)
	#define SNDRV_PCM_MODE_PCMIF (1<<0) /* pcm mode or iis mode */
#endif
#if defined(CONFIG_SND_SOC_DAUDIO_CLOCK_ROLE_SWITCHING)
	#define SNDRV_PCM_MODE_SLAVE (1<<1) /* master or slave mode */
#endif
#endif

#if defined(CONFIG_SND_SOC_TCC_CDIF)
    #define SNDRV_PCM_MODE_CDIF (1<<2) /* cdif mode */
#endif

#endif /* __SOUND_ASOUND_H */
