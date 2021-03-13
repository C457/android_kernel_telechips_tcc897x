/*
 * (C) COPYRIGHT 2014 Telechips Inc.
 * (C) COPYRIGHT ARM Limited. All rights reserved.
 *
 * This program is free software and is provided to you under the terms of the
 * GNU General Public License version 2 as published by the Free Software
 * Foundation, and any use by you of this program is subject to the terms
 * of such GNU licence.
 *
 * A copy of the licence is included with the program, and can also be obtained
 * from Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
 * Boston, MA  02110-1301, USA.
 *
 */



#include "mali_kbase_cpu_TCC8960.h"

/**
 * Maximum frequency GPU will be clocked at. Given in kHz.
 * This must be specified as there is no default value.
 *
 * Attached value: number in kHz
 * Default value: NA
 */
#define GPU_FREQ_KHZ_MAX (5000)
/**
 * Minimum frequency GPU will be clocked at. Given in kHz.
 * This must be specified as there is no default value.
 *
 * Attached value: number in kHz
 * Default value: NA
 */
#define GPU_FREQ_KHZ_MIN (5000)
#define CPU_SPEED_FUNC (&kbase_get_TCC8960_cpu_clock_speed)
#define GPU_SPEED_FUNC (NULL)
#define POWER_MANAGEMENT_CALLBACKS (&pm_callbacks)
#define PLATFORM_FUNCS (NULL)
extern struct kbase_pm_callback_conf pm_callbacks;
