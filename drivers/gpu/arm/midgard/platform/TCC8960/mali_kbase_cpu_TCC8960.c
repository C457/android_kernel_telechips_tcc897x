/****************************************************************************
One line to give the program's name and a brief idea of what it does.
Copyright (C) 2014 Telechips Inc.

This program is free software; you can redistribute it and/or modify it under the terms
of the GNU General Public License as published by the Free Software Foundation;
either version 2 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
this program; if not, write to the Free Software Foundation, Inc., 59 Temple Place,
Suite 330, Boston, MA 02111-1307 USA
****************************************************************************/

#include <linux/io.h>
#include <mali_kbase.h>
#include "mali_kbase_cpu_TCC8960.h"

/**
 * kbase_get_TCC8960_cpu_clock_speed
 * @brief  Retrieves the CPU clock speed.
 * @param[out]    cpu_clock - the value of CPU clock speed in MHz
 * @return        0 on success, 1 otherwise
*/
int kbase_get_TCC8960_cpu_clock_speed(u32 *cpu_clock)
{
	return 0;
}
