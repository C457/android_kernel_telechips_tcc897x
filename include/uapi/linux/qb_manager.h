//+[TCCQB] QuickBootManager
/*
 * include/linux/qb_manager.h
 *
 * TCC QUICKBOOT HEADER
 *
 * Copyright (C) 2009 Telechips, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */


/*	
 *	Property Definitions should be same with frameworks/base/core/java/android/os/QBManager.java
 *	If this header is changed, copy it to external/kernel-headers/original/linux/qb_manager.h
 *	Also, excute bionic/libc/kernel/tools/update_all.py,
 *	then bionic/libc/kernel/common/linux/qb_manager.h file is created.
 *	After bionic/ ~ /qb_manager.h is created, it would be effect to native sources.
 */


/********************************************************************
 *                      QuickBoot Properties                        *
 ********************************************************************/

#define QBMANAGER_QB_ENABLED		"ro.QB.enable"

/*          System Properties                   */
#define QBMANAGER_MK_QB				"persist.sys.QB.user.allow"
#define QBMANAGER_SWAP_PATH			"tcc.swap.path"
#define QBMANAGER_RELOADED			"tcc.hibernate.property"
#define QBMANAGER_BOOT_WITH			"tcc.QB.boot.with"
#define QBMANAGER_SYSTEM_UPDATED	"persist.sys.QB.updated"
#define QBMANAGER_ RELOAD_PI		"tcc.QB.reloadPI.flag"
#define QBMANAGER_BOOT_TIMEOUT		"persist.sys.QB.home.timeout"
#define QBMANAGER_SKIP_BOOT			"persist.sys.QB.skip.qb_popup"

/*          Services Properties                 */
#define QBMANAGER_TIME_ZONE			"tcc.QB.timezone"
#define QBMANAGER_DEFAULT_IME		"persist.QB.imm.reset"
#define QBMANAGER_USB_CONFIG		"persist.sys.usb.config"

/*          Log Properties                      */
#define QBMANAGER_FW_BOOTTIME		"tcc.QB.start.framework"
#define QBMANAGER_PROFILE_DLOG		"tcc.QB.profile.dlog"
#define QBMANAGER_PROFILE_DTIME		"tcc.QB.profile.dtime"

/*          Aging Properties                    */
#define QBMANAGER_AGING_CNT			"persist.sys.QB.aging.count"
#define QBMANAGER_AGING_CONT		"persist.sys.QB.aging"
#define QBMANAGER_AGING_DUMP		"persist.sys.QB.aging.dump"


//-[TCCQB]
//
