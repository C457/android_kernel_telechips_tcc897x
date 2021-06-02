/*!
* TCC Version 1.0
* Copyright (c) Telechips Inc.
* All rights reserved 
*  \file        include.h
*  \brief       HDMI TX controller driver
*  \details   
*  \version     1.0
*  \date        2014-2015
*  \copyright
This source code contains confidential information of Telechips.
Any unauthorized use without a written  permission  of Telechips including not 
limited to re-distribution in source  or binary  form  is strictly prohibited.
This source  code is  provided "AS IS"and nothing contained in this source 
code  shall  constitute any express  or implied warranty of any kind, including
without limitation, any warranty of merchantability, fitness for a   particular 
purpose or non-infringement  of  any  patent,  copyright  or  other third party 
intellectual property right. No warranty is made, express or implied, regarding 
the information's accuracy, completeness, or performance. 
In no event shall Telechips be liable for any claim, damages or other liability 
arising from, out of or in connection with this source  code or the  use in the 
source code. 
This source code is provided subject  to the  terms of a Mutual  Non-Disclosure 
Agreement between Telechips and Company.
*******************************************************************************/
#ifndef __INCLUDES_H__
#define __INCLUDES_H__

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/kthread.h>
#include <linux/freezer.h>
#include <linux/semaphore.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/of_irq.h>
#include <linux/list.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/proc_fs.h>
#include <linux/fb.h>
#include <linux/mm.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/of.h>
#include <linux/of_address.h> // of_iomap
#include <linux/poll.h> // poll_table
#include <linux/device.h> // dev_xet_drv_data
#include <asm/io.h>


// HDMI COMPONENTS
#define PROTO_HDMI_TX_CORE      0
#define PROTO_HDMI_TX_HDCP      1
#define PROTO_HDMI_TX_PHY       2
#define PROTO_HDMI_TX_CEC       3
#define PROTO_HDMI_TX_MAX       4

// HDMI IRQS
#define HDMI_IRQ_TX_CORE        0
#define HDMI_IRQ_TX_HDCP        1
#define HDMI_IRQ_TX_CEC         2
#define HDMI_IRQ_TX_MAX         3

// HDMI CLOCKS
#define HDMI_CLK_INDEX_PHY27M   0
#define HDMI_CLK_INDEX_SPDIF    1        
#define HDMI_CLK_INDEX_PCLK     2
#define HDMI_CLK_INDEX_CEC      3
#define HDMI_CLK_INDEX_HDCP14   4
#define HDMI_CLK_INDEX_HDCP22   5
#define HDMI_CLK_INDEX_DDIBUS   6
#define HDMI_CLK_INDEX_PHY      7
#define HDMI_CLK_INDEX_MAX      8


// STATUS BITS
#define HDMI_STATUS_BIT_CORE_POWERON      1


// CLOCK FREQ
#define HDMI_PHY_REF_CLK_RATE (24000000)
#define HDMI_HDCP14_CLK_RATE (50000000)
#define HDMI_HDCP22_CLK_RATE (50000000)
#define HDMI_SPDIF_REF_CLK_RATE (44100*64*2)

/**
 * This macro only works in C99 so you can disable this, by placing
 * #define FUNC_NAME "HDMI_RX", or __FUNCTION__
 */
#define FUNC_NAME __func__

/**
 * Verbose levels
 */
typedef enum {
	VERBOSE_BASIC = 1,
	VERBOSE_IO,
	VERBOSE_IRQ
}verbose_level;


enum {
        MUTE_ID_IRQ=1,
        MUTE_ID_API,
        MUTE_ID_HPD,
        MUTE_ID_DDC,

};


/**
 * @short HDMI TX controller status information
 *
 * Initialize @b user fields (set status to zero).
 * After opening this data is for internal use only.
 */
struct hdmi_tx_ctrl {
        /** (@b user) Context status: closed (0), opened (<0) and
         *  configured (>0) */
        //int status;

        uint8_t data_enable_polarity;

        /** This is used to check if a cable is connected and if so the
         * assume green light to configure the the core */
        //int hpd;

        uint32_t pixel_clock;
        uint8_t pixel_repetition;
        uint8_t color_resolution;
        uint8_t csc_on;
        uint8_t audio_on;
        uint8_t cec_on;
        uint8_t hdcp_on;
        uint8_t hdmi_on;
};


struct drv_enable_entry
{
    struct list_head list;
    unsigned int id;
};


/**
 * @short Main structures to instantiate the driver
 */
struct hdmi_tx_dev{

	/** Device node */
	struct device 		*parent_dev;
        
	/** Device list */
	struct list_head 	devlist;
        
        /** Misc Device */
        struct miscdevice       *misc;

	/** Verbose */
	int 			verbose;

	/** Interrupts */
	uint32_t		irq[PROTO_HDMI_TX_MAX];

        /** Clock */
        struct clk              *clk[HDMI_CLK_INDEX_MAX];

        unsigned int            clk_freq_pclk;

        /** Display device ide of HDMI output */
        unsigned int            display_device;

        /** Device Open Count */
        int                     open_cs;
        int                     clock_enable_count;


        struct drv_enable_entry irq_enable_entry;
                
        #if defined(CONFIG_REGULATOR)
        struct regulator        *regulator;
        #endif
        
	/** Spinlock */
	spinlock_t 		slock;
	/** Mutex */
	struct mutex 		mutex;

	/** Device Tree Information */
	char 			*device_name;
	/** HDMI TX Controller */
	volatile void __iomem   *dwc_hdmi_tx_core_io;

	/** HDCP */
        volatile void __iomem   *hdcp_io;
        
	/** HDMI TX PHY interface */
        volatile void __iomem   *hdmi_tx_phy_if_io;

        /** DDI_BUS interface */
        volatile void __iomem   *ddibus_io;

        /** CKC interface */
        volatile void __iomem   *io_bus;

		unsigned int	audio_offset;
        
	/** EDID block of data */
	uint8_t 		edid_data[128];

	/** Proc file system */
	struct proc_dir_entry	*hdmi_proc_dir;
	struct proc_dir_entry	*hdmi_proc_hdcp_keys;
	struct proc_dir_entry   *hdmi_proc_hpd;
	struct proc_dir_entry   *hdmi_proc_hdcp22;

	int                     hpd;
        int                     hpd_enable;
        
	int                     hdcp22;
        
        volatile unsigned long  status;

        // Controller Information. 

        struct hdmi_tx_ctrl     hdmi_tx_ctrl;

        void                    *videoParam;
        void                    *audioParam;
        void                    *productParam;
        void                    *hdcpParam;
        
        /** Interrupt Work */
        struct work_struct      tx_handler;        
};

/**
 * @brief Dynamic memory allocation
 * Instance 0 will have the total size allocated so far and also the number
 * of calls to this function (number of allocated instances)
 */
struct mem_alloc{
	int instance;
	const char *info;
	size_t size;
	void *pointer; // the pointer allocated by this instance
	struct mem_alloc *last;
	struct mem_alloc *prev;
};

/**
 * @short Allocate memory for the driver
 * @param[in] into allocation name
 * @param[in] size allocation size
 * @param[in,out] allocated return structure for the allocation, may be NULL
 * @return if successful, the pointer to the new created memory
 * 	   if not, NULL
 */
void *
alloc_mem(char *info, size_t size, struct mem_alloc *allocated);

#endif /* __INCLUDES_H__ */
