/****************************************************************************
 * FileName    : kernel/drivers/char/hdmi_v1_3/cec/cec.c
 * Description : hdmi cec driver
 *
 * Copyright (C) 2013 Telechips Inc.
 *
 * This program is free software; you can redistribute it and/or modify it under the terms
 * of the GNU General Public License as published by the Free Software Foundation;
 * either version 2 of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc., 59 Temple Place,
 * Suite 330, Boston, MA 02111-1307 USA
 * ****************************************************************************/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/errno.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <asm/io.h>

#ifdef CONFIG_PM_RUNTIME
#include <linux/pm_runtime.h>
#endif

#ifdef CONFIG_PM
#include <linux/pm.h>
#endif
#include <asm/mach-types.h>
#include <asm/uaccess.h>

#include "../hdmi/regs-hdmi.h"

#include <mach/cec.h>
#include <mach/gpio.h>

#include "regs-cec.h"

#include "tcc_cec_interface.h"

#define CEC_DEBUG 0

#if CEC_DEBUG
#define DPRINTK(args...)    printk(args)
#else
#define DPRINTK(args...)
#endif


#define VERSION   "1.0" /* Driver version number */
#define CEC_MINOR 242 /* Major 10, Minor 242, /dev/cec */

#define CEC_MESSAGE_BROADCAST_MASK    0x0F
#define CEC_MESSAGE_BROADCAST         0x0F

#define CEC_FILTER_THRESHOLD          0x15

static unsigned cec_port;
static int cec_irq;


/**
 * @enum cec_state
 * Defines all possible states of CEC software state machine
 */
enum cec_state {
    STATE_RX,
    STATE_TX,
    STATE_DONE,
    STATE_ERROR
};

/**
 * @struct cec_rx_struct
 * Holds CEC Rx state and data
 */
struct cec_rx_struct {
    spinlock_t lock;
    wait_queue_head_t waitq;
    atomic_t state;
    u8 *buffer;
    unsigned int size;
	unsigned int flag;
};

/**
 * @struct cec_tx_struct
 * Holds CEC Tx state and data
 */
struct cec_tx_struct {
    wait_queue_head_t waitq;
    atomic_t state;
};

static struct clk *hdmi_cec_hclk;
static struct clk *hdmi_cec_pclk;

static struct cec_rx_struct cec_rx_struct;
static struct cec_tx_struct cec_tx_struct;

static unsigned int cec_resume_status = 0;

extern unsigned int standby_status;
extern unsigned char hdmi_get_power_status(void);

static int cec_open(struct inode *inode, struct file *file);
static int cec_release(struct inode *inode, struct file *file);
static ssize_t cec_read(struct file *file, char __user *buffer, size_t count, loff_t *ppos);
static ssize_t cec_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos);
static long cec_ioctl(struct file *file, unsigned int cmd, unsigned long arg);
static unsigned int cec_poll(struct file *file, poll_table *wait);

static irqreturn_t cec_irq_handler(int irq, void *dev_id);

static void cec_start(void);
static void cec_stop(void);

inline static void cec_set_divider(void);
inline static void cec_enable_interrupts(void);
inline static void cec_disable_interrupts(void);
inline static void cec_enable_rx(void);
inline static void cec_mask_rx_interrupts(void);
inline static void cec_unmask_rx_interrupts(void);
inline static void cec_mask_tx_interrupts(void);
inline static void cec_unmask_tx_interrupts(void);
inline static void cec_set_rx_state(enum cec_state state);
inline static void cec_set_tx_state(enum cec_state state);

static inline u8 cec_readb (unsigned offset){
    return readb (IOMEM(offset));
}
static inline void cec_writeb(u8 b, unsigned offset){
    writeb(b, IOMEM(offset));
}
static inline u32 cec_readl(unsigned offset){
    return readl(IOMEM(offset));
}
static inline void cec_writel(u32 b, unsigned offset){
    writel(b, IOMEM(offset));
}

static const struct file_operations cec_fops =
{
    .owner          = THIS_MODULE,
    .open           = cec_open,
    .release        = cec_release,
    .read           = cec_read,
    .write          = cec_write,
    .unlocked_ioctl = cec_ioctl,
    .poll           = cec_poll,
};

static struct miscdevice cec_misc_device =
{
    .minor = CEC_MINOR,
    .name  = "CEC",
    .fops  = &cec_fops,
};

static struct device *pdev_cec;

int cec_open(struct inode *inode, struct file *file)
{
	DPRINTK(KERN_INFO "%s\n", __FUNCTION__);

	clk_prepare_enable(hdmi_cec_hclk);
	clk_prepare_enable(hdmi_cec_pclk);

    return 0;
}

int cec_release(struct inode *inode, struct file *file)
{
    DPRINTK(KERN_INFO "%s\n", __FUNCTION__);

	clk_disable_unprepare(hdmi_cec_pclk);
	clk_disable_unprepare(hdmi_cec_hclk);

    return 0;
}

ssize_t cec_read(struct file *file, char __user *buffer, size_t count, loff_t *ppos)
{
    ssize_t retval;

    if (atomic_read(&cec_rx_struct.state) == STATE_ERROR) {
		DPRINTK(KERN_INFO "%s end, because of RX error\n", __FUNCTION__);
		return -1;
    }

	if( cec_rx_struct.flag != 1)
		return 0;

    DPRINTK(KERN_INFO "%s start\n", __FUNCTION__);

#if 0
    if (wait_event_interruptible(cec_rx_struct.waitq, atomic_read(&cec_rx_struct.state) == STATE_DONE)) {
        return -ERESTARTSYS;
    }
#endif /* 0 */

    spin_lock_irq(&cec_rx_struct.lock);

    if (cec_rx_struct.size > count) {
        spin_unlock_irq(&cec_rx_struct.lock);
        return -1;
    }

    if (copy_to_user(buffer, cec_rx_struct.buffer, cec_rx_struct.size)) {
        spin_unlock_irq(&cec_rx_struct.lock);
        printk(KERN_ERR "CEC: copy_to_user() failed!\n");
        return -EFAULT;
    }

    retval = cec_rx_struct.size;
    cec_set_rx_state(STATE_RX);
    spin_unlock_irq(&cec_rx_struct.lock);

	cec_rx_struct.flag = 0;

	DPRINTK(KERN_INFO "%s end\n", __FUNCTION__);

    return retval;
}

ssize_t cec_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
    char *data;
    unsigned char reg;
    int i = 0;

    DPRINTK(KERN_INFO "%s\n", __FUNCTION__);

    /* check data size */
    if (count > CEC_TX_BUFF_SIZE || count == 0)
        return -1;

    data = kmalloc(count, GFP_KERNEL);
    if (!data)
    {
        printk(KERN_ERR "CEC: kmalloc() failed!\n");
        return -1;
    }

    if (copy_from_user(data, buffer, count))
    {
        printk(KERN_ERR "CEC: copy_from_user() failed!\n");
        kfree(data);
        return -EFAULT;
    }

    /* copy packet to hardware buffer */
    while (i < count) {
        cec_writeb(data[i], CEC_TX_BUFF0 + (i*4));
        i++;
    }

    /* set number of bytes to transfer */
    cec_writeb(count, CEC_TX_BYTES);

    cec_set_tx_state(STATE_TX);

    /* start transfer */
    reg = cec_readb (CEC_TX_CTRL);
    reg |= CEC_TX_CTRL_START;

    /* if message is broadcast message - set corresponding bit */
    if ((data[0] & CEC_MESSAGE_BROADCAST_MASK) == CEC_MESSAGE_BROADCAST)
        reg |= CEC_TX_CTRL_BCAST;
    else
        reg &= ~CEC_TX_CTRL_BCAST;

    /* set number of retransmissions */
    reg |= 0x50;

    cec_writeb(reg, CEC_TX_CTRL);

    kfree(data);

    /* wait for interrupt */
    if (wait_event_interruptible(cec_tx_struct.waitq, atomic_read(&cec_tx_struct.state) != STATE_TX)) {
        return -ERESTARTSYS;
    }

    if (atomic_read(&cec_tx_struct.state) == STATE_ERROR) {
		DPRINTK(KERN_INFO "%s end, because of TX error\n", __FUNCTION__);
        return -1;
    }

	DPRINTK(KERN_INFO "%s end\n", __FUNCTION__);

    return count;
}

void cec_start(void)
{
    DPRINTK(KERN_INFO "%s\n", __FUNCTION__);

	tcc_gpio_config(cec_port, GPIO_FN(1));

    cec_writeb(CEC_RX_CTRL_RESET, CEC_RX_CTRL); // reset CEC Rx
    cec_writeb(CEC_TX_CTRL_RESET, CEC_TX_CTRL); // reset CEC Tx

    cec_set_divider();

    writeb(CEC_FILTER_THRESHOLD, CEC_RX_FILTER_TH); // setup filter

    cec_enable_interrupts();

    cec_unmask_tx_interrupts();

    cec_set_rx_state(STATE_RX);
    cec_unmask_rx_interrupts();
    cec_enable_rx();

	cec_rx_struct.flag = 0;
}

void cec_stop(void)
{
    u32 status;
	unsigned int wait_cnt = 0;

    DPRINTK(KERN_INFO "%s\n", __FUNCTION__);

	do {
	    status = cec_readb (CEC_STATUS_0);
	    status |= cec_readb (CEC_STATUS_1) << 8;
	    status |= cec_readb (CEC_STATUS_2) << 16;
	    status |= cec_readb (CEC_STATUS_3) << 24;

		udelay(1000);
		wait_cnt++;

	}while(status & CEC_STATUS_TX_RUNNING);

    cec_mask_tx_interrupts();
    cec_mask_rx_interrupts();
    cec_disable_interrupts();

	printk("%s : wait_cnt = %d\n", __func__, wait_cnt);

}

/**
 *      tccfb_blank
 *	@blank_mode: the blank mode we want.
 *	@info: frame buffer structure that represents a single frame buffer
 *
 *	Blank the screen if blank_mode != 0, else unblank. Return 0 if
 *	blanking succeeded, != 0 if un-/blanking failed due to e.g. a
 *	video mode which doesn't support it. Implements VESA suspend
 *	and powerdown modes on hardware that supports disabling hsync/vsync:
 *	blank_mode == 2: suspend vsync
 *	blank_mode == 3: suspend hsync
 *	blank_mode == 4: powerdown
 *
 *	Returns negative errno on error, or zero on success.
 *
 */
#ifdef CONFIG_PM_RUNTIME

static int cec_enable(void)
{		
	printk("%s\n", __func__);

	pm_runtime_get_sync(pdev_cec);

	return 0;
}

static int cec_disable(void)
{
	printk("%s\n", __func__);

	pm_runtime_put_sync(pdev_cec);

	return 0;
}

#endif

static int cec_blank(int blank_mode)
{
	int ret = 0;

	printk("%s : blank(mode=%d)\n", __func__, blank_mode);

#ifdef CONFIG_PM_RUNTIME
	if( (pdev_cec->power.usage_count.counter==1) && (blank_mode == 0))
	{
		// usage_count = 1 ( resume ), blank_mode = 0 ( FB_BLANK_UNBLANK ) is stable state when booting
		// don't call runtime_suspend or resume state 
	        //printk("%s ### state = [%d] count =[%d] power_cnt=[%d] \n",__func__,blank_mode, pdev_cec->power.usage_count, pdev_cec->power.usage_count.counter);		  
		return 0;
	}

	switch(blank_mode)
	{
		case FB_BLANK_POWERDOWN:
		case FB_BLANK_NORMAL:
			cec_disable();
			break;
		case FB_BLANK_UNBLANK:
			cec_enable();
			break;
		case FB_BLANK_HSYNC_SUSPEND:
		case FB_BLANK_VSYNC_SUSPEND:
		default:
			ret = -EINVAL;
	}
#endif

	return ret;
}



long  cec_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    unsigned int laddr;
	unsigned int uiData;
	unsigned int resume_status;

    DPRINTK(KERN_INFO "%s\n", __FUNCTION__);

    switch (cmd) {
		case CEC_IOC_START:
			cec_start();
			break;
		case CEC_IOC_STOP:
			cec_stop();
			break;
        case CEC_IOC_SETLADDR:
            DPRINTK(KERN_INFO "CEC: ioctl(CEC_IOC_SETLADDR)\n");
            if (get_user(laddr, (unsigned int __user *) arg))
                return -EFAULT;
            DPRINTK(KERN_INFO "CEC: logical address = 0x%02x\n", laddr);
            cec_writeb(laddr & 0x0F, CEC_LOGIC_ADDR);
            break;
		case CEC_IOC_SENDDATA:
			DPRINTK(KERN_INFO "CEC: ioctl(CEC_IOC_SENDDATA)\n");
            if (get_user(uiData, (unsigned int __user *) arg))
                return -EFAULT;
			DPRINTK(KERN_INFO "CEC: SendData = 0x%02x\n", uiData);
			TccCECInterface_SendData( 0, uiData);
			break;
		case CEC_IOC_RECVDATA:
			break;

		case CEC_IOC_GETRESUMESTATUS:
			DPRINTK(KERN_INFO "CEC: ioctl(CEC_IOC_GETRESUMESTATUS)\n");
			resume_status = cec_resume_status;
			put_user(resume_status,(unsigned char __user*)arg);
			break;
		case CEC_IOC_CLRRESUMESTATUS:
			DPRINTK(KERN_INFO "CEC: ioctl(CEC_IOC_CLRRESUMESTATUS)\n");
			if (get_user(resume_status, (unsigned int __user *) arg))
				return -EFAULT;
			cec_resume_status = resume_status;
			break;
		case CEC_IOC_BLANK:
		{
			unsigned int cmd;

			if (get_user(cmd, (unsigned int __user *) arg))
				return -EFAULT;

			printk(KERN_INFO "CEC: ioctl(CEC_IOC_BLANK :  %d )\n", cmd);

			cec_blank(cmd);

			break;

		}
        default:
            return -EINVAL;
    }

    return 0;
}

unsigned int cec_poll(struct file *file, poll_table *wait)
{
//    DPRINTK(KERN_INFO "%s\n", __FUNCTION__);

    poll_wait(file, &cec_rx_struct.waitq, wait);

    if (atomic_read(&cec_rx_struct.state) == STATE_DONE)
        return POLLIN | POLLRDNORM;

    return 0;
}

/**
 * @brief CEC interrupt handler
 *
 * Handles interrupt requests from CEC hardware. \n
 * Action depends on current state of CEC hardware.
 */
irqreturn_t cec_irq_handler(int irq, void *dev_id)
{
    u8 flag;
    u32 status;
#if (CEC_DEBUG)
	u32 tx_stat_23=0;
	u32 rx_stat_23 = 0;
#endif
    /* read flag register */
    flag = cec_readb (HDMI_SS_INTC_FLAG);

	DPRINTK(KERN_INFO "%s flag = 0x%08x\n", __FUNCTION__, flag);

    /* is this our interrupt? */
    if (!(flag & (1<<HDMI_IRQ_CEC))) {
        return IRQ_NONE;
    }

    DPRINTK(KERN_INFO "%s\n", __FUNCTION__);

    status = cec_readb (CEC_STATUS_0);
    status |= cec_readb (CEC_STATUS_1) << 8;
    status |= cec_readb (CEC_STATUS_2) << 16;
    status |= cec_readb (CEC_STATUS_3) << 24;

#if (CEC_DEBUG)
	tx_stat_23 = cec_readb (CEC_TX_STAT0);
	tx_stat_23 = cec_readb (CEC_TX_STAT1) << 8;

//	DPRINTK(KERN_INFO "CEC: status = 0x%x!\n", status);
	DPRINTK(KERN_INFO "CEC: status = 0x%x! [0x%x] \n", status, tx_stat_23);
#endif

    if (status & CEC_STATUS_TX_DONE) {
        if (status & CEC_STATUS_TX_ERROR) {
            DPRINTK(KERN_INFO "CEC: CEC_STATUS_TX_ERROR!\n");
			cec_writeb(CEC_TX_CTRL_RESET, CEC_TX_CTRL); // reset CEC Tx

            cec_set_tx_state(STATE_ERROR);
        } else {
            DPRINTK(KERN_INFO "CEC: CEC_STATUS_TX_DONE!\n");
            cec_set_tx_state(STATE_DONE);
        }
        /* clear interrupt pending bit */
        cec_writeb(CEC_IRQ_TX_DONE | CEC_IRQ_TX_ERROR, CEC_IRQ_CLEAR);
        wake_up_interruptible(&cec_tx_struct.waitq);
    }

    if (status & CEC_STATUS_RX_DONE) {
        if (status & CEC_STATUS_RX_ERROR) {
			DPRINTK(KERN_INFO "CEC: CEC_STATUS_RX_DONE!\n");
            DPRINTK(KERN_INFO "CEC: CEC_STATUS_RX_ERROR!\n");

			#if (CEC_DEBUG)
			rx_stat_23 = cec_readb (CEC_RX_STAT0);
			rx_stat_23 |= cec_readb (CEC_RX_STAT1) << 8;

			DPRINTK(KERN_INFO "CEC: rx_status = 0x%x\n", rx_stat_23);
			#endif

			cec_set_rx_state(STATE_ERROR);
        } else {
            unsigned int size, i = 0;

            DPRINTK(KERN_INFO "CEC: CEC_STATUS_RX_DONE!\n");

            /* copy data from internal buffer */
            size = status >> 24;

            spin_lock(&cec_rx_struct.lock);

            while (i < size) {
                cec_rx_struct.buffer[i] = cec_readb (CEC_RX_BUFF0 + (i*4));
                i++;
            }

    if (size > 0) {
        DPRINTK(KERN_INFO "fsize: %d ", size);
        DPRINTK(KERN_INFO "frame: ");
        for (i = 0; i < size; i++) {
            DPRINTK(KERN_INFO "0x%02x ", cec_rx_struct.buffer[i]);
        }
        DPRINTK(KERN_INFO "End ~\n");
    }

			TccCECInterface_ParseMessage(cec_rx_struct.buffer, size);
			
            cec_rx_struct.size = size;
            cec_set_rx_state(STATE_DONE);
			cec_rx_struct.flag = 1;

            spin_unlock(&cec_rx_struct.lock);

            cec_enable_rx();
        }
        /* clear interrupt pending bit */
        cec_writeb(CEC_IRQ_RX_DONE | CEC_IRQ_RX_ERROR, CEC_IRQ_CLEAR);
        wake_up_interruptible(&cec_rx_struct.waitq);
    }

    if (status & CEC_STATUS_RX_ERROR) {
        DPRINTK(KERN_INFO "CEC: CEC_STATUS_RX_ERROR!\n");

		#if (CEC_DEBUG)
		rx_stat_23 = cec_readb (CEC_RX_STAT0);
		rx_stat_23 |= cec_readb (CEC_RX_STAT1) << 8;

		DPRINTK(KERN_INFO "CEC: rx_status = 0x%x\n", rx_stat_23);
		#endif

		cec_set_rx_state(STATE_ERROR);

        /* clear interrupt pending bit */
        cec_writeb(CEC_IRQ_RX_ERROR, CEC_IRQ_CLEAR);
    }

    return IRQ_HANDLED;
}

static int cec_probe(struct platform_device *pdev)
{
    u8 *buffer;

	struct tcc_hdmi_platform_data *cec_dev;

	if (!pdev->dev.of_node) {
		cec_dev = (struct tcc_hdmi_platform_data *)pdev_cec->platform_data;
		if (!cec_dev) {
			dev_err(&pdev->dev, "no platform data\n");
			return -EINVAL;
		}
	}

	pdev_cec = &pdev->dev;
	
	cec_port = of_get_gpio(pdev->dev.of_node,0);;

	gpio_request(cec_port, "CEC");
	tcc_gpio_config(cec_port, GPIO_FN(1));
	
    DPRINTK(KERN_INFO "%s\n", __FUNCTION__);

    if (!machine_is_hdmidp())
        return -ENODEV;

	if (hdmi_cec_pclk == NULL && hdmi_cec_hclk == NULL ) {
	
		hdmi_cec_hclk = of_clk_get(pdev->dev.of_node, 1);
		hdmi_cec_pclk = of_clk_get(pdev->dev.of_node, 0);

		if (IS_ERR(hdmi_cec_hclk)) {
			printk(KERN_WARNING "HDMI: failed to get cec hclock\n");
			hdmi_cec_hclk = NULL;
			return -ENODEV;
		}

		if (IS_ERR(hdmi_cec_pclk)) {
			printk(KERN_WARNING "HDMI: failed to get cec pclock\n");
			hdmi_cec_pclk = NULL;
			return -ENODEV;
		}
	}
	
	if(hdmi_cec_pclk)
		clk_prepare_enable(hdmi_cec_pclk);
	
	if(hdmi_cec_hclk)
		clk_prepare_enable(hdmi_cec_hclk);

	printk(KERN_INFO "CEC Driver ver. %s\n", VERSION);

	cec_irq = platform_get_irq(pdev, 0);
	if (cec_irq < 0) {
		dev_err(&pdev->dev, "no irq resource?\n");
        return -EIO;
	}

	if (misc_register(&cec_misc_device))
	{
	    printk(KERN_WARNING "CEC: Couldn't register device 10, %d.\n", CEC_MINOR);
	    return -EBUSY;
	}

    cec_disable_interrupts();

	if (request_irq(cec_irq, cec_irq_handler, IRQF_SHARED, "cec", cec_irq_handler))
	{
	    printk(KERN_WARNING "CEC: IRQ %d is not free.\n", cec_irq);
	    misc_deregister(&cec_misc_device);
	    return -EIO;
	}

	init_waitqueue_head(&cec_rx_struct.waitq);
	spin_lock_init(&cec_rx_struct.lock);
	init_waitqueue_head(&cec_tx_struct.waitq);

	buffer = kmalloc(CEC_TX_BUFF_SIZE, GFP_KERNEL);
	if (!buffer)
	{
	    printk(KERN_ERR "CEC: kmalloc() failed!\n");
	    misc_deregister(&cec_misc_device);
	    return -EIO;
	}

	cec_rx_struct.buffer = buffer;
	cec_rx_struct.size   = 0;

	TccCECInterface_Init();

#ifdef CONFIG_PM_RUNTIME
	pm_runtime_set_active(pdev_cec);	
	pm_runtime_enable(pdev_cec);  
	pm_runtime_get_noresume(pdev_cec);  //increase usage_count 
#endif

	if(hdmi_cec_pclk)
		clk_disable(hdmi_cec_pclk);

	if(hdmi_cec_hclk)
		clk_disable(hdmi_cec_hclk);

    return 0;
}

static int cec_remove(struct platform_device *pdev)
{
    DPRINTK(KERN_INFO "%s\n", __FUNCTION__);

    free_irq(cec_irq, cec_irq_handler);
    misc_deregister(&cec_misc_device);

    kfree(cec_rx_struct.buffer);

	return 0;
}

#ifdef CONFIG_PM_RUNTIME
int cec_runtime_suspend(struct device *dev)
{
	printk("%s:  \n", __FUNCTION__);

	if(hdmi_get_power_status())
		cec_stop();

	tcc_gpio_config(cec_port, GPIO_FN(1));
	
	cec_resume_status = 0;

	printk("%s: finish \n", __FUNCTION__);

	return 0;
}

int cec_runtime_resume(struct device *dev)
{
	printk("%s:  \n", __FUNCTION__);

	tcc_gpio_config(cec_port, GPIO_FN(1));
	
	if(hdmi_get_power_status())
		cec_start();

	cec_resume_status = 1;
	standby_status = 0;

	return 0;
}

#endif

#ifdef CONFIG_PM
int cec_suspend(struct platform_device *pdev, pm_message_t state)
{
	DPRINTK(KERN_INFO "%s\n", __FUNCTION__);

	return 0;
}

int cec_resume(struct platform_device *pdev)
{
	DPRINTK(KERN_INFO "%s\n", __FUNCTION__);

	return 0;
}
#else
#define cec_suspend NULL
#define cec_resume NULL
#endif

/**
 * Set CEC divider value.
 */
void cec_set_divider(void)
{
    /**
     * (CEC_DIVISOR) * (clock cycle time) = 0.05ms \n
     * for 27MHz clock divisor is 0x0545
     */
	#if (0)
	cec_writeb(0x05, CEC_DIVISOR_1);
	cec_writeb(0x45, CEC_DIVISOR_0);
	#else
	
	unsigned int ddibus_clk;
	unsigned int div = 0;
	ddibus_clk = clk_get_rate(hdmi_cec_hclk);

	ddibus_clk = ddibus_clk/(1000*1000);
	div = ddibus_clk * 50 - 1;

	DPRINTK("cec src clk = %dmhz\n", ddibus_clk);
	DPRINTK("cec divisor = %d\n", div);

	cec_writeb((unsigned char)((div>>24)&0xff), CEC_DIVISOR_3);
	cec_writeb((unsigned char)((div>>16)&0xff), CEC_DIVISOR_2);
	cec_writeb((unsigned char)((div>> 8)&0xff), CEC_DIVISOR_1);
	cec_writeb((unsigned char)((div>> 0)&0xff), CEC_DIVISOR_0);
	#endif
}

/**
 * Enable CEC interrupts
 */
void cec_enable_interrupts(void)
{
    unsigned char reg;
    reg = cec_readb (HDMI_SS_INTC_CON);
    cec_writeb(reg | (1<<HDMI_IRQ_CEC) | (1<<HDMI_IRQ_GLOBAL), HDMI_SS_INTC_CON);
}

/**
 * Disable CEC interrupts
 */
void cec_disable_interrupts(void)
{
    unsigned char reg;
    reg = cec_readb (HDMI_SS_INTC_CON);
    cec_writeb(reg & ~(1<<HDMI_IRQ_CEC), HDMI_SS_INTC_CON);
}

/**
 * Enable CEC Rx engine
 */
void cec_enable_rx(void)
{
    unsigned char reg;
    reg = cec_readb (CEC_RX_CTRL);
    reg |= CEC_RX_CTRL_ENABLE;
	reg |= CEC_RX_CTRL_CHK_SAMPLING_ERROR;
	reg |= CEC_RX_CTRL_CHK_LOW_TIME_ERROR;
	reg |= CEC_RX_CTRL_CHK_START_BIT_ERROR;
    cec_writeb(reg, CEC_RX_CTRL);
}

/**
 * Mask CEC Rx interrupts
 */
void cec_mask_rx_interrupts(void)
{
    unsigned char reg;
    reg = cec_readb (CEC_IRQ_MASK);
    reg |= CEC_IRQ_RX_DONE;
    reg |= CEC_IRQ_RX_ERROR;
    cec_writeb(reg, CEC_IRQ_MASK);
}

/**
 * Unmask CEC Rx interrupts
 */
void cec_unmask_rx_interrupts(void)
{
    unsigned char reg;
    reg = cec_readb (CEC_IRQ_MASK);
    reg &= ~CEC_IRQ_RX_DONE;
    reg &= ~CEC_IRQ_RX_ERROR;
    cec_writeb(reg, CEC_IRQ_MASK);
}

/**
 * Mask CEC Tx interrupts
 */
void cec_mask_tx_interrupts(void)
{
    unsigned char reg;
    reg = cec_readb (CEC_IRQ_MASK);
    reg |= CEC_IRQ_TX_DONE;
    reg |= CEC_IRQ_TX_ERROR;
    cec_writeb(reg, CEC_IRQ_MASK);
}

/**
 * Unmask CEC Tx interrupts
 */
void cec_unmask_tx_interrupts(void)
{
    unsigned char reg;
    reg = cec_readb (CEC_IRQ_MASK);
    reg &= ~CEC_IRQ_TX_DONE;
    reg &= ~CEC_IRQ_TX_ERROR;
    cec_writeb(reg, CEC_IRQ_MASK);
}

/**
 * Change CEC Tx state to state
 * @param state [in] new CEC Tx state.
 */
void cec_set_tx_state(enum cec_state state)
{
    atomic_set(&cec_tx_struct.state, state);
}

/**
 * Change CEC Rx state to @c state.
 * @param state [in] new CEC Rx state.
 */
void cec_set_rx_state(enum cec_state state)
{
    atomic_set(&cec_rx_struct.state, state);
}

#if 0
#ifdef CONFIG_OF
static void cec_parse_dt(struct device_node *np, struct tcc_hdmi_cec *cec)
{
}
#else
static void cec_parse_dt(struct device_node *np, struct tcc_hdmi_cec *cec)
{
}
#endif
#endif


#ifdef CONFIG_OF
static struct of_device_id cec_of_match[] = {
	{ .compatible = "telechips,tcc893x-hdmi-cec" },
	{}
};
MODULE_DEVICE_TABLE(of, cec_of_match);
#endif


#ifdef CONFIG_PM
static const struct dev_pm_ops cec_pm_ops = {
#ifdef CONFIG_PM_RUNTIME
	.runtime_suspend      = cec_runtime_suspend,
	.runtime_resume       = cec_runtime_resume,
#else
	.suspend	= cec_suspend,
	.resume = cec_resume,
#endif
};
#endif


static struct platform_driver tcc_hdmi_cec = {
	.probe	= cec_probe,
	.remove	= cec_remove,
	.driver	= {
		.name	= "tcc_hdmi_cec",
		.owner	= THIS_MODULE,
#ifdef CONFIG_PM_RUNTIME
		.pm		= &cec_pm_ops,
#endif
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(cec_of_match),
#endif
	},
};


static __init int cec_init(void)
{
	DPRINTK(KERN_INFO "%s\n", __FUNCTION__);

	return platform_driver_register(&tcc_hdmi_cec);
}

static __exit void cec_exit(void)
{
	DPRINTK(KERN_INFO "%s\n", __FUNCTION__);

	platform_driver_unregister(&tcc_hdmi_cec);
}


module_init(cec_init);
module_exit(cec_exit);

MODULE_AUTHOR("Telechips Inc. <linux@telechips.com>");
MODULE_DESCRIPTION("TCCxxx hdmi cec driver");
MODULE_LICENSE("GPL");


