/*
 * linux/drivers/i2c/busses/i2c-tcc.c
 *
 * Author:  <linux@telechips.com>
 * Created: 10th Jun, 2008
 * Description: Telechips I2C Controller
 *
 * Copyright (c) Telechips, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * ----------------------------
 * for TCC DIBCOM DVB-T module
 * [M] i2c-core.c, i2c-dev.c
 * [M] include/linux/i2c-dev.h, include/linux/i2c.h
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/completion.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/of_address.h>
//#include <linux/of_i2c.h>
#include <linux/of_device.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <linux/irq.h>

#define I2C_M_MODE              0x0040
#define I2C_M_WR_RD             0x0080
#define I2C_M_WM899x_CODEC      0x0100
#define I2C_M_TCC_IPOD		0x0200

/* I2C Configuration Reg. */
#define I2C_PRES		0x00
#define I2C_CTRL		0x04
#define I2C_TXR			0x08
#define I2C_CMD			0x0C
#define I2C_RXR			0x10
#define I2C_SR			0x14
#define I2C_TIME		0x18

/* I2C Port Configuration Reg. */
#define I2C_PORT_CFG0		0x0
#define I2C_PORT_CFG1		0x4
#define I2C_IRQ_STS		0xC

#define i2c_readl	__raw_readl
#define i2c_writel	__raw_writel

enum tcc_i2c_state {
	STATE_IDLE,
	STATE_START,
	STATE_READ,
	STATE_WRITE,
	STATE_STOP
};

struct tcc_i2c {
	spinlock_t			lock;
	wait_queue_head_t		wait;
	void __iomem			*regs;
	void __iomem			*port_cfg;

	int				core;
	unsigned int			i2c_clk_rate;
	unsigned int			core_clk_rate;
	struct clk			*pclk;
	struct clk			*hclk;
	struct i2c_msg			*msg;
	unsigned int			msg_num;
	unsigned int			msg_idx;
	unsigned int			msg_ptr;

	struct completion 		msg_complete;
	enum tcc_i2c_state		state;
	struct device			*dev;
	struct i2c_adapter		adap;
	bool				is_suspended;
	unsigned int			port_mux;

	int					irq;
	unsigned int 			interrupt_mode;
#if defined(CONFIG_I2C_DEBUG_BUS_SYSFS)
	unsigned int sent;
	unsigned int send_fail;
	unsigned int recv;
	unsigned int recv_fail;
#endif
};

#if defined(CONFIG_I2C_DEBUG_BUS_SYSFS)
enum
{
	I2C_SENT, I2C_RECV, I2C_SEND_FAIL, I2C_RECV_FAIL
};

//i2c->sent += (UINT_MAX-i2c->msg->len >= i2c->sent)?i2c->msg->len:0;

/**
 * clear_all_rec_idx - clear all record index
 * @i2c: tcc_i2c
 */
static void clear_all_rec_idx(struct tcc_i2c *i2c)
{
	i2c->sent = i2c->send_fail = i2c->recv = i2c->recv_fail = 0;
}

/**
 * incr_sent_bytes- increse i2c sent byte
 * @i2c: tcc_i2c
 * It returns number of bytes sent to now.
 * If sent byte exceed the UINT_MAX, it will return 0.
 */
static unsigned int incr_sent_bytes(struct tcc_i2c *i2c)
{
	if(i2c->sent + i2c->msg->len >= UINT_MAX)
		return 0;

	return i2c->sent += i2c->msg->len;
}

/**
 * incr_send_fail_bytes - increse i2c send fail byte
 * @i2c: tcc_i2c
 * It returns number of bytes send failed to now.
 * If sent byte exceed the UINT_MAX, it will return 0.
 */
static unsigned int incr_send_fail_bytes(struct tcc_i2c *i2c)
{
	if(i2c->send_fail + i2c->msg->len >= UINT_MAX)
		return 0;

	return i2c->send_fail += i2c->msg->len;
}

/**
 * incr_send_fail_bytes - increse i2c received byte
 * @i2c: tcc_i2c
 * It returns number of bytes received to now.
 * If sent byte exceed the UINT_MAX, it will return 0.
 */
static unsigned int incr_recv_bytes(struct tcc_i2c *i2c)
{
	if(i2c->recv + i2c->msg->len >= UINT_MAX)
		return 0;

	return i2c->recv += i2c->msg->len;
}

/**
 * incr_recv_fail_bytes - increse i2c receive fail byte
 * @i2c: tcc_i2c
 * It returns number of bytes receive failed to now.
 * If sent byte exceed the UINT_MAX, it will return 0.
 */
static unsigned int incr_recv_fail_bytes(struct tcc_i2c *i2c)
{
	if(i2c->recv_fail + i2c->msg->len >= UINT_MAX)
		return 0;

	return i2c->recv_fail += i2c->msg->len;
}

/**
 * incr_trans_bytes - count send (fail) or receive (fail) bytes 
 * @i2c: tcc_i2c
 * @type: transfer type 
 */
static int incr_trans_bytes(struct tcc_i2c *i2c, int type)
{
	unsigned int ret = 0;
	switch(type){
		case I2C_SENT:
			ret = incr_sent_bytes(i2c);
			break;
		case I2C_RECV:
			ret = incr_recv_bytes(i2c);
			break;
		case I2C_SEND_FAIL:
			ret = incr_send_fail_bytes(i2c);
			break;
		case I2C_RECV_FAIL:
			ret = incr_recv_fail_bytes(i2c);
			break;
		default:
			return -EINVAL;
	}
	
	if(!ret){
		clear_all_rec_idx(i2c);
		return -EOVERFLOW;
	}

	return 0;
}
#endif

static inline void tcc_i2c_enable_irq(struct tcc_i2c *i2c)
{
	i2c_writel(i2c_readl(i2c->regs+I2C_CTRL) | (1<<6), i2c->regs+I2C_CTRL);
//	enable_irq(i2c->irq);

//	printk("@@@ %s: core:%d, irq_sts:0x08%x @@@\n", __func__, i2c->core, i2c_readl(i2c->port_cfg));
}

static inline void tcc_i2c_disable_irq(struct tcc_i2c *i2c)
{
//	printk("@@@ %s: core:%d, irq_sts:0x08%x @@@\n", __func__, i2c->core, i2c_readl(i2c->port_cfg));

	i2c_writel(i2c_readl(i2c->regs+I2C_CTRL) & ~(1<<6), i2c->regs+I2C_CTRL);
	i2c_writel(i2c_readl(i2c->regs+I2C_CMD) | (1<<0), i2c->regs+I2C_CMD);
//	disable_irq_nosync(i2c->irq);
}

static irqreturn_t tcc_i2c_isr(int irq, void *dev_id)
{
	struct tcc_i2c *i2c = dev_id;

//	printk("@@@ %s: core:%d, irq_sts:0x08%x @@@\n", __func__, i2c->core, i2c_readl(i2c->port_cfg));

	//if (i2c_readl(i2c->port_cfg+0xc) & (1<<(i2c->core))) {
	if(i2c_readl(i2c->regs+I2C_SR) & (1<<0)) { // check status register interrupt flag
		i2c_writel(i2c_readl(i2c->regs+I2C_CMD) | (1<<0), i2c->regs+I2C_CMD);
		complete(&i2c->msg_complete);
		return IRQ_HANDLED;
	}
	return IRQ_NONE;
}

static int tcc_i2c_bus_busy(struct tcc_i2c *i2c, int start_stop)
{
	unsigned long orig_jiffies = jiffies;
	unsigned int temp;

	while(1){
		temp = i2c_readl(i2c->regs+I2C_SR);
		//printk("SR -> 0x%08x\n", temp);
		if(start_stop && (temp & (1<<6)))
			break;
		if(!start_stop && !(temp & (1<<6)))
			break;
		if (time_after(jiffies, orig_jiffies + msecs_to_jiffies(100))) {
			dev_dbg(&i2c->adap.dev,
				"<%s> I2C bus is busy\n", __func__);
			return -ETIMEDOUT;
		}		
		schedule();
	}

	return 0;
}

static int wait_intr(struct tcc_i2c *i2c)
{
	unsigned long cnt = 0;
	int ret;

	if(i2c->interrupt_mode){
		//INIT_COMPLETION(i2c->msg_complete);
		reinit_completion(&i2c->msg_complete); 
		tcc_i2c_enable_irq(i2c);
		ret = wait_for_completion_timeout(&i2c->msg_complete, msecs_to_jiffies(1000));
		tcc_i2c_disable_irq(i2c);
		if (ret == 0) {
			dev_err(i2c->dev, "i2c transfer timed out\n");
			return -ETIMEDOUT;
		}		
	}
	else{
		//while (!(i2c_readl(i2c->port_cfg + I2C_IRQ_STS) & (1<<i2c->core))) {
		while(1) {
			cnt++;
			if(i2c_readl(i2c->regs+I2C_SR) & (1<<0)) // check status register interrupt flag
				break;
			if (cnt > 100000) {
				printk("i2c-tcc: time out!  core%d ch%d\n", i2c->core, i2c->adap.nr);
				return -ETIMEDOUT;
			}
		}
		/* Clear a pending interrupt */
		i2c_writel(i2c_readl(i2c->regs+I2C_CMD)|(1<<0), i2c->regs+I2C_CMD);
	}

	return 0;
}

/* tcc_i2c_message_start
 *
 * put the start of a message onto the bus
 */
static int tcc_i2c_message_start(struct tcc_i2c *i2c, struct i2c_msg *msg)
{
	unsigned int addr = (msg->addr & 0x7f) << 1;

	if (msg->flags & I2C_M_RD)
		addr |= 1;
	if (msg->flags & I2C_M_REV_DIR_ADDR)
		addr ^= 1;

	i2c_writel(addr, i2c->regs+I2C_TXR);
	i2c_writel((1<<7)|(1<<4), i2c->regs+I2C_CMD);

	return wait_intr(i2c);
}

static int tcc_i2c_stop(struct tcc_i2c *i2c)
{
	int ret = 0;

	i2c_writel(1<<6, i2c->regs+I2C_CMD);
	ret = wait_intr(i2c);
	return ret;
}

static int tcc_i2c_acked(struct tcc_i2c *i2c)
{
	unsigned long orig_jiffies = jiffies;
	unsigned int temp;
	if((i2c->msg->flags&0x1000) ==  I2C_M_IGNORE_NAK)
		return 0;

	while(1){
		temp = i2c_readl(i2c->regs+I2C_SR);
		if(!(temp & (1<<7))) break;

		tcc_i2c_message_start(i2c, i2c->msg);
		
		if (time_after(jiffies, orig_jiffies + msecs_to_jiffies(50))) {
			dev_dbg(&i2c->adap.dev,
				"<%s> No ACK\n", __func__);
				//printk("<%s> No ACK\n", __func__);
			return -EIO;
		}		
		schedule();
	}
	dev_dbg(&i2c->adap.dev, "<%s> ACK received\n", __func__);
	return 0;
}


static int recv_i2c(struct tcc_i2c *i2c)
{
	int ret, i;
	dev_dbg(&i2c->adap.dev, "READ [%x][%d]\n", i2c->msg->addr, i2c->msg->len);

	ret = tcc_i2c_message_start(i2c, i2c->msg);
	if(ret) 
		return ret;
	
	ret = tcc_i2c_acked(i2c);
	if(ret) 
		return ret;

	for (i = 0; i < i2c->msg->len; i++) {
		 if (i2c->msg->flags & I2C_M_WM899x_CODEC) { /* B090183 */
			 if (i == 0)
				i2c_writel(1<<5, i2c->regs+I2C_CMD);
			 else
				i2c_writel((1<<5)|(1<<3), i2c->regs+I2C_CMD);
		} else {
			if (i == (i2c->msg->len - 1))
				i2c_writel((1<<5)|(1<<3), i2c->regs+I2C_CMD);
			else
				i2c_writel(1<<5, i2c->regs+I2C_CMD);
		}

		ret = wait_intr(i2c);
		if (ret)
			return ret;

		i2c->msg->buf[i] = (unsigned char)i2c_readl(i2c->regs+I2C_RXR);
	}

#if defined(CONFIG_I2C_DEBUG_BUS_SYSFS)
	incr_recv_bytes(i2c);
#endif

	return 0;
}

static int send_i2c(struct tcc_i2c *i2c)
{
	int ret, i;
	dev_dbg(&i2c->adap.dev, "SEND [%x][%d]", i2c->msg->addr, i2c->msg->len);

	ret = tcc_i2c_message_start(i2c, i2c->msg);
	if(ret)
		return ret;

	ret = tcc_i2c_acked(i2c);
	if(ret) 
		return ret;

	for (i = 0; i < i2c->msg->len; i++) {
		i2c_writel(i2c->msg->buf[i], i2c->regs+I2C_TXR);

		 if(i2c->msg->flags == I2C_M_WM899x_CODEC)	/* B090183 */
			i2c_writel((1<<4)|(1<<3), i2c->regs+I2C_CMD);
		 else
			i2c_writel(1<<4, i2c->regs+I2C_CMD);

		ret = wait_intr(i2c);
		if (ret)
			return ret;

		ret = tcc_i2c_acked(i2c);
		if(ret) 
			return ret;
	}
#if defined(CONFIG_I2C_DEBUG_BUS_SYSFS)
	incr_trans_bytes(i2c, I2C_SENT);
#endif

	return 0;
}

/* tcc_i2c_doxfer
 *
 * this starts an i2c transfer
 */
static int tcc_i2c_doxfer(struct tcc_i2c *i2c, struct i2c_msg *msgs, int num)
{
	int ret = 0;
	int i;

	for (i = 0; i < num; i++) {
		spin_lock_irq(&i2c->lock);
		i2c->msg		= &msgs[i];
		i2c->msg->flags	= msgs[i].flags;
		i2c->msg_num	= num;
		i2c->msg_ptr	= 0;
		i2c->msg_idx	= 0;
		i2c->state		= STATE_START;
		spin_unlock_irq(&i2c->lock);

		if (i2c->msg->flags & I2C_M_RD) {
			ret = recv_i2c(i2c);
			if (ret){
				printk("recv_i2c failed! - addr(0x%02X)\n", i2c->msg->addr);
#if defined(CONFIG_I2C_DEBUG_BUS_SYSFS)
				incr_trans_bytes(i2c, I2C_RECV_FAIL);
#endif
				goto fail;
			 }
		} else {
			ret = send_i2c(i2c);
			if (ret){
				printk("send_i2c failed! - addr(0x%02X)\n", i2c->msg->addr);
#if defined(CONFIG_I2C_DEBUG_BUS_SYSFS)
				incr_trans_bytes(i2c, I2C_SEND_FAIL);
#endif
				goto fail;
			}
		}
	}

	if(i2c->msg->flags & I2C_M_NO_STOP)
		goto no_stop;


fail:
	tcc_i2c_stop(i2c);
	tcc_i2c_bus_busy(i2c, 0);	
	return (ret < 0) ? ret : i;

no_stop:
	//printk("\x1b[1;33m[%s:%d] no stop\x1b[0m\n", __func__, __LINE__);
	tcc_i2c_bus_busy(i2c, 1);
	return (ret < 0) ? ret : i;
}

/* tcc_i2c_xfer
 *
 * first port of call from the i2c bus code when an message needs
 * transferring across the i2c bus.
 */
static int tcc_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg *msgs, int num)
{
	struct tcc_i2c *i2c = (struct tcc_i2c *)adap->algo_data;
	int retry;
	int ret=0;

	if (i2c->is_suspended)
		return -EBUSY;

	for (retry = 0; retry < adap->retries; retry++) {
		ret = tcc_i2c_doxfer(i2c, msgs, num);
		if (ret>0) {
			return ret;
		}
		dev_dbg(&i2c->adap.dev, "Retrying transmission (%d)\n", retry);
		udelay(100);
	}
	return ret;
}

static u32 tcc_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

/* i2c bus registration info */
static const struct i2c_algorithm tcc_i2c_algo = {
	.master_xfer	= tcc_i2c_xfer,
	.functionality	= tcc_i2c_func,
};

#if defined(CONFIG_I2C_DEBUG_BUS_SYSFS)
/**
 * show_info - attribute show func 
 * @dev : device
 * @attr: device attribute
 * @buf : buffer to copy message
 * display i2c transfer information
 */
static ssize_t show_info(struct device *dev,struct device_attribute *attr, char *buf)
{
	struct tcc_i2c *i2c =  dev_get_drvdata(dev);
	
	if(!i2c) 
		return 0;

	return sprintf(buf, "i2c core : %d\n"
			//"i2c ch : %d\n"
			//"smu i2c flag : %d\n"
			"i2c clock rate : %d\n"
			"i2c core clock rate : %d\n"
			//"i2c core clock name : %s\n"
			"sent : %d bytes / %d fail\n"
			"recv : %d bytes / %d fail\n",
			i2c->core, 
			/*i2c->ch, i2c->smu_i2c_flag, */
			i2c->i2c_clk_rate, i2c->core_clk_rate,
			/*i2c->core_clk_name, */
			i2c->sent, i2c->send_fail, i2c->recv, i2c->recv_fail);
}

/**
 * store_info - attribute store func 
 * @dev : device
 * @attr: device attribute
 * @buf : buffer from user massage
 * process user command  
 */
ssize_t store_info(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct tcc_i2c *i2c =  dev_get_drvdata(dev);
	
	if(!i2c) 
		return 0;

	if(!strncmp(buf, "init", strlen("init"))){
		clear_all_rec_idx(i2c);
	}

	return count;
}

/**
 * send_sample_byte - func. to process i2c send 
 * @dev : device
 * @data : data
 * This func. transfers 8 byte to slave devices.
 */
static int send_sample_byte(struct device *dev, void *data)
{
	int cnt = 0;
	char tmp[8] = {0x0AA};
	struct i2c_client *client = i2c_verify_client(dev);

	if(!client)
		return -EFAULT;

	cnt = sizeof(tmp);

	printk(KERN_INFO "send 8 bytes to slave (addr:0x%02X)\n", client->addr);
	if(i2c_master_send(client, tmp, cnt) != cnt) 
		return -EIO;

	return 0;
}

/**
 * show_debug - attribute show func 
 * @dev : device
 * @attr: device attribute
 * @buf : buffer to copy message
 */
static ssize_t show_debug(struct device *dev,struct device_attribute *attr, char *buf)
{
	return 0;	
}

/**
 * store_debug - attribute store func 
 * @dev : device
 * @attr: device attribute
 * @buf : buffer from user massage
 * calls i2c sending function for each i2c child devices 
 */
ssize_t store_debug(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct tcc_i2c *i2c =  dev_get_drvdata(dev);
	
	if(!i2c) 
		return 0;

	if(device_for_each_child(&i2c->adap.dev, NULL, send_sample_byte) < 0)
		return 0;

	return count;
}

static DEVICE_ATTR(debug, S_IWUSR | S_IRUGO, show_debug, store_debug);
static DEVICE_ATTR(info, S_IWUSR | S_IRUGO, show_info, store_info);
#endif

/* tcc_i2c_init
 *
 * initialize the I2C controller, set the IO lines and frequency
 */
static int tcc_i2c_init(struct tcc_i2c *i2c)
{
	unsigned int prescale;
	struct device_node *np = i2c->dev->of_node;

	if (i2c->hclk == NULL)
		i2c->hclk = of_clk_get(np, 1);
	if (i2c->pclk == NULL)
		i2c->pclk = of_clk_get(np, 0);

		if(clk_prepare_enable(i2c->hclk) != 0) {
			dev_err(i2c->dev, "can't do i2c_hclk clock enable\n");
			return -1;
		}
		if(clk_prepare_enable(i2c->pclk) != 0) {
			dev_err(i2c->dev, "can't do i2c_pclk clock enable\n");
			return -1;
		}
		clk_set_rate(i2c->pclk, i2c->core_clk_rate);

#if defined(CONFIG_TCC_CP_I2C0) // temporary code for test 
		if(i2c->core == 0)
	#if defined(CONFIG_TCC_CP_20C)
		i2c->i2c_clk_rate = 100000; // 100k
	#else
		i2c->i2c_clk_rate = 50000; // 50k
	#endif
#endif // after making device tree for CP chip, have to be removed.
		prescale = (clk_get_rate(i2c->pclk) / (i2c->i2c_clk_rate * 5)) - 1;
		i2c_writel(prescale, i2c->regs+I2C_PRES);
		i2c_writel((1<<7)|(1<<6)|0, i2c->regs+I2C_CTRL);		// start enable, stop enable, 8bit mode
		i2c_writel((1<<0)|i2c_readl(i2c->regs+I2C_CMD), i2c->regs+I2C_CMD);	// clear pending interrupt

		/* set port mux */
		if (i2c->port_mux != 0xFF) {
			int i;
			unsigned port_value;
			// Check Master port /
			port_value = i2c_readl(i2c->port_cfg+I2C_PORT_CFG0);
			for (i=0 ; i<4 ; i++) {
 				if ((i2c->port_mux&0xFF) == ((port_value>>(i*8))&0xFF))
					port_value |= (0xFF<<(i*8));
			}
			i2c_writel(port_value, i2c->port_cfg+I2C_PORT_CFG0);

			// Check Slave port /
			port_value = i2c_readl(i2c->port_cfg+I2C_PORT_CFG1);
			for (i=0 ; i<4 ; i++) {
			    if ((i2c->port_mux&0xFF) == ((port_value>>(i*8))&0xFF))
				port_value |= (0xFF<<(i*8));
			}
			i2c_writel(port_value, i2c->port_cfg+I2C_PORT_CFG1);

			// Dummy(?) Port /
			i2c_writel(0xFFFFFFFF, i2c->port_cfg+I2C_PORT_CFG0+0x8);

			if (i2c->core < 4){
			        i2c_writel((i2c_readl(i2c->port_cfg+I2C_PORT_CFG0)& ~(0xFF<<(i2c->core*8)))
					| ((i2c->port_mux&0xFF)<<(i2c->core*8)), i2c->port_cfg+I2C_PORT_CFG0);
			}
			else
				BUG();
	}

	return 0;
}

#ifdef CONFIG_OF
static void tcc_i2c_parse_dt(struct device_node *np, struct tcc_i2c *i2c)
{
	of_property_read_u32(np, "clock-frequency", &i2c->core_clk_rate);
	of_property_read_u32(np, "scl-clock-frequency", &i2c->i2c_clk_rate);
	of_property_read_u32(np, "port-mux", &i2c->port_mux);
	of_property_read_u32(np, "interrupt_mode", &i2c->interrupt_mode);
}
#else
static void tcc_i2c_parse_dt(struct device_node *np, struct tcc_i2c *i2c)
{
}
#endif

static int tcc_i2c_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct tcc_i2c *i2c;
	struct resource *res;

	if (!pdev->dev.of_node)
		return -EINVAL;

	i2c = devm_kzalloc(&pdev->dev, sizeof(struct tcc_i2c), GFP_KERNEL);
	if (!i2c)
		return -ENOMEM;

	/* get base register */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "no mem resource?\n");
		return -ENODEV;
	}
	i2c->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(i2c->regs)) {
		ret = PTR_ERR(i2c->regs);
		goto err_io;
	}

	/* get i2c port config register */
	i2c->port_cfg = of_iomap(pdev->dev.of_node, 1);

	i2c->irq = platform_get_irq(pdev, 0);
	if (i2c->irq < 0) {
		dev_err(&pdev->dev, "no irq resource?\n");
		ret = i2c->irq;
		goto err_io;
	}

	tcc_i2c_parse_dt(pdev->dev.of_node, i2c);

	init_completion(&i2c->msg_complete); // for test by kch
	
	pdev->id = of_alias_get_id(pdev->dev.of_node, "i2c");
	i2c->core = pdev->id;
	i2c->adap.owner = THIS_MODULE;
 	i2c->adap.algo = &tcc_i2c_algo;
	i2c->adap.retries = 2;
	i2c->dev = &(pdev->dev);
	sprintf(i2c->adap.name, "%s", pdev->name);
	printk("%s - inerrupt mode:%d\n", i2c->adap.name, i2c->interrupt_mode);
	spin_lock_init(&i2c->lock);
	init_waitqueue_head(&i2c->wait);

	/* set clock & port_mux */
	if (tcc_i2c_init(i2c) < 0)
		 goto err_io;

	if(i2c->interrupt_mode){
		ret = request_irq(i2c->irq, tcc_i2c_isr, IRQF_SHARED, i2c->adap.name, i2c);
		tcc_i2c_disable_irq(i2c);
	}
	
	if (ret) {
		dev_err(&pdev->dev, "Failed to request irq %i\n", i2c->irq);
		return ret;
	}

	i2c->adap.algo_data = i2c;
	i2c->adap.dev.parent = &pdev->dev;
	i2c->adap.class = I2C_CLASS_HWMON | I2C_CLASS_SPD;
	i2c->adap.nr = pdev->id;
	i2c->adap.dev.of_node = pdev->dev.of_node;

	ret = i2c_add_numbered_adapter(&(i2c->adap));
	if (ret < 0) {
		printk("%s: failed to add bus\n", i2c->adap.name);
		i2c_del_adapter(&i2c->adap);
		goto err_clk;
	}

	/* of_i2c_register_devices() is already called at i2c_add_numbered_adapter() */
	//of_i2c_register_devices(&i2c->adap);

#if defined(CONFIG_I2C_DEBUG_BUS_SYSFS)
	ret = device_create_file(&pdev->dev, &dev_attr_info);
	if (ret)
		goto err_clk;
	ret = device_create_file(&pdev->dev, &dev_attr_debug);
	if (ret)
		goto err_dev;
#endif

	platform_set_drvdata(pdev, i2c);
	return 0;

#if defined(CONFIG_I2C_DEBUG_BUS_SYSFS)
err_dev:
	device_remove_file(&pdev->dev, &dev_attr_info);
#endif

err_clk:
	if(i2c->pclk) {
		 clk_disable(i2c->pclk);
		 clk_put(i2c->pclk);
		 i2c->pclk = NULL;
	}
	if(i2c->hclk) {
		 clk_disable(i2c->hclk);
		 clk_put(i2c->hclk);
		 i2c->hclk = NULL;
	}

err_io:
	kfree(i2c);
	return ret;
}

static int tcc_i2c_remove(struct platform_device *pdev)
{
	struct tcc_i2c *i2c = platform_get_drvdata(pdev);
	platform_set_drvdata(pdev, NULL);

	/* I2C clock Disable */
	i2c_del_adapter(&(i2c->adap));
	/* I2C bus & clock disable */
	if (i2c->pclk) {
		clk_disable(i2c->pclk);
		clk_put(i2c->pclk);
		i2c->pclk = NULL;
	}
	if (i2c->hclk) {
		clk_disable(i2c->hclk);
		clk_put(i2c->hclk);
		i2c->hclk = NULL;
	}

#if defined(CONFIG_I2C_DEBUG_BUS_SYSFS)
	device_remove_file(&pdev->dev, &dev_attr_info);
	device_remove_file(&pdev->dev, &dev_attr_debug);
#endif

	kfree(i2c);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int tcc_i2c_suspend(struct device *dev)
{
	struct tcc_i2c *i2c = dev_get_drvdata(dev);
	i2c_lock_adapter(&i2c->adap);
	if(i2c->pclk)
		clk_disable_unprepare(i2c->pclk);
	if(i2c->hclk)
		clk_disable_unprepare(i2c->hclk);
	i2c->is_suspended = true;
	i2c_unlock_adapter(&i2c->adap);
	return 0;
}

static int tcc_i2c_resume(struct device *dev)
{
	struct tcc_i2c *i2c = dev_get_drvdata(dev);
	i2c_lock_adapter(&i2c->adap);
	tcc_i2c_init(i2c);
	i2c->is_suspended = false;
	i2c_unlock_adapter(&i2c->adap);
	return 0;
}
static SIMPLE_DEV_PM_OPS(tcc_i2c_pm, tcc_i2c_suspend, tcc_i2c_resume);
#define TCC_I2C_PM (&tcc_i2c_pm)
#else
#define TCC_I2C_PM NULL
#endif

#ifdef CONFIG_OF
static const struct of_device_id tcc_i2c_of_match[] = {
	{ .compatible = "telechips,i2c" },
	{},
};
MODULE_DEVICE_TABLE(of, tcc_i2c_of_match);
#endif

static struct platform_driver tcc_i2c_driver = {
	.probe			= tcc_i2c_probe,
	.remove			= tcc_i2c_remove,
	.driver			= {
		.owner		= THIS_MODULE,
		.name		= "tcc-i2c",
		.pm		= TCC_I2C_PM,
		.of_match_table	= of_match_ptr(tcc_i2c_of_match),
	},
};

static int __init tcc_i2c_adap_init(void)
{
	 return platform_driver_register(&tcc_i2c_driver);
}

static void __exit tcc_i2c_adap_exit(void)
{
	 platform_driver_unregister(&tcc_i2c_driver);
}

subsys_initcall(tcc_i2c_adap_init);
module_exit(tcc_i2c_adap_exit);

MODULE_AUTHOR("Telechips Inc. androidce@telechips.com");
MODULE_DESCRIPTION("Telechips H/W I2C driver");
MODULE_LICENSE("GPL");
