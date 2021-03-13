#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 18, 24)
#include <linux/tty_flip.h>
#endif

#include "dup_serial.h"

#define DUPUART_MAX_CHILD (4)

#define DP_OP_FLAG_STOP_TX 1
#define DP_OP_FLAG_STOP_RX 2

static struct du_port *to_du_port(struct uart_port *port)
{
	struct du_port *dp = container_of(port, struct du_port, up);
	if (unlikely(dp->magic != DUPPORT_MAGIC)) {
		pr_err("%s(): failed\n", __func__);
		return NULL;
	}
	return dp;
}

static void dupuart_port_rx(struct du_port *du_port,
		const unsigned char *cp, int count)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 18, 24)
	struct tty_port *tty = &du_port->up.state->port;
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 35)
	/* 2.6.35.3 */
	struct tty_struct *tty = du_port->up.state->port.tty;
#else
	/* 2.6.28 */
	struct tty_struct *tty = du_port->up.info->port.tty;
#endif
	int i;

	for (i = 0; i < count; i++)
		tty_insert_flip_char(tty, *cp++, TTY_NORMAL);

	tty_flip_buffer_push(tty);
}

static void dupuart_port_tx(struct du_port *du_port,
		const unsigned char *cp, int count)
{
	dt_ldops_put_char(du_port, cp, count);
}

/*----------------------------------------------------------*/

static unsigned int du_pops_tx_empty(struct uart_port *port)
{
	return TIOCSER_TEMT;
}

static void du_pops_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	struct du_port *dp;

	if (unlikely(!(dp = to_du_port(port)))) {
		pr_err("%s(): failed\n", __func__);
		return;
	}
	dp->mctrl = mctrl;
}

static unsigned int du_pops_get_mctrl(struct uart_port *port)
{
	struct du_port *dp;
	unsigned int mctrl;

	if (unlikely(!(dp = to_du_port(port)))) {
		pr_err("%s(): failed\n", __func__);
		return 0;
	}
	mctrl = dp->mctrl;

	return mctrl;
}

static void du_pops_stop_tx(struct uart_port *port)
{
	struct du_port *dp;
	unsigned long iflags;

	if (unlikely(!(dp = to_du_port(port)))) {
		pr_err("%s(): failed\n", __func__);
		return;
	}

	spin_lock_irqsave(&dp->lock, iflags);

	__set_bit(DP_OP_FLAG_STOP_TX, &dp->op_flag);

	spin_unlock_irqrestore(&dp->lock, iflags);
}

static void du_pops_start_tx(struct uart_port *port)
{
	struct du_port *dp;
	unsigned long iflags;
	struct tty_struct *tty;

	if (unlikely(!(dp = to_du_port(port)))) {
		pr_err("%s(): failed\n", __func__);
		return;
	}

	spin_lock_irqsave(&dp->lock, iflags);

	__clear_bit(DP_OP_FLAG_STOP_TX, &dp->op_flag);

	spin_unlock_irqrestore(&dp->lock, iflags);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 35)
	/* 2.6.35.3 */
	tty = port->state->port.tty;
#else
	/* 2.6.28 */
	tty = port->info->port.tty;
#endif

#if 1
	{
	struct circ_buf *xmit;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 35)
	xmit = &port->state->xmit;
#else
	xmit = &port->info->xmit;
#endif
	if (port->x_char) {
		unsigned char c = port->x_char;
		dupuart_port_tx(dp, &c, 1);
		port->icount.tx++;
		port->x_char = 0;
		goto __out;
	}

	if (uart_circ_empty(xmit) || uart_tx_stopped(port)) {
		du_pops_stop_tx(port);
		return;
	}

	do {
		unsigned char c = xmit->buf[xmit->tail];
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		dupuart_port_tx(dp, &c, 1);
		port->icount.tx++;
	} while (!uart_circ_empty(xmit));
	}
#endif

 __out:
	return;
}

static void du_pops_stop_rx(struct uart_port *port)
{
	struct du_port *dp;
	unsigned long iflags;

	if (unlikely(!(dp = to_du_port(port)))) {
		pr_err("%s(): failed\n", __func__);
		return;
	}

	spin_lock_irqsave(&dp->lock, iflags);

	__set_bit(DP_OP_FLAG_STOP_RX, &dp->op_flag);

	spin_unlock_irqrestore(&dp->lock, iflags);

}

static void du_pops_enable_ms(struct uart_port *port)
{
	/* needless, do nothing for DTR */
}

static void du_pops_break_ctl(struct uart_port *port, int break_state)
{
	/* needless, do nothing for IBRK */
}

static int du_pops_startup(struct uart_port *port)
{
	struct du_port *dp;
	unsigned long iflags;

	if (unlikely(!(dp = to_du_port(port)))) {
		pr_err("%s(): failed\n", __func__);
		return -ENODEV;
	}

	spin_lock_irqsave(&dp->lock, iflags);
	__clear_bit(DP_OP_FLAG_STOP_TX, &dp->op_flag);
	__clear_bit(DP_OP_FLAG_STOP_RX, &dp->op_flag);
	spin_unlock_irqrestore(&dp->lock, iflags);

	return 0;
}

static void du_pops_shutdown(struct uart_port *port)
{
	struct du_port *dp;
	unsigned long iflags;

	if (unlikely(!(dp = to_du_port(port)))) {
		pr_err("%s(): failed\n", __func__);
		return;
	}

	spin_lock_irqsave(&dp->lock, iflags);
	__set_bit(DP_OP_FLAG_STOP_TX, &dp->op_flag);
	__set_bit(DP_OP_FLAG_STOP_RX, &dp->op_flag);
	spin_unlock_irqrestore(&dp->lock, iflags);

}

static void du_pops_set_termios(struct uart_port *port,
		struct ktermios *termios, struct ktermios *old)
{
	/* do nothing */
}

static void du_pops_pm(struct uart_port *port, unsigned int state,
		unsigned int oldstate)
{
	/* do nothing */
}

static const char *du_pops_type(struct uart_port *port)
{
	/* ToDo, maybe needness */
	return "DUPuart";
}

static void du_pops_release_port(struct uart_port *port)
{
	/* do nothing */
}

static int du_pops_request_port(struct uart_port *port)
{
	/* do nothing */
	return 0;
}

static void du_pops_config_port(struct uart_port *port, int uflags)
{
	if (uflags & UART_CONFIG_TYPE)
		port->type = PORT_DUPUART;
}

static struct uart_ops dupuart_port_ops = {
	.tx_empty	= du_pops_tx_empty,
	.set_mctrl	= du_pops_set_mctrl,
	.get_mctrl	= du_pops_get_mctrl,
	.stop_tx	= du_pops_stop_tx,
	.start_tx	= du_pops_start_tx,
	.stop_rx	= du_pops_stop_rx,
	.enable_ms	= du_pops_enable_ms,
	.break_ctl	= du_pops_break_ctl,
	.startup	= du_pops_startup,
	.shutdown	= du_pops_shutdown,
	.set_termios	= du_pops_set_termios,
	.pm		= du_pops_pm,
	.type		= du_pops_type,
	.release_port	= du_pops_release_port,
	.request_port	= du_pops_request_port,
	.config_port	= du_pops_config_port,
};

static const struct uart_driver du_drv_template = {
	.owner		= THIS_MODULE,
	.driver_name	= "dupuart",
	.dev_name	= "ttyUNKNOWN",
	.major		= -1,
	.minor		= -1,
	.nr		= -1,
};

static const struct du_port du_port_template = { /* FIXME */
	.magic	= DUPPORT_MAGIC,
		.up	= {
			.membase		= (void *)0xdeadbeef,
			.mapbase		= 0xdeadbeef,
			.iotype			= SERIAL_IO_MEM,
			.irq			= 9999,
			.uartclk		= 115200 * 16,
			.fifosize		= 16,
			.ops			= &dupuart_port_ops,
			.flags			= ASYNC_BOOT_AUTOCONF,
			.line			= 0,
		},
};

static inline struct du_port *alloc_du_port(void)
{
	struct du_port *du_port;

	du_port = kmemdup(&du_port_template, sizeof(*du_port), GFP_KERNEL);
	if (unlikely(!du_port))
		return NULL;

	spin_lock_init(&du_port->lock);
	set_bit(DP_OP_FLAG_STOP_TX, &du_port->op_flag);
	set_bit(DP_OP_FLAG_STOP_RX, &du_port->op_flag);
	INIT_LIST_HEAD(&du_port->port_node);

	return du_port;
}

static inline void init_du_dev(struct du_dev *du_dev)
{
	memset(du_dev, 0, sizeof(*du_dev));
	du_dev->magic = DUPUART_MAGIC;
	portlist_init(du_dev);

}

static int du_add_one_port_unlock(struct du_dev *du_dev, int index)
{
	struct du_port *du_port;

	du_port = alloc_du_port();
	if (!du_port)
		return -ENOMEM;

	du_port->index = index;
	du_port->up.line = index;
	du_port->du_dev = du_dev;

	uart_add_one_port(du_dev->du_drv, &du_port->up);

	portlist_add(du_port, du_dev);

	return 0;
}

static int du_del_one_port_unlock(struct du_dev *du_dev,
		struct du_port *du_port)
{
	struct uart_driver *du_drv;

	portlist_del(du_port);

	du_drv = du_dev->du_drv;
	uart_remove_one_port(du_drv, &du_port->up);

	kfree(du_port);

	return 0;
}

static inline int dupuart_validate_dev(struct du_dev *du_dev)
{
	if (!du_dev || du_dev->magic != DUPUART_MAGIC)
		return -EINVAL;
	return 0;
}

/* ------------------------------------------------------- */

const char *dupuart_get_devname(struct du_dev *du_dev)
{
	struct uart_driver *du_drv;
	if (unlikely(dupuart_validate_dev(du_dev)))
		return "(unknown)";
	du_drv = du_dev->du_drv;
	return du_drv->dev_name;
}

int dupuart_receive_buf(struct du_dev *du_dev,
			const unsigned char *cp, int count)
{
	struct du_port *du_port;
	struct list_head *head;

	if (unlikely(!du_dev)) {
		pr_err("%s: !du_dev\n", __func__);
		return -EINVAL;
	}
	head = &du_dev->port_head;

	portlist_lock(du_dev);
	list_for_each_entry(du_port, head, port_node) {
		if (test_bit(DP_OP_FLAG_STOP_RX, &du_port->op_flag))
			continue;
		dupuart_port_rx(du_port, cp, count);
	}
	portlist_unlock(du_dev);
	return 0;
}

int dupuart_add_device(struct du_dev *du_dev, const char *dev_name,
		int major, int minor, int nr)
{
	int ret, i;
	struct uart_driver *du_drv;

	if (unlikely(nr > DUPUART_MAX_CHILD))
		return -EINVAL;

	if (unlikely(dupuart_validate_dev(du_dev) == 0))
		return -EBUSY;

	du_drv = kmemdup(&du_drv_template, sizeof(*du_drv), GFP_KERNEL);
	if (unlikely(!du_drv))
		return -ENOMEM;

	init_du_dev(du_dev);

	du_drv->dev_name = kstrndup(dev_name, 16, GFP_KERNEL); /* FIXME */
	du_drv->major = major;
	du_drv->minor = minor;
	du_drv->nr = nr;

	du_dev->du_drv = du_drv;

	ret = uart_register_driver(du_drv);
	if (unlikely(ret)) {
		pr_err("%s(): could not register new uart_driver (%d)\n",
				__func__, ret);
		goto err_out;
	}

	portlist_lock(du_dev);
	for (i = 0; i < nr; i++) {
		ret = du_add_one_port_unlock(du_dev, i);
		if (ret)
			break;
	}
	if (ret) {
		struct du_port *du_port, *tmp;
		struct list_head *head = &du_dev->port_head;
		list_for_each_entry_safe(du_port, tmp, head, port_node)
		du_del_one_port_unlock(du_dev, du_port);
	}
	portlist_unlock(du_dev);
	if (ret)
		goto err_out_unregister;

	return 0;

 err_out_unregister:
	uart_unregister_driver(du_drv);
 err_out:
	kfree(du_drv->dev_name);
	kfree(du_drv);
	du_dev->magic = 0xdead; /* mark as deleted */
	return ret;
}

void dupuart_del_device(struct du_dev *du_dev)
{
	struct uart_driver *du_drv;
	struct du_port *du_port, *tmp;
	struct list_head *head;

	if (unlikely(dupuart_validate_dev(du_dev) != 0)) {
		pr_err("%s: something wrong #2\n", __func__);
		return;
	}

	if (!du_dev->du_drv) {
		pr_err("%s: something wrong #3\n", __func__);
		du_dev->magic = 0xdead; /* mark as deleted */
		return;
	}
	du_drv = du_dev->du_drv;

	portlist_lock(du_dev);
	head = &du_dev->port_head;
	list_for_each_entry_safe(du_port, tmp, head, port_node)
	du_del_one_port_unlock(du_dev, du_port);
	portlist_unlock(du_dev);

	uart_unregister_driver(du_drv);
	kfree(du_drv->dev_name);
	kfree(du_drv);
}

