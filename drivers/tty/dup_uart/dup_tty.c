/* Virtual duplicated TTY ldisc driver for UART */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/init.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include "ioctl_duptty_function.h"

#include "dup_serial.h"

#define DT_dev_info(_tty, args...) pr_info(args)
#define DT_dev_err(_tty, args...) pr_err(args)

/* refer driver/serial/mxc_uart.c */
#define DUPTTY_PARENT_MAJOR		(207)

static const struct dt_dev dt_template = {
	.magic = DUPTTY_MAGIC,
};

static inline void set_ldisc_priv(struct tty_struct *tty, struct dt_dev *dt)
{
	tty->disc_data = dt;
}

static inline struct dt_dev *get_ldisc_priv(struct tty_struct *tty)
{
	return tty->disc_data;
}

static inline struct dt_dev *to_dt_dev(struct tty_struct *tty)
{
	struct dt_dev *dt = get_ldisc_priv(tty);
	if (unlikely(!dt || dt->magic != DUPTTY_MAGIC)) {
		pr_err("%s(): failed\n", __func__);
		return NULL;
	}
	return dt;
}

static inline struct dt_dev *from_du_dev(struct du_dev *du_dev)
{
	struct dt_dev *dt = container_of(du_dev, struct dt_dev, du_dev);
	if (unlikely(!dt || dt->magic != DUPTTY_MAGIC)) {
		pr_err("%s(): failed\n", __func__);
		return NULL;
	}
	return dt;
}

static inline int __is_dt_supported_parent(struct tty_struct *tty)
{
	/* refer driver/serial/mxc_uart.c */
	if (!tty || !tty->driver
			/*|| tty->driver->major != DUPTTY_PARENT_MAJOR*/)
		return -1;
	return tty->index;
}

void dt_ldops_put_char(struct du_port *du_port, const unsigned char *ch, int count)
{
	unsigned long iflags;
	struct dt_dev *dt = from_du_dev(du_port->du_dev);
	struct tty_struct *tty = dt->tty;
	int space;
	bool need_mutex_unlock = false;

	if (unlikely(!in_atomic())) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 18, 24)	
		mutex_lock(&dt->output_lock);
#else
		mutex_lock(&tty->output_lock);
#endif		
		need_mutex_unlock = true;
	}
	space = tty_write_room(tty);

	if (space >= count) {
		if (tty->ops->write)
			tty->ops->write(tty, ch, count);
		if (unlikely(need_mutex_unlock))
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 18, 24)	
			mutex_unlock(&dt->output_lock);
#else
			mutex_unlock(&tty->output_lock);
#endif		
	} else {
		if (unlikely(need_mutex_unlock))
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 18, 24)	
			mutex_unlock(&dt->output_lock);
#else
			mutex_unlock(&tty->output_lock);
#endif		
		spin_lock_irqsave(&du_port->lock, iflags);
		__set_bit(TTY_DO_WRITE_WAKEUP, &tty->flags);
		spin_unlock_irqrestore(&du_port->lock, iflags);
	}
}

static int dt_ldops_open(struct tty_struct *tty)
{
	struct dt_dev *dt = NULL, *dt_tmp;
	int p_minor = __is_dt_supported_parent(tty);
	int ret = 0;

	if (p_minor < 0) {
		DT_dev_err(tty, "%s is not one of my parents.\n", tty->name);
		return -EINVAL;
	}
	DT_dev_info(tty, "%s:%d is one of my parents.\n", tty->name, p_minor);

	dt = kmemdup(&dt_template, sizeof(*dt), GFP_KERNEL);
	if (unlikely(!dt)) {
		DT_dev_err(tty, "%s: not enough memory.\n", __func__);
		return -ENOMEM;
	}
	dt->tty = tty;
	dt->add_device = 0;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 18, 24)	
	mutex_init(&dt->output_lock);
#endif

	dt_tmp = get_ldisc_priv(tty);
	if (dt_tmp && dt_tmp->magic == DUPTTY_MAGIC) {
		ret = -EEXIST;
		DT_dev_err(tty, "%s: already duplicated by %s.\n",
			__func__, dupuart_get_devname(&dt_tmp->du_dev));
		goto err_out;
	}
	set_ldisc_priv(tty, dt);

	return 0;

 err_out:
	kfree(dt);
	return ret;
}

static void dt_ldops_close(struct tty_struct *tty)
{
	struct dt_dev *dt = to_dt_dev(tty);

	if (unlikely(!dt)) {
		DT_dev_err(tty, "%s is not mine.\n", tty->name);
		return;
	}

	dupuart_del_device(&dt->du_dev);
	kfree(dt);
	set_ldisc_priv(tty, NULL);
}

static void dt_ldops_flush_buffer(struct tty_struct *tty)
{
	/* ToDo: call child's flush_buffer() */
	struct dt_dev *dt = to_dt_dev(tty);
	dt->tty->ops->flush_buffer(tty);
}

static ssize_t dt_ldops_chars_in_buffer(struct tty_struct *tty)
{
	/* ToDo: call primary child's chars_in_buffer() */
	struct dt_dev *dt = to_dt_dev(tty);
	dt->tty->ops->chars_in_buffer(tty);
	return -1;
}

static ssize_t dt_ldops_read(struct tty_struct *tty, struct file *file,
			unsigned char __user *buf, size_t nr)
{
	return -EPERM;
}

static ssize_t dt_ldops_write(struct tty_struct *tty, struct file *file,
			 const unsigned char *buf, size_t nr)
{
	return nr;
}

static int dt_ldops_ioctl(struct tty_struct *tty, struct file *file,
			 unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int ret = -ENOIOCTLCMD;
	struct dt_dev *dt = to_dt_dev(tty);

	if (!dt)
		goto err_out;

	switch (cmd) {
	case IOC_UART_DUPLICATE: {
		struct dup_serial_config ds_config;
		const char *dev_name;
		if (copy_from_user(&ds_config, argp, sizeof(ds_config)))
			return -EFAULT;
		dev_name = kstrndup(ds_config.dev_name, 16, GFP_KERNEL);
		ret = dupuart_add_device(&dt->du_dev, dev_name,
				ds_config.major, ds_config.minor, ds_config.nr);
		if (ret)
			kfree(dev_name);
		else
			dt->add_device = 1;
	} break;
#if 0
	case IOC_UART_RESET: {
		/* ToDo:
			take negate time.
			stop Tx/Rx parent.
			stop Tx/Rx all childs.
			flush buffer for parent and all childs.
			if possible, do H/W reset; ie.GPIO.
			start Tx/Rx all childs.
			start Tx/Rx parent.
			done.
		*/
	} break;
#endif
	}
 err_out:
	return ret;
}

static long dt_ldops_compat_ioctl(struct tty_struct *tty,
			struct file *file, unsigned int cmd, unsigned long arg)
{
	/* maybe needless */
	return -ENODEV;
}

static void dt_ldops_set_termios(struct tty_struct *tty,
			struct ktermios *old)
{
	/* maybe needless */
}

static unsigned int dt_ldops_poll(struct tty_struct *tty, struct file *file,
		     struct poll_table_struct *poll)
{
	/* maybe needless */
	return 0;
}

static int dt_ldops_hangup(struct tty_struct *tty)
{
	/* maybe needless */
	return -ENODEV;
}

static void dt_ldops_receive_buf(struct tty_struct *tty,
			const unsigned char *cp, char *fp, int count)
{
	struct dt_dev *dt = to_dt_dev(tty);
	int i, err_cnt = 0;

	if (!dt->add_device)
		return;

	if (unlikely(!dt || (count <= 0)))
		return;

	for (i = 0; i < count; i++)
		if (fp && *fp++)
			err_cnt++;
	if (unlikely(err_cnt))
		pr_debug("%s: %s reports %d frame/parity error\n", __func__, tty->name, err_cnt);

	dupuart_receive_buf(&dt->du_dev, cp, count);
}

static void dt_ldops_write_wakeup(struct tty_struct *tty)
{
	/* maybe needless ? */
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 18, 24)
static void dt_ldops_dcd_change(struct tty_struct *tty, unsigned int i)
{
	/* maybe needless */
}
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 35)
static void dt_ldops_dcd_change(struct tty_struct *tty, unsigned int i,
				struct timespec *ts)
{
	/* maybe needless */
}
#endif

static struct tty_ldisc_ops dup_ldisc = {
	.magic		= TTY_LDISC_MAGIC,
	.name		= "dup",
	.flags		= 0,
	.open		= dt_ldops_open,
	.close		= dt_ldops_close,
	.flush_buffer		= dt_ldops_flush_buffer,
	.chars_in_buffer	= dt_ldops_chars_in_buffer,
	.read		= dt_ldops_read,
	.write		= dt_ldops_write,
	.ioctl		= dt_ldops_ioctl,
	.compat_ioctl	= dt_ldops_compat_ioctl,
	.set_termios	= dt_ldops_set_termios,
	.poll		= dt_ldops_poll,
	.hangup		= dt_ldops_hangup,
	.receive_buf	= dt_ldops_receive_buf,
	.write_wakeup	= dt_ldops_write_wakeup,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 35)
	.dcd_change	= dt_ldops_dcd_change,
#endif
	.owner		= THIS_MODULE,
};

/* ------------------------------------------------------- */

static int __init duptty_init(void)
{
	int err;

	if ((err = tty_register_ldisc(N_DUP, &dup_ldisc)) != 0)
		pr_err("DUPtty: can't register line discipline (%d)\n", err);
	else
		pr_info("DUPtty: registered\n");

	/* ToDo:
		initialize and parse param.s of self-duplication.
	 */
	return err;
}

static void __exit duptty_cleanup(void)
{
	int err;

	/* ToDo:
		de-initialize and cleanup self-duplication.
	 */

	if ((err = tty_unregister_ldisc(N_DUP))) {
		pr_err("DUPtty: can't unregister line discipline (err = %d)\n", err);
	}
	pr_info("DUPtty: unregistered\n");
}

module_init(duptty_init);
module_exit(duptty_cleanup);

MODULE_ALIAS_LDISC(N_DUP);
MODULE_DESCRIPTION("TTY duplicate device driver");
MODULE_VERSION("0.1");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("SeonKon Choi <seonkon.choi@windriver.com>");

