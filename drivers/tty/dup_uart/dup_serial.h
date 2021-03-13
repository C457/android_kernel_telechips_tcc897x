#ifndef __DUP_TTY_H__
#define __DUP_TTY_H__

#include <linux/spinlock.h>
#include <linux/list.h>
#include <linux/tty.h>
#include <linux/serial_core.h>
#include <linux/serial.h>

struct du_dev;

#define DUPTTY_MAGIC	0x12340001
#define DUPUART_MAGIC	0x12340002
#define DUPPORT_MAGIC	0x12340003

#define portlist_init(_du_dev) \
		do { mutex_init(&(_du_dev)->port_mutex); \
		     INIT_LIST_HEAD(&(_du_dev)->port_head); } while (0)
#define portlist_lock(_du_dev) \
		mutex_lock(&(_du_dev)->port_mutex)
#define portlist_unlock(_du_dev) \
		mutex_unlock(&(_du_dev)->port_mutex)
#define portlist_add(_du_port, _du_dev) \
		list_add_tail(&(_du_port)->port_node, &(_du_dev)->port_head)
#define portlist_del(_du_port) \
		list_del(&(_du_port)->port_node)

struct du_port {
	unsigned int magic;
	unsigned long op_flag;
	struct du_dev *du_dev;
	struct list_head port_node;
	spinlock_t lock;
	struct uart_port up;
	int index;
	unsigned int mctrl;
};

struct du_dev {
	unsigned int magic;
	struct list_head port_head;
	struct mutex port_mutex;
	struct uart_driver *du_drv;
};

struct dt_dev {
	unsigned int magic;
	struct tty_struct *tty;
	struct du_dev du_dev;
	int add_device;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 18, 24)	
	struct mutex output_lock;
#endif	
};

/* dup_uart.c */
extern const char *dupuart_get_devname(struct du_dev *);
extern int dupuart_add_device(struct du_dev *, const char *dev_name,
		int major, int minor, int nr);
extern void dupuart_del_device(struct du_dev *);
extern int dupuart_receive_buf(struct du_dev *, const unsigned char *, int);

/* dup_tty.c */
extern void dt_ldops_put_char(struct du_port *, const unsigned char *, int);

#endif /* __DUP_TTY_H__ */

