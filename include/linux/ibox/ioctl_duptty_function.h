#ifndef __duplication_serial_API_H__
#define __duplication_serial_API_H__

struct dup_serial_config {
	int major;
	int minor;
	int nr;
	char dev_name[16];
};

#define IOC_UART_DUPLICATE	_IOW('u', 0x01, struct dup_serial_config)
#define IOC_UART_RESET		_IOW('u', 0x02, unsigned int) /* ms time */

#endif /* __duplication_serial_API_H__ */
