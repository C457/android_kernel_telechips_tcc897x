/*
 *  linux/arch/arm/mach-tcc897x/include/mach/lk_debug_logbuf.h
 *
 *  Copyright (C) 2019 kibum.lee@mobis.co.kr
 *  Original Copyright (C) 1995  Linus Torvalds
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __LK_DEBUG_LOGBUF_H__
#define __LK_DEBUG_LOGBUF_H__

#ifdef CONFIG_LK_DEBUG_LOG_BUF
#define UART_LOG_BUF_SIZE (31*1024) /* align on 31k */
#define PHYS_DDR_BASE					0x80000000
#define ABOOT_FORCE_UART_ADDR_OFFSET			0x3600000
#define ABOOT_FORCE_UART_SIZE				0x8000     /* 32k */

#define LK_LOG_COOKIE    0x474f4c52 /* "RLOG" in ASCII */
struct lk_log {
	struct lk_log_header {
		unsigned cookie;
		unsigned max_size;
		unsigned size_written;
		unsigned idx;
	} header;
	char data[UART_LOG_BUF_SIZE];
};

int get_lk_memblock_reserved(void);
void set_lk_memblock_reserved(int value);
int lk_log_print_show(void);

#else // CONFIG_LK_DEBUG_LOG_BUF

static inline int get_lk_memblock_reserved(void)
{
	return 0;
}
static inline void set_lk_memblock_reserved(int value);
{
	return;
}
static inline int lk_log_print_show(void)
{
	return 0;
}

#endif // CONFIG_LK_DEBUG_LOG_BUF

#endif //__LK_DEBUG_LOGBUF_H__

