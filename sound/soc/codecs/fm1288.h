/*
 * fm1288.h  --  FM1288 Soc Audio driver
 *
 */

#ifndef _FM1288_H
#define _FM128_H

/* FM1288 register space */

#define FM1288_LINVOL   0x00
#define FM1288_RINVOL   0x01
#define FM1288_LOUT1V   0x02
#define FM1288_ROUT1V   0x03
#define FM1288_APANA    0x04
#define FM1288_APDIGI   0x05
#define FM1288_PWR      0x06
#define FM1288_IFACE    0x07
#define FM1288_SRATE    0x08
#define FM1288_ACTIVE   0x09
#define FM1288_RESET	0x0f

#define FM1288_CACHEREGNUM 	10

#define FM1288_SYSCLK_XTAL 1
#define FM1288_SYSCLK_MCLK 2

#define FM1288_DAI		0

#endif
