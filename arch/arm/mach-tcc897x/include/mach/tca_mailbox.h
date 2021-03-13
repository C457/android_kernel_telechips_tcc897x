/* 
 * arch/arm/mach-tcc893x/tca_hwdemux_tsif.h
 *
 * Author:  <linux@telechips.com>
 * Created: 10th Jun, 2013 
 * Description: LINUX H/W DEMUX FUNCTIONS
 *
 *   Copyright (c) Telechips, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#ifndef __TCC_MAILBOX_H__
#define __TCC_MAILBOX_H__

typedef void (*tca_mbox_handler_t)(unsigned int *);

extern int  tca_mailbox_init(tca_mbox_handler_t handler);
extern void tca_mailbox_deinit(void);
extern int  tca_mailbox_send(unsigned int *pMsg, int iWaitTime);

#endif /*__TCC_TSIF_MODULE_HWSET_H__*/
