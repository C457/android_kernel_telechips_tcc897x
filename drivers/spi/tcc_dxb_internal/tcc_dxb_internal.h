/*
 *	tcc_dxb_internal.h
 *
 *	This program is free software; you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation; either version 2 of the License, or
 *	(at your option) any later version.
 *
 *	This program is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *
 *	GNU General Public License for more details.
 *
 *	You should have received a copy of the GNU General Public License
 *	along with this program; if not, write to the Free Software
 *	Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.=
 */

#include <linux/in.h>

#define INTERNAL_INTPUT_THREAD
#ifdef INTERNAL_INTPUT_THREAD
//#define TCC_DXB_INTERNAL_FILE_OUT
#endif
#define TCC_DXB_INTERNAL_WRITE
#define TCC_DXB_INTERNAL_PRE_ALLOC

enum {
	TCC_DXB_INTERNAL_START = 0,
	TCC_DXB_INTERNAL_END,
	TCC_DXB_INTERNAL_WHATISTHIS, //HELPME
	TCC_DXB_INTERNAL_GET_DMX_ID,
	TCC_DXB_INTERNAL_CMD_MAX
};

//for RTP parse
typedef struct tcc_rtp_t{
	int 	  Len;
	unsigned char Version;
	unsigned char Padding;
	unsigned char Extension;
	unsigned char CSRC_Cnt;
	unsigned char Marker;
	unsigned char PayloadType;
	unsigned short SequenceNo;
	unsigned int TimeStamp;
	unsigned int SSRC;
	unsigned int CSRC[15];
	unsigned char ExtensionHeader[512];
	unsigned int PayloadLen;
	unsigned char *Payload;
	unsigned int prev_SequenceNo;
} TCC_RTP_t;

typedef struct tcc_dxb_internal_t{
	struct task_struct *thread;
	struct socket *sock;
	struct sockaddr_in addr;
	struct ip_mreq	req;
	struct tcc_rtp_t gRTPStatus;
	unsigned dmx_id;
	unsigned char *sock_recieve_buf;
	unsigned char *dma_buf_vir_addr;
	unsigned int dma_buf_phy_addr;
	unsigned int write_buf_count;
	unsigned int write_ptr_pos;
} TCC_DXB_INTERNAL_t;

typedef struct tcc_dxb_internal_drvdata_t{
	unsigned int state;
	unsigned char *sock_buf;
	unsigned char *dma_vir_buf;
	unsigned int dma_phy_buf;
} TCC_DXB_INTERNAL_DRVDATA_t;

