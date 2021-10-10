/*
 *  tcc_dxb_kernel.h
 *
 *  Written by C2-G1-3T
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.=
 */

#ifndef	_TCC_DXB_DRV_H_
#define	_TCC_DXB_DRV_H_

typedef struct tcc_dxb_inst_t
{
	struct device *dev;

	struct dvb_adapter     adapter;
	struct tcc_fe_inst_t   fe;
	struct tcc_tsif_inst_t tsif;
	struct tcc_dmx_inst_t  dmx;
} tcc_dxb_inst_t;

#endif//_TCC_DXB_DRV_H_
