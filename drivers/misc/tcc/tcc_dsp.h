#ifndef _TCC_DSP_H_
#define _TCC_DSP_H_

#include <asm/ioctl.h>
#include "AM3DZirene.h"

#define CMD_TYPE    0x00010000
#define INFO_TYPE   0x00020000

#define ALSA_OUT_BUF_PARAM 1
#define ALSA_IN_BUF_PARAM 2
#define ALSA_HW_PARAM 3
 
#define ALSA_PLAYBACK_STOP 100
#define ALSA_PLAYBACK_START 101
 
#define DSP_PCM_FORMAT_S16_LE   1
#define DSP_PCM_FORMAT_S24_LE   2

#define TCC_AM3D_MAGIC	'A'
#define IOCTL_TCC_AM3D_SET_PARAM				_IO(TCC_AM3D_MAGIC ,0)
#define IOCTL_TCC_AM3D_GET_PARAM				_IO(TCC_AM3D_MAGIC ,1)
#define IOCTL_TCC_AM3D_SET_PARAM_TABLE			_IO(TCC_AM3D_MAGIC ,2)
#define IOCTL_TCC_AM3D_GET_PARAM_TABLE			_IO(TCC_AM3D_MAGIC ,3)
#define IOCTL_TCC_AM3D_CREATE_AND_ADD_FILTER	_IO(TCC_AM3D_MAGIC ,4)
#define IOCTL_TCC_AM3D_SELECT_FILTER_BY_INDEX	_IO(TCC_AM3D_MAGIC ,5)

#define IOCTL_TCC_CONTROL_SET_PARAM				_IO(TCC_AM3D_MAGIC ,10)
#define IOCTL_TCC_CONTROL_GET_PARAM				_IO(TCC_AM3D_MAGIC ,11)

struct tcc_control_param_t {
    int ctl_idx;
    int ctl_value;
} __attribute__((packed));

struct tcc_am3d_param_t {
	eZirene_Effect eEffect;
	AM3D_INT32     eParm;
	AM3D_INT32     eChannelMask;
	AM3D_INT32     iValue;
	eZireneStatus  retStatus;
} __attribute__((packed));

#define PARAM_TBL_MAX	(10)
struct tcc_am3d_param_tbl_t {
	AM3D_INT32 iNumOfParameters;
	struct tcc_am3d_param_t tbl[PARAM_TBL_MAX];
} __attribute__((packed));


#define FIR_MAX_CNT		(80)
#define IIR_MAX_CNT		(80)
struct eq_filter_tbl_t {
	tZireneEqFilter filter;
	AM3D_INT16 fir[FIR_MAX_CNT];
	AM3D_INT16 iir[IIR_MAX_CNT];
};

#define EQ_FILTER_TBL_MAX   (6)
struct tcc_am3d_eq_filter_t {
	eZireneTransducerEqSupportedChannelCombinations iChannelMask;
	AM3D_INT32 iSectionIndex;
	AM3D_INT32 iNumOfTables;
	eZireneStatus  retStatus;
	struct eq_filter_tbl_t tbl[EQ_FILTER_TBL_MAX];
} __attribute__((packed));

struct tcc_am3d_select_filter_by_index_t {
	eZireneTransducerEqSupportedChannelCombinations iChannelMask;
	AM3D_INT32 iSectionIndex;
	eZireneStatus  retStatus;
} __attribute__((packed));

#endif
