#ifndef _TCC_AM3D_H_
#define _TCC_AM3D_H_

#include "tcc_dsp_ipc.h"

#define AM3D_MSG_FLAG_OFFSET	(24)
#define AM3D_MSG_FLAG_MASK		(0xff << AM3D_MSG_FLAG_OFFSET)
#define AM3D_MSG_SINGLE_FLAG	(1 << AM3D_MSG_FLAG_OFFSET)
#define AM3D_MSG_BULK_FLAG		(2 << AM3D_MSG_FLAG_OFFSET)
#define AM3D_MSG_ACK_FLAG		(4 << AM3D_MSG_FLAG_OFFSET)

#define AM3D_MSG_TYPE_OFFSET	(0)
#define AM3D_MSG_TYPE_MASK		(0xffff << AM3D_MSG_TYPE_OFFSET)

enum { // AM3D MSG TYPE
	TCC_DSP_AM3D_SET_PARAM = 0,
	TCC_DSP_AM3D_GET_PARAM,
	TCC_DSP_AM3D_SET_PARAM_TABLE,
	TCC_DSP_AM3D_GET_PARAM_TABLE,
	TCC_DSP_AM3D_CREATE_AND_ADD_FILTER,
	TCC_DSP_AM3D_SELECT_FILETER_BY_INDEX,
	TCC_DSP_CONTROL_SET_PARAM = 10,
	TCC_DSP_CONTROL_GET_PARAM,
};

//////////////////// Control Parameter ///////////////////////////

enum{//Control Parmeter Index
	TCC_DSP_CTL_IDX_EARLY_FM_ONOFF = 0,
	TCC_DSP_CTL_IDX_SET_DEBUG_INPUT,
	TCC_DSP_CTL_IDX_END
};


enum{//Control Parmeter Set Debug Input
	TCC_DSP_CTL_IDX_SET_DEBUG_INPUT_NONE = 0,
	TCC_DSP_CTL_IDX_SET_DEBUG_INPUT_ENHANCED_AUDIO,
	TCC_DSP_CTL_IDX_SET_DEBUG_INPUT_AUDIO0, 
	TCC_DSP_CTL_IDX_SET_DEBUG_INPUT_END
};

////////////////////////////////////////////////////////////////

#define SINGLE_MSG_MAX	(PARAM_MAX - 1)
struct tcc_am3d_msg_t {
	unsigned int seq_no;
	union {
		struct {
			unsigned int phys_addr; // bulk
			unsigned int size; 
		} bulk;
		struct {
			unsigned int value[SINGLE_MSG_MAX];
		} single;
	} u;
} __attribute__((packed));

struct tcc_am3d_t {
	struct device *dev;
	unsigned int seq_no;
	struct completion wait;
	struct mutex lock;
	unsigned int *cur_single_msgbuf;
};

int tcc_am3d_init(struct tcc_am3d_t *p, struct device *dev);
int tcc_am3d_set_and_get_message(struct tcc_am3d_t *p, unsigned int type, bool is_bulk, void *buf, unsigned int size);
int tcc_am3d_set_param(struct tcc_am3d_t *p, struct tcc_am3d_param_t *param);
int tcc_am3d_get_param(struct tcc_am3d_t *p, struct tcc_am3d_param_t *param);
int tcc_am3d_set_param_tbl(struct tcc_am3d_t *p, struct tcc_am3d_param_tbl_t *param_tbl); 
int tcc_am3d_get_param_tbl(struct tcc_am3d_t *p, struct tcc_am3d_param_tbl_t *param_tbl);
int tcc_am3d_create_and_add_filter(struct tcc_am3d_t *p, struct tcc_am3d_eq_filter_t *filter);
int tcc_am3d_select_filter_by_index(struct tcc_am3d_t *p, struct tcc_am3d_select_filter_by_index_t *select);

int tcc_control_set_param(struct tcc_am3d_t *p, struct tcc_control_param_t *param);
int tcc_control_get_param(struct tcc_am3d_t *p, struct tcc_control_param_t *param);
#endif /*_TCC_AM3D_H_*/
