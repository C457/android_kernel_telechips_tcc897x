/* 
 * linux/drivers/char/tcc_dxb/ctrl/tcc_dxb_control.c
 *
 * Author:  <linux@telechips.com>
 * Created: 10th Jun, 2008 
 * Description: Telechips Linux DxB Control DRIVER
 *
 * Copyright (c) Telechips, Inc.
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

static inline int dib9090_init(struct tcc_dxb_ctrl_t *ctrl, int deviceIdx)
{
	GPIO_OUT_INIT(ctrl->gpio_dxb_on);

	GPIO_OUT_INIT(ctrl->gpio_dxb_0_pwdn);
	GPIO_OUT_INIT(ctrl->gpio_dxb_0_rst);
	GPIO_OUT_INIT(ctrl->gpio_dxb_0_sdo);

	return 0;
}

static inline int dib9090_reset(struct tcc_dxb_ctrl_t *ctrl, int deviceIdx)
{
	GPIO_SET_VALUE(ctrl->gpio_dxb_0_rst, 0);
	msleep (100);
	GPIO_SET_VALUE(ctrl->gpio_dxb_0_rst, 1);
	msleep (100);

	return 0;
}

static inline int dib9090_on(struct tcc_dxb_ctrl_t *ctrl, int deviceIdx)
{
	GPIO_SET_VALUE(ctrl->gpio_dxb_on, 1);

	GPIO_SET_VALUE(ctrl->gpio_dxb_0_rst, 0);
	GPIO_SET_VALUE(ctrl->gpio_dxb_0_pwdn, 0);
	msleep (20);
	GPIO_SET_VALUE(ctrl->gpio_dxb_0_pwdn, 1);
	msleep (20);
	GPIO_SET_VALUE(ctrl->gpio_dxb_0_rst, 0);
	msleep (20);
	GPIO_SET_VALUE(ctrl->gpio_dxb_0_rst, 1);

	/* RF Control */
	GPIO_SET_VALUE(ctrl->gpio_dxb_0_sdo, 1);

	return 0;
}

static inline int dib9090_off(struct tcc_dxb_ctrl_t *ctrl, int deviceIdx)
{
	/* RF Control */
	GPIO_SET_VALUE(ctrl->gpio_dxb_0_sdo, 0);

	switch(deviceIdx)
	{
	case 0:
		GPIO_SET_VALUE(ctrl->gpio_dxb_0_pwdn, 0);
		GPIO_SET_VALUE(ctrl->gpio_dxb_0_rst, 0);
		break;
	default:
		break;
	}

	GPIO_SET_VALUE(ctrl->gpio_dxb_on, 0);

	return 0;
}

static inline int DIB9090_IOCTL(struct tcc_dxb_ctrl_t *ctrl, unsigned int cmd, unsigned long arg)
{
	unsigned int deviceIdx;

	switch (cmd)
	{
		case IOCTL_DXB_CTRL_SET_BOARD:
			deviceIdx = (arg == 0) ? 0 : *(unsigned int *)arg;
			dib9090_init(ctrl, deviceIdx);
			break;

		case IOCTL_DXB_CTRL_OFF:
			deviceIdx = (arg == 0) ? 0 : *(unsigned int *)arg;
			dib9090_off(ctrl, deviceIdx);
			break;

		case IOCTL_DXB_CTRL_ON:
			deviceIdx = (arg == 0) ? 0 : *(unsigned int *)arg;
			dib9090_on(ctrl, deviceIdx);
			break;

		case IOCTL_DXB_CTRL_RESET:
			deviceIdx = (arg == 0) ? 0 : *(unsigned int *)arg;
			dib9090_reset(ctrl, deviceIdx);
			break;

		case IOCTL_DXB_CTRL_GET_CTLINFO:
		case IOCTL_DXB_CTRL_RF_PATH:
		case IOCTL_DXB_CTRL_RESET_LOW:
		case IOCTL_DXB_CTRL_RESET_HIGH:
		case IOCTL_DXB_CTRL_PURE_ON:
		case IOCTL_DXB_CTRL_PURE_OFF:
		default:
			break;
	}
	return 0;
}
