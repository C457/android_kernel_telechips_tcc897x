/*
 * @file:halEdid.c
 *
 *  Created on: Jul 5, 2010
 *
 *  Synopsys Inc.
 *  SG DWC PT02
 */


#include "../../../include/hdmi_includes.h"
#include "../../../include/hdmi_access.h"
#include "../../../include/hdmi_log.h"

#include "../../include/core/interrupt/interrupt_reg.h"
#include "../../include/core/irq.h"

#include "../../include/bsp/eddc_reg.h"
#include "../../include/bsp/i2cm.h"
#include "../../include/phy/phy_reg.h"

#define I2CM_OPERATION_READ		0x01
#define I2CM_OPERATION_READ_EXT		0x02
#define I2CM_OPERATION_READ_SEQ		0x04
#define I2CM_OPERATION_READ_SEQ_EXT     0x08
#define I2CM_OPERATION_WRITE		0x10


#define I2C_DIV_FACTOR	 100000

/*********************  PRIVATE FUNCTIONS ***********************/

/**
 * calculate the fast sped high time counter - round up
 */
static u16 _scl_calc(u16 sfrClock, u16 sclMinTime)
{
	unsigned long tmp_scl_period = 0;
	if (((sfrClock * sclMinTime) % I2C_DIV_FACTOR) != 0) {
		tmp_scl_period = (unsigned long)((sfrClock * sclMinTime) + (I2C_DIV_FACTOR - ((sfrClock * sclMinTime) % I2C_DIV_FACTOR))) / I2C_DIV_FACTOR;
	}
	else {
		tmp_scl_period = (unsigned long)(sfrClock * sclMinTime) / I2C_DIV_FACTOR;
	}
	return (u16)(tmp_scl_period);
}

static void _fast_speed_high_clk_ctrl(struct hdmi_tx_dev *dev, u16 value)
{
	LOG_TRACE2(I2CM_FS_SCL_HCNT_1_ADDR, value);
	hdmi_dev_write(dev, I2CM_FS_SCL_HCNT_1_ADDR, (u8) (value >> 8));
	hdmi_dev_write(dev, I2CM_FS_SCL_HCNT_0_ADDR, (u8) (value >> 0));
}

static void _fast_speed_low_clk_ctrl(struct hdmi_tx_dev *dev, u16 value)
{
	LOG_TRACE2((I2CM_FS_SCL_LCNT_1_ADDR), value);
	hdmi_dev_write(dev, I2CM_FS_SCL_LCNT_1_ADDR, (u8) (value >> 8));
	hdmi_dev_write(dev, I2CM_FS_SCL_LCNT_0_ADDR, (u8) (value >> 0));
}

static void _standard_speed_high_clk_ctrl(struct hdmi_tx_dev *dev, u16 value)
{
	LOG_TRACE2((I2CM_SS_SCL_HCNT_1_ADDR), value);
	hdmi_dev_write(dev, I2CM_SS_SCL_HCNT_1_ADDR, (u8) (value >> 8));
	hdmi_dev_write(dev, I2CM_SS_SCL_HCNT_0_ADDR, (u8) (value >> 0));
}

static void _standard_speed_low_clk_ctrl(struct hdmi_tx_dev *dev, u16 value)
{
	LOG_TRACE2((I2CM_SS_SCL_LCNT_1_ADDR), value);
	hdmi_dev_write(dev, I2CM_SS_SCL_LCNT_1_ADDR, (u8) (value >> 8));
	hdmi_dev_write(dev, I2CM_SS_SCL_LCNT_0_ADDR, (u8) (value >> 0));
}

static int _write(struct hdmi_tx_dev *dev, u8 i2cAddr, u8 addr, u8 data)
{
	int timeout = I2CDDC_TIMEOUT;
	u32 status = 0;

	LOG_TRACE1(addr);

	hdmi_dev_write_mask(dev, I2CM_SLAVE, I2CM_SLAVE_SLAVEADDR_MASK, i2cAddr);
	hdmi_dev_write(dev, I2CM_ADDRESS, addr);
	hdmi_dev_write(dev, I2CM_DATAO, data);
	hdmi_dev_write(dev, I2CM_OPERATION, I2CM_OPERATION_WRITE);
	do {
		udelay(10);
		status = hdmi_dev_read_mask(dev, IH_I2CM_STAT0, IH_I2CM_STAT0_I2CMASTERERROR_MASK |
							   IH_I2CM_STAT0_I2CMASTERDONE_MASK);
	} while (status == 0 && (timeout--));

	hdmi_dev_write(dev, IH_I2CM_STAT0, status); //clear read status

	if(status & IH_I2CM_STAT0_I2CMASTERERROR_MASK){
		LOGGER(SNPS_NOTICE, "%s: I2C DDC write failed",__func__);
		return -1;
	}

	if(status & IH_I2CM_STAT0_I2CMASTERDONE_MASK){
		return 0;
	}

	//LOGGER(SNPS_ERROR, "%s: ASSERT I2C Write timeout - check system - exiting",__func__);
	return -1;
}

static int _read(struct hdmi_tx_dev *dev, u8 i2cAddr, u8 segment, u8 pointer, u8 addr,   u8 * value)
{
	int timeout = I2CDDC_TIMEOUT;
	u32 status = 0;

	LOG_TRACE1(addr);
	hdmi_dev_write_mask(dev, I2CM_SLAVE, I2CM_SLAVE_SLAVEADDR_MASK, i2cAddr);
	hdmi_dev_write(dev, I2CM_ADDRESS, addr);
	hdmi_dev_write(dev, I2CM_SEGADDR, segment);
	hdmi_dev_write(dev, I2CM_SEGPTR, pointer);

	if(pointer)
		hdmi_dev_write(dev, I2CM_OPERATION, I2CM_OPERATION_READ_EXT);
	else
		hdmi_dev_write(dev, I2CM_OPERATION, I2CM_OPERATION_READ);

	do {
		udelay(10);
		status = hdmi_dev_read_mask(dev, IH_I2CM_STAT0, IH_I2CM_STAT0_I2CMASTERERROR_MASK |
							   IH_I2CM_STAT0_I2CMASTERDONE_MASK);
	} while (status == 0 && (timeout--));

	hdmi_dev_write(dev, IH_I2CM_STAT0, status); //clear read status

	if(status & IH_I2CM_STAT0_I2CMASTERERROR_MASK){
		LOGGER(SNPS_NOTICE, "%s: I2C DDC Read failed for i2cAddr 0x%x seg 0x%x pointer 0x%x addr 0x%x",__func__,
					i2cAddr, segment, pointer, addr);
		return -1;
	}

	if(status & IH_I2CM_STAT0_I2CMASTERDONE_MASK){
		*value = (u8) hdmi_dev_read(dev, I2CM_DATAI);
		return 0;
	}

	//LOGGER(SNPS_ERROR, "%s: ASSERT I2C DDC Read timeout - check system - exiting",__func__);
	return -1;
}

static int _read8(struct hdmi_tx_dev *dev, u8 i2cAddr, u8 segment, u8 pointer, u8 addr, u8 * value)
{
	int timeout = I2CDDC_TIMEOUT;
	u32 status = 0;

	LOG_TRACE1(addr);
	hdmi_dev_write_mask(dev, I2CM_SLAVE, I2CM_SLAVE_SLAVEADDR_MASK, i2cAddr);
	hdmi_dev_write(dev, I2CM_SEGADDR, segment);
	hdmi_dev_write(dev, I2CM_SEGPTR, pointer);
	hdmi_dev_write(dev, I2CM_ADDRESS, addr);

	if(pointer)
		hdmi_dev_write(dev, I2CM_OPERATION, I2CM_OPERATION_READ_SEQ_EXT);
	else
		hdmi_dev_write(dev, I2CM_OPERATION, I2CM_OPERATION_READ_SEQ);

	do {
		udelay(10);
		status = hdmi_dev_read_mask(dev, IH_I2CM_STAT0, IH_I2CM_STAT0_I2CMASTERERROR_MASK |
							   IH_I2CM_STAT0_I2CMASTERDONE_MASK);
	} while (status == 0 && (timeout--));

	hdmi_dev_write(dev, IH_I2CM_STAT0, status); //clear read status

	if(status & IH_I2CM_STAT0_I2CMASTERERROR_MASK){
	//LOGGER(SNPS_ERROR, "%s: I2C DDC Read8 extended failed for i2cAddr 0x%x seg 0x%x pointer 0x%x addr 0x%x",__func__,
		//i2cAddr, segment, pointer, addr);
		return -1;
	}

	if(status & IH_I2CM_STAT0_I2CMASTERDONE_MASK){
		int i = 0;
		while(i < 8){ //read 8 bytes
			value[i] = (u8) hdmi_dev_read(dev, I2CM_READ_BUFF0 + (4 * i) );
			i +=1;
		}
		return 0;
	}

	//LOGGER(SNPS_ERROR, "%s: ASSERT I2C DDC Read extended timeout - check system - exiting",__func__);
	return -1;
}

/*********************  PUBLIC FUNCTIONS ***********************/

void hdmi_i2cddc_clk_config(struct hdmi_tx_dev *dev, u16 sfrClock, u16 ss_low_ckl, u16 ss_high_ckl, u16 fs_low_ckl, u16 fs_high_ckl)
{
	_standard_speed_low_clk_ctrl(dev, _scl_calc(sfrClock, ss_low_ckl));
	_standard_speed_high_clk_ctrl(dev, _scl_calc(sfrClock, ss_high_ckl));
	_fast_speed_low_clk_ctrl(dev, _scl_calc(sfrClock, fs_low_ckl));
	_fast_speed_high_clk_ctrl(dev, _scl_calc(sfrClock, fs_high_ckl));
}

void hdmi_i2cddc_fast_mode(struct hdmi_tx_dev *dev, u8 value)
{
	LOG_TRACE1(value);
	/* bit 4 selects between high and standard speed operation */
	hdmi_dev_write_mask(dev, I2CM_DIV, I2CM_DIV_FAST_STD_MODE_MASK, value);
}

int hdmi_ddc_write(struct hdmi_tx_dev *dev, u8 i2cAddr, u8 addr, u8 len, u8 * data)
{
	int i, status = 0;
        hdmi_irq_mute(dev, MUTE_ID_DDC);
	for(i = 0; i < len; i++){
		int tries = 3;
		do {
			status = _write(dev, i2cAddr, addr, data[i]);
		} while (status && tries--);

		if(status) //Error after 3 failed writes
			goto end_process;
	}
        status = 0;
end_process:
        hdmi_irq_unmute(dev, MUTE_ID_DDC);
        return status;
}


int hdmi_ddc_read(struct hdmi_tx_dev *dev, u8 i2cAddr, u8 segment, u8 pointer, u8 addr, u8 len, u8 * data)
{
        int i, status = 0;
        hdmi_irq_mute(dev, MUTE_ID_DDC);
	for(i = 0; i < len;){
		int tries = 3;
		if ((len - i) >= 8){
			do {
				status = _read8(dev, i2cAddr, segment, pointer, addr + i,  &(data[i]));
			} while (status && tries--);

			if(status) //Error after 3 failed writes
				goto end_process;
			i +=8;
		} else {
			do {
				status = _read(dev, i2cAddr, segment, pointer, addr + i,  &(data[i]));
			} while (status && tries--);

			if(status) //Error after 3 failed writes
				goto end_process;
			i++;
		}
	}
        status = 0;
end_process:
        hdmi_irq_unmute(dev, MUTE_ID_DDC);
	return status;
}

