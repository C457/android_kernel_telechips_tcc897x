/*
 * scdc.c
 *
 *  Created on: Feb 11, 2014
 *  Synopsys Inc.
 *  SG DWC PT02
 */
#include "../../../include/hdmi_includes.h"
#include "../../../include/hdmi_access.h"
#include "../../../include/hdmi_log.h"

#include "../../../include/hdmi_ioctls.h"
#include "../../include/scdc/scdc.h"

#include "../../include/core/irq.h"
#include "../../include/bsp/i2cm.h"
#include "../../include/bsp/eddc_reg.h"






int scdc_read(struct hdmi_tx_dev *dev, u8 address, u8 size, u8 * data)
{
	if(hdmi_ddc_read(dev, SCDC_SLAVE_ADDRESS, 0,0 , address, size, data)){
		//LOGGER(SNPS_ERROR, "%s: SCDC addr 0x%x read failed ",__func__, address);
		return -1;
	}
	return 0;
}

int scdc_write(struct hdmi_tx_dev *dev, u8 address, u8 size, u8 * data)
{
	if(hdmi_ddc_write(dev, SCDC_SLAVE_ADDRESS, address, size, data)){
		//LOGGER(SNPS_ERROR, "%s: SCDC addr 0x%x write failed ",__func__, address);
		return -1;
	}
	return 0;
}

void scdc_enable_rr(struct hdmi_tx_dev *dev, u8 enable)
{

	if (enable == 1) {
		/* Enable Readrequest from the Tx controller */
		hdmi_dev_write_mask(dev, I2CM_SCDC_READ_UPDATE, I2CM_SCDC_READ_UPDATE_READ_REQUEST_EN_MASK, 1);
		scdc_set_rr_flag(dev, 0x01);
		hdmi_irq_scdc_read_request(dev, TRUE);
	}
	if (enable == 0) {
		/* Disable ReadRequest on Tx controller */
		hdmi_irq_scdc_read_request(dev, FALSE);
		scdc_set_rr_flag(dev, 0x00);
		hdmi_dev_write_mask(dev, I2CM_SCDC_READ_UPDATE, I2CM_SCDC_READ_UPDATE_READ_REQUEST_EN_MASK, 0);
	}
}

int scdc_scrambling_status(struct hdmi_tx_dev *dev)
{
	u8 read_value = 0;
	if(scdc_read(dev, SCDC_SCRAMBLER_STAT, 1, &read_value)){
		//LOGGER(SNPS_ERROR, "%s: SCDC addr 0x%x read failed ",__func__, SCDC_SINK_VER);
		return 0;
	}
	return (read_value & 0x01);
}

int scdc_scrambling_enable_flag(struct hdmi_tx_dev *dev, u8 enable)
{
        u8 read_value = 0;
        if(scdc_read(dev, SCDC_TMDS_CONFIG, 1 , &read_value)){
                //LOGGER(SNPS_ERROR, "%s: SCDC addr 0x%x read failed ",__func__, SCDC_TMDS_CONFIG);
                return -1;
        }
        //printk("1) scdc_scrambling_enable_flag read_value=0x%x\r\n", read_value);
        read_value = (read_value & ~0x1) | (enable ? 0x1 : 0x0);
        //printk("2) scdc_scrambling_enable_flag read_value=0x%x\r\n", read_value);
        if(scdc_write(dev, SCDC_TMDS_CONFIG, 1, &read_value)){
                //LOGGER(SNPS_ERROR, "%s: SCDC addr 0x%x write failed ",__func__, SCDC_TMDS_CONFIG);
                return -1;
        }
        return 0;
}

int scdc_tmds_config_status(struct hdmi_tx_dev *dev)
{
        u8 read_value = 0;
        if(scdc_read(dev, SCDC_TMDS_CONFIG, 1, &read_value)){
                //LOGGER(SNPS_ERROR, "%s: SCDC addr 0x%x read failed ",__func__, SCDC_SINK_VER);
                return 0;
        }
        return (read_value & 0x2);
}

int scdc_tmds_bit_clock_ratio_enable_flag(struct hdmi_tx_dev *dev, u8 enable)
{
        u8 read_value = 0;
        if(scdc_read(dev, SCDC_TMDS_CONFIG, 1 , &read_value)){
                //LOGGER(SNPS_ERROR, "%s: SCDC addr 0x%x read failed ",__func__, SCDC_TMDS_CONFIG);
                return -1;
        }
        //printk("1) scdc_tmds_bit_clock_ratio_enable_flag read_value=0x%x\r\n", read_value);
        read_value = (read_value & ~0x2) | (enable ? 0x2 : 0x0);
        //printk("2) scdc_tmds_bit_clock_ratio_enable_flag read_value=0x%x\r\n", read_value);
        if(scdc_write(dev, SCDC_TMDS_CONFIG, 1, &read_value)){
                //LOGGER(SNPS_ERROR, "%s: SCDC addr 0x%x write failed ",__func__, SCDC_TMDS_CONFIG);
                return -1;
        }
        return 0;
}


void scdc_set_rr_flag(struct hdmi_tx_dev *dev, u8 enable)
{

	if(hdmi_ddc_write(dev, SCDC_SLAVE_ADDRESS, SCDC_CONFIG_0, 1, &enable)){
		//LOGGER(SNPS_ERROR, "%s: SCDC addr 0x%x - 0x%x write failed ",__func__, SCDC_CONFIG_0, enable);
	}
}

int scdc_get_rr_flag(struct hdmi_tx_dev *dev, u8 * flag)
{
	if(hdmi_ddc_read(dev, SCDC_SLAVE_ADDRESS, 0, 0 , SCDC_CONFIG_0, 1, flag)){
		//LOGGER(SNPS_ERROR, "%s: SCDC addr 0x%x read failed ",__func__, SCDC_CONFIG_0);
		return -1;
	}
	return 0;
}

void scdc_test_rr(struct hdmi_tx_dev *dev, u8 test_rr_delay)
{
	scdc_enable_rr(dev, 0x01);
	//test_rr_delay = set(test_rr_delay, 0x80, 1);
	test_rr_delay |= 0x80;
	scdc_write(dev, SCDC_TEST_CFG_0, 1, &test_rr_delay);
}

int scdc_test_rr_update_flag(struct hdmi_tx_dev *dev)
{
	u8 read_value = 0;
	if(scdc_read(dev, SCDC_UPDATE_0, 1, &read_value)){
		//LOGGER(SNPS_ERROR, "%s: SCDC addr 0x%x read failed ",__func__, SCDC_UPDATE_0);
		return 0;
	}
	return read_value;
}

