/*
 * phy.h
 *
 *  Synopsys Inc.
 *  SG DWC PT02 
 */
/**
 * @file
 * Physical line interface configuration
 */
#ifndef __HDMI_PHY_H__
#define __HDMI_PHY_H__

#include "phy_reg.h"

#define PHY_TIMEOUT 		100
#define PHY_I2C_SLAVE_ADDR   	0x54

#define PHY_MODEL_301		301
#define PHY_MODEL_303		303
#define PHY_MODEL_108		108

#define PHY_MODEL_TCC           400


/**
 * Initialize PHY and put into a known state
 * @param dev device structure
 * @param phy_model
 * return always TRUE
 */
int hdmi_phy_initialize(struct hdmi_tx_dev *dev, u16 phy_model);

/**
 * Enable HPD sensing circuitry
 * @param dev device structure
 */
int hdmi_phy_enable_hpd_sense(struct hdmi_tx_dev *dev);

/**
 * Disable HPD sensing circuitry
 * @param dev device structure
 */
int hdmi_phy_disable_hpd_sense(struct hdmi_tx_dev *dev);

/**
 * Detects the signal on the HPD line and 
 * upon change, it inverts polarity of the interrupt
 * bit so that interrupt raises only on change
 * @param dev device structure
 * @return TRUE the HPD line is asserted
 */
int hdmi_phy_hot_plug_detected(struct hdmi_tx_dev *dev);

/**
 * @param dev device structure
 * @param value of mask of interrupt register
 */
int hdmi_phy_interrupt_enable(struct hdmi_tx_dev *dev, u8 value);

/**
 * @param baseAddr of controller
 * @param value
 */
int hdmi_phy_test_control(struct hdmi_tx_dev *dev, u8 value);

/**
 * @param dev device structure
 * @param value
 */
int hdmi_phy_test_data(struct hdmi_tx_dev *dev, u8 value);


int hdmi_phy_phase_lock_loop_state(struct hdmi_tx_dev *dev);

void hdmi_phy_interrupt_mask(struct hdmi_tx_dev *dev, u8 mask);

void hdmi_phy_interrupt_unmask(struct hdmi_tx_dev *dev, u8 mask);

u8 hdmi_phy_hot_plug_state(struct hdmi_tx_dev *dev);

int hdmi_phy_write(struct hdmi_tx_dev *dev, u32 addr, u8 data);

uint8_t hdmi_phy_read(struct hdmi_tx_dev *dev, uint32_t offset);

#endif	/* __HDMI_PHY_H__ */
