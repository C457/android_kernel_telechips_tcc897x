/*
 * api.h
 *
 *  Created on: Jul 7, 2010
 * 
 * Synopsys Inc.
 * SG DWC PT02
 */
/** 
 * @file
 * This is the upper layer of the HDMI TX API.
 * It is the entry point and interface of the software package
 * It crosses information from the sink and information passed on from
 * the user to check if it violates the HDMI protocol or not.
 * It coordinates between the various modules of the API to program the 
 * controller successfully in the correct steps and in a compliant 
 * configuration.
 * Errors are written to the error buffer. use error_Get in util/error.h to read
 * last error.
 */

#ifndef API_H_
#define API_H_

/** @addtogroup init_api_grp Initialization API Routines
 *
 * These routines handle initialization of the CIL and PCD driver components
 * and the DWC_usb3 controller.
 */
/** @{ */

/**
 * Configure API.
 * Configure the modules of the API according to the parameters given by
 * the user. If EDID at sink is read, it does parameter checking using the 
 * Check methods against the sink's E-EDID. Violations are outputted to the 
 * buffer.
 * Shall only be called after an Init call or configure.
 * @param video parameters pointer
 * @param audio parameters pointer
 * @param product parameters pointer
 * @param hdcp parameters pointer
 * @return TRUE when successful
 * @note during this function, all controller's interrupts are disabled
 * @note this function needs to have the HW initialized before the first call
 */
int hdmi_api_Configure(struct hdmi_tx_dev *dev, videoParams_t * video, audioParams_t * audio,
	      productParams_t * product, hdcpParams_t * hdcp);


/**
 * AV Mute in the General Control Packet
 * @param enable TRUE set the AVMute in the general control packet
 */
void hdmi_api_avmute(struct hdmi_tx_dev *dev, int enable);

/** @} */

#endif	/* API_H_ */

