#ifndef __IC_ADS__H
#define __IC_ADS__H

/**
 * @file
 * @brief ADS1115 driver header
 */
#include "ic_lib_config.h"


/** SLAVE ADDRESS */
#define ADS_TWI_ADDRESS 			0x48

/** REGISTER ADDRESS */
#define ADS_CONVERTION_REG 			0x00
#define ADS_CONFIG_REG			 	0x01
#define ADS_LO_THS_REG				0x02
#define ADS_HI_THS_REG				0x03

/** REGISTER VALUE */

/** CONFIGURATION REGISTER */
#define ADS_SINGLE_SHOT_CONV		0x80

#define ADS_MUX_1					0x00		/// AIN(P) = AIN0 and AIN(N) = AIN1
#define ADS_MUX_2					0x40		/// AIN(P) = AIN0 and AIN(N) = GND

#define ADS_GAIN_1  				0x02
#define ADS_GAIN_2					0x04
#define ADS_GAIN_4					0x06
#define ADS_GAIN_8  				0x08
#define ADS_GAIN_16 				0x0A

#define ADS_POWER_DOWN				0x01

#define ADS_DATA_RATE_8				0x00
#define ADS_DATA_RATE_16			0x20
#define ADS_DATA_RATE_32			0x40
#define ADS_DATA_RATE_64			0x60
#define ADS_DATA_RATE_128			0x80
#define ADS_DATA_RATE_250			0xA0
#define ADS_DATA_RATE_475			0xC0
#define ADS_DATA_RATE_860			0xE0

#define ADS_COMP_MODE				0x10		/// Window comparator
#define ADS_COMP_POL				0x08		/// Active high
#define ADS_COMP_LAT				0x04		/// Latching comparator

#define ADS_COMP_QUE_1				0x00
#define	ADS_COMP_QUE_2				0x01
#define	ADS_COMP_QUE_4				0x02
#define	ADS_COMP_QUE_DIS			0x03

#define ADS_LO_THS					0x8000
#define ADS_HI_THS					0x7FFF

/** DEVICE DEPENDENT FUNCTIONS */
bool ads_init(void);
void ads_deinit(void);

void ads_power_down (void);
void ads_power_up (void);

int16_t ads_get_value();

bool ads_change_gain(uint8_t new_gain);
bool ads_change_data_rate(uint8_t rate_code);

#endif
