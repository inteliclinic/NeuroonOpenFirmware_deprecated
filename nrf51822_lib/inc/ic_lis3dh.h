#ifndef IC_ACC_DRIVER_H_
#define IC_ACC_DRIVER_H_

/**
 * @file
 * @brief LIS3DH accelerometer driver header
 */

#include "ic_lib_config.h"

/** SLAVE ADDRESS */
#define TWI_ACC_ADDRESS					0x18

/** REGISTER ADDRESS */
#define LIS3DH_STATUS_AUX				0x07
#define LIS3DH_OUT_ADC1_L				0x08
#define LIS3DH_OUT_ADC1_H				0x09
#define LIS3DH_OUT_ADC2_L				0x0A
#define LIS3DH_OUT_ADC2_H				0x0B
#define LIS3DH_OUT_ADC3_L				0x0C
#define LIS3DH_OUT_ADC3_H				0x0D
#define LIS3DH_INT_COUNTER				0x0E
#define LIS3DH_WHO_AM_I				    0x0F
#define LIS3DH_TEMP_CFG_REG				0x1F

#define LIS3DH_CTRL_REG1				0x20
#define LIS3DH_CTRL_REG2				0x21
#define LIS3DH_CTRL_REG3				0x22
#define LIS3DH_CTRL_REG4				0x23
#define LIS3DH_CTRL_REG5				0x24
#define LIS3DH_CTRL_REG6				0x25

#define LIS3DH_REFERENCE_REG		    0x26
#define LIS3DH_STATUS_REG				0x27

#define LIS3DH_X_LSB					0x28
#define LIS3DH_X_MSB					0x29
#define LIS3DH_Y_LSB					0x2A
#define LIS3DH_Y_MSB					0x2B
#define LIS3DH_Z_LSB					0x2C
#define LIS3DH_Z_MSB					0x2D

#define LIS3DH_FIFO_CTRL_REG            0x2E
#define LIS3DH_FIFO_SRC_REG			    0x2F //red only
#define LIS3DH_INT1_CFG					0x30
#define LIS3DH_INT1_SRC					0x31 //read only
#define LIS3DH_INT1_THS                 0x32
#define LIS3DH_INT1_DURATION            0x33

#define LIS3DH_CLICK_CFG				0x38
#define LIS3DH_CLICK_SRC                0x39 //read only
#define LIS3DH_CLICK_THS                0x3A
#define LIS3DH_TIME_LIMIT               0x3B
#define LIS3DH_TIME_LATENCY             0x3C
#define LIS3DH_TIME_WINDOW				0X3D

/** REGISTER VALUE */

/** CLIK-CLICK MODE PARAMETERS */
//#define CLICK_CFG						0x15
//#define CLICK_THS						0x08		/// CLICK-CLICK threshold
//#define CLICK_TIME_LIMIT				0x10		/// CLICK-CLICK Time Limit
//#define CLICK_TIME_WINDOW				0x30		/// CLICK-CLICK time window
#define CLICK_CFG						0x20
#define CLICK_THS						0x5f		/// CLICK-CLICK threshold
#define CLICK_TIME_LIMIT				0x7f		/// CLICK-CLICK Time Limit
#define CLICK_TIME_LATENCY				0x7f		/// CLICK-CLICK Time letancy
#define CLICK_TIME_WINDOW				0x7f		/// CLICK-CLICK time window

/** MOVEMENT THRESHOLD MODE PARAMETERS */
#define INT1_CFG						0x95		/// AND cobination of int event; Z,Y.X low event detect
#define INT1_THS						0x01
#define INT1_DURATION					0x03

/**
 * @struct lis3dh_config
 * @brief configuration structure
 */
typedef struct
{
	uint8_t reg1;
	uint8_t reg2;
	uint8_t reg3;
	uint8_t reg4;
	uint8_t reg5;

	uint8_t click_cfg;
	uint8_t click_src;
	uint8_t click_threshold;
	uint8_t click_time_limit;
	uint8_t click_time_latency;
	uint8_t	click_time_window;

	uint8_t int1_cfg;
	uint8_t int1_threshold;
	uint8_t int1_duration;

	uint8_t fifo_ctrl;

} lis3dh_config, *lis3dh_config_p;

typedef struct
{
	uint8_t ACC_DATA_REG[6];
	int16_t ACC_X, ACC_Y, ACC_Z;
} measurement;

/** DEVICE DEPENDENT FUNCTIONS */
/** CONTROL */
lis3dh_config* LIS3DH_Set_Mode(uint8_t mode);
bool LIS3DH_Config(lis3dh_config *data);
void LIS3DH_INT1_Config(void);
void LIS3DH_INT1_Deinit(void);

void LIS3DH_Start(void);
void LIS3DH_Stop(void);
void LIS3DH_Fifo_Clear(void);

/** CONFIG GETTERS */
lis3dh_config_p app_lis3dh_config(void);

void LIS3DH_getTestData(measurement *acc_axis);

/* SELF TEST */
int16_t LIS3DH_Self_Test(uint8_t *buffer);
int32_t LIS3DH_amplitude(int length);

#endif /* IC_ACC_DRIVER_H_ */





