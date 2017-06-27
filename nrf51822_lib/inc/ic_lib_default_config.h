#define IC_LIB_DEFAULT_CONFIG_H_
#ifndef IC_LIB_DEFAULT_CONFIG_H_
#define IC_LIB_DEFAULT_CONFIG_H_

/**
 * @file
 * @brief general default configuration library
 * @attention MAKE THE CHANGES IN THE LIBRARY ON THE NEEDS OF THE PROJECT
 */

#include <stdint.h>
#include <stdbool.h>

#include "nrf.h"
#include "nrf_soc.h"
#include "app_util.h"
#include "app_error.h"
#include "nrf51.h"
#include "nrf51_bitfields.h"
#include "app_util_platform.h"

/* INCLUDED LIBRARIES (uncomment libraries which you are going to use) */
//#define USE_IC_ADS
//#define USE_IC_AFE4400
//#define USE_IC_ARRAYS
//#define USE_IC_BATTERY_TESTER
//#define USE_IC_BQ27742
//#define USE_IC_DELAY
//#define USE_IC_ELECTROTEST
//#define USE_IC_FILTERS
//#define USE_IC_FIXFFT
//#define USE_IC_FLASH
//#define USE_IC_GPIO
//#define USE_IC_I2C
//#define USE_IC_IIR_FILTER
//#define USE_IC_LIS3DH
//#define USE_IC_LOGGER_BLE
//#define USE_IC_LOGGER_UART
//#define USE_IC_LSM9DS0
//#define USE_IC_LTC2631
//#define USE_IC_LTC3220
//#define USE_IC_MCP9804
//#define USE_IC_MLX90615
//#define USE_IC_PRINTF
//#define USE_IC_PWM
//#define USE_IC_READBACK_PROTECTION
//#define USE_IC_SCHEDULER
//#define USE_IC_SPI
//#define USE_IC_UART
//#define USE_IC_UTILS
//#define USE_IC_WDT
//#define USE_IC_FLASH_MAP
//#define USE_IC_ANALOG_THERMOMETER
//#define USE_IC_DATA_PROCESSING

						/** NEUROON */
/** TWI BUS */
#define TWI_I2C							NRF_TWI1
#define GPIO_SDA_PIN					28
#define GPIO_SCL_PIN					29
#define GPIO_SDA_T_PIN					4
#define GPIO_SCL_T_PIN					5
#define PPI_I2C_CHANNEL 				0

/** SPI BUS */
#define SPI_MASTER						NRF_SPI0
#define SPI_SPEED						SPI_FREQUENCY_FREQUENCY_M4
#define GPIO_SCLK_PIN					6
#define GPIO_MISO_PIN					22
#define GPIO_MOSI_PIN					23

/** UART BUS IN NEUROON BOARD*/
#define UART_RXD						19
#define UART_TXD						20

/** AFE */
#define GPIO_AFE_ADC_RDY				0
#define GPIO_AFE_CS						2
#define GPIO_AFE_PDN					10
#define GPIO_AFE_RESET					30

/** AFE DIAG */
#define GPIO_AFE_PD_ALM					7
#define GPIO_AFE_LED_ALM				8
#define GPIO_AFE_DIAG_END				9
#define GPIO_AFE_CLKOUT					21

/** EXTERNAL FLASH */
#define GPIO_FLASH_CS					13
#define FLASH_OPERATION_TIMEOUT			10000

/** POWER SUPPLY */
#define GPIO_POWER_ANALOG				15
#define GPIO_POWER_DIGITAL				16

/** ELECTRODE TEST */
#define GPIO_TEST_EL2					1
#define GPIO_TEST_EL1					3
#define GPIO_GEN						14

/** SWITCH ON/OFF */
#define GPIO_POWER_ON_OFF 				12

/** USB CHARGER */
#define GPIO_USB_CONNECTED				11

/** LIS3DH */
#define GPIO_INT1_ACC_PIN				17
#define GPIOTE_INT1_ACC_CHANNEL			0					/// interrupt channel for ACC

/** ANALOG THERMOMETER (PT1000) */
#define GPIO_ANALOG_T_RIGHT_PIN			4
#define GPIO_ANALOG_T_LEFT_PIN			5
#define ANALOG_T_RIGHT					1
#define ANALOG_T_LEFT					2

/** 2MS DRIVER */
#define GPIO_2MS_DRIVER_ON				24

/** LTC */
#define GPIO_LEDS_ON					25

/** LTC CHANNEL */
/// RIGHT EYE
#define RGB1_R_CHANNEL			3
#define RGB1_G_CHANNEL			2
#define RGB1_B_CHANNEL			1
/// LEFT EYE
#define RGB2_R_CHANNEL			18
#define RGB2_G_CHANNEL			17
#define RGB2_B_CHANNEL			16

/// RIGHT EYE - upper
#define POWER_LED1_CHANNEL1		4
#define POWER_LED1_CHANNEL2		5
/// RIGHT EYE - lower
#define POWER_LED2_CHANNEL1		6
#define POWER_LED2_CHANNEL2		7
/// LEFT EYE - lower
#define POWER_LED3_CHANNEL1		12
#define POWER_LED3_CHANNEL2		13
/// LEFT EYE - upper
#define POWER_LED4_CHANNEL1		14
#define POWER_LED4_CHANNEL2		15

#define VIBRATOR1_CHANNEL1 		8
#define VIBRATOR1_CHANNEL2 		9

#define VIBRATOR2_CHANNEL1 		10
#define VIBRATOR2_CHANNEL2 		11

#define RGB_COLOR_R				0
#define RGB_COLOR_G				1
#define RGB_COLOR_B				2

						/** SCHEDULER */
#define SCHEDULER_TIMER_PRESCALER             1023	/// hardware timer prescaler, Frtc = 32.768KHz/(PRESCALER+1) = 16Hz
#define RTC_MIN_TIMEOUT_TICKS					5

						/** DONGLE */

#if DONGL_BOARD_TYPE == 0
	//nrf board
	#define UART_RTS 8
	#define UART_TXD 9
	#define UART_CTS 10
	#define UART_RXD 11

	/** LED PINS */
	#define LED1_PIN					18//3
	#define LED2_PIN					19//4
	#define LED3_PIN					19//3

	#define UART_HWF_CONTROL_ENABLE

	#define UART_BAUDRATE				UART_BAUDRATE_BAUDRATE_Baud38400

#elif DONGL_BOARD_TYPE == 1
	//dongl v1.0
	#define UART_TXD 1; //in ver 2.x - 9 in ver 1.x - 1
	#define UART_RXD 3; //in ver 2.x in ver 1.x - 3
	#define UART_CTS 2;
	#define UART_RTS 0;

	#define LED1_PIN					5//3
	#define LED2_PIN					5//4
	#define LED3_PIN					5//3

	#define UART_HWF_CONTROL_ENABLE

	#define UART_BAUDRATE				UART_BAUDRATE_BAUDRATE_Baud38400
#elif DONGL_BOARD_TYPE == 2
	//dongl v2.0
	#define UART_TXD 9; //in ver 2.x - 9 in ver 1.x - 1
	#define UART_RXD 11; //in ver 2.x in ver 1.x - 3
	#define UART_CTS 10;
	#define UART_RTS 8;

	#define LED1_PIN					21//3
	#define LED2_PIN					22//4
	#define LED3_PIN					23//3

	#define UART_HWF_CONTROL_ENABLE

	#define UART_BAUDRATE				UART_BAUDRATE_BAUDRATE_Baud38400

#elif DONGL_BOARD_TYPE == 3
	//nrf board
	#define UART_TXD	7
	#define UART_RXD	6

	#define LED1_PIN	3
	#define LED2_PIN	4
	#define LED3_PIN	5

	#define UART_BAUDRATE				UART_BAUDRATE_BAUDRATE_Baud115200
#endif

						/** SPIDER */
/** SPI BUS */
#define GPIO_CS_ACC_PIN 				17
#define GPIO_MOSI_ACC_PIN				18
#define GPIO_MISO_ACC_PIN				19
#define GPIO_CLK_ACC_PIN				20

/** GPIO */
#define GPIO_INT2_ACC_PIN				0

/** GPIOTE */
#define GPIOTE_INT1_ACC_IN				GPIOTE_INTENSET_IN0_Pos
#define GPIOTE_INT1_ACC_CHANNEL			0
#define GPIOTE_INT2_ACC_CHANNEL			1

/** LSM9DS0 */
#define GPIO_CS_A_PIN 					10		/// accelerometer SPI CS pin
#define GPIO_CS_G_PIN 					5		/// gyro SPI CS pin

/* INCLUSION (do not modify!) */

#ifdef USE_IC_ADS
	#include "ic_ads.h"
#endif
#ifdef USE_IC_AFE4400
	#include "ic_afe4400.h"
#endif
#ifdef USE_IC_ARRAYS
	#include "ic_arrays.h"
#endif
#ifdef USE_IC_BATTERY_TESTER
	#include "ic_battery_tester.h"
#endif
#ifdef USE_IC_BQ27742
	#include "ic_bq27742.h"
#endif
#ifdef USE_IC_DELAY
	#include "ic_delay.h"
#endif
#ifdef USE_IC_ELECTROTEST
	#include "ic_electrotest.h"
#endif
#ifdef USE_IC_FILTERS
	#include "ic_filters.h"
#endif
#ifdef USE_IC_FIXFFT
	#include "ic_fixfft.h"
#endif
#ifdef USE_IC_FLASH
	#include "ic_flash.h"
#endif
#ifdef USE_IC_GPIO
	#include "ic_gpio.h"
#endif
#ifdef USE_IC_I2C
	#include "ic_i2c.h"
#endif
#ifdef USE_IC_IIR_FILTER
	#include "ic_irr_filter.h"
#endif
#ifdef USE_IC_LIS3DH
	#include "ic_lis3dh.h"
#endif
#ifdef USE_IC_LOGGER_BLE
	#include "ic_logger_ble.h"
#endif
#ifdef USE_IC_LOGGER_UART
	#include "ic_logger_uart.h"
#endif
#ifdef USE_IC_LSM9DS0
	#include "ic_lsm9ds0.h"
#endif
#ifdef USE_IC_LTC2631
	#include "ic_ltc2631.h"
#endif
#ifdef USE_IC_LTC3220
	#include "ic_ltc3220.h"
#endif
#ifdef USE_IC_MCP9804
	#include "ic_mcp9804.h"
#endif
#ifdef USE_IC_MLX90615
	#include "ic_mlx90615.h"
#endif
#ifdef USE_IC_PRINTF
	#include "ic_printf.h"
#endif
#ifdef USE_IC_PWM
	#include "ic_pwm.h"
#endif
#ifdef USE_IC_READBACK_PROTECTION
	#include "ic_readback_protection.h"
#endif
#ifdef USE_IC_SCHEDULER
	#include "ic_scheduler.h"
#endif
#ifdef USE_IC_SPI
	#include "ic_spi.h"
#endif
#ifdef USE_IC_UART
	#include "ic_uart.h"
#endif
#ifdef USE_IC_UTILS
	#include "ic_utils.h"
#endif
#ifdef USE_IC_WDT
	#include "ic_wdt.h"
#endif
#ifdef USE_IC_FLASH_MAP
	#include "ic_flash_map.h"
#endif
#ifdef USE_ANALOG_THERMOMETER
	#include "ic_analog_thermometer.h"
#endif
#ifdef USE_IC_DATA_PROCESSING
	#include "ic_data_processing.h"
#endif

#endif /* IC_CONFIG_H_ */
