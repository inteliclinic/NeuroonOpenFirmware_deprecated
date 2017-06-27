#ifndef IC_LIB_DEFAULT_CONFIG_H_
#define IC_LIB_DEFAULT_CONFIG_H_

#include <stdint.h>
#include <stdbool.h>

#include "nrf.h"
#include "nrf_soc.h"
#include "nrf_sdm.h"
#include "app_util.h"
#include "app_error.h"
#include "nrf51.h"
#include "nrf51_bitfields.h"
#include "app_util_platform.h"

/* INCLUDED LIBRARIES (uncomment libraries which you going to use) */
#define USE_IC_ADS
#define USE_IC_AFE4400
//#define USE_IC_ARRAYS
//#define USE_IC_BATTERY_TESTER
#define USE_IC_BQ27742
#define USE_IC_DELAY
#define USE_IC_ELECTROTEST
//#define USE_IC_FILTERS
//#define USE_IC_FIXFFT
#define USE_IC_FLASH
#define USE_IC_GPIO
#define USE_IC_I2C
//#define USE_IC_IIR_FILTER
#define USE_IC_LIS3DH
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
#define USE_IC_SPI
#define USE_IC_UART
//#define USE_IC_UTILS
#define USE_IC_WDT
#define USE_IC_FLASH_MAP
#define USE_IC_ANALOG_THERMOMETER
#define USE_IC_DATA_PROCESSING
#define USE_IC_TEST
#define USE_IC_MOVEMENT


#define USE_SOFTDEVICE
//#define USE_DIGITAL_THERMOMETER
//#define USE_ANALOG_THERMOMETER
//#define LEDS_LOWPASS_FILTER

						/** PERIPHERALS AND SENSORS */
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
#define UART_BAUDRATE	 				UART_BAUDRATE_BAUDRATE_Baud460800

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
#define PWR_SWITCH_DELAY_MS				1000

/** USB CHARGER */
#define GPIO_USB_CONNECTED				11

/** LIS3DH */
#define GPIO_INT1_ACC_PIN				17
#define GPIOTE_INT1_ACC_CHANNEL			0					/// interrupt channel for ACC
#define ACC_VECTOR_TRESHOLD				1000*1000

/** ANALOG THERMOMETER (PT1000) */
#define GPIO_ANALOG_T_RIGHT_PIN			4
#define GPIO_ANALOG_T_LEFT_PIN			5

/** 2MS DRIVER */
#define GPIO_2MS_DRIVER_ON				24

/** LTC */
#define GPIO_LEDS_ON					25

/** LTC CHANNEL */

/// RIGHT EYE
#define RGB1_R_CHANNEL			2
#define RGB1_G_CHANNEL			1
#define RGB1_B_CHANNEL			3
/// LEFT EYE
#define RGB2_R_CHANNEL			17
#define RGB2_G_CHANNEL			16
#define RGB2_B_CHANNEL			18

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

						/** SCHEDULER */
#define SCHEDULER_TIMER_PRESCALER             1023	/// hardware timer prescaler, Frtc = 32.768KHz/(PRESCALER+1) = 16Hz
#define RTC_MIN_TIMEOUT_TICKS					5

/** SCHEDULER - EVENTS TIME */
#define SLEEP_STAGE_PERIOD_SECONDS			30		/// time between sleep stage events
#define ELECTROTEST_TIME					1 		/// time to start task inside sleep stage task
#define BIOSIGNAL_MEASUREMENT_TIME			2 		/// time to start task inside sleep stage task
#define TEMP_MEASUREMENT_TIME				12		/// time to start task inside sleep stage task
#define PULSE_MEASUREMENT_TIME				13 		/// time to start task inside sleep stage task


/** ADS */
#define BUS_I2C_SPI_DRIVER_TIM				NRF_TIMER1
#define ADS_ET_GEN_DRIVER_TIM_IRQ			TIMER1_IRQn
#define ADS_ET_GEN_DRIVER_ADS_TIM_CC		16000
#define ADS_ET_GEN_DRIVER_ADS_TIM_PRESC		3
#define ADS_ET_GEN_DRIVER_ADS_TIM_CH		0
#define ADS_SAMPLES_NUMBER					1250

/** AFE */
#define AFE_DRIVER_TIM						NRF_TIMER1
#define AFE_DRIVER_TIM_IRQ					TIMER1_IRQn
#define AFE_DRIVER_TIM_CC					10000
#define AFE_DRIVER_TIM_PRESC				6
#define AFE_DRIVER_TIM_CH					1

/** PWM LEDS */
#define LED_DRIVER_TIM						NRF_TIMER2
#define LED_DRIVER_TIM_IRQn					TIMER2_IRQn
#define LED_DRIVER_TIM_PRESC				9
#define LED_DRIVER_TIM_CC_CH				3
#define LED_DRIVER_TIM_CC					3125

/** ELECTROTEST */
#define TIM_ELECTROTEST						NRF_TIMER1
#define TIM_ELECTROTEST_PRESCALER			8
#define TIM_ELECTROTEST_GEN_CC_VALUE		30
#define TIM_ELECTROTEST_ADC_CC_VALUE		15

#define TIM_ELECTROTEST_GEN_CHANNEL			0
#define TIM_ELECTROTEST_ADC_CHANNEL			1

#define ELECTROTEST_E1_ADC_CHANNEL			4
#define ELECTROTEST_E2_ADC_CHANNEL			2

#define PPI_ELECTROTEST_ADC_CHANNEL1		1
#define PPI_ELECTROTEST_ADC_CHANNEL2		2

#define GPIOTE_ELECTROTEST_GEN_CHANNEL		0
#define PPI_ELECTROTEST_GEN_CHANNEL			3

#define ELECTROTEST_SAMPLE_COUNT			1000

#define ELECTROTEST_TRESHOLD_E1				200
#define ELECTROTEST_TRESHOLD_E2				200

/** BATTERY CONTROL */
#define BATTERY_VOLTAGE_MAX					4100
#define BATTERY_VOLTAGE_MIN					3600
#define BATTERY_LOW_STATE_LED_THS			25
#define BATTERY_HIGH_STATE_LED_THS			88
#define BATTERY_FULL_CHARGE					100
#define BATTERY_MEASUREMENT_PERIOD			10

/** DEBUG LOGS DEFINITIONS */
#ifdef DEBUG_LOGS_BLE
	#define DEBUG_LOGS_MAIN
	#define DEBUG_LOGS_SCHEDULER
	#define DEBUG_LOGS_INIT_TIMEOUT
	#define DEBUG_LOGS_SLEEP_STAGE
	#define DEBUG_LOGS_ELECTROTEST
	#define DEBUG_LOGS_BIOSIGNAL_MEASUREMENT
	#define DEBUG_LOGS_TEMP_MEASUREMENT
	#define DEBUG_LOGS_PULSE_MEASUREMENT
	#define DEBUG_LOGS_LD
	#define DEBUG_LOGS_BLT
	#define DEBUG_LOGS_JETLAG
	#define DEBUG_LOGS_FIRST_ALARM
	#define DEBUG_LOGS_SECOND_ALARM
	#define DEBUG_LOGS_WAKEUP_NOTIFICATION
	#define DEBUG_LOGS_REPORT_NOTIFICATION
	#define DEBUG_LOGS_CHARGING
#endif

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
#ifdef USE_IC_ANALOG_THERMOMETER
	#include "ic_analog_thermometer.h"
#endif
#ifdef USE_IC_TEST
	#include "ic_test.h"
#endif
#ifdef USE_IC_MOVEMENT
	#include "ic_movement.h"
#endif

#endif /* IC_CONFIG_H_ */
