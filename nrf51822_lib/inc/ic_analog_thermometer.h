#ifndef IC_ANALOG_THERMOMETER_H_
#define IC_ANALOG_THERMOMETER_H_

/**
 * @file
 * @brief analog thermometer (PT1000) driver header
 */

#include "ic_lib_config.h"

#define ANALOG_T_TIMEOUT				1000
#define ANALOG_T_ERROR					0xFFFF

/** THERMOMETER INIT */
void analog_thermometer_init(void);
void analog_thermometer_deinit(void);
void analog_thermometer_select (uint8_t therm_number);

/** THERMOMETER CONTROL */
void analog_thermometer_start(void);
void analog_thermometer_stop(void);
void analog_thermometer_IRQHandler(void);

/** THERMOMETER READ  RESULT */
uint8_t analog_thermometer_result(uint16_t* result);
uint8_t analog_thermometer_getResultStatus(uint8_t therm_number);
uint16_t analog_thermometer_getAverage(uint8_t therm_number);
uint8_t analog_thermometer_getLastResult(uint8_t therm_number, uint16_t* result);

#endif /** IC_ANALOG_THERMOMETER_H_ */
