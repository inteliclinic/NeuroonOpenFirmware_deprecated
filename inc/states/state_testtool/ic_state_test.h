/*
 * ic_state_test.h
 *
 *  Created on: 7 paź 2015
 *      Author: PB
 */

#ifndef IC_STATE_TEST_H_
#define IC_STATE_TEST_H_
#include <stdint.h>

typedef enum{
	LEDR1,
	LEDG1,
	LEDB1,
	LEDR2,
	LEDG2,
	LEDB2,
	POWER_LED,
	LEDS_OFF,
	ACC_ST,
	AFE_ST,
	AFE_ON,
	AFE_OFF,
	TEST_FINISHED,
	POWER_LED_NO_REFRESH,
	BANDWIDTH_INFO,
	VIBRA_ON,
	VIBRA_OFF,
	VIBRA_ST,
	ACC_ON,
	ALL_ST_CHECK,
	ACC_OFF,
	ACC_X,
	ACC_Y,
	ACC_Z,
	BATT_TEMP,
	SEND_SERIAL_RAM,
	SEND_SERIAL_RAM_HIDDEN,		//51 Q
	SEND_SERIAL_FLASH,
	SEND_SERIAL_FLASH_HIDDEN,	//52 R
	REMOVE_SERIAL,
	REMOVE_SERIAL_HIDDEN, 		//53 S
	GET_SERIAL_RAM,
	GET_SERIAL_FLASH,
	ELECTROTEST_ON,
	AFE_DIODE1,
	AFE_DIODE2,
	ELECTRODE_START_MEASUREMENT,
	ELECTRODE_STOP_MEASUREMENT,
	BQ_SHUTDOWN,
	BLUETOOTH_ON,
	BLUETOOTH_OFF,
	TEST_BLUETOOTH,
	BQ_PROGRAMMING
}state_flag;


void state_test_init(void);
void state_test_loop(void);
void state_test_deinit(void);
int test_vibrator_st(void);
void test_compute_electrotest(void);
void test_electrotest(void);
state_flag interpret_command(uint8_t command);

#endif /* IC_STATE_TEST_H_ */
