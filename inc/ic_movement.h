/*
 * @file ic_movement.h
 * @brief accelerometer measurements task header
 *
 *  Created on: 28 wrz 2015
 *      Author: wojtek
 *
 * HACK (taki jak u Tomka):
 * 1. In ic_movement.c file two parameters of accelerometer data acquisition can be configured:
 * 		#define ACC_FREQ		///frequency of reading data from accelerometer (in timer cycles)
 * 		#define ACC_CBUFF_SIZE	///size of accelerometer cyclic buffer
 *
 * 2. MOVE_IRQHandler() should be called in Timer IRQ handler.
 *
 * 3. In main() after hardware, devices and measurement timer initialization, MOVE module should be initialized by function MOVE_init()
 *
 * 4. Accelerometer starts working after use of MOVE_start() function and will be working until call of MOVE_stop() function.
 *
 * 5. An object of the acc_data structure should be implemented where functions to getting data are used - those functions work by modifying object, which is put as a parameter.
 *
 */

#ifndef IC_MOVEMENT_H_
#define IC_MOVEMENT_H_
#include "ic_lis3dh.h"



#ifndef ACC_DATA
#define ACC_DATA
typedef struct __attribute__((packed))
{
	int16_t ACC_X, ACC_Y, ACC_Z;
} acc_data;
#endif /* !ACC_DATA */
/** DEVICE DEPENDENT FUNCTIONS */
/** CONTROL */
/**
 * @fn MOVE_init()
 * @brief accelerometer module initialization via I2C (mode 1)
 * @return true if communication correct
 */
bool MOVE_init();

/**
 * @fn MOVE_Start ()
 * @brief accelerometer module start measurement by interrupt enable
 */
void MOVE_Start(void);

/**
 * @fn MOVE_Stop ()
 * @brief accelerometer module stop measurement by interrupt disable
 */
void MOVE_Stop(void);

/**
 * @fn MOVE_IRQHandler ()
 * @brief Handler to use in TIMER_IRQHandler function in ic_timers.c. Capturing data with period configurable via ACC_FREQ macro.
 */
acc_data MOVE_IRQHandler(void);

/**
 * @fn MOVE_getCountCapturedData ()
 * @brief Get number of accelerometer data captured to cyclic buffer.
 * @return Accelerometer measurements cyclic buffer counter.
 */
uint32_t MOVE_getCountCapturedData();

/**
 * @fn MOVE_getLastCapturedData ()
 * @brief Get recently captured values of 3 axis from accelerometer. Reset accelerometer measurements cyclic buffer counter.
 * @param structure with values of 3 axis, that will be modified
 */
void MOVE_getLastCapturedData(acc_data* axis_data);

/**
 * @fn MOVE_getNCapturedData ()
 * @brief Get N recently captured values of 3 axis from accelerometer. Reset accelerometer measurements cyclic buffer counter.
 * @param axis_data pointer to array of structures with values of 3 axis, that will be modified
 * @param n number of structures
 * @return 1 when n is higher than ACC_CBUFF_SIZE, 0 otherwise
 */
uint8_t MOVE_getNCapturedData(acc_data* axis_data, uint16_t n);

void MOVE_compute_data();

uint8_t MOVE_getEvents();

uint16_t MOVE_getMaxDiffSum();

void MOVE_clear_module(void);

void MOVE_clearEpoch();

#endif /* IC_MOVEMENT_H_ */
