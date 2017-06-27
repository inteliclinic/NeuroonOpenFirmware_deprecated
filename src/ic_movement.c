/*
 * ic_movement.c
 *
 *  Created on: 29 wrz 2015
 *      Author: wojtek
 */
//---------------------------------------------------------------------------------------------------
#include "ic_movement.h"
#include "ic_fifo.h"
#include "ic_log.h"
#include "global_conf.h"
#include "ic_cc.h"
//---------------------------------------------------------------------------------------------------

#define ACC_FREQ				25			///frequency of reading data from accelerometer (in [Hz])
#define ACC_FIFO_SIZE			20			///size of accelerometer cyclic buffer
//---------------------------------------------------------------------------------------------------
static acc_data acc_buf[ACC_FIFO_SIZE];
static FIFO_MOVE* acc_fifobuff_ = &(FIFO_MOVE)FIFO_INIT(acc_buf, ACC_FIFO_SIZE);

//---------------------------------------------------------------------------------------------------
static acc_data acc_measurement_={
		5,6,7
};
static uint8_t acc_tmp_buffer[6];
uint16_t MOVE_data_counter=0;
int16_t MOVE_x_data_prev=0;
int16_t MOVE_y_data_prev=0;
int16_t MOVE_z_data_prev=0;
int32_t MOVE_x_data_diff=0;
int32_t MOVE_y_data_diff=0;
int32_t MOVE_z_data_diff=0;
uint64_t MOVE_current_acc_vector=0;

uint8_t MOVE_event=0;
uint32_t MOVE_maxDiffSum=0;
uint32_t MOVE_counter=0;

//---------------------------------------------------------------------------------------------------

//---------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------

bool MOVE_init() {
#ifdef ACC_LOG_RAW_DATA
	logfacc_clear(&lfacc);
#endif
	return LIS3DH_Config(LIS3DH_Set_Mode(6));
}
//---------------------------------------------------------------------------------------------------
void MOVE_Start(void) {
	LIS3DH_Start ();
}
//---------------------------------------------------------------------------------------------------
void MOVE_Stop(void) {
	LIS3DH_Stop ();
}
//---------------------------------------------------------------------------------------------------
/**
 * @fn MOVE_getRawData()
 * @brief read data from axis X,Y,Z
 * @return measurement data structure
 */
void MOVE_getRawData(acc_data* acc_axis) {
	TWI_ReadReg(TWI_ACC_ADDRESS, LIS3DH_X_LSB|0x80, &acc_tmp_buffer[0],6);

	acc_axis->ACC_X = (int16_t)((acc_tmp_buffer[1]<<8) | (acc_tmp_buffer[0]));
	acc_axis->ACC_Y = (int16_t)((acc_tmp_buffer[3]<<8) | (acc_tmp_buffer[2]));
	acc_axis->ACC_Z = (int16_t)((acc_tmp_buffer[5]<<8) | (acc_tmp_buffer[4]));
}
//---------------------------------------------------------------------------------------------------
/**
 * @fn MOVE_captureData()
 * @brief read data from axis X,Y,Z and write it down to the cycle buffer
 */
acc_data MOVE_captureData() {
	MOVE_getRawData(&acc_measurement_);
	fifoMOVE_add(acc_fifobuff_,acc_measurement_);
        return acc_measurement_;
}
//---------------------------------------------------------------------------------------------------
acc_data MOVE_IRQHandler(void) {
	///capturing data from the accelerometer
  return MOVE_captureData();
}
//---------------------------------------------------------------------------------------------------
uint32_t MOVE_getCountCapturedData() {
	return fifoMOVE_getCount(acc_fifobuff_);
}
//---------------------------------------------------------------------------------------------------
void MOVE_getLastCapturedData(acc_data* axis_data) {
	*axis_data = fifoMOVE_sget(acc_fifobuff_);
}
//---------------------------------------------------------------------------------------------------
uint8_t MOVE_getEvents(){
	return MOVE_event;
}
//---------------------------------------------------------------------------------------------------
uint16_t MOVE_getMaxDiffSum(){
	if (MOVE_maxDiffSum > 0xFFFF)
		return 0xFFFF;

	return (uint16_t)MOVE_maxDiffSum;
}
//---------------------------------------------------------------------------------------------------
void MOVE_clear_module(void){
	MOVE_counter = 0;
	MOVE_event = 0;
	MOVE_maxDiffSum = 0;
	fifoMOVE_clear(acc_fifobuff_);
}
//---------------------------------------------------------------------------------------------------
void MOVE_clearEpoch() {
	MOVE_event		= 0;
	MOVE_maxDiffSum	= 0;
	MOVE_counter = 0;
}
//---------------------------------------------------------------------------------------------------
