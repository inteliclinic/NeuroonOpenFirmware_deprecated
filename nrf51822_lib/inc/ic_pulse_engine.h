//---------------------------------------------------------------------------------------------------
#ifndef NRF51822_LIB_INC_IC_PULSE_ENGINE_H_
#define NRF51822_LIB_INC_IC_PULSE_ENGINE_H_
//---------------------------------------------------------------------------------------------------

//#define PULSE_ENG_MODULE_TEST

#ifdef PULSE_ENG_MODULE_TEST
#include <stdint.h>
#include <stdio.h>
#include <float.h>
#include <math.h>
#else
#include "ic_lib_config.h"
#endif

#define PE_SAMPLE_TYPE		int32_t
#define PE_SAMPLE_PERIOD	20		//in milliseconds
//---------------------------------------------------------------------------------------------------
/**
 * @brief Clear module and reset to default state. Should be call before any other module operation.
 */
void PE_ClearAndInit();
//---------------------------------------------------------------------------------------------------
/**
 * @brief Add next sample to module engine and make essential computation.
 * @param sample New sample inserted to module.
 */
void PE_AddSample(PE_SAMPLE_TYPE sample);

//---------------------------------------------------------------------------------------------------
/**
 * @brief Add next sample to module engine and make essential computation.
 * @param sample New sample inserted to module.
 */
void PE_AddSampleToFilter(PE_SAMPLE_TYPE sample);

//---------------------------------------------------------------------------------------------------
/**
 * @brief	This function return information about how many pulse incidents (time between two signal
 * 			maximum) are captured during operation since last PE_ClearAndInit() call.
 * @return	Positive number of captured pulse incident or zero if there is not enough data.
 * @see		PE_AddSample
 */
uint16_t PE_GetNumberOfCapturedPulseIncident();
//---------------------------------------------------------------------------------------------------
/**
 * @brief	Get average pulse computed from captured pulse incidents.
 * @return	Average value of computed pulse if module has enough data or -1 otherwise.
 * @see		PE_AddSample, PE_GetNumberOfCapturedPulseIncident
 */
int16_t PE_GetAveragePulse();
//---------------------------------------------------------------------------------------------------
#endif /* NRF51822_LIB_INC_IC_PULSE_ENGINE_H_ */
//---------------------------------------------------------------------------------------------------
