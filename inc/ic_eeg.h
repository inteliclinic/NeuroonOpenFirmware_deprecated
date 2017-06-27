//---------------------------------------------------------------------------------------------------
#ifndef INC_IC_EEG_H_
#define INC_IC_EEG_H_

#include <stdint.h>
#include <stdbool.h>
//---------------------------------------------------------------------------------------------------
typedef struct {
	//Structure validity indicators
	uint32_t	samplesN;

	//Two average powers
	uint32_t	avgTotalPower;
	uint32_t	avgTotalNoDelta;

	//Bands part
	uint8_t		partDelta;
	uint8_t		partTheta;
	uint8_t		partAlpha;
	uint8_t		partBeta;
	uint8_t		partSpindles;

	//Contact indicator for whole epoch
	uint8_t		contact;
} EEG_TesttoolingEpoch;
//---------------------------------------------------------------------------------------------------
/**
 * @brief	Clear whole EEG module. All filters and computed epoch data.
 *
 * @note	This function have to be called before using EEG module.
 */
void eeg_clearModule();
//---------------------------------------------------------------------------------------------------
/**
 * @brief	Add sample to eeg machine.
 * 			This function insert sample from ADS to internal FIFO. All needed computation is performed
 * 			in @ref eeg_computeNextSample().
 *
 * @param	sample	New sample inserted to internal FIFO.
 */
void eeg_addSample(int16_t sample);
//---------------------------------------------------------------------------------------------------
/**
 * @brief	Compute next epoch sample from internal FIFO.
 *			This is function computing powers and bands parts for testtooling.
 *
 * @return	True if next sample was computed, false otherwise. Value false is returned only if internal
 * 			FIFO is empty.
 */
bool eeg_computeNextSampleForTesttooling();
//---------------------------------------------------------------------------------------------------
/**
 * @brief	Get number of samples computed since @ref eeg_clearEpoch() or @ref eeg_clearModule() are called.
 *
 * @note 	This function can be used to make decision when program should close one epoch computation.
 *
 * @return	Number of samples computed by EEG module.
 */
uint16_t eeg_getComputedEpochSamplesNumber();
//---------------------------------------------------------------------------------------------------
/**
 * @brief	Get epoch testtooling data.
 *
 * @return	Object of @ref EEG_TesttoolingEpoch type with data computed from samples.
 */
EEG_TesttoolingEpoch eeg_getTesttoolingEpoch();
//---------------------------------------------------------------------------------------------------
#endif /* INC_IC_EEG_H_ */
//---------------------------------------------------------------------------------------------------
