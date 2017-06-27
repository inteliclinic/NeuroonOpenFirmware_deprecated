//---------------------------------------------------------------------------------------------------
#include "ic_eeg.h"
#include <string.h>
#include <math.h>
#include "nordic_common.h"
//---------------------------------------------------------------------------------------------------
#define EEG_CONTACT_MAX_AMPLITUDE	32000	/// maximum signal amplitude for contact indicators calculation
#define EEG_INTERNAL_FIFO_SIZE		128		/// size of internal FIFO for new samples
//Epoch data objects
static EEG_TesttoolingEpoch testepoch;
//Internal fifo for samples
#define FIFO_TEMPLATE_DEF
#define FIFO_UNDERFLOW_RET		0
#define FIFO_TEMPLATE_DEC
#define FIFO_TEMPLATE_FUN_PREFIX	fifoi16
#define FIFO_TEMPLATE_TYPE		int16_t
#define	FIFO_TEMPLATE_STR_NAME		FIFO_i16
#include "ic_fifo_template.h"
#undef FIFO_UNDERFLOW_RET
#undef FIFO_TEMPLATE_DEC
#undef FIFO_TEMPLATE_FUN_PREFIX
#undef FIFO_TEMPLATE_TYPE
#undef FIFO_TEMPLATE_STR_NAME
#undef FIFO_TEMPLATE_DEF
static int16_t _fifo_buf_[EEG_INTERNAL_FIFO_SIZE];
static FIFO_i16 eegfifo = FIFO_INIT(_fifo_buf_, EEG_INTERNAL_FIFO_SIZE);
//Internal storage object
static struct {
	uint32_t	samplesN;
	uint32_t	contactFails;
	uint64_t	spindlesStdXsum;
	uint64_t	spindlesStdX2sum;
} iepoch;
//---------------------------------------------------------------------------------------------------
#include "ic_eeg_filters_def.h"
//---------------------------------------------------------------------------------------------------
void eeg_clearModule() {
	//Clear testtooling filters
	iir_clear(&iirTesttoolingTotal);
	iir_clear(&iirTesttoolingTotalNoDelta);
	iir_clear(&iirTesttoolingDelta);
	iir_clear(&iirTesttoolingTheta);
	iir_clear(&iirTesttoolingAlpha);
	iir_clear(&iirTesttoolingBeta);
	iir_clear(&iirTesttoolingSpindles);

	//Clear internal data associated with epoch computation
	memset(&iepoch, 0, sizeof(iepoch));

	//Clear fifo
	fifoi16_clear(&eegfifo);
}
//---------------------------------------------------------------------------------------------------
void eeg_addSample(int16_t sample) {
	fifoi16_add(&eegfifo, sample);
}
//---------------------------------------------------------------------------------------------------
uint16_t eeg_getComputedEpochSamplesNumber() {
	return iepoch.samplesN;
}
//---------------------------------------------------------------------------------------------------
bool eeg_computeNextSampleForTesttooling() {
	int16_t sample, tmp;
	if (fifoi16_sgetSecure(&eegfifo, &sample) == false)
		return false;

	//Increment samples in epoch
	iepoch.samplesN++;

	//Compute total filters
	iir_insert(&iirTesttoolingTotal, sample);
	iir_insert(&iirTesttoolingTotalNoDelta, sample);

	//Compute delta filter
	tmp = iir_getOut(&iirTesttoolingTotal);
	iir_insert(&iirTesttoolingDelta, tmp);

	//Compute other filters
	tmp = iir_getOut(&iirTesttoolingTotalNoDelta);
	iir_insert(&iirTesttoolingAlpha, tmp);
	iir_insert(&iirTesttoolingBeta, tmp);
	iir_insert(&iirTesttoolingTheta, tmp);
	iir_insert(&iirTesttoolingSpindles, tmp);

	if (sample >= EEG_CONTACT_MAX_AMPLITUDE || sample <= -EEG_CONTACT_MAX_AMPLITUDE) {
		iepoch.contactFails++;
	}

	return true;
}
//---------------------------------------------------------------------------------------------------
EEG_TesttoolingEpoch eeg_getTesttoolingEpoch() {
	uint32_t n = iepoch.samplesN;
	uint64_t tmpavg = 0;

	testepoch.samplesN = n;

	testepoch.avgTotalPower = (uint32_t)(iir_getOut2Sum(&iirTesttoolingTotal)/n);
	testepoch.avgTotalNoDelta = (uint32_t)(iir_getOut2Sum(&iirTesttoolingTotalNoDelta)/n);

	tmpavg = iir_getOut2Sum(&iirTesttoolingDelta)/n;
	testepoch.partDelta = (uint8_t)MIN((255*tmpavg)/testepoch.avgTotalPower, 255);

	tmpavg = iir_getOut2Sum(&iirTesttoolingAlpha)/n;
	testepoch.partAlpha = (uint8_t)MIN((255*tmpavg)/testepoch.avgTotalNoDelta, 255);

	tmpavg = iir_getOut2Sum(&iirTesttoolingBeta)/n;
	testepoch.partBeta = (uint8_t)MIN((255*tmpavg)/testepoch.avgTotalNoDelta, 255);

	tmpavg = iir_getOut2Sum(&iirTesttoolingTheta)/n;
	testepoch.partTheta = (uint8_t)MIN((255*tmpavg)/testepoch.avgTotalNoDelta, 255);

	tmpavg = iir_getOut2Sum(&iirTesttoolingSpindles)/n;
	testepoch.partSpindles = (uint8_t)MIN((255*tmpavg)/testepoch.avgTotalNoDelta, 255);

	//Simple contact indicator
	testepoch.contact = (255*iepoch.contactFails)/n;

	return testepoch;
}
//---------------------------------------------------------------------------------------------------
