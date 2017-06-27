//---------------------------------------------------------------------------------------------------
#include <ic_pulse.h>
#include <ic_log.h>
#include <ic_fifo.h>
#include <ic_cbuffer.h>
//---------------------------------------------------------------------------------------------------
static int32_t	_buf_for_fifo[25];
static FIFO_i32	pfifo = FIFO_INIT(_buf_for_fifo, sizeof(_buf_for_fifo)/sizeof(int32_t));
//---------------------------------------------------------------------------------------------------
/* FIR(100, [0.83, 1.83]/12.50) den=32000 */
static const int32_t pulsefir[101] = {0,4,9,14,18,20,16,6,-11,-34,-61,-88,-109,-118,-112,-87,-46,6,61,107,136,140,120,81,35,0,-7,28,111,235,381,518,609,616,510,276,-76,-514,-983,-1414,-1733,-1872,-1787,-1461,-917,-208,579,1343,1980,2403,2550,2403,1980,1343,579,-208,-917,-1461,-1787,-1872,-1733,-1414,-983,-514,-76,276,510,616,609,518,381,235,111,28,-7,0,35,81,120,140,136,107,61,6,-46,-87,-112,-118,-109,-88,-61,-34,-11,6,16,20,18,14,9,4,0};
static int32_t _buf_for_cb[101];
static CBi32 cbPulse = CBUFFER_INIT(_buf_for_cb, 101);
//---------------------------------------------------------------------------------------------------
static int32_t	lastSample	= 0;	///Last sample
static uint32_t	n			= 0;	///Num of samples from last (+)->(-) derivative transition
static uint32_t	lastPulse	= 0;	///Last pulse
static uint32_t	lastPrv		= 0;	///Last prv
static int32_t	dx			= 0;	///Last derivative (-constant)
//---------------------------------------------------------------------------------------------------
static uint32_t	pulseSum	= 0;	///Sum of pulse samples from peak to peak in all epoch
static uint32_t	prvSum		= 0;	///Sum of abs(pulse_prev - pulse_act)
static uint32_t	numSum		= 0;	///Num of pulses in epoch
//---------------------------------------------------------------------------------------------------
void pulse_init() {
	lastSample	= 0;
	n			= 0;
	lastPulse	= 0;
	lastPrv		= 0;
	dx			= 0;

	pulseSum	= 0;
	prvSum		= 0;
	numSum		= 0;
}
//---------------------------------------------------------------------------------------------------
void pulse_clearEpoch() {
	pulseSum	= 0;
	prvSum		= 0;
	numSum		= 0;
}
//---------------------------------------------------------------------------------------------------
bool pulse_addSample(int32_t sample) {
	return fifoi32_sadd(&pfifo, sample);
}
//---------------------------------------------------------------------------------------------------
bool pulse_compute() {
	int32_t sample;
	if (fifoi32_sgetSecure(&pfifo, &sample)) {
		log_irled_byfifo(sample);
		cbi32_add(&cbPulse, sample);
		sample = (int32_t)(cbi32_weightedSumFromNewest(&cbPulse, pulsefir)/32000);

		//log_redled_byfifo(sample);

		int32_t ndx = sample - lastSample - 15;

		n++;

		//Maximum detection
		if (dx > 0 && ndx < 0 && n >= 12) {
			if (n <= 54) {
				//Calculate pulse
				uint32_t pulse	= 60000/PULSE_SAMPLE_RATE/n;
				uint32_t prv	= (pulse > lastPulse)?(pulse - lastPulse):(lastPulse - pulse);

				//Update sums
				numSum++;
				pulseSum	+= pulse;
				prvSum		+= prv;

				//Update last values
				lastPulse	= pulse;
				lastPrv 	= prv;
			}
			n = 0;
		}

		//log_nn_byfifo((int32_t)lastPulse);
		//log_redled_byfifo((int32_t)lastPrv);

		lastSample	= sample;
		dx			= ndx;

		return true;
	}

	return false;
}
//---------------------------------------------------------------------------------------------------
PulseData pulse_get() {
	PulseData	pd = {0, 0, 0};

	pd.n		= numSum;

	if (pd.n == 0)
		return pd;

	pd.pulse	= pulseSum/numSum;
	pd.prv		= prvSum/numSum;

	return pd;
}
//---------------------------------------------------------------------------------------------------


