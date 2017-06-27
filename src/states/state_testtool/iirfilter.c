/*

 * iirfilter.c
 *
 *  Created on: 5 sie 2015
 *      Author: Tomasz Szleter
 */
//---------------------------------------------------------------------------------------------------------------------------
#include "iirfilter.h"
//---------------------------------------------------------------------------------------------------------------------------
int16_t iir_insert(IIR *iir, int16_t sample) {
	biquad_insert(&iir->bqarray[0], (int64_t)sample);
	for (uint8_t u = 1; u < iir->cascadeLen; u++) {
		biquad_insert(&iir->bqarray[u], iir->bqarray[u-1].out);
	}
	int64_t tmp = iir->bqarray[iir->cascadeLen-1].out;
	int64_t tmp2 = tmp;
	tmp *= iir->gNum;
	tmp /= iir->gDen;
	iir->out = (int16_t)tmp;

	tmp2 = tmp2*tmp2;
	tmp2 *= iir->g2Num;
	tmp2 /= iir->g2Den;
	iir->out2 = tmp2;
	iir->out2sum += tmp2;

	if (tmp < 0)
		tmp *= -1;

	return (int16_t)tmp;
}
//---------------------------------------------------------------------------------------------------------------------------
void  iir_clear(IIR *iir) {
	for (uint8_t u = 0; u < iir->cascadeLen; u++) {
		biquad_clear(&iir->bqarray[u]);
	}
	iir->out = 0;
	iir->out2sum = 0;
}
//---------------------------------------------------------------------------------------------------------------------------
