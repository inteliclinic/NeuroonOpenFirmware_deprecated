//---------------------------------------------------------------------------------------------------------------------------
/*
 * iirfilter.h
 *
 *  Created on: 5 sie 2015
 *      Author: Tomasz Szleter
 */
//---------------------------------------------------------------------------------------------------------------------------
#ifndef INC_IIRFILTER_H_
#define INC_IIRFILTER_H_
//---------------------------------------------------------------------------------------------------------------------------
#include <stdint.h>
#include "biquad_filter.h"
//---------------------------------------------------------------------------------------------------------------------------
typedef struct {
	BIQUAD		*bqarray;		///< pointer to BIQUAD array
	uint8_t		cascadeLen;		///< length of BIQUAD array (cascade)
	uint32_t	gNum, gDen;		///< numerator and denominator of g
	uint64_t	g2Num, g2Den;	///< pre-compute g square
	int16_t		out;			///< last calculated filter output
	uint32_t	out2;			///< last calculated filter output square
	uint64_t	out2sum;		///< output square sum (for mean signal power)
} IIR;
//---------------------------------------------------------------------------------------------------------------------------
/**
 * Add sample to IIR filter and compute results.
 * @param iir Pointer to IIR filter instance.
 * @param sample Sample of signal.
 * @return Filtered sample behind iir filter.
 */
int16_t iir_insert(IIR *iir, int16_t sample);
//---------------------------------------------------------------------------------------------------------------------------
/**
 *Clear IIR filter instance.
 * @param iir Pointer to IIR filter instance.
 */
void  iir_clear(IIR *iir);
//---------------------------------------------------------------------------------------------------------------------------
/**
 * Clear output square sum used to mean signal power computation.
 * @param iir Pointer to IIR filter instance.
 */
static inline void iir_clearOut2Sum(IIR *iir) {
	iir->out2sum = 0;
}
//---------------------------------------------------------------------------------------------------------------------------
static inline int16_t iir_getOut(IIR *iir) {
	return iir->out;
}
//---------------------------------------------------------------------------------------------------------------------------
static inline uint32_t iir_getOut2(IIR *iir) {
	return iir->out2;
}
//---------------------------------------------------------------------------------------------------------------------------
static inline uint64_t iir_getOut2Sum(IIR *iir) {
	return iir->out2sum;
}
//---------------------------------------------------------------------------------------------------------------------------
#endif /* INC_IIRFILTER_H_ */
//---------------------------------------------------------------------------------------------------------------------------
