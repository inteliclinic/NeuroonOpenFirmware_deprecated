//---------------------------------------------------------------------------------------------------------------------------
/*

 * biquad_filter.h
 *
 *  Created on: 5 sie 2015
 *      Author: Tomasz Szleter
 */
//---------------------------------------------------------------------------------------------------------------------------
#ifndef INC_BIQUAD_FILTER_H_
#define INC_BIQUAD_FILTER_H_
//---------------------------------------------------------------------------------------------------------------------------
#include <stdint.h>
//---------------------------------------------------------------------------------------------------------------------------
typedef struct {
	int16_t a1, a2, b0, b1, b2;
	int16_t den;
	int64_t zi1, zi2;
	int64_t zo;
	int64_t out;
} BIQUAD;
//---------------------------------------------------------------------------------------------------------------------------
/**
 * Add sample to BIQUAD filter and compute results.
 * @param bq Pointer to BIQUAD filter instance.
 * @param sample Sample of signal.
 * @return Filtered sample behind BIQUAD filter.
 */
int64_t biquad_insert(BIQUAD *bq, int64_t sample);
//---------------------------------------------------------------------------------------------------------------------------
/**
 * Clear biquad filter.
 * @param bq Pointer to BIQUAD filter instance.
 */
static inline void biquad_clear(BIQUAD *bq)  {
	bq->zi1 = bq->zi2 = bq->zo = bq->out = 0;
}
//---------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------

//---------------------------------------------------------------------------------------------------------------------------
#endif /* INC_BIQUAD_FILTER_H_ */
//---------------------------------------------------------------------------------------------------------------------------
