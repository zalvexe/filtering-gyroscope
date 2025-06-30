/*
 * filter_FIR.h
 *
 *  Created on: Jun 4, 2025
 *      Author: zalfa
 */

#ifndef INC_FILTER_FIR_H_
#define INC_FILTER_FIR_H_

#include <stdint.h>

#define FIR_LEN 16

typedef struct{
	float buf[FIR_LEN];
	uint8_t bufIndex;
	float out;
}FIRFilter;

void FIRFilter_Init(FIRFilter *fir);
float FIRFilter_Update(FIRFilter *fir, float inp);
#endif /* INC_FILTER_FIR_H_ */
