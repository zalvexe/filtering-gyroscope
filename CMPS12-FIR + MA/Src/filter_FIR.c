/*
* filter_FIR.c
*
*  Created on: Jun 4, 2025
*      Author: zalfa
*/
#include "filter_FIR.h"

static float FIR_IMPULSE_RESPONSE[FIR_LEN] = {-0.0032906f,-0.0052635f,-0.0068811f,0.0000000f,0.0254209f,0.0724719f,0.1311260f,0.1805961f,0.2000000f,0.1805961f,0.1311260f,0.0724719f,0.0254209f,0.0000000f,-0.0068811f,-0.0052635f};

void FIRFilter_Init(FIRFilter *fir){
	//clear buffer
	for(uint8_t n = 0; n < FIR_LEN; n++){
		fir->buf[n] = 0.0f;
	}

	//resetting index buffer
	fir->bufIndex = 0;
	//clear filter output
	fir->out = 0.0f;
}

float FIRFilter_Update(FIRFilter *fir, float input_fir){
	//sotring input to circular buffer
	fir->buf[fir->bufIndex] = input_fir;

	//increment index and limiting
	fir->bufIndex++;
	if(fir->bufIndex == FIR_LEN){
		fir->bufIndex = 0; // index back to 0
	}

	//set output to 0 before storing the output filter
	fir->out = 0.0f;
	uint8_t sumIndex = fir->bufIndex; //temp to check the total index already checked

	for(uint8_t n = 0; n < FIR_LEN; n++){
		if(sumIndex > 0) sumIndex--;
		else sumIndex=FIR_LEN-1;
		//y[n] = sigma from j = 0 to j = n-1 of h[j] * x[n-j]
//		y[n] = sigma from n = 0 to n = FIR_LEN-1 of h[n] * x[FIR-LEN-n]
		fir->out += FIR_IMPULSE_RESPONSE[n]*fir->buf[sumIndex];
	}
	return fir->out;
}

