


#pragma once     

//------------------------------------------------------------------------------------
#include "../platform.h" 


//------------------------------------------------------------------------------------

#define DELTA_MAX_SAMPLES 12

typedef struct pt1Filter_s {
    float state;
    float RC;
    float dT;
} pt1Filter_t;

/* this holds the data required to update samples thru a filter */
typedef struct biquadFilter_s {
    float b0, b1, b2, a1, a2;
    float d1, d2;
} biquadFilter_t;

typedef enum {
    FILTER_PT1 = 0,
    FILTER_BIQUAD,
} filterType_e;

typedef enum {
    FILTER_LPF,
    FILTER_NOTCH
} biquadFilterType_e;


//------------------------------------------------------------------------------------
typedef float (*filterApplyFnPtr)(void *filter, float input);

float nullFilterApply(void *filter, float input);

void pt1FilterInit(pt1Filter_t *filter, uint8_t f_cut, float dT);
float pt1FilterApply(pt1Filter_t *filter, float input);
float pt1FilterApply4(pt1Filter_t *filter, float input, uint8_t f_cut, float dT);

void biquadFilterInitLPF(biquadFilter_t *filter, float filterFreq, uint32_t sampleDeltaUs);
float biquadFilterApply(biquadFilter_t *filter, float input);

	 





