

#include "filter.h"
#include <math.h>

#define M_LN2_FLOAT 0.69314718055994530942f
#define M_PI_FLOAT  3.14159265358979323846f

#define BIQUAD_BANDWIDTH 1.9f           // bandwidth in octaves
#define BIQUAD_Q (1.0f / sqrtf(2.0f))   // 2nd order Butterworth Q


#define UNUSED(x) (void)(x)

//------------------------------------------------------------------------------------
// NULL filter
float nullFilterApply(void *filter, float input)
{
    UNUSED(filter);
    return input;
}

//------------------------------------------------------------------------------------
// PT1 Low Pass filter
void pt1FilterInit(pt1Filter_t *filter, uint8_t f_cut, float dT)
{
    filter->RC = 1.0f / ( 2.0f * M_PI_FLOAT * f_cut );
    filter->dT = dT;
}

float pt1FilterApply(pt1Filter_t *filter, float input)
{
    filter->state = filter->state + filter->dT / (filter->RC + filter->dT) * (input - filter->state);
    return filter->state;
}

float pt1FilterApply4(pt1Filter_t *filter, float input, uint8_t f_cut, float dT)
{
    // Pre calculate and store RC
    if (!filter->RC) {
        pt1FilterInit(filter, f_cut, dT);
    }
    filter->state = filter->state + filter->dT / (filter->RC + filter->dT) * (input - filter->state);
    return filter->state;
}

//------------------------------------------------------------------------------------
// BIQUAD Filter
static void biquadFilterInit(biquadFilter_t *filter, float filterFreq, uint32_t sampleDeltaUs, float Q, biquadFilterType_e filterType)
{
    // setup variables
    const float sampleHz = 1 / ((float)sampleDeltaUs * 0.000001f);
    const float omega = 2 * M_PI_FLOAT * filterFreq / sampleHz;
    const float sn = sinf(omega);
    const float cs = cosf(omega);
    const float alpha = sn / (2 * Q);

    float b0, b1, b2, a0, a1, a2;

    switch (filterType) {
        default:
        case FILTER_LPF:
            b0 = (1 - cs) / 2;
            b1 = 1 - cs;
            b2 = (1 - cs) / 2;
            a0 = 1 + alpha;
            a1 = -2 * cs;
            a2 = 1 - alpha;
            break;
        case FILTER_NOTCH:
            b0 =  1;
            b1 = -2 * cs;
            b2 =  1;
            a0 =  1 + alpha;
            a1 = -2 * cs;
            a2 =  1 - alpha;
            break;
    }

    // precompute the coefficients
    filter->b0 = b0 / a0;
    filter->b1 = b1 / a0;
    filter->b2 = b2 / a0;
    filter->a1 = a1 / a0;
    filter->a2 = a2 / a0;

    // zero initial samples
    filter->d1 = filter->d2 = 0;
}

/* sets up a biquad Filter */
void biquadFilterInitLPF(biquadFilter_t *filter, float filterFreq, uint32_t sampleDeltaUs)
{
    biquadFilterInit(filter, filterFreq, sampleDeltaUs, BIQUAD_Q, FILTER_LPF);
}

/* Computes a biquadFilter_t filter on a sample */
float biquadFilterApply(biquadFilter_t *filter, float input)
{
    const float result = filter->b0 * input + filter->d1;

    filter->d1 = filter->b1 * input - filter->a1 * result + filter->d2;
    filter->d2 = filter->b2 * input - filter->a2 * result;

#ifdef DEBUG_BIQUAD_INFINITY
    if (!isfinite(filter->d1) || !isfinite(filter->d2) ) {
        failureMode(FAILURE_DEVELOPER);
    }
#endif

    return result;
}







