
#pragma once

#include "../platform.h"

typedef bool (*baroOpFuncPtr)(void);
typedef bool (*baroCalculateFuncPtr)(int32_t *pressure, int32_t *temperature);

typedef struct baroDev_s {
    uint16_t ut_delay;
    uint16_t up_delay;
    baroOpFuncPtr start_ut;
    baroOpFuncPtr get_ut;
    baroOpFuncPtr start_up;
    baroOpFuncPtr get_up;
    baroCalculateFuncPtr calculate;
} baroDev_t;

extern uint8_t baroUpdateTimer;
extern baroDev_t baro;
extern int32_t baroPressure, baroTemperature;

bool spl06Detect(baroDev_t *baro);

void baro_init(void);
bool baro_update(void);

void baro_test(void);


