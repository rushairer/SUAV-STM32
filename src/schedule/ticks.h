

#pragma once

#include "../platform.h"


extern uint8_t printTestTimer;
extern uint8_t powerupTimer;

void ticks_init(void);

void delay_ms(uint16_t cnt);

void ticks_stop(void);
uint16_t micros16(void);


