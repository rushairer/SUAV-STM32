

#pragma once

#include "../platform.h"


//------------------------------------------------------------------------------------
typedef void (*hw_timer_iqr_callback)(void);


void hw_tim6_init(hw_timer_iqr_callback cb);
void hw_tim4_init(void);

void hw_tim3_init(void);


