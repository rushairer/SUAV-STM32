
#pragma once

#include "../platform.h"


/*********** RC alias *****************/
#define ROLL       0
#define PITCH      1
#define YAW        2
#define THROTTLE   3


typedef struct {
    uint8_t uartCmdNew:1;  
    uint8_t motorArmed:1;
    uint8_t motorIdle:1;  
    uint8_t bVelocityControl:1;  
    uint8_t bAccGyroCalibrationEnb:1;
    
    uint8_t bTakeoff:1;
    uint8_t bLanding:1;
} fc_ctrl_flags_s;

typedef struct {
    fc_ctrl_flags_s flags;
    uint8_t takeofflandTimer;
    int32_t takeoffAlt;
} fc_ctrl_s;

extern fc_ctrl_s fc;

void flight_ctrl(void);
void fc_init(void);



