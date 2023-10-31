
#pragma once

#include "../platform.h"


/*********** RC CMD INDEX *****************/
#define FC_CMD_LENGTH     10

#define FC_CMD_ROLL       0
#define FC_CMD_PITCH      1
#define FC_CMD_THROTTLE   2
#define FC_CMD_YAW        3

#define FC_CMD_ROLL_TRIM       4
#define FC_CMD_PITCH_TRIM      5
#define FC_CMD_THROTTLE_TRIM   6
#define FC_CMD_YAW_TRIM        7

#define FC_CMD_FLAGS_1    8
#define FC_CMD_FLAGS_2    9      


#define STICK_TRIM_LIMIT_RANGE             63
#define STICK_ENCODE_LIMIT_MIDDLE          128

extern uint8_t calibrationEnbTimer;

extern uint8_t fcCmd[FC_CMD_LENGTH];
extern int16_t rcCommand[4], rcCommandTrim[3];
extern int16_t setVelocity;

void decode(uint8_t* cmd);
void annexCode(void);

void decode_test(void);



