

#pragma once

#include "../platform.h"
#include "../common/fifo.h"
#include "../bus/uart.h"
#include "../schedule/ticks.h"
#include "altitude.h"
#include "imu.h"
#include "../common/mlib.h"
#include "../fc/decode.h"
#include "../fc/fc.h"
#include <stdio.h>

extern uint8_t opflowUartBuf[9];
extern uint8_t opflowUartIndex;

extern uint8_t opticalflowCtrlTimer;
extern int32_t OpticalflowAltitude;
extern float AngleDegreeForOF[2], HeadforOpticalflow;

extern uint8_t fOpticalflowLost;

extern float pixel_flow_distance[2];
extern float pixelVel[2];

extern float optGyro[2];
extern float optGyroLast[2];

extern uint8_t flagVelCtrl[2], opticalFlowEnbAfterStick[2];
extern int32_t opticalflowPID[2];

void opflow_init(void);
void getEstimatedPosition(void);
void opticalflow_ctrl(void);

// static void optGyro_buf_data_in(void);
