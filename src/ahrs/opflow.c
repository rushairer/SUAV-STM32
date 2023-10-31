

#include "opflow.h"
// #include "../bus/uart.h"
// #include "../schedule/ticks.h"
// #include "altitude.h"
// #include "imu.h"
// #include "../common/mlib.h"
// #include "../fc/decode.h"
// #include "../fc/fc.h"
// #include <stdio.h>
#include <string.h>

uint8_t opflowUartBuf[9];
uint8_t opflowUartIndex;

uint8_t opticalflowCtrlTimer, opticalflowMonitorTimer;
int32_t OpticalflowAltitude = 0;
float AngleDegreeForOF[2], HeadforOpticalflow = 0;
int32_t OpticalflowAltitudeScaled;

uint8_t fOpticalflowLost;

float pixel_flow_distance[2] = {0.0f, 0.0f};
float pixelVel[2];

float optGyro[2]       = {0};
float optGyroFilter[2] = {0};

uint8_t flagVelCtrl[2] = {0, 0}, opticalFlowEnbAfterStick[2] = {0, 0};
int32_t opticalflowPID[2] = {0, 0};

#define OPT_GYRO_DELAY_IN_10MS 2
#define OPT_GYRO_BUF_SIZE      OPT_GYRO_DELAY_IN_10MS * 2
#define OPT_GYRO_SCALE         764

float optGyro_buf[OPT_GYRO_BUF_SIZE][2];
uint8_t optGyro_buf_head;

static void optGyro_buf_data_in(void)
{
    optGyro_buf[optGyro_buf_head][0] = optGyro[0];
    optGyro_buf[optGyro_buf_head][1] = optGyro[1];
    if (++optGyro_buf_head >= OPT_GYRO_BUF_SIZE) {
        optGyro_buf_head = 0;
    }
    memset(optGyro, 0, sizeof(optGyro));
}
static void optGyro_get(int16_t *rads)
{
    float sum[2];
    uint8_t index, cnt;

    memset(sum, 0, sizeof(sum));
    for (index = optGyro_buf_head, cnt = 0; cnt < OPT_GYRO_DELAY_IN_10MS; index++, cnt++) {
        if (index >= OPT_GYRO_BUF_SIZE) {
            index = 0;
        }
        sum[0] += optGyro_buf[index][0];
        sum[1] += optGyro_buf[index][1];
    }

    rads[0] = sum[0] * OPT_GYRO_SCALE;
    rads[1] = sum[1] * OPT_GYRO_SCALE;
}

void opflow_init(void)
{
    opflow_uart_init();
    opflowUartIndex = 0;

    memset(optGyro_buf, 0, sizeof(optGyro_buf));
    optGyro_buf_head = 0;

    HeadforOpticalflow = 0;
    fOpticalflowLost   = 0;
}

void opticalflow_update(void)
{
    uint8_t opflowReady = 0;
    int16_t pixel_flow_integral[2];
    int16_t pixel_flow_integral_filter[2];
    static int16_t pixel_flow_integral_last[2] = {0};

    static uint16_t lastTimeForPixel = 0;
    uint16_t currentTimeForPixel;
    float pixelDelta;

    int16_t optGyroRad[2];
    float optSpeed[2];

    uint8_t axis;
    float AngleDistanceDelta;

    if (opflow_uart_available()) {
        opflowUartBuf[opflowUartIndex] = opflow_uart_read_rx_fifo();

        // 0xfe 0x04 d0 d1 d2 d3 sum q 0xaa
        if (opflowUartIndex == 0) {
            if (opflowUartBuf[opflowUartIndex] == 0xfe) opflowUartIndex++;
        } else {
            opflowUartIndex++;
            if (opflowUartIndex >= 9) {
                if (opflowUartBuf[9 - 1] == 0xaa) {
                    opflowReady = 1;
                    // printf("or\r\n");
                }
                opflowUartIndex = 0;
            }
        }
    }

    if (opflowReady) {
        //------------------------------------------------------------------------------------------------------------
        pixel_flow_integral[0] = (int16_t)(((uint16_t)opflowUartBuf[4]) | ((uint16_t)opflowUartBuf[5] << 8));
        pixel_flow_integral[1] = (int16_t)(((uint16_t)opflowUartBuf[2]) | ((uint16_t)opflowUartBuf[3] << 8));

        pixel_flow_integral_filter[0] = (pixel_flow_integral[0] + pixel_flow_integral_last[0]) / 2;
        pixel_flow_integral_filter[1] = (pixel_flow_integral[1] + pixel_flow_integral_last[1]) / 2;
        pixel_flow_integral_last[0]   = pixel_flow_integral_filter[0];
        pixel_flow_integral_last[1]   = pixel_flow_integral_filter[1];

        opticalflowMonitorTimer = 0;
        fOpticalflowLost        = 0;

        //------------------------------------------------------------------------------------------------------------
        optGyro_get(optGyroRad);
        optGyroRad[0]    = constrain(optGyroRad[0], -28, 28);
        optGyroRad[1]    = constrain(optGyroRad[1], -28, 28);
        optGyroFilter[0] = (optGyroFilter[0] + optGyroRad[0]) / 2;
        optGyroFilter[1] = (optGyroFilter[1] + optGyroRad[1]) / 2;

        // printf("%d,%d,%d,%d\n",pixel_flow_integral[0], pixel_flow_integral[1], optGyroRad[0], optGyroRad[1]);
        // printf("%f,%f,%f,%f\n",optSpeed[0]*10, optSpeed[1]*10, optGyroFilter[0]*10, optGyroFilter[1]*10);

        //------------------------------------------------------------------------------------------------------------
        // 2 计算delta、光流摄像头离地高度(mm)
        currentTimeForPixel = micros16();
        pixelDelta          = (uint16_t)(currentTimeForPixel - lastTimeForPixel) * 1e-6;
        ;
        lastTimeForPixel = currentTimeForPixel;

        OpticalflowAltitude = (OpticalflowAltitude * 3 + max(EstAlt, 250)) / 4;
        OpticalflowAltitude = constrain(OpticalflowAltitude, 100, 6000);

        if (OpticalflowAltitude >= 2500) {
            OpticalflowAltitudeScaled = 2500 + (OpticalflowAltitude - 2500) / 3;
        } else {
            OpticalflowAltitudeScaled = OpticalflowAltitude;
        }

        // 3 计算光流位移及速度
        //   @ 飞行器位移 = 光流位移(测量值) - 机体旋转位移(由陀螺仪积分得到旋转角度)
        //   @ 光流输出量/10000 = 光流摄像头移动(旋转)角度，单位是弧度
        for (axis = 0; axis < 2; axis++) {
            optSpeed[axis] = optGyroFilter[axis] - pixel_flow_integral_filter[axis];
            optSpeed[axis] /= 500.0f;
            AngleDistanceDelta = optSpeed[axis] * OpticalflowAltitudeScaled; // OpticalflowAltitude;

            pixel_flow_distance[axis] += AngleDistanceDelta; // * pixelDelta;
            pixel_flow_distance[axis] = constrain_float(pixel_flow_distance[axis], -3000.0f, 3000.0f);
            pixelVel[axis]            = AngleDistanceDelta / pixelDelta;
        }

        // printf("%d,%d,%d,%d\n", (int)pixel_flow_distance[0], (int)pixel_flow_distance[1], (int)pixelVel[0], (int)pixelVel[1]);
        // printf("%d,%d,%d,%d,%d,%d\n", (int)velGlobal[0], (int)velGlobal[1], (int)pixelVel[0], (int)pixelVel[1],(int)pixel_flow_distance[0],(int)pixel_flow_distance[1]);
    }
}

void __attribute__((weak)) getEstimatedPosition(void)
{
    // static uint32_t previousT;                                     //Used for update vel & altitude
    // uint32_t currentT;
    float dt;
    uint16_t axis;

    volatile float Pf_Pos;
    volatile float Pf_Vel, If_Vel, Df_Vel;

    static int32_t setVelOpticalflow[2] = {0, 0};
    static float VelCurrent[2] = {0.0f}, VelCurrentEarth[2] = {0.0f};
    int32_t error;
    static int32_t lastError[2]         = {0, 0};
    static float errorOpitalflowVelI[2] = {0, 0};
    static int32_t delta1[2] = {0, 0}, delta2[2] = {0, 0};
    int32_t delta, deltaSum;

    static uint16_t stickTimer[2];

    //-------------------------------------------------------------------------------------------------
    dt = 0.01f;

    for (axis = 0; axis < 2; axis++) {
        Pf_Pos = 0.6f;
        Pf_Vel = 1.2f;
        If_Vel = 0.12f;
        Df_Vel = 0.01f;

        if (!fc.flags.motorArmed) {
            pixel_flow_distance[axis] = 0;
            pixelVel[axis]            = 0;
            distanceGlobal[axis]      = 0;
            velGlobal[axis]           = 0;
            errorOpitalflowVelI[axis] = 0;

            flagVelCtrl[axis]       = 0;
            setVelOpticalflow[axis] = 0;
        }

        //- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        //- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        if (rcCommand[axis] != 0 || rcCommand[YAW] != 0 || stickTimer[axis] < 100) {
            //----------------------------------------------------------------------------------
            VelCurrentEarth[axis] += ((float)(angle[axis] - rcCommandTrim[0])) * dt - 0.10f * VelCurrentEarth[axis] * dt;
            VelCurrentEarth[axis] = constrain_float(VelCurrentEarth[axis], -1000.0f, +1000.0f);

            VelCurrent[axis] = VelCurrentEarth[axis];

            //----------------------------------------------------------------------------------
            setVelOpticalflow[axis]   = 0;
            pixelVel[axis]            = 0;
            pixel_flow_distance[axis] = 0;
            distanceGlobal[axis]      = 0;

            pixelVel[axis]            = 0;
            errorOpitalflowVelI[axis] = 0;
            lastError[axis]           = 0;
            delta1[axis]              = 0;
            delta2[axis]              = 0;

            if (rcCommand[axis] != 0 || rcCommand[YAW] != 0) {
                if (stickTimer[axis] != 0) {
                    VelCurrentEarth[axis] = 0;
                }
                stickTimer[axis] = 0;
                VelCurrent[axis] = 0;
            } else {
                stickTimer[axis]++;
            }

            velGlobal[axis] = VelCurrent[axis];

        } else {
            distanceGlobal[axis] = distanceGlobal[axis] * 0.92f + pixel_flow_distance[axis] * 0.08f;
            velGlobal[axis]      = velGlobal[axis] * 0.92f + (float)pixelVel[axis] * 0.08f;
            VelCurrent[axis]     = velGlobal[axis];

            setVelOpticalflow[axis] = constrain((int16_t)(-Pf_Pos * distanceGlobal[axis]), -3000, +3000);

            VelCurrentEarth[axis] = 0;
        }

//        printf("%d,%d,%d,%d\n", (int)velGlobal[0], (int)velGlobal[1], (int)distanceGlobal[0], (int)distanceGlobal[1]);

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// Part3  PID
//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// Velocity PID-Controller
// P
#define OPTICALFLOW_PROPORTION_LIMITED 2500
        error                = setVelOpticalflow[axis] - VelCurrent[axis];
        opticalflowPID[axis] = constrain((Pf_Vel * error), -OPTICALFLOW_PROPORTION_LIMITED, +OPTICALFLOW_PROPORTION_LIMITED);

// I
#define OPTICALFLOW_INTEGRAL_LIMITED 2000
        errorOpitalflowVelI[axis] += (If_Vel * error) * dt;
        if (errorOpitalflowVelI[axis] > OPTICALFLOW_INTEGRAL_LIMITED) errorOpitalflowVelI[axis] = OPTICALFLOW_INTEGRAL_LIMITED;
        if (errorOpitalflowVelI[axis] < -OPTICALFLOW_INTEGRAL_LIMITED) errorOpitalflowVelI[axis] = -OPTICALFLOW_INTEGRAL_LIMITED;
        opticalflowPID[axis] += errorOpitalflowVelI[axis];

// D
#define OPTICALFLOW_DIFFERENTIAL_LIMITED 1500
        if (dt != 0)
            delta = (error - lastError[axis]) / dt;
        else
            delta = 0;
        deltaSum     = delta1[axis] + delta2[axis] + delta;
        delta2[axis] = delta1[axis];
        delta1[axis] = delta;
        opticalflowPID[axis] += constrain(deltaSum * Df_Vel, -OPTICALFLOW_DIFFERENTIAL_LIMITED, OPTICALFLOW_DIFFERENTIAL_LIMITED);
        lastError[axis] = error;

        if (fOpticalflowLost) {
            opticalflowPID[axis] = 0;
        }
    }
}

void opticalflow_ctrl(void)
{
    //-------------------------------------------------------------------------------
    opticalflow_update();

    if (opticalflowCtrlTimer >= 10) {
        opticalflowCtrlTimer = 0;
        optGyro_buf_data_in();
        getEstimatedPosition();

        if (++opticalflowMonitorTimer >= 100) { // 1s
            fOpticalflowLost = 1;
        }
    }
}
