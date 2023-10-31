
#include "altitude.h"
// #include "barometer.h"
// #include "../common/mlib.h"
// #include "../fc/fc.h"
// #include "../fc/decode.h"
// #include "../ahrs/imu.h"
// #include "../schedule/ticks.h"
// #include <math.h>
// #include <stdio.h>

int16_t tiltAngle = 0;

#define BCALIBRATING_BARO_CYCLES 50

// Baro => BaroAlt baroVel
#define BARO_TAB_SIZE_MAX       7 // 21 //15  //11 //21
#define PRESSURE_SAMPLES_MEDIAN 5 // 3  //5   //7  //3

uint16_t calibratingB      = BCALIBRATING_BARO_CYCLES;
int32_t baroGroundAltitude = 0;

int32_t BaroAlt = 0, lastBaroAlt = 0;
int32_t baroVel = 0;

// Acc => vel accAlt(EstAlt=accAlt)  & EstAlt
int32_t accSumZ[3]    = {0, 0, 0};
uint16_t accSumZCount = 1;
float accSumZTimer    = 0;
int16_t accZGlobal    = 0;
float vel             = 0;
float accAlt          = 0.0f;
int32_t EstAlt; // in mm

float velGlobal[3] = {0.0f, 0.0f, 0.0f}, distanceGlobal[3] = {0.0f, 0.0f, 0.0f};

int32_t AltHold;
int32_t BaroPID      = 0;
int16_t BaroThrottle = 1550;

// 控制变量
uint8_t altitudeCtrlTimer = 0;
int16_t setVel            = 0;

float Pf_Pos = 1.00f;
float Pf_Vel = 0.50f;
float If_Vel = 1.00f;
float Df_Vel = 0.005f;

int32_t integralBaroThrottle = 0;

void getEstimatedAltitude(void);

//--------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
int32_t applyBarometerMedianFilter(int32_t newPressureReading)
{
    static int32_t barometerFilterSamples[PRESSURE_SAMPLES_MEDIAN];
    static uint16_t currentFilterSampleIndex = 0;
    static bool medianFilterReady            = false;
    uint16_t nextSampleIndex;

    nextSampleIndex = (currentFilterSampleIndex + 1);
    if (nextSampleIndex == PRESSURE_SAMPLES_MEDIAN) {
        nextSampleIndex   = 0;
        medianFilterReady = true;
    }

    barometerFilterSamples[currentFilterSampleIndex] = newPressureReading;
    currentFilterSampleIndex                         = nextSampleIndex;

    if (medianFilterReady) {
        if (PRESSURE_SAMPLES_MEDIAN == 3)
            return quickMedianFilter3(barometerFilterSamples);
        else if (PRESSURE_SAMPLES_MEDIAN == 5)
            return quickMedianFilter5(barometerFilterSamples);
        else if (PRESSURE_SAMPLES_MEDIAN == 7)
            return quickMedianFilter7(barometerFilterSamples);
        else
            return newPressureReading;
    } else
        return newPressureReading;
}

//--------------------------------------------------------------------------------------------------------------
//                               定高(外环PID)
//--------------------------------------------------------------------------------------------------------------
void altitude_baro_update(void)
{
    static int32_t baroHistTab[BARO_TAB_SIZE_MAX];
    static uint16_t baroHistIdx = 0, indexplus1, bBaroSumReady = 0;
    static int32_t baroPressureSum = 0, baroGroundPressure = 0;
    int32_t baroPressureAvg;
    int32_t BaroAlt_tmp;
    uint8_t i, j;

    if (baro_update()) {
        //----------------------------------------------------------------------------------------------------
        if (fc.flags.bVelocityControl || (!fc.flags.motorArmed && !fc.flags.motorIdle)) {
            j = BARO_TAB_SIZE_MAX / 1; // 3;
        } else {
            j = 1;
        }
        for (i = 0; i < j; i++) {
            indexplus1 = (baroHistIdx + 1);
            if (indexplus1 == BARO_TAB_SIZE_MAX) {
                indexplus1    = 0;
                bBaroSumReady = 1;
            }
            baroHistTab[baroHistIdx] = applyBarometerMedianFilter(baroPressure);
            baroPressureSum += baroHistTab[baroHistIdx];
            baroPressureSum -= baroHistTab[indexplus1];
            baroHistIdx = indexplus1;
        }
        if (bBaroSumReady)
            baroPressureAvg = (float)baroPressureSum / (BARO_TAB_SIZE_MAX - 1);
        else
            baroPressureAvg = baroPressure;
        //----------------------------------------------------------------------------------------------------

        // 计算当前高度
        if ((calibratingB > 0) || (!fc.flags.motorArmed && !fc.flags.motorIdle)) {
            if (calibratingB == BCALIBRATING_BARO_CYCLES) baroGroundPressure = baroPressureAvg * 10;
            baroGroundPressure -= baroGroundPressure / 10;
            baroGroundPressure += baroPressureAvg;
            baroGroundAltitude = (1.0f - powf((baroGroundPressure / 10.0f) / 101325.0f, 0.190295f)) * 44330000.0f; // in mm

            vel    = 0;
            accAlt = 0;
            if (calibratingB > 0) calibratingB--;
        }

        // calculates height from ground via baro readings
        // see: https://github.com/diydrones/ardupilot/blob/master/libraries/AP_Baro/AP_Baro.cpp#L140
        // BaroAlt_tmp = (1.0f - powf(((float)baroPressureAvg) / 101325.0f, 0.190295f)) * 4433000.0f; // in cm
        BaroAlt_tmp = (1.0f - powf(((float)baroPressureAvg) / 101325.0f, 0.190295f)) * 44330000.0f; // in mm
        BaroAlt_tmp -= baroGroundAltitude;
        BaroAlt = (float)BaroAlt * 0.7f + (float)BaroAlt_tmp * (1.0f - 0.7f); // additional LPF to reduce baro noise

        // 计算速度	mm/s
        baroVel = baroVel * 3.0f / 4.0f + (BaroAlt - lastBaroAlt) * 8;
        baroVel = constrain(baroVel, -2000, 3000); // constrain baro velocity +/- 5000mm/s
        //      baroVel = applyDeadband(baroVel, 100);                                     // to reduce noise near zero	 10cm/s			去温飘

        lastBaroAlt = BaroAlt;

        //      printf("%d,%d\n", BaroAlt,baroVel);
    }
}

//--------------------------------------------------------------------------------------------------------------
//                                定高(中环PID)
//--------------------------------------------------------------------------------------------------------------

void __attribute__((weak)) getEstimatedAltitude(void)
{
    static uint32_t previousT; // Used for update vel & altitude
    uint32_t currentT;
    float dt;
    float velAcc, accTemp;
    const float accVelScale = 9.80665f * 1000.0f / acc_1G; // mm/sec^2
    uint8_t axis;

    int32_t error;
    static float errorVelocityI = 0;
    static int32_t velErrorLast = 0;
    int32_t baroD;
    int16_t vel_tmp;

    tiltAngle = max(abs(angle[ROLL]), abs(angle[PITCH]));

    currentT  = micros16();
    dt        = (uint16_t)(currentT - previousT) * 1e-6;
    previousT = currentT;

    //-------------------------------------------------------------------------------------
    // Integrator - velocity, cm/sec^2
    for (axis = 0; axis < 3; axis++) {
        if (accSumZCount == 0) accSumZCount = 1;
        if (axis == 2) {
            velGlobal[axis]      = vel;
            distanceGlobal[axis] = accAlt;
        }

        accTemp = accSumZ[axis] / accSumZCount;
        velAcc  = (float)accTemp * accVelScale * accSumZTimer;
        distanceGlobal[axis] += (velAcc * 0.5f) * accSumZTimer + velGlobal[axis] * accSumZTimer; // integrate velocity to get distance (x= a/2 * t^2)
        velGlobal[axis] += velAcc;
        accSumZ[axis] = 0;

        if (axis == 2) {
            accAlt = distanceGlobal[axis];
            vel    = velGlobal[axis];
        }
    }
    accSumZCount = 0;
    accSumZTimer = 0;

    //    printf("%d,%d,%d,%d,%d,%d\n", (int)velGlobal[0], (int)velGlobal[1], (int)velGlobal[2], (int)distanceGlobal[0], (int)distanceGlobal[1], (int)distanceGlobal[2]);

    // Integrator - Altitude in cm
    // accAlt += (vel_acc * 0.5f) * dt + vel * dt;  // integrate velocity to get distance (x= a/2 * t^2)
    accAlt = accAlt * 0.965f + (float)BaroAlt * (1.0f - 0.965f); // complementary filter for altitude estimation (baro & acc)
    EstAlt = accAlt;

    //-------------------------------------------------------------------------------------
    // apply Complimentary Filter to keep the calculated velocity based on baro velocity (i.e. near real velocity).
    // By using CF it's possible to correct the drift of integrated accZ (velocity) without loosing the phase, i.e without delay
    vel = vel * 0.987f + baroVel * 0.013f;

    vel = constrain_float(vel, VELOCITY_SPEEDDOWN_MAX - 200, VELOCITY_SPEEDUP_MAX + 200);

    //----------------------------------------------------------------------------
    // 如果马达未解锁或者处于待机状态，所有状态归零
    if (!fc.flags.motorArmed) {
        fc.flags.bVelocityControl = 0;
        AltHold                   = EstAlt;

        errorVelocityI = 0;
        BaroPID        = 0;
        vel            = 0;

        velGlobal[0]      = 0;
        velGlobal[1]      = 0;
        velGlobal[2]      = 0;
        distanceGlobal[0] = 0;
        distanceGlobal[1] = 0;
        distanceGlobal[2] = 0;
    }

    //----------------------------------------------------------------------------
    vel_tmp = vel;

    // Part2
    if ((tiltAngle < 8000)) { // only calculate pid if the copters thrust is facing downwards(<80deg)
        // Altitude P-Controller
        if (!fc.flags.bVelocityControl) {
            Pf_Pos = 0.7f;
            Pf_Vel = 2.0f;
            If_Vel = 2.0f;
            Df_Vel = 4.0f;

            error  = constrain(AltHold - EstAlt, -5000, 5000);
            setVel = constrain(Pf_Pos * error, -3000, +3000); // limit velocity to +/- 3 m/s
        } else {
            Pf_Vel = 1.00f;
            If_Vel = 0.00f;
            Df_Vel = 2.0f;

            //					setVel = setVelocity;
            if (setVelocity == 0) {
                if (setVel > 800)
                    setVel = 800;
                else if (setVel < -800)
                    setVel = -800;
                else if (abs(setVel) < 200)
                    setVel = 0;
                else {
                    if (setVel > setVelocity) {
                        setVel -= 15;
                        if (setVel < setVelocity) setVel = setVelocity;
                    }
                    if (setVel < setVelocity) {
                        setVel += 15;
                        if (setVel > setVelocity) setVel = setVelocity;
                    }
                }
            } else {
                setVel = setVelocity;
            }

            if (setVel == 0) {
                accAlt  = BaroAlt;
                AltHold = accAlt;
                EstAlt  = accAlt;

                fc.flags.bVelocityControl = 0;
            }
        }

// Velocity PID-Controller
// P
#define ALTITUDE_PROPORTION_LIMITED 500
        error   = setVel - vel_tmp;
        BaroPID = constrain((Pf_Vel * error), -ALTITUDE_PROPORTION_LIMITED, +ALTITUDE_PROPORTION_LIMITED);

// I
#define ALTITUDE_INTEGRAL_LIMITED 400
        errorVelocityI += (If_Vel * error) * dt;
        if (errorVelocityI > ALTITUDE_INTEGRAL_LIMITED) errorVelocityI = ALTITUDE_INTEGRAL_LIMITED;
        if (errorVelocityI < -ALTITUDE_INTEGRAL_LIMITED) errorVelocityI = -ALTITUDE_INTEGRAL_LIMITED;
        BaroPID += errorVelocityI;

// D
#define ALTITUDE_DIFFERENTIAL_LIMITED 400
        baroD        = error - velErrorLast;
        velErrorLast = error;

        BaroPID += constrain(baroD * Df_Vel, -ALTITUDE_DIFFERENTIAL_LIMITED, ALTITUDE_DIFFERENTIAL_LIMITED);

        //        printf("%d,%d,%d,%d,%d\n", baroVel, vel_tmp, BaroAlt, EstAlt, AltHold);
        //        printf("%d,%d,%d,%d,%d\n", setVelocity, setVel, BaroAlt, EstAlt, AltHold);

    } else {
        fc.flags.motorArmed = 0;

        fc.flags.bVelocityControl = 0;
        AltHold                   = EstAlt;
        errorVelocityI            = 0;
        BaroPID                   = 0;
        vel                       = 0;

        error = 0;
    }
}

// 定高
void altitude_ctrl(void)
{
    altitude_baro_update();

    if (altitudeCtrlTimer >= 10) {
        altitudeCtrlTimer = 0;
        getEstimatedAltitude();
    }
}

void __attribute__((weak)) altitude_init(void)
{
    BaroThrottle = 1650;
}
