
#pragma once

#include "../platform.h"
#include "barometer.h"
#include "../common/mlib.h"
#include "../fc/fc.h"
#include "../fc/decode.h"
#include "../ahrs/imu.h"
#include "../schedule/ticks.h"
#include "accgyro.h"
#include <math.h>
#include <stdio.h>

#define VELOCITY_SPEEDUP_MAX   +1000
#define VELOCITY_SPEEDDOWN_MAX -700



#define BCALIBRATING_BARO_CYCLES      50
  
//Baro => BaroAlt baroVel
#define BARO_TAB_SIZE_MAX                    7//21 //15  //11 //21
#define PRESSURE_SAMPLES_MEDIAN              5//3  //5   //7  //3

extern int16_t tiltAngle;	
extern uint16_t calibratingB; 
extern int32_t baroGroundAltitude;

extern int32_t BaroAlt, lastBaroAlt;
extern int32_t baroVel;

//Acc => vel accAlt(EstAlt=accAlt)  & EstAlt
extern int32_t  accSumZ[3]; 
extern uint16_t accSumZCount; 
extern float    accSumZTimer;
extern int16_t  accZGlobal;
extern float    vel;
extern float    accAlt;   
extern int32_t  EstAlt;                                // in mm  

extern float velGlobal[3], distanceGlobal[3];

extern int32_t AltHold;	
extern int32_t BaroPID;
extern int16_t BaroThrottle;

//øÿ÷∆±‰¡ø
extern uint8_t altitudeCtrlTimer;
extern int16_t setVel;
		
extern float Pf_Pos;      
extern float Pf_Vel;        
extern float If_Vel;            
extern float Df_Vel;  		
			
extern int32_t integralBaroThrottle;					
		
    
void getEstimatedAltitude(void);
void altitude_ctrl(void);
void altitude_init(void);

