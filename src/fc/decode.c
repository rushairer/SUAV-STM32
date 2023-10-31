

#include "decode.h"
#include "fc.h"
#include "../common/mlib.h"
#include "../ahrs/altitude.h"
#include "../ahrs/accgyro.h"
#include "../schedule/ticks.h"
#include <stdio.h>

uint8_t calibrationEnbTimer;

uint8_t fcCmd[FC_CMD_LENGTH];
int16_t rcCommand[4] = {0}, rcCommandTrim[3] = {0};

int16_t setVelocity = 0;

void decode(uint8_t* cmd)
{
  fcCmd[FC_CMD_ROLL]        = cmd[0];
  fcCmd[FC_CMD_PITCH]       = cmd[1];
  fcCmd[FC_CMD_THROTTLE]    = cmd[2];
  fcCmd[FC_CMD_YAW]         = cmd[3];
    
  fcCmd[FC_CMD_ROLL_TRIM]   = cmd[4];
  fcCmd[FC_CMD_PITCH_TRIM]  = cmd[5];
  fcCmd[FC_CMD_THROTTLE_TRIM] = cmd[6];
  fcCmd[FC_CMD_YAW_TRIM]    = cmd[7];
  
  fcCmd[FC_CMD_FLAGS_1] = cmd[8];
  fcCmd[FC_CMD_FLAGS_2] = cmd[9];
  
  fc.flags.uartCmdNew = 1;
}

void annexCode(void)
{
    int16_t codeRoll, codePitch, codeGas, codeYaw;
    int16_t codeRollTrim, codePitchTrim;
    static uint8_t armCheckEnb = 0;

    if(fc.flags.uartCmdNew) {	 	
        fc.flags.uartCmdNew= 0;	

        //指令解析
        //调整后的指令方向:
        //  1)左右 CodeRoll[rcCommand-Roll],  左小右大
        //  2)前后 CodePitch[rcCommand-Pitch],后小前大
        codeGas     = fcCmd[FC_CMD_THROTTLE];
        codeRoll    = fcCmd[FC_CMD_ROLL];
        codePitch   = fcCmd[FC_CMD_PITCH];
        codeYaw     = fcCmd[FC_CMD_YAW];
        codeRollTrim = fcCmd[FC_CMD_ROLL_TRIM];
        codePitchTrim = fcCmd[FC_CMD_PITCH_TRIM];

        codeGas   = constrain(codeGas*4, 0, 1000);
        codeRoll  -= STICK_ENCODE_LIMIT_MIDDLE;
        codePitch -= STICK_ENCODE_LIMIT_MIDDLE;
        codeYaw   -= STICK_ENCODE_LIMIT_MIDDLE;
        codeRollTrim  -= STICK_TRIM_LIMIT_RANGE;
        codePitchTrim -= STICK_TRIM_LIMIT_RANGE;

        codeRoll = applyDeadband(codeRoll, 28);
        codePitch = applyDeadband(codePitch, 28);
        codeYaw = applyDeadband(codeYaw, 37);    		

        #define VEL_CON_STICK_LIMIT_UP    650
        #define VEL_CON_STICK_LIMIT_DOWN  375    
        if((codeGas > VEL_CON_STICK_LIMIT_UP)) {
            fc.flags.bVelocityControl = 1;	
            setVelocity = (codeGas - VEL_CON_STICK_LIMIT_UP) * 5 / 2; 
            if(setVelocity > VELOCITY_SPEEDUP_MAX) setVelocity = VELOCITY_SPEEDUP_MAX;	
            fc.flags.motorIdle = 0;
            fc.flags.bTakeoff = 0;
            fc.flags.bLanding = 0;
        } else if(codeGas < VEL_CON_STICK_LIMIT_DOWN) {
            fc.flags.bVelocityControl = 1;						
            setVelocity = (codeGas - VEL_CON_STICK_LIMIT_DOWN) * 3 / 2;
            if(setVelocity < VELOCITY_SPEEDDOWN_MAX) setVelocity = VELOCITY_SPEEDDOWN_MAX;	 
            fc.flags.bTakeoff = 0;
            fc.flags.bLanding = 0;   
            if(fc.flags.motorIdle) {
              fc.flags.motorArmed = 0;
              fc.flags.motorIdle = 0;
            }   
        } else {	
            setVelocity = 0;
        }  

        //马达第一次运转前,方向杆至右下角开始传感器矫正		
        if(fc.flags.bAccGyroCalibrationEnb){	     
            if((codePitch <= -40) && (codeRoll >= 40)){
                if(calibrationEnbTimer < 50) goto ANEDXCODE_CALIBRATIONBK;
                fc.flags.bAccGyroCalibrationEnb = 0;
                accgyro.gyroCalibrating = 1;
                accgyro.accCalibrating = 1;         
            }	else {
                calibrationEnbTimer = 0;
            }
        }
        ANEDXCODE_CALIBRATIONBK:

        // 解锁按键按下
        if(fcCmd[FC_CMD_FLAGS_1] & 0x01) { 
            if(armCheckEnb) {
                armCheckEnb = 0;
                
                if(fc.flags.bLanding) {
                    fc.flags.motorArmed = 0;
                    fc.flags.motorIdle = 0;
                    fc.flags.bLanding = 0;
                } else {                
                    if(!fc.flags.motorArmed) {
                        fc.flags.motorArmed = 1;
                        fc.flags.motorIdle = 1;
                        fc.flags.bAccGyroCalibrationEnb = 0;
                        
                        fc.flags.bTakeoff = 1;
                        fc.takeofflandTimer = 0;
                        fc.flags.bLanding = 0;
                        fc.takeoffAlt = EstAlt;
                    } else if(!fc.flags.motorIdle){
                        fc.flags.bLanding = 1;
                        fc.takeofflandTimer = 0;       
                        fc.flags.bTakeoff = 0;
                    }
                }
            }
        } else {
            armCheckEnb = 1;
        }    
        
        // 急停
        if(fcCmd[FC_CMD_FLAGS_1] & 0x02) {
            fc.flags.motorArmed = 0;
            fc.flags.motorIdle = 0;
        }

        // 一键起飞
        if(fc.flags.bTakeoff) {
//            if(fc.flags.motorIdle) {
//                if(fc.takeofflandTimer >= 50) {
//                    fc.flags.motorIdle = 0;
//                    fc.takeoffAlt = EstAlt;
//                    fc.takeofflandTimer = 0;
//                }
//            }     
            
            if(!fc.flags.motorIdle) {       
                fc.flags.bVelocityControl = 1;            
                setVelocity = 200 + fc.takeofflandTimer*10; 
                if(setVelocity >= 800) setVelocity = 800;
                
                if((EstAlt-fc.takeoffAlt >= 1000) || fc.takeofflandTimer >= 150) {
                    fc.flags.bTakeoff = 0;
                }
            }
        }
        
        // 一键降落
        if(fc.flags.bLanding) {            
            fc.flags.bVelocityControl = 1;	
            setVelocity = -200 - fc.takeofflandTimer*10; 
            if(setVelocity < VELOCITY_SPEEDDOWN_MAX) setVelocity = VELOCITY_SPEEDDOWN_MAX;
        }        
        
        rcCommand[THROTTLE]   = constrain((codeGas + 1000), +1000, +2000);	      //1000 ~ 2000

        rcCommand[ROLL]       = constrain(codeRoll,   -500, +500);	  						//-500 ~ +500
        rcCommand[PITCH]      = constrain(codePitch,  -500, +500);
        rcCommand[YAW]        = constrain(-3*codeYaw, -500, +500);

        rcCommandTrim[ROLL]   = codeRollTrim * 10;  
        rcCommandTrim[PITCH]  = codePitchTrim * 10; 
        rcCommandTrim[YAW]    = 0;

    } //if(f.bRxRedayCmd)

}

void decode_test(void)
{ 
  if(printTestTimer >= 10) {
    printTestTimer = 0;
    printf("ROLL:%d, PITCH:%d, THRO:%d, YAW:%d, TrimRoll:%d, TrimPitch:%d\r\n", rcCommand[ROLL], rcCommand[PITCH],fcCmd[FC_CMD_THROTTLE],rcCommand[YAW], rcCommandTrim[ROLL], rcCommandTrim[PITCH]);  
  }  
}




