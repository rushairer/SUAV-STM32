
#include "pid.h"
#include "fc.h"
#include "decode.h"
#include "../ahrs/accgyro.h"
#include "../ahrs/imu.h"
#include "../common/mlib.h"
#include "../motor/motor.h"
#include "../ahrs/altitude.h"
#include "../ahrs/opflow.h"

pid_conf_s pidconf;

void pid(float deltaT)
{							   
  int32_t target_angle, target_rate;	 	
									  
  static int32_t last_rate[3] = {0,0,0};  
  int32_t current_rate;   		                                 
  int32_t rate_error;  
  
  static float rate_i[3] = {0.0f, 0.0f, 0.0f}; 	
  static float delta1[3] = {0.0f, 0.0f, 0.0f}, delta2[3] = {0.0f, 0.0f, 0.0f};	  
  float delta, deltaSum;
  volatile int32_t p, i, d;            
  int16_t axisPID[3];	
                                              
  int16_t motor[4], maxMotor; 
  int8_t axis, index;	
			
  const float _imaxYaw   = 228.0f;			//360¡ã	
  const float _imaxPR    = 350.0f;				
  
	pidconf.PAngleToRate[ROLL] = 1.0f;   pidconf.PAngleToRate[PITCH] = 1.0f;   pidconf.PAngleToRate[YAW] = 0.3f;
	pidconf.PRate[ROLL] = 0.10f;         pidconf.PRate[PITCH] = 0.10f;         pidconf.PRate[YAW] = 0.3f;	
	pidconf.IRate[ROLL] = 0.25f;         pidconf.IRate[PITCH] = 0.25f;         pidconf.IRate[YAW] = 0.6f;  
	pidconf.DRate[ROLL] = 0.0012f;       pidconf.DRate[PITCH] = 0.0012f;       pidconf.DRate[YAW] = 0.000f;

    optGyro[ROLL]  += accgyro.gyroData[ROLL]  * deltaT * gyroScale;   // rad/s
    optGyro[PITCH] += accgyro.gyroData[PITCH] * deltaT * gyroScale;   

    angle[2] = 0; 
    for(axis = 0; axis < 3; axis++) {	
        //    target_angle = constrain(20*rcCommand[axis], -30000, +30000) + rcCommandTrim[axis] - angle[axis];		
        if(axis < 2) {
            if(rcCommand[axis] != 0) {
                target_angle = constrain(20*rcCommand[axis], -30000, +30000) + rcCommandTrim[axis] - angle[axis];	
            } else {
                target_angle = constrain(20*rcCommand[axis], -30000, +30000) + rcCommandTrim[axis] - angle[axis] + opticalflowPID[axis];		
            }
        } else {
            target_angle = constrain(20*rcCommand[axis], -30000, +30000) + rcCommandTrim[axis] - angle[axis];		
        }
    
		target_rate  = target_angle * pidconf.PAngleToRate[axis];  
				
        current_rate = accgyro.gyroData[axis];  

        rate_error = target_rate - current_rate;
        p = rate_error * pidconf.PRate[axis];

        rate_i[axis] += (rate_error * pidconf.IRate[axis]) * deltaT;   
        if(axis == 2){	   			
            if(rate_i[axis] < -_imaxYaw)     rate_i[axis] = -_imaxYaw;
            else if(rate_i[axis] > _imaxYaw) rate_i[axis] = _imaxYaw;
        } else {	
            if(rate_i[axis] < -_imaxPR)     rate_i[axis] = -_imaxPR;			  
            else if(rate_i[axis] > _imaxPR) rate_i[axis] = _imaxPR;	
        }    		
        i = rate_i[axis];

        delta = rate_error - last_rate[axis];
        last_rate[axis] = rate_error;
        delta /= deltaT;

        deltaSum     = delta1[axis] + delta2[axis] + delta;
        delta2[axis] = delta1[axis];
        delta1[axis] = delta; 
        d = deltaSum * pidconf.DRate[axis];

        axisPID[axis] = p + i + d;	
    }								 		   			
    
  #define PIDMIX(X,Y,Z) constrain(BaroThrottle+BaroPID, 1000, 2000) + axisPID[ROLL]*X + axisPID[PITCH]*Y + axisPID[YAW]*Z	
//  #define PIDMIX(X,Y,Z) constrain(rcCommand[THROTTLE], 1000, 2000) + axisPID[ROLL]*X + axisPID[PITCH]*Y + axisPID[YAW]*Z	
  
//  #define PIDMIX(X,Y,Z) constrain(1000 + 2*constrain(rcCommand[THROTTLE]-1500, 0, 500), 1000, 2000) + axisPID[ROLL]*X + axisPID[PITCH]*Y + axisPID[YAW]*Z	

  motor[0] = PIDMIX(-1,-1,-1); 
  motor[1] = PIDMIX(-1,+1,+1); 
  motor[2] = PIDMIX(+1,-1,+1); 
  motor[3] = PIDMIX(+1,+1,-1); 

  #define MAXTHROTTLE 	2000
  #define MINTHROTTLE		1100
  #define MOTORSTOPVAL	1000
	
	//-------------------------------------------------------------------------------------
  maxMotor = motor[0];
  for(axis=1; axis< 4; axis++) if(motor[axis] > maxMotor) maxMotor = motor[axis];
  for(axis = 0; axis < 4; axis++){
	  if(maxMotor > MAXTHROTTLE) motor[axis] -= maxMotor - MAXTHROTTLE;
    motor[axis] = constrain(motor[axis], MINTHROTTLE, MAXTHROTTLE);   	          
    
    if(!fc.flags.motorArmed || fc.flags.motorIdle){
      if(fc.flags.motorIdle) {
        motor[axis] = MINTHROTTLE;
      } else {
        motor[axis] = MOTORSTOPVAL;
      }
        	
      for(index=0; index<3; index++){
        rate_i[index] = 0;
        delta1[index] = 0;
        delta2[index] = 0;
        last_rate[index] = 0;
      }							
    }       
  }
	
	motorPWMValue[0] = (uint16_t)motor[0] - MOTORSTOPVAL; 	//0~1000   ÓÒÉÏ
	motorPWMValue[1] = (uint16_t)motor[2] - MOTORSTOPVAL;     // ×óÉÏ
	motorPWMValue[2] = (uint16_t)motor[3] - MOTORSTOPVAL;
	motorPWMValue[3] = (uint16_t)motor[1] - MOTORSTOPVAL;		
	
}





