

#include "motor.h"
#include "../hardware/hw_timer.h"
#include "../fc/decode.h"
#include "../fc/fc.h"
#include "../common/mlib.h"

uint16_t motorPWMValue[4];


void motor_init(void)
{
  hw_tim3_init();  
  
  motorPWMValue[0] = 0;
  motorPWMValue[1] = 0;
  motorPWMValue[2] = 0;
  motorPWMValue[3] = 0;
}

void motor_pwm_output(void)
{
  TIM_SetCompare1(TIM3, motorPWMValue[0]);
  TIM_SetCompare2(TIM3, motorPWMValue[1]);
  TIM_SetCompare3(TIM3, motorPWMValue[2]);
  TIM_SetCompare4(TIM3, motorPWMValue[3]);
}


void motor_pwm_output_duty25(void)
{
  motorPWMValue[0] = MOTOR_PWM_PERIOD / 4;
  motorPWMValue[1] = MOTOR_PWM_PERIOD / 4;
  motorPWMValue[2] = MOTOR_PWM_PERIOD / 4;
  motorPWMValue[3] = MOTOR_PWM_PERIOD / 4;
  
  motor_pwm_output();  
}

void motor_test_rc(void)
{
  motorPWMValue[0] = abs(rcCommand[PITCH])*8;
  motorPWMValue[1] = 0;
  motorPWMValue[2] = 0;
  motorPWMValue[3] = 0;
  
  motor_pwm_output();  
}


