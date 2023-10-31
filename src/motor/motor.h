
#pragma once

#include "../platform.h"


#define MOTOR_PWM_PERIOD              (1000-1)

extern uint16_t motorPWMValue[4];

void motor_init(void);
void motor_pwm_output(void);

void motor_pwm_output_duty25(void);
void motor_test_rc(void);


