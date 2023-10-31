
#pragma once

#include "../platform.h"


typedef struct {	 
  float PAngleToRate[3];
  float IAngleToRate[3];
  float PRate[3];
  float IRate[3];
  float DRate[3];
  int16_t angleTrim[3];
} pid_conf_s; 

extern pid_conf_s pidconf;

void pid(float deltaT);

