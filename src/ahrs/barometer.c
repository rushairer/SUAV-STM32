
#include "barometer.h"
#include "spl06.h"
#include "../schedule/ticks.h"
#include <math.h>
#include <stdio.h>

uint8_t baroUpdateTimer;

baroDev_t baro;
int32_t baroPressure, baroTemperature;

static uint8_t baroSensorReady;

typedef enum {
	BARO_UPT_NEEDS_SAMPLES = 0,
	BARO_UPT_NEEDS_CALCULATION,	
	BARO_UPT_SUM
} BaroUpt_e;


void baro_init(void)
{
  if(spl06Detect(&baro)) baroSensorReady = true;
  else baroSensorReady = false;
  baroPressure = 0;
}

bool baro_update(void)
{   
  static uint8_t baroUpdateCyc = 0;
  static BaroUpt_e baroUptEnum = BARO_UPT_SUM;
  bool getPressureAndTempeture = false;
  
  if(baroSensorReady != true) return false;
  
  if(baroUpdateTimer >= baroUpdateCyc){			
			baroUpdateTimer = 0;
						
			switch(baroUptEnum){  
				case BARO_UPT_NEEDS_SAMPLES:
					baro.get_ut();
          baro.start_up();
          baroUptEnum = BARO_UPT_NEEDS_CALCULATION;
          baroUpdateCyc = baro.up_delay;
				  break;
				
				case BARO_UPT_NEEDS_CALCULATION:
          baro.get_up();
          baro.start_ut();
        
          baro.calculate(&baroPressure, &baroTemperature);
          baroUptEnum = BARO_UPT_NEEDS_SAMPLES;
          baroUpdateCyc = baro.ut_delay;
          getPressureAndTempeture = true;
          break;
				
        default:
          baro.start_ut();
          baroUptEnum = BARO_UPT_NEEDS_SAMPLES;
          baroUpdateCyc = baro.ut_delay;
          break;
			}			
  }	
  
  return getPressureAndTempeture;
}

void baro_test(void)
{
  static u8 _init = 0;
  static int32_t _baroGroundAltitude;
  int32_t _baroAlt;
  
  baro_update();
  if(printTestTimer > 0) {
    printTestTimer = 0;
    
    _baroAlt = (1.0f - powf(((float)baroPressure) / 101325.0f, 0.190295f)) * 4433000.0f; // in cm
    if(_init == 0) {
      if(baroPressure != 0) {
        _init = 1;
      }
      _baroGroundAltitude = _baroAlt;
    }
    
		_baroAlt -= _baroGroundAltitude;
    printf("%d\n", _baroAlt);
    
  }  
}


