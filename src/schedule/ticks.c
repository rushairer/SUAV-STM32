

#include "ticks.h"
#include "../hardware/hw_timer.h"

#include "../led/led.h"
#include "../iap/iap.h"
#include "../bus/i2c_soft.h"

#include "../ahrs/mpu6050.h"
#include "../ahrs/barometer.h"
#include "../ahrs/altitude.h"
#include "../ahrs/opflow.h"

#include "../fc/decode.h"
#include "../fc/fc.h"


static uint16_t delayTimer;

/** 系统滴答，每1m回调一次
  * 
  */
void ticks_callback(void)
{
    static uint8_t _cnt20ms;

    delayTimer++;
    slot_timer++;
    baroUpdateTimer++;
    altitudeCtrlTimer++;
    opticalflowCtrlTimer++;

    if(++_cnt20ms >= 20) {
        _cnt20ms = 0;
        ledCtrlTimer++;
        i2cTestTimer++;
        mpu6050TestTimer++;
        calibrationEnbTimer++;
        printTestTimer++;
        powerupTimer++;
        fc.takeofflandTimer++;
    }

}

void ticks_init(void)
{
  hw_tim6_init(ticks_callback);
  hw_tim4_init(); 
}

//----------------------------------------------------------------
//----------------------------------------------------------------
void delay_ms(uint16_t cnt)
{
  delayTimer = 0;
  while(delayTimer < cnt);
}

void ticks_stop(void)
{
//  TIM_Off(TIM6);
  TIM_Cmd(TIM2, DISABLE);
}

// 返回上电后微秒计时
uint16_t micros16(void)
{
  uint16_t mic;
  
//  do {
//    cur_tick = SysTick->VAL;
//    mic = sysTickMs*1000 + (SysTick->LOAD+1-cur_tick)/72;    
//  } while(cur_tick > SysTick->VAL);
  
  mic = TIM_GetCounter(TIM4);
  
  return mic;  
}

