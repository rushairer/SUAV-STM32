
#include "led.h"
#include "../hardware/hw_led.h"


uint8_t ledCtrlTimer;


//----------------------------------------------------------------
//----------------------------------------------------------------
void led_on(led_index_e index)
{
  if(index & LED_FRONT_RIGHT){
    LED1_ON;
  }  
  if(index & LED_FRONT_LEFT){
    LED2_ON;
  }
  if(index & LED_REAR_LEFT){
    LED3_ON;
  }
  if(index & LED_REAR_RIGHT){
    LED4_ON;
  }
}

void led_off(led_index_e index)
{  
  if(index & LED_FRONT_RIGHT){
    LED1_OFF;
  }  
  if(index & LED_FRONT_LEFT){
    LED2_OFF;
  }
  if(index & LED_REAR_LEFT){
    LED3_OFF;
  }
  if(index & LED_REAR_RIGHT){
    LED4_OFF;
  }
}

void led_reverse(led_index_e index)
{  
  if(index & LED_FRONT_RIGHT){
    LED1_TOGGLE;
  }  
  if(index & LED_FRONT_LEFT){
    LED2_TOGGLE;
  }
  if(index & LED_REAR_LEFT){
    LED3_TOGGLE;
  }
  if(index & LED_REAR_RIGHT){
    LED4_TOGGLE;
  }
}


//----------------------------------------------------------------
//----------------------------------------------------------------
void led_init(void)
{
  hw_led_initialize();
  
  led_on(LED_ALL);
}

void led_ctrl(led_status_e ledStatus)
{
  static uint8_t ledCnt;
  
  switch(ledStatus) {
    case LED_GYRO_CALIBRATION:
      if(ledCtrlTimer >= 5) {
        ledCtrlTimer = 0;
        led_reverse(LED_ALL);
      }    
      break;
      
    case LED_FLIGHT:
      if(ledCtrlTimer >= 4) {
        ledCtrlTimer = 0;
        
        if(ledCnt++ == 0) {
          led_on(LED_ALL);
        } else if(ledCnt <= 11) {
          led_off((led_index_e)(LED_FRONT_RIGHT | LED_FRONT_LEFT));
          if(ledCnt==5 || ledCnt==7) {
            led_off((led_index_e)(LED_REAR_LEFT | LED_REAR_RIGHT));
          } else {
            led_on((led_index_e)(LED_REAR_LEFT | LED_REAR_RIGHT));
          }
        } else {
          ledCnt = 0;
        }        
      }    
      break;
      
    case LED_IDLE:
    default:
      led_on(LED_ALL);      
      break;
  } 
}


//----------------------------------------------------------------
//----------------------------------------------------------------
void led_test_x(void)
{
  static led_index_e ledindex = LED_FRONT_RIGHT;
  
  if(ledCtrlTimer >= 50) {
    ledCtrlTimer = 0;
    
    switch(ledindex) {
      case LED_FRONT_RIGHT:
        led_off(LED_ALL);
        led_on(LED_FRONT_RIGHT);
        ledindex = LED_FRONT_LEFT;
        break;
      case LED_FRONT_LEFT:
        led_off(LED_ALL);
        led_on(LED_FRONT_LEFT);
        ledindex = LED_REAR_LEFT;
        break;
      case LED_REAR_LEFT:
        led_off(LED_ALL);
        led_on(LED_REAR_LEFT);
        ledindex = LED_REAR_RIGHT;
        break;
      case LED_REAR_RIGHT:
      default:
        led_off(LED_ALL);
        led_on(LED_REAR_RIGHT);
        ledindex = LED_FRONT_RIGHT;
        break;
    }
    
  }
}



