

#pragma once

#include "../platform.h"


typedef enum {
  LED_FRONT_RIGHT = 1<<0,
  LED_FRONT_LEFT = 1<<1,
  LED_REAR_LEFT = 1<<2,
  LED_REAR_RIGHT = 1<<3,
  LED_ALL = 0x0f
} led_index_e;


typedef enum {
  LED_IDLE = 0,
  LED_GYRO_CALIBRATION,
  LED_FLIGHT,
  LED_STATUS_SUM
} led_status_e;

extern uint8_t ledCtrlTimer;

void led_on(led_index_e index);
void led_off(led_index_e index);
void led_reverse(led_index_e index);

void led_init(void);
void led_ctrl(led_status_e ledStatus);

void led_test_x(void);




