

#pragma once

#include "../platform.h" 

typedef struct {
  uint8_t* buf;
  uint16_t head;
  uint16_t tail;
  uint16_t length;
} fifo_s;

void fifo_reset(fifo_s* fifo);



