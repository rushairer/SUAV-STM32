
#pragma once

#include "../platform.h"
#include "../common/fifo.h"

typedef struct {
  fifo_s rxfifo;
  fifo_s txfifo;
  
  uint32_t boudrate;
} uart_ctrl_s;


void debug_uart_init(void);

void uart_send_data(void);
uint8_t serial_read_rx_fifo(void);
uint16_t serial_available(void);
void serial_flush_rx_fifo(void);
void serial_write_tx_fifo(uint8_t a);
uint16_t serial_used_tx_fifo(void);

void serial_write_buffer(uint8_t *buf, uint16_t length);

void uart_test_x(void);

void opflow_uart_init(void);
uint8_t opflow_uart_read_rx_fifo(void);
uint16_t opflow_uart_available(void);



