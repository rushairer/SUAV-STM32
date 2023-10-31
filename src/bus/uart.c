
#include "uart.h"
#include "../hardware/hw_uart.h"

#include <string.h>
#include <stdio.h>


uart_ctrl_s debugUART;

#define DEBUG_UART_FIFO_LENGTH_RX    (256+10)
#define DEBUG_UART_FIFO_LENGTH_TX    128
static uint8_t debugUartRxbuf[DEBUG_UART_FIFO_LENGTH_RX], debugUartTxbuf[DEBUG_UART_FIFO_LENGTH_TX];


uart_ctrl_s opflowUART;

#define OPFLOW_UART_FIFO_LENGTH_RX    (64)
#define OPFLOW_UART_FIFO_LENGTH_TX    (64)
static uint8_t opflowUartRxbuf[OPFLOW_UART_FIFO_LENGTH_RX], opflowUartTxbuf[OPFLOW_UART_FIFO_LENGTH_TX];

//----------------------------------------------------------------
//----------------------------------------------------------------
/* retarget the C library printf function to the USART */
int fputc(int ch, FILE *f)
{
    serial_write_tx_fifo(ch);
    uart_send_data();
    return ch;
}

//----------------------------------------------------------------
//----------------------------------------------------------------
void debug_uart_init(void)
{  
  memset(&debugUART, 0, sizeof(debugUART));  
  debugUART.boudrate = 115200;
  debugUART.rxfifo.buf = debugUartRxbuf;
  debugUART.rxfifo.length = DEBUG_UART_FIFO_LENGTH_RX;  
  debugUART.txfifo.buf = debugUartTxbuf;
  debugUART.txfifo.length = DEBUG_UART_FIFO_LENGTH_TX;
  
  hwDebugUart.boudrate = debugUART.boudrate;
  hwDebugUart.fifoRx = &debugUART.rxfifo;
  hwDebugUart.fifoTx = &debugUART.txfifo;
  hw_debug_uart_init();    
}

//----------------------------------------------------------------
//----------------------------------------------------------------
void uart_send_data(void) {  
    MY_UART_ENABLE_TX_INT; 
}

uint8_t serial_read_rx_fifo(void) {
  uint16_t t = debugUART.rxfifo.tail;
  uint8_t c = debugUART.rxfifo.buf[t];
  if (t != debugUART.rxfifo.head) {
    if (++t >= debugUART.rxfifo.length) t = 0;
    debugUART.rxfifo.tail = t;
  }
  return c;
}
uint16_t serial_available(void) {	
  return ((uint16_t)(debugUART.rxfifo.head - debugUART.rxfifo.tail))%debugUART.rxfifo.length;
}
void serial_flush_rx_fifo(void)
{
  debugUART.rxfifo.head = debugUART.rxfifo.tail;
}


void serial_write_tx_fifo(uint8_t a) {
  uint16_t t = debugUART.txfifo.head; 	
  if(++t >= debugUART.txfifo.length) t = 0;	
	if(t != debugUART.txfifo.tail) {
    debugUART.txfifo.buf[debugUART.txfifo.head] = a;
    debugUART.txfifo.head = t;
	}
}
uint16_t serial_used_tx_fifo(void) {
  return ((uint16_t)(debugUART.txfifo.head - debugUART.txfifo.tail))%debugUART.txfifo.length;
}

void serial_write_buffer(uint8_t *buf, uint16_t length)
{
	uint16_t i;	
	for(i=0; i<length; i++) {
		serial_write_tx_fifo(buf[i]);
	}
	uart_send_data();
}

//----------------------------------------------------------------
//----------------------------------------------------------------
void uart_test_x(void)
{
  if(serial_available()) {
    serial_write_tx_fifo(serial_read_rx_fifo());
    uart_send_data();
  }
}


//------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------
void opflow_uart_init(void)
{  
  memset(&opflowUART, 0, sizeof(opflowUART));  
  opflowUART.boudrate = 19200;
  opflowUART.rxfifo.buf = opflowUartRxbuf;
  opflowUART.rxfifo.length = OPFLOW_UART_FIFO_LENGTH_RX;  
  opflowUART.txfifo.buf = opflowUartTxbuf;
  opflowUART.txfifo.length = OPFLOW_UART_FIFO_LENGTH_TX;
  
  hwOpflowUart.boudrate = opflowUART.boudrate;
  hwOpflowUart.fifoRx = &opflowUART.rxfifo;
  hwOpflowUart.fifoTx = &opflowUART.txfifo;
  hw_opticalflow_uart_init();    
}

uint8_t opflow_uart_read_rx_fifo(void) {
  uint16_t t = opflowUART.rxfifo.tail;
  uint8_t c = opflowUART.rxfifo.buf[t];
  if (t != opflowUART.rxfifo.head) {
    if (++t >= opflowUART.rxfifo.length) t = 0;
    opflowUART.rxfifo.tail = t;
  }
  return c;
}

uint16_t opflow_uart_available(void) {	
  return ((uint16_t)(opflowUART.rxfifo.head - opflowUART.rxfifo.tail))%opflowUART.rxfifo.length;
}





