

#include "msp.h"
#include "../bus/uart.h"
#include "../iap/iap.h"
#include "../schedule/ticks.h"
#include "../fc/decode.h"
#include <string.h>
#include <stdio.h>

static uint8_t pack_index_last = 0;
static uint8_t pack_atleastonce = 0;
static uint32_t receive_cnt = 0;  

void msp(void)
{
	uint8_t i;
	uint8_t sum_check;
  uint8_t current_pack_length;
  
  uint8_t rollCnt = FC_CMD_LENGTH+5;
  
	while(serial_available() && rollCnt--) {    
		slot_timer = 0;
    
		if(receive_cnt <= 134) {
			RX_buf[receive_cnt++] = serial_read_rx_fifo();
			current_pack_length = RX_buf[3]+5;													
//			if((RX_buf[0] == 0x01)&&(RX_buf[1] == 0x01)&&(receive_cnt== current_pack_length)) {	   
			if((RX_buf[0] == 0x01)&&(receive_cnt == current_pack_length)) {	          																
        receive_cnt = 0;
        sum_check = 0;
        for(i = 0; i < current_pack_length -1 ; i++) {
          sum_check = sum_check + RX_buf[i];									
        }
        sum_check = ~sum_check + 1;
        if((sum_check == RX_buf[current_pack_length-1]))		
        {
          // 回发ACK，遥控指令除外
          if(RX_buf[1] != MSP_INDEX_CMD) {
            send_ack();	
          }
          
          if(!pack_atleastonce) {
            pack_atleastonce = 1;
            pack_index_last = RX_buf[2]-1;
          }
          if(RX_buf[2] != pack_index_last) {               
            pack_index_last = RX_buf[2];
            
            switch(RX_buf[1]) {
              case MSP_INDEX_IAP:
                iap_for_app(&RX_buf[4]);
                break;
              case MSP_INDEX_CMD:
                decode(&RX_buf[4]);
                break;
              default:
                break;              
            }            
          }
        } 
        memset(RX_buf, 0, sizeof(RX_buf));
			}
      
      if(receive_cnt == 1) {
        if(RX_buf[0] != 0x01) receive_cnt = 0;
      }
//      if(receive_cnt == 2) {
//        if(RX_buf[1] != 0x01) receive_cnt = 0;
//      }
		}	else {
      receive_cnt = 0;
    }      
  }
  
  if(slot_timer >= 200) {
    receive_cnt = 0;
  }
  
}




