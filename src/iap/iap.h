
#pragma once


#include "../platform.h"

typedef  void (*iapfun)(void);									  // 定义一个函数类型的参数.   	

#define FLASH_APP_BASE_ADDR 		0x08004000				// BOOTLOAD 预留16K空间，APP程序从0x08004000开始  						  
#define FLASH_START_ADDR        FLASH_APP_BASE_ADDR

// APP有效标志存放地址
// @ flash单次擦除2k，一个标志占用两k地址
#define APP_UPDATE_FLAG_ADDR (0x08004000-0x800)   // bootloader最后2k地址
//#define APP_UPDATE_FLAG_ADDR (0x08010000-0x800)   // bootloader最后2k地址

#define FLASH_APP_EFFECT_MAGIC_NUMBER 0x12345678


#define ACC_GYRO_DATA_OFFSET_ADDR_TEMP  (APP_UPDATE_FLAG_ADDR+16)

typedef struct {
  uint32_t good;
  int16_t acc_offset[3];
  int16_t gyro_offset[3];
  float temperature;
} config_accgyro_temp_s;

extern config_accgyro_temp_s cfgTemp;

extern uint8_t f_IAP_flashing;
extern uint8_t f_final_frame;
extern uint8_t f_receive_app_done;
extern uint16_t slot_timer;

extern uint8_t RX_buf[256];

uint32_t FLASH_ReadWord(uint32_t address);
int32_t app_flag_write(uint32_t data ,uint32_t start_add);
void IAP_UPDATE_APP(void);
int32_t app_flash_verification(uint32_t *data ,uint32_t Flash_address);

void iap_load_app(u32 appxaddr);								  // 跳转到APP程序执行
void iap_write_appbin(u32 appxaddr,u8 *appbuf,u32 applen);		//在指定地址开始,写入bin

void iap_uart_receive(void);
void iap_init(void);

void send_ack(void);
void iap_for_bootloader(void);

void iap_for_app_init(void);
void iap_for_app(uint8_t *buf);

void cfg_save_temp(void);

