

#include "iap.h"
#include <string.h>
#include <stdio.h>

#include "../bus/uart.h"
#include "../led/led.h"
#include "../schedule/ticks.h"
#include "../hardware/hw_uart.h"

config_accgyro_temp_s cfgTemp;

uint8_t RX_buf[256]     = {0};
uint8_t flash_buf[2048] = {0};

uint8_t f_IAP_flashing;
uint8_t f_final_frame;
uint8_t f_receive_app_done;

uint16_t slot_timer;

static uint8_t pack_index_last  = 0;
static uint8_t pack_atleastonce = 0;
static uint32_t receive_cnt     = 0;
static uint16_t rx_nunber       = 0;

void send_ack(void)
{
    uint8_t data_buf[6];
    //
    data_buf[0] = RX_buf[0];
    data_buf[1] = RX_buf[1];
    data_buf[2] = RX_buf[2];
    data_buf[3] = 0;
    data_buf[4] = 0x100 - (RX_buf[0] + RX_buf[1] + RX_buf[2]);

    serial_write_buffer(data_buf, 5);
}
void iap_uart_receive(void)
{
    uint8_t i;
    uint8_t sum_check;
    uint8_t current_pack_length;

    if (serial_available()) {
        slot_timer = 0;

        if (receive_cnt <= 134) {
            RX_buf[receive_cnt++] = serial_read_rx_fifo();
            current_pack_length   = RX_buf[3] + 5;
            if ((RX_buf[0] == 0x01) && (RX_buf[1] == 0x01) && (receive_cnt == current_pack_length)) {
                receive_cnt = 0;
                sum_check   = 0;
                for (i = 0; i < current_pack_length - 1; i++) {
                    sum_check = sum_check + RX_buf[i];
                }
                sum_check = ~sum_check + 1;
                if ((sum_check == RX_buf[current_pack_length - 1]) && (f_IAP_flashing == 0)) {
                    send_ack();
                    delay_ms(10);

                    if (!pack_atleastonce) {
                        pack_atleastonce = 1;
                        pack_index_last  = RX_buf[2] - 1;
                    }
                    if (RX_buf[2] != pack_index_last) {
                        pack_index_last = RX_buf[2];

                        memcpy(&flash_buf[rx_nunber * 128], &RX_buf[4], current_pack_length - 5);
                        rx_nunber++;
                        if (rx_nunber >= 16) {
                            rx_nunber      = 0;
                            f_IAP_flashing = 1;
                        }

                        if ((current_pack_length == 5) && (RX_buf[3] == 0)) {
                            rx_nunber      = 0;
                            f_IAP_flashing = 1;
                            f_final_frame  = 1;
                        }
                    }
                }
                memset(RX_buf, 0x00, sizeof(RX_buf));
            }

            if (receive_cnt == 1) {
                if (RX_buf[0] != 0x01) receive_cnt = 0;
            }
            if (receive_cnt == 2) {
                if (RX_buf[1] != 0x01) receive_cnt = 0;
            }
        } else {
            receive_cnt = 0;
        }
    }

    if (slot_timer >= 200) {
        receive_cnt = 0;
    }
}

iapfun jump2app;
uint32_t pages_number = 0;
uint32_t ready_write_addr;

/**================================================================
        APP 跳转
        appxaddr:用户代码起始地址.
================================================================*/
void iap_load_app(u32 appxaddr)
{
    if (((*(vu32 *)appxaddr) & 0x0FFFFFFF) < 1024 * 64) // 检查栈顶地址是否合法.最大64k Flash
    {
        jump2app = (iapfun) * (vu32 *)(appxaddr + 4);
        // 初始化堆栈指针
        __set_MSP(*(__IO uint32_t *)appxaddr);
        //  __set_MSP(*(__IO uint32_t*) appxaddr);
        jump2app(); // 跳转到APP.
    }
}
/**================================================================
================================================================*/
int32_t app_flag_write(uint32_t data, uint32_t start_add)
{
    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

    FLASH_ErasePage(start_add); // 写之前先擦一遍，每次擦2K/1k
    if (FLASH_COMPLETE != FLASH_ProgramWord(start_add, data)) {
        FLASH_Lock();
        // printf("flash write fail! \r\n");
        return 1;
    }
    FLASH_Lock();
    return 0;
}

uint32_t FLASH_ReadWord(uint32_t address)
{
    return *(__IO uint32_t *)address;
}

/**================================================================
================================================================*/
// #define FLASH_PAGE_SIZE 		2048
#define FLASH_CODE_SIZE 2048

#if defined(STM32F10X_HD) || defined(STM32F10X_HD_VL) || defined(STM32F10X_CL) || defined(STM32F10X_XL)
#define FLASH_PAGE_SIZE ((uint16_t)0x800)
#else
#define FLASH_PAGE_SIZE ((uint16_t)0x400)
#endif

/**
 * @brief
 * @param void
 * @return
 * - `SUCCESS： 表示操作成功
 * - 其它值表示出错
 */
int32_t app_flash_write(uint32_t *data, uint32_t Flash_address)
{
    uint32_t i;
    uint32_t start_add;
    start_add = Flash_address;

    FLASH_Unlock();

    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

    for (i = 0; i < FLASH_CODE_SIZE / FLASH_PAGE_SIZE; i++) {
        //      FLASH_One_Page_Erase(start_add+i*FLASH_PAGE_SIZE);
        FLASH_ErasePage(start_add + (FLASH_PAGE_SIZE * i));
    }

    for (i = 0; i < FLASH_CODE_SIZE / 4; i++) {
        if (FLASH_COMPLETE != FLASH_ProgramWord(start_add + i * 4, data[i])) {
            FLASH_Lock();
            //      printf("flash write fail! \r\n");
            //      pages_number = 0;
            return 1;
        }
    }

    FLASH_Lock();
    return 0;
}

/**================================================================
        //升级APP
================================================================*/
void IAP_UPDATE_APP(void)
{
    ready_write_addr = FLASH_APP_BASE_ADDR + pages_number * 2048;
    while (app_flash_write((uint32_t *)flash_buf, ready_write_addr))
        ; // IAP每次升级2K
    memset(flash_buf, 0x00, 2048);
    pages_number++;
}

void iap_init(void)
{
    f_IAP_flashing     = 0;
    f_final_frame      = 0;
    f_receive_app_done = 0;
    pages_number       = 0;
    slot_timer         = 0;

    pack_index_last  = 0;
    pack_atleastonce = 0;
    receive_cnt      = 0;
    rx_nunber        = 0;
}

// void iap_for_bootloader(void)
//{
//   if(FLASH_ReadWord(APP_UPDATE_FLAG_ADDR) == FLASH_APP_EFFECT_MAGIC_NUMBER) {
//     f_receive_app_done = 1;
//	}	else {
//     printf("start suav bootloader! \r\n");
//   }
//
//	while(1)
//	{
//     iap_uart_receive();
//
//		if(f_receive_app_done == 0)
//		{
//			if(f_IAP_flashing == 1) {
//				f_IAP_flashing = 0;
//         IAP_UPDATE_APP();
//				if(f_final_frame == 1) {
//					f_final_frame = 0;
//					f_receive_app_done = 1;
//					app_flag_write(FLASH_APP_EFFECT_MAGIC_NUMBER ,APP_UPDATE_FLAG_ADDR);
//				}
//			}
//		}
//
//		if(f_receive_app_done)
//     {
//			printf("suav address：%x\r\n",(FLASH_START_ADDR));
//			printf("开始执行SUAV代码!!\r\n");

//      delay_ms(200);
//      USART_Disable(DEBUG_USARTx);
//      ticks_stop();
//
////      delay_ms(1000);                     ***** delay 需要中断支持，tick关闭后不可以再调用中断。这个delay浪费好几个小时查bug！！！！！
//
//			iap_load_app(FLASH_START_ADDR);				// 跳转到APP起始地址，期间不能被其他中断打断，否则会跳转失败
//		}
//
//    led_test_x();
//  }
//}

void cfg_init_temp(void)
{
    uint32_t i;
    uint32_t *p = (uint32_t *)&cfgTemp;

    for (i = 0; i < sizeof(cfgTemp); i += 4) {
        *p++ = FLASH_ReadWord(ACC_GYRO_DATA_OFFSET_ADDR_TEMP + i);
    }
}
void cfg_save_temp(void)
{
    uint32_t i;
    uint32_t *p = (uint32_t *)&cfgTemp;

    USART_Cmd(DEBUG_USARTx, DISABLE);

    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
    FLASH_ErasePage(APP_UPDATE_FLAG_ADDR);

    FLASH_ProgramWord(APP_UPDATE_FLAG_ADDR, FLASH_APP_EFFECT_MAGIC_NUMBER);

    for (i = 0; i < sizeof(cfgTemp); i += 4) {
        if (FLASH_COMPLETE != FLASH_ProgramWord(ACC_GYRO_DATA_OFFSET_ADDR_TEMP + i, p[i / 4])) {
            FLASH_Lock();
            break;
        }
    }
    FLASH_Lock();

    USART_Cmd(DEBUG_USARTx, ENABLE);
}

void iap_for_app_init(void)
{
    if (FLASH_ReadWord(APP_UPDATE_FLAG_ADDR) != FLASH_APP_EFFECT_MAGIC_NUMBER) {
        app_flag_write(FLASH_APP_EFFECT_MAGIC_NUMBER, APP_UPDATE_FLAG_ADDR);
    }
    memset(RX_buf, 0, sizeof(RX_buf));

    cfg_init_temp();
}

void iap_for_app(uint8_t *buf)
{
    // 72 65 62 6f 6f 74
    if (buf[0] == 'r') { // 0x72 0x65 0x62 0x6f 0x6f 0x74
        if (buf[1] == 'e') {
            if (buf[2] == 'b') {
                if (buf[3] == 'o') {
                    if (buf[4] == 'o') {
                        if (buf[5] == 't') {
                            delay_ms(100);
                            send_ack();
                            delay_ms(100);
                            send_ack();
                            delay_ms(100);
                            printf("systemreset to bootloader!\r\n");
                            delay_ms(100);
                            USART_Cmd(DEBUG_USARTx, DISABLE);
                            ticks_stop();
                            app_flag_write(0xffffffff, APP_UPDATE_FLAG_ADDR);
                            NVIC_SystemReset();
                        }
                    }
                }
            }
        }
    }
}
