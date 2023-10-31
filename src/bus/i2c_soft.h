


#pragma once     

//------------------------------------------------------------------------------------
#include "../platform.h" 


//------------------------------------------------------------------------------------
extern uint8_t i2cTestTimer;

//------------------------------------------------------------------------------------
void soft_i2c_init(void);
uint8_t soft_i2c_write_reg(uint8_t devAddr, uint8_t regAddr, uint8_t regData);
uint8_t soft_i2c_read_reg(uint8_t devAddr, uint8_t regAddr);
uint8_t soft_i2c_buffer_read(uint8_t devAddr, uint8_t regAddr, uint8_t* pBuffer, uint8_t numByteToRead);

void i2c_test(void);





