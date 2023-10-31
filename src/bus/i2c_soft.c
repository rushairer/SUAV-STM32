

#include "i2c_soft.h"
#include "../hardware/hw_i2c_soft.h"
#include "../schedule/ticks.h"
#include <stdio.h>

uint8_t i2cTestTimer;


static inline void soft_i2c_delay(void)
{
  uint16_t cnt = 6;
  while(cnt--);
}

void soft_i2c_init(void)
{
  hw_soft_i2c_init();
  
  SOFT_I2C_SDA_HIGH;
  SOFT_I2C_SCL_HIGH;  
}


// 起始位：时钟高电平时，SDA由高变低
void soft_i2c_start(void)
{
	SOFT_I2C_SDA_HIGH;		
	soft_i2c_delay();
  
	SOFT_I2C_SCL_HIGH;	
	soft_i2c_delay();
	SOFT_I2C_SDA_LOW;
	soft_i2c_delay();
	
	SOFT_I2C_SCL_LOW;
}

// 停止位：时钟高电平时，SDA由低变高
void soft_i2c_stop(void)
{
	SOFT_I2C_SDA_LOW;		
  soft_i2c_delay();	
  
	SOFT_I2C_SCL_HIGH;	
	soft_i2c_delay();	
	SOFT_I2C_SDA_HIGH;	
}


// 写一字节数据到I2C
// @ 传参数据，返回ACK(收到ACK返回0，失败返回1)
uint8_t soft_i2c_write(uint8_t data) 
{
	uint8_t i = 8, ack;
	
	while(i--)
	{		
		if(data & (uint8_t)0x80) {
      SOFT_I2C_SDA_HIGH;
    } else {
      SOFT_I2C_SDA_LOW;
    }
		data <<= 1;
		soft_i2c_delay();
		SOFT_I2C_SCL_HIGH;		
		soft_i2c_delay();
		SOFT_I2C_SCL_LOW;
	}
	
	//检测ACK(0表示应答)
	hw_soft_i2c_sda_input();		//端口配置为输入
  
	soft_i2c_delay();
	SOFT_I2C_SCL_HIGH;
	soft_i2c_delay();
	ack = (uint8_t)SOFT_I2C_SDA_READ;
	SOFT_I2C_SCL_LOW;
  
	hw_soft_i2c_sda_output();		//恢复端口方向(输出)	
	
	return ack;
}

// 读一字节数据
// @ 返回数据，传参ACK或者NACK。如果还要读，ACK=0，不再接着读ACK=1。
// @ ACK即IO电平。
uint8_t soft_i2c_read(uint8_t ACK)
{	
	uint8_t i=8, data = 0;
	
	while(i--)
	{		
		data <<= 1;		
		soft_i2c_delay();
		SOFT_I2C_SCL_HIGH;
		soft_i2c_delay();			
		if(SOFT_I2C_SDA_READ) {
      data |= (uint8_t)0x01;
    }
		SOFT_I2C_SCL_LOW;
	}
	
	//Ack(0)应答
	hw_soft_i2c_sda_output();
  
	if(ACK) {
    SOFT_I2C_SDA_HIGH;
  } else {
    SOFT_I2C_SDA_LOW;
  }
	soft_i2c_delay();
	SOFT_I2C_SCL_HIGH;
	soft_i2c_delay();
	SOFT_I2C_SCL_LOW;
  
	hw_soft_i2c_sda_input();
	
	return data;
}

/*==================================================================================
				写寄存器
==================================================================================*/
uint8_t soft_i2c_write_reg(uint8_t devAddr, uint8_t regAddr, uint8_t regData)
{	
	uint8_t ack;
	
	soft_i2c_start();
	soft_i2c_write(devAddr| (uint8_t)0x00);	//Write 0
	soft_i2c_write(regAddr);
	
	ack = soft_i2c_write(regData);
	soft_i2c_stop();
	
	return ack;
}

/*==================================================================================
				读寄存器
==================================================================================*/
uint8_t soft_i2c_read_reg(uint8_t devAddr, uint8_t regAddr)
{	
	uint8_t data;
	
	soft_i2c_start();
	soft_i2c_write(devAddr | (uint8_t)0x00);		//Write 0
	soft_i2c_write(regAddr);
	
	soft_i2c_start();
	soft_i2c_write(devAddr | (uint8_t)0x01);    //Read 1
	   
	hw_soft_i2c_sda_input();	 
	data = soft_i2c_read(1);	
	hw_soft_i2c_sda_output();		
	soft_i2c_stop();
	
	return data;
}

/*==================================================================================
					连续读寄存器
==================================================================================*/
uint8_t soft_i2c_buffer_read(uint8_t devAddr, uint8_t regAddr, uint8_t* pBuffer, uint8_t numByteToRead)
{
	uint8_t ack;
	
	soft_i2c_start();
	ack = soft_i2c_write(devAddr | (uint8_t)0x00);       //Write 0
	if(ack) goto I2CBUFREADBAD1;	 
  ack = soft_i2c_write(regAddr);	
	if(ack) goto I2CBUFREADBAD2;
	
	soft_i2c_start();
	ack = soft_i2c_write(devAddr | (uint8_t)0x01);
	if(ack) goto I2CBUFREADBAD2;                            //Read 1
	   
	hw_soft_i2c_sda_input();	 
  /* While there is data to be read */
  while(numByteToRead)  
  {
    if(numByteToRead == 1){
      *pBuffer = soft_i2c_read(1);		
    }	else {
      *pBuffer = soft_i2c_read(0);
    }
		pBuffer++; 
		
		numByteToRead--;  
	}	 
	
	hw_soft_i2c_sda_output();
  I2CBUFREADBAD2:		
	soft_i2c_stop();	
  I2CBUFREADBAD1:

	return ack;
}


void i2c_test(void)
{      
  uint8_t productId;
  uint8_t ack;
  
  if(i2cTestTimer >= 10) {
    i2cTestTimer = 0;
    
    ack = !soft_i2c_buffer_read((0x68<<1), 0x0C, &productId, 1);
    if(!ack) {
      printf("i2c read error!\r\n");
    } else {
      printf("gyro product id is: %d\r\n", productId);
    }
  }
}


