

#pragma once     

//------------------------------------------------------------------------------------
#include "../platform.h" 


//------------------------------------------------------------------------------------
#define SOFT_I2C_SCL_PIN                         GPIO_Pin_6
#define SOFT_I2C_SCL_GPIO_PORT                   GPIOB
#define SOFT_I2C_SCL_GPIO_CLK                    RCC_APB2Periph_GPIOB

#define SOFT_I2C_SCL_HIGH                        {SOFT_I2C_SCL_GPIO_PORT->BSRR = SOFT_I2C_SCL_PIN;}
#define SOFT_I2C_SCL_LOW                         {SOFT_I2C_SCL_GPIO_PORT->BRR = SOFT_I2C_SCL_PIN;}

#define SOFT_I2C_SDA_PIN                         GPIO_Pin_7
#define SOFT_I2C_SDA_GPIO_PORT                   GPIOB
#define SOFT_I2C_SDA_GPIO_CLK                    RCC_APB2Periph_GPIOB

#define SOFT_I2C_SDA_HIGH                        {SOFT_I2C_SDA_GPIO_PORT->BSRR = SOFT_I2C_SDA_PIN;}
#define SOFT_I2C_SDA_LOW                         {SOFT_I2C_SDA_GPIO_PORT->BRR = SOFT_I2C_SDA_PIN;}
#define SOFT_I2C_SDA_READ                        (SOFT_I2C_SDA_GPIO_PORT->IDR & SOFT_I2C_SDA_PIN)


//------------------------------------------------------------------------------------
void hw_soft_i2c_init(void);

void hw_soft_i2c_sda_output(void);
void hw_soft_i2c_sda_input(void);



