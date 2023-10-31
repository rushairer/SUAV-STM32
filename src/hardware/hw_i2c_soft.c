

#include "hw_i2c_soft.h"


void hw_soft_i2c_init(void)
{
//	GPIO_InitType GPIO_InitStructure;
//  
//  RCC_AHB_Peripheral_Clock_Enable(SOFT_I2C_SCL_GPIO_CLK);  
//  
//  /* Assign default value to GPIO_InitStructure structure */
//  GPIO_Structure_Initialize(&GPIO_InitStructure);
//  
//  /* Select the GPIO pin to control */
//  GPIO_InitStructure.Pin          = SOFT_I2C_SCL_PIN;
//  /* Set pin mode to general push-pull output */
//  GPIO_InitStructure.GPIO_Mode    = GPIO_MODE_OUT_PP;
//  /* Set the pin drive current to 4MA*/
//  GPIO_InitStructure.GPIO_Current = GPIO_DS_8MA;
//  /* Initialize GPIO */
//  GPIO_Peripheral_Initialize(SOFT_I2C_SCL_GPIO_PORT, &GPIO_InitStructure);
//  
//  RCC_AHB_Peripheral_Clock_Enable(SOFT_I2C_SDA_GPIO_CLK);  
//  GPIO_InitStructure.Pin          = SOFT_I2C_SDA_PIN;
//  GPIO_Peripheral_Initialize(SOFT_I2C_SDA_GPIO_PORT, &GPIO_InitStructure);
       
  GPIO_InitTypeDef  GPIO_InitStructure;
    
  RCC_APB2PeriphClockCmd(SOFT_I2C_SCL_GPIO_CLK, ENABLE);  
  GPIO_InitStructure.GPIO_Pin = SOFT_I2C_SCL_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(SOFT_I2C_SCL_GPIO_PORT, &GPIO_InitStructure);
      
  RCC_APB2PeriphClockCmd(SOFT_I2C_SDA_GPIO_CLK, ENABLE);  
  GPIO_InitStructure.GPIO_Pin = SOFT_I2C_SDA_PIN;
  GPIO_Init(SOFT_I2C_SDA_GPIO_PORT, &GPIO_InitStructure);
  
}

void hw_soft_i2c_sda_output(void)
{
//    GPIO_Mode_Set(SOFT_I2C_SDA_GPIO_PORT, GPIO_MODE_OUT_PP, 7);  // ** 最后一个参数是positon
  
  SOFT_I2C_SDA_GPIO_PORT->CRL &= 0x0FFFFFFF;
  SOFT_I2C_SDA_GPIO_PORT->CRL |= (u32)1<<28;
}
void hw_soft_i2c_sda_input(void)
{
//    GPIO_Mode_Set(SOFT_I2C_SDA_GPIO_PORT, GPIO_MODE_INPUT, 7);
  
  SOFT_I2C_SDA_GPIO_PORT->CRL &= 0x0FFFFFFF;
  SOFT_I2C_SDA_GPIO_PORT->CRL |= (u32)0x08<<28;
}




