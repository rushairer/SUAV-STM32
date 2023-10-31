

#include "hw_led.h"

/**
 *\*\name   LED_Initialize.
 *\*\fun    Initialize the specified LED.
 *\*\param  GPIOx :
 *\*\          - GPIOA
 *\*\          - GPIOB
 *\*\          - GPIOC
 *\*\          - GPIOD
 *\*\param  pin :
 *\*\          - GPIO_PIN_0
 *\*\          - GPIO_PIN_1
 *\*\          - GPIO_PIN_2
 *\*\          - GPIO_PIN_3
 *\*\          - GPIO_PIN_4
 *\*\          - GPIO_PIN_5
 *\*\          - GPIO_PIN_6
 *\*\          - GPIO_PIN_7
 *\*\          - GPIO_PIN_8
 *\*\          - GPIO_PIN_9
 *\*\          - GPIO_PIN_10
 *\*\          - GPIO_PIN_11
 *\*\          - GPIO_PIN_12
 *\*\          - GPIO_PIN_13
 *\*\          - GPIO_PIN_14
 *\*\          - GPIO_PIN_15
 *\*\return  none.
**/
void hw_led_initialize(void)
{
//	GPIO_InitType GPIO_InitStructure;
//  
//  RCC_AHB_Peripheral_Clock_Enable(LED1_GPIO_CLK);  
//  
//  /* Assign default value to GPIO_InitStructure structure */
//  GPIO_Structure_Initialize(&GPIO_InitStructure);
//  
//  /* Select the GPIO pin to control */
//  GPIO_InitStructure.Pin          = LED1_GPIO_PIN;
//  /* Set pin mode to general push-pull output */
//  GPIO_InitStructure.GPIO_Mode    = GPIO_MODE_OUT_PP;
//  /* Set the pin drive current to 4MA*/
//  GPIO_InitStructure.GPIO_Current = GPIO_DS_4MA;
//  /* Initialize GPIO */
//  GPIO_Peripheral_Initialize(LED1_GPIO_PORT, &GPIO_InitStructure);
//  
//  RCC_AHB_Peripheral_Clock_Enable(LED2_GPIO_CLK);  
//  GPIO_InitStructure.Pin          = LED2_GPIO_PIN;
//  GPIO_Peripheral_Initialize(LED2_GPIO_PORT, &GPIO_InitStructure);
//  
//  RCC_AHB_Peripheral_Clock_Enable(LED3_GPIO_CLK);  
//  GPIO_InitStructure.Pin          = LED3_GPIO_PIN;
//  GPIO_Peripheral_Initialize(LED3_GPIO_PORT, &GPIO_InitStructure);
//  
//  RCC_AHB_Peripheral_Clock_Enable(LED4_GPIO_CLK);  
//  GPIO_InitStructure.Pin          = LED4_GPIO_PIN;
//  GPIO_Peripheral_Initialize(LED4_GPIO_PORT, &GPIO_InitStructure);
  
    
  GPIO_InitTypeDef  GPIO_InitStructure;
  
//  GPIO_StructInit(&GPIO_InitStructure);
//  GPIO_DeInit(LED1_GPIO_PORT);
  
  /* Enable the GPIO_LED Clock */
  RCC_APB2PeriphClockCmd(LED1_GPIO_CLK, ENABLE);
  
  /* Configure the GPIO_LED pin */
  GPIO_InitStructure.GPIO_Pin = LED1_GPIO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(LED1_GPIO_PORT, &GPIO_InitStructure);
    
  RCC_APB2PeriphClockCmd(LED2_GPIO_CLK, ENABLE);
  GPIO_InitStructure.GPIO_Pin = LED2_GPIO_PIN;
  GPIO_Init(LED2_GPIO_PORT, &GPIO_InitStructure);
  
  RCC_APB2PeriphClockCmd(LED3_GPIO_CLK, ENABLE);
  GPIO_InitStructure.GPIO_Pin = LED3_GPIO_PIN;
  GPIO_Init(LED3_GPIO_PORT, &GPIO_InitStructure);
  
  RCC_APB2PeriphClockCmd(LED4_GPIO_CLK, ENABLE);
  GPIO_InitStructure.GPIO_Pin = LED4_GPIO_PIN;
  GPIO_Init(LED4_GPIO_PORT, &GPIO_InitStructure);
    
}


