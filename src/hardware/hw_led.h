

#pragma once

#include "../platform.h"

//#define LED1_GPIO_PORT      GPIOB			            /* GPIO port */
//#define LED1_GPIO_CLK       RCC_AHB_PERIPH_GPIOB		/* GPIO port clock */
//#define LED1_GPIO_PIN       GPIO_PIN_3			        /* GPIO connected to the SCL clock line */

//#define LED2_GPIO_PORT      GPIOB			            /* GPIO port */
//#define LED2_GPIO_CLK       RCC_AHB_PERIPH_GPIOB		/* GPIO port clock */
//#define LED2_GPIO_PIN       GPIO_PIN_13			        /* GPIO connected to the SCL clock line */

//#define LED3_GPIO_PORT      GPIOB			            /* GPIO port */
//#define LED3_GPIO_CLK       RCC_AHB_PERIPH_GPIOB		/* GPIO port clock */
//#define LED3_GPIO_PIN       GPIO_PIN_14			        /* GPIO connected to the SCL clock line */

//#define LED4_GPIO_PORT      GPIOB			            /* GPIO port */
//#define LED4_GPIO_CLK       RCC_AHB_PERIPH_GPIOB		/* GPIO port clock */
//#define LED4_GPIO_PIN       GPIO_PIN_8			        /* GPIO connected to the SCL clock line */

///** Define macros that control IO **/
//#define LED1_TOGGLE         {LED1_GPIO_PORT->POD ^= LED1_GPIO_PIN;}
//#define LED1_ON             {LED1_GPIO_PORT->PBC = LED1_GPIO_PIN;}      // BIT CLEAR  Êä³ö0
//#define LED1_OFF            {LED1_GPIO_PORT->PBSC = LED1_GPIO_PIN;}

//#define LED2_TOGGLE         {LED2_GPIO_PORT->POD ^= LED2_GPIO_PIN;}
//#define LED2_ON             {LED2_GPIO_PORT->PBC = LED2_GPIO_PIN;}
//#define LED2_OFF            {LED2_GPIO_PORT->PBSC = LED2_GPIO_PIN;}

//#define LED3_TOGGLE         {LED3_GPIO_PORT->POD ^= LED3_GPIO_PIN;}
//#define LED3_ON             {LED3_GPIO_PORT->PBC = LED3_GPIO_PIN;}
//#define LED3_OFF            {LED3_GPIO_PORT->PBSC = LED3_GPIO_PIN;}

//#define LED4_TOGGLE         {LED4_GPIO_PORT->POD ^= LED4_GPIO_PIN;}
//#define LED4_ON             {LED4_GPIO_PORT->PBC = LED4_GPIO_PIN;}
//#define LED4_OFF            {LED4_GPIO_PORT->PBSC = LED4_GPIO_PIN;}

#define LED1_GPIO_PORT      GPIOB			            /* GPIO port */
#define LED1_GPIO_CLK       RCC_APB2Periph_GPIOB		/* GPIO port clock */
#define LED1_GPIO_PIN       GPIO_Pin_3			        /* GPIO connected to the SCL clock line */

#define LED2_GPIO_PORT      GPIOB			            /* GPIO port */
#define LED2_GPIO_CLK       RCC_APB2Periph_GPIOB		/* GPIO port clock */
#define LED2_GPIO_PIN       GPIO_Pin_13			        /* GPIO connected to the SCL clock line */

#define LED3_GPIO_PORT      GPIOB			            /* GPIO port */
#define LED3_GPIO_CLK       RCC_APB2Periph_GPIOB		/* GPIO port clock */
#define LED3_GPIO_PIN       GPIO_Pin_14			        /* GPIO connected to the SCL clock line */

#if (_VERSION_ == VERSION_3)     
#define LED4_GPIO_PORT      GPIOB			            /* GPIO port */
#define LED4_GPIO_CLK       RCC_APB2Periph_GPIOB		/* GPIO port clock */
#define LED4_GPIO_PIN       GPIO_Pin_8			        /* GPIO connected to the SCL clock line */
#elif (_VERSION_ == VERSION_4) 
#define LED4_GPIO_PORT      GPIOA			            /* GPIO port */
#define LED4_GPIO_CLK       RCC_APB2Periph_GPIOA		/* GPIO port clock */
#define LED4_GPIO_PIN       GPIO_Pin_4	
#endif

/** Define macros that control IO **/
#define LED1_TOGGLE         {LED1_GPIO_PORT->ODR ^= LED1_GPIO_PIN;}
#define LED1_ON             {LED1_GPIO_PORT->BRR = LED1_GPIO_PIN;}      // BIT CLEAR  Êä³ö0
#define LED1_OFF            {LED1_GPIO_PORT->BSRR = LED1_GPIO_PIN;}

#define LED2_TOGGLE         {LED2_GPIO_PORT->ODR ^= LED2_GPIO_PIN;}
#define LED2_ON             {LED2_GPIO_PORT->BRR = LED2_GPIO_PIN;}
#define LED2_OFF            {LED2_GPIO_PORT->BSRR = LED2_GPIO_PIN;}

#define LED3_TOGGLE         {LED3_GPIO_PORT->ODR ^= LED3_GPIO_PIN;}
#define LED3_ON             {LED3_GPIO_PORT->BRR = LED3_GPIO_PIN;}
#define LED3_OFF            {LED3_GPIO_PORT->BSRR = LED3_GPIO_PIN;}

#define LED4_TOGGLE         {LED4_GPIO_PORT->ODR ^= LED4_GPIO_PIN;}
#define LED4_ON             {LED4_GPIO_PORT->BRR = LED4_GPIO_PIN;}
#define LED4_OFF            {LED4_GPIO_PORT->BSRR = LED4_GPIO_PIN;}

void hw_led_initialize(void);


