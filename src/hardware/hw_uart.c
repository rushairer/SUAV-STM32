
#include "hw_uart.h"


hw_uart_cfg_s hwDebugUart, hwOpflowUart;


//------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------
void DEBUG_USART_IRQHandler(void)
{    
  uint16_t t = hwDebugUart.fifoTx->tail;
	uint16_t h = hwDebugUart.fifoRx->head;
	uint8_t c;
	  
	if(USART_GetITStatus(DEBUG_USARTx, USART_IT_RXNE) != RESET) {   
//		USART_Flag_Clear(DEBUG_USARTx, USART_FLAG_RXDNE);  
      USART_ClearITPendingBit(DEBUG_USARTx, USART_IT_RXNE);
    
//		while(MY_UART_IS_RX_READY) {  //Get all the input characters		  
		  c = USART_ReceiveData(DEBUG_USARTx);;
      if (++h >= hwDebugUart.fifoRx->length) h = 0;
      if (h != hwDebugUart.fifoRx->tail) {
        hwDebugUart.fifoRx->buf[hwDebugUart.fifoRx->head] = c;
        hwDebugUart.fifoRx->head = h;
	    }
//	  }
  }

  // Is TxFIFO Empty Interrupt?
  if(USART_GetITStatus(DEBUG_USARTx, USART_IT_TXE) != RESET) {    
    USART_ClearITPendingBit(DEBUG_USARTx, USART_IT_TC);
    
    if (t != hwDebugUart.fifoTx->head) { 
      if (++t >= hwDebugUart.fifoTx->length) t = 0;
      USART_SendData(DEBUG_USARTx, hwDebugUart.fifoTx->buf[hwDebugUart.fifoTx->tail]);  // Transmit next byte in the ring
      hwDebugUart.fifoTx->tail = t;
    }
		
    if (t == hwDebugUart.fifoTx->head)              // Check if all data is transmitted . if yes disable transmitter UDRE interrupt
    {
			USART_ITConfig(DEBUG_USARTx, USART_IT_TXE, DISABLE);
    }
  }	
}


void hw_debug_uart_init(void)
{  
//    //----------------------------------------------------------------
//    GPIO_InitType GPIO_InitStructure;
//    USART_InitType USART_InitStructure;

//    // Turn on the clock of usart port GPIO
//    DEBUG_USART_GPIO_APBxClkCmd(DEBUG_USART_GPIO_CLK);               

//    // Turn on the clock of usart peripheral
//    DEBUG_USART_APBxClkCmd(DEBUG_USART_CLK);
//  
//    GPIO_Structure_Initialize(&GPIO_InitStructure);
//  
//    // Configure GPIO of USART TX as push pull multiplexing mode
//    GPIO_InitStructure.Pin        = DEBUG_USART_TX_GPIO_PIN;
//    GPIO_InitStructure.GPIO_Mode  = GPIO_MODE_AF_PP;
//    GPIO_InitStructure.GPIO_Alternate = DEBUG_USART_Tx_GPIO_AF;
//    GPIO_Peripheral_Initialize(DEBUG_USART_TX_GPIO_PORT, &GPIO_InitStructure);

//    // Configure GPIO of USART RX as floating input mode
//    GPIO_InitStructure.Pin        = DEBUG_USART_RX_GPIO_PIN;
//    GPIO_InitStructure.GPIO_Alternate = DEBUG_USART_Rx_GPIO_AF;
//    GPIO_Peripheral_Initialize(DEBUG_USART_RX_GPIO_PORT, &GPIO_InitStructure);

//    // Configure parameters of usart port
//    // Configure baud rate
//    USART_InitStructure.BaudRate = hwDebugUart.boudrate;
//    // Configure the length of frame data
//    USART_InitStructure.WordLength = USART_WL_8B;
//    // Configure stop bits
//    USART_InitStructure.StopBits = USART_STPB_1;
//    // Configure check bit
//    USART_InitStructure.Parity = USART_PE_NO;
//    // Configure hardware flow control
//    USART_InitStructure.HardwareFlowControl = USART_HFCTRL_NONE;
//    // Configure working mode, send and receive together
//    USART_InitStructure.Mode = USART_MODE_RX | USART_MODE_TX;
//    // Complete initialization configuration of usart port
//    USART_Initializes(DEBUG_USARTx, &USART_InitStructure);

//    // Configuration interrupt priority of the usart port    
//    NVIC_InitType NVIC_InitStructure;
//    /* Enable the USARTy Interrupt */
//    NVIC_InitStructure.NVIC_IRQChannel                   = DEBUG_USART_IRQ;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
//    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
//    NVIC_Initializes(&NVIC_InitStructure);

//    // Enable usart port receive interrupt    
//    USART_Interrput_Enable(DEBUG_USARTx, USART_INT_RXDNE);

//    // Enable usart
//    USART_Enable(DEBUG_USARTx);

    //----------------------------------------------------------------
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // Turn on the clock of usart port GPIO        
    DEBUG_USART_GPIO_APBxClkCmd(DEBUG_USART_GPIO_TX_CLK | DEBUG_USART_GPIO_RX_CLK | RCC_APB2Periph_AFIO, ENABLE);      

    // Turn on the clock of usart peripheral
    DEBUG_USART_APBxClkCmd(DEBUG_USART_CLK, ENABLE); 
      
    /* Configure USART Tx as alternate function push-pull */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = DEBUG_USART_TX_GPIO_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(DEBUG_USART_TX_GPIO_PORT, &GPIO_InitStructure);

    /* Configure USART Rx as input floating */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Pin = DEBUG_USART_RX_GPIO_PIN;
    GPIO_Init(DEBUG_USART_RX_GPIO_PORT, &GPIO_InitStructure);
        
    USART_InitStructure.USART_BaudRate = hwDebugUart.boudrate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
      
    /* USART configuration */
    USART_Init(DEBUG_USARTx, &USART_InitStructure);
      
    // Configuration interrupt priority of the usart port    
    /* Configure the NVIC Preemption Priority Bits */  
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
    
    /* Enable the USART Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = DEBUG_USART_IRQ;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_ITConfig(DEBUG_USARTx, USART_IT_RXNE, ENABLE);
//    USART_ITConfig(DEBUG_USARTx, USART_IT_TXE, ENABLE);
    
    /* Enable USART */
    USART_Cmd(DEBUG_USARTx, ENABLE);
    
}

//------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------
void OPFLOW_USART_IRQHandler(void)
{    
    uint16_t t = hwOpflowUart.fifoTx->tail;
    uint16_t h = hwOpflowUart.fifoRx->head;
    uint8_t c;

    if(USART_GetITStatus(OPFLOW_USARTx, USART_IT_RXNE) != RESET) { 
        //		USART_Flag_Clear(OPFLOW_USARTx, USART_FLAG_RXDNE);  

        //		while(MY_UART_IS_RX_READY) {  //Get all the input characters		  
        c = USART_ReceiveData(OPFLOW_USARTx);;
        if (++h >= hwOpflowUart.fifoRx->length) h = 0;
        if (h != hwOpflowUart.fifoRx->tail) {
            hwOpflowUart.fifoRx->buf[hwOpflowUart.fifoRx->head] = c;
            hwOpflowUart.fifoRx->head = h;
        }
        //	  }

    }

  // Is TxFIFO Empty Interrupt?
  if(USART_GetITStatus(OPFLOW_USARTx, USART_IT_TXE) != RESET) {    
    if (t != hwOpflowUart.fifoTx->head) { 
      if (++t >= hwOpflowUart.fifoTx->length) t = 0;
      USART_SendData(OPFLOW_USARTx, hwOpflowUart.fifoTx->buf[hwOpflowUart.fifoTx->tail]);  // Transmit next byte in the ring
      hwOpflowUart.fifoTx->tail = t;
    }
		
    if (t == hwOpflowUart.fifoTx->head)              // Check if all data is transmitted . if yes disable transmitter UDRE interrupt
    {
			USART_ITConfig(OPFLOW_USARTx, USART_IT_TXE, DISABLE);
    }
  }	
}

void hw_opticalflow_uart_init(void)
{  
//    //----------------------------------------------------------------
//    GPIO_InitType GPIO_InitStructure;
//    USART_InitType USART_InitStructure;

//    // Turn on the clock of usart port GPIO
//    OPFLOW_USART_GPIO_APBxClkCmd(OPFLOW_USART_GPIO_CLK);               

//    // Turn on the clock of usart peripheral
//    OPFLOW_USART_APBxClkCmd(OPFLOW_USART_CLK);
//  
//    GPIO_Structure_Initialize(&GPIO_InitStructure);
//  
//    // Configure GPIO of USART TX as push pull multiplexing mode
//    GPIO_InitStructure.Pin            = OPFLOW_USART_TX_GPIO_PIN;
//    GPIO_InitStructure.GPIO_Mode      = GPIO_MODE_AF_PP;
//    GPIO_InitStructure.GPIO_Alternate = OPFLOW_USART_Tx_GPIO_AF;
//    GPIO_Peripheral_Initialize(OPFLOW_USART_TX_GPIO_PORT, &GPIO_InitStructure);

//    // Configure GPIO of USART RX as floating input mode
//    GPIO_InitStructure.Pin            = OPFLOW_USART_RX_GPIO_PIN;
//    GPIO_InitStructure.GPIO_Alternate = OPFLOW_USART_Rx_GPIO_AF;
//    GPIO_Peripheral_Initialize(OPFLOW_USART_RX_GPIO_PORT, &GPIO_InitStructure);

//    // Configure parameters of usart port
//    // Configure baud rate
//    USART_InitStructure.BaudRate = hwOpflowUart.boudrate;
//    // Configure the length of frame data
//    USART_InitStructure.WordLength = USART_WL_8B;
//    // Configure stop bits
//    USART_InitStructure.StopBits = USART_STPB_1;
//    // Configure check bit
//    USART_InitStructure.Parity = USART_PE_NO;
//    // Configure hardware flow control
//    USART_InitStructure.HardwareFlowControl = USART_HFCTRL_NONE;
//    // Configure working mode, send and receive together
//    USART_InitStructure.Mode = USART_MODE_RX | USART_MODE_TX;
//    // Complete initialization configuration of usart port
//    USART_Initializes(OPFLOW_USARTx, &USART_InitStructure);

//    // Configuration interrupt priority of the usart port    
//    NVIC_InitType NVIC_InitStructure;
//    /* Enable the USARTy Interrupt */
//    NVIC_InitStructure.NVIC_IRQChannel                   = OPFLOW_USART_IRQ;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
//    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
//    NVIC_Initializes(&NVIC_InitStructure);

//    // Enable usart port receive interrupt    
//    USART_Interrput_Enable(OPFLOW_USARTx, USART_INT_RXDNE);

//    // Enable usart
//    USART_Enable(OPFLOW_USARTx);


    //----------------------------------------------------------------
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // Turn on the clock of usart port GPIO        
    OPFLOW_USART_GPIO_APBxClkCmd(OPFLOW_USART_GPIO_TX_CLK | OPFLOW_USART_GPIO_RX_CLK | RCC_APB2Periph_AFIO, ENABLE);      

    // Turn on the clock of usart peripheral
    OPFLOW_USART_APBxClkCmd(OPFLOW_USART_CLK, ENABLE); 
      
    /* Configure USART Tx as alternate function push-pull */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = OPFLOW_USART_TX_GPIO_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(OPFLOW_USART_TX_GPIO_PORT, &GPIO_InitStructure);

    /* Configure USART Rx as input floating */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Pin = OPFLOW_USART_RX_GPIO_PIN;
    GPIO_Init(OPFLOW_USART_RX_GPIO_PORT, &GPIO_InitStructure);
        
    USART_InitStructure.USART_BaudRate = hwOpflowUart.boudrate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
      
    /* USART configuration */
    USART_Init(OPFLOW_USARTx, &USART_InitStructure);
      
    // Configuration interrupt priority of the usart port    
    /* Configure the NVIC Preemption Priority Bits */  
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
    
    /* Enable the USART Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = OPFLOW_USART_IRQ;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_ITConfig(OPFLOW_USARTx, USART_IT_RXNE, ENABLE);
//    USART_ITConfig(OPFLOW_USARTx, USART_IT_TXE, ENABLE);
    
    /* Enable USART */
    USART_Cmd(OPFLOW_USARTx, ENABLE);
    

}




