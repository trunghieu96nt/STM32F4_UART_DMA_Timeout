/**
  ******************************************************************************
  * @file    uart_dma_timeout.h
  * @author  Vu Trung Hieu
  * @version V1.0
  * @date    14-April-2017
  * @brief   This file contains all the functions prototypes for uart_dma_timeout
  *          library.  
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UART_DMA_TIMEOUT_H
#define __UART_DMA_TIMEOUT_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

/* Define --------------------------------------------------------------------*/
#define BUFF_SIZE 64 //Maximun number of data including a \0 in end of message
#define UART_DMA_TIMEOUT 10 // *0.1ms - For this case: 1ms

/* Initialization and Configuration functions --------------------------------*/
void UART_DMA_Timeout_Init(void);

/* Send function -------------------------------------------------------------*/
void UART4_DMA_Send(char* p_message, uint16_t message_size);

#ifdef __cplusplus
}
#endif

#endif /*__UART_DMA_TIMEOUT_H */


/*********************************END OF FILE**********************************/
