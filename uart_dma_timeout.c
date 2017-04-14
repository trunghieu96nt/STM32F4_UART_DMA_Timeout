/**
  ******************************************************************************
  * @file    uart_dma_timeout.c
  * @author  Vu Trung Hieu
  * @version V1.0
  * @date    14-April-2017
  * @brief   This file provides firmware functions to use UART - DMA 
  *          with unknown number of data:           
  *           + Initialization
  *           + Send & Receive
  * 
@verbatim  
 ===============================================================================
                      ##### How to use this driver #####
 ===============================================================================                   
   (#) Enable the GPIO AHB clock using the following function
               
   (#) Configure the GPIO pin(s) using GPIO_Init()
       Four possible configuration are available for each pin:
       (++) Input: Floating, Pull-up, Pull-down.
       (++) Output: Push-Pull (Pull-up, Pull-down or no Pull)          
@endverbatim        
  *
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "uart_dma_timeout.h"
#include "stdbool.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static uint8_t rxbuff[BUFF_SIZE];
/* Public variables ----------------------------------------------------------*/
bool b_UART_DMA_Timeout = false; //Timeout Flag - clear by software
uint8_t rcv_message[BUFF_SIZE]; //Contains data receive
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initializes UART4 (PA0, PA1) - DMA1 Stream2 Channel4 
	*         TIM3 Count up - Input capture Channel1 - Slave Mode - Compare
  * @note   ...
  * @param  None
  * @retval None
  */
void UART_DMA_Timeout_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure; 
	USART_InitTypeDef USART_InitStructure;  
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	
	uint16_t uhPrescalerValue = 0;

  /* Enable GPIOA clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	/* Enable GPIOC clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  /* Enable UART clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
	/* Enable DMA1 clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	/* Enable TIM3 clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	
  /* Connect UART4 pins to AF2 */  
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_UART4);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_UART4); 

  /* GPIO Configuration for UART4 Tx */
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* GPIO Configuration for USART Rx */
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
       
  /* USARTx configured as follow:
		- BaudRate = 115200 baud  
    - Word Length = 8 Bits
    - One Stop Bit
    - No parity
    - Hardware flow control disabled (RTS and CTS signals)
    - Receive and transmit enabled
  */
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(UART4, &USART_InitStructure);

  /* Enable USART */
  USART_Cmd(UART4, ENABLE);
	/* Enable UART4 DMA */
  USART_DMACmd(UART4, USART_DMAReq_Rx, ENABLE);
  USART_DMACmd(UART4, USART_DMAReq_Tx, ENABLE);
	
	/* DMA1 Stream2 Channel4 for USART4 Rx configuration */			
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;  
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&UART4->DR;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)rxbuff;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = BUFF_SIZE;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;//DMA_Mode_Circular; MA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA1_Stream2, &DMA_InitStructure);
  DMA_Cmd(DMA1_Stream2, ENABLE);
	
	/* DMA1 Stream4 Channel4 for UART4 Tx configuration */			
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;  
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&UART4->DR;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)rxbuff; //Temporary
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  DMA_InitStructure.DMA_BufferSize = BUFF_SIZE;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA1_Stream4, &DMA_InitStructure);
  DMA_Cmd(DMA1_Stream4, ENABLE);
   
  /* GPIOC Configuration: TIM3 CH1 (PC6) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	/* Connect TIM Channels to AF2 */
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM3);
	
	/* Time base configuration - TIM3 counter clock at 10 KHz */
	uhPrescalerValue = (uint16_t) ((SystemCoreClock / 2) / 10000) - 1;
  TIM_TimeBaseInitStructure.TIM_Period = 65535; //Maximun
  TIM_TimeBaseInitStructure.TIM_Prescaler = uhPrescalerValue;
  TIM_TimeBaseInitStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);
	
	/* TIM3 Input capture configuration - Channel1, Rising */
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter = 0x0000;
	TIM_ICInit(TIM3, &TIM_ICInitStructure);
	/* Enable TIM3 Capture Compare 1 Interrupt */
	TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE);
	
	/* TIM3 Output Compare Configuration - Channel2, Active Mode, Output Disable */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Active;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
  TIM_OCInitStructure.TIM_Pulse = UART_DMA_TIMEOUT;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OC2Init(TIM3, &TIM_OCInitStructure);
	/* Enable TIM3 Capture Compare 2 Interrupt */
	TIM_ITConfig(TIM3, TIM_IT_CC2, DISABLE);
	
	/* Enable TIM3 Interrupt - Preemption: 0, Sub: 0 */
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	/* Slave Mode selection: TIM3 */
	TIM_SelectInputTrigger(TIM3, TIM_TS_TI1F_ED);
  TIM_SelectSlaveMode(TIM3, TIM_SlaveMode_Reset);
  TIM_SelectMasterSlaveMode(TIM3, TIM_MasterSlaveMode_Enable);
	TIM_Cmd(TIM3, ENABLE);
}

/**
  * @brief  Send Data to UART4 through DMA1 Stream4 Channel4
  * @note   ...
  * @param  pointer of array contains message
  * @retval None
  */
void UART4_DMA_Send(char* p_message, uint16_t message_size)
{
	if(message_size > 64)
	{
	}
	else
	{
		DMA_ClearFlag(DMA1_Stream4, DMA_FLAG_TCIF4);
		DMA_MemoryTargetConfig(DMA1_Stream4, (uint32_t) p_message, DMA_Memory_0);
		DMA_SetCurrDataCounter(DMA1_Stream4, message_size);//DMA1_Stream4->NDTR = BUFF_SIZE;
		DMA_Cmd(DMA1_Stream4, ENABLE);
	}
}

/**
  * @brief  TIM3_IRQHandler
  * @note   ...
  * @param  None
  * @retval None
  */
void TIM3_IRQHandler(void)
{ 
	static uint16_t pre_DataCounter;
	uint16_t DataCounter, i, j;
  if(TIM_GetITStatus(TIM3, TIM_IT_CC1) == SET) 
  {
    /* Clear TIM3 Capture Compare 1 interrupt pending bit */
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
		/* When a rising edge is detected */
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);
		TIM_ITConfig(TIM3, TIM_IT_CC2, ENABLE);
  }
	else if(TIM_GetITStatus(TIM3, TIM_IT_CC2) == SET) 
  {
		/* Clear TIM3 Capture Compare 1 interrupt pending bit  - start compare */
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);
		/* Disable interrupt CC2 - waiting new data */
		TIM_ITConfig(TIM3, TIM_IT_CC2, DISABLE);
		/* Timeout */
		DataCounter = BUFF_SIZE - DMA_GetCurrDataCounter(DMA1_Stream2);
		if(pre_DataCounter < DataCounter)
		{
			for(i = pre_DataCounter; i < DataCounter; i++)
			{
				rcv_message[i - pre_DataCounter] = rxbuff[i];
			}
			rcv_message[i - pre_DataCounter] = 0;
		}
		else
		{
			for(i = pre_DataCounter; i < BUFF_SIZE; i++)
			{
				rcv_message[i - pre_DataCounter] = rxbuff[i];
			}
			i -= pre_DataCounter; //Current pointer of rcv_messages
			for(j = 0; j < DataCounter; j++)
			{
				rcv_message[i + j] = rxbuff[j];
			}
			rcv_message[i + j] = 0;
		}
		pre_DataCounter = DataCounter;
		b_UART_DMA_Timeout = true;
  }
}
/*********************************END OF FILE**********************************/
