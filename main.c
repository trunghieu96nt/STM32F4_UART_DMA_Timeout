#include "stm32f4xx.h"
#include "stdbool.h"
#include "string.h"
#include "system_timetick.h"
#include "uart_dma_timeout.h"

extern uint8_t rcv_message[BUFF_SIZE];
extern bool b_UART_DMA_Timeout;

void init_main(void);

int main(void)
{
	/* Enable SysTick at 10ms interrupt */
	SysTick_Config(SystemCoreClock/100);

	init_main();

	while(1)
	{
		if(tick_count == 100)
		{
			tick_count = 0;
			//while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
			//USART_SendData(UART4,(uint8_t)65);	
		}	
		else
		{
			if(b_UART_DMA_Timeout)
			{
				b_UART_DMA_Timeout = false;
				if(!strcmp((char*) rcv_message, "[PD12_TOGGLE]"))
				{
					GPIO_ToggleBits(GPIOD, GPIO_Pin_12);
				}
				else if(!strcmp((char*) rcv_message, "[PD12_ON]"))
				{
					GPIO_SetBits(GPIOD, GPIO_Pin_12);
				}
				else if(!strcmp((char*) rcv_message, "[PD12_OFF]"))
				{
					GPIO_ResetBits(GPIOD, GPIO_Pin_12);
				}
			}
		}
	}
}

void init_main(void)
{
  GPIO_InitTypeDef GPIO_InitStructure; 
   
	/* Enable GPIOD clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

  /* Configure PD12, PD13 in output pushpull mode */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	UART_DMA_Timeout_Init();
  
}
