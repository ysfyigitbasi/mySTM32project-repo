#include "stm32f10x.h"
#include "_UART.h"

#define DMA_TX1_PRIORITY 0	// 0:LOW,1:MEDIUM,2:HIGH,3:VERYHIGH
#define DMA_RX1_PRIORITY 0	// 0:LOW,1:MEDIUM,2:HIGH,3:VERYHIGH
#define DMA1_CH4_IRQn_PRI	5 //TX
#define DMA1_CH5_IRQn_PRI	5 //RX


#define _USART1_DMA_TX()	{RCC->AHBENR |= RCC_AHBENR_DMA1EN;	\
							 USART1->CR3 |= USART_CR3_DMAT;		\
							 DMA1_Channel4->CPAR = (uint32_t)&USART1->DR;	\
							 DMA1_Channel4->CMAR = (uint32_t)(transmitTX1);	\
							 DMA1_Channel4->CNDTR = BUFFER_SIZE_1;	\
							 DMA1_Channel4->CCR |= (DMA_TX1_PRIORITY << 12);	\
							 DMA1_Channel4->CCR |= DMA_CCR4_MINC | DMA_CCR4_DIR | DMA_CCR4_TCIE | DMA_CCR4_CIRC;	\
							 NVIC_EnableIRQ(DMA1_Channel4_IRQn);	\
							 NVIC_SetPriority(DMA1_Channel4_IRQn, DMA1_CH4_IRQn_PRI);	}

#define _USART1_DMA_TX_EN()	{DMA1_Channel4->CCR |= DMA_CCR4_EN;}

#define _USART1_DMA_RX()	{RCC->AHBENR |= RCC_AHBENR_DMA1EN;	\
							 USART1->CR3 |= USART_CR3_DMAR;		\
							 DMA1_Channel5->CPAR = (uint32_t)&USART1->DR;	\
							 DMA1_Channel5->CMAR = (uint32_t)(receiveRX1);	\
							 DMA1_Channel5->CNDTR = BUFFER_SIZE_1;			\
							 DMA1_Channel5->CCR |= (DMA_RX1_PRIORITY << 12);	\
							 DMA1_Channel5->CCR |= DMA_CCR5_MINC | DMA_CCR5_CIRC | DMA_CCR5_TCIE;	\
							 NVIC_EnableIRQ(DMA1_Channel5_IRQn);	\
							 NVIC_SetPriority(DMA1_Channel5_IRQn, DMA1_CH5_IRQn_PRI);	}

#define _USART1_DMA_RX_EN()	{DMA1_Channel5->CCR |= DMA_CCR5_EN;}

#define _init_USART1_DMA()	{	_USART1_DMA_TX()	\
								_USART1_DMA_RX()	\
								_USART1_DMA_RX_EN()	}