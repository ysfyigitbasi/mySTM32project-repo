#include "stm32f10x.h"

//************ || ENABLE THE TIMER-1 || *****************
#define _USART1_EN()	{USART1->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;}
//#define _USART2_EN()	{USART2->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;}
//#define _USART3_EN()	{USART3->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;}


//************ || PARAMETERS OF THE USART-1 || *****************
#define USART1_BR	9600
#define BUFFER_SIZE_1 32
#define DMA_TX1_PRIORITY 0	// 0:LOW,1:MEDIUM,2:HIGH,3:VERYHIGH
#define DMA_RX1_PRIORITY 0	// 0:LOW,1:MEDIUM,2:HIGH,3:VERYHIGH
#define DMA1_CH4_IRQn_PRI	5 //TX
#define DMA1_CH5_IRQn_PRI	5 //RX
//#define USART1_REMAP


//************ || PARAMETERS OF THE USART-2 || *****************
#define USART2_BR 	9600
#define BUFFER_SIZE_2 32
#define DMA_TX2_PRIORITY 0	// 0:LOW,1:MEDIUM,2:HIGH,3:VERYHIGH
#define DMA1_CH6_IRQn_PRI	5 //RX
#define DMA1_CH7_IRQn_PRI	5 //TX

//************ || PARAMETERS OF THE USART-3 || *****************
#define USART3_BR 	9600
#define BUFFER_SIZE_3 32
#define DMA_TX3_PRIORITY 0	// 0:LOW,1:MEDIUM,2:HIGH,3:VERYHIGH
#define DMA1_CH2_IRQn_PRI	5	//TX
#define DMA1_CH3_IRQn_PRI	5	//RX

//************ || MACROS OF THE USART-1 || *****************
#ifdef _USART1_EN

#define _USART1_RCC_CLCK()	{RCC->APB2ENR |= RCC_APB2ENR_USART1EN | RCC_APB2ENR_AFIOEN;}

#define _USART1_BR_SET()		{uint32_t mantissa = (72000000/(16*USART1_BR)) & 0xFFF;	\
								 uint32_t fraction = ((72000000 % (16*USART1_BR)) / USART1_BR);	\
								 USART1->BRR = (mantissa << 4) | fraction;	}							
										
#ifndef USART1_REMAP
#define _USART1_GPIO()		{RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;									\
							 GPIOA->CRH |= GPIO_CRH_CNF9_1 | GPIO_CRH_MODE9_0 | GPIO_CRH_MODE9_1; 	\
							 GPIOA->CRH &= ~GPIO_CRH_CNF9_0;										\
							 GPIOA->CRH &= ~( GPIO_CRH_MODE10_0 | GPIO_CRH_MODE10_1 | GPIO_CRH_CNF10_1 );	\
							 GPIOA->CRH |= GPIO_CRH_CNF10_0;	}
#else
#define _USART1_GPIO()		{RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;									\
							 AFIO->MAPR |= AFIO_MAPR_USART1_REMAP;									\
							 GPIOB->CRL |= GPIO_CRL_CNF6_1 | GPIO_CRL_MODE6_0 | GPIO_CRL_MODE6_1; 	\
							 GPIOB->CRL &= ~GPIO_CRL_CNF6_0;										\
							 GPIOB->CRL &= ~( GPIO_CRL_MODE7_0 | GPIO_CRL_MODE7_1 | GPIO_CRL_CNF7_1 );	\
							 GPIOB->CRL |= GPIO_CRL_CNF7_0;	}
#endif							 

#define _USART1_DMA_TX()	{RCC->AHBENR |= RCC_AHBENR_DMA1EN;	\
							 USART1->CR3 |= USART_CR3_DMAT;		\
							 DMA1_Channel4->CPAR = (uint32_t)&USART1->DR;	\
							 DMA1_Channel4->CMAR = (uint32_t)transmitTX1;	\
							 DMA1_Channel4->CNDTR = BUFFER_SIZE_1;	\
							 DMA1_Channel4->CCR |= (DMA_TX1_PRIORITY << 12);	\
							 DMA1_Channel4->CCR |= DMA_CCR4_MINC | DMA_CCR4_DIR | DMA_CCR4_TCIE | DMA_CCR4_CIRC;	\
							 NVIC_EnableIRQ(DMA1_Channel4_IRQn);	\
							 NVIC_SetPriority(DMA1_Channel4_IRQn, DMA1_CH4_IRQn_PRI);	}

#define _USART1_DMA_TX_EN()	{DMA1_Channel4->CCR |= DMA_CCR4_EN;}

#define _USART1_DMA_RX()	{RCC->AHBENR |= RCC_AHBENR_DMA1EN;	\
							 USART1->CR3 |= USART_CR3_DMAR;		\
							 DMA1_Channel5->CPAR = (uint32_t)&USART1->DR;	\
							 DMA1_Channel5->CMAR = (uint32_t)receiveRX1;	\
							 DMA1_Channel5->CNDTR = BUFFER_SIZE_1;			\
							 DMA1_Channel5->CCR |= (DMA_RX1_PRIORITY << 12);	\
							 DMA1_Channel5->CCR |= DMA_CCR5_MINC | DMA_CCR5_CIRC | DMA_CCR5_TCIE;	\
							 NVIC_EnableIRQ(DMA1_Channel5_IRQn);	\
							 NVIC_SetPriority(DMA1_Channel5_IRQn, DMA1_CH5_IRQn_PRI);	}

#define _USART1_DMA_RX_EN()	{DMA1_Channel5->CCR |= DMA_CCR5_EN;}

#define _init_usart1()		{	_USART1_RCC_CLCK()	\
								_USART1_GPIO()		\
								_USART1_BR_SET()	\
								_USART1_DMA_TX()	\
								_USART1_DMA_RX()	\
								_USART1_DMA_RX_EN()	\
								_USART1_EN()		}
#endif

//************ || MACROS OF THE USART-2 || *****************
#ifdef _USART2_EN

#define _USART2_RCC_CLCK()	{RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;	\
							 RCC->APB1ENR |= RCC_APB1ENR_USART2EN;	}

#define _USART2_BR_SET()	{uint32_t mantissa = (36000000/(16*USART2_BR)) & 0xFFF;	\
							 uint32_t fraction = ((36000000 % (16*USART2_BR)) / USART2_BR);	\
							 USART2->BRR = (mantissa << 4) | fraction;	}

#define _USART2_GPIO()		{RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;									\
							 GPIOA->CRL |= GPIO_CRL_CNF2_1 | GPIO_CRL_MODE2_0 | GPIO_CRL_MODE2_1; 	\
							 GPIOA->CRL &= ~GPIO_CRL_CNF2_0;										\
							 GPIOA->CRL &= ~( GPIO_CRL_MODE3_0 | GPIO_CRL_MODE3_1 | GPIO_CRL_CNF3_1 );	\
							 GPIOA->CRL |= GPIO_CRL_CNF3_0;	}

#define _USART2_DMA_TX()	{RCC->AHBENR |= RCC_AHBENR_DMA1EN;	\
							 USART2->CR3 |= USART_CR3_DMAT;		\
							 DMA1_Channel7->CPAR = (uint32_t)&USART2->DR;	\
							 DMA1_Channel7->CMAR = (uint32_t)transmitTX2;	\
							 DMA1_Channel7->CNDTR = BUFFER_SIZE_2;	\
							 DMA1_Channel7->CCR |= (DMA_TX2_PRIORITY << 12);	\
							 DMA1_Channel7->CCR |= DMA_CCR7_MINC | DMA_CCR7_DIR | DMA_CCR7_TCIE | DMA_CCR7_CIRC;	\
							 NVIC_EnableIRQ(DMA1_Channel7_IRQn);	\
							 NVIC_SetPriority(DMA1_Channel7_IRQn, DMA1_CH7_IRQn_PRI);}

#define _USART2_DMA_TX_EN()	{DMA1_Channel7->CCR |= DMA_CCR7_EN;}

#define _USART2_DMA_RX()	{RCC->AHBENR |= RCC_AHBENR_DMA1EN;	\
							 USART2->CR3 |= USART_CR3_DMAR;		\
							 DMA1_Channel6->CPAR = (uint32_t)&USART2->DR;	\
							 DMA1_Channel6->CMAR = (uint32_t)receiveRX2;	\
							 DMA1_Channel6->CNDTR = BUFFER_SIZE_2;			\
							 DMA1_Channel6->CCR |= (DMA_RX2_PRIORITY << 12);	\
							 DMA1_Channel6->CCR |= DMA_CCR6_MINC | DMA_CCR6_CIRC | DMA_CCR6_TCIE;	\
							 NVIC_EnableIRQ(DMA1_Channel6_IRQn);	\
							 NVIC_SetPriority(DMA1_Channel6_IRQn, DMA1_CH6_IRQn_PRI);	}

#define _USART2_DMA_RX_EN()	{DMA1_Channel6->CCR |= DMA_CCR6_EN;}

#define _init_usart2(X)		{	_USART2_RCC_CLCK()	\
								_USART2_GPIO()		\
								_USART2_BR_SET()	\
								_USART2_DMA_TX()	\
								_USART2_DMA_RX()	\
								_USART2_DMA_RX_EN()	\
								_USART2_EN()		}
#endif				

//************ || MACROS OF THE USART-3 || *****************
#ifdef _USART3_EN

#define _USART3_RCC_CLCK()	{RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;	\
							 RCC->APB1ENR |= RCC_APB1ENR_USART3EN;	}

#define _USART3_BR_SET()	{uint32_t mantissa = (36000000/(16*USART3_BR)) & 0xFFF;	\
							 uint32_t fraction = ((36000000 % (16*USART3_BR)) / USART3_BR);	\
							 USART3->BRR = (mantissa << 4) | fraction;	}									
								 
#define _USART3_GPIO()		{RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;									\
							 GPIOB->CRH |= GPIO_CRH_CNF10_1 | GPIO_CRH_MODE10_0 | GPIO_CRH_MODE10_1; 	\
							 GPIOB->CRH &= ~GPIO_CRH_CNF10_0;										\
							 GPIOB->CRH &= ~( GPIO_CRH_MODE11_0 | GPIO_CRH_MODE11_1 | GPIO_CRH_CNF11_1 );	\
							 GPIOB->CRH |= GPIO_CRH_CNF11_0;	}

#define _USART3_DMA_TX()	{RCC->AHBENR |= RCC_AHBENR_DMA1EN;	\
							 USART3->CR3 |= USART_CR3_DMAT;		\
							 DMA1_Channel2->CPAR = (uint32_t)&USART3->DR;	\
							 DMA1_Channel2->CMAR = (uint32_t)transmitTX3;	\
							 DMA1_Channel2->CNDTR = BUFFER_SIZE_3;	\
							 DMA1_Channel2->CCR |= (DMA_TX3_PRIORITY << 12);	\
							 DMA1_Channel2->CCR |= DMA_CCR2_MINC | DMA_CCR2_DIR | DMA_CCR2_TCIE | DMA_CCR2_CIRC;	\
							 NVIC_EnableIRQ(DMA1_Channel2_IRQn);	\
							 NVIC_SetPriority(DMA1_Channel2_IRQn, DMA1_CH2_IRQn_PRI);	}

#define _USART3_DMA_TX_EN()	{DMA1_Channel2->CCR |= DMA_CCR2_EN;}

#define _USART3_DMA_RX()	{RCC->AHBENR |= RCC_AHBENR_DMA1EN;	\
							 USART3->CR3 |= USART_CR3_DMAR;		\
							 DMA1_Channel3->CPAR = (uint32_t)&USART3->DR;	\
							 DMA1_Channel3->CMAR = (uint32_t)receiveRX3;	\
							 DMA1_Channel3->CNDTR = BUFFER_SIZE_3;			\
							 DMA1_Channel3->CCR |= (DMA_RX3_PRIORITY << 12);	\
							 DMA1_Channel3->CCR |= DMA_CCR3_MINC | DMA_CCR3_CIRC | DMA_CCR3_TCIE;	\
							 NVIC_EnableIRQ(DMA1_Channel3_IRQn);	\
							 NVIC_SetPriority(DMA1_Channel3_IRQn, DMA1_CH3_IRQn_PRI);	}

#define _USART3_DMA_RX_EN()	{DMA1_Channel3->CCR |= DMA_CCR3_EN;}							 

#define _init_usart3(X)		{	_USART3_RCC_CLCK()	\
								_USART3_GPIO()		\
								_USART3_BR_SET()	\
								_USART3_DMA_TX()	\
								_USART3_DMA_RX()	\
								_USART3_DMA_RX_EN()	\
								_USART3_EN()		}

#endif
// ***************** || FUNCTIONS || ********************

