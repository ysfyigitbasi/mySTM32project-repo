#include "stm32f10x.h"

#define CLCK_SPEED_72MHz	(uint32_t)72000000
#define CLCK_SPEED_36MHz	(uint32_t)36000000
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

#ifdef	_USART2_EN
//************ || PARAMETERS OF THE USART-2 || *****************
#define USART2_BR 	(uint16_t)9600
#define BUFFER_SIZE_2 32
#define DMA_TX2_PRIORITY 0	// 0:LOW,1:MEDIUM,2:HIGH,3:VERYHIGH
#define DMA_RX2_PRIORITY 0	// 0:LOW,1:MEDIUM,2:HIGH,3:VERYHIGH
#define DMA1_CH6_IRQn_PRI	5 //RX
#define DMA1_CH7_IRQn_PRI	5 //TX
#endif

#ifdef	_USART3_EN
//************ || PARAMETERS OF THE USART-3 || *****************
#define USART3_BR 	9600
#define BUFFER_SIZE_3 32
#define DMA_TX3_PRIORITY 0	// 0:LOW,1:MEDIUM,2:HIGH,3:VERYHIGH
#define DMA_RX3_PRIORITY 0	// 0:LOW,1:MEDIUM,2:HIGH,3:VERYHIGH
#define DMA1_CH2_IRQn_PRI	5	//TX
#define DMA1_CH3_IRQn_PRI	5	//RX
#endif

//************ || MACROS OF THE USART-1 || *****************
#ifdef _USART1_EN

#define _USART1_RCC_CLCK()	{RCC->APB2ENR |= RCC_APB2ENR_USART1EN | RCC_APB2ENR_AFIOEN;}

#define _USART1_BR_SET()		{uint32_t mantissa = (CLCK_SPEED_72MHz/(16*USART1_BR)) & 0xFFF;	\
								 uint32_t fraction = ((CLCK_SPEED_72MHz % (16*USART1_BR)) / USART1_BR);	\
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

#define _init_usart1()		{	_USART1_RCC_CLCK()	\
								_USART1_GPIO()		\
								_USART1_BR_SET()	\
								_USART1_EN()		}
#endif

//************ || MACROS OF THE USART-2 || *****************
#ifdef _USART2_EN

#define _USART2_RCC_CLCK()	{RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;	\
							 RCC->APB1ENR |= RCC_APB1ENR_USART2EN;	}

#define _USART2_BR_SET()	{uint32_t mantissa = (CLCK_SPEED_36MHz/(16*USART2_BR)) & 0xFFF;	\
							 uint32_t fraction = ((CLCK_SPEED_36MHz % (16*USART2_BR)) / USART2_BR);	\
							 USART2->BRR = (mantissa << 4) | fraction;	}

#define _USART2_GPIO()		{RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;									\
							 GPIOA->CRL |= GPIO_CRL_CNF2_1 | GPIO_CRL_MODE2_0 | GPIO_CRL_MODE2_1; 	\
							 GPIOA->CRL &= ~GPIO_CRL_CNF2_0;										\
							 GPIOA->CRL &= ~( GPIO_CRL_MODE3_0 | GPIO_CRL_MODE3_1 | GPIO_CRL_CNF3_1 );	\
							 GPIOA->CRL |= GPIO_CRL_CNF3_0;	}



#define _init_usart2()		{	_USART2_RCC_CLCK()	\
								_USART2_GPIO()		\
								_USART2_BR_SET()	\
								_USART2_EN()		}
#endif				

//************ || MACROS OF THE USART-3 || *****************
#ifdef _USART3_EN

#define _USART3_RCC_CLCK()	{RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;	\
							 RCC->APB1ENR |= RCC_APB1ENR_USART3EN;	}

#define _USART3_BR_SET()	{uint32_t mantissa = (CLCK_SPEED_36MHz/(16*USART3_BR)) & 0xFFF;	\
							 uint32_t fraction = ((CLCK_SPEED_36MHz % (16*USART3_BR)) / USART3_BR);	\
							 USART3->BRR = (mantissa << 4) | fraction;	}									
								 
#define _USART3_GPIO()		{RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;									\
							 GPIOB->CRH |= GPIO_CRH_CNF10_1 | GPIO_CRH_MODE10_0 | GPIO_CRH_MODE10_1; 	\
							 GPIOB->CRH &= ~GPIO_CRH_CNF10_0;										\
							 GPIOB->CRH &= ~( GPIO_CRH_MODE11_0 | GPIO_CRH_MODE11_1 | GPIO_CRH_CNF11_1 );	\
							 GPIOB->CRH |= GPIO_CRH_CNF11_0;	}


#define _init_usart3()		{	_USART3_RCC_CLCK()	\
								_USART3_GPIO()		\
								_USART3_BR_SET()	\
								_USART3_EN()		}

#endif
// ***************** || FUNCTIONS || ********************
#ifdef _USART1_EN								
void init_USART1TX_DMA(volatile char *p);
void usart1_DMA_TX_EN(void);
void init_USART1RX_DMA(volatile char* p);
void usart1_DMA_RX_EN(void);
#endif
#ifdef _USART2_EN
void usart2_DMA_TX_EN(void);
void init_USART2TX_DMA(volatile char* ptr);
void init_USART2RX_DMA(volatile char *ptr);
void usart2_DMA_RX_EN(void);
#endif								
#ifdef _USART3_EN
void usart3_DMA_RX_EN(void);
void init_USART3RX_DMA(volatile char *ptr);
void usart3_DMA_TX_EN(void);
void init_USART3TX_DMA(volatile char *ptr);
#endif
