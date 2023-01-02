#include "stm32f10x.h"

//************ || ENABLE THE TIMER-1 || *****************
#define _USART1_EN	{USART1->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;}
#define _USART2_EN	{USART2->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;}
#define _USART3_EN	{USART3->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;}


//************ || PARAMETERS OF THE USART-1 || *****************
#define USART1_BR	9600
//#define USART1_REMAP

//************ || PARAMETERS OF THE USART-2 || *****************
#define USART2_BR 	9600

//************ || PARAMETERS OF THE USART-3 || *****************
#define USART3_BR 	9600

//************ || MACROS OF THE USART-1 || *****************
#ifdef _USART1_EN

#define _USART1_RCC_CLCK()	{RCC->APB2ENR |= RCC_APB2ENR_USART1EN | RCC_APB2ENR_AFIOEN;}

#define _USART1_BR_SET()		{switch(USART1_BR) {	\
									case 2400:		\
										USART1->BRR = 0x7530;	\
										break;		\
									case 9600:		\
										USART1->BRR = 0x1D4C;	\
										break;		\
									case 19200:		\
										USART1->BRR = 0x0EA6;	\
										break;		\
									case 57600:		\
										USART1->BRR = 0x04E2;	\
										break;		\
									case 115200:	\
										USART1->BRR = 0x0271;	\
										break;		\
									case 2250000:	\
										USART1->BRR = 0x0020;	\
										break;		\
									case 4500000:	\
										USART1->BRR = 0x0010;	\
										break;		}			}

#define _USART1_IRQ_EN(PRIORITY)	{USART1->CR1 |= USART_CR1_RXNEIE | USART_CR1_TXEIE | USART_CR1_TCIE;	\
									 NVIC_EnableIRQ(USART1_IRQn);	\
									 NVIC_SetPriority(USART1_IRQn, (PRIORITY) );	}										
										
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

#define _init_usart1(X)		{	_USART1_RCC_CLCK()	\
								_USART1_GPIO()		\
								_USART1_BR_SET()	\
								_USART1_IRQ_EN(X)	}
#endif

//************ || MACROS OF THE USART-2 || *****************
#ifdef _USART2_EN

#define _USART2_RCC_CLCK()	{RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;	\
							 RCC->APB1ENR |= RCC_APB1ENR_USART2EN;	}

#define _USART2_BR_SET()		{switch(USART2_BR) {	\
									case 2400:		\
										USART2->BRR = 0x3A98;	\
										break;		\
									case 9600:		\
										USART2->BRR = 0x0EA6;	\
										break;		\
									case 19200:		\
										USART2->BRR = 0x0753;	\
										break;		\
									case 57600:		\
										USART2->BRR = 0x0271;	\
										break;		\
									case 2250000:	\
										USART2->BRR = 0x0010;	\
										break;		}			}

#define _USART2_IRQ_EN(PRIORITY)	{USART2->CR1 |= USART_CR1_RXNEIE | USART_CR1_TXEIE | USART_CR1_TCIE;	\
									 NVIC_EnableIRQ(USART2_IRQn);	\
									 NVIC_SetPriority(USART2_IRQn, (PRIORITY) );	}										

#define _USART2_GPIO()		{RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;									\
							 GPIOA->CRL |= GPIO_CRL_CNF2_1 | GPIO_CRL_MODE2_0 | GPIO_CRL_MODE2_1; 	\
							 GPIOA->CRL &= ~GPIO_CRL_CNF2_0;										\
							 GPIOA->CRL &= ~( GPIO_CRL_MODE3_0 | GPIO_CRL_MODE3_1 | GPIO_CRL_CNF3_1 );	\
							 GPIOA->CRL |= GPIO_CRL_CNF3_0;	}						 

#define _init_usart2(X)		{	_USART2_RCC_CLCK()	\
								_USART2_GPIO()		\
								_USART2_BR_SET()	\
								_USART2_IRQ_EN(X)	}
#endif				

//************ || MACROS OF THE USART-3 || *****************
#ifdef _USART3_EN

#define _USART3_RCC_CLCK()	{RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;	\
							 RCC->APB1ENR |= RCC_APB1ENR_USART3EN;	}

#define _USART3_BR_SET()		{switch(USART3_BR) {	\
									case 2400:		\
										USART3->BRR = 0x3A98;	\
										break;		\
									case 9600:		\
										USART3->BRR = 0x0EA6;	\
										break;		\
									case 19200:		\
										USART3->BRR = 0x0753;	\
										break;		\
									case 57600:		\
										USART3->BRR = 0x0271;	\
										break;		\
									case 2250000:	\
										USART3->BRR = 0x0010;	\
										break;		}			}

#define _USART3_IRQ_EN(PRIORITY)	{USART3->CR1 |= USART_CR1_RXNEIE | USART_CR1_TXEIE | USART_CR1_TCIE;	\
									 NVIC_EnableIRQ(USART3_IRQn);					\
									 NVIC_SetPriority(USART3_IRQn, (PRIORITY) );	}										

#define _USART3_GPIO()		{RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;									\
							 GPIOB->CRH |= GPIO_CRH_CNF10_1 | GPIO_CRH_MODE10_0 | GPIO_CRH_MODE10_1; 	\
							 GPIOB->CRH &= ~GPIO_CRH_CNF10_0;										\
							 GPIOB->CRH &= ~( GPIO_CRH_MODE11_0 | GPIO_CRH_MODE11_1 | GPIO_CRH_CNF11_1 );	\
							 GPIOB->CRH |= GPIO_CRH_CNF11_0;	}						 

#define _init_usart3(X)		{	_USART3_RCC_CLCK()	\
								_USART3_GPIO()		\
								_USART3_BR_SET()	\
								_USART3_IRQ_EN(X)	}
#endif
		
								
// ***************** || FUNCTIONS || ********************
void printMsg(USART_TypeDef *UsartP, const char *msg, ...);
