#include "_UART.h"
#include <stdint.h>
#include "string.h"
#include "stdlib.h"
#include "stdarg.h"
#include "_HAL_GPIO.h"

#ifdef _USART3_EN

void init_USART3TX_DMA(volatile char *ptr){
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
	USART3->CR3 |= USART_CR3_DMAT;
	DMA1_Channel2->CPAR = (uint32_t)&USART3->DR;
	DMA1_Channel2->CMAR = (uint32_t)ptr;
	DMA1_Channel2->CNDTR = BUFFER_SIZE_3;
	DMA1_Channel2->CCR |= (DMA_TX3_PRIORITY << 12);
	DMA1_Channel2->CCR |= DMA_CCR2_MINC | DMA_CCR2_DIR | DMA_CCR2_TCIE | DMA_CCR2_CIRC;
	NVIC_EnableIRQ(DMA1_Channel2_IRQn);
	NVIC_SetPriority(DMA1_Channel2_IRQn, DMA1_CH2_IRQn_PRI);}

void usart3_DMA_TX_EN(void)	{DMA1_Channel2->CCR |= DMA_CCR2_EN;}

void init_USART3RX_DMA(volatile char *ptr){
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
	USART3->CR3 |= USART_CR3_DMAR;
	DMA1_Channel3->CPAR = (uint32_t)&USART3->DR;
	DMA1_Channel3->CMAR = (uint32_t)ptr;
	DMA1_Channel3->CNDTR = BUFFER_SIZE_3;
	DMA1_Channel3->CCR |= (DMA_RX3_PRIORITY << 12);
	DMA1_Channel3->CCR |= DMA_CCR3_MINC | DMA_CCR3_CIRC | DMA_CCR3_TCIE;
	NVIC_EnableIRQ(DMA1_Channel3_IRQn);
	NVIC_SetPriority(DMA1_Channel3_IRQn, DMA1_CH3_IRQn_PRI);	}

void usart3_DMA_RX_EN(void)	{DMA1_Channel3->CCR |= DMA_CCR3_EN;}							 

void DMA1_Channel2_IRQHandler(void){
	DMA1->IFCR = DMA_IFCR_CTCIF2;
	DMA1_Channel2->CCR &= ~(uint32_t)DMA_CCR2_EN;	
}
void DMA1_Channel3_IRQHandler(void){
	DMA1->IFCR = DMA_IFCR_CTCIF3;	}
#endif
	

#ifdef _USART1_EN
	
void init_USART1RX_DMA(volatile char* ptr){
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
	USART1->CR3 |= USART_CR3_DMAR;
	DMA1_Channel5->CPAR = (uint32_t)&USART1->DR;
	DMA1_Channel5->CMAR = (uint32_t)ptr;
	DMA1_Channel5->CNDTR = BUFFER_SIZE_1;
	DMA1_Channel5->CCR |= (DMA_RX1_PRIORITY << 12);
	DMA1_Channel5->CCR |= DMA_CCR5_MINC | DMA_CCR5_CIRC | DMA_CCR5_TCIE;
	NVIC_EnableIRQ(DMA1_Channel5_IRQn);
	NVIC_SetPriority(DMA1_Channel5_IRQn, DMA1_CH5_IRQn_PRI);	}

void usart1_DMA_RX_EN(void)	{DMA1_Channel5->CCR |= DMA_CCR5_EN;}

void init_USART1TX_DMA(volatile char *p){
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
	USART1->CR3 |= USART_CR3_DMAT;
	DMA1_Channel4->CPAR = (uint32_t)&USART1->DR;
	DMA1_Channel4->CMAR = (uint32_t)p;
	DMA1_Channel4->CNDTR = BUFFER_SIZE_1;
	DMA1_Channel4->CCR |= (DMA_TX1_PRIORITY << 12);
	DMA1_Channel4->CCR |= DMA_CCR4_MINC | DMA_CCR4_DIR | DMA_CCR4_TCIE | DMA_CCR4_CIRC;
	NVIC_EnableIRQ(DMA1_Channel4_IRQn);
	NVIC_SetPriority(DMA1_Channel4_IRQn, DMA1_CH4_IRQn_PRI);
}
void usart1_DMA_TX_EN(void){  DMA1_Channel4->CCR |= DMA_CCR4_EN;}

void DMA1_Channel4_IRQHandler(void){
	DMA1->IFCR = DMA_IFCR_CTCIF4;
	DMA1_Channel4->CCR &= ~(uint32_t)DMA_CCR4_EN;
}
void DMA1_Channel5_IRQHandler(void){
	DMA1->IFCR = DMA_IFCR_CTCIF5;	}
#endif
	

#ifdef _USART2_EN
	
void init_USART2TX_DMA(volatile char* ptr){
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
	USART2->CR3 |= USART_CR3_DMAT;
	DMA1_Channel7->CPAR = (uint32_t)&USART2->DR;
	DMA1_Channel7->CMAR = (uint32_t)ptr;
	DMA1_Channel7->CNDTR = BUFFER_SIZE_2;
	DMA1_Channel7->CCR |= (DMA_TX2_PRIORITY << 12);
	DMA1_Channel7->CCR |= DMA_CCR7_MINC | DMA_CCR7_DIR | DMA_CCR7_TCIE | DMA_CCR7_CIRC;
	NVIC_EnableIRQ(DMA1_Channel7_IRQn);
	NVIC_SetPriority(DMA1_Channel7_IRQn, DMA1_CH7_IRQn_PRI);
}

void usart2_DMA_TX_EN(void)	{DMA1_Channel7->CCR |= DMA_CCR7_EN;}

void init_USART2RX_DMA(volatile char *ptr){
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
	USART2->CR3 |= USART_CR3_DMAR;
	DMA1_Channel6->CPAR = (uint32_t)&USART2->DR;
	DMA1_Channel6->CMAR = (uint32_t)ptr;
	DMA1_Channel6->CNDTR = BUFFER_SIZE_2;
	DMA1_Channel6->CCR |= (DMA_RX2_PRIORITY << 12);
	DMA1_Channel6->CCR |= DMA_CCR6_MINC | DMA_CCR6_CIRC | DMA_CCR6_TCIE;
	NVIC_EnableIRQ(DMA1_Channel6_IRQn);
	NVIC_SetPriority(DMA1_Channel6_IRQn, DMA1_CH6_IRQn_PRI);	}

void usart2_DMA_RX_EN(void)	{DMA1_Channel6->CCR |= DMA_CCR6_EN;}

void DMA1_Channel7_IRQHandler(void){
	DMA1->IFCR = DMA_IFCR_CTCIF7;
	DMA1_Channel7->CCR &= ~(uint32_t)DMA_CCR7_EN;
}
void DMA1_Channel6_IRQHandler(void){
	DMA1->IFCR = DMA_IFCR_CTCIF6;	}
#endif
