#include "_UART.h"
#include <stdint.h>
#include "string.h"
#include "stdlib.h"
#include "stdarg.h"
#include "_HAL_GPIO.h"


void USART1_IRQHandler(void){
	
	if(USART1->SR & USART_SR_RXNE){
		char temp = USART1->DR;
		USART1->DR = temp;
		while(!(USART1->SR & USART_SR_TC));
	}
	
	if(USART1->SR & USART_SR_TXE){
		
		gpio_toggle(PORTC,13);
	}
		
}
#ifdef _USART1_EN
void DMA1_Channel2_IRQHandler(void){
	DMA1->IFCR = DMA_IFCR_CTCIF2;
	DMA1_Channel2->CCR &= ~(uint32_t)DMA_CCR2_EN;	
}
#endif
#ifdef _USART2_EN
void DMA1_Channel4_IRQHandler(void){
	DMA1->IFCR = DMA_IFCR_CTCIF4;
	DMA1_Channel4->CCR &= ~(uint32_t)DMA_CCR4_EN;
}
#endif
#ifdef _USART3_EN
void DMA1_Channel7_IRQHandler(void){
	DMA1->IFCR = DMA_IFCR_CTCIF7;
	DMA1_Channel7->CCR &= ~(uint32_t)DMA_CCR7_EN;
}
#endif
