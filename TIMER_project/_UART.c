#include "_UART.h"
#include <stdint.h>
#include "string.h"
#include "stdlib.h"
#include "stdarg.h"
#include "_HAL_GPIO.h"



#ifdef _USART3_EN
void DMA1_Channel2_IRQHandler(void){
	DMA1->IFCR = DMA_IFCR_CTCIF2;
	DMA1_Channel2->CCR &= ~(uint32_t)DMA_CCR2_EN;	
}
void DMA1_Channel3_IRQHandler(void){
	DMA1->IFCR = DMA_IFCR_CTCIF3;	}
#endif
#ifdef _USART1_EN
void DMA1_Channel4_IRQHandler(void){
	DMA1->IFCR = DMA_IFCR_CTCIF4;
	DMA1_Channel4->CCR &= ~(uint32_t)DMA_CCR4_EN;
}
void DMA1_Channel5_IRQHandler(void){
	DMA1->IFCR = DMA_IFCR_CTCIF5;	}
#endif
#ifdef _USART2_EN
void DMA1_Channel7_IRQHandler(void){
	DMA1->IFCR = DMA_IFCR_CTCIF7;
	DMA1_Channel7->CCR &= ~(uint32_t)DMA_CCR7_EN;
}
void DMA1_Channel6_IRQHandler(void){
	DMA1->IFCR = DMA_IFCR_CTCIF6;	}
#endif
