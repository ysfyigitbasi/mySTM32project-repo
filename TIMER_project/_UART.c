#include "_UART.h"
#include <stdint.h>
#include "string.h"
#include "stdlib.h"
#include "stdarg.h"
#include "_HAL_GPIO.h"

void printMsg(USART_TypeDef *UsartP, const char *msg, ...){
	
	char buff[50];
	va_list args;
	va_start(args, msg);
	vsprintf(buff, msg, args);
		
	for(uint8_t i=0; i<strlen(buff); i++){
		UsartP->DR = buff[i];
		while( !(UsartP->SR & USART_SR_TXE) );
	}	
	
}

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