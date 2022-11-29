// #################|	INCLUDES	|#########################

#include "stm32f10x.h"
#include <stdint.h>
#include "string.h"
#include "stdlib.h"
#include "stdarg.h"

// #################|	var/defines	|########################

#define DEBUG_UART		USART1
#define delay			for(int i=0;i<500000;i++)

// #################|	Prototypes	|########################

static void printMsg(char *msg, ...);


int main(){
	
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN | RCC_APB2ENR_USART1EN;
	GPIOA->CRH |= GPIO_CRH_CNF9_1 | GPIO_CRH_MODE9_0 | GPIO_CRH_MODE9_1; // MODE(1,1) 50MHZ, CNF(1,0) AF_PP
	GPIOA->CRH &= ~GPIO_CRH_CNF9_0;
	
	
	//##### BAUDRATE CLOCK CONFIG
	// clkFreq / (16 x BaudR) = 72MHz / (16 x 9600)
	// = 468.75
	USART1->BRR = 0x1D4C;
	USART1->CR1 |= USART_CR1_TE | USART_CR1_UE;
	
	while(1){
		printMsg("Hello, I am %d years old.\n",23);
		
		delay;
	}
	
}

static void printMsg(char *msg, ...){
	
	char buff[50];
	#ifdef DEBUG_UART
		va_list args;
		va_start(args, msg);
		vsprintf(buff, msg, args);
		
		for(uint8_t i=0; i<strlen(buff); i++){
			USART1->DR = buff[i];
			while( !(USART1->SR & USART_SR_TXE) );
		}	
	#endif
}

