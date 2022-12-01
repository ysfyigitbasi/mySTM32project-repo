// #################|	INCLUDES	|#########################
#include "printMsg.h"
int ticks;
void delayUs(int us);
void delamMs(int ms);
void SysTick_Handler(void);

void gotoSleep(void);
int test = 0;

int main(void){
	

	
	
	//##### BAUDRATE CLOCK CONFIG
	// clkFreq / (16 x BaudR) = 72MHz / (16 x 9600)
	// = 468.75
	USART1->BRR = 0x1D4C;
	USART1->CR1 |= USART_CR1_TE | USART_CR1_UE;
	
	while(1){
		print("Hello, I am %d years old.\n",23);
		
		delay;
	}
	
}

