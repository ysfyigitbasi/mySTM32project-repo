#include "_TIM_CONFIG.h"
#include "_HAL_GPIO.h"
#include "_UART.h"


int main(){
		
	GPIO_TYPE myled;
	myled.port = PORTC;
	myled.pin = 13;
	myled.mode = OUTPUT_MODE;
	myled.mode_type = OUTPUT_GEN_PURPOSE;
	myled.speed = SPEED_50MHZ;
	
	initSysClck();
	gpio_init(myled);
	
	initTimer();
	DMA1_Channel4->CCR |= DMA_CCR7_MINC
	
	uint16_t x = 0;
	
	while(1){
		x = TIM2->CNT;
	}
	
}


// External interrupt handler function, for gpio port interrupts.
void EXTI4_IRQHandler(void){
	if((EXTI->PR & EXTI_PR_PR4) == EXTI_PR_PR4)
		clear_gpio_interrupt(4);
}

// Timer 2 Interrupt Request Handler Function


