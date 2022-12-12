#include <stm32f10x.h>
#include "_HAL_GPIO.h"


int main(){

/*
	// init the clock for the GPIOC
	RCC->APB2ENR |= (1<<4);

	//Config pin13 on GPIOC as output
	GPIOC->CRH |= ( (1<<20) | (1<<21) );	//output 50mhz
	GPIOC->CRH &= ~( (1<<22) | (1<<23) ); // GPO P-Pull */
	
	GPIO_TYPE myGPIO;
	myGPIO.port = PORTC; myGPIO.pin = 13; myGPIO.mode = OUTPUT_MODE; myGPIO.mode_type = OUTPUT_GEN_PURPOSE;
	myGPIO.speed = SPEED_50MHZ;
	
	gpio_init(myGPIO);
	//config_gpio_interrupt(PORTB, 4, EDGE_RISING);
	//enable_gpio_interrupt(4, EXTI4_IRQn);
	

	while(1)
	{
		/*	GPIOC->BSRR = (1<<13); // SET
			for(int i = 0; i<10;i++);
			GPIOC->BSRR = (1<<(13+16)); // RESET
			for(int i=0; i<10;i++); */
		

		
		gpio_write(myGPIO.port, myGPIO.pin, HIGH);
		for(int i=0;i<500000;i++);
		gpio_write(myGPIO.port, myGPIO.pin, LOW);
		for(int j=0;j<500000;j++);
		
	} 	
}

/*
void EXTI4_IRQ_IRQHandler(){
	clear_gpio_interrupt(4);
	gpio_toggle(PORTC,13);
	
	
}*/

