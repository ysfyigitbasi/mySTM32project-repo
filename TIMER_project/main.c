#include "_TIM_CONFIG.h"
#include "_HAL_GPIO.h"
#include "printMsg.h"

void Delay_us (uint16_t us, myTIMERcfg *timer);
void Delay_ms (uint16_t ms, myTIMERcfg *timer);
int main(){
	
	myTIMERcfg timer2;
	timer2.timer = TIM2;	timer2.preScalar = 36000;	timer2.limitValue = (uint16_t)2000;
	
	GPIO_TYPE myLED;
	myLED.port = PORTC; myLED.pin = 13; myLED.mode = OUTPUT_MODE; myLED.mode_type = OUTPUT_GEN_PURPOSE;
	myLED.speed = SPEED_50MHZ;
	
	GPIO_TYPE mybutton;
	mybutton.port = PORTB; mybutton.pin = 4; mybutton.mode = INPUT_MODE; mybutton.mode_type = INPUT_PU_PD;
	
	printMsg_config printer;
	printer.baud = 9600;
	printer.tx_port = GPIOA;
	printer.Uart_instance = USART1;
	
	
	initSysClck();
	printMsg_init(printer);
	gpio_init(myLED);
	initTimer(timer2, 2);
	timerEnable(timer2.timer);
	//config_gpio_interrupt(PORTB,4,EDGE_RISING);
	//enable_gpio_IRQ(4,EXTI4_IRQn);


	uint32_t x = 0;
	
	while(1){
		//gpio_write(myLED.port, myLED.pin, HIGH);
		x = timer2.timer->CNT;
		//Delay_us(8000, &timer2);
		//gpio_write(myLED.port, myLED.pin, LOW);
		//Delay_us(8000, &timer2);
	}
	
}

void Delay_us (uint16_t us, myTIMERcfg *timer)
{
	/************** STEPS TO FOLLOW *****************
	1. RESET the Counter
	2. Wait for the Counter to reach the entered value. As each count will take 1 us, 
		 the total waiting time will be the required us delay
	************************************************/
	timer->timer->CNT = 0;
	while (timer->timer->CNT < us);
}

void Delay_ms (uint16_t ms, myTIMERcfg *timer)
{
	for (uint16_t i=0; i<ms; i++)
	{
		Delay_us (1000, timer); // delay of 1 ms
	}
}


// External interrupt handler function, for gpio port interrupts.
void EXTI4_IRQHandler(void){
	if((EXTI->PR & EXTI_PR_PR4) == EXTI_PR_PR4)
		clear_gpio_interrupt(4);
}

// Timer 2 Interrupt Request Handler Function
void TIM2_IRQHandler(void){
	TIM2->SR &= ~TIM_SR_UIF;
	gpio_toggle(PORTC, 13);
}

