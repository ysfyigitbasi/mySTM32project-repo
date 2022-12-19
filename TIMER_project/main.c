#include "_TIM_CONFIG.h"
#include "_HAL_GPIO.h"
#include "printMsg.h"

void Delay_us (uint16_t us, myTIMERcfg *timer);
void Delay_ms (uint16_t ms, myTIMERcfg *timer);
int main(){
	myTIMERcfg timer3;
	timer3.timer = TIM3;	timer3.preScalar = 36000;	timer3.limitValue = 0xFFFF;
	
	GPIO_TYPE myLED;
	myLED.port = PORTC; myLED.pin = 13; myLED.mode = OUTPUT_MODE; myLED.mode_type = OUTPUT_GEN_PURPOSE;
	myLED.speed = SPEED_50MHZ;
	
	
	printMsg_config printer;
	printer.baud = 9600;
	printer.tx_port = GPIOA;
	printer.Uart_instance = USART1;
	printMsg_init(printer);
	
	
	gpio_init(myLED);
	initTimer(timer3);
	
	int x = 0;
	
	while(1){
		gpio_write(myLED.port, myLED.pin, HIGH);
		x = timer3.timer->CNT;
		Delay_us(8000, &timer3);
		gpio_write(myLED.port, myLED.pin, LOW);
		Delay_us(8000, &timer3);
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

