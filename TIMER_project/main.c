#include "_TIM_CONFIG.h"
#include "_HAL_GPIO.h"

void Delay_us (uint16_t us, myTIMERcfg *timer);
void Delay_ms (uint16_t ms, myTIMERcfg *timer);
int main(){
	myTIMERcfg timer2;
	timer2.timer = TIM2;	timer2.preScalar = 72;	timer2.limitValue = 0xFFFF;
	
	GPIO_TYPE myLED;
	myLED.port = PORTC; myLED.pin = 13; myLED.mode = OUTPUT_MODE; myLED.mode_type = OUTPUT_GEN_PURPOSE;
	myLED.speed = SPEED_50MHZ;
	
	SystemInit();
	gpio_init(myLED);
	initTimer(timer2);
	
	int x = 0;
	
	while(1){
		gpio_write(myLED.port, myLED.pin, HIGH);
		x = timer2.timer->CNT;
		Delay_ms(60, &timer2);
		Delay_ms(60, &timer2);
		Delay_ms(60, &timer2);
		Delay_ms(60, &timer2);
		Delay_ms(60, &timer2);
		Delay_ms(60, &timer2);
		Delay_ms(60, &timer2);
		Delay_ms(60, &timer2);
		Delay_ms(60, &timer2);
		Delay_ms(60, &timer2);
		Delay_ms(60, &timer2);
		Delay_ms(60, &timer2);
		Delay_ms(60, &timer2);
		Delay_ms(60, &timer2);
		Delay_ms(60, &timer2);
		Delay_ms(60, &timer2);
		Delay_ms(60, &timer2);
		Delay_ms(60, &timer2);
		Delay_ms(60, &timer2);
		Delay_ms(60, &timer2);
		Delay_ms(60, &timer2);
		Delay_ms(60, &timer2);
		Delay_ms(60, &timer2);
		Delay_ms(60, &timer2);
		Delay_ms(60, &timer2);
		gpio_write(myLED.port, myLED.pin, LOW);
		Delay_ms(60, &timer2);
		Delay_ms(60, &timer2);
		Delay_ms(60, &timer2);
		Delay_ms(60, &timer2);
		Delay_ms(60, &timer2);
		Delay_ms(60, &timer2);
		Delay_ms(60, &timer2);
		Delay_ms(60, &timer2);
		Delay_ms(60, &timer2);
		Delay_ms(60, &timer2);
		Delay_ms(60, &timer2);
		Delay_ms(60, &timer2);
		Delay_ms(60, &timer2);
		Delay_ms(60, &timer2);
		Delay_ms(60, &timer2);
		Delay_ms(60, &timer2);
		Delay_ms(60, &timer2);
		Delay_ms(60, &timer2);
		Delay_ms(60, &timer2);
		Delay_ms(60, &timer2);
		Delay_ms(60, &timer2);
		Delay_ms(60, &timer2);
		Delay_ms(60, &timer2);
		Delay_ms(60, &timer2);
		Delay_ms(60, &timer2);
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

