#include "_TIM_CONFIG.h"
#include "_HAL_GPIO.h"
#include "_UART.h"

extern volatile char receiveRX1[BUFFER_SIZE_1];
extern volatile char transmitTX1[BUFFER_SIZE_1];

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
	_init_usart1()
	
	uint16_t x = 0;
	
	while(1){
		x = TIM2->CNT;
		if (x > 2000){
			//enable dmatx etc.
			}
	}
}



