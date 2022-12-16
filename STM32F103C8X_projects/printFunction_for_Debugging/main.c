// #################|	INCLUDES	|#########################
#include "printMsg.h"
#include "_HAL_GPIO.h"
/*
static int ticks;
void delayUs(int us);
void delayMs(int ms);
void SysTick_Handler(void);

void gotoSleep(void);
static int test = 0;
*/

int main(void){
	// config systick for micro second interrupts
	//SysTick_Config(SystemCoreClock / 960000);
	
	GPIO_TYPE mytimer;
	printMsg_config printer;
	
	mytimer.port = GPIOB; mytimer.pin = 9; mytimer.mode = OUTPUT_MODE; mytimer.mode_type = OUTPUT_ALT_FUNCTION_PP;
	mytimer.speed = SPEED_50MHZ;
	//RCC->APB2ENR |= RCC_APB2ENR_AFIOEN; for enabling pwm gpio output mode, however, I had enabled it in UARTconfig.
	
	// -----------| config debug port for printMsg |-----
	printer.baud = 9600;
	printer.tx_port = GPIOA;
	printer.Uart_instance = USART1;
	printMsg_init(printer);
	// --------------------------------------------------
	/*
	if(((PWR->CSR)&(PWR_CSR_SBF))){	// check standby flag
		// clear PWR Wake up flag
		PWR->CR |= PWR_CR_CWUF;
		PWR->CR |= PWR_CR_CSBF;
		printMsg(USART1, "I have awaken from standby\n");
	}
	else
		printMsg(USART1, "I've awaken from power cycle\n");
	*/

	// TIMER SETTINGS |----------------------------------
	//ENABLIE OUTPUT GPIO PB9 - TIM4_CH4
	gpio_init(mytimer);
	
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN; // ENABLE THE CLOCK
	
	//PWM freq = Fclk/PSC/ARR
	//PWM Duty = CCR4/ARR
	
	// TIMER SETTINGS FOR PWM	
	TIM4->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2; // setting the pwm mode 110
	TIM4->CCMR2 &= ~TIM_CCMR2_OC4M_0; // upcounting, channel 4 is active as long as TIM4_CNT<TIM4_CCR4, else inactive.
	TIM4->CCMR2 |= TIM_CCMR2_OC4PE; // enable preload register.
	TIM4->CR1 	|= TIM_CR1_ARPE;	// enable auto reload preload enable bit.
	
	
	TIM4->PSC = 65535;
	TIM4->ARR = 6000;
	TIM4->CR1 |= TIM_CR1_CEN;
	
	printMsg(USART1,"UART connection established\r\n");
	
	while(1){
		
		/*
		printMsg(USART1, "-----Going to sleep\n\r");
		gotoSleep();
		printMsg(USART1, "fail");
		*/
		
		printMsg(USART1,"%d\r\n",TIM4->CNT);
		
		
	}
}	// ------------------| END MAIN | ------------------------
/*
void gotoSleep(void){
	// enable the PWR control clock
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	
	// Set SLEEPDEEP bit of Cortex System Control Register
	SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
	
	// Select Standby mode
	PWR->CR |= PWR_CR_PDDS;
	
	// Clear wake up flag
	PWR->CR |= PWR_CR_CWUF;
	
	// enable wake up pin
	PWR->CSR |= PWR_CSR_EWUP;
	
	// Request wait for interrupt
	__WFI();	
}
void delayUs(int us){
	ticks = us;
	while(ticks);	
}
void delayMs(int ms){
	while(ms--){
		delayUs(1000);
	}
}

void SysTick_Handler(){
	if(ticks != 0)
		ticks--;
}
*/
