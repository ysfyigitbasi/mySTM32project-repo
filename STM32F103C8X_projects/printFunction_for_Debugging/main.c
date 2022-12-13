// #################|	INCLUDES	|#########################
#include "printMsg.h"
static int ticks;
void delayUs(int us);
void delayMs(int ms);
void SysTick_Handler(void);

void gotoSleep(void);
static int test = 0;

int main(void){
	// config systick for micro second interrupts
	SysTick_Config(SystemCoreClock / 960000);
	
	// -----------| config debug port for printMsg |-----
	printMsg_config printer;
	printer.baud = 9600;
	printer.tx_port = GPIOA;
	printer.Uart_instance = USART1;
	printMsg_init(printer);
	// --------------------------------------------------
	
	if(((PWR->CSR)&(PWR_CSR_SBF))){	// check standby flag
		// clear PWR Wake up flag
		PWR->CR |= PWR_CR_CWUF;
		PWR->CR |= PWR_CR_CSBF;
		printMsg(USART1, "I have awaken from standby\n");
	}
	else
		printMsg(USART1, "I've awaken from power cycle\n");
	
	while(1){
		printMsg(USART1, "-----Going to sleep\n\r");
		gotoSleep();
		printMsg(USART1, "fail");
	}
}	// ------------------| END MAIN | ------------------------

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

