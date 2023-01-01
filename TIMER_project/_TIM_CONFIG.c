#include "_TIM_CONFIG.h"
#include "printMsg.h"
#include "_HAL_GPIO.h"

//****IRQ Handler functions are defined at the bottom.
//-------------------FUNCTIONS-----------------------------|
static uint64_t myTicks = 0;
static uint16_t msTicks = 0;
void initTimer(void){
	// IF SysTick is going to be used..
	SysTick_Config(SystemCoreClock / 1000); // 1ms interval
	
// IF TIMER 2 IS GOING TO BE USED... MODES: COMPARE-TOGGLE,PWM,ONEPULSE
	_TIM2_RCC_EN()	
	_TIM2_GeneralSetup()
	
	// IF UPCOUNTING MODE WILL BE USED
	_TIM2_UPCOUNTER()
	
	// IF CH1 IS GOING TO BE USED
//	_TIM2_CH1_GPIO_EN()
	// IF CH2 IS GOING TO BE USED
//	_TIM2_CH2_GPIO_EN()
	// IF CH3 IS GOING TO BE USED
//	_TIM2_CH3_GPIO_EN()
	// IF CH4 IS GOING TO BE USED
//	_TIM2_CH4_GPIO_EN()
	
	//IF TIMER 2 IS GOING TO BE USED FOR COMPARE-TOGGLE MODE
//	_TIM2_CH1_OUTP_TGL() // CH1
//	_TIM2_CH2_OUTP_TGL() // CH2
//	_TIM2_CH3_OUTP_TGL() // CH3
//	_TIM2_CH4_OUTP_TGL() // CH4
	
	//IF TIMER 2 IS GOING TO BE USED FOR PWM OUTP
//	_TIM2_CH1_OUTP_PWM() // CH1
//	_TIM2_CH2_OUTP_PWM() // CH2
//	_TIM2_CH3_OUTP_PWM() // CH3
//	_TIM2_CH4_OUTP_PWM() // CH4
	
	//IF TIMER 2 IS GOING TO BE USED FOR ONE PULSE MODE
//	_TIM2_CH1_OUTP_OPM() // CH1
//	_TIM2_CH2_OUTP_OPM() // CH2
//	_TIM2_CH3_OUTP_OPM() // CH3
//	_TIM2_CH4_OUTP_OPM() // CH4

//	_TIM2_CC1IEN()
//	_TIM2_IRQ(2)	
	
/*	
	// IF TIMER 3 IS GOING TO BE USED... MODES: COMPARE-TOGGLE,PWM,ONEPULSE
	_TIM3_RCC_EN()	
	_TIM3_GeneralSetup()
	
	// IF CH1 IS GOING TO BE USED
	_TIM3_CH1_GPIO_EN()
	// IF CH2 IS GOING TO BE USED
//	_TIM3_CH2_GPIO_EN()
	// IF CH3 IS GOING TO BE USED
//	_TIM3_CH3_GPIO_EN()
	// IF CH4 IS GOING TO BE USED
//	_TIM3_CH4_GPIO_EN()
	
	//IF TIMER 3 IS GOING TO BE USED FOR COMPARE-TOGGLE MODE
	_TIM3_CH1_OUTP_TGL() // CH1
//	_TIM3_CH2_OUTP_TGL() // CH2
//	_TIM3_CH3_OUTP_TGL() // CH3
//	_TIM3_CH4_OUTP_TGL() // CH4
	
	//IF TIMER 3 IS GOING TO BE USED FOR PWM OUTP
	_TIM3_CH1_OUTP_PWM() // CH1
//	_TIM3_CH2_OUTP_PWM() // CH2
//	_TIM3_CH3_OUTP_PWM() // CH3
//	_TIM3_CH4_OUTP_PWM() // CH4
	
	//IF TIMER 3 IS GOING TO BE USED FOR ONE PULSE MODE
	_TIM3_CH1_OUTP_OPM() // CH1
//	_TIM3_CH2_OUTP_OPM() // CH2
//	_TIM3_CH3_OUTP_OPM() // CH3
//	_TIM3_CH4_OUTP_OPM() // CH4

	_TIM3_CC1IEN()
	_TIM3_IRQ(2)				*/
	
/*	
	// IF TIMER 4 IS GOING TO BE USED... MODES: COMPARE-TOGGLE,PWM,ONEPULSE
	_TIM4_RCC_EN()	
	_TIM4_GeneralSetup()
	
	// IF CH1 IS GOING TO BE USED
	_TIM4_CH1_GPIO_EN()
	// IF CH2 IS GOING TO BE USED
//	_TIM4_CH2_GPIO_EN()
	// IF CH3 IS GOING TO BE USED
//	_TIM4_CH3_GPIO_EN()
	// IF CH4 IS GOING TO BE USED
//	_TIM4_CH4_GPIO_EN()
	
	//IF TIMER 4 IS GOING TO BE USED FOR COMPARE-TOGGLE MODE
	_TIM4_CH1_OUTP_TGL() // CH1
//	_TIM4_CH2_OUTP_TGL() // CH2
//	_TIM4_CH3_OUTP_TGL() // CH3
//	_TIM4_CH4_OUTP_TGL() // CH4
	
	//IF TIMER 4 IS GOING TO BE USED FOR PWM OUTP
	_TIM4_CH1_OUTP_PWM() // CH1
//	_TIM4_CH2_OUTP_PWM() // CH2
//	_TIM4_CH3_OUTP_PWM() // CH3
//	_TIM4_CH4_OUTP_PWM() // CH4
	
	//IF TIMER 4 IS GOING TO BE USED FOR ONE PULSE MODE
	_TIM4_CH1_OUTP_OPM() // CH1
//	_TIM4_CH2_OUTP_OPM() // CH2
//	_TIM4_CH3_OUTP_OPM() // CH3
//	_TIM4_CH4_OUTP_OPM() // CH4

	_TIM4_CC1IEN()
	_TIM4_IRQ(2)			*/
}
//--------------------------------------------------------------------
void timerEnable(TIM_TypeDef *timer){
	timer->CR1 |= TIM_CR1_CEN;	}

void timerDisable(TIM_TypeDef *timer){
	timer->CR1 &= ~TIM_CR1_CEN;	}
//---------------------------------------------------------------------
void timerSetARR(TIM_TypeDef *timer, uint16_t period){
	timer->ARR = period - 1;	}

//---------------------------------------------------------------------
uint16_t timerGetCounterValue(TIM_TypeDef *timer){
	return timer->CNT;			}

void delayUS(uint16_t uS){

	// if TIMER2 IS WANTED TO BE USED.
	_TIM2_EN()
	myTicks = 0;
	while(myTicks < uS);
	timerDisable(TIM2);
	
	/* if TIMER3 IS WANTED TO BE USED.
	_TIM3_EN()
	myTicks = 0;
	while(myTicks < uS);
	timerDisable(TIM3); */
	
	/* if TIMER4 IS WANTED TO BE USED.
	_TIM4_EN()
	myTicks = 0;
	while(myTicks < uS);
	timerDisable(TIM4); */
}

void delayMS(uint16_t mS){
	
	// if SysTick timer is going to be used.
	msTicks = 0;
	while(msTicks < mS);

	/* if TIMER2 IS WANTED TO BE USED.
	_TIM2_EN()
	myTicks = 0;
	while(myTicks < (mS * 1000) );
	timerDisable(TIM2);		*/
	
	/* if TIMER3 IS WANTED TO BE USED.
	_TIM3_EN()
	myTicks = 0;
	while(myTicks < (mS * 1000) );
	timerDisable(TIM3); */
	
	/* if TIMER4 IS WANTED TO BE USED.
	_TIM4_EN()
	myTicks = 0;
	while(myTicks < (mS * 1000) );
	timerDisable(TIM4); */
}

void initSysClck(void){ // 72MHz
	
	//Enables the interrupts HSE and PLL
	RCC->CIR |= RCC_CIR_HSERDYIE | RCC_CIR_PLLRDYIE;
	
	// Enable HSE and wait fot the HSE to become Ready
	RCC->CR |= RCC_CR_HSEON;
	while(!((RCC->CR & (RCC_CR_HSERDY) ) | RCC_CIR_HSERDYF ) );
	
	// Enable Power Interface clock enable
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	
	// Set PLL Source MUX
	RCC->CFGR &= ~(unsigned)(1 << 17); //HSE clock not divided.
	RCC->CFGR |= RCC_CFGR_PLLSRC_HSE | RCC_CFGR_PLLMULL9;
	RCC->CFGR &= ~(unsigned)( 1 << 21); // to ensure that 21th bit is 0
	
	RCC->CR |= RCC_CR_PLLON;
	while(!((RCC->CR & (RCC_CR_PLLRDY) ) | RCC_CIR_PLLRDYF ) );
	
	//Enable the Clock security system to detect any collapsion of the HSE
	RCC->CR |= RCC_CR_CSSON;
	
	// Choose the clock source of the System Clock SYSCLK
	RCC->CFGR |= RCC_CFGR_SW_PLL;
	RCC->CFGR &= ~(uint32_t)(1<<0); // to ensure that 0th bit is 0.
	
	while(!(RCC->CFGR & (RCC_CFGR_SWS_PLL)) );
	
	// Set AHB prescalar to 1 not to divide - 72MHz
	RCC->CFGR &= ~RCC_CFGR_HPRE_3;
	
	// Set APB1 prescaler to div 2
	RCC->CFGR |= RCC_CFGR_PPRE1_2;
	RCC->CFGR &= ~(RCC_CFGR_PPRE1_0 | RCC_CFGR_PPRE1_1); // to ensure that they are zero.
	
	// Set APB2 prescaler to div -1-
	RCC->CFGR &= ~(RCC_CFGR_PPRE2_2);
	
	// Set Cortex System Timer SysTick to 72MHz
	SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;	
}

//****************************************************
//****************************************************
//********----INTERRUPT HANDLER FUNCTIONS----*********
//****************************************************
//****************************************************
void TIM2_IRQHandler(void){
/* 	not necessary to check, Update Request Source bit is SET.
	if(TIM2->SR & TIM_SR_UIF){
		TIM2->SR &= ~TIM_SR_UIF;
		myTicks++;
	}
*/
	TIM2->SR &= ~TIM_SR_UIF;
	myTicks++;
	//gpio_toggle(PORTC, 13);
}

void TIM3_IRQHandler(void){
	//TIM3->SR &= ~TIM_SR_UIF;
	TIM3->SR &= ~TIM_SR_CC1IF;
	gpio_toggle(PORTC, 13);		}

void TIM4_IRQHandler(void){
	//TIM4->SR &= ~TIM_SR_UIF;
	TIM4->SR &= ~TIM_SR_CC1IF;
	gpio_toggle(PORTC, 13);		}

void SysTick_Handler(void){
	msTicks++;		}	
