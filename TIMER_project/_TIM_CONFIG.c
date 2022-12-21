#include "_TIM_CONFIG.h"
#include "printMsg.h"

void initTimer(myTIMERcfg mytimer, uint32_t priority){
	
	if(mytimer.timer == TIM1){ //ADVANCED
		RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
		TIM1->PSC = mytimer.preScalar - 1;
		TIM1->ARR = mytimer.limitValue - 1;
		TIM1->CR1 |= TIM_CR1_CEN;
		while (!(TIM1->SR & TIM_SR_UIF))
			printMsg(USART1,"Timer register is not setted!\r\n");}
	
	else if(mytimer.timer == TIM2){
		RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
		TIM2->PSC = mytimer.preScalar - 1;
		TIM2->ARR = mytimer.limitValue - 1;
		//TIM2->CR1 |= TIM_CR1_CEN;
		//while (!(TIM2->SR & TIM_SR_UIF))
			//printMsg(USART1,"Timer register is not setted!\r\n");
		
		// Enable Timer Update Interrupt
		TIM2->DIER |= TIM_DIER_UIE;
		// Enable Timer Interrupt on NVIC
		NVIC_EnableIRQ(TIM2_IRQn);
		// Set IRQn of the Priority
		NVIC_SetPriority(TIM2_IRQn, priority);
	}

	else if(mytimer.timer == TIM3){
		RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
		TIM3->PSC = mytimer.preScalar - 1;
		TIM3->ARR = mytimer.limitValue - 1;
		TIM3->CR1 |= TIM_CR1_CEN;
		while (!(TIM3->SR & TIM_SR_UIF))
			printMsg(USART1,"Timer register is not setted!\r\n");}
	
	// DOES NOT WORK WITH MY STM32
	else if(mytimer.timer == TIM4){
		RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
		TIM4->PSC = mytimer.preScalar - 1;
		TIM4->ARR = mytimer.limitValue - 1;
		TIM4->CR1 |= TIM_CR1_CEN;
		while (!(TIM4->SR & TIM_SR_UIF))
			printMsg(USART1,"Timer register is not setted!\r\n");}
}

void timerEnable(TIM_TypeDef *timer){
	timer->CR1 |= TIM_CR1_CEN;	}

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
