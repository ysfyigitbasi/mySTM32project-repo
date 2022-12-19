#include "_TIM_CONFIG.h"
#include "printMsg.h"
void initTimer(myTIMERcfg mytimer){
	
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
		TIM2->CR1 |= TIM_CR1_CEN;
		while (!(TIM2->SR & TIM_SR_UIF))
			printMsg(USART1,"Timer register is not setted!\r\n");}

	else if(mytimer.timer == TIM3){
		RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
		TIM3->PSC = mytimer.preScalar - 1;
		TIM3->ARR = mytimer.limitValue - 1;
		TIM3->CR1 |= TIM_CR1_CEN;
		while (!(TIM3->SR & TIM_SR_UIF))
			printMsg(USART1,"Timer register is not setted!\r\n");}
	
	/* DOES NOT WORK WITH MY STM32
	else if(mytimer.timer == TIM4){
		RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
		TIM4->PSC = mytimer.preScalar - 1;
		TIM4->ARR = mytimer.limitValue - 1;
		TIM4->CR1 |= TIM_CR1_CEN;
		while (!(TIM4->SR & TIM_SR_UIF))
			printMsg(USART1,"Timer register is not setted!\r\n");}	*/ 
	
}

