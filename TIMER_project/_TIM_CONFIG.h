#ifndef _TIM_CONFIG_H
#define _TIM_CONFIG_H

#include <stm32f10x.h>
#include "_HAL_GPIO.h"

//************ || PARAMETERS OF THE TIMER-3 || *****************
#define TIM3_CHnumber	1
#define	TIM3_OUTP_CMP_PSC	(36000 - 1)
#define TIM3_OUTP_CMP_CCR1	(2000 - 1)
#define TIM3_OUTP_CMP_ARR	(4000 - 1)

//************ || MACROS OF THE TIMER-3	|| *********************
#define _TIM3_RCC_EN()			{RCC->APB2ENR |= RCC_APB2ENR_AFIOEN; 				\
								 RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;}

#define _TIM3_CH1_GPIO_EN()		{RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;									\
								 GPIOA->CRL |= GPIO_CRL_MODE6_0 | GPIO_CRL_MODE6_1 | GPIO_CRL_CNF6_1;	\
								 GPIOA->CRL &= ~GPIO_CRL_CNF6_0;}
								 
#define _TIM3_CH2_GPIO_EN()		{RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;									\
								 GPIOA->CRL |= GPIO_CRL_MODE7_0 | GPIO_CRL_MODE7_1 | GPIO_CRL_CNF7_1;	\
								 GPIOA->CRL &= ~GPIO_CRL_CNF7_0;}								 
								 
#define _TIM3_CH3_GPIO_EN()		{RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;									\
								 GPIOA->CRL |= GPIO_CRL_MODE0_0 | GPIO_CRL_MODE0_1 | GPIO_CRL_CNF0_1;	\
								 GPIOA->CRL &= ~GPIO_CRL_CNF0_0;}
								 
#define _TIM3_CH4_GPIO_EN()		{RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;									\
								 GPIOA->CRL |= GPIO_CRL_MODE1_0 | GPIO_CRL_MODE1_1 | GPIO_CRL_CNF1_1;	\
								 GPIOA->CRL &= ~GPIO_CRL_CNF1_0;}	
								 
#define _TIM3_CH1RM_GPIO_EN()	{RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;									\
								 GPIOA->CRL |= GPIO_CRL_MODE4_0 | GPIO_CRL_MODE4_1 | GPIO_CRL_CNF4_1;	\
								 GPIOA->CRL &= ~GPIO_CRL_CNF4_0;}

#define _TIM3_CH2RM_GPIO_EN()	{RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;									\
								 GPIOA->CRL |= GPIO_CRL_MODE5_0 | GPIO_CRL_MODE5_1 | GPIO_CRL_CNF5_1;	\
								 GPIOA->CRL &= ~GPIO_CRL_CNF5_0;}


#define _TIM3_OUTP_COMPARE()		{TIM3->CR1 |= TIM_CR1_ARPE;		\
/* TOGGLE MODE - PRELOAD ENABLED */	 TIM3->CCMR1 |= TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE;	\
									 TIM3->CCMR1 &= ~TIM_CCMR1_OC1M_2;	\
									 TIM3->CCR1 = TIM3_OUTP_CMP_CCR1;	\
									 TIM3->PSC 	= TIM3_OUTP_CMP_PSC;	\
									 TIM3->ARR 	= TIM3_OUTP_CMP_ARR;	\
								 	 TIM3->CCER |= TIM_CCER_CC1E;		\
									 TIM3->EGR |= TIM_EGR_UG;			}

#define _TIM3_ENABLE()				{TIM3->DIER |= TIM_DIER_CC1IE;	\
/*Enable Timer Interrupt on NVIC*/	 NVIC_EnableIRQ(TIM3_IRQn);		\
/* Set IRQn of the Priority*/		 NVIC_SetPriority(TIM3_IRQn, 3);	\
									 TIM3->CR1 |= TIM_CR1_CEN;		}

//************ || MACROS OF THE TIMER-3	|| *********************									 
#define _TIM2_RCC_EN()			{RCC->APB2ENR |= RCC_APB2ENR_AFIOEN; 				\
								 RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;}




							
/*
typedef enum{
	MOD_inputCapt,
	MOD_pwmInput,
	MOD_forcedOutp,
	MOD_outputComp,
	MOD_PWM_edge,
	MOD_PWM_center,
	MOD_onePulse	
}timerModes;


typedef enum{
	timer1,
	timer2,
	timer3,
	timer4
}mytimers;

typedef enum{
	CH1,
	CH2,
	CH3,
	CH4
}channels;
*/
typedef struct{
	TIM_TypeDef *timer;
	//timerModes channelMod;
	uint16_t preScalar;
	uint16_t limitValue;
	
	//GPIO_TYPE gpio;
	//mytimers _TIMx_;
	//timerModes mode;
	//channels _CHx_;
	
}myTIMERcfg;

//static setGPIO(myTIMERcfg mytimer);
//static configTimer(myTIMERcfg mytimer);
//static gpioTimerConfig(GPIO_TYPE gpio);
void initTimer(myTIMERcfg mytimer, uint32_t priority);

void timerEnable(TIM_TypeDef *timer);
void timerDisable(TIM_TypeDef *timer);

void timerSetPeriod(TIM_TypeDef *timer, uint16_t period);
uint16_t timerGetCounterValue(TIM_TypeDef *timer);

void initSysClck(void); // Only HSE, then PLL mult is used

void Delay_us (uint16_t us, TIM_TypeDef *timer);
void Delay_ms (uint16_t ms, TIM_TypeDef *timer);

#endif
