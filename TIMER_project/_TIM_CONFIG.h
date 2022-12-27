#ifndef _TIM_CONFIG_H
#define _TIM_CONFIG_H

#include <stm32f10x.h>
#include "_HAL_GPIO.h"

/* 
knowledge:
	1- to enable the clock, each timer should be defined -> _TIMx_EN
	2- define and set the Output compare ccrX values
	3- Choose the mode, output toggle or pwm output, one pulse mode etc.
*/

//************ || ENABLE THE TIMERS	|| ********************
//#define _TIM1_EN()			{TIM1->CR1 |= TIM_CR1_CEN;}
//#define _TIM2_EN()			{TIM2->CR1 |= TIM_CR1_CEN;}
#define _TIM3_EN()				{TIM3->CR1 |= TIM_CR1_CEN;}
//#define _TIM4_EN()			{TIM4->CR1 |= TIM_CR1_CEN;}

//************ || PARAMETERS OF THE TIMER-3 || *****************
#define	TIM3_OUTP_CMP_PSC	(36000 - 1)
#define TIM3_OUTP_CMP_ARR	(4000 - 1)
#define TIM3_OUTP_CMP_CCR1	(2000 - 1)
//#define TIM3_OUTP_CMP_CCR2	(2000 - 1)
//#define TIM3_OUTP_CMP_CCR3	(2000 - 1)
//#define TIM3_OUTP_CMP_CCR4	(2000 - 1)
//#define TIM3_partREMAP
#define TIM3_CH1_OUTP_CMP_TOGGLE
//#define TIM3_CH2_OUTP_CMP_TOGGLE
//#define TIM3_CH3_OUTP_CMP_TOGGLE
//#define TIM3_CH4_OUTP_CMP_TOGGLE
//#define TIM3_CH1_OUTP_CMP_PWM
//#define TIM3_CH2_OUTP_CMP_PWM
//#define TIM3_CH3_OUTP_CMP_PWM
//#define TIM3_CH4_OUTP_CMP_PWM

//************ || interrupts OF THE TIMER-3 || *****************
#define _TIM3_CC1IEN()				{TIM3->DIER |= TIM_DIER_CC1IE;}
//#define _TIM3_CC2IEN()				{TIM3->DIER |= TIM_DIER_CC2IE;}
//#define _TIM3_CC3IEN()				{TIM3->DIER |= TIM_DIER_CC3IE;}
//#define _TIM3_CC4IEN()				{TIM3->DIER |= TIM_DIER_CC4IE;}

//************ || MACROS OF THE TIMER-3	|| *********************
#ifdef _TIM3_EN
#define _TIM3_RCC_EN()			{RCC->APB2ENR |= RCC_APB2ENR_AFIOEN; 				\
								 RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;}

#ifdef TIM3_OUTP_CMP_CCR1
	#define _TIM3_CH1_GPIO_EN()		{RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;	/*PA6*/							\
									 GPIOA->CRL |= GPIO_CRL_MODE6_0 | GPIO_CRL_MODE6_1 | GPIO_CRL_CNF6_1;	\
									 GPIOA->CRL &= ~GPIO_CRL_CNF6_0;}
#endif

#ifdef TIM3_OUTP_CMP_CCR2									 
	#define _TIM3_CH2_GPIO_EN()		{RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;	/*PA7*/							\
									 GPIOA->CRL |= GPIO_CRL_MODE7_0 | GPIO_CRL_MODE7_1 | GPIO_CRL_CNF7_1;	\
									 GPIOA->CRL &= ~GPIO_CRL_CNF7_0;}								 
#endif							 
		
#ifdef TIM3_OUTP_CMP_CCR3									 
	#define _TIM3_CH3_GPIO_EN()		{RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;	/*PB0*/							\
									 GPIOB->CRL |= GPIO_CRL_MODE0_0 | GPIO_CRL_MODE0_1 | GPIO_CRL_CNF0_1;	\
									 GPIOB->CRL &= ~GPIO_CRL_CNF0_0;}
#endif								 

#ifdef TIM3_OUTP_CMP_CCR4									 
	#define _TIM3_CH4_GPIO_EN()		{RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;	/*PB1*/							\
									 GPIOB->CRL |= GPIO_CRL_MODE1_0 | GPIO_CRL_MODE1_1 | GPIO_CRL_CNF1_1;	\
									 GPIOB->CRL &= ~GPIO_CRL_CNF1_0;}	
#endif

#ifdef TIM3_partREMAP
	#ifdef TIM3_OUTP_CMP_CCR1
		#define _TIM3_CH1RM_GPIO_EN()	{RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;	/*PB4*/							\
										 GPIOB->CRL |= GPIO_CRL_MODE4_0 | GPIO_CRL_MODE4_1 | GPIO_CRL_CNF4_1;	\
										 GPIOB->CRL &= ~GPIO_CRL_CNF4_0;}
	#endif

	#ifdef TIM3_OUTP_CMP_CCR2
		#define _TIM3_CH2RM_GPIO_EN()	{RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;	/*PB5*/							\
										 GPIOB->CRL |= GPIO_CRL_MODE5_0 | GPIO_CRL_MODE5_1 | GPIO_CRL_CNF5_1;	\
										 GPIOB->CRL &= ~GPIO_CRL_CNF5_0;}
	#endif
#endif
									 
#define _TIM3_GeneralSetup()		{TIM3->CR1 |= TIM_CR1_ARPE;			\
									 TIM3->PSC 	= TIM3_OUTP_CMP_PSC;	\
									 TIM3->ARR 	= TIM3_OUTP_CMP_ARR;	}

#ifdef 	TIM3_CH1_OUTP_CMP_TOGGLE								 
#define _TIM3_CH1_OUTP_COMPARE()		{TIM3->CCMR1 |= TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE;	\
/* TOGGLE MODE - PRELOAD ENABLED */		 TIM3->CCMR1 &= ~TIM_CCMR1_OC1M_2;	\
										 TIM3->CCR1 = TIM3_OUTP_CMP_CCR1;	\
										 TIM3->CCER |= TIM_CCER_CC1E;		\
										 TIM3->CCER &= ~TIM_CCER_CC1P;		\
										 TIM3->EGR |= TIM_EGR_UG;			}
#endif

#ifdef 	TIM3_CH1_OUTP_CMP_PWM								 
#define _TIM3_CH1_OUTP_PWM()			{TIM3->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1PE;	\
/* PWM MODE - PRELOAD ENABLED */		 TIM3->CCMR1 &= ~TIM_CCMR1_OC1M_0;	\
										 TIM3->CCR1 = TIM3_OUTP_CMP_CCR1;	\
										 TIM3->CCER |= TIM_CCER_CC1E;		\
										 TIM3->CCER &= ~TIM_CCER_CC1P;		\
										 TIM3->EGR |= TIM_EGR_UG;			}
#endif										 
										 
#ifdef TIM3_CH2_OUTP_CMP_TOGGLE										 
#define _TIM3_CH2_OUTP_COMPARE()		{TIM3->CCMR1 |= TIM_CCMR1_OC2M_0 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2PE;	\
										 TIM3->CCMR1 &= ~TIM_CCMR1_OC2M_2;	\
										 TIM3->CCR2 = TIM3_OUTP_CMP_CCR2;	\
										 TIM3->CCER |= TIM_CCER_CC2E;		\
										 TIM3->CCER &= ~TIM_CCER_CC2P;		\
										 TIM3->EGR |= TIM_EGR_UG;			}										 
#endif
										 
#ifdef TIM3_CH2_OUTP_CMP_PWM										 
#define _TIM3_CH2_OUTP_PWM()			{TIM3->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2PE;	\
										 TIM3->CCMR1 &= ~TIM_CCMR1_OC2M_0;	\
										 TIM3->CCR2 = TIM3_OUTP_CMP_CCR2;	\
										 TIM3->CCER |= TIM_CCER_CC2E;		\
										 TIM3->CCER &= ~TIM_CCER_CC2P;		\
										 TIM3->EGR |= TIM_EGR_UG;			}										 
#endif										 
										 
#ifdef TIM3_CH3_OUTP_CMP_TOGGLE										 
#define _TIM3_CH3_OUTP_COMPARE()		{TIM3->CCMR2 |= TIM_CCMR2_OC3M_0 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3PE;	\
										 TIM3->CCMR2 &= ~TIM_CCMR2_OC3M_2;	\
										 TIM3->CCR3 = TIM3_OUTP_CMP_CCR3;	\
										 TIM3->CCER |= TIM_CCER_CC3E;		\
										 TIM3->CCER &= ~TIM_CCER_CC3P;		\
										 TIM3->EGR |= TIM_EGR_UG;			}
#endif
										 
#ifdef TIM3_CH3_OUTP_CMP_PWM										 
#define _TIM3_CH3_OUTP_PWM()		{TIM3->CCMR2 |= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3PE;	\
										 TIM3->CCMR2 &= ~TIM_CCMR2_OC3M_0;	\
										 TIM3->CCR3 = TIM3_OUTP_CMP_CCR3;	\
										 TIM3->CCER |= TIM_CCER_CC3E;		\
										 TIM3->CCER &= ~TIM_CCER_CC3P;		\
										 TIM3->EGR |= TIM_EGR_UG;			}
#endif										 

#ifdef TIM3_CH4_OUTP_CMP_TOGGLE										 
#define _TIM3_CH4_OUTP_COMPARE()		{TIM3->CCMR2 |= TIM_CCMR2_OC4M_0 | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4PE;	\
										 TIM3->CCMR2 &= ~TIM_CCMR2_OC4M_2;	\
										 TIM3->CCR4 = TIM3_OUTP_CMP_CCR4;	\
										 TIM3->CCER |= TIM_CCER_CC4E;		\
										 TIM3->CCER &= ~TIM_CCER_CC4P;		\
										 TIM3->EGR |= TIM_EGR_UG;			}
#endif
										 
#ifdef TIM3_CH4_OUTP_CMP_PWM										 
#define _TIM3_CH4_OUTP_PWM()			{TIM3->CCMR2 |= TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4PE;	\
										 TIM3->CCMR2 &= ~TIM_CCMR2_OC4M_0;	\
										 TIM3->CCR4 = TIM3_OUTP_CMP_CCR4;	\
										 TIM3->CCER |= TIM_CCER_CC4E;		\
										 TIM3->CCER &= ~TIM_CCER_CC4P;		\
										 TIM3->EGR |= TIM_EGR_UG;			}
#endif


#ifdef _TIM3_CC1IEN
#define _TIM3_IRQ_CC1(Pri)			{_TIM3_CC1IEN()						\
/*Enable Timer Interrupt on NVIC*/	 NVIC_EnableIRQ(TIM3_IRQn);			\
/* Set IRQn of the Priority*/		 NVIC_SetPriority(TIM3_IRQn, (Pri));	\
									 _TIM3_EN()		}
#endif
									 
#ifdef _TIM3_CC2IEN
#define _TIM3_IRQ_CC2(Pri)			{_TIM3_CC2IEN()						\
/*Enable Timer Interrupt on NVIC*/	 NVIC_EnableIRQ(TIM3_IRQn);			\
/* Set IRQn of the Priority*/		 NVIC_SetPriority(TIM3_IRQn, (Pri));	\
									 _TIM3_EN()		}
#endif									 

#ifdef _TIM3_CC3IEN
#define _TIM3_IRQ_CC3(Pri)			{_TIM3_CC3IEN()						\
/*Enable Timer Interrupt on NVIC*/	 NVIC_EnableIRQ(TIM3_IRQn);			\
/* Set IRQn of the Priority*/		 NVIC_SetPriority(TIM3_IRQn, (Pri));	\
									 _TIM3_EN()		}
#endif
									 
#ifdef _TIM3_CC4IEN
#define _TIM3_ENABLE(Pri)			{_TIM3_CC4IEN()						\
/*Enable Timer Interrupt on NVIC*/	 NVIC_EnableIRQ(TIM3_IRQn);			\
/* Set IRQn of the Priority*/		 NVIC_SetPriority(TIM3_IRQn, (Pri));	\
									 _TIM3_EN()		}
#endif									 
#endif
								








							
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
