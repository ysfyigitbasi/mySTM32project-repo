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
//#define _TIM2_EN()				{TIM2->CR1 |= TIM_CR1_CEN;}
//#define _TIM3_EN()			{TIM3->CR1 |= TIM_CR1_CEN;}
//#define _TIM4_EN()			{TIM4->CR1 |= TIM_CR1_CEN;}

//************ || PARAMETERS OF THE TIMER-2 || *****************
#define	TIM2_OUTP_CMP_PSC	(36000 - 1)
#define TIM2_OUTP_CMP_ARR	(4000 - 1)
#define TIM2_OUTP_CMP_CCR1	(2000 - 1)
//#define TIM2_OUTP_CMP_CCR2	(1000 - 1)
//#define TIM2_OUTP_CMP_CCR3	(2000 - 1)
//#define TIM2_OUTP_CMP_CCR4	(2000 - 1)
//#define TIM2_partREMAP_CH1	
//#define TIM2_partREMAP_CH2
//#define TIM2_partREMAP_CH3
//#define TIM2_partREMAP_CH4
//#define TIM2_UPCNT
#define TIM2_CH1_OUTP_CMP_TOGGLE
//#define TIM2_CH2_OUTP_CMP_TOGGLE
//#define TIM2_CH3_OUTP_CMP_TOGGLE
//#define TIM2_CH4_OUTP_CMP_TOGGLE
//#define TIM2_CH1_OUTP_CMP_PWM
//#define TIM2_CH2_OUTP_CMP_PWM
//#define TIM2_CH3_OUTP_CMP_PWM
//#define TIM2_CH4_OUTP_CMP_PWM
//#define TIM2_CH1_OUTP_OPMode
//#define TIM2_CH2_OUTP_OPMode
//#define TIM2_CH3_OUTP_OPMode
//#define TIM2_CH4_OUTP_OPMode

//************ || interrupts OF THE TIMER-3 || *****************
#define _TIM2_CC1IEN()					{TIM2->DIER |= TIM_DIER_CC1IE;}
//#define _TIM2_CC2IEN()				{TIM2->DIER |= TIM_DIER_CC2IE;}
//#define _TIM2_CC3IEN()				{TIM2->DIER |= TIM_DIER_CC3IE;}
//#define _TIM2_CC4IEN()				{TIM2->DIER |= TIM_DIER_CC4IE;}

//************ || PARAMETERS OF THE TIMER-3 || *****************
//#define	TIM3_OUTP_CMP_PSC	(36000 - 1)
//#define TIM3_OUTP_CMP_ARR	(4000 - 1)
//#define TIM3_OUTP_CMP_CCR1	(2000 - 1)
//#define TIM3_OUTP_CMP_CCR2	(2000 - 1)
//#define TIM3_OUTP_CMP_CCR3	(2000 - 1)
//#define TIM3_OUTP_CMP_CCR4	(2000 - 1)
//#define TIM3_partREMAP
//#define TIM3_CH1_OUTP_CMP_TOGGLE
//#define TIM3_CH2_OUTP_CMP_TOGGLE
//#define TIM3_CH3_OUTP_CMP_TOGGLE
//#define TIM3_CH4_OUTP_CMP_TOGGLE
//#define TIM3_CH1_OUTP_CMP_PWM
//#define TIM3_CH2_OUTP_CMP_PWM
//#define TIM3_CH3_OUTP_CMP_PWM
//#define TIM3_CH4_OUTP_CMP_PWM
//#define TIM3_CH1_OUTP_OPMode
//#define TIM3_CH2_OUTP_OPMode
//#define TIM3_CH3_OUTP_OPMode
//#define TIM3_CH4_OUTP_OPMode

//************ || interrupts OF THE TIMER-3 || *****************
//#define _TIM3_CC1IEN()					{TIM3->DIER |= TIM_DIER_CC1IE;}
//#define _TIM3_CC2IEN()				{TIM3->DIER |= TIM_DIER_CC2IE;}
//#define _TIM3_CC3IEN()				{TIM3->DIER |= TIM_DIER_CC3IE;}
//#define _TIM3_CC4IEN()				{TIM3->DIER |= TIM_DIER_CC4IE;}

//************ || PARAMETERS OF THE TIMER-4 || *****************
//#define	TIM4_OUTP_CMP_PSC	(36000 - 1)
//#define TIM4_OUTP_CMP_ARR	(4000 - 1)
//#define TIM4_OUTP_CMP_CCR1	(2000 - 1)
//#define TIM4_OUTP_CMP_CCR2	(2000 - 1)
//#define TIM4_OUTP_CMP_CCR3	(2000 - 1)
//#define TIM4_OUTP_CMP_CCR4	(2000 - 1)
//#define TIM4_CH1_OUTP_CMP_TOGGLE
//#define TIM4_CH2_OUTP_CMP_TOGGLE
//#define TIM4_CH3_OUTP_CMP_TOGGLE
//#define TIM4_CH4_OUTP_CMP_TOGGLE
//#define TIM4_CH1_OUTP_CMP_PWM
//#define TIM4_CH2_OUTP_CMP_PWM
//#define TIM4_CH3_OUTP_CMP_PWM
//#define TIM4_CH4_OUTP_CMP_PWM
//#define TIM4_CH1_OUTP_OPMode
//#define TIM4_CH2_OUTP_OPMode
//#define TIM4_CH3_OUTP_OPMode
//#define TIM4_CH4_OUTP_OPMode

//************ || interrupts OF THE TIMER-4 || *****************
//#define _TIM4_CC1IEN()					{TIM4->DIER |= TIM_DIER_CC1IE;}
//#define _TIM4_CC2IEN()				{TIM4->DIER |= TIM_DIER_CC2IE;}
//#define _TIM4_CC3IEN()				{TIM4->DIER |= TIM_DIER_CC3IE;}
//#define _TIM4_CC4IEN()				{TIM4->DIER |= TIM_DIER_CC4IE;}

//************ || MACROS OF THE TIMER-2	|| *********************
#ifdef _TIM2_EN
#define _TIM2_RCC_EN()			{RCC->APB2ENR |= RCC_APB2ENR_AFIOEN; 				\
								 RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;}

//************ || AFIO-GPIO ENABLE MACROS of TIMER-2	|| *********************								 
#if defined(TIM2_OUTP_CMP_CCR1) && !defined(TIM2_partREMAP_CH1)
	#define _TIM2_CH1_GPIO_EN()		{RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;	/*PA0*/							\
									 GPIOA->CRL |= GPIO_CRL_MODE0_0 | GPIO_CRL_MODE0_1 | GPIO_CRL_CNF0_1;	\
									 GPIOA->CRL &= ~GPIO_CRL_CNF0_0;}
#endif

#if defined(TIM2_OUTP_CMP_CCR2) && !defined(TIM2_partREMAP_CH2)									 
	#define _TIM2_CH2_GPIO_EN()		{RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;	/*PA1*/							\
									 GPIOA->CRL |= GPIO_CRL_MODE1_0 | GPIO_CRL_MODE1_1 | GPIO_CRL_CNF1_1;	\
									 GPIOA->CRL &= ~GPIO_CRL_CNF1_0;}								 
#endif							 
		
#if defined(TIM2_OUTP_CMP_CCR3) && !defined(TIM2_partREMAP_CH3)									 
	#define _TIM2_CH3_GPIO_EN()		{RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;	/*PA2*/							\
									 GPIOA->CRL |= GPIO_CRL_MODE2_0 | GPIO_CRL_MODE2_1 | GPIO_CRL_CNF2_1;	\
									 GPIOA->CRL &= ~GPIO_CRL_CNF2_0;}
#endif								 

#if defined(TIM2_OUTP_CMP_CCR4) && !defined(TIM2_partREMAP_CH4)									 
	#define _TIM2_CH4_GPIO_EN()		{RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;	/*PA3*/							\
									 GPIOA->CRL |= GPIO_CRL_MODE3_0 | GPIO_CRL_MODE3_1 | GPIO_CRL_CNF3_1;	\
									 GPIOA->CRL &= ~GPIO_CRL_CNF3_0;	}	
#endif

#if defined(TIM2_partREMAP_CH1) && defined(TIM2_OUTP_CMP_CCR1)
	#define _TIM2_CH1_GPIO_EN()		{RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;	/*PA15*/							\
									 GPIOA->CRH |= GPIO_CRH_MODE15_0 | GPIO_CRH_MODE15_1 | GPIO_CRH_CNF15_1;	\
									 GPIOA->CRH &= ~GPIO_CRH_CNF15_0;	\
									 AFIO->MAPR |= AFIO_MAPR_TIM2_REMAP_0 | AFIO_MAPR_TIM2_REMAP_1;	}
#endif

#if defined(TIM2_partREMAP_CH2) && defined(TIM2_OUTP_CMP_CCR2)
	#define _TIM2_CH2_GPIO_EN()		{RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;	/*PB3*/							\
									 GPIOB->CRL |= GPIO_CRL_MODE3_0 | GPIO_CRL_MODE3_1 | GPIO_CRL_CNF3_1;	\
									 GPIOB->CRL &= ~GPIO_CRL_CNF3_0;	\
									 AFIO->MAPR |= AFIO_MAPR_TIM2_REMAP_0;		}
#endif

#if defined(TIM2_partREMAP_CH3) && defined(TIM2_OUTP_CMP_CCR3)
	#define _TIM2_CH3_GPIO_EN()		{RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;	/*PB10*/							\
									 GPIOB->CRH |= GPIO_CRH_MODE10_0 | GPIO_CRH_MODE10_1 | GPIO_CRH_CNF10_1;	\
									 GPIOB->CRH &= ~GPIO_CRH_CNF10_0;	\
									 AFIO->MAPR |= AFIO_MAPR_TIM2_REMAP_1;		}
#endif

#if defined(TIM2_partREMAP_CH4) && defined(TIM2_OUTP_CMP_CCR4)
	#define _TIM2_CH4_GPIO_EN()		{RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;	/*PB11*/							\
									 GPIOB->CRH |= GPIO_CRH_MODE11_0 | GPIO_CRH_MODE11_1 | GPIO_CRH_CNF11_1;	\
									 GPIOB->CRH &= ~GPIO_CRH_CNF11_0;	\
									 AFIO->MAPR |= AFIO_MAPR_TIM2_REMAP_1;		}
#endif										 
										 
//************ || TIMER SETTINGS OF THE TIMER-2	|| *********************
#define _TIM2_GeneralSetup()		{TIM2->CR1 |= TIM_CR1_ARPE;			\
									 TIM2->PSC 	= TIM2_OUTP_CMP_PSC;	\
									 TIM2->ARR 	= TIM2_OUTP_CMP_ARR;	}

#ifdef TIM2_UPCNT									 
#define _TIM2_UPCOUNTER()			{TIM2->CR1 |= TIM_CR1_URS;		\
									 TIM2->DIER |= TIM_DIER_UIE;	\
									 TIM2->EGR |= TIM_EGR_UG;		\
									 NVIC_EnableIRQ(TIM2_IRQn);		}
#endif									 
	

#ifdef 	TIM2_CH1_OUTP_CMP_TOGGLE								 
#define _TIM2_CH1_OUTP_TGL()			{TIM2->CCMR1 |= TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE;	\
/* TOGGLE MODE - PRELOAD ENABLED */		 TIM2->CCMR1 &= ~TIM_CCMR1_OC1M_2;	\
										 TIM2->CCR1 = TIM2_OUTP_CMP_CCR1;	\
										 TIM2->CCER |= TIM_CCER_CC1E;		\
										 TIM2->CCER &= ~TIM_CCER_CC1P;		\
										 TIM2->EGR |= TIM_EGR_UG;			}
#endif

#ifdef 	TIM2_CH1_OUTP_CMP_PWM								 
#define _TIM2_CH1_OUTP_PWM()			{TIM2->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1PE;	\
/* PWM MODE - PRELOAD ENABLED */		 TIM2->CCMR1 &= ~TIM_CCMR1_OC1M_0;	\
										 TIM2->CCR1 = TIM2_OUTP_CMP_CCR1;	\
										 TIM2->CCER |= TIM_CCER_CC1E;		\
										 TIM2->CCER &= ~TIM_CCER_CC1P;		\
										 TIM2->EGR |= TIM_EGR_UG;			}
#endif

#ifdef TIM2_CH1_OUTP_OPMode
#define _TIM2_CH1_OUTP_OPM()			{TIM2->CCMR1 |= TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1PE;	\
										 TIM2->CCR1 = TIM2_OUTP_CMP_CCR1;	\
										 TIM2->CCER |= TIM_CCER_CC1E;		\
										 TIM2->CCER &= ~TIM_CCER_CC1P;		\
										 TIM2->CR1 |= TIM_CR1_OPM;			\
										 TIM2->EGR |= TIM_EGR_UG;			}
#endif
												 
#ifdef TIM2_CH2_OUTP_CMP_TOGGLE										 
#define _TIM2_CH2_OUTP_TGL()			{TIM2->CCMR1 |= TIM_CCMR1_OC2M_0 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2PE;	\
										 TIM2->CCMR1 &= ~TIM_CCMR1_OC2M_2;	\
										 TIM2->CCR2 = TIM2_OUTP_CMP_CCR2;	\
										 TIM2->CCER |= TIM_CCER_CC2E;		\
										 TIM2->CCER &= ~TIM_CCER_CC2P;		\
										 TIM2->EGR |= TIM_EGR_UG;			}										 
#endif										 
										 
#ifdef TIM2_CH2_OUTP_CMP_PWM										 
#define _TIM2_CH2_OUTP_PWM()			{TIM2->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2PE;	\
										 TIM2->CCMR1 &= ~TIM_CCMR1_OC2M_0;	\
										 TIM2->CCR2 = TIM2_OUTP_CMP_CCR2;	\
										 TIM2->CCER |= TIM_CCER_CC2E;		\
										 TIM2->CCER &= ~TIM_CCER_CC2P;		\
										 TIM2->EGR |= TIM_EGR_UG;			}										 
#endif										 
										 
#ifdef TIM2_CH2_OUTP_OPMode
#define _TIM2_CH2_OUTP_OPM()			{TIM2->CCMR1 |= TIM_CCMR1_OC2M_0 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2PE;	\
										 TIM2->CCR2 = TIM2_OUTP_CMP_CCR2;	\
										 TIM2->CCER |= TIM_CCER_CC2E;		\
										 TIM2->CCER &= ~TIM_CCER_CC2P;		\
										 TIM2->CR1 |= TIM_CR1_OPM;			\
										 TIM2->EGR |= TIM_EGR_UG;			}
#endif
										 
#ifdef TIM2_CH3_OUTP_CMP_TOGGLE										 
#define _TIM2_CH3_OUTP_TGL()			{TIM2->CCMR2 |= TIM_CCMR2_OC3M_0 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3PE;	\
										 TIM2->CCMR2 &= ~TIM_CCMR2_OC3M_2;	\
										 TIM2->CCR3 = TIM2_OUTP_CMP_CCR3;	\
										 TIM2->CCER |= TIM_CCER_CC3E;		\
										 TIM2->CCER &= ~TIM_CCER_CC3P;		\
										 TIM2->EGR |= TIM_EGR_UG;			}
#endif
										 
#ifdef TIM2_CH3_OUTP_CMP_PWM										 
#define _TIM2_CH3_OUTP_PWM()			{TIM2->CCMR2 |= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3PE;	\
										 TIM2->CCMR2 &= ~TIM_CCMR2_OC3M_0;	\
										 TIM2->CCR3 = TIM2_OUTP_CMP_CCR3;	\
										 TIM2->CCER |= TIM_CCER_CC3E;		\
										 TIM2->CCER &= ~TIM_CCER_CC3P;		\
										 TIM2->EGR |= TIM_EGR_UG;			}
#endif										 

#ifdef TIM2_CH3_OUTP_OPMode
#define _TIM2_CH3_OUTP_OPM()			{TIM2->CCMR2 |= TIM_CCMR2_OC3M_0 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3PE;	\
										 TIM2->CCR3 = TIM2_OUTP_CMP_CCR3;	\
										 TIM2->CCER |= TIM_CCER_CC3E;		\
										 TIM2->CCER &= ~TIM_CCER_CC3P;		\
										 TIM2->CR1 |= TIM_CR1_OPM;			\
										 TIM2->EGR |= TIM_EGR_UG;			}
#endif	
										 
#ifdef TIM2_CH4_OUTP_CMP_TOGGLE										 
#define _TIM2_CH4_OUTP_TGL()			{TIM2->CCMR2 |= TIM_CCMR2_OC4M_0 | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4PE;	\
										 TIM2->CCMR2 &= ~TIM_CCMR2_OC4M_2;	\
										 TIM2->CCR4 = TIM2_OUTP_CMP_CCR4;	\
										 TIM2->CCER |= TIM_CCER_CC4E;		\
										 TIM2->CCER &= ~TIM_CCER_CC4P;		\
										 TIM2->EGR |= TIM_EGR_UG;			}
#endif
										 
#ifdef TIM2_CH4_OUTP_CMP_PWM										 
#define _TIM2_CH4_OUTP_PWM()			{TIM2->CCMR2 |= TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4PE;	\
										 TIM2->CCMR2 &= ~TIM_CCMR2_OC4M_0;	\
										 TIM2->CCR4 = TIM2_OUTP_CMP_CCR4;	\
										 TIM2->CCER |= TIM_CCER_CC4E;		\
										 TIM2->CCER &= ~TIM_CCER_CC4P;		\
										 TIM2->EGR |= TIM_EGR_UG;			}
#endif

#ifdef TIM2_CH4_OUTP_OPMode
#define _TIM2_CH4_OUTP_OPM()			{TIM2->CCMR2 |= TIM_CCMR2_OC4M_0 | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4PE;	\
										 TIM2->CCR4 = TIM2_OUTP_CMP_CCR4;	\
										 TIM2->CCER |= TIM_CCER_CC4E;		\
										 TIM2->CCER &= ~TIM_CCER_CC4P;		\
										 TIM2->CR1 |= TIM_CR1_OPM;			\
										 TIM2->EGR |= TIM_EGR_UG;			}
#endif										 

//************ || INTERRUPT SETTINGS OF THE TIMER-2	|| *********************
#if defined(_TIM2_CC1IEN) || defined(_TIM2_CC2IEN) || defined(_TIM2_CC3IEN) || defined(_TIM2_CC4IEN) || defined(_TIM2_UPCOUNTER)
#define _TIM2_IRQ(Pri)				{NVIC_EnableIRQ(TIM2_IRQn);						\
/* Set IRQn of the Priority*/	 	 NVIC_SetPriority(TIM2_IRQn, (Pri));	\
									 _TIM2_EN()	}
#endif									 
#endif


//************ || MACROS OF THE TIMER-3	|| *********************
#ifdef _TIM3_EN
#define _TIM3_RCC_EN()			{RCC->APB2ENR |= RCC_APB2ENR_AFIOEN; 				\
								 RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;}

//************ || AFIO-GPIO ENABLE MACROS of TIMER-3	|| *********************								 
#if defined(TIM3_OUTP_CMP_CCR1) && !defined(TIM3_partREMAP)
	#define _TIM3_CH1_GPIO_EN()		{RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;	/*PA6*/							\
									 GPIOA->CRL |= GPIO_CRL_MODE6_0 | GPIO_CRL_MODE6_1 | GPIO_CRL_CNF6_1;	\
									 GPIOA->CRL &= ~GPIO_CRL_CNF6_0;}
#endif

#if defined(TIM3_OUTP_CMP_CCR2)	&& !defined(TIM3_partREMAP)								 
	#define _TIM3_CH2_GPIO_EN()		{RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;	/*PA7*/							\
									 GPIOA->CRL |= GPIO_CRL_MODE7_0 | GPIO_CRL_MODE7_1 | GPIO_CRL_CNF7_1;	\
									 GPIOA->CRL &= ~GPIO_CRL_CNF7_0;}								 
#endif							 
		
#if defined(TIM3_OUTP_CMP_CCR3)	&& !defined(TIM3_partREMAP)								 
	#define _TIM3_CH3_GPIO_EN()		{RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;	/*PB0*/							\
									 GPIOB->CRL |= GPIO_CRL_MODE0_0 | GPIO_CRL_MODE0_1 | GPIO_CRL_CNF0_1;	\
									 GPIOB->CRL &= ~GPIO_CRL_CNF0_0;}
#endif								 

#if defined(TIM3_OUTP_CMP_CCR4) && !defined(TIM3_partREMAP)									 
	#define _TIM3_CH4_GPIO_EN()		{RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;	/*PB1*/							\
									 GPIOB->CRL |= GPIO_CRL_MODE1_0 | GPIO_CRL_MODE1_1 | GPIO_CRL_CNF1_1;	\
									 GPIOB->CRL &= ~GPIO_CRL_CNF1_0;}	
#endif

#ifdef TIM3_partREMAP
	#ifdef TIM3_OUTP_CMP_CCR1
		#define _TIM3_CH1RM_GPIO_EN()	{RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;	/*PB4*/							\
										 GPIOB->CRL |= GPIO_CRL_MODE4_0 | GPIO_CRL_MODE4_1 | GPIO_CRL_CNF4_1;	\
										 GPIOB->CRL &= ~GPIO_CRL_CNF4_0;	\
										 AFIO->MAPR |= AFIO_MAPR_TIM3_REMAP_1;}
	#endif

	#ifdef TIM3_OUTP_CMP_CCR2
		#define _TIM3_CH2RM_GPIO_EN()	{RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;	/*PB5*/							\
										 GPIOB->CRL |= GPIO_CRL_MODE5_0 | GPIO_CRL_MODE5_1 | GPIO_CRL_CNF5_1;	\
										 GPIOB->CRL &= ~GPIO_CRL_CNF5_0;	\
										 AFIO->MAPR |= AFIO_MAPR_TIM3_REMAP_1;}
	#endif
#endif

//************ || TIMER SETTINGS OF THE TIMER-3	|| *********************
#define _TIM3_GeneralSetup()		{TIM3->CR1 |= TIM_CR1_ARPE;			\
									 TIM3->PSC 	= TIM3_OUTP_CMP_PSC;	\
									 TIM3->ARR 	= TIM3_OUTP_CMP_ARR;	}

#ifdef 	TIM3_CH1_OUTP_CMP_TOGGLE								 
#define _TIM3_CH1_OUTP_TGL()		{TIM3->CCMR1 |= TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE;	\
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

#ifdef TIM3_CH1_OUTP_OPMode
#define _TIM3_CH1_OUTP_OPM()			{TIM3->CCMR1 |= TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1PE;	\
										 TIM3->CCR1 = TIM3_OUTP_CMP_CCR1;	\
										 TIM3->CCER |= TIM_CCER_CC1E;		\
										 TIM3->CCER &= ~TIM_CCER_CC1P;		\
										 TIM3->CR1 |= TIM_CR1_OPM;			\
										 TIM3->EGR |= TIM_EGR_UG;			}
#endif
												 
#ifdef TIM3_CH2_OUTP_CMP_TOGGLE										 
#define _TIM3_CH2_OUTP_TGL()		{TIM3->CCMR1 |= TIM_CCMR1_OC2M_0 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2PE;	\
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
										 
#ifdef TIM3_CH2_OUTP_OPMode
#define _TIM3_CH2_OUTP_OPM()			{TIM3->CCMR1 |= TIM_CCMR1_OC2M_0 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2PE;	\
										 TIM3->CCR2 = TIM3_OUTP_CMP_CCR2;	\
										 TIM3->CCER |= TIM_CCER_CC2E;		\
										 TIM3->CCER &= ~TIM_CCER_CC2P;		\
										 TIM3->CR1 |= TIM_CR1_OPM;			\
										 TIM3->EGR |= TIM_EGR_UG;			}
#endif
										 
#ifdef TIM3_CH3_OUTP_CMP_TOGGLE										 
#define _TIM3_CH3_OUTP_TGL()		{TIM3->CCMR2 |= TIM_CCMR2_OC3M_0 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3PE;	\
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

#ifdef TIM3_CH3_OUTP_OPMode
#define _TIM3_CH3_OUTP_OPM()			{TIM3->CCMR2 |= TIM_CCMR2_OC3M_0 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3PE;	\
										 TIM3->CCR3 = TIM3_OUTP_CMP_CCR3;	\
										 TIM3->CCER |= TIM_CCER_CC3E;		\
										 TIM3->CCER &= ~TIM_CCER_CC3P;		\
										 TIM3->CR1 |= TIM_CR1_OPM;			\
										 TIM3->EGR |= TIM_EGR_UG;			}
#endif	
										 
#ifdef TIM3_CH4_OUTP_CMP_TOGGLE										 
#define _TIM3_CH4_OUTP_TGL()		{TIM3->CCMR2 |= TIM_CCMR2_OC4M_0 | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4PE;	\
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

#ifdef TIM3_CH4_OUTP_OPMode
#define _TIM3_CH4_OUTP_OPM()			{TIM3->CCMR1 |= TIM_CCMR2_OC4M_0 | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4PE;	\
										 TIM3->CCR4 = TIM3_OUTP_CMP_CCR4;	\
										 TIM3->CCER |= TIM_CCER_CC4E;		\
										 TIM3->CCER &= ~TIM_CCER_CC4P;		\
										 TIM3->CR1 |= TIM_CR1_OPM;			\
										 TIM3->EGR |= TIM_EGR_UG;			}
#endif										 

//************ || INTERRUPT SETTINGS OF THE TIMER-3	|| *********************
#if defined(_TIM3_CC1IEN) || defined(_TIM3_CC2IEN) || defined(_TIM3_CC3IEN) || defined(_TIM3_CC4IEN)
#define _TIM3_IRQ(Pri)				{NVIC_EnableIRQ(TIM3_IRQn);			\
/* Set IRQn of the Priority*/		 NVIC_SetPriority(TIM3_IRQn, (Pri));	\
									 _TIM3_EN()		}
#endif								 
#endif
								
//************ || MACROS OF THE TIMER-4	|| *********************
#ifdef _TIM4_EN
#define _TIM4_RCC_EN()			{RCC->APB2ENR |= RCC_APB2ENR_AFIOEN; 				\
								 RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;}

#ifdef TIM4_OUTP_CMP_CCR1
	#define _TIM4_CH1_GPIO_EN()		{RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;	/*PB6*/							\
									 GPIOB->CRL |= GPIO_CRL_MODE6_0 | GPIO_CRL_MODE6_1 | GPIO_CRL_CNF6_1;	\
									 GPIOB->CRL &= ~GPIO_CRL_CNF6_0;}
#endif

#ifdef TIM4_OUTP_CMP_CCR2									 
	#define _TIM4_CH2_GPIO_EN()		{RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;	/*PB7*/							\
									 GPIOB->CRL |= GPIO_CRL_MODE7_0 | GPIO_CRL_MODE7_1 | GPIO_CRL_CNF7_1;	\
									 GPIOB->CRL &= ~GPIO_CRL_CNF7_0;}								 
#endif							 
		
#ifdef TIM4_OUTP_CMP_CCR3									 
	#define _TIM4_CH3_GPIO_EN()		{RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;	/*PB8*/							\
									 GPIOB->CRH |= GPIO_CRH_MODE8_0 | GPIO_CRH_MODE8_1 | GPIO_CRH_CNF8_1;	\
									 GPIOB->CRH &= ~GPIO_CRH_CNF8_0;}
#endif								 

#ifdef TIM4_OUTP_CMP_CCR4									 
	#define _TIM4_CH4_GPIO_EN()		{RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;	/*PB9*/							\
									 GPIOB->CRH |= GPIO_CRH_MODE9_0 | GPIO_CRH_MODE9_1 | GPIO_CRH_CNF9_1;	\
									 GPIOB->CRH &= ~GPIO_CRH_CNF2_0;}	
#endif
									 
#define _TIM4_GeneralSetup()		{TIM4->CR1 |= TIM_CR1_ARPE;			\
									 TIM4->PSC 	= TIM4_OUTP_CMP_PSC;	\
									 TIM4->ARR 	= TIM4_OUTP_CMP_ARR;	}
								 
#ifdef 	TIM4_CH1_OUTP_CMP_TOGGLE								 
#define _TIM4_CH1_OUTP_TGL()		{TIM4->CCMR1 |= TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE;	\
/* TOGGLE MODE - PRELOAD ENABLED */		 TIM4->CCMR1 &= ~TIM_CCMR1_OC1M_2;	\
										 TIM4->CCR1 = TIM4_OUTP_CMP_CCR1;	\
										 TIM4->CCER |= TIM_CCER_CC1E;		\
										 TIM4->CCER &= ~TIM_CCER_CC1P;		\
										 TIM4->EGR |= TIM_EGR_UG;			}
#endif

#ifdef 	TIM4_CH1_OUTP_CMP_PWM								 
#define _TIM4_CH1_OUTP_PWM()			{TIM4->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1PE;	\
/* PWM MODE - PRELOAD ENABLED */		 TIM4->CCMR1 &= ~TIM_CCMR1_OC1M_0;	\
										 TIM4->CCR1 = TIM4_OUTP_CMP_CCR1;	\
										 TIM4->CCER |= TIM_CCER_CC1E;		\
										 TIM4->CCER &= ~TIM_CCER_CC1P;		\
										 TIM4->EGR |= TIM_EGR_UG;			}
#endif

#ifdef TIM4_CH1_OUTP_OPMode
#define _TIM4_CH1_OUTP_OPM()			{TIM4->CCMR1 |= TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1PE;	\
										 TIM4->CCR1 = TIM4_OUTP_CMP_CCR1;	\
										 TIM4->CCER |= TIM_CCER_CC1E;		\
										 TIM4->CCER &= ~TIM_CCER_CC1P;		\
										 TIM4->CR1 |= TIM_CR1_OPM;			\
										 TIM4->EGR |= TIM_EGR_UG;			}
#endif
												 
#ifdef TIM4_CH2_OUTP_CMP_TOGGLE										 
#define _TIM4_CH2_OUTP_TGL()		{TIM4->CCMR1 |= TIM_CCMR1_OC2M_0 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2PE;	\
										 TIM4->CCMR1 &= ~TIM_CCMR1_OC2M_2;	\
										 TIM4->CCR2 = TIM4_OUTP_CMP_CCR2;	\
										 TIM4->CCER |= TIM_CCER_CC2E;		\
										 TIM4->CCER &= ~TIM_CCER_CC2P;		\
										 TIM4->EGR |= TIM_EGR_UG;			}										 
#endif										 
										 
#ifdef TIM4_CH2_OUTP_CMP_PWM										 
#define _TIM4_CH2_OUTP_PWM()			{TIM4->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2PE;	\
										 TIM4->CCMR1 &= ~TIM_CCMR1_OC2M_0;	\
										 TIM4->CCR2 = TIM4_OUTP_CMP_CCR2;	\
										 TIM4->CCER |= TIM_CCER_CC2E;		\
										 TIM4->CCER &= ~TIM_CCER_CC2P;		\
										 TIM4->EGR |= TIM_EGR_UG;			}										 
#endif										 
										 
#ifdef TIM4_CH2_OUTP_OPMode
#define _TIM4_CH2_OUTP_OPM()			{TIM4->CCMR1 |= TIM_CCMR1_OC2M_0 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2PE;	\
										 TIM4->CCR2 = TIM4_OUTP_CMP_CCR2;	\
										 TIM4->CCER |= TIM_CCER_CC2E;		\
										 TIM4->CCER &= ~TIM_CCER_CC2P;		\
										 TIM4->CR1 |= TIM_CR1_OPM;			\
										 TIM4->EGR |= TIM_EGR_UG;			}
#endif
										 
#ifdef TIM4_CH3_OUTP_CMP_TOGGLE										 
#define _TIM4_CH3_OUTP_TGL()		{TIM4->CCMR2 |= TIM_CCMR2_OC3M_0 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3PE;	\
										 TIM4->CCMR2 &= ~TIM_CCMR2_OC3M_2;	\
										 TIM4->CCR3 = TIM4_OUTP_CMP_CCR3;	\
										 TIM4->CCER |= TIM_CCER_CC3E;		\
										 TIM4->CCER &= ~TIM_CCER_CC3P;		\
										 TIM4->EGR |= TIM_EGR_UG;			}
#endif
										 
#ifdef TIM4_CH3_OUTP_CMP_PWM										 
#define _TIM4_CH3_OUTP_PWM()		{TIM4->CCMR2 |= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3PE;	\
										 TIM4->CCMR2 &= ~TIM_CCMR2_OC3M_0;	\
										 TIM4->CCR3 = TIM4_OUTP_CMP_CCR3;	\
										 TIM4->CCER |= TIM_CCER_CC3E;		\
										 TIM4->CCER &= ~TIM_CCER_CC3P;		\
										 TIM4->EGR |= TIM_EGR_UG;			}
#endif										 

#ifdef TIM4_CH3_OUTP_OPMode
#define _TIM4_CH3_OUTP_OPM()			{TIM4->CCMR2 |= TIM_CCMR2_OC3M_0 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3PE;	\
										 TIM4->CCR3 = TIM4_OUTP_CMP_CCR3;	\
										 TIM4->CCER |= TIM_CCER_CC3E;		\
										 TIM4->CCER &= ~TIM_CCER_CC3P;		\
										 TIM4->CR1 |= TIM_CR1_OPM;			\
										 TIM4->EGR |= TIM_EGR_UG;			}
#endif	
										 
#ifdef TIM4_CH4_OUTP_CMP_TOGGLE										 
#define _TIM4_CH4_OUTP_TGL()		{TIM4->CCMR2 |= TIM_CCMR2_OC4M_0 | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4PE;	\
										 TIM4->CCMR2 &= ~TIM_CCMR2_OC4M_2;	\
										 TIM4->CCR4 = TIM4_OUTP_CMP_CCR4;	\
										 TIM4->CCER |= TIM_CCER_CC4E;		\
										 TIM4->CCER &= ~TIM_CCER_CC4P;		\
										 TIM4->EGR |= TIM_EGR_UG;			}
#endif
										 
#ifdef TIM4_CH4_OUTP_CMP_PWM										 
#define _TIM4_CH4_OUTP_PWM()			{TIM4->CCMR2 |= TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4PE;	\
										 TIM4->CCMR2 &= ~TIM_CCMR2_OC4M_0;	\
										 TIM4->CCR4 = TIM4_OUTP_CMP_CCR4;	\
										 TIM4->CCER |= TIM_CCER_CC4E;		\
										 TIM4->CCER &= ~TIM_CCER_CC4P;		\
										 TIM4->EGR |= TIM_EGR_UG;			}
#endif

#ifdef TIM4_CH4_OUTP_OPMode
#define _TIM4_CH4_OUTP_OPM()			{TIM4->CCMR1 |= TIM_CCMR2_OC4M_0 | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4PE;	\
										 TIM4->CCR4 = TIM4_OUTP_CMP_CCR4;	\
										 TIM4->CCER |= TIM_CCER_CC4E;		\
										 TIM4->CCER &= ~TIM_CCER_CC4P;		\
										 TIM4->CR1 |= TIM_CR1_OPM;			\
										 TIM4->EGR |= TIM_EGR_UG;			}
#endif										 

#if defined(_TIM4_CC1IEN) || defined(_TIM4_CC2IEN) || defined(_TIM4_CC3IEN) || defined(_TIM4_CC4IEN)
#define _TIM4_IRQ(Pri)				{NVIC_EnableIRQ(TIM4_IRQn);			\
/* Set IRQn of the Priority*/		 NVIC_SetPriority(TIM4_IRQn, (Pri));	\
									 _TIM4_EN()		}
#endif									 
#endif

//*************************|| FUNCTIONS ||**************************
void initTimer(void);

void timerEnable(TIM_TypeDef *timer);
void timerDisable(TIM_TypeDef *timer);

void timerSetARR(TIM_TypeDef *timer, uint16_t period);
uint16_t timerGetCounterValue(TIM_TypeDef *timer);

void initSysClck(void); // Only HSE, then PLL mult is used

void delayUS(uint16_t uS);
void delayMS(uint16_t mS);


#endif
//-------------- | TIMER'S GPIO PARAMETERS | ---------------

// DIPNOT FOR SETTING GPIO SETTINGS:******************
// IF YOU WOULD LIKE TO MAKE REMAP FOR STM32F103C8
// TIMER 1 HAS NO REMAP FOR (CH1-2-3-4 and ETR), just for (BKIN,CH1N,CH2N,CH3N)

// TIMER 2 HAS REMAPS [1:0]
// TIM2_REMAP[1:0] = "00" -> NO REMAP
// -------------CH1 - PA0
// -------------CH2 - PA1
// -------------CH3 - PA2 also remap 01
// -------------CH4 - PA3 also remap 01

// TIM2_REMAP[1:0] = "01" -> PARTIAL REMAP
// ------------- CH1 - PA15
// ------------- CH2 - PB3

// TIM2_REMAP[1:0] = "10" -> PARTIAL REMAP
// ------------- CH1 - PA0
// ------------- CH2 - PA1
// ------------- CH3 - PB10 - also fullremap
// ------------- CH4 - PB11 - also fullremap

// TIM2_REMAP[1:0] = "11" -> FULL REMAP
// ------------- CH1 - PA15
// ------------- CH2 - PB3

// TIMER 3 HAS ONLY PARTIAL REMAP FOR STM32F103C8
// TIM3_REMAP[1:0] = "00" -> NO REMAP
// -------------CH1 - PA6
// -------------CH2 - PA7
// -------------CH3 - PB0 also remap 01
// -------------CH4 - PB1 also remap 01

// TIM3_REMAP[1:0] = "01" -> PARTIAL REMAP
// -------------CH1 - PB4
// -------------CH2 - PB5
