#ifndef _TIM_CONFIG_H
#define _TIM_CONFIG_H

#include <stm32f10x.h>
#include "_HAL_GPIO.h"

#define _TIM1_CH1_GPIO_EN()		(GPIOA->)

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

typedef struct{
	TIM_TypeDef *timer;
	timerModes channelMod;
	uint16_t preScalar;
	uint16_t limitValue;
	
	GPIO_TYPE gpio;
	mytimers _TIMx_;
	timerModes mode;
	channels _CHx_;
	
}myTIMERcfg;

static setGPIO(myTIMERcfg mytimer);
static configTimer(myTIMERcfg mytimer);
static gpioTimerConfig(GPIO_TYPE gpio);
void initTimer(myTIMERcfg mytimer, uint32_t priority);

void timerEnable(TIM_TypeDef *timer);
void timerDisable(TIM_TypeDef *timer);

void timerSetPeriod(TIM_TypeDef *timer, uint16_t period);
uint16_t timerGetCounterValue(TIM_TypeDef *timer);

void initSysClck(void); // Only HSE, then PLL mult is used

void Delay_us (uint16_t us, TIM_TypeDef *timer);
void Delay_ms (uint16_t ms, TIM_TypeDef *timer);

#endif
