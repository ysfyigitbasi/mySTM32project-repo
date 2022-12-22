#ifndef _TIM_CONFIG_H
#define _TIM_CONFIG_H

#include <stm32f10x.h>

typedef struct{
	TIM_TypeDef *timer;
	uint16_t preScalar;
	uint16_t limitValue;	
}myTIMERcfg;

void initTimer(myTIMERcfg mytimer, uint32_t priority);

void timerEnable(TIM_TypeDef *timer);
void timerDisable(TIM_TypeDef *timer);

void timerSetPeriod(TIM_TypeDef *timer, uint16_t period);
uint16_t timerGetCounterValue(TIM_TypeDef *timer);

void initSysClck(void); // Only HSE, then PLL mult is used

void Delay_us (uint16_t us, TIM_TypeDef *timer);
void Delay_ms (uint16_t ms, TIM_TypeDef *timer);

#endif
