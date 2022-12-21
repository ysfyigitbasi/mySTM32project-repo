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
void initSysClck(void); // Only HSE, then PLL mult is used

#endif
