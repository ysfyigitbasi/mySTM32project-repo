#ifndef _I2C_H
#define _I2C_H

#include "stm32f10x.h"
//#define I2C1_REMAP
#define FASTMODE
#define CLCK_FREQ_MHZ	36
#define device_address	29
#define WRITE	0
#define READ	0


extern char receiveBUFF[10];
extern char transmitBUFF[10];

void initI2C1(void);
void initI2C1_DMA(void);

void enableTX_DMA(void);
void enableRX_DMA(void);










#endif
