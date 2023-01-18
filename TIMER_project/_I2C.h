#ifndef _I2C_H
#define _I2C_H

#include "stm32f10x.h"
//#define I2C1_REMAP
#define FASTMODE
#define CLCK_FREQ_MHZ	36
#define WRITE	2
#define READ	1
#define MAX_BUFFER_SIZE_CHAR	40
#define MAX_BUFFER_SIZE_INT		40

//#define BMP388_ADDRESS 100


typedef struct{
	char transmitBUFF[MAX_BUFFER_SIZE_CHAR];
	char transmitData;
	uint8_t sizeTX;
	uint8_t memory_address, device_address;
	uint8_t state_TX;
	uint8_t I2C_MODE;
}TX_BUFFER;

typedef enum{
	NOT_USED,
	START,
	ADDRESS,
	READ_ADDRESS,
	TRANSMIT,
	STOP,
	DMA_RX
}txFlags;

extern TX_BUFFER bufferTX;


void initI2C1(void);

static void init_queue(TX_BUFFER *q);
static void initI2C1_DMA(void);

static void enableTX_DMA(void);
static void enableRX_DMA(void);

void i2c_write_single(uint8_t slave_address, uint8_t mem_address, uint8_t data);







#endif
