#ifndef _I2C_H
#define _I2C_H

#include "stm32f10x.h"
//#define I2C1_REMAP
//#define FASTMODE
#define CLCK_FREQ_MHZ	36
#define MAX_BUFFER_SIZE_CHAR	40
#define MAX_BUFFER_SIZE_INT		40

//#define BMP388_ADDRESS 100


typedef struct{
	char *transmitBUFF;
	char transmitData;
	uint8_t sizeTX;
	uint8_t memory_address, device_address;
	uint8_t state_TX;
	uint8_t I2C_MODE;
}TX_BUFFER;

typedef struct{
	char receiveData;
	//uint8_t *sizeRX;
	char *receiveBUFF;
}RX_BUFFER;


typedef enum{
	STOP = 0,
	SLAVE_ADDRESS = 1,	// FOR WRITE PURPOSES
	REG_ADDRESS = 2,	// FOR READ PURPOSES
	RECEIVE = 3,
	RECEIVE_DMA = 4,
	TRANSMIT = 5,
	TRANSMITTED = 6,
	TRANSMIT_DMA_CMPLT = 7,
}I2C_STATES;

typedef enum{
	WRITE = 0,
	READ = 1,
	WRITE_DMA = 2,
	READ_DMA= 3,
}I2C_MODES;

extern TX_BUFFER bufferTX;
extern RX_BUFFER bufferRX;


void initI2C1(void);
static void i2c_stop(void);

static void config_DMA_TX(void);
static void config_DMA_RX(void);
static void init_queue(TX_BUFFER *q);
static void initI2C1_DMA(void);

static void enableTX_DMA(void);
static void enableRX_DMA(void);

void i2c_write_single(uint8_t slave_address, uint8_t mem_address, uint8_t data);
void i2c_writeMULT(uint8_t slave_address, uint8_t sensor_mem_address, char* mem_ptr, uint8_t mem_size);

void i2c_read_single( uint8_t slave_address, uint8_t sensor_mem_address);
void i2c_readMULT(uint8_t slave_address, uint8_t sensor_mem_address, char *mem_ptr, uint8_t mem_size);






#endif
