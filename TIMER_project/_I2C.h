#ifndef _I2C_H
#define _I2C_H

#include "stm32f10x.h"
//#define I2C1_REMAP
//#define FASTMODE
#define CLCK_FREQ_MHZ	36
#define MAX_BUFFER_SIZE_CHAR	40
#define MAX_BUFFER_SIZE_INT		40

//#define BMP388_ADDRESS 100

void initI2C1(void);
static void i2c_stop(void);

static void initI2C1_DMA(void);

static void enableTX_DMA(void);
static void enableRX_DMA(void);
static void disableRX_DMA(void);
static void disableTX_DMA(void);

void i2c_write_single(uint8_t slave_address, uint8_t mem_address, uint8_t data);
void i2c_writeMULT(uint8_t slave_address, uint8_t sensor_mem_address, char* mem_ptr, uint8_t mem_size);

char i2c_read_single( uint8_t slave_address, uint8_t sensor_mem_address);
void i2c_readMULT(uint8_t slave_address, uint8_t sensor_mem_address, char *mem_ptr, uint8_t mem_size);






#endif
