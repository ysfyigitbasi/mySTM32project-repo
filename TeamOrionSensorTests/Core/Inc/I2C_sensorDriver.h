#ifndef I2C_sensorDriver_H_
#define I2C_sensorDriver_H_
#include <stdint.h>
#include "stm32f1xx_hal.h"

#define I2C_TIMEOUT_ms		10
extern I2C_HandleTypeDef hi2c1;

typedef enum {
	SENSOR_ERROR,
	SENSOR_OK	
}sensor_status_e;

uint8_t read8bit(uint8_t chipAdd, uint8_t regAdd);
uint16_t read16bit(uint8_t chipAdd, uint8_t regAdd);
int32_t read32bit(uint8_t chipAdd, uint8_t regAdd);
sensor_status_e readMultBytes(uint8_t chipAdd, uint8_t regAdd, uint8_t* data, uint8_t length);

sensor_status_e write8bit(uint8_t chipAdd, uint8_t regAdd, uint8_t value);
sensor_status_e writeMultBytes(uint8_t chipAdd, uint8_t regAdd, uint8_t* data, uint8_t length);

#endif
