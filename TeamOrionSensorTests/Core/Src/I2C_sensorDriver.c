#include "I2C_sensorDriver.h"

uint8_t read8bit(I2C_HandleTypeDef *hi2c_device, uint8_t chipAdd, uint8_t regAdd)
{
	HAL_StatusTypeDef status;
	uint8_t data;

	status = HAL_I2C_Mem_Read( hi2c_device, chipAdd, regAdd, 1, &data, 1, I2C_TIMEOUT_ms);
	
	return data;
}

uint16_t read16bit(I2C_HandleTypeDef *hi2c_device, uint8_t chipAdd, uint8_t regAdd)
{
	uint8_t temp[2];
	HAL_StatusTypeDef status;
	status = HAL_I2C_Mem_Read_DMA( hi2c_device, chipAdd, regAdd, 1, temp, 2);

	return (uint16_t)(temp[1]<<8) | (uint16_t)temp[0];
}

int32_t read32bit(I2C_HandleTypeDef *hi2c_device, uint8_t chipAdd, uint8_t regAdd)
{
	uint8_t temp[3];
	HAL_StatusTypeDef status;
	status = HAL_I2C_Mem_Read_DMA( hi2c_device, chipAdd, regAdd, 1, temp, 3);

	return (int32_t)(temp[2]<<16) | (int32_t)(temp[1]<<8) | (int32_t)temp[0] ;
}

sensor_status_e write8bit(I2C_HandleTypeDef *hi2c_device, uint8_t chipAdd, uint8_t regAdd, uint8_t value)
{
    HAL_StatusTypeDef status;
    status = HAL_I2C_Mem_Write( hi2c_device, chipAdd, regAdd, 1, &value, 1, I2C_TIMEOUT_ms );

    if (HAL_OK != status)
	{
        return SENSOR_ERROR;
	}
	else{
		return SENSOR_OK;
	}
}

sensor_status_e readMultBytes(I2C_HandleTypeDef *hi2c_device, uint8_t chipAdd, uint8_t regAdd, uint8_t* data, uint8_t length)
{

	HAL_StatusTypeDef status;
    status = HAL_I2C_Mem_Read_DMA( hi2c_device, chipAdd, regAdd, 1, data, length);
	
	if(HAL_OK != status)
	{
		return SENSOR_ERROR;
	}
	else{
		return SENSOR_OK;
	}
}

sensor_status_e readMultBytesWNoDMA(I2C_HandleTypeDef *hi2c_device, uint8_t chipAdd, uint8_t regAdd, uint8_t *data, uint8_t length)
{
	HAL_StatusTypeDef status;
    status = HAL_I2C_Mem_Read( hi2c_device, chipAdd, regAdd, 1, data, length, I2C_TIMEOUT_ms);
	
	if(HAL_OK != status)
	{
		return SENSOR_ERROR;
	}
	else{
		return SENSOR_OK;
	}
}

sensor_status_e writeMultBytes(I2C_HandleTypeDef *hi2c_device, uint8_t chipAdd, uint8_t regAdd, uint8_t* data, uint8_t length)
{

	HAL_StatusTypeDef status;
    status = HAL_I2C_Mem_Write_DMA( hi2c_device, chipAdd, regAdd, 1, data, length);
	
	if(HAL_OK != status)
	{
		return SENSOR_ERROR;
	}
	else{
		return SENSOR_OK;
	}
}
