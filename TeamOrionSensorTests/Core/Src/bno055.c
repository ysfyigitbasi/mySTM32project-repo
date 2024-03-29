#include "bno055.h"
#include <string.h>
#include "I2C_sensorDriver.h"

//uint16_t accelScale = 100;
//uint16_t tempScale = 1;
//uint16_t angularRateScale = 16;
uint16_t eulerScale = 16;
//uint16_t magScale = 16;
uint16_t quaScale = (1<<14);    // 2^14

uint8_t calibrationDATA[22] = {};

uint8_t bno055_setPage(uint8_t page)
{
	sensor_status_e i2c_status;
	i2c_status = write8bit(&hi2c2, BNO055_I2C_ADDR, BNO055_PAGE_ID, page);
	if(SENSOR_OK != i2c_status)
	{
//		printf("\n I2C write ERROR in setOperationMode!!!");
		return 1; // ERROR STATE
	}
	return 0;

}

bno055_opmode_t bno055_getOperationMode() {
	bno055_opmode_t mode;
	mode = read8bit(&hi2c2, BNO055_I2C_ADDR, BNO055_OPR_MODE);
	return mode;
}

uint8_t bno055_setOperationMode(bno055_opmode_t mode) {
	sensor_status_e i2c_status;
	i2c_status = write8bit(&hi2c2, BNO055_I2C_ADDR, BNO055_OPR_MODE, mode);
	if(SENSOR_OK != i2c_status)
	{
//		printf("\n I2C write ERROR in setOperationMode!!!");
		return 1; // ERROR STATE
	}
	if (mode == BNO055_OPERATION_MODE_CONFIG) {
		HAL_Delay(19);
	} else {
		HAL_Delay(7);
	}
	return 0;
}

uint8_t bno055_getSelfTestResult(void)
{
	uint8_t selfTestState = 0;
	uint8_t temp = 0;
	selfTestState = read8bit(&hi2c2, BNO055_I2C_ADDR, BNO055_ST_RESULT);	
	
	if( !((selfTestState >> 3) & 0x01) )
	{
//		printf("\nMCU self test failed!!");
		temp = 1;
	}
	if( !((selfTestState >> 2) & 0x01) )
	{
//		printf("\nGYRO self test failed!!");
		temp = 1;
	}
	if( !((selfTestState >> 1) & 0x01) )
	{
//		printf("\nMAG self test failed!!");
		temp = 1;
	}
	if( !((selfTestState) & 0x01) )
	{
//		printf("\nACC self test failed!!");
		temp = 1;
	}
	if( temp != 0)
		return selfTestState | (0x10);
	
	return 0;
}

uint8_t bno055_reset() {
	
	uint8_t temp = 0;
	sensor_status_e i2c_status;
	temp = bno055_setPage(0);
	if( temp != 0)
	{
//		printf("\nSET PAGE I2C ERROR");
		return 1;
	}
	
	// external clock set bit7, system reset bit 5.
	i2c_status = write8bit(&hi2c2, BNO055_I2C_ADDR, BNO055_SYS_TRIGGER, 0xA1);
	if(SENSOR_OK != i2c_status)
	{
//		printf("\n I2C write ERROR!!!");
		return 2; // ERROR STATE
	}
	HAL_Delay(800);
	
	temp = bno055_getSystemError();
	if( temp != 0)
	{// error found.
//		printf("\nSystem Error has occured during Power on reset. %d number",temp);
		return 3;
	}
	temp = bno055_getSelfTestResult();	
	
	if( temp != 0)
	{
//		printf("\n Self test is failed: %d .",temp);
		return 4;
	}
	
	return 0;	
}

uint8_t bno055_setup() {
	uint8_t status = 0;
	sensor_status_e i2c_status;
	status = bno055_reset();
	if (status != 0)
	{
//		printf("\nReset is failed!!!");
		return 1;
	}
	
	uint8_t id = 0;
	id = read8bit(&hi2c2, BNO055_I2C_ADDR, BNO055_CHIP_ID);
	
	if (id != BNO055_ID) {
//		printf("Can't find BNO055, id: 0x%02x. Please check your wiring.\r\n", id);
		return 2;
	}

	i2c_status = write8bit(&hi2c2, BNO055_I2C_ADDR, BNO055_SYS_TRIGGER, 0x80);
	if(SENSOR_OK != i2c_status)
	{
//		printf("\n I2C write ERROR!!!");
		return 3; // ERROR STATE
	}
	
	status = bno055_setOperationMode(BNO055_OPERATION_MODE_NDOF);
	if (status != 0)
		return 4;
	HAL_Delay(10);
	status = 1;
	while( status != 0 )
	{
		status = bno055_getCalibrationState();
		HAL_Delay(1);
	}
	bno055_getCalibrationData(calibrationDATA);
	
	return 0;
}

uint8_t bno055_getCalibrationState() {
	uint8_t calState = 0;
	bno055_setPage(0);
  
	calState = read8bit(&hi2c2, BNO055_I2C_ADDR, BNO055_CALIB_STAT);	
	if( (calState >> 6) == 0x03)
	{		
		return 0;
	}
	else{
		// in fusion mode, not calibrated.	
		switch(calState){
			case 0x3F:
//				printf("\n MCU is not calibrated yet..");
				calState = 1;
				break;
			case 0x0F:
//				printf("\n MCU and GYR are not calibrated yet !!!");
				calState = 2;
				break;
			case 0x03:
//				printf("\n MCU-GYR-ACC are not calibrated yet !!!");
				calState = 3;
				break;
			case 0x00:
//				printf("\n No calibration is detected !!!");
				calState = 4;
				break;
			default:
//				printf("\n Unknown CALIBRATION STATE !!!!");
				calState = 5;
				break;		
		}
		return calState;
	}		
}

uint8_t bno055_getCalibrationData(uint8_t* calData) {
	uint8_t temp = 0;
	sensor_status_e status;
	
	bno055_opmode_t operationMode = bno055_getOperationMode();
	
	temp = bno055_setOperationMode(BNO055_OPERATION_MODE_CONFIG);
	
	if (temp != 0)	{	return 1; }
	bno055_setPage(0);

	status = readMultBytesWNoDMA(&hi2c2, BNO055_I2C_ADDR, BNO055_ACC_OFFSET_X_LSB, calData, 22);
	//HAL_Delay(4);
	if(SENSOR_OK != status)
	{
//		printf("\n I2C_DMA error in getCalibrationData");
		return 2;
	}
	
	temp = bno055_setOperationMode(operationMode);
	if (temp != 0)	{	return 3; }
  
	return 0;
}

uint8_t bno055_setCalibrationData(uint8_t* calData) {
	uint8_t temp = 0;
	sensor_status_e status;
	
	bno055_opmode_t operationMode = bno055_getOperationMode();
	temp = bno055_setOperationMode(BNO055_OPERATION_MODE_CONFIG);
	
	if (temp != 0)	{	return 1; }

	status = writeMultBytes(&hi2c2, BNO055_I2C_ADDR, BNO055_ACC_OFFSET_X_LSB, calData, 22);
	//HAL_Delay(20);
	if(SENSOR_OK != status)
	{
//		printf("\n I2C_DMA error in getCalibrationData");
		return 2;
	}
	temp = bno055_setOperationMode(operationMode);
	if (temp != 0)	{	return 3; }
	return 0;
}

bno055_vector_t bno055_getVector(uint8_t vec) {
	uint8_t buffer[8];    // Quaternion need 8 bytes
	sensor_status_e status;

	if (vec == BNO055_VECTOR_QUATERNION)
	{
		status = readMultBytesWNoDMA(&hi2c2, BNO055_I2C_ADDR, vec, buffer, 8);
		//HAL_Delay(10);
		if( SENSOR_OK != status)
		{
			int number = 5;
//			printf("\nI2C readMult error in vector_quaternion!!");
		}
	}
	else
	{
		status = readMultBytesWNoDMA(&hi2c2, BNO055_I2C_ADDR, vec, buffer, 6);
		//HAL_Delay(10);
		if( SENSOR_OK != status)
		{
//			printf("\nI2C readMult error in vector_quaternion!!");
			int number = 6;
		}
	}

	double scale = 1;

//	if (vec == BNO055_VECTOR_MAGNETOMETER) {
//		scale = magScale;
//	} else if (vec == BNO055_VECTOR_ACCELEROMETER || vec == BNO055_VECTOR_LINEARACCEL || vec == BNO055_VECTOR_GRAVITY) {
//		scale = accelScale;
//	} else if (vec == BNO055_VECTOR_GYROSCOPE) {
//		scale = angularRateScale;
	if (vec == BNO055_VECTOR_EULER) {
		scale = eulerScale;
	} else if (vec == BNO055_VECTOR_QUATERNION) {
		scale = quaScale;
	}

	bno055_vector_t xyz = {.w = 0, .x = 0, .y = 0, .z = 0};
	if (vec == BNO055_VECTOR_QUATERNION) {
		xyz.w = (int16_t)((buffer[1] << 8) | buffer[0]) / scale;
		xyz.x = (int16_t)((buffer[3] << 8) | buffer[2]) / scale;
		xyz.y = (int16_t)((buffer[5] << 8) | buffer[4]) / scale;
		xyz.z = (int16_t)((buffer[7] << 8) | buffer[6]) / scale;
	} else {
		xyz.x = (int16_t)((buffer[1] << 8) | buffer[0]) / scale;
		xyz.y = (int16_t)((buffer[3] << 8) | buffer[2]) / scale;
		xyz.z = (int16_t)((buffer[5] << 8) | buffer[4]) / scale;
	}

	return xyz;
}

uint8_t bno055_getSystemError() {
	uint8_t temp;
	temp = read8bit(&hi2c2, BNO055_I2C_ADDR, BNO055_SYS_ERR);
	return temp;
}

//void bno055_setAxisMap(bno055_axis_map_t axis) {
//	uint8_t axisRemap = (axis.z << 4) | (axis.y << 2) | (axis.x);
//	uint8_t axisMapSign = (axis.x_sign << 2) | (axis.y_sign << 1) | (axis.z_sign);
//	bno055_writeData(BNO055_AXIS_MAP_CONFIG, axisRemap);
//	bno055_writeData(BNO055_AXIS_MAP_SIGN, axisMapSign);
//}

//uint8_t bno055_getSystemStatus() {
//  bno055_setPage(0);
//  uint8_t tmp;
//  bno055_readData(BNO055_SYS_STATUS, &tmp, 1);
//  return tmp;
//}

