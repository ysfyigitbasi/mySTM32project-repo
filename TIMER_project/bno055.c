#include "bno055.h"
#include "_I2C.h"
#include "_TIM_CONFIG.h"
#include <string.h>

uint16_t accelScale = 100;
uint16_t tempScale = 1;
uint16_t angularRateScale = 16;
uint16_t eulerScale = 16;
uint16_t magScale = 16;
uint16_t quaScale = (1<<14);    // 2^14

int8_t bno055_setup(enum bno055_opmode_t opmode){

	char chip_id = 0x00;
	#ifndef LAUNCH
		bno055_reset();
	#endif
	chip_id = i2c_read_single(BNO055_I2C_ADDR, BNO055_CHIP_ID);
	
	if (chip_id != BNO055_ID)
		return -1;
	
	setOperationMode(opmode);
	
	
	return 1;
}

void setPage(uint8_t page) { i2c_write_single(BNO055_I2C_ADDR, BNO055_PAGE_ID, page);	}


enum bno055_opmode_t getOperationMode(void) {  return i2c_read_single(BNO055_I2C_ADDR,BNO055_OPR_MODE); }

void setOperationMode(enum bno055_opmode_t mode) {
  
	i2c_write_single(BNO055_I2C_ADDR, BNO055_OPR_MODE, mode);
	if (mode == BNO055_OPERATION_MODE_CONFIG) {
		delayMS(22); // 19+3
	}
	else {
		delayMS(10); // 7+3
	}
}

void enableExternalCrystal(void) {	
	char temp;
	temp = i2c_read_single(BNO055_I2C_ADDR, BNO055_SYS_TRIGGER);
	temp |= 0x80;
	i2c_write_single(BNO055_I2C_ADDR, BNO055_SYS_TRIGGER, temp);
}

void disableExternalCrystal(void) {	
	char temp;
	temp = i2c_read_single(BNO055_I2C_ADDR, BNO055_SYS_TRIGGER);
	temp &= ~0x80;
	i2c_write_single(BNO055_I2C_ADDR, BNO055_SYS_TRIGGER, temp);
}

void bno055_reset(void) {
	i2c_write_single(BNO055_I2C_ADDR, BNO055_SYS_TRIGGER, 0x20);
	delayMS(700);
}

int8_t getTemp(void) {
	setPage(0);
	return (int8_t)i2c_read_single(BNO055_I2C_ADDR, BNO055_TEMP);
}

int16_t getSWRevision(void) {
	char buffer[2];
	setPage(0);
	i2c_readMULT(BNO055_I2C_ADDR, BNO055_SW_REV_ID_LSB, buffer, 2);
	return (int16_t)((buffer[1] << 8) | buffer[0]);
}

uint8_t getBootloaderRevision(void) {
	setPage(0);
	return i2c_read_single(BNO055_I2C_ADDR,BNO055_BL_REV_ID);
}

enum bno055_system_status_t getSystemStatus(void) {
	setPage(0);
	return i2c_read_single(BNO055_I2C_ADDR, BNO055_SYS_STATUS);
}

uint8_t getSelfTestResult(void) {
	char temp;
	setPage(0);
	temp = i2c_read_single(BNO055_I2C_ADDR,BNO055_ST_RESULT);
	temp &= 0x0F; // use last four bits: 4mcu, 3gyr, 2mag, 1acc
  
	return temp;
}

enum bno055_system_error_t getSystemError(void) {
	
	if( getSystemStatus() == BNO055_SYSTEM_STATUS_SYSTEM_ERROR ){
		char temp;
		temp = i2c_read_single(BNO055_I2C_ADDR, BNO055_SYS_ERR);
		return temp;
	}
	return BNO055_SYSTEM_ERROR_NO_ERROR;
}

uint8_t getCalibrationState(void) {
  
	char calState;
	setPage(0);
	
	calState = i2c_read_single( BNO055_I2C_ADDR, BNO055_CALIB_STAT);
  
	return calState; // 7->11-sys, 11-gyr, 11-acc, 11-mag ->0.
}


bno055_calibration_data_t bno055_getCalibrationData() {
	// After every power on reset 
	bno055_calibration_data_t calData;
	uint8_t buffer[22];
	
	if( wait4Calibration() != -1){
		
		bno055_readData(BNO055_ACC_OFFSET_X_LSB, buffer, 22);

  // Assumes little endian processor. memcpy degil direkt atama yapilacak kaydirmali. x << 8 | y gibi.
  memcpy(&calData.offset.accel, buffer, 6);
  memcpy(&calData.offset.mag, buffer + 6, 6);
  memcpy(&calData.offset.gyro, buffer + 12, 6);
  memcpy(&calData.radius.accel, buffer + 18, 2);
  memcpy(&calData.radius.mag, buffer + 20, 2);
		
	}


  return calData;
}

void bno055_setCalibrationData(bno055_calibration_data_t calData) {
  uint8_t buffer[22];
  operationMode = bno055_getOperationMode();
  bno055_setOperationModeConfig();
  bno055_setPage(0);

  // Assumes litle endian processor
  memcpy(buffer, &calData.offset.accel, 6);
  memcpy(buffer + 6, &calData.offset.mag, 6);
  memcpy(buffer + 12, &calData.offset.gyro, 6);
  memcpy(buffer + 18, &calData.radius.accel, 2);
  memcpy(buffer + 20, &calData.radius.mag, 2);

  for (uint8_t i=0; i < 22; i++) {
    // TODO(oliv4945): create multibytes write
    bno055_writeData(BNO055_ACC_OFFSET_X_LSB+i, buffer[i]);
  }

  bno055_setOperationMode(operationMode);
}

int8_t wait4Calibration( uint8_t internalDelay, uint8_t mainDelay){
	
	uint8_t myCalibration = 0;
	uint8_t out = 0, in1 = 0, in2 = 0, in3 = 0;
	
	myCalibration = getCalibrationState();
	while( myCalibration != CALIBRATED_FULLY )
	{
		while( !(myCalibration & CALIBRATED_ACC) ){
			if(++in1 > 254)
				return -1;
			myCalibration = getCalibrationState();
			//printf(" Calibration ACC continues...");
			delayMS(internalDelay);
		}
		while( !(myCalibration & CALIBRATED_GYRO) ){
			if(++in2 > 254)
				return -1;
			myCalibration = getCalibrationState();
			//printf(" Calibration GYRO continues...");
			delayMS(internalDelay);
		}
		while( !(myCalibration & CALIBRATED_MAG) ){y
			if(++in3 > 254)
				return -1;
			myCalibration = getCalibrationState();
			//printf(" Calibration MAGG continues...");
			delayMS(internalDelay);
		}
		if( ++out > 254)
			return -1;
		
		delayMS(mainDelay);
		myCalibration = getCalibrationState();
	}
	
	return 1;
}

bno055_vector_t bno055_getVector(uint8_t vec) {
  bno055_setPage(0);
  uint8_t buffer[8];    // Quaternion need 8 bytes

  if (vec == BNO055_VECTOR_QUATERNION)
    bno055_readData(vec, buffer, 8);
  else
    bno055_readData(vec, buffer, 6);

  double scale = 1;

  if (vec == BNO055_VECTOR_MAGNETOMETER) {
    scale = magScale;
  } else if (vec == BNO055_VECTOR_ACCELEROMETER ||
           vec == BNO055_VECTOR_LINEARACCEL || vec == BNO055_VECTOR_GRAVITY) {
    scale = accelScale;
  } else if (vec == BNO055_VECTOR_GYROSCOPE) {
    scale = angularRateScale;
  } else if (vec == BNO055_VECTOR_EULER) {
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

bno055_vector_t bno055_getVectorAccelerometer() {
  return bno055_getVector(BNO055_VECTOR_ACCELEROMETER);
}
bno055_vector_t bno055_getVectorMagnetometer() {
  return bno055_getVector(BNO055_VECTOR_MAGNETOMETER);
}
bno055_vector_t bno055_getVectorGyroscope() {
  return bno055_getVector(BNO055_VECTOR_GYROSCOPE);
}
bno055_vector_t bno055_getVectorEuler() {
  return bno055_getVector(BNO055_VECTOR_EULER);
}
bno055_vector_t bno055_getVectorLinearAccel() {
  return bno055_getVector(BNO055_VECTOR_LINEARACCEL);
}
bno055_vector_t bno055_getVectorGravity() {
  return bno055_getVector(BNO055_VECTOR_GRAVITY);
}
bno055_vector_t bno055_getVectorQuaternion() {
  return bno055_getVector(BNO055_VECTOR_QUATERNION);
}

void bno055_setAxisMap(bno055_axis_map_t axis) {
  uint8_t axisRemap = (axis.z << 4) | (axis.y << 2) | (axis.x);
  uint8_t axisMapSign = (axis.x_sign << 2) | (axis.y_sign << 1) | (axis.z_sign);
  bno055_writeData(BNO055_AXIS_MAP_CONFIG, axisRemap);
  bno055_writeData(BNO055_AXIS_MAP_SIGN, axisMapSign);
}
