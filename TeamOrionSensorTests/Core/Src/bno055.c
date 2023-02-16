#include "bno055.h"
#include <string.h>

uint16_t accelScale = 100;
uint16_t tempScale = 1;
uint16_t angularRateScale = 16;
uint16_t eulerScale = 16;
uint16_t magScale = 16;
uint16_t quaScale = (1<<14);    // 2^14

void bno055_setPage(uint8_t page) { bno055_writeData(BNO055_PAGE_ID, page); }

bno055_opmode_t bno055_getOperationMode() {
	bno055_opmode_t mode;
	bno055_readData(BNO055_OPR_MODE, &mode, 1);
	return mode;
}

void bno055_setOperationMode(bno055_opmode_t mode) {
	
	bno055_writeData(BNO055_OPR_MODE, mode);
	if (mode == BNO055_OPERATION_MODE_CONFIG) {
		bno055_delay(19);
	} else {
		bno055_delay(7);
	}
}

//void bno055_setExternalCrystalUse(bool state) {
//  bno055_setPage(0);
//  uint8_t tmp = 0;
//  bno055_readData(BNO055_SYS_TRIGGER, &tmp, 1);
//  tmp |= (state == true) ? 0x80 : 0x0;
//  bno055_writeData(BNO055_SYS_TRIGGER, tmp);
//  bno055_delay(700);
//}

//void bno055_enableExternalCrystal() { bno055_setExternalCrystalUse(true); }
//void bno055_disableExternalCrystal() { bno055_setExternalCrystalUse(false); }

uint8_t bno055_reset() {
	
	uint8_t temp = 0;
	bno055_setPage(0);
	
	// external clock set bit7, system reset bit 5.
	bno055_writeData(BNO055_SYS_TRIGGER, 0xA0);
	bno055_delay(800);
	
	temp = bno055_getSystemError();
	if( temp != 0)
	{// error found.
		printf("\nSystem Error has occured during Power on reset. %d number",temp);
		return temp;
	}
	temp = bno055_getSelfTestResult();	
	temp &= 0x0F;
	if( temp != 0x0F)
	{
		printf("\n Self test is failed: %d .",temp);
		return temp;
	}
	
	return 0;	
}

void bno055_setup() {
	bno055_reset();

	uint8_t id = 0;
	bno055_readData(BNO055_CHIP_ID, &id, 1);
	if (id != BNO055_ID) {
		printf("Can't find BNO055, id: 0x%02x. Please check your wiring.\r\n", id);
	}
	bno055_setPage(0);
	bno055_writeData(BNO055_SYS_TRIGGER, 0x81); // External Clock bit7

	// Select BNO055 config mode
	bno055_setOperationMode(BNO055_OPERATION_MODE_CONFIG);
	bno055_delay(10);
}

uint8_t bno055_getCalibrationState() {
	uint8_t calState = 0;
	bno055_setPage(0);
  
	bno055_readData(BNO055_CALIB_STAT, &calState, 1);
	
	switch(calState){
		case 0xFF:
			printf("\nMCU-GYR-MAG-ACC is calibrated");
			calState = 0;
			break;
		case 0x3F:
			printf("\n MCU is not calibrated yet..");
			calState = 1;
			break;
		case 0x0F:
			printf("\n MCU and GYR are not calibrated yet !!!");
			calState = 2;
			break;
		case 0x03:
			printf("\n MCU-GYR-ACC are not calibrated yet !!!");
			calState = 3;
			break;
		case 0x00:
			printf("\n No calibration is detected !!!");
			calState = 4;
			break;
		default:
			printf("\n Unknown CALIBRATION STATE !!!!");
			calState = 5;
			break;		
	}	
  return calState;
}


bno055_calibration_data_t bno055_getCalibrationData() {
  bno055_calibration_data_t calData;
  uint8_t buffer[22];
  bno055_opmode_t operationMode = bno055_getOperationMode();
  bno055_setOperationModeConfig();
  bno055_setPage(0);

  bno055_readData(BNO055_ACC_OFFSET_X_LSB, buffer, 22);

  // Assumes little endian processor
  memcpy(&calData.offset.accel, buffer, 6);
  memcpy(&calData.offset.mag, buffer + 6, 6);
  memcpy(&calData.offset.gyro, buffer + 12, 6);
  memcpy(&calData.radius.accel, buffer + 18, 2);
  memcpy(&calData.radius.mag, buffer + 20, 2);

  bno055_setOperationMode(operationMode);

  return calData;
}

void bno055_setCalibrationData(bno055_calibration_data_t calData) {
  uint8_t buffer[22];
  bno055_opmode_t operationMode = bno055_getOperationMode();
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
uint8_t bno055_getSystemError() {
  //bno055_setPage(0);
  uint8_t tmp;
  bno055_readData(BNO055_SYS_ERR, &tmp, 1);
  return tmp;
}

//uint8_t bno055_getSystemStatus() {
//  bno055_setPage(0);
//  uint8_t tmp;
//  bno055_readData(BNO055_SYS_STATUS, &tmp, 1);
//  return tmp;
//}

//int8_t bno055_getTemp() {
//  bno055_setPage(0);
//  uint8_t t;
//  bno055_readData(BNO055_TEMP, &t, 1);
//  return t;
//}

//int16_t bno055_getSWRevision() {
//  bno055_setPage(0);
//  uint8_t buffer[2];
//  bno055_readData(BNO055_SW_REV_ID_LSB, buffer, 2);
//  return (int16_t)((buffer[1] << 8) | buffer[0]);
//}

//uint8_t bno055_getBootloaderRevision() {
//  bno055_setPage(0);
//  uint8_t tmp;
//  bno055_readData(BNO055_BL_REV_ID, &tmp, 1);
//  return tmp;
//}
