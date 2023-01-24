#include "_I2C.h"
#include "_TIM_CONFIG.h"


#define BMP180_ADDRESS		(0x77)
#define MPU6050_ADDRESS	0x68
#define ACCEL_XOUT_H	0x3B
#define GYRO_XOUT_H		0x43

void setupMPU(void);
void recordAccelRegisters(char* ptr, int16_t* ptr1, int16_t* ptr2, int16_t* ptr3);
float* processAccelData(int16_t* ptr1, int16_t* ptr2, int16_t* ptr3);

int main(void){

	char rawData[6];
	int16_t accelX,accelY,accelZ;
	float ac[3];
	float *pointer;
	
	initI2C1();
	setupMPU();
	
	while(1){
		recordAccelRegisters(rawData,&accelX,&accelY,&accelZ);
		pointer = processAccelData(&accelX,&accelY,&accelZ);
		ac[0] = *pointer;
		ac[1] = *(pointer + 1);
		ac[2] = *(pointer + 2);
	}
}

void setupMPU(void){
	i2c_write_single(MPU6050_ADDRESS,0x6B,0);
	i2c_write_single(MPU6050_ADDRESS,0x1B,0);
	i2c_write_single(MPU6050_ADDRESS,0x1C,0);	
}

void recordAccelRegisters(char* ptr, int16_t* ptr1, int16_t* ptr2, int16_t* ptr3){
	i2c_readMULT(MPU6050_ADDRESS,0x3B,ptr,6);
	*ptr1 = (int16_t)(ptr[0] << 8) | ptr[1];
	*ptr2 = (int16_t)(ptr[2] << 8) | ptr[3];
	*ptr3 = (int16_t)(ptr[4] << 8) | ptr[5];
}
float* processAccelData(int16_t* ptr1, int16_t* ptr2, int16_t* ptr3){
	float ax[3];
	ax[0] = (float)*ptr1 / 16384.0f;
	ax[1] = (float)*ptr2 / 16384.0f;
	ax[2] = (float)*ptr3 / 16384.0f;
	return ax;
}
	
