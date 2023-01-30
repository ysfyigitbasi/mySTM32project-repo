#include "_BMP388.h"

#include "_I2C.h"
#include "math.h"

#define PRESS_EN 	(uint8_t)1	// Set power control register to enable pressure sensor
#define TEMP_EN 	(uint8_t)1	// Set power control register to enable temperature sensor

uint8_t begin(uint8_t mode, uint8_t iirFilter, uint8_t timeStand, uint8_t presOSR, uint8_t tempOSR){
	
	initI2C1();				
	if (!reset())	// Reset the BMP388 barometer
	{		
		return 0;				// If unable to reset return 0
	}
	char chipId;
	chipId = i2c_read_single(BMP388_I2C_ADDR,BMP388_CHIP_ID);		// Read the device ID	
	if (chipId != BMP388_ID && chipId != BMP390_ID)            		// Check the device ID  
		return 0;                                                 	// If the ID is incorrect return 0
	
	calcCoefPARAM();
	
	setIIRFilter(iirFilter);	// Initialise the BMP388 IIR filter register
	setTimeStandby(timeStand); 	// Initialise the BMP388 standby time register
	setOversamplingRegister(presOSR, tempOSR);	// Initialise the BMP388 oversampling register
	setMode(mode);				// Set the BMP388 mode
	return 1;					// Report successful initialisation
}

void calcCoefPARAM(void){
	// Read the trim parameters into the params structure
	i2c_readMULT(BMP388_I2C_ADDR, BMP388_TRIM_PARAMS, (uint8_t*)&calibINT, sizeof(calibINT));
	// Calculate the floating point trim parameters
	calibFLOAT.param_T1 = (double)((int32_t)calibINT.param_T1 << 8);
	calibFLOAT.param_T2 = (double)calibINT.param_T2 / (int32_t)(1 << 30);
	calibFLOAT.param_T3 = (double)calibINT.param_T3 / (int64_t)(1 << 48);
	calibFLOAT.param_P1 = (double)(calibINT.param_P1 - (int16_t)(1 << 14)) / (int32_t)(1 << 20);
	calibFLOAT.param_P2 = (double)(calibINT.param_P2 - (int16_t)(1 << 14)) / (int32_t)(1 << 29);
	calibFLOAT.param_P3 = (double)calibINT.param_P3 / (int64_t)(1 << 32);
	calibFLOAT.param_P4 = (double)calibINT.param_P4 / (int64_t)(1 << 37);
	calibFLOAT.param_P5 = (double)((int32_t)calibINT.param_P5 << 3);
	calibFLOAT.param_P6 = (double)calibINT.param_P6 / (int8_t)(1 << 6);
	calibFLOAT.param_P7 = (double)calibINT.param_P7 / (int16_t)(1 << 8);
	calibFLOAT.param_P8 = (double)calibINT.param_P8 / (int32_t)(1 << 15);
	calibFLOAT.param_P9 = (double)calibINT.param_P9 / (int64_t)(1 << 48);
	calibFLOAT.param_P10 = (double)calibINT.param_P10 / (int64_t)(1 << 48);
	calibFLOAT.param_P11 = (double)(calibINT.param_P11 / (int8_t) (1 << 5)) / (int64_t)(1 << 60);
}

void setMode(enum Mode mode){
	
	uint8_t modeSelect = (mode << 4) | (0x03) ; //Mode is 2 bit, shifting 4 times to (write temp_en ve pres_en denenecek)
	i2c_write_single(BMP388_I2C_ADDR, BMP388_PWR_CTRL, modeSelect); //Using I2C write mode to the PWR_CTRL register
	// 5 ve 4. bite 
}

void setIIRFilter(enum IIRFilter iirCoef){
	uint8_t iirCoefSelect = iirCoef << 1;
	//iirCoef 3 bit, 3 2 1 e yazilacak 1 bit kaydirmak yeter
	i2c_write_single(BMP388_I2C_ADDR, BMP388_CONFIG, iirCoefSelect);
}

void setTimeStandby(enum TimeStandby standby_t){

	i2c_write_single(BMP388_I2C_ADDR,BMP388_ODR, standby_t);
	//ODR_SEL 4 3 2 1 0 a yazilacak
}

void setOversamplingRegister(enum Oversampling pressOSR, enum Oversampling tempOSR){
	
	uint8_t overSamplingSelector = (pressOSR) | (tempOSR<<3);
	i2c_write_single(BMP388_I2C_ADDR, BMP388_OSR, overSamplingSelector);
	
}

void setInterrupts(enum OutputDrive gpio, enum ActiveLevel active_L_H, enum LatchConfig irqLatch){

	uint8_t interruptBits = (irqLatch << 2) | (active_L_H << 1) | (gpio) | (1<<6);
	i2c_write_single(BMP388_I2C_ADDR, BMP388_INT_CTRL, interruptBits);
}

uint8_t reset(void){

	i2c_write_single(BMP388_I2C_ADDR,BMP388_CMD, RESET_CODE);
	return 1;
}

static uint8_t dataReady(void){
	
	static char readySignal;
	readySignal	= i2c_read_single(BMP388_I2C_ADDR,BMP388_INT_STATUS);
	
	return (readySignal & (1<<3)); //data ready demek sleep mode check yapilacak. ysf: gerek yok gibi, normal modda calistiricaz.
}

uint8_t getTempPres(double* temperature,	double* pressure){

	if(!dataReady()){
		return 0;
	}
	char data[6];
	i2c_readMULT(BMP388_I2C_ADDR, BMP388_DATA_0, data, 6);

	int32_t adcPress = (	((int32_t)data[2]) << 16 ) | (	((int32_t)data[1]) << 8 ) | ((int32_t)data[0]);
	int32_t adcTemp  = (	((int32_t)data[5]) << 16 ) | (	((int32_t)data[4]) << 8 ) | ((int32_t)data[3]);
	
	*temperature = bmp388_compensate_temp(adcTemp);
	*pressure = bmp388_compensate_press(adcPress, *temperature);  
	
	return 1;
}	

int8_t getAltitude(double* altitude){
	
	double *pressure;
	double *temperature;
	
	if( !(getTempPres(temperature,pressure)) )
		return -1;
																			
	
	*altitude =(pow(sea_level_pressure / (*pressure), 1.0/5.257) - 1.0) * ((*temperature) + 273.15) / 0.0065; 
	return 1;
}
//buranin asagisinda calibFloat kullandik adam floatParams kullanmis
//ama ayni struct yapisi var isim karistirmasin

double bmp388_compensate_temp(int32_t uncomp_temp){
	
	double partial_data1 = (double)uncomp_temp - calibFLOAT.param_T1;
	double partial_data2 = partial_data1 * calibFLOAT.param_T2;
	
	return partial_data2 + partial_data1 * partial_data1 * calibFLOAT.param_T3;
}


double bmp388_compensate_press(int32_t uncomp_press,double t_lin){
	
	double partial_data1, partial_data2, partial_data3, partial_data4, partial_out1,partial_out2;

	partial_data1 = calibFLOAT.param_P6 * t_lin;
	partial_data2 = calibFLOAT.param_P7 * t_lin * t_lin;
	partial_data3 = calibFLOAT.param_P8 * t_lin * t_lin * t_lin;
	partial_out1 = calibFLOAT.param_P5 + partial_data1 + partial_data2 + partial_data3;
	
	partial_data1 = calibFLOAT.param_P2 * t_lin;
	partial_data2 = calibFLOAT.param_P3 * t_lin * t_lin;
	partial_data3 = calibFLOAT.param_P4 * t_lin * t_lin * t_lin;
	
	partial_out2 = (double)uncomp_press * (calibFLOAT.param_P1 + 
											partial_data1 + 
											partial_data2 + 
											partial_data3);
	
	partial_data1 = (double)uncomp_press * (double)uncomp_press;
	partial_data2 = calibFLOAT.param_P9 + calibFLOAT.param_P10 * t_lin;
	partial_data3 = partial_data1 * partial_data2;
	
	partial_data4 = partial_data3 + (double)uncomp_press * 
									(double)uncomp_press * 
									(double)uncomp_press * 
									calibFLOAT.param_P11;
	
	return (partial_out1 + partial_out2 + partial_data4);
	
}

