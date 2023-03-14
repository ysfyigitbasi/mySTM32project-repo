#include "_BMP388.h"
#include "math.h"

uint8_t BMP388_TestSensor(void){
	
	HAL_StatusTypeDef status;
	status = HAL_I2C_IsDeviceReady(&hi2c1, BMP388_ADDR , 4, 100);
	if( status == HAL_OK)
	{
		return 1;
	}
	else if( status == HAL_ERROR){
		return 2;	
	}
	else if( status == HAL_BUSY){
		return 3;
	}
	else{
		return 4;
	}
}

uint8_t BMP388_ReadID(void){
	
	uint8_t idVal = 0;
	HAL_StatusTypeDef status2;
	
	status2 = HAL_I2C_Mem_Read(&hi2c1, BMP388_ADDR, BMP388_CHIP_ID, 1, &idVal, 1, 100);
	
	if( HAL_OK == status2){
		
		if( BMP388_ID == idVal){
			return 1;
		}
	}
	return 0;
}
void setMode(enum Mode modeX){
	// 5 ve 4. bite	
	uint8_t modeSelect = (uint8_t)(modeX << 4) | (0x03) ; //Mode is 2 bit, shifting 4 times to (write temp_en ve pres_en denenecek)
	write8bit(&hi2c1, BMP388_ADDR, BMP388_PWR_CTRL, modeSelect); //Using I2C write mode to the PWR_CTRL register
 
}
void setIIRFilter(enum IIRFilter iirCoef){
	uint8_t iirCoefSelect = (uint8_t)(iirCoef << 1);
	//iirCoef 3 bit, 3 2 1 e yazilacak 1 bit kaydirmak yeter
	write8bit(&hi2c1, BMP388_ADDR, BMP388_CONFIG, iirCoefSelect);
}
void setTimeStandby(enum TimeStandby standby_t){
	write8bit(&hi2c1, BMP388_ADDR,BMP388_ODR, standby_t);
	//ODR_SEL 4 3 2 1 0 a yazilacak
}
void setOversamplingRegister(enum Oversampling pressOSR, enum Oversampling tempOSR){
	uint8_t overSamplingSelector = (uint8_t)( pressOSR | (tempOSR<<3));
	write8bit(&hi2c1, BMP388_ADDR, BMP388_OSR, overSamplingSelector);
}
uint8_t begin(uint8_t modeX, uint8_t iirFilterX, uint8_t timeStand, uint8_t presOSR, uint8_t tempOSR){
	uint8_t chipId;
	if (!reset())	// Reset the BMP388 barometer
	{		
		return 0;				// If unable to reset return 0
	}

	chipId = read8bit(&hi2c1, BMP388_ADDR, BMP388_CHIP_ID);		// Read the device ID
	
	if (chipId != BMP388_ID && chipId != BMP390_ID){            		// Check the device ID  
		return 0;                                                 	// If the ID is incorrect return 0
	}
	setMode(modeX);				// Set the BMP388 mode
	setIIRFilter(iirFilterX);	// Initialise the BMP388 IIR filter register
	setTimeStandby(timeStand); 	// Initialise the BMP388 standby time register
	setOversamplingRegister(presOSR, tempOSR);	// Initialise the BMP388 oversampling register	
	return 1;					// Report successful initialisation
}

uint8_t reset(void){

	return write8bit(&hi2c1, BMP388_ADDR,BMP388_CMD, RESET_CODE);
}

uint8_t readCoef(void){

	uint8_t array[21] = {};
//	uint8_t status;
//	status = readMultBytes(BMP388_ADDR, BMP388_TRIM_PARAMS, array, 21);
//	if(!status)
//		return 0;
	//HAL_I2C_Mem_Read_DMA(&hi2c1, BMP388_ADDR, BMP388_TRIM_PARAMS, 1, array, 21);
	readMultBytes(&hi2c1, BMP388_ADDR, BMP388_TRIM_PARAMS, array, 21);
	HAL_Delay(4);
	calibINT.param_T1 = ( ( (uint16_t)array[1]) << 8) | array[0];
	calibINT.param_T2 = ( ( (uint16_t)array[3]) << 8) | array[2];
	calibINT.param_T3 = (int8_t)array[4];
	calibINT.param_P1 = ( ( (int16_t)array[6]) << 8) | array[5];
	calibINT.param_P2 = ( ( (int16_t)array[8]) << 8) | array[7];
	calibINT.param_P3 = (int8_t)array[9];
	calibINT.param_P4 = (int8_t)array[10];
	calibINT.param_P5 = ( ( (uint16_t)array[12]) << 8) | array[11];
	calibINT.param_P6 = ( ( (uint16_t)array[14]) << 8) | array[13];
	calibINT.param_P7 = (int8_t)array[15];
	calibINT.param_P8 = (int8_t)array[16];
	calibINT.param_P9 = ( ( (int16_t)array[18]) << 8) | array[17];
	calibINT.param_P10 = (int8_t)array[19];
	calibINT.param_P11 = (int8_t)array[20];
	return 1;
}

//void setInterrupts(enum OutputDrive gpio, enum ActiveLevel active_L_H, enum LatchConfig irqLatch){

//	uint8_t interruptBits = (irqLatch << 2) | (active_L_H << 1) | (gpio) | (1<<6);
//	write8bit(BMP388_ADDR, BMP388_INT_CTRL, interruptBits);
//}

uint8_t dataReady(void){
	
	uint8_t readySignal;
	readySignal	= read8bit(&hi2c1, BMP388_ADDR, BMP388_STATUS);
	
	return (readySignal & (0x70)); //data ready demek sleep mode check yapilacak. ysf: gerek yok gibi, normal modda calistiricaz.
}

uint8_t getBMP_Data(volatile double* temperature, volatile double* pressure, volatile double* altitude){
	uint8_t data[6];
	uint32_t adcPress = 0;
	uint32_t adcTemp = 0;
	int64_t myPressure = 0;
	while(!dataReady());
	

	readMultBytes(&hi2c1, BMP388_ADDR, BMP388_DATA_0, data, 6);
	// if status kosullamasi yapilabilir.

	adcPress = ((uint32_t)(data[2] << 16))  | ((uint32_t)(data[1] << 8)) | (uint32_t)data[0];
	adcTemp  = ((uint32_t)(data[5] << 16))  | ((uint32_t)(data[4] << 8)) | (uint32_t)data[3];
	
	*temperature = a_bmp388_compensate_temperature(adcTemp);
	*temperature /= 100.0;
	//*temperature = (float)compensateTemperature(adcTemp);
	myPressure = a_bmp388_compensate_pressure(adcPress);
	*pressure = (double)(myPressure / 100.0);
	//*pressure = (float)compensatePressure(adcPress) / 100.0f;
	
	//*altitude =(pow(sea_level_pressure / (*pressure), 1.0 / 5.257 ) - 1.0) * ((*temperature) + 273.15) / 0.0065; 
	*altitude = 44330.0 * ( 1.0 - pow( (*pressure / sea_level_pressure), 1/5.255)  );
	return 1;
}
double a_bmp388_compensate_temperature(uint32_t data)
{ 
    uint64_t partial_data1;
    uint64_t partial_data2;
    uint64_t partial_data3;
    int64_t partial_data4;
    int64_t partial_data5;
    int64_t partial_data6;
	double partial_data7;
    double comp_temp;

    /* calculate compensate temperature */
    partial_data1 = (uint64_t)(data - (256 * (uint64_t)(calibINT.param_T1)));
    partial_data2 = (uint64_t)(calibINT.param_T2 * partial_data1);
	
    partial_data3 = (uint64_t)(partial_data1 * partial_data1);
    partial_data4 = (int64_t)(((int64_t)partial_data3) * ((int64_t)calibINT.param_T3));
    partial_data5 = ((int64_t)(((int64_t)partial_data2) * 262144) + (int64_t)partial_data4);
    partial_data6 = (int64_t)(((int64_t)partial_data5) / 4294967296U);
    calibINT.t_fine = partial_data6;
	partial_data7 = (double)(((int64_t)partial_data5) / 4294967296U);
    comp_temp = (double)((partial_data7 * 25.0)  / 16384.0);
    
    return comp_temp;
}

int64_t a_bmp388_compensate_pressure( uint32_t data)
{
    int64_t partial_data1;
    int64_t partial_data2;
    int64_t partial_data3;
    int64_t partial_data4;
    int64_t partial_data5;
    int64_t partial_data6;
    int64_t offset;
    int64_t sensitivity;
    uint64_t comp_press;

    /* calculate compensate pressure */
    partial_data1 = calibINT.t_fine * calibINT.t_fine;
    partial_data2 = partial_data1 / 64;
    partial_data3 = (partial_data2 * calibINT.t_fine) / 256;
    partial_data4 = (calibINT.param_P8 * partial_data3) / 32;
    partial_data5 = (calibINT.param_P7 * partial_data1) * 16;
    partial_data6 = (calibINT.param_P6 * calibINT.t_fine) * 4194304;
    offset = (int64_t)((int64_t)(calibINT.param_P5) * (int64_t)140737488355328U) + partial_data4 + partial_data5 + partial_data6;
    partial_data2 = (((int64_t)calibINT.param_P4) * partial_data3) / 32;
    partial_data4 = (calibINT.param_P3 * partial_data1) * 4;
    partial_data5 = ((int64_t)(calibINT.param_P2) - 16384) * ((int64_t)calibINT.t_fine) * 2097152;
    sensitivity = (((int64_t)(calibINT.param_P1) - 16384) * (int64_t)70368744177664U) + partial_data2 + partial_data4 + partial_data5;
    partial_data1 = (sensitivity / 16777216) * data;
    partial_data2 = (int64_t)(calibINT.param_P10) * (int64_t)(calibINT.t_fine);
    partial_data3 = partial_data2 + (65536 * (int64_t)(calibINT.param_P9));
    partial_data4 = (partial_data3 * data) / 8192;
    partial_data5 = (partial_data4 * data) / 512;
    partial_data6 = (int64_t)((uint64_t)data * (uint64_t)data);
    partial_data2 = ((int64_t)(calibINT.param_P11) * (int64_t)(partial_data6)) / 65536;
    partial_data3 = (partial_data2 * data) / 128;
    partial_data4 = (offset / 4) + partial_data1 + partial_data5 + partial_data3;
    comp_press = (((uint64_t)partial_data4 * 25) / (uint64_t)1099511627776U);
    
    return comp_press;
}

/*
float bmp388_compensate_temp(uint32_t uncomp_temp){
	
	float partial_data1 = (float)uncomp_temp - calibFLOAT.param_T1;
	float partial_data2 = partial_data1 * calibFLOAT.param_T2;
	
	return partial_data1 * partial_data1 * calibFLOAT.param_T3 + partial_data2;
}
float bmp388_compensate_press(uint32_t uncomp_press,float t_lin){
	
	float partial_data1, partial_data2, partial_data3, partial_data4, partial_out1,partial_out2;

	partial_data1 = calibFLOAT.param_P6 * t_lin;
	partial_data2 = calibFLOAT.param_P7 * t_lin * t_lin;
	partial_data3 = calibFLOAT.param_P8 * t_lin * t_lin * t_lin;
	partial_out1 = calibFLOAT.param_P5 + partial_data1 + partial_data2 + partial_data3;
	
	partial_data1 = calibFLOAT.param_P2 * t_lin;
	partial_data2 = calibFLOAT.param_P3 * t_lin * t_lin;
	partial_data3 = calibFLOAT.param_P4 * t_lin * t_lin * t_lin;
	
	partial_out2 = (float)uncomp_press * (calibFLOAT.param_P1 + 
											partial_data1 + 
											partial_data2 + 
											partial_data3);
	
	partial_data1 = (float)uncomp_press * (float)uncomp_press;
	partial_data2 = calibFLOAT.param_P9 + calibFLOAT.param_P10 * t_lin;
	partial_data3 = partial_data1 * partial_data2;
	
	partial_data4 = partial_data3 + (float)uncomp_press * 
									(float)uncomp_press * 
									(float)uncomp_press * 
									calibFLOAT.param_P11;
	
	return (partial_out1 + partial_out2 + partial_data4);	
}
*/

/*
	calibINT.param_T1 = read16bit(BMP388_ADDR, BMP388_TRIM_PARAMS);
	calibINT.param_T2 = read16bit(BMP388_ADDR, (BMP388_TRIM_PARAMS + 2));
	calibINT.param_T3 = (int8_t)read8bit(BMP388_ADDR, (BMP388_TRIM_PARAMS + 4));
	calibINT.param_P1 = (int16_t)read16bit(BMP388_ADDR, (BMP388_TRIM_PARAMS + 5));
	calibINT.param_P2 = (int16_t)read16bit(BMP388_ADDR, (BMP388_TRIM_PARAMS + 7));
	calibINT.param_P3 = (int8_t)read8bit(BMP388_ADDR, (BMP388_TRIM_PARAMS + 9) );
	calibINT.param_P4 = (int8_t)read8bit(BMP388_ADDR, (BMP388_TRIM_PARAMS + 10));
	calibINT.param_P5 = read16bit(BMP388_ADDR, (BMP388_TRIM_PARAMS + 11));
	calibINT.param_P6 = read16bit(BMP388_ADDR, (BMP388_TRIM_PARAMS + 13));
	calibINT.param_P7 = (int8_t)read8bit(BMP388_ADDR, (BMP388_TRIM_PARAMS + 15));
	calibINT.param_P8 = (int8_t)read8bit(BMP388_ADDR, (BMP388_TRIM_PARAMS + 16));
	calibINT.param_P9 = (int16_t)read16bit(BMP388_ADDR, (BMP388_TRIM_PARAMS + 17));
	calibINT.param_P10 = (int8_t)read8bit(BMP388_ADDR, (BMP388_TRIM_PARAMS + 19));
	calibINT.param_P11 = (int8_t)read8bit(BMP388_ADDR, (BMP388_TRIM_PARAMS + 20));
	return 1;
*/

/*
  uint8_t calcCoefPARAM(void){
	// Read the trim parameters into the params structure
	uint8_t temp;
	temp = readCoef();
	if(!temp)
		return 0;
	// Calculate the floating point trim parameters
	
	calibFLOAT.param_T1 = (float)calibINT.param_T1 / powf(2.0f, -8.0f);
	calibFLOAT.param_T2 = (float)calibINT.param_T2 / powf(2.0f, 30.0f);
	calibFLOAT.param_T3 = (float)calibINT.param_T3 / powf(2.0f, 48.0f);
	calibFLOAT.param_P1 = ((float)calibINT.param_P1 - powf(2.0f, 14.0f)) / powf(2.0f, 20.0f);
	calibFLOAT.param_P2 = ((float)calibINT.param_P2 - powf(2.0f, 14.0f)) / powf(2.0f, 29.0f);
	calibFLOAT.param_P3 = (float)calibINT.param_P3 / powf(2.0f, 32.0f);
	calibFLOAT.param_P4 = (float)calibINT.param_P4 / powf(2.0f, 37.0f);
	calibFLOAT.param_P5 = (float)calibINT.param_P5 / powf(2.0f, -3.0f);
	calibFLOAT.param_P6 = (float)calibINT.param_P6 / powf(2.0f, 6.0f);
	calibFLOAT.param_P7 = (float)calibINT.param_P7 / powf(2.0f, 8.0f);
	calibFLOAT.param_P8 = (float)calibINT.param_P8 / powf(2.0f, 15.0f);
	calibFLOAT.param_P9 = (float)calibINT.param_P9 / powf(2.0f, 48.0f);
	calibFLOAT.param_P10 = (float)calibINT.param_P11 / powf(2.0f, 48.0f);
	calibFLOAT.param_P11 = (float)calibINT.param_P11 / powf(2.0f, 65.0f);
	return 1;
}
*/