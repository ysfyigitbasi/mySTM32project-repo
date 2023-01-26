#ifndef _BMP388_H
#define _BMP388_H

#include "_I2C.h"

// BMP388_DEV Definitions

#define BMP388_I2C_ADDR		 	0x77	// The BMP388 I2C address
//#define BMP388_I2C_ALT_ADDR 	0x76	// The BMP388 I2C alternate address
#define BMP388_ID 				0x50	// The BMP388 device ID
#define BMP390_ID				0x60	// The BMP390 device ID
#define RESET_CODE				0xB6	// The BMP388 reset code
#define FIFO_FLUSH				0xB0	// The BMP388 flush FIFO code


// *********************| BMP388_DEV Registers |*****************************
#define	BMP388_CHIP_ID			0x00	// Chip ID register sub-address
#define	BMP388_ERR_REG			0x02	// Error register sub-address
#define	BMP388_STATUS			0x03	// Status register sub-address
#define	BMP388_DATA_0			0x04 	// Pressure eXtended Least Significant Byte (XLSB) register sub-address
#define	BMP388_DATA_1			0x05 	// Pressure Least Significant Byte (LSB) register sub-address
#define	BMP388_DATA_2			0x06 	// Pressure Most Significant Byte (MSB) register sub-address
#define	BMP388_DATA_3			0x07 	// Temperature eXtended Least Significant Byte (XLSB) register sub-address
#define	BMP388_DATA_4			0x08 	// Temperature Least Significant Byte (LSB) register sub-address
#define	BMP388_DATA_5			0x09 	// Temperature Most Significant Byte (MSB) register sub-address
#define	BMP388_SENSORTIME_0		0x0C 	// Sensor time register 0 sub-address
#define	BMP388_SENSORTIME_1		0x0D 	// Sensor time register 1 sub-address
#define	BMP388_SENSORTIME_2		0x0E 	// Sensor time register 2 sub-address
#define	BMP388_EVENT			0x10 	// Event register sub-address
#define	BMP388_INT_STATUS		0x11 	// Interrupt Status register sub-address
#define	BMP388_FIFO_LENGTH_0	0x12 	// FIFO Length Least Significant Byte (LSB) register sub-address
#define	BMP388_FIFO_LENGTH_1	0x13 	// FIFO Length Most Significant Byte (MSB) register sub-address
#define	BMP388_FIFO_DATA		0x14 	// FIFO Data register sub-address
#define	BMP388_FIFO_WTM_0		0x15 	// FIFO Water Mark Least Significant Byte (LSB) register sub-address
#define	BMP388_FIFO_WTM_1		0x16 	// FIFO Water Mark Most Significant Byte (MSB) register sub-address
#define	BMP388_FIFO_CONFIG_1    0x17 	// FIFO Configuration 1 register sub-address
#define	BMP388_FIFO_CONFIG_2    0x18 	// FIFO Configuration 2 register sub-address
#define	BMP388_INT_CTRL			0x19 	// Interrupt Control register sub-address
#define	BMP388_IF_CONFIG		0x1A 	// Interface Configuration register sub-address
#define	BMP388_PWR_CTRL			0x1B 	// Power Control register sub-address
#define	BMP388_OSR				0x1C 	// Oversampling register sub-address
#define	BMP388_ODR				0x1D 	// Output Data Rate register sub-address
#define	BMP388_CONFIG			0x1F 	// Configuration register sub-address
#define	BMP388_TRIM_PARAMS		0x31    // Trim parameter registers' base sub-address
#define	BMP388_CMD				0x7E	// Command register sub-address
     
	 
#define sea_level_pressure 		1013.25f	 
//************************| BMP388_DEV Modes |******************************************
enum Mode{          // Device mode bitfield in the control and measurement register 
	SLEEP_MODE          	 = 0x00,
	FORCED_MODE         	 = 0x01,
	NORMAL_MODE         	 = 0x03
};

//************************| BMP388_DEV Register bit field Definitions |*****************
enum Oversampling {	// Oversampling bit fields in the control and measurement register
	OVERSAMPLING_SKIP 		 = 0x00,
	OVERSAMPLING_X2   		 = 0x01,
	OVERSAMPLING_X4  		 = 0x02,
	OVERSAMPLING_X8    		 = 0x03,
	OVERSAMPLING_X16   	 	 = 0x04,
	OVERSAMPLING_X32   	 	 = 0x05
};

enum IIRFilter {	// Infinite Impulse Response (IIR) filter bit field in the configuration register
	IIR_FILTER_OFF		= 0x00,
	IIR_FILTER_2		= 0x01,
	IIR_FILTER_4        = 0x02,
	IIR_FILTER_8        = 0x03,
	IIR_FILTER_16       = 0x04,
	IIR_FILTER_32       = 0x05,
	IIR_FILTER_64       = 0x06,
	IIR_FILTER_128      = 0x07
};

enum TimeStandby {	// Time standby bit field in the Output Data Rate (ODR) register
	TIME_STANDBY_5MS       = 0x00,      		 
	TIME_STANDBY_10MS      = 0x01,
	TIME_STANDBY_20MS      = 0x02,
	TIME_STANDBY_40MS      = 0x03,
	TIME_STANDBY_80MS      = 0x04,
	TIME_STANDBY_160MS     = 0x05,
	TIME_STANDBY_320MS     = 0x06,
	TIME_STANDBY_640MS     = 0x07,
	TIME_STANDBY_1280MS    = 0x08,
	TIME_STANDBY_2560MS    = 0x09,
	TIME_STANDBY_5120MS    = 0x0A,
	TIME_STANDBY_10240MS   = 0x0B,
	TIME_STANDBY_20480MS   = 0x0C,
	TIME_STANDBY_40960MS   = 0x0D,
	TIME_STANDBY_81920MS   = 0x0E,
	TIME_STANDBY_163840MS  = 0x0F,
	TIME_STANDBY_327680MS  = 0x10,
	TIME_STANDBY_655360MS  = 0x11
};

enum OutputDrive {			// Interrupt output drive configuration
	PUSH_PULL				= 0x00,
	OPEN_COLLECTOR			= 0x01
};

enum ActiveLevel {			// Interrupt output active level configuration
	ACTIVE_LOW  		 	= 0x00,
	ACTIVE_HIGH			 	= 0x01
};

enum LatchConfig {			// Interrupt output latch configuration
	UNLATCHED				= 0x00,		// UNLATCHED: interrupt automatically clears after 2.5ms
	LATCHED					= 0x01		// LATCHED	: interrupt requires INT_STATUS register read to clear
};

typedef struct {	// The BMP388 compensation trim parameters (coefficients)
	uint16_t param_T1;
	uint16_t param_T2;
	int8_t   param_T3;
	int16_t  param_P1;
	int16_t  param_P2;
	int8_t   param_P3;
	int8_t   param_P4;
	uint16_t param_P5;
	uint16_t param_P6;
	int8_t   param_P7;
	int8_t   param_P8;
	int16_t  param_P9;
	int8_t 	 param_P10;
	int8_t 	 param_P11;
}calibINTCoefficents;

typedef struct{		// The BMP388 float point compensation trim parameters
	double param_T1;
	double param_T2;
	double param_T3;
	double param_P1;
	double param_P2;
	double param_P3;
	double param_P4;
	double param_P5;
	double param_P6;
	double param_P7;
	double param_P8;
	double param_P9;
	double param_P10;
	double param_P11;
}calibFLOATCoefficents;

/* we will discuss later
enum PressEnable {												// FIFO pressure enable configuration
	PRESS_DISABLED			= 0x00,
	PRESS_ENABLED			= 0x01
};

enum AltEnable {													// FIFO altitude enable configuration
	ALT_DISABLED			= 0x00,
	ALT_ENABLED				= 0x01
};
	
enum TimeEnable {													// FIFO time enable configuration
	TIME_DISABLED			= 0x00,
	TIME_ENABLED			= 0x01
};	

enum Subsampling {												// FIFO sub-sampling configuration
	SUBSAMPLING_OFF			= 0x00,
	SUBSAMPLING_DIV2		= 0x01,
	SUBSAMPLING_DIV4		= 0x02,
	SUBSAMPLING_DIV8		= 0x03,
	SUBSAMPLING_DIV16		= 0x04,
	SUBSAMPLING_DIV32		= 0x05,
	SUBSAMPLING_DIV64		= 0x06,
	SUBSAMPLING_DIV128		= 0x07
};

enum DataSelect {													// FIFO data select configuration
	UNFILTERED				= 0x00,
	FILTERED				= 0x01
};

enum StopOnFull {													// FIFO stop on full configuration
	STOP_ON_FULL_DISABLED	 = 0x00,
	STOP_ON_FULL_ENABLED 	 = 0x01
};

enum FIFOStatus {													// FIFO status
	DATA_PENDING			 = 0x00,
	DATA_READY				 = 0X01,
	CONFIG_ERROR			 = 0x02
};

enum WatchdogTimout {											// I2C watchdog time-out
	WATCHDOG_TIMEOUT_1MS	 = 0x00,
	WATCHDOG_TIMEOUT_40MS	 = 0x01
};
*/
// ***********************| ENUMERATORS |****************************************
extern enum Mode mode;
extern enum Oversampling presOversampling; 
extern enum Oversampling tempOversampling;
extern enum IIRFilter iirFilter;
extern enum TimeStandby timeStandby;
extern enum OutputDrive outputDrive;
extern enum ActiveLevel activeLevel;
extern enum LatchConfig latchConfig;
static calibINTCoefficents calibINT;
static calibFLOATCoefficents calibFLOAT;
//************************| FUNCTIONS |******************************************

uint8_t begin(uint8_t mode, uint8_t iirFilter, uint8_t timeStand, uint8_t presOSR, uint8_t tempOSR);

// *****| SET FUNCTIONS |*****

// set registers..
static void setMode(uint8_t mode);			// Set the barometer mode
static void setIIRFilter(uint8_t iirCoef);		// Set the IIR filter setting: OFF, 2, 3, 8, 16, 32
static void setTimeStandby(uint8_t timeStandby); 	// Set the time standby measurement interval: 5, 62, 125, 250, 500ms...
static void setOversamplingRegister(uint8_t pressOSR, uint8_t tempOSR); 	// pressure and temperature
// INTERRUPTS
static void setIntOutputDrive(void);// Sets the interrupt pin's output drive, PUSH_PULL OR OPEN_DRAIN, default PUSH_PULL
static void setIntActiveLevel(void);// Set the interrupt active level, ACTIVE_LOW or ACTIVE_HIGH, default ACTIVE_HIGH
static void setIntLatchConfig(void);// Set the interrupt latch, UNLATCHED or LATCHED, default UNLATCHED

// ACTIONS
uint8_t reset(void);				// Soft reset the barometer
void enableInterrupt(void);
void disableInterrupt(void);	// Disable the BMP388's interrupt pin

// *****| GET FUNCTIONS |*****
// measurements
static uint8_t dataReady(void);		// Checks if a measurement is ready
void getTemperature(float* temperature);						// Get a temperature measurement
void getPressure(float* pressure);								// Get a pressure measurement
void getTempPres(float* temperature,	float* pressure);		
void getAltitude(float* altitude);								// Get an altitude measurement
void getMeasurements(float* temperature, float* pressure, float* altitude);
void calcCoefPARAM(void);

float bmp388_compensate_temp(float* uncomp_temp);				// Bosch temperature compensation function
float bmp388_compensate_press(float* uncomp_press,float* t_lin);// Bosch pressure compensation function		
#endif
