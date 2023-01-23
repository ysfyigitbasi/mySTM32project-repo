#include "_I2C.h"
#include "_TIM_CONFIG.h"


#define BMP180_ADDRESS		(0x77)

TX_BUFFER bufferTX;
RX_BUFFER bufferRX;

int main(void){
	
	char ac1[2]; //b1[2],b2[2],mb[2],mc[2],md[2];
	//unsigned char ac4[2],ac5[2],ac6[2];
	
	short int ac_1;
	
	initI2C1();
	
	i2c_readMULT(BMP180_ADDRESS,0xAA,ac1,2);
	ac_1 = (short int)(ac1[0] << 8) | ac1[1];

	while(1){
		ac_1 = (short int)(ac1[0] << 8) | ac1[1];
	}
	
}
