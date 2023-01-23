#include "_I2C.h"
#include "_TIM_CONFIG.h"
#include <stdio.h>


#define BMP180_ADDRESS		(0x77)

TX_BUFFER bufferTX;
RX_BUFFER bufferRX;

int main(void){
	
	char ac1; //ac3[2],b1[2],b2[2],mb[2],mc[2],md[2];
	//unsigned char ac4[2],ac5[2],ac6[2];
	
	
	
	initI2C1();
	
	i2c_read_single(0x68,0x75);
	ac1 = bufferRX.receiveData;

	while(1){
		ac1 = bufferRX.receiveData;
		printf("%c",ac1);
		i2c_read_single(0x68,0x75);
	}
	
}
