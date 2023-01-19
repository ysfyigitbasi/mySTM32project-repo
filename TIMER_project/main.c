#include "_I2C.h"
#include "_TIM_CONFIG.h"

#define MESSAGE1	"HELLO_REVEIVER_1"

TX_BUFFER bufferTX;
RX_BUFFER bufferRX;

int main(void){
	
	initI2C1();
	char GPS[20];
	char* single_ptr;
	char single_rx;
	
	single_ptr = &bufferRX.receiveData;
	single_rx = *single_ptr;
	
	
	i2c_write_single(0x28,0x55,'C');
	
	while(1){
		
		i2c_write_single(0x28,0x55,'C');
		delayMS(20);
		i2c_read_single(0x28,0x12);
		delayMS(5);
		single_rx = *single_ptr;
		i2c_writeMULT(0x28,0x30,MESSAGE1,16);
		delayMS(10);
		i2c_readMULT(0x28,0x44,GPS,20);
	}
}


/*
void i2c_read(uint8_t device_address, uint8_t* memory, uint8_t length){
	
	uint16_t temp = 0;
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
	I2C1->CR2 |= I2C_CR2_DMAEN;
	I2C1->CR1 |= I2C_CR1_ACK;
	DMA1_Channel7->CMAR = (uint32_t)memory;
	DMA1_Channel7->CPAR = (uint32_t)&I2C1->DR;
	DMA1_Channel7->CNDTR = length;
	NVIC_EnableIRQ(DMA1_Channel7_IRQn);
	DMA1_Channel7->CCR |= DMA_CCR7_TCIE | DMA_CCR7_MINC | DMA_CCR7_EN;
	
	I2C1->CR1 |= I2C_CR1_START;
	
	while(	!(I2C1->SR1 & I2C_SR1_SB)	)	{}
	
	I2C1->DR = device_address + 1;	// when you write, add 1 at LSB.
	
	while(	!(I2C1->SR1 & I2C_SR1_ADDR)	)	{}
		
	temp = I2C1->SR2;
		
	while(	( DMA1->ISR & DMA_ISR_TCIF7 ) == 0	);
	I2C1->CR1 &= ~I2C_CR1_ACK;	//NACK

	I2C1->CR1 |= I2C_CR1_STOP;	
}
*/
