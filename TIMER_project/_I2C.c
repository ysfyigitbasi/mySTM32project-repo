#include "_I2C.h"

void initI2C1(void){
	
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
		
	#ifndef I2C1_REMAP
	GPIOB->CRL |= GPIO_CRL_CNF6 | GPIO_CRL_MODE6 | GPIO_CRL_CNF7 | GPIO_CRL_MODE7;	// SCL-PB6 SDA-PB7 OPEN-DRAIN ALTERNATE FNCTION.
	#else
	GPIOB->CRL |= GPIO_CRL_CNF8 | GPIO_CRL_MODE8 | GPIO_CRL_CNF9 | GPIO_CRL_MODE9;	// SCL-PB8 SDA-PB9 OPEN-DRAIN ALTERNATE FNCTION.
	#endif
	
	#ifndef FASTMODE
	/* Clock Control Register Math --- 100KHz. %50 duty cycle
	T_high / T_pclk = CCR -> (1/100khz * 1/2) / (1/36MHz) = 180. */
		I2C1->CCR |= CLCK_FREQ_MHZ * 1000 / 200;
		I2C1->TRISE |= CLCK_FREQ_MHZ + 1;		// 1000ns t_rise [max for standard mode 100kHz] 300ns[max for fast mode], 1000ns/(1/36MHz) = TRISE + 1.
	#else
		I2C1->CCR |= I2C_CCR_FS;
		I2C1->CCR &= ~I2C_CCR_DUTY;
		I2C1->CCR |= CLCK_FREQ_MHZ * 1000 / 1200;
		I2C1->TRISE |= (CLCK_FREQ_MHZ * 300 / 1000) + 1;
	#endif
	
	I2C1->CR1 |= I2C_CR1_ACK;	//ENABLE ACK
	I2C1->CR2 |= I2C_CR2_ITBUFEN | I2C_CR2_ITEVTEN | CLCK_FREQ_MHZ;
	
	// stretch mode enabled by default
	// 7bit addressing mode is enabled by default.
	
	I2C1->CR1 |= I2C_CR1_PE;
	
}

void initI2C1_DMA(void){
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
	I2C1->CR2 |= I2C_CR2_DMAEN;
	
	/*	RECEIVE	*/
	NVIC_EnableIRQ(DMA1_Channel7_IRQn);
	DMA1_Channel7->CCR |= DMA_CCR7_TCIE | DMA_CCR7_MINC; // DMA_CCR7_EN
	
	/* 	TRANSMIT		*/
	NVIC_EnableIRQ(DMA1_Channel6_IRQn);
	DMA1_Channel6->CCR |= DMA_CCR6_TCIE | DMA_CCR6_MINC | DMA_CCR6_DIR; // DMA_CCR6_EN
}

void enableRX_DMA(void){ DMA1_Channel7->CCR |= DMA_CCR7_EN;	}
void enableTX_DMA(void){ DMA1_Channel6->CCR |= DMA_CCR6_EN;	}

void i2c_write_single(uint8_t device_address, uint8_t mem_address, uint8_t data){
	
	uint16_t temp = 0;
	I2C1->CR1 |= I2C_CR1_ACK;	//ENABLE ACK
	
	I2C1->CR1 |= I2C_CR1_START;		// generate start condition.
	
	while(	!(I2C1->SR1 & I2C_SR1_SB)	)	{}
	
	I2C1->DR = device_address;		// write 0x78; send address.
	
	while(	!(I2C1->SR1 & I2C_SR1_ADDR)	)	{}
		
	temp = I2C1->SR2;
		
	I2C1->DR = mem_address;		// address to write to
		
	while(	!(I2C1->SR1 & I2C_SR1_TXE)	)	{}	// wait for byte transfer complete.
		
	I2C1->DR = data;		// data to write in that address.
		
	while(	!(I2C1->SR1 & I2C_SR1_TXE)	)	{}	// wait for byte transfer complete.
	
	I2C1->CR1 |= I2C_CR1_STOP;		// send another start condition to get the data.	
}

void i2c_read(uint8_t* memory, uint8_t length){
	
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

void DMA1_Channel7_IRQHandler(void){	// I2C1 rx
	// if i2c1 is used
	DMA1->IFCR = DMA_IFCR_CTCIF7;
	DMA1_Channel7->CCR &= ~(uint32_t)DMA_CCR7_EN;
}

void I2C1_EV_IRQHandler(void){
	
	if( I2C1->SR1 & I2C_SR1_SB ){
		I2C1->DR = device_address;
		if(READ)
			I2C1->DR++;
	}
	
	if( I2C1->SR1 & I2C_SR1_ADDR ){
		(void) I2C1->SR2;
		I2C1->DR = mem_address;		// address to write to------struct TX_char.
	}
	
}
