#include "_I2C.h"
//--------------------------------------------------------------------------------------------------------------------
//-----------------------------------INIT I2C-1-----------------------------------------------------------------------
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
		I2C1->TRISE = CLCK_FREQ_MHZ + 1;		// 1000ns t_rise [max for standard mode 100kHz] 300ns[max for fast mode], 1000ns/(1/36MHz) = TRISE + 1.
	#else
		I2C1->CCR |= I2C_CCR_FS;
		I2C1->CCR &= ~I2C_CCR_DUTY;
		I2C1->CCR |= CLCK_FREQ_MHZ * 1000 / 1200;
		I2C1->TRISE |= (CLCK_FREQ_MHZ * 300 / 1000) + 1;
	#endif
	
	I2C1->CR2 |= I2C_CR2_ITEVTEN | CLCK_FREQ_MHZ;
	
	// stretch mode enabled by default
	// 7bit addressing mode is enabled by default.
	
	initI2C1_DMA();

	
}

//--------------------------------------------------------------------------------------------------------------------
//----------------------------------- DMA FUNCTIONS ------------------------------------------------------------------
/*
static void config_DMA_TX(void){
	DMA1_Channel6->CMAR = (uint32_t)&bufferTX.transmitBUFF;
	DMA1_Channel6->CPAR = (uint32_t)&I2C1->DR;
	DMA1_Channel6->CNDTR = bufferTX.sizeTX;	
}*/


static void initI2C1_DMA(void){
			
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
	I2C1->CR2 |= I2C_CR2_DMAEN;
	
	//config_DMA_TX();
	
	/*	RECEIVE	*/
	//NVIC_EnableIRQ(DMA1_Channel7_IRQn);
	DMA1_Channel7->CCR |= DMA_CCR7_TCIE | DMA_CCR7_MINC; // DMA_CCR7_EN
	
	/* 	TRANSMIT		*/
	//NVIC_EnableIRQ(DMA1_Channel6_IRQn);
	DMA1_Channel6->CCR |= DMA_CCR6_TCIE | DMA_CCR6_MINC | DMA_CCR6_DIR; // DMA_CCR6_EN
}

static void enableRX_DMA(void){DMA1_Channel7->CCR |= DMA_CCR7_EN;}
static void enableTX_DMA(void){DMA1_Channel6->CCR |= DMA_CCR6_EN;}
static void disableRX_DMA(void){
	DMA1->IFCR = DMA_IFCR_CTCIF7;
	DMA1_Channel7->CCR &= ~(uint32_t)DMA_CCR7_EN;
}
static void disableTX_DMA(void){
	DMA1->IFCR = DMA_IFCR_CTCIF6;
	DMA1_Channel6->CCR &= ~(uint32_t)DMA_CCR6_EN;
}

// I'm going to add disable functions. i2c and dmas.
static void i2c_stop(void){
	I2C1->CR1 |= I2C_CR1_STOP;	//STOP TRANSMISION
	I2C1->CR1 &= ~I2C_CR1_PE;
}


// ********************************* USER FUNCTIONS ***********************************************
void i2c_write_single(uint8_t slave_address, uint8_t mem_address, uint8_t data){
	
	I2C1->CR1 |= I2C_CR1_PE;
	I2C1->CR1 |= I2C_CR1_ACK;	//ENABLE ACK	
	I2C1->CR1 |= I2C_CR1_START;		// generate start condition.	
	
	while(!(I2C1->SR1 & I2C_SR1_SB));
	
	I2C1->DR = (uint8_t)(slave_address << 1);
	
	while(!(I2C1->SR1 & I2C_SR1_ADDR));	// send address
	
	(void)I2C1->SR2;
	
	I2C1->DR = mem_address;
	
	while(!(I2C1->SR1 & I2C_SR1_TXE));	// wait for transfer
	
	I2C1->DR = data;
	
	while(!(I2C1->SR1 & I2C_SR1_TXE));
	
	I2C1->CR1 |= I2C_CR1_STOP;
	
}

void i2c_writeMULT(uint8_t slave_address, uint8_t sensor_mem_address, char* mem_ptr, uint8_t mem_size){
	
	DMA1_Channel6->CNDTR = mem_size;
	DMA1_Channel6->CMAR = (uint32_t)mem_ptr;
	DMA1_Channel6->CPAR = (uint32_t)&I2C1->DR;
	enableTX_DMA();
	
	I2C1->CR1 |= I2C_CR1_PE;
	I2C1->CR1 |= I2C_CR1_ACK;	//ENABLE ACK
	I2C1->CR1 |= I2C_CR1_START;
	
	while(!(I2C1->SR1 & I2C_SR1_SB));
	
	I2C1->DR = (uint8_t)(slave_address << 1);
	
	while(!(I2C1->SR1 & I2C_SR1_ADDR));	// send address
	
	(void)I2C1->SR2;
	
	I2C1->DR = sensor_mem_address;
	
	while(!(I2C1->SR1 & I2C_SR1_TXE));	// wait for transfer
	
	while(!(DMA1->ISR & DMA_ISR_TCIF6));
	
	disableTX_DMA();
	i2c_stop();

}

void i2c_readMULT(uint8_t slave_address, uint8_t sensor_mem_address, char *mem_ptr, uint8_t mem_size){
		
	DMA1_Channel7->CNDTR = mem_size;
	DMA1_Channel7->CMAR = (uint32_t)mem_ptr;
	DMA1_Channel7->CPAR = (uint32_t)&I2C1->DR;
	enableRX_DMA();
	I2C1->CR1 |= I2C_CR1_PE;
	I2C1->CR1 |= I2C_CR1_ACK;	//ENABLE ACK
	I2C1->CR1 |= I2C_CR1_START;
	
	while(!(I2C1->SR1 & I2C_SR1_SB));
	
	I2C1->DR = (uint8_t)(slave_address << 1);
	
	while(!(I2C1->SR1 & I2C_SR1_ADDR));	// send address
	
	(void)I2C1->SR2;
	
	I2C1->DR = sensor_mem_address;
	
	while(!(I2C1->SR1 & I2C_SR1_TXE));	// wait for transfer
	
	I2C1->CR1 |= I2C_CR1_START;
	
	while(!(I2C1->SR1 & I2C_SR1_SB));
	
	I2C1->DR = (uint8_t)((slave_address << 1) + 1);
	
	while(!(I2C1->SR1 & I2C_SR1_ADDR));	// send address
	
	(void)I2C1->SR2;
	
	while(!(DMA1->ISR & DMA_ISR_TCIF7));
	
	disableRX_DMA();
	I2C1->CR1 &= ~I2C_CR1_ACK;	//NACK- unnecessary - I'm going to test it without writing this line.
	i2c_stop();
}

char i2c_read_single( uint8_t slave_address, uint8_t sensor_mem_address){
		
	char temp;
	I2C1->CR1 |= I2C_CR1_PE;
	I2C1->CR1 |= I2C_CR1_ACK;	//ENABLE ACK	
	I2C1->CR1 |= I2C_CR1_START;	// generate start condition.
	
	while(!(I2C1->SR1 & I2C_SR1_SB));
	
	I2C1->DR = (uint8_t)(slave_address << 1);
	
	while(!(I2C1->SR1 & I2C_SR1_ADDR));	// send address
	
	(void)I2C1->SR2;
	
	I2C1->DR = sensor_mem_address;
	
	while(!(I2C1->SR1 & I2C_SR1_TXE));	// wait for transfer
	
	I2C1->CR1 |= I2C_CR1_START;
	
	while(!(I2C1->SR1 & I2C_SR1_SB));
	
	I2C1->DR = (uint8_t)((slave_address << 1) + 1);
	
	while(!(I2C1->SR1 & I2C_SR1_ADDR));	// send address
	
	(void)I2C1->SR2;
	
	while(!(I2C1->SR1 & I2C_SR1_RXNE));
	temp = (char)I2C1->DR;
	
	
	disableRX_DMA();
	I2C1->CR1 &= ~I2C_CR1_ACK;	//NACK- unnecessary - I'm going to test it without writing this line.
	i2c_stop();
	return temp;
}
