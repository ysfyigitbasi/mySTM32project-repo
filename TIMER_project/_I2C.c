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
		I2C1->TRISE |= CLCK_FREQ_MHZ + 1;		// 1000ns t_rise [max for standard mode 100kHz] 300ns[max for fast mode], 1000ns/(1/36MHz) = TRISE + 1.
	#else
		I2C1->CCR |= I2C_CCR_FS;
		I2C1->CCR &= ~I2C_CCR_DUTY;
		I2C1->CCR |= CLCK_FREQ_MHZ * 1000 / 1200;
		I2C1->TRISE |= (CLCK_FREQ_MHZ * 300 / 1000) + 1;
	#endif
	
	I2C1->CR2 |= I2C_CR2_ITEVTEN | CLCK_FREQ_MHZ;
	NVIC_EnableIRQ(I2C1_EV_IRQn);
	NVIC_SetPriority(I2C1_EV_IRQn, 3);
	
	// stretch mode enabled by default
	// 7bit addressing mode is enabled by default.
	
	initI2C1_DMA();

	
}

//--------------------------------------------------------------------------------------------------------------------
//----------------------------------- DMA FUNCTIONS ------------------------------------------------------------------

static void config_DMA_RX(void){

}

static void config_DMA_TX(void){
	DMA1_Channel6->CMAR = (uint32_t)&bufferTX.transmitBUFF;
	DMA1_Channel6->CPAR = (uint32_t)&I2C1->DR;
	DMA1_Channel6->CNDTR = bufferTX.sizeTX;	
}


static void initI2C1_DMA(void){
	
	init_queue(&bufferTX);
		
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
	I2C1->CR2 |= I2C_CR2_DMAEN;
	
	config_DMA_TX();
	config_DMA_RX();
	
	/*	RECEIVE	*/
	NVIC_EnableIRQ(DMA1_Channel7_IRQn);
	DMA1_Channel7->CCR |= DMA_CCR7_TCIE | DMA_CCR7_MINC; // DMA_CCR7_EN
	
	/* 	TRANSMIT		*/
	NVIC_EnableIRQ(DMA1_Channel6_IRQn);
	DMA1_Channel6->CCR |= DMA_CCR6_TCIE | DMA_CCR6_MINC | DMA_CCR6_DIR; // DMA_CCR6_EN
}

static void init_queue(TX_BUFFER *q){
	q->transmitData = 0;
	q->sizeTX = 0; q->device_address = 0; q->memory_address = 0;
	q->state_TX = STOP;
}

static void enableRX_DMA(void){
	I2C1->CR2 &= ~I2C_CR2_ITBUFEN;
	I2C1->CR2 |= I2C_CR2_LAST;
	DMA1_Channel7->CCR |= DMA_CCR7_EN;	
}
static void enableTX_DMA(void){ 
	I2C1->CR2 &= ~I2C_CR2_ITBUFEN;
	I2C1->CR2 |= I2C_CR2_LAST;	
	DMA1_Channel6->CCR |= DMA_CCR6_EN;	
}

// I'm going to add disable functions. i2c and dmas.
static void i2c_stop(void){
	I2C1->CR1 |= I2C_CR1_STOP;	//STOP TRANSMISION
	bufferTX.state_TX = STOP;
	I2C1->CR1 &= ~I2C_CR1_PE;
}


// ********************************* USER FUNCTIONS ***********************************************
void i2c_write_single(uint8_t slave_address, uint8_t mem_address, uint8_t data){
	
	bufferTX.memory_address = mem_address;
	bufferTX.device_address = (uint8_t)(slave_address << 1);
	bufferTX.transmitData = data;
	bufferTX.I2C_MODE = WRITE;
	
	I2C1->CR1 |= I2C_CR1_PE;
	I2C1->CR1 |= I2C_CR1_ACK;	//ENABLE ACK
	I2C1->CR2 |= I2C_CR2_ITBUFEN;	// ENABLE TX-RXNE INTERRUPTS
	I2C1->CR2 &= ~I2C_CR2_LAST;		// DISABLE LAST
	
	
	I2C1->CR1 |= I2C_CR1_START;		// generate start condition.	
}

void i2c_writeMULT(uint8_t slave_address, uint8_t sensor_mem_address, char* mem_ptr, uint8_t mem_size){
	
	bufferTX.device_address = (uint8_t)(slave_address << 1);
	bufferTX.memory_address = sensor_mem_address;
	bufferTX.transmitBUFF = mem_ptr;
	bufferTX.sizeTX = mem_size;
	bufferTX.I2C_MODE = WRITE_DMA;
	
	I2C1->CR1 |= I2C_CR1_PE;
	I2C1->CR1 |= I2C_CR1_ACK;	//ENABLE ACK
	
	I2C1->CR1 |= I2C_CR1_START;	
}

void i2c_readMULT(uint8_t slave_address, uint8_t sensor_mem_address, char *mem_ptr, uint8_t mem_size){
	
	bufferTX.device_address = (uint8_t)(slave_address << 1);
	bufferTX.memory_address = sensor_mem_address;
	
	DMA1_Channel7->CNDTR = mem_size;
	DMA1_Channel7->CMAR = (uint32_t)mem_ptr;
	DMA1_Channel7->CPAR = (uint32_t)&I2C1->DR;
	bufferTX.I2C_MODE = READ_DMA;
	
	
	I2C1->CR1 |= I2C_CR1_PE;
	I2C1->CR1 |= I2C_CR1_ACK;	//ENABLE ACK
	
	I2C1->CR1 |= I2C_CR1_START;
				
}

void i2c_read_single( uint8_t slave_address, uint8_t sensor_mem_address){
	
	bufferTX.device_address = (uint8_t)(slave_address << 1);
	bufferTX.memory_address = sensor_mem_address;
	
	bufferTX.I2C_MODE = READ;
	
	I2C1->CR1 |= I2C_CR1_PE;
	I2C1->CR1 |= I2C_CR1_ACK;	//ENABLE ACK
	I2C1->CR2 |= I2C_CR2_ITBUFEN;	// ENABLE TX-RXNE INTERRUPTS
	I2C1->CR2 &= ~I2C_CR2_LAST;		// DISABLE LAST
	
	//I2C1->CR1 |= I2C_CR1_STOP;	//STOP TRANSMISION
	I2C1->CR1 |= I2C_CR1_START;		// generate start condition.
}


// ********************************* INTERRUPT SERVISE ROUTINE FUNCTIONS *****************************
void I2C1_EV_IRQHandler(void){
	
	if( I2C1->SR1 & I2C_SR1_SB ){			// START BIT FLAG
		
		if( bufferTX.state_TX == STOP ){
			I2C1->DR = bufferTX.device_address;			// WRITE PROCESS - slave address
			bufferTX.state_TX = SLAVE_ADDRESS;
		}
		else if( bufferTX.state_TX == REG_ADDRESS ){
			I2C1->DR = bufferTX.device_address + 1;	// READ PROCESS
			if( bufferTX.I2C_MODE == READ )				// ONE BYTE RECEIVE
				bufferTX.state_TX = RECEIVE;
			else{	// read_dma
				bufferTX.state_TX = RECEIVE_DMA;		// ENABLE RX DMA
				enableRX_DMA();
			}
		}
	}

	if( I2C1->SR1 & I2C_SR1_ADDR ){		// ADDRESS BIT FLAG
		
		uint32_t temp;
		temp = I2C1->SR2;	// READ SR2 - clear addr flag
	}
	
	if( I2C1->SR1 & I2C_SR1_TXE){				// TRANSMIT DATA REG IS EMPTY BIT FLAG
		
		if( bufferTX.state_TX == SLAVE_ADDRESS ){
			
			I2C1->DR = bufferTX.memory_address;		// address to write to reg address of the sensor.
			bufferTX.state_TX = TRANSMIT;
		}

		else if( bufferTX.state_TX == TRANSMITTED ){
			
			I2C1->CR1 |= I2C_CR1_STOP;	// GENERATE STOP
			I2C1->CR1 &= ~I2C_CR1_PE;	// DISABLE I2C
			bufferTX.state_TX = STOP;
		}
		
		else if( bufferTX.state_TX == TRANSMIT){
			
			if( bufferTX.I2C_MODE == WRITE ){
				
				I2C1->DR = bufferTX.transmitData;
				bufferTX.state_TX = TRANSMITTED;
			}
			else if( bufferTX.I2C_MODE == WRITE_DMA )			// enable the DMA.
				enableTX_DMA();
			
			else{			// read one byte or read with dma.
				bufferTX.state_TX = REG_ADDRESS;
				I2C1->CR1 |= I2C_CR1_START;	// generate restart condition
			}
		}	
	}
	
	if( I2C1->SR1 & I2C_SR1_RXNE){
		
		if( bufferTX.state_TX == RECEIVE){
			bufferRX.receiveData = (char)I2C1->DR;
			I2C1->CR1 &= ~I2C_CR1_ACK;
			i2c_stop();
		}
	}
	
	if( I2C1->SR1 & I2C_SR1_BTF){
		
		if( bufferTX.state_TX == TRANSMIT_DMA_CMPLT ){
			i2c_stop();
		}	
	}
}

//********************* | DMA IRQ HANDLER FUNCTIONS | **************************************
void DMA1_Channel7_IRQHandler(void){	// I2C1 rx
	// if i2c1 is used
	DMA1->IFCR = DMA_IFCR_CTCIF7;
	DMA1_Channel7->CCR &= ~(uint32_t)DMA_CCR7_EN;
	I2C1->CR1 &= ~I2C_CR1_ACK;	//NACK- unnecessary - I'm going to test it without writing this line.
	i2c_stop();
}

void DMA1_Channel6_IRQHandler(void){	// I2C1 tx
	// if i2c1 is used
	DMA1->IFCR = DMA_IFCR_CTCIF6;
	DMA1_Channel6->CCR &= ~(uint32_t)DMA_CCR6_EN;
	bufferTX.state_TX = TRANSMIT_DMA_CMPLT;
}
