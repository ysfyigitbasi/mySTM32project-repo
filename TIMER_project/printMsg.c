#include "printMsg.h"

// #####################|	UART-CONFIG	  |#######################
uint8_t printMsg_init(printMsg_config Transmit){
	
	if(Transmit.Uart_instance == USART1){
	
		if(Transmit.tx_port == GPIOA){ // pa9-tx
			RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN | RCC_APB2ENR_USART1EN;
			GPIOA->CRH |= GPIO_CRH_CNF9_1 | GPIO_CRH_MODE9_0 | GPIO_CRH_MODE9_1; // MODE(1,1) 50MHZ, CNF(1,0) AF_PP
			GPIOA->CRH &= ~GPIO_CRH_CNF9_0;
			
			return UsartBaudRateSet(USART1, Transmit.baud);	} // IF-END GPIOA

			
		else if(Transmit.tx_port == GPIOB){ // pb6-tx
			
			AFIO->MAPR |= AFIO_MAPR_USART1_REMAP;
			RCC->APB2ENR |= RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN | RCC_APB2ENR_USART1EN;
			GPIOB->CRL |= GPIO_CRL_MODE6_0 | GPIO_CRL_MODE6_1 | GPIO_CRL_CNF6_1;
			GPIOB->CRL &= ~GPIO_CRL_CNF6_0;
			return UsartBaudRateSet(USART1, Transmit.baud);	} // IF-END GPIOB

		else
			return 0;	} // IF-END USART1
	
	if(Transmit.Uart_instance == USART2){ // WARNING! ONLY PA2 OUR MC HAS 48 PINS, THEREFORE REMAP DOES NOT AVALIABLE!!!!

		RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
		RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN;
		GPIOA->CRL |= GPIO_CRL_MODE2_0 | GPIO_CRL_MODE2_1 | GPIO_CRL_CNF2_1;
		GPIOA->CRL &= ~GPIO_CRL_CNF2_0;
		return UsartBaudRateSet(USART2, Transmit.baud);	} // IF-END USART2
		
	if(Transmit.Uart_instance == USART3){
			
		RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
		RCC->APB2ENR |= RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN;
		GPIOB->CRH |= GPIO_CRH_MODE10_0 | GPIO_CRH_MODE10_1 | GPIO_CRH_CNF10_1;
		GPIOB->CRH &= ~GPIO_CRH_CNF10_0;
			
		return UsartBaudRateSet(USART3, Transmit.baud);	} // IF-End GPIOB
	return 0;	} // END FUNCTION

static uint8_t UsartBaudRateSet(USART_TypeDef *UsartP, uint32_t baudR){

	switch(baudR){
		
		case 2400:
			
			if(UsartP == USART1){
				
				USART1->BRR = 0x7530;
				USART1->CR1 |= USART_CR1_TE | USART_CR1_UE;
				return 1;	} // END IF USART1 2.4KBPS
			
			if(UsartP == USART2){
				
				USART2->BRR = 0x3A98;
				USART2->CR1 |= USART_CR1_TE | USART_CR1_UE;
				return 1;	} // END IF USART2 2.4KBPS
			
			if(UsartP == USART3){
				
				USART3->BRR = 0x3A98;
				USART3->CR1 |= USART_CR1_TE | USART_CR1_UE;
				return 1;	} // END IF USART3 2.4KBPS
			
			break;	// CASE 1 IS GENERATED...
		
		case 9600:
			
			if(UsartP == USART1){
				
				USART1->BRR = 0x1D4C;
				USART1->CR1 |= USART_CR1_TE | USART_CR1_UE;
				return 1;	} // END IF USART1 9.6KBPS
			
			if(UsartP == USART2){
				
				USART2->BRR = 0x0EA6;
				USART2->CR1 |= USART_CR1_TE | USART_CR1_UE;
				return 1;	} // END IF USART2 9.6KBPS
			
			if(UsartP == USART3){
				
				USART3->BRR = 0x0EA6;
				USART3->CR1 |= USART_CR1_TE | USART_CR1_UE;
				return 1;	} // END IF USART3 9.6KBPS
			
			break;	// CASE 2 IS GENERATED...

		case 19200:
			
			if(UsartP == USART1){
				
				USART1->BRR = 0x0EA6;
				USART1->CR1 |= USART_CR1_TE | USART_CR1_UE;
				return 1;	} // END IF USART1 19.2KBPS
			
			if(UsartP == USART2){
				
				USART2->BRR = 0x0753;
				USART2->CR1 |= USART_CR1_TE | USART_CR1_UE;
				return 1;	} // END IF USART2 19.2KBPS
			
			if(UsartP == USART3){
				
				USART3->BRR = 0x0753;
				USART3->CR1 |= USART_CR1_TE | USART_CR1_UE;
				return 1;	} // END IF USART3 19.2KBPS
			
			break;	// CASE 3 IS GENERATED...
		
		case 57600:
			
			if(UsartP == USART1){
				
				USART1->BRR = 0x04E2;
				USART1->CR1 |= USART_CR1_TE | USART_CR1_UE;
				return 1;	} // END IF USART1 57.6KBPS
			
			if(UsartP == USART2){
				
				USART2->BRR = 0x0271;
				USART2->CR1 |= USART_CR1_TE | USART_CR1_UE;
				return 1;	} // END IF USART2 57.6KBPS
			
			if(UsartP == USART3){
				
				USART3->BRR = 0x0271;
				USART3->CR1 |= USART_CR1_TE | USART_CR1_UE;
				return 1;	} // END IF USART3 57.6KBPS
			
			break;	// CASE 4 IS GENERATED...
			
		case 115200:
			if(UsartP == USART1){
				
				USART1->BRR = 0x0271;
				USART1->CR1 |= USART_CR1_TE | USART_CR1_UE;
				return 1;		}
			
			break;	// CASE 5 IS GENERATED...

		case 2250000:
			
			if(UsartP == USART1){
				
				USART1->BRR = 0x0020;
				USART1->CR1 |= USART_CR1_TE | USART_CR1_UE;
				return 1;		}
			
			if(UsartP == USART2){
				
				USART2->BRR = 0x0010;
				USART2->CR1 |= USART_CR1_TE | USART_CR1_UE;
				return 1;		}

			if(UsartP == USART3){
				
				USART3->BRR = 0x0010;
				USART3->CR1 |= USART_CR1_TE | USART_CR1_UE;
				return 1;		}	
			
			break;
			
		case 4500000:
			if(UsartP == USART1){

				USART1->BRR = 0x0010;
				USART1->CR1 |= USART_CR1_TE | USART_CR1_UE;
				return 1;		}
			
			break;
	}	// end switch
	return 0;
}		// end function


void printMsg(USART_TypeDef *UsartP, const char *msg, ...){
	
	char buff[50];
	va_list args;
	va_start(args, msg);
	vsprintf(buff, msg, args);
		
	for(uint8_t i=0; i<strlen(buff); i++){
		UsartP->DR = buff[i];
		while( !(UsartP->SR & USART_SR_TXE) );
	}	
	
}

