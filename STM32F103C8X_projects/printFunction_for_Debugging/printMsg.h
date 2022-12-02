#include <stdint.h>
#include "string.h"
#include "stdlib.h"
#include "stdarg.h"
#include "stm32f10x.h"

// #################|	var/defines	|########################

#define DEBUG_UART		USART1
#define delay			for(int i=0;i<500000;i++)

// #################| 	STRUCTURES	|########################
typedef struct{
	uint16_t 		baud;
	GPIO_TypeDef 	*tx_port;
//	uint8_t 		TX_pinNumber;
	USART_TypeDef	*Uart_instance;
}printMsg_config;

// #################| 	UART-CONFIG	|########################
uint8_t printMsg_init(printMsg_config Transmit);
static uint8_t UsartBaudRateSet(USART_TypeDef *UsartP, uint16_t baudR); 
// #################|	Prototypes	|########################

void print(const char *msg, ...);

