#ifndef _HAL_GPIO
#define _HAL_GPIO

#include "stm32f10x.h"

#define HIGH		1
#define LOW			0

//port names
#define PORTA		GPIOA
#define PORTB		GPIOB
#define PORTC		GPIOC
#define PORTD		GPIOD
#define PORTE		GPIOE
#define PORTF		GPIOF
#define PORTG		GPIOG

//PIN NAMES

#define OUTPUT_MODE					((uint32_t)	0x01)
#define INPUT_MODE					((uint32_t) 0x02)

//INPUT MODES TYPE
#define INPUT_ANALOG				((uint32_t) 0x00)
#define INPUT_FLOATING				((uint32_t) 0x01) // default at reset
#define INPUT_PU_PD					((uint32_t) 0x02)

//OUTPUT MODES TYPE
#define OUTPUT_GEN_PURPOSE			((uint32_t) 0x00)
#define OUTPUT_OD					((uint32_t) 0x01) // default at reset
#define OUTPUT_ALT_FUNCTION_PP		((uint32_t) 0x02)
#define OUTPUT_ALT_FUNCTION_OD		((uint32_t) 0x03)

//PIN SPEEDS/SLEW RATE
#define SPEED_2MHZ					((uint32_t) 0x02)
#define SPEED_10MHZ					((uint32_t) 0x01)
#define SPEED_50MHZ					((uint32_t) 0x03)

//clock enabling
#define GPIO_CLOCK_ENABLE_ALT_FUNC	(RCC->APB2ENR |= (1<<0))
#define GPIO_CLOCK_ENABLE_PORTA		(RCC->APB2ENR |= (1<<2))
#define GPIO_CLOCK_ENABLE_PORTB		(RCC->APB2ENR |= (1<<3))
#define GPIO_CLOCK_ENABLE_PORTC		(RCC->APB2ENR |= (1<<4))
#define GPIO_CLOCK_ENABLE_PORTD		(RCC->APB2ENR |= (1<<5))
//#define GPIO_CLOCK_ENABLE_PORTE		(RCC->APB2ENR |= (1<<6))

//HIGH BIT POSITION FOR CRH REGISTER CNF AND MODE
#define CNF_POS_BIT1				(PINPOS[pinNumber] + 2)
#define CNF_POS_BIT2				(PINPOS[pinNumber] + 3)

// Configuration Structure
typedef struct{
	GPIO_TypeDef *port;
	uint32_t pin;
	uint32_t mode;
	uint32_t mode_type;
	uint32_t speed;
}GPIO_TYPE;

// GPIO-INTERRUPTS ENUM
typedef enum{
	EDGE_RISING,
	EDGE_FALLING,
	EDGE_RISING_FALLING
}edge_select;

//Function Prototypes
//***************************************************************
//							GPIO CONFIGURATION
static void config_pin(GPIO_TypeDef *port, uint32_t pinNumber, uint32_t mode_type);
static void config_pin_speed(GPIO_TypeDef *port, uint32_t pinNumber, uint32_t pinSpeed, uint32_t mode);

//****************************************************************
//						  GPIO USER PIN FUNCTIONS
void gpio_write(GPIO_TypeDef *port, uint32_t pinNumber, uint8_t state);
void gpio_toggle(GPIO_TypeDef *port, uint32_t pinNumber);
void gpio_init( GPIO_TYPE gpio_type);

//****************************************************************
//							INTERRUPT FUNCTIONS

void config_gpio_interrupt(GPIO_TypeDef *port, uint32_t pinNumber, edge_select edge);
void enable_gpio_interrupt(uint32_t pinNumber, IRQn_Type irqNumber);
void clear_gpio_interrupt(uint32_t pinNumber);



#endif
