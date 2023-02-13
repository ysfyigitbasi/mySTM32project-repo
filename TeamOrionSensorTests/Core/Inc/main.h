/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define USART2_TX_GPS_RX_Pin GPIO_PIN_2
#define USART2_TX_GPS_RX_GPIO_Port GPIOA
#define USART2_GPS_TX_Pin GPIO_PIN_3
#define USART2_GPS_TX_GPIO_Port GPIOA
#define SPI1_NSS_CS_DAT3_Pin GPIO_PIN_4
#define SPI1_NSS_CS_DAT3_GPIO_Port GPIOA
#define SPI1_SCK_SDcard_CLK_Pin GPIO_PIN_5
#define SPI1_SCK_SDcard_CLK_GPIO_Port GPIOA
#define SPI1_MISO_SDO_DAT0_Pin GPIO_PIN_6
#define SPI1_MISO_SDO_DAT0_GPIO_Port GPIOA
#define SPI1_MOSI_SDI_CMD_Pin GPIO_PIN_7
#define SPI1_MOSI_SDI_CMD_GPIO_Port GPIOA
#define LORA_M1_Pin GPIO_PIN_15
#define LORA_M1_GPIO_Port GPIOB
#define LORA_M0_Pin GPIO_PIN_8
#define LORA_M0_GPIO_Port GPIOA
#define USART1_TX_LORA_RX_Pin GPIO_PIN_9
#define USART1_TX_LORA_RX_GPIO_Port GPIOA
#define USART1_RX_LORA_TX_Pin GPIO_PIN_10
#define USART1_RX_LORA_TX_GPIO_Port GPIOA
#define LORA_AUX_Pin GPIO_PIN_11
#define LORA_AUX_GPIO_Port GPIOA
#define BMP388_INT_Pin GPIO_PIN_3
#define BMP388_INT_GPIO_Port GPIOB
#define BNO_055_INT_Pin GPIO_PIN_4
#define BNO_055_INT_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
