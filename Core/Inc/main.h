/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal.h"
#include "system_stm32g4xx.h"
//#include "startup.hs"
#include "stm32g4xx_hal_conf.h"
//#include "stm32g4xx_hal_msp.h"
#include "stm32g4xx_it.h"
#include <stdbool.h>


//include our can chip definitions
#define NO_MAVLINK_ENABLED 1
#include "libcanard_module.hpp"
#include "driver_module.hpp"
#include "chip.h"

#include "fdcan.h"
//#include "i2c.h"
//#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include "tim.h"
#include "common.h"


#include "menu.h"
volatile uint8_t uart_buffer[20];
volatile uint8_t buffer_ptr;
volatile bool ymodem_upload;
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
#define STATUS2_Pin GPIO_PIN_1
#define STATUS2_GPIO_Port GPIOB
#define STATSU3_Pin GPIO_PIN_2
#define STATSU3_GPIO_Port GPIOB
#define ERROR0_Pin GPIO_PIN_14
#define ERROR0_GPIO_Port GPIOB
#define ERROR1_Pin GPIO_PIN_15
#define ERROR1_GPIO_Port GPIOB
#define BOOT0_Pin GPIO_PIN_8
#define BOOT0_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
