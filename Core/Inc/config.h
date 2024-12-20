/*
 * config.h
 *
 *  Created on: Nov 14, 2024
 *      Author: jmorritt
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

#define FIRMWARE_VERSION 1.0

#define STM32G4XX
#define MAIN_HEADER "SprayController.hpp"

#define STM_HAL "stm32g4xx_hal.h"
#define STM_CAN "fdcan.h"
#define STM_HAL_IWDG "stm32g4xx_hal_iwdg.h"
#define STM_HAL_FLASH "stm32g4xx_hal_flash.h"

// Enable libcanard
#define LIBCANARD_ENABLED

#define LIBCANARD_MESSAGE_NODE
#define LIBCANARD_MESSAGE_FIRMWARE_UPGRADE

#define NO_MAVLINK_ENABLED

#define UART1_ENABLED

#define CANARD_MULTI_IFACE

#endif /* INC_CONFIG_H_ */
