#include "stm32g4xx.h"

#define STM32F1XX
#define STM32_PCLK1		(36000000U)		// 36 MHz
#define STM32_TIMCLK1		(2*STM32_PCLK1) //72MHz

#define CAN1_TX_IRQHandler      USB_HP_CAN1_TX_IRQHandler 
#define CAN1_RX0_IRQHandler     USB_LP_CAN1_RX0_IRQHandler
#define CAN1_RX1_IRQHandler     CAN1_RX1_IRQHandler
