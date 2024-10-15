/**
 ******************************************************************************
 * @file    IAP_Main/Src/main.c
 * @author  MCD Application Team
 * @version V1.6.0
 * @date    12-May-2017
 * @brief   Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include <stm32G474RTC6_bootloader.hpp>

bool is_timeout_relevant;
bool boot_to_application;
static volatile uint64_t counter;
//initiate drivers
static volatile bool run_sloppy_100hz = false;
static volatile bool run_sloppy_10hz = false;
static volatile bool run_sloppy_1hz = true;
extern uint8_t bootloader_info_location;

static Driver_module *const drivers[] =
{ static_cast<Driver_module*>(&Libcanard_module::get_driver()) };
static const size_t NUM_DRIVERS = sizeof(drivers) / sizeof(Driver_module*);

/* Private function prototypes -----------------------------------------------*/
static void
IAP_Init(void);
void driverhost_broadcast_can(CanardRxTransfer const *transfer,
                              uint64_t data_type_signature,
                              uint16_t data_type_id, uint8_t *inout_transfer_id,
                              uint8_t priority, const void *payload,
                              uint16_t payload_len, Driver_module *const except)
{
}
void
Do_Bootloader(bool normal_boot_on_flash, bool timeout);

bool first_fire;

/**
 * @brief  Main program
 * @param  None
 * @retval None
 */
int main(void)
{
	//Check we are meant to go into bootloader mode
	//    volatile uint8_t *param_values = (volatile uint8_t *)(RAM_BOOTLOADER_ACTION_LOCATION);
	//Also check we have had an actual crash
	//if (bootloader_info_location == RAM_DO_BOOTLOADER_BYTE || __HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)
	if(bootloader_info_location == RAM_DO_BOOTLOADER_BYTE)
	{
		//reset the iwd flag. This can cause trouble and get into an infinite loop of bootloadering
		__HAL_RCC_CLEAR_RESET_FLAGS();
//	__HAL_RCC_CLEAR_FLAG(RCC_FLAG_IWDGRST);
		//overwrite
		bootloader_info_location = 0;
		//Carry on with bootloader
	}
	else if(bootloader_info_location == RAM_FASTBOOT_BYTE)
	{
		//overwrite
		bootloader_info_location = 0;
		Boot();
	}
	else
	{
		//we default to booting.
		Boot();
	}

	Libcanard_module::get_driver().set_name("bootloader.aeronavics.com");
	HAL_Init();

	/* Configure the system clock to 72 MHz */
	SystemClock_Config();
	MX_GPIO_Init();
	//turn on power on the ground station
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_SET);
	//keep the peripheral power on
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_SET);
	//    MX_I2C1_Init();
	MX_TIM1_Init();
	MX_TIM2_Init();
	//    MX_IWDG_Init();
	//start the interrupt - this should take 5 seconds
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_Base_Start_IT(&htim2);

	MX_USART1_UART_Init();

	//start a uart receive, this will interrupt and cause our code to split into uart bootloader mode or into can bootloader mode
	//(or nothing! which just boots!)

	Serial_PutString(
	    (uint8_t*) "  Enter \"ymodem\" to upload code via ymodem, or init a CAN device\r\n\n");
	HAL_UART_Receive_IT(&UartHandle, (uint8_t*) uart_buffer, 1);

	while(1)
	{
		//        HAL_IWDG_Refresh(&hiwdg);

		if(true)
		{
			// Iterate over each of the drivers and run the relevant handler.
			for(size_t i = 0; i < NUM_DRIVERS; i++)
			{
				if(drivers[i] != nullptr)
					drivers[i]->sync_update_unthrottled();
			}
			if(ymodem_upload)
			{
				Do_Bootloader(true, false);
			}
		}
		// Check if we need to run the sloppy 100Hz tasks.
		if(run_sloppy_100hz)
		{
			// Iterate over each of the drivers and run the relevant handler.
			for(size_t i = 0; i < NUM_DRIVERS; i++)
			{
				if(drivers[i] != nullptr)
					drivers[i]->sync_update_100Hz();
			}
			// Clear the 100Hz task flag now that the tasks have been run.
			run_sloppy_100hz = false;
		}

		// Check if we need to run the 10Hz tasks.
		if(run_sloppy_10hz)
		{
			// Iterate over each of the drivers and run the relevant handler.
			//            display_uart_status();
			for(size_t i = 0; i < NUM_DRIVERS; i++)
			{
				if(drivers[i] != nullptr)
					drivers[i]->sync_update_10Hz();
			}
			// Clear the 10Hz task flag now that the tasks have been run.
			run_sloppy_10hz = false;
		}

		// Check if we need to run the 1Hz tasks.
		if(run_sloppy_1hz)
		{
			// // Iterate over each of the drivers and run the relevant handler.
			for(size_t i = 0; i < NUM_DRIVERS; i++)
			{
				if(drivers[i] != nullptr)
					drivers[i]->sync_update_1Hz();
			}
			HAL_GPIO_TogglePin(GPIO_ALIVE_GPIO_Port, GPIO_ALIVE_Pin);

			bool are_drivers_init = false;
			bool are_drivers_normal = false;
			bool are_drivers_error = false;
			for(size_t i = 0; i < NUM_DRIVERS; i++)
			{
				if(drivers[i] != nullptr)
				{
					if(drivers[i]->get_state() == DRIVER_STATE_INIT)
						are_drivers_init = true;
					if(drivers[i]->get_state() == DRIVER_STATE_NORMAL)
						are_drivers_normal = true;
					if(drivers[i]->get_state() == DRIVER_STATE_ERROR)
						are_drivers_error = true;
				}
			}

			// Clear the 1Hz task flag now that the tasks have been run.
			run_sloppy_1hz = false;
		}
	}
}

void Do_Boot(void)
{
	if(((*(__IO uint32_t*) (APPLICATION_ADDRESS + APPLICATION_OFFSET ))
	    & 0x2FFE0000) == 0x20000000)
	{
		Set_Boot();
		NVIC_SystemReset();
	}
	else
	{
		//jump to bootloader if we have nothing programmed
		//if we flash successfully, lets update the boot eeprom value
		Do_Bootloader(true, false);
	}
}

void Do_Bootloader(bool normal_boot_on_flash, bool timeout)
{
	/* Initialise Flash */
	FLASH_If_Init();
	/* Execute the IAP driver in order to reprogram the Flash */
	IAP_Init();
	/* Display main menu */
	Main_Menu(normal_boot_on_flash, timeout);
}

/**
 * @brief  Initialize the IAP: Configure USART.
 * @param  None
 * @retval None
 */
void IAP_Init(void)
{
	/* USART resources configuration (Clock, GPIO pins and USART registers) ----*/
	/* USART configured as follow:
	 - BaudRate = 115200 baud
	 - Word Length = 8 Bits
	 - One Stop Bit
	 - No parity
	 - Hardware flow control disabled (RTS and CTS signals)
	 - Receive and transmit enabled
	 */
	MX_USART1_UART_Init();

}

#ifdef USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

    /* Infinite loop */
    while (1)
    {
    }
}
#endif

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1)
	{
		Serial_PutByte(uart_buffer[buffer_ptr]);

		if(strncmp((const char*) uart_buffer, "ymodem", 6) == 0)
		{
			//break the CAN loading
			Libcanard_module::get_driver().kill_loading = true;
			//jump to the main ymodem application
			ymodem_upload = true;
			//do not restart the uart
			return;
		}
		if(buffer_ptr > 10)
		{
			buffer_ptr = 0;
		}

		if(uart_buffer[buffer_ptr] == 'z')
		{
			int i;
			for(int i = 0; i < buffer_ptr; i++)
			{
				uart_buffer[i] = NULL;
			}
			Serial_PutString((uint8_t*) "\r\n");
			buffer_ptr = 0;
		}
		else
		{
			buffer_ptr++;
		}

		HAL_UART_Receive_IT(&UartHandle, (uint8_t*) uart_buffer + buffer_ptr, 1);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM1)
	{
		if(!first_fire)
		{
			first_fire = true;
			return;
		}
		//        HAL_GPIO_TogglePin(GPIO_ALIVE_GPIO_Port, GPIO_ALIVE_Pin);
		//if we are expecting the timeout to do something then lets boot
		//we want to kill the can module from loading if it has not already and get onto booting. If there is nothing to boot this will have no effect
		//check we have an app before we kill the loading
		if(((*(__IO uint32_t*) (APPLICATION_ADDRESS + APPLICATION_OFFSET ))
		    & 0x2FFE0000) == 0x20000000)
		{
			Libcanard_module::get_driver().kill_loading = true;
		}
		if(is_timeout_relevant)
		{
			boot_to_application = true;
			HAL_UART_AbortReceive(&huart1);
		}
	}
	if(htim->Instance == TIM2)
	{
		counter++;
		if(counter % 100 == 0)
		{
			run_sloppy_1hz = true;
		}
		if(counter % 10 == 0)
		{
			run_sloppy_10hz = true;
		}
		run_sloppy_100hz = true;

	}
}

uint64_t driverhost_get_monotonic_time_us(void)
{
	/*
	 *     the counter interrupts every 10000 microseconds, so we can extrapolate this and add where we currently are up to in the timer
	 */
	return (uint64_t) ((counter * 10000)
	    + (uint64_t) __HAL_TIM_GET_COUNTER(&htim2));
}
/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
