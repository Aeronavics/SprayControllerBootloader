/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "fdcan.h"
#include "i2c.h"
#include "iwdg.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

bool is_timeout_relevant;
bool boot_to_application;
static volatile uint64_t counter;

static volatile bool run_sloppy_100hz = false;
static volatile bool run_sloppy_10hz = false;
static volatile bool run_sloppy_1hz = true;
extern uint8_t bootloader_info_location;

static Driver_module * const drivers[] = {
    static_cast<Driver_module*> (&Libcanard_module::get_driver())
};
static const size_t NUM_DRIVERS = sizeof (drivers) / sizeof (Driver_module*);

bool first_fire;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void IAP_Init(void);
void driverhost_broadcast_can(CanardRxTransfer const* transfer, uint64_t data_type_signature, uint16_t data_type_id, uint8_t* inout_transfer_id, uint8_t priority, const void* payload, uint16_t payload_len, Driver_module * const except)
{}
void Do_Bootloader(bool normal_boot_on_flash, bool timeout);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	//Check we are meant to go into bootloader mode
	    //    volatile uint8_t *param_values = (volatile uint8_t *)(RAM_BOOTLOADER_ACTION_LOCATION);
	    //Also check we have had an actual crash
	    //if (bootloader_info_location == RAM_DO_BOOTLOADER_BYTE || __HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)
	    if (bootloader_info_location == RAM_DO_BOOTLOADER_BYTE)
	    {
		//reset the iwd flag. This can cause trouble and get into an infinite loop of bootloadering
	       __HAL_RCC_CLEAR_RESET_FLAGS();
	//	__HAL_RCC_CLEAR_FLAG(RCC_FLAG_IWDGRST);
	        //overwrite
	       bootloader_info_location = 0;
	        //Carry on with bootloader
	   }
	   else if (bootloader_info_location == RAM_FASTBOOT_BYTE)
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
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  MX_I2C4_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_IWDG_Init();

  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_Base_Start_IT(&htim2);

  /* USER CODE BEGIN 2 */

  Serial_PutString((uint8_t *) "  Enter \"ymodem\" to upload code via ymodem, or init a CAN device\r\n\n");
  HAL_UART_Receive_IT(&UartHandle, (uint8_t *)uart_buffer, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  	 if (true)
  	    {
  	            // Iterate over each of the drivers and run the relevant handler.
  	        for (size_t i = 0; i < NUM_DRIVERS; i++)
  	        {
  	            if (drivers[i] != nullptr) drivers[i]->sync_update_unthrottled();
  	        }
  	        if (ymodem_upload)
  	        {
  	            Do_Bootloader(true, false);
  	        }
  	    }
  	        // Check if we need to run the sloppy 100Hz tasks.
  	    if (run_sloppy_100hz)
  	    {
  	            // Iterate over each of the drivers and run the relevant handler.
  	        for (size_t i = 0; i < NUM_DRIVERS; i++)
  	        {
  	            if (drivers[i] != nullptr) drivers[i]->sync_update_100Hz();
  	        }
  	            // Clear the 100Hz task flag now that the tasks have been run.
  	        run_sloppy_100hz = false;
  	    }

  	        // Check if we need to run the 10Hz tasks.
  	    if (run_sloppy_10hz)
  	    {
  	            // Iterate over each of the drivers and run the relevant handler.
  	            //            display_uart_status();
  	        for (size_t i = 0; i < NUM_DRIVERS; i++)
  	        {
  	            if (drivers[i] != nullptr) drivers[i]->sync_update_10Hz();
  	        }
  	            // Clear the 10Hz task flag now that the tasks have been run.
  	        run_sloppy_10hz = false;
  	    }

  	        // Check if we need to run the 1Hz tasks.
  	    if (run_sloppy_1hz)
  	    {
  	            // // Iterate over each of the drivers and run the relevant handler.
  	        for (size_t i = 0; i < NUM_DRIVERS; i++)
  	        {
  	            if (drivers[i] != nullptr) drivers[i]->sync_update_1Hz();
  	        }
  	        HAL_GPIO_TogglePin(GPIO_ALIVE_GPIO_Port, GPIO_ALIVE_Pin);

  	        bool are_drivers_init = false;
  	        bool are_drivers_normal = false;
  	        bool are_drivers_error = false;
  	        for (size_t i = 0; i < NUM_DRIVERS; i++)
  	        {
  	            if (drivers[i] != nullptr)
  	            {
  	                if (drivers[i]->get_state() == DRIVER_STATE_INIT) are_drivers_init = true;
  	                if (drivers[i]->get_state() == DRIVER_STATE_NORMAL) are_drivers_normal = true;
  	                if (drivers[i]->get_state() == DRIVER_STATE_ERROR) are_drivers_error = true;
  	            }
  	        }

  	            // Clear the 1Hz task flag now that the tasks have been run.
  	        run_sloppy_1hz = false;
  	    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void Do_Boot(void)
{
    if (((*(__IO uint32_t*) (APPLICATION_ADDRESS + APPLICATION_OFFSET)) & 0x2FFE0000) == 0x20000000)
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

uint64_t driverhost_get_monotonic_time_us(void)
{
    /*
     *     the counter interrupts every 10000 microseconds, so we can extrapolate this and add where we currently are up to in the timer
     */
    return (uint64_t) ((counter * 10000) + (uint64_t) __HAL_TIM_GET_COUNTER(&htim2));
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

void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart)
{
    if (huart->Instance == USART1)
    {
        Serial_PutByte(uart_buffer[buffer_ptr]);

        if (strncmp((const char *) uart_buffer, "ymodem", 6) == 0)
        {
            //break the CAN loading
            Libcanard_module::get_driver().kill_loading = true;
            //jump to the main ymodem application
            ymodem_upload = true;
            //do not restart the uart
            return;
        }
        if (buffer_ptr > 10){
            buffer_ptr = 0;
        }

        if (uart_buffer[buffer_ptr] == 'z')
        {
            int i;
            for (int i = 0; i < buffer_ptr; i++)
            {
                uart_buffer[i] = NULL;
            }
            Serial_PutString((uint8_t *) "\r\n");
            buffer_ptr = 0;
        }
        else {
            buffer_ptr++;
        }

        HAL_UART_Receive_IT(&UartHandle, (uint8_t *)uart_buffer + buffer_ptr, 1);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim)
{
    if (htim->Instance == TIM1)
    {
        if (!first_fire)
        {
            first_fire = true;
            return;
        }
        //        HAL_GPIO_TogglePin(GPIO_ALIVE_GPIO_Port, GPIO_ALIVE_Pin);
        //if we are expecting the timeout to do something then lets boot
        //we want to kill the can module from loading if it has not already and get onto booting. If there is nothing to boot this will have no effect
        //check we have an app before we kill the loading
        if (((*(__IO uint32_t*) (APPLICATION_ADDRESS + APPLICATION_OFFSET)) & 0x2FFE0000) == 0x20000000)
        {
            Libcanard_module::get_driver().kill_loading = true;
        }
        if (is_timeout_relevant)
        {
            boot_to_application = true;
            HAL_UART_AbortReceive(&huart1);
        }
    }
    if (htim->Instance == TIM2)
    {
        counter++;
        if (counter % 100 == 0)
        {
            run_sloppy_1hz = true;
        }
        if (counter % 10 == 0)
        {
            run_sloppy_10hz = true;
        }
        run_sloppy_100hz = true;

    }
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
