/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "can.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_usart.h"
#include "bsp_bno085.h"
#include "bsp_motor.h"
#include "bsp_chassis_test.h"

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
Bno085ProductId bno085_product_id;
Bno085RotationVector bno085_rotation_vector;
Bno085SensorData bno085_sensor_data;
Bno085I2cProbeResult bno085_probe_result;
uint8_t bno085_last_header[4];
float bno085_yaw_deg;
uint8_t bno085_zero_key_last;
uint32_t bno085_last_print_tick;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  MX_CAN1_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  if (BspMotor_Init() != HAL_OK)
  {
    Error_Handler();
  }
  BspChassisTest_Init();

  Printf(BSP_USART_6, "BNO085 test start\r\n");
  if (Bno085_Init() == HAL_OK)
  {
    if (Bno085_GetProductId(&bno085_product_id) == HAL_OK)
    {
      Printf(BSP_USART_6, "BNO085 addr=0x%02X sw=%u.%u.%u build=%lu part=%lu\r\n",
             bno085_product_id.i2c_addr,
             bno085_product_id.sw_major,
             bno085_product_id.sw_minor,
             bno085_product_id.sw_patch,
             bno085_product_id.sw_build_number,
             bno085_product_id.sw_part_number);
    }

    if (Bno085_EnableDefaultReports(10000U) == HAL_OK)
    {
      Printf(BSP_USART_6, "BNO085 default reports enabled\r\n");
    }
    else
    {
      Printf(BSP_USART_6, "BNO085 enable default reports failed\r\n");
    }
  }
  else
  {
    Bno085_GetI2cProbeResult(&bno085_probe_result);
    Printf(BSP_USART_6, "BNO085 init failed addr=0x%02X int=%u err=%u i2cerr=0x%08lX\r\n",
           Bno085_GetI2cAddress(),
           Bno085_GetIntPinLevel(),
           Bno085_GetLastError(),
           Bno085_GetLastHalI2cError());
    Printf(BSP_USART_6, "BNO085 probe 0x4B ready=%u err=0x%08lX, 0x4A ready=%u err=0x%08lX\r\n",
           bno085_probe_result.ready_4b,
           bno085_probe_result.error_4b,
           bno085_probe_result.ready_4a,
           bno085_probe_result.error_4a);
    Bno085_GetLastHeader(bno085_last_header);
    Printf(BSP_USART_6, "BNO085 last header=%02X %02X %02X %02X len=%u\r\n",
           bno085_last_header[0],
           bno085_last_header[1],
           bno085_last_header[2],
           bno085_last_header[3],
           Bno085_GetLastPacketLength());
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    BspChassisTest_Task();

    if (Bno085_ReadSensorData(&bno085_sensor_data) == HAL_OK)
    {
      uint8_t zero_key_pressed = Bno085_IsZeroKeyPressed();

      if ((zero_key_pressed != 0U) && (bno085_zero_key_last == 0U))
      {
        if (Bno085_SetYawZero(&bno085_sensor_data.rotation) == HAL_OK)
        {
          Printf(BSP_USART_6, "imu_zero:1\n");
        }
      }
      bno085_zero_key_last = zero_key_pressed;

      if ((bno085_sensor_data.has_rotation != 0U) &&
          (Bno085_GetYawDegrees(&bno085_sensor_data.rotation, &bno085_yaw_deg) == HAL_OK) &&
          ((HAL_GetTick() - bno085_last_print_tick) >= 50U))
      {
        bno085_last_print_tick = HAL_GetTick();
        Printf(BSP_USART_6,
               "imu:%.2f,%u,%ld\n",
               bno085_yaw_deg,
               bno085_sensor_data.rotation.status,
               (long)bno085_sensor_data.gyro.z_raw);
      }
    }
    HAL_Delay(5);
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
