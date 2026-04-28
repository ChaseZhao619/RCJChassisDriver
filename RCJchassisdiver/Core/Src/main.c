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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_usart.h"
#include "bsp_bno085.h"
#include "bsp_motor.h"
#include "bsp_suction_motor.h"
#include "bsp_suction_motor_test.h"
#include "app_chassis_task.h"
#include "app_pi_comm.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#ifndef MAIN_IMU_PRINT_ENABLE
#define MAIN_IMU_PRINT_ENABLE 0U
#endif

#ifndef MAIN_CHASSIS_YAW_TIMEOUT_MS
#define MAIN_CHASSIS_YAW_TIMEOUT_MS 300U
#endif

#ifndef MAIN_ZERO_KEY_DEBOUNCE_MS
#define MAIN_ZERO_KEY_DEBOUNCE_MS 30U
#endif

#ifndef MAIN_LOOP_DELAY_MS
#define MAIN_LOOP_DELAY_MS 1U
#endif

#ifndef MAIN_DEBUG_USART
#define MAIN_DEBUG_USART BSP_USART_1
#endif

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
float bno085_gyro_z_deg_s;
uint8_t bno085_zero_key_sample;
uint8_t bno085_zero_key_stable;
uint8_t bno085_zero_key_handled;
uint8_t bno085_rotation_valid;
uint32_t bno085_last_print_tick;
uint32_t bno085_yaw_update_tick;
uint32_t bno085_gyro_update_tick;
uint32_t bno085_zero_key_debounce_tick;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
HAL_StatusTypeDef Main_ResetYawZero(void)
{
  uint32_t now = HAL_GetTick();

  if (bno085_rotation_valid == 0U)
  {
    return HAL_BUSY;
  }

  if (Bno085_SetYawZero(&bno085_rotation_vector) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (Bno085_GetYawDegrees(&bno085_rotation_vector, &bno085_yaw_deg) != HAL_OK)
  {
    return HAL_ERROR;
  }

  bno085_yaw_update_tick = now;
  AppChassisTask_OnYawZero(bno085_yaw_deg);
  Printf(MAIN_DEBUG_USART, "imu_zero:1\n");

  return HAL_OK;
}

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
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  if (BspMotor_Init() != HAL_OK)
  {
    Error_Handler();
  }
  if (BspSuctionMotor_Init() != HAL_OK)
  {
    Error_Handler();
  }
  BspSuctionMotorTest_Init();
  AppChassisTask_Init();
  AppPiComm_Init();

  Printf(MAIN_DEBUG_USART, "BNO085 test start\r\n");
  if (Bno085_Init() == HAL_OK)
  {
    if (Bno085_GetProductId(&bno085_product_id) == HAL_OK)
    {
      Printf(MAIN_DEBUG_USART, "BNO085 addr=0x%02X sw=%u.%u.%u build=%lu part=%lu\r\n",
             bno085_product_id.i2c_addr,
             bno085_product_id.sw_major,
             bno085_product_id.sw_minor,
             bno085_product_id.sw_patch,
             bno085_product_id.sw_build_number,
             bno085_product_id.sw_part_number);
    }

    if (Bno085_EnableDefaultReports(10000U) == HAL_OK)
    {
      Printf(MAIN_DEBUG_USART, "BNO085 default reports enabled\r\n");
    }
    else
    {
      Printf(MAIN_DEBUG_USART, "BNO085 enable default reports failed\r\n");
    }
  }
  else
  {
    Bno085_GetI2cProbeResult(&bno085_probe_result);
    Printf(MAIN_DEBUG_USART, "BNO085 init failed addr=0x%02X int=%u err=%u i2cerr=0x%08lX\r\n",
           Bno085_GetI2cAddress(),
           Bno085_GetIntPinLevel(),
           Bno085_GetLastError(),
           Bno085_GetLastHalI2cError());
    Printf(MAIN_DEBUG_USART, "BNO085 probe 0x4B ready=%u err=0x%08lX, 0x4A ready=%u err=0x%08lX\r\n",
           bno085_probe_result.ready_4b,
           bno085_probe_result.error_4b,
           bno085_probe_result.ready_4a,
           bno085_probe_result.error_4a);
    Bno085_GetLastHeader(bno085_last_header);
    Printf(MAIN_DEBUG_USART, "BNO085 last header=%02X %02X %02X %02X len=%u\r\n",
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
    uint8_t chassis_yaw_valid = 0U;
    uint8_t chassis_gyro_valid = 0U;
    uint32_t now = HAL_GetTick();

    AppPiComm_Task();

    if (Bno085_ReadSensorData(&bno085_sensor_data) == HAL_OK)
    {
      if ((bno085_sensor_data.has_rotation != 0U) &&
          (Bno085_GetYawDegrees(&bno085_sensor_data.rotation, &bno085_yaw_deg) == HAL_OK))
      {
        bno085_rotation_vector = bno085_sensor_data.rotation;
        bno085_rotation_valid = 1U;
        bno085_yaw_update_tick = now;
#if MAIN_IMU_PRINT_ENABLE
        if ((now - bno085_last_print_tick) >= 50U)
        {
          bno085_last_print_tick = now;
          Printf(MAIN_DEBUG_USART,
                 "imu:%.2f,%u,%ld\n",
                 bno085_yaw_deg,
                 bno085_sensor_data.rotation.status,
                 (long)bno085_sensor_data.gyro.z_raw);
        }
#endif
      }

      if (bno085_sensor_data.has_gyro != 0U)
      {
        bno085_gyro_z_deg_s = bno085_sensor_data.gyro.z * 57.2957795f;
        bno085_gyro_update_tick = now;
      }
    }
    {
      uint8_t zero_key_pressed = Bno085_IsZeroKeyPressed();

      if (zero_key_pressed != bno085_zero_key_sample)
      {
        bno085_zero_key_sample = zero_key_pressed;
        bno085_zero_key_debounce_tick = now;
      }

      if ((now - bno085_zero_key_debounce_tick) >= MAIN_ZERO_KEY_DEBOUNCE_MS)
      {
        if (bno085_zero_key_stable != bno085_zero_key_sample)
        {
          bno085_zero_key_stable = bno085_zero_key_sample;
          if (bno085_zero_key_stable == 0U)
          {
            bno085_zero_key_handled = 0U;
          }
        }

        if ((bno085_zero_key_stable != 0U) &&
            (bno085_zero_key_handled == 0U) &&
            (bno085_rotation_valid != 0U))
        {
          if (Main_ResetYawZero() == HAL_OK)
          {
            bno085_zero_key_handled = 1U;
          }
        }
      }
    }
    if ((now - bno085_yaw_update_tick) <= MAIN_CHASSIS_YAW_TIMEOUT_MS)
    {
      chassis_yaw_valid = 1U;
    }
    if ((now - bno085_gyro_update_tick) <= MAIN_CHASSIS_YAW_TIMEOUT_MS)
    {
      chassis_gyro_valid = 1U;
    }
    AppChassisTask_Task(chassis_yaw_valid,
                        bno085_yaw_deg,
                        chassis_gyro_valid,
                        bno085_gyro_z_deg_s);
    BspSuctionMotorTest_Task();
    AppPiComm_Task();
    HAL_Delay(MAIN_LOOP_DELAY_MS);
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
