/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "dma.h"
#include "fdcan.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "common.h"
#include "modlue.h"
#include "mt6701.h"
#include "ntc.h"
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t adc1_buff[2];
uint16_t adc2_buff[4];

/* ===========================================================================
 *  开环 V/f 测试开关
 *
 *  0 = 关闭。1 = 开启: 上电 5 秒后自动跑一段 (加速 -> 匀速 6s -> 减速停机)。
 *
 *  ⚠️ 打开前确认: 电机三相线接好、轴固定好、母线电源接上 (否则欠压会停 PWM)。
 *     本板没有硬件过流保护, U14 未焊时也没有精确的电流环保护 ——
 *     建议母线上串一个 2 欧 / 10W 功率电阻再试。
 *     参数在 User/motor/foc_loop.c 顶部的 OL_xxx 宏
 * =========================================================================== */
#define OPENLOOP_TEST       0

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
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_USB_Device_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_USART3_UART_Init();
  MX_FDCAN1_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(1000);

  /* MT6701 磁编码器: 自动探测 SSI 的 SPI 模式 (会重配 SPI1 为 8bit) */
  mt6701_init();

  HAL_TIM_Base_Start(&htim3);
  // HAL_TIM_Base_Start_IT(&htim1);

  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
  HAL_ADCEx_InjectedStart_IT(&hadc1);
  HAL_ADCEx_InjectedStart(&hadc2);

  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc1_buff, 2);
  HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adc2_buff, 4);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 3900);

  pmsm_init();
  ntc_init();
  foc_pwm_start();
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* ===== 调试输出 (100Hz), 用 VOFA+ (数据引擎选 JustFloat) 看波形 =====
     * 通道: 0=机械角(0~360)   1=磁场状态 mg      2=CRC通过
     *       3=SPI模式         4=累计失败次数     5=累计角度(带圈数)
     *       6=MOS温度(℃)      7=绕组温度(℃)
     *       8=故障位掩码      9=母线电压(V)     10=A相电流(A)   11=q轴电流(A)
     */
    mt6701_read(&mt6701);
    temp_calc();

#if OPENLOOP_TEST
    /* ===== 开环 V/f 测试 =====
     * 上电 5 秒后自动开始 (留时间确认电机和电源状态)。
     * 测试期间由状态机自己管理 ctrl_bit / wr_set / vq_set, 别手动覆盖 */
    {
        static uint32_t ol_boot_cnt = 0u;

        if (ol_boot_cnt < 500u)         /* 5s @100Hz */
        {
            ol_boot_cnt++;
            if (ol_boot_cnt == 500u)
            {
                openloop_test_start();
            }
        }
        openloop_test_run();            /* 内部有故障会自己停 */
    }
#endif

    vofa_send_data(0, mt6701.angle);
    vofa_send_data(1, (float)mt6701.mg);
    vofa_send_data(2, (float)mt6701.crc_ok);
    vofa_send_data(3, (float)mt6701.spi_mode);
    vofa_send_data(4, (float)mt6701.err_cnt);
    vofa_send_data(5, mt6701.total_rad);
    vofa_send_data(6, ntc_mos.temp);      /* NTC1 板载 MOS   (PB1)  */
    vofa_send_data(7, ntc_coil.temp);     /* NTC3 外接绕组   (PB12) */
    vofa_send_data(8, (float)pm.fault.all);  /* 故障掩码: 0 = 正常 */
    vofa_send_data(9, pm.foc.vbus);          /* 母线电压 (V) */
    /* 10/11 发原始 ADC 计数而不是换算后的电流:
     * U14 没焊时 VREF=0, RS624 输出是单极性的, 换算后的电流值是错的,
     * 但原始计数仍然单调反映电流大小 —— 诊断"只抖不转"要看的就是它。
     * 换算: I ≈ (counts - 20) / 24.8  安培 */
    vofa_send_data(10, (float)pm.adc.ia);    /* A 相原始计数 */
    vofa_send_data(11, (float)pm.adc.ic);    /* C 相原始计数 */
    vofa_sendframetail();

    HAL_Delay(10);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
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
