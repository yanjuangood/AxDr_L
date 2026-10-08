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
#include "host_cmd.h"
/* USER CODE END Includes */
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
/* ===================== 测试模式开关 (两个只能开一个) =====================
 *
 *  HWTEST_MODE  : 硬件自检。三相固定占空比 0.25/0.50/0.75,
 *                 **不接电机**, 用万用表量三个输出端对 GND。
 *                 期望 2.85 / 5.70 / 8.55 V (母线 11.4V)。
 *                 新板首次上电 / 修完功率级之后先跑这个。
 *
 *  OPENLOOP_TEST: 开环 V/f 驱动电机。上电 5 秒后自动开始,
 *                 加速到 15 rad/s (143rpm), 跑 6 秒后自动减速停机。
 *
 *  CALIB_MODE   : 电角度零点 e_off 自动标定。**必须空载**。
 *                 把电压矢量先后钉在 0 度和 180 度电角度上, 各等转子停稳,
 *                 读出编码器算出 e_off, 并直接写回 pm.para.e_off。
 *                 中间结果在 calib_e_off_a / calib_e_off_b, 两者应一致。
 *
 *  ENCV_TEST    : 用标定后的电角度加 1.5V q 轴电压。空载应持续转动约 5 秒。
 *                 只抖不转说明 e_off 还不对。
 *
 *  CURR_TEST    : 电流环静态测试。电角度钉死在 0 度, 命令 id=0 / iq=0.15A,
 *                 电流闭环。转子会被吸住停着不转 —— 这是有意的。
 *                 看 ct_i_q 是否跟到 0.15, ct_i_d 是否接近 0。
 *                 任一相电流超过 0.6A 会自动停机 (防 PI 极性接反时发散)。
 *
 *  CURR_ENC_TEST: 真 FOC —— 电角度来自编码器 (p_e = rad*pn + e_off) + 电流环。
 *                 iq 给 0.03A, 电机会平稳转起来; 超过 40 rad/s 撤 iq 滑行。
 *                 看 ce_wr 是否往正方向涨、ce_i_q 是否跟得上。
 *                 这是验证 e_off 标定对不对的最终测试。
 *
 *  HOST_MODE    : 上位机控制。所有测试模式关闭, 由 PC 通过 USB CDC 发命令驱动。
 *                 中断里的 pmsm_mode_ctrl() 本来就在按 pm.foc.mode 分派,
 *                 所以这里只要按 START/STOP 开关 ctrl_bit, 剩下的交给中断。
 *                 协议和参数表见 User/moldue/host_cmd.c, PC 端见 tools/host_gui.py。
 *
 *  ⚠️ 优先级: HWTEST_MODE > CALIB_MODE > HOST_MODE > CURR_TEST > CURR_ENC_TEST
 *             > ENCV_TEST > OPENLOOP_TEST, 只会跑一个。
 * ======================================================================== */
#define HWTEST_MODE         0
#define CALIB_MODE          0
#define HOST_MODE           1
#define CURR_TEST           0
#define CURR_ENC_TEST       0
#define ENCV_TEST           0
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
  /* 六路栅极先拉成 GPIO 低电平。MOE 保持打开，否则注入 ADC 没有触发。 */
  foc_pwm_stop();

  pmsm_init();
  ntc_init();
  /* 三相桥先保持关闭，进入 opera（开环开始）时由状态机打开。
   * 这里若 foc_pwm_start()，上电后就会 50% 互补开关，MOS 空载发热。 */
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* ===== 调试输出 (100Hz), 用 VOFA+ (数据引擎选 JustFloat) 看波形 =====
     * 通道: 0=机械角(0~360)   1=磁场状态 mg      2=CRC通过
     *       3=SPI模式         4=相电流原始计数峰值   5=累计角度(带圈数)
     *       6=MOS温度(℃)      7=q轴电压给定(V)  0 表示现在没在驱动
     *       8=故障位掩码      9=母线电压(V)     10=A相原始计数  11=C相原始计数
     */
    mt6701_read(&mt6701);
    temp_calc();

#if HWTEST_MODE
    /* ===== 硬件自检 =====
     * 三相固定占空比 0.25 / 0.50 / 0.75, 全程保持不变。
     * 不接电机, 用万用表量板上三个输出端对 GND:
     *     2.85 V / 5.70 V / 8.55 V  (母线 11.4V)
     * 三个值明显不同就是好的。等母线起来再量 (Ctrl 里 vbus > 8V)。 */
    hwt_self_test_run();

#elif CALIB_MODE
    /* ===== 电角度零点标定 =====
     * 必须空载! 上电后自动跑, 约 6 秒完成, 结果写回 pm.para.e_off。
     * 用 VOFA+ 或 SWD 看 calib_e_off_a / calib_e_off_b, 两者应一致。 */
    calib_run();

#elif HOST_MODE
    /* ===== 上位机控制 =====
     * 收到上位机的 CAL 命令就跑一次标定, 否则正常听命令。
     *
     * 这里不直接驱动电机: 中断里的 pmsm_mode_ctrl() 按 pm.foc.mode 分派,
     * 已经把 V/f / 电流环 / 速度环三套逻辑都接好了。
     * 主循环只需要:
     *   1. 处理串口命令 (改参数、START/STOP)
     *   2. 把回复发出去
     *   3. START 之后把 ctrl_bit 保持在 opera, 让中断去跑控制
     *
     * 这样上位机改一个 mode 或 spd, 下一个 50us 中断就生效, 没有额外延迟。 */
    host_cmd_poll();

    /* 转速测量必须在主循环做: 编码器是这里读的, 在 20kHz 中断里微分
     * 同一个角度值只会得到尖峰噪声。详见 spd_measure_update() 的说明 */
    spd_measure_update();

    if (host_cal_request != 0u)
    {
        /* 标定期间由 calib_run 自己管 ctrl_bit, 结束后清标志 */
        calib_run();
        if (calib_step == 5u)       /* CALIB_DONE */
        {
            host_cal_request = 0u;
            pm.ctrl_bit      = reset;
        }
    }
    else
    {
        /* START / STOP 只改 ctrl_bit; 已经在跑就保持 opera。
         * host_start_request 是运动类参数写入 (mode/spd/iq...) 时置的 ——
         * STOP 之后单靠下面那句是起不来的 (ctrl_bit 已经是 reset),
         * 所以这里要显式回到 start 让状态机重新开 PWM */
        if (host_start_request != 0u)
        {
            host_start_request = 0u;
            pm.ctrl_bit = start;
        }

        if (pm.ctrl_bit != reset)
        {
            pm.ctrl_bit = opera;
        }
    }

    host_cmd_tx_task();

#elif CURR_ENC_TEST
    /* ===== 真 FOC: 编码器电角度 + 电流环 =====
     * p_e = mt6701.rad * pn + e_off, 电流环跟着转子走。
     * 给 iq = 0.03A 电机应平稳转起来, 超过 40 rad/s 撤 iq 滑行。
     * 看 ce_wr / ce_i_q / ce_i_d / ce_p_e / ce_done。 */
    curr_enc_test_run();

#elif CURR_TEST
    /* ===== 电流环静态测试 =====
     * 电角度钉死在 0 度 -> 空间矢量固定 -> 转子被吸住不转。
     * 电流环应该把 i_q 调到 0.15A、i_d 调到 0。
     * 看 ct_i_d / ct_i_q / ct_v_d / ct_v_q / ct_done (SWD 直接读)。 */
    curr_test_run();

#elif ENCV_TEST
    /* 上电 2 秒后，用编码器电角度输出 0.8V。转够约 5 秒或发现不转就停 */
    {
        static uint32_t enc_n  = 0u;
        static float    rad0   = 0.0f;
        static uint8_t  active = 0u;

        enc_n++;
        if (pm.fault.bit.enc_err != 0u)
        {
            enc_volt_en    = 0u;
            pm.ctrl.vq_set = 0.0f;
            pm.ctrl_bit    = reset;
            active         = 0u;
        }
        else if (enc_n < 200u)
        {
            /* 等母线和编码器稳定 */
        }
        else if (enc_n == 200u)
        {
            rad0           = mt6701.total_rad;
            enc_volt_en    = 1u;
            pm.foc.mode    = foc_volt_mode;
            pm.ctrl.vd_set = 0.0f;
            pm.ctrl.vq_set = 1.5f;
            pm.ctrl.wr_set = 0.0f;
            pm.ctrl_bit    = opera;
            active         = 1u;
        }
        else if (active != 0u && enc_n < 700u)
        {
            float moved = mt6701.total_rad - rad0;
            if (moved < 0.0f)
            {
                moved = -moved;
            }
            /* 出力 1.5 秒后机械角几乎没变，判定没对上，立刻停 */
            if (enc_n > 350u && moved < 0.4f)
            {
                enc_volt_en    = 0u;
                pm.ctrl.vq_set = 0.0f;
                pm.ctrl_bit    = reset;
                active         = 0u;
            }
            else
            {
                pm.foc.mode    = foc_volt_mode;
                pm.ctrl.vq_set = 1.5f;
                pm.ctrl_bit    = opera;
            }
        }
        else
        {
            enc_volt_en    = 0u;
            pm.ctrl.vq_set = 0.0f;
            pm.ctrl_bit    = reset;
            active         = 0u;
        }
    }

#elif OPENLOOP_TEST
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

    /* ===== 调试输出 16 通道 (100Hz) =====
     * U14 (TLV9001) 焊上之后电流采样已经有效, 所以这里直接发换算后的安培值。
     * 之前发的是原始 ADC 计数, 那是 U14 没焊时的临时方案, 上位机看不出来是多少 A。
     *
     *   0  机械角(°)      1  累计角(rad)    2  母线(V)      3  vq给定(V)
     *   4  故障掩码       5  ia(A)         6  ib(A)        7  ic(A)
     *   8  id实测(A)      9  iq实测(A)    10  iq给定(A)   11  id给定(A)
     *  12  实测转速(rad/s) 13 MOS温度(°C)  14 电流峰值(A)  15 编码器CRC
     */
    vofa_send_data(0,  mt6701.angle);
    vofa_send_data(1,  mt6701.total_rad);
    vofa_send_data(2,  pm.foc.vbus);
    vofa_send_data(3,  pm.ctrl.vq_set);
    vofa_send_data(4,  (float)pm.fault.all);

    vofa_send_data(5,  pm.foc.i_a);
    vofa_send_data(6,  pm.foc.i_b);
    vofa_send_data(7,  pm.foc.i_c);
    vofa_send_data(8,  pm.foc.i_d);
    vofa_send_data(9,  pm.foc.i_q);
    vofa_send_data(10, pm.ctrl.iq_set);
    vofa_send_data(11, pm.ctrl.id_set);

    vofa_send_data(12, spd_wr_meas);
    vofa_send_data(13, ntc_mos.temp);
    /* 电流幅值峰值, 一直保持。停机之后也能看出刚才最大到过多少 A */
    {
        static float i_amp_peak = 0.0f;
        float i_amp = pm.foc.i_a;
        float t;

        if (pm.foc.i_b > i_amp) { i_amp = pm.foc.i_b; }
        if (pm.foc.i_c > i_amp) { i_amp = pm.foc.i_c; }
        if (-pm.foc.i_b > i_amp) { i_amp = -pm.foc.i_b; }
        if (-pm.foc.i_c > i_amp) { i_amp = -pm.foc.i_c; }
        t = i_amp < 0.0f ? -i_amp : i_amp;
        if (t > i_amp_peak) { i_amp_peak = t; }
        vofa_send_data(14, i_amp_peak);
    }
    vofa_send_data(15, (float)mt6701.crc_ok);
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
