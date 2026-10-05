/**
  ******************************************************************************
  * @file    ntc.h
  * @brief   NTC 温度采样 (MOS 温度 / 电机绕组温度)
  *
  *  硬件接线:
  *    NTC1  板载, 测 MOS 温度 : PB1  -> ADC1_IN12 -> adc1_buff[1]
  *    NTC3  外接 CN9, 测绕组  : PB12 -> ADC1_IN11 -> adc1_buff[0]
  *    (PB0 是空的, 原理图上打了叉, 不用管)
  *
  *  分压电路:  VCC --- Rs --- 节点 --- NTC --- GND   (节点 100nF 到地)
  *    NTC1: Rs = R101 = 10K, C = C52 100nF
  *    NTC3: Rs = R103 = 10K, C = C53 100nF
  *    NTC 型号: HNTC0603-103F3450FA  (10K@25C, B = 3450K, ±1%, 0603)
  *
  *  关键点: 分压上端接 VCC(3.3V), ADC 参考也是 AVCC(3.3V),
  *          两者在比值里抵消  ->  Rntc = Rs * adc / (4096 - adc)
  *          所以不需要精确知道 VCC/VREF 的绝对值。
  *
  *  温度换算 (B 值方程):
  *          1/T = 1/T0 + ln(Rntc/R0) / B
  *          T[摄氏度] = T[K] - 273.15
  ******************************************************************************
  */
#ifndef __NTC_H__
#define __NTC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ======================= 采样参数 ======================= */
#define NTC_ADC_FULL     4096.0f    /* 12 位 ADC */
#define NTC_T0_K         298.15f    /* NTC 的 B 值定义温度: 25C = 298.15K */
#define NTC_KELVIN       273.15f    /* 摄氏度 <-> 开尔文 */

/* ===================== 保护阈值 (摄氏度) ===================== */
#define NTC_TMOS_WARN    85.0f
#define NTC_TMOS_FAULT   100.0f
#define NTC_TCOIL_WARN   100.0f
#define NTC_TCOIL_FAULT  120.0f

/* ============ 一阶低通系数 (100Hz 调用, 约 1s 时间常数) ============ */
#define NTC_LPF_ALPHA    0.01f

/* 低于这个温度就判为 NTC 开路/脱落 (真实 NTC 不会这么低)。
   重要: 否则 NTC 断线时会读成"很冷", 过温保护会永久失效。 */
#define NTC_TMIN_VALID   (-40.0f)

typedef struct
{
    uint16_t raw;       /* ADC 原始值 */
    float    res;       /* 换算出的 NTC 阻值 (欧姆) */
    float    temp_raw;  /* 未滤波温度 (摄氏度) */
    float    temp;      /* 滤波后温度 (摄氏度) */
    uint8_t  valid;     /* 1 = 接线正常 (既没短路也没开路) */
    uint8_t  over;      /* 1 = 超过故障阈值 */
} ntc_t;

extern ntc_t ntc_mos;    /* NTC1  PB1  板载, ADC1_IN12 -> adc1_buff[1] */
extern ntc_t ntc_coil;   /* NTC3  PB12 外接 CN9, ADC1_IN11 -> adc1_buff[0] */

/**
  * @brief  初始化 (只是清状态, ADC 由 MX_ADC1_Init 配置)
  */
void  ntc_init(void);

/**
  * @brief  读 ADC -> 换算温度 -> 滤波 -> 过温判断 -> 写回 pm
  * @note   建议在主循环里 100Hz 调用
  */
void  ntc_update(void);

/**
  * @brief  ADC 原始值 -> 摄氏度
  * @param  adc  12 位 ADC 值
  * @param  rs   分压上拉电阻 (欧姆)
  * @param  r0   NTC 在 25C 的标称阻值 (欧姆)
  * @param  b    NTC 的 B 值 (K)
  * @retval 摄氏度; 接线异常时返回 -273.15
  */
float ntc_adc_to_temp(uint16_t adc, float rs, float r0, float b);

#ifdef __cplusplus
}
#endif

#endif /* __NTC_H__ */
