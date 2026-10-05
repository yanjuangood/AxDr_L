/**
  ******************************************************************************
  * @file    ntc.c
  * @brief   NTC 温度采样实现
  ******************************************************************************
  */
#include "ntc.h"
#include "common.h"
#include <math.h>

/* adc1_buff 定义在 main.c */
extern uint16_t adc1_buff[];

ntc_t ntc_mos;    /* NTC1  板载, PB0  */
ntc_t ntc_coil;   /* NTC3  外接, PB12 */

/**
  * @brief  ADC 原始值 -> 摄氏度 (B 值方程)
  */
float ntc_adc_to_temp(uint16_t adc, float rs, float r0, float b)
{
    float r;
    float t;

    /* 全 0 = 短路/没接, 满量程 = 开路; 参数非法也直接返回 */
    if ((adc == 0u) || (adc >= 4095u) || (rs <= 0.0f) || (r0 <= 0.0f) || (b <= 0.0f))
    {
        return -NTC_KELVIN;
    }

    /* 分压: Vnode = VCC * Rntc / (Rs + Rntc)
     *      且 VCC 与 ADC 参考同源, 比值里抵消
     *  ->  Rntc = Rs * adc / (4096 - adc)
     */
    r = rs * (float)adc / (NTC_ADC_FULL - (float)adc);
    if (r <= 0.0f)
    {
        return -NTC_KELVIN;
    }

    /* 1/T = 1/T0 + ln(R/R0) / B */
    t = 1.0f / (1.0f / NTC_T0_K + logf(r / r0) / b);

    return t - NTC_KELVIN;
}

/**
  * @brief  单路采样 + 滤波 + 过温判断
  */
static void ntc_sample(ntc_t *n, uint16_t adc,
                       float rs, float r0, float b, float fault_th)
{
    uint8_t was_valid = n->valid;
    float   t;

    n->raw   = adc;
    n->valid = ((adc > 16u) && (adc < 4080u)) ? 1u : 0u;

    if (n->valid == 0u)
    {
        n->over = 0u;       /* 接线异常不报过温, 交给别的机制 */
        return;
    }

    n->res = rs * (float)adc / (NTC_ADC_FULL - (float)adc);

    t = ntc_adc_to_temp(adc, rs, r0, b);
    n->temp_raw = t;

    /* 低于 -40C 判为 NTC 开路/脱落。
       否则断线会被当成"很冷", 过温保护就永久失效了。 */
    if (t < NTC_TMIN_VALID)
    {
        n->valid = 0u;
        n->over  = 0u;
        return;
    }

    if (was_valid != 0u)
    {
        n->temp += (t - n->temp) * NTC_LPF_ALPHA;
    }
    else
    {
        n->temp = t;        /* 首次直接采纳, 避免从 0 慢慢爬上来 */
    }

    n->over = (n->temp >= fault_th) ? 1u : 0u;
}

void ntc_init(void)
{
    ntc_mos.valid  = 0u;
    ntc_mos.over   = 0u;
    ntc_mos.temp   = 0.0f;

    ntc_coil.valid = 0u;
    ntc_coil.over  = 0u;
    ntc_coil.temp  = 0.0f;
}

void ntc_update(void)
{
    /* NTC1 板载测 MOS: PB1 -> ADC1_IN12 -> adc1_buff[1] */
    ntc_sample(&ntc_mos, adc1_buff[1],
               pm.board.Rt_Mos_res,   /* 上拉 10K */
               pm.board.Rt_Mos,       /* R25  10K */
               pm.board.Rt_Mos_B,     /* B 值 */
               NTC_TMOS_FAULT);

    /* NTC3 外接测绕组: PB12 -> ADC1_IN11 -> adc1_buff[0] */
    ntc_sample(&ntc_coil, adc1_buff[0],
               pm.board.Rt_rotor_res,
               pm.board.Rt_rotor,
               pm.board.Rt_rotor_B,
               NTC_TCOIL_FAULT);

    /* 写回统一状态结构 */
    pm.foc.Tmos  = ntc_mos.temp;
    pm.foc.Tcoil = ntc_coil.temp;

    /* 过温 -> 置故障标志 (只置位, 由状态机 / pmsm_fault_check() 消费) */
    if (ntc_mos.over != 0u)
    {
        pm.fault.bit.ov_tmos = 1u;
    }
    if (ntc_coil.over != 0u)
    {
        pm.fault.bit.ov_tcoi = 1u;
    }
}
