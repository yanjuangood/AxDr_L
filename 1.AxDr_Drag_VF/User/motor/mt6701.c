/**
  ******************************************************************************
  * @file    mt6701.c
  * @brief   MT6701 磁编码器 SSI 驱动
  *
  *  时序: CSN 拉低 -> 送 24 个 CLK -> DO 上依次输出
  *        D13..D0 (14 位角度), Mg3..Mg0 (4 位磁场状态), CRC5..CRC0
  *        CSN 拉高结束一帧
  ******************************************************************************
  */
#include "mt6701.h"
#include <string.h>
#include <math.h>

mt6701_t mt6701;

/* ===================== 内部小工具 ===================== */

/* 片选: 0 = 选中(低), 1 = 释放(高) */
static inline void mt6701_csn(uint8_t level)
{
    HAL_GPIO_WritePin(MT6701_CSN_PORT, MT6701_CSN_PIN,
                      level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* 短延时, 160MHz 下约 200~300ns, 满足 CSN->CLK >= 100ns */
static inline void mt6701_gap(void)
{
    for (volatile uint32_t i = 0; i < 8u; i++)
    {
        __NOP();
    }
}

/**
  * @brief  CRC-6, 多项式 X^6 + X + 1, 覆盖 18 bit (D[13:0] + Mg[3:0])
  * @param  d18  18 位数据
  */
static uint8_t mt6701_crc6(uint32_t d18)
{
    uint8_t crc = 0u;
    for (int8_t i = 17; i >= 0; i--)
    {
        uint8_t bit = (uint8_t)((d18 >> i) & 1u);
        uint8_t msb = (uint8_t)((crc >> 5) & 1u);
        crc = (uint8_t)((crc << 1) & 0x3Fu);
        if (msb ^ bit)
        {
            crc ^= 0x03u;
        }
    }
    return crc;
}

/**
  * @brief  帧合理性检查: 排除总线卡死 (全 0 / 全 1)
  * @note   crc6(0) == 0, 所以全 0 的帧会"碰巧"通过 CRC 校验, 必须单独剔除,
  *         否则编码器没插/没供电时会误判成读取正常
  */
static uint8_t mt6701_frame_ok(uint32_t f)
{
    return ((f != 0x000000u) && (f != 0xFFFFFFu)) ? 1u : 0u;
}

/* 一次 24 bit 传输, 成功返回 1 */
static uint8_t mt6701_transfer(uint32_t *out)
{
    uint8_t rx[3];

    mt6701_csn(0);
    mt6701_gap();

    if (HAL_SPI_Receive(&hspi1, rx, 3u, 10u) != HAL_OK)
    {
        mt6701_csn(1);
        mt6701_gap();
        return 0u;
    }

    mt6701_csn(1);
    mt6701_gap();

    *out = ((uint32_t)rx[0] << 16) | ((uint32_t)rx[1] << 8) | (uint32_t)rx[2];
    return 1u;
}

/* 按模式 0~3 重新配置 SPI1 (bit1 = CPOL, bit0 = CPHA) */
static void mt6701_apply_mode(uint8_t mode)
{
    hspi1.Init.Mode              = SPI_MODE_MASTER;
    hspi1.Init.Direction         = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize          = SPI_DATASIZE_8BIT;      /* 24 bit 用 3 字节 */
    hspi1.Init.CLKPolarity       = (mode & 0x2u) ? SPI_POLARITY_HIGH : SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase          = (mode & 0x1u) ? SPI_PHASE_2EDGE   : SPI_PHASE_1EDGE;
    hspi1.Init.NSS               = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = MT6701_BAUD_PRESCALER;
    hspi1.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial     = 7u;
    hspi1.Init.CRCLength         = SPI_CRC_LENGTH_DATASIZE;
    hspi1.Init.NSSPMode          = SPI_NSS_PULSE_DISABLE;

    (void)HAL_SPI_Init(&hspi1);
}

/* ===================== 对外接口 ===================== */

/**
***********************************************************************
* @brief:      mt6701_init(void)
* @param[in]:  void
* @retval:     void
* @details:    把 SPI1 配成 8 位，并在 4 种 CPOL/CPHA 里选出 CRC 能通过的模式
***********************************************************************
**/
void mt6701_init(void)
{
    uint8_t best_score = 0u;
    uint8_t best_mode  = 0xFFu;

    memset(&mt6701, 0, sizeof(mt6701));
    mt6701.spi_mode = 0xFFu;

    /* 依次试 4 种 SPI 模式, 先试当前工程默认的 Mode3 */
    static const uint8_t order[4] = { 3u, 0u, 1u, 2u };

    for (uint8_t k = 0u; k < 4u; k++)
    {
        uint8_t mode  = order[k];
        uint8_t crc_ok_cnt = 0u;
        uint8_t mg_first   = 0xFFu;
        uint8_t mg_same    = 0u;

        mt6701_apply_mode(mode);
        HAL_Delay(1);

        for (uint8_t n = 0u; n < MT6701_PROBE_READS; n++)
        {
            uint32_t f;
            if (!mt6701_transfer(&f))
            {
                break;
            }

            uint8_t mg  = (uint8_t)((f >> 6) & 0x0Fu);
            uint8_t crc = mt6701_crc6((f >> 6) & 0x3FFFFu);

            if ((crc == (uint8_t)(f & 0x3Fu)) && mt6701_frame_ok(f))
            {
                crc_ok_cnt++;
            }
            if (mg_first == 0xFFu)
            {
                mg_first = mg;
                mg_same  = 1u;
            }
            else if (mg == mg_first)
            {
                mg_same++;
            }

            HAL_Delay(1);
        }

        /* 评分: CRC 通过数为主(权重 4), 磁场状态稳定为辅 */
        uint16_t score = (uint16_t)crc_ok_cnt * 4u + (uint16_t)mg_same;

        if (score > best_score)
        {
            best_score = score;
            best_mode  = mode;
        }

        if (crc_ok_cnt >= MT6701_PROBE_PASS)
        {
            break;      /* 已经确认, 不用再试其它模式 */
        }
    }

    if (best_mode == 0xFFu)
    {
        best_mode = MT6701_FALLBACK_MODE;
    }

    mt6701_apply_mode(best_mode);
    mt6701.spi_mode = best_mode;
    HAL_Delay(1);
}

/**
***********************************************************************
* @brief:      mt6701_read(mt6701_t *e)
* @param[in]:  e  指向编码器数据结构的指针
* @retval:     1 CRC 通过；0 传输失败或 CRC 不通过
* @details:    读一帧 24 位 SSI，校验后更新机械角、磁场状态和累计圈数
***********************************************************************
**/
uint8_t mt6701_read(mt6701_t *e)
{
    uint32_t f;
    float prev_rad;

    if (e == NULL)
    {
        return 0u;
    }

    if (!mt6701_transfer(&f))
    {
        e->crc_ok = 0u;
        e->err_cnt++;
        return 0u;
    }

    e->frame    = f;
    e->raw      = (uint16_t)((f >> 10) & 0x3FFFu);
    e->mg       = (uint8_t)((f >> 6) & 0x0Fu);
    e->crc_rx   = (uint8_t)(f & 0x3Fu);
    e->crc_calc = mt6701_crc6((f >> 6) & 0x3FFFFu);
    e->crc_ok   = ((e->crc_calc == e->crc_rx) && mt6701_frame_ok(f)) ? 1u : 0u;

    if (e->crc_ok)
    {
        e->ok_cnt++;
    }
    else
    {
        e->err_cnt++;

        /* ⚠️ CRC 不过就到此为止, 绝对不能拿这一帧的 raw 去更新角度。
         *
         * 原来的代码不管 CRC 结果都往下走, 后果是:
         *   垃圾 raw -> 垃圾 rad -> 圈数判方向的差值 d 很大 ->
         *   turns 误加/误减 1 -> total_rad 直接跳 2*pi。
         *
         * 而速度环的测速就是拿 total_rad 做差分的, 于是一次 CRC 错
         * 就在转速上打出一个大尖峰 —— 表现成"转速老是突然抖一下,
         * 但平均值其实很准"(实测均值误差 0.05%, 峰峰却有 3.9 rad/s)。
         *
         * 保留上一帧的角度继续用, 50us 的延迟对机械时间常数可忽略;
         * 连续错太多次由 sensory_pos_calc() 那边判故障。 */
        return 0u;
    }

    e->angle = (float)e->raw * 360.0f / 16384.0f;
    prev_rad = e->rad;
    e->rad   = e->angle * MT6701_PI / 180.0f;

    /* 圈数累加: 跨越 0/360 时判断方向 */
    if (e->first == 0u)
    {
        e->first = 1u;
    }
    else
    {
        float d = e->rad - prev_rad;
        if (d > MT6701_PI)
        {
            e->turns--;
        }
        else if (d < -MT6701_PI)
        {
            e->turns++;
        }
    }
    e->total_rad = (float)e->turns * MT6701_2PI + e->rad;

    return e->crc_ok;
}

/**
***********************************************************************
* @brief:      mt6701_elec_angle(float pole_pairs)
* @param[in]:  pole_pairs  极对数
* @retval:     电角度，范围 [0, 2pi)
* @details:    用累计机械角乘极对数，再归一化
***********************************************************************
**/
float mt6701_elec_angle(float pole_pairs)
{
    float a = mt6701.total_rad * pole_pairs;
    /* 归一到 0 ~ 2pi */
    a = fmodf(a, MT6701_2PI);
    if (a < 0.0f)
    {
        a += MT6701_2PI;
    }
    return a;
}

/**
***********************************************************************
* @brief:      mt6701_angle_offset(float offset_deg)
* @param[in]:  offset_deg  要减去的机械角偏移，单位度
* @retval:     去掉偏移后的机械角，范围 [0, 360)
* @details:    安装零点补偿。只改返回值，不改 mt6701.angle
***********************************************************************
**/
float mt6701_angle_offset(float offset_deg)
{
    float a = mt6701.angle - offset_deg;
    while (a < 0.0f)
    {
        a += 360.0f;
    }
    while (a >= 360.0f)
    {
        a -= 360.0f;
    }
    return a;
}
