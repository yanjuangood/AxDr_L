/**
  ******************************************************************************
  * @file    mt6701.h
  * @brief   MT6701 磁编码器 SSI 驱动 (STM32G431 + SPI1)
  *
  *  板上接线 (原理图 U8 / U2, 1:1 直连):
  *      MT6701 pin1 VDD  -> VCC 3.3V
  *      MT6701 pin2 MODE -> VDD          (SSI 模式, 同手册图-22 参考电路)
  *      MT6701 pin4 GND  -> GND
  *      MT6701 pin6 A/DO -> SPI1_MISO (PB4)
  *      MT6701 pin7 B/CLK-> SPI1_SCK  (PB3)
  *      MT6701 pin8 Z/CSN-> PD2        (软件片选)
  *      MT6701 pin3 OUT / pin5 PUSH 悬空
  *
  *  SSI 帧 (24 CLK, MSB 先出):
  *      bit23..10  D[13:0]   14 位绝对角度
  *      bit9..6    Mg[3:0]   4 位磁场状态
  *      bit5..0    CRC[5:0]  6 位 CRC, 多项式 X^6 + X + 1
  *
  *  角度 = D[13:0] * 360 / 16384
  ******************************************************************************
  */
#ifndef __MT6701_H__
#define __MT6701_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "spi.h"

/* ======================= 可调参数 ======================= */
/* 片选脚: main.h 里 SPI1_CSN 就是 PD2 */
#define MT6701_CSN_PORT         SPI1_CSN_GPIO_Port
#define MT6701_CSN_PIN          SPI1_CSN_Pin

/* SPI1 在 APB2 = 160MHz。128 分频 -> 1.25MHz, 手册要求 CLK 高低各 >= 30ns */
#define MT6701_BAUD_PRESCALER   SPI_BAUDRATEPRESCALER_128

/* 自动探测时每个模式连续读几次、要求几次 CRC 正确 */
#define MT6701_PROBE_READS      8
#define MT6701_PROBE_PASS       6

/* 未探测到任何可用模式时的兜底 SPI 模式 (0~3) */
#define MT6701_FALLBACK_MODE    3

/* ======================================================== */

#define MT6701_PI               3.14159265358979f
#define MT6701_2PI              6.28318530717959f

typedef struct
{
    uint32_t frame;         /* 原始 24 bit 帧 */
    uint16_t raw;           /* D[13:0], 0 ~ 16383 */
    uint8_t  mg;            /* Mg[3:0], 磁场状态 (正常约 0x3) */
    uint8_t  crc_rx;        /* 收到的 CRC[5:0] */
    uint8_t  crc_calc;      /* 本地计算的 CRC[5:0] */
    uint8_t  crc_ok;        /* 1 = CRC 通过 */
    float    angle;         /* 机械角 0 ~ 360 度 */
    float    rad;           /* 机械角 0 ~ 2pi */
    float    total_rad;     /* 累计角度 (可正可负, 带圈数) */
    int32_t  turns;         /* 圈数 */
    uint8_t  spi_mode;      /* 自动探测到的模式 0~3, 0xFF = 没找到 */
    uint8_t  first;         /* 首次有效读数标志 (内部用) */
    uint32_t ok_cnt;        /* CRC 通过次数 */
    uint32_t err_cnt;       /* 失败次数 */
} mt6701_t;

extern mt6701_t mt6701;

/**
  * @brief  初始化: 重新配置 SPI1 为 8 位, 并自动探测 CPOL/CPHA
  * @note   会覆盖 MX_SPI1_Init 里的 DataSize / 波特率 / 极性相位
  */
void    mt6701_init(void);

/**
  * @brief  读一次角度 (24 bit SSI + CRC 校验 + 圈数累加)
  * @param  e  编码器实例
  * @retval 1 = CRC 通过, 0 = 失败
  */
uint8_t mt6701_read(mt6701_t *e);

/**
  * @brief  取电角度 (rad), 供 FOC 使用
  * @param  pole_pairs  极对数
  */
float   mt6701_elec_angle(float pole_pairs);

/**
  * @brief  取机械角, 并做零点偏移 (deg)
  */
float   mt6701_angle_offset(float offset_deg);

#ifdef __cplusplus
}
#endif

#endif /* __MT6701_H__ */
