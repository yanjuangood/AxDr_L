/**
***********************************************************************
* @file    host_cmd.h
* @brief   上位机命令解析 (走 USB CDC, 和 VOFA+ 数据流共用同一个口)
*
*  协议:
*      下行 (板子 -> PC) : JustFloat 二进制流, 12 通道, 原样不变
*      上行 (PC -> 板子) : ASCII 文本, 以 '\n' 结尾
*
*  为什么下行用二进制、上行用文本:
*      下行要 100Hz 连续发, 二进制最省带宽; 上行只是偶尔发几条命令,
*      文本可以直接用串口助手手打调试, 不用写工具就能验证。
*
*  命令表见 host_cmd.c 的 s_params[]。支持:
*      GET?            回读全部参数 + 实时量
*      SET <名> <值>   设置参数
*      <名> <值>       简写, 等价于 SET
*      <名>?           回读单个参数
*      START / STOP    启动 / 停机
*      RST             清故障
*      CAL             触发一次电角度标定
*      HELP            列出所有参数名
*
*  PC 端工具: tools/host_gui.py
***********************************************************************
**/
#ifndef __HOST_CMD_H
#define __HOST_CMD_H

#include "main.h"

/* 编码器计数方向 +1/-1 (运行时可变, 上位机可改)。见 sensory_pos_calc() */
extern float host_enc_dir;

/* 上位机发 CAL 时置 1, 主循环看到后跑一次电角度标定 */
extern volatile uint8_t host_cal_request;

/* 运动类参数 (mode/spd/iq/id/vq/vd) 被写入时置 1, 主循环看到后重新使能输出 */
extern volatile uint8_t host_start_request;

/* 在 CDC_Receive_FS() 里调用: 把收到的字节喂进环形缓冲 (中断上下文, 只搬运) */
void host_cmd_feed(const uint8_t *buf, uint32_t len);

/* 主循环调用: 解析并执行命令 (非阻塞) */
void host_cmd_poll(void);

/* 主循环调用: 重试发送还没发出去的回复 (CDC 忙的时候会排队) */
void host_cmd_tx_task(void);

#endif /* __HOST_CMD_H */
