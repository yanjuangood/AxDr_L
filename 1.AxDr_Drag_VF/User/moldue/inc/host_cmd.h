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

/* ---- 编码器角度误差谐波补偿 ----
 *
 * 编码器磁铁装不正时, 测出来的角度带一个和角度本身有关的误差:
 *     rad_meas = rad_true + eps(rad_true)
 * eps 主要是 1 次谐波 (偏心) + 2 次谐波 (充磁不均/倾斜) + 3 次谐波。
 * 补偿就是减掉它:  rad_fix = rad + sum A_h * sin(h*rad + phi_h)
 * 系数由上位机 tools/ecc_cal.py 标定后下发; host_ecc_en = 0 时不补偿。
 *
 * 实测这颗磁铁 (手工调到极限后) 的角度误差:
 *     1 次 1.54 度, 2 次 2.91 度, 3 次 1.33 度 (机械角)
 *     合起来约 8 度机械角 = 57 度电角度
 * 补偿掉之后 FOC 的转矩脉动会明显减小。 */
extern volatile uint32_t host_ecc_en;
extern float host_ecc_a1;   /* 1 次谐波幅度 (rad) */
extern float host_ecc_p1;   /* 1 次谐波相位 (rad) */
extern float host_ecc_a2;
extern float host_ecc_p2;
extern float host_ecc_a3;
extern float host_ecc_p3;

/* 在 CDC_Receive_FS() 里调用: 把收到的字节喂进环形缓冲 (中断上下文, 只搬运) */
void host_cmd_feed(const uint8_t *buf, uint32_t len);

/* 主循环调用: 解析并执行命令 (非阻塞) */
void host_cmd_poll(void);

/* 主循环调用: 重试发送还没发出去的回复 (CDC 忙的时候会排队) */
void host_cmd_tx_task(void);

#endif /* __HOST_CMD_H */
