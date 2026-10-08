#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
读取 VOFA+ 的 JustFloat 数据流并打印各通道。

JustFloat 帧格式:
    [ float32 小端 ] x N  +  [ 00 00 80 7F ]   (帧尾 = +Inf)

用法:
    python vofa_read.py                 # 读 3 秒, 打印统计
    python vofa_read.py 5               # 读 5 秒
    python vofa_read.py 3 20            # 读 3 秒, 同时打印前 20 帧明细

⚠️ VOFA+ 想同时打开这个口是不行的, Windows 串口独占。
   要么先关 VOFA+, 要么用这个脚本代替它。
"""

import sys
import struct
import time

PORT = r'\\.\COM12'
NCH = 16
TAIL = b'\x00\x00\x80\x7f'
FRAME = NCH * 4 + 4

# 和 main.c 里 vofa_send_data() 的顺序严格对应
NAMES = [
    'angle(deg)',    # 0  机械角 0~360
    'total_rad',     # 1  累计机械角 (rad)
    'vbus(V)',       # 2  母线电压
    'vq_set(V)',     # 3  q 轴电压给定
    'fault',         # 4  故障掩码, 0 = 正常
    'ia(A)',         # 5  A 相电流
    'ib(A)',         # 6  B 相电流
    'ic(A)',         # 7  C 相电流
    'id(A)',         # 8  d 轴电流实测
    'iq(A)',         # 9  q 轴电流实测
    'iq_set(A)',     # 10 q 轴电流给定
    'id_set(A)',     # 11 d 轴电流给定
    'wr(rad/s)',     # 12 实测机械角速度
    'T_mos(C)',      # 13 板载 NTC MOS 温度
    'i_peak(A)',     # 14 电流幅值峰值 (保持)
    'crc_ok',        # 15 编码器 CRC 状态
]

# 只看这几个就够日常用
KEY = [5, 6, 7, 9, 10, 12]


def read_frames(seconds):
    """读指定秒数, 返回解出来的帧列表"""
    f = open(PORT, 'rb', buffering=0)

    buf = b''
    t0 = time.time()
    while time.time() - t0 < seconds:
        chunk = f.read(4096)
        if chunk:
            buf += chunk
    f.close()

    rows = []
    i = 0
    n = len(buf)
    while i + FRAME <= n:
        if buf[i + NCH * 4: i + NCH * 4 + 4] == TAIL:
            rows.append(struct.unpack('<%df' % NCH, buf[i:i + NCH * 4]))
            i += FRAME
        else:
            i += 1
    return rows, n


def main():
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 3.0
    detail = int(sys.argv[2]) if len(sys.argv) > 2 else 0

    try:
        rows, nbytes = read_frames(seconds)
    except OSError as e:
        print('打开 %s 失败: %s' % (PORT, e))
        print('  - 检查板子插好没有')
        print('  - 检查 VOFA+ 是不是占着这个口')
        return 1

    print('收到 %d 字节, 解出 %d 帧 (每帧 %d 字节, %d 通道)'
          % (nbytes, len(rows), FRAME, NCH))

    if not rows:
        print('一帧都没解出来。可能原因:')
        print('  - 板子没在跑 (上电了吗)')
        print('  - 通道数不是 %d (改 NCH)' % NCH)
        print('  - 串口波特率: USB CDC 虚拟串口不需要匹配波特率, 一般不是这个原因')
        return 1

    print('帧率约 %.1f Hz' % (len(rows) / seconds))

    print('\n=== 各通道统计 ===')
    print('%-14s %12s %12s %12s' % ('channel', 'min', 'max', 'last'))
    for k in range(NCH):
        v = [r[k] for r in rows]
        mark = ' *' if k in KEY else ''
        print('%-14s %12.4f %12.4f %12.4f%s'
              % (NAMES[k], min(v), max(v), v[-1], mark))

    if detail > 0:
        print('\n=== 前 %d 帧明细 (只看关键通道) ===' % detail)
        print('%-8s' % 'frame', end='')
        for k in KEY:
            print('%14s' % NAMES[k], end='')
        print()
        for idx, r in enumerate(rows[:detail]):
            print('%-8d' % idx, end='')
            for k in KEY:
                print('%14.4f' % r[k], end='')
            print()

    return 0


if __name__ == '__main__':
    sys.exit(main())
