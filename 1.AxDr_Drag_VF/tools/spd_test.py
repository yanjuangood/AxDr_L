#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
速度闭环测试。

做三件事:
  1. 切到速度模式 (mode 3), 给一个转速给定
  2. 采一段数据, 看实测转速能不能跟住
  3. 测完自动停机

用法:
    python tools/spd_test.py             # 默认 10 rad/s
    python tools/spd_test.py 20 6        # 20 rad/s, 采 6 秒
"""

import sys
import time
import struct

PORT = r'\\.\COM12'
NCH = 16
TAIL = b'\x00\x00\x80\x7f'
FRAME = NCH * 4 + 4

# 通道索引 (和 main.c 的 vofa_send_data 对应)
C_ANGLE, C_TOTAL, C_VBUS, C_VQ = 0, 1, 2, 3
C_FAULT = 4
C_IA, C_IB, C_IC = 5, 6, 7
C_ID, C_IQ, C_IQSET, C_IDSET = 8, 9, 10, 11
C_WR, C_TMOS, C_IPEAK, C_CRC = 12, 13, 14, 15


def norm(p):
    p = p.strip().upper()
    if p.isdigit():
        p = 'COM' + p
    return p if p.startswith('\\\\.\\') else '\\\\.\\' + p


class Link:
    def __init__(self, port=PORT):
        self.f = open(norm(port), 'r+b', buffering=0)
        self.buf = bytearray()
        self.tacc = bytearray()
        self.texts = []

    def close(self):
        self.f.close()

    def send(self, s):
        if not s.endswith('\n'):
            s += '\n'
        self.f.write(s.encode('ascii'))
        self.f.flush()

    def read(self, seconds):
        """采一段时间, 返回帧列表"""
        frames = []
        t0 = time.time()
        while time.time() - t0 < seconds:
            chunk = self.f.read(8192)
            if not chunk:
                continue
            self.buf += chunk
            self._parse(frames)
        return frames

    def _parse(self, frames):
        buf, i, n = self.buf, 0, len(self.buf)
        tbytes = bytearray()
        while i < n:
            j = buf.find(TAIL, i)
            if j < 0:
                break
            st = j - NCH * 4
            if st < i:
                break
            vals = struct.unpack('<%df' % NCH, bytes(buf[st:j]))
            if all(abs(v) < 1e12 for v in vals):
                tbytes += buf[i:st]
                frames.append(vals)
                i = j + 4
            else:
                i = st + 1
        self.buf = bytearray(buf[i:]) if i < n else bytearray()
        # 文本
        if tbytes:
            self.tacc += tbytes
            d = bytes(self.tacc)
            k = 0
            while k < len(d):
                a = d.find(b'<<', k)
                if a < 0:
                    break
                b = d.find(b'>>', a + 2)
                if b < 0:
                    break
                for ln in d[a + 2:b].decode('utf-8', 'replace').split('\n'):
                    ln = ln.strip()
                    if ln:
                        self.texts.append(ln)
                k = b + 2
            self.tacc = bytearray(d[k:])
            if len(self.tacc) > 4096:
                self.tacc = self.tacc[-512:]

def main():
    target = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
    secs = float(sys.argv[2]) if len(sys.argv) > 2 else 6.0

    L = Link()
    time.sleep(0.5)
    L.read(0.5)

    print('=== 速度闭环测试: 给定 %.1f rad/s, 采 %.0f 秒 ===' % (target, secs))
    print()

    # 先停机, 清故障, 切速度模式
    # ⚠️ foc_mode_e 的编号是 volt=0 curr=1 vel=2 pos=3, 速度模式是 2 不是 3
    for c in ('STOP', 'RST', 'mode 2', 'id 0', 'spd 0'):
        L.send(c)
        time.sleep(0.15)
    L.read(0.4)

    print('切到速度模式 (mode=2), 现在给转速给定 %.1f' % target)
    L.send('spd %.3f' % target)
    time.sleep(0.2)

    frames = L.read(secs)
    print('采到 %d 帧' % len(frames))
    if not frames:
        print('没采到数据!')
        L.send('spd 0')
        L.send('STOP')
        L.close()
        return 1

    # 分两段看: 前 1/3 是启动过程, 后 2/3 是稳态
    n = len(frames)
    head = frames[:max(1, n // 3)]
    tail = frames[-max(1, n // 3):]

    def stat(fs, idx):
        v = [f[idx] for f in fs]
        return min(v), max(v), sum(v) / len(v)

    print()
    print('--- 启动段 (前 1/3) ---')
    for nm, idx in [('转速 wr', C_WR), ('iq 实测', C_IQ), ('iq 给定', C_IQSET),
                    ('ia', C_IA), ('id', C_ID)]:
        lo, hi, av = stat(head, idx)
        print('  %-10s %8.3f ~ %8.3f   平均 %8.3f' % (nm, lo, hi, av))

    print()
    print('--- 稳态段 (后 1/3) ---')
    for nm, idx in [('转速 wr', C_WR), ('iq 实测', C_IQ), ('iq 给定', C_IQSET),
                    ('ia', C_IA), ('id', C_ID), ('母线', C_VBUS), ('温度', C_TMOS)]:
        lo, hi, av = stat(tail, idx)
        print('  %-10s %8.3f ~ %8.3f   平均 %8.3f' % (nm, lo, hi, av))

    lo, hi, av = stat(tail, C_WR)
    print()
    print('稳态转速平均 %.3f rad/s, 给定 %.3f, 误差 %.3f (%.1f%%)'
          % (av, target, av - target, 100.0 * (av - target) / target if target else 0))
    print('稳态转速波动 %.3f rad/s (峰峰值)' % (hi - lo))

    # 停机
    print()
    print('停机...')
    L.send('spd 0')
    time.sleep(0.2)
    L.send('STOP')
    L.read(0.4)
    L.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
