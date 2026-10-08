#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""分析速度闭环的稳定性: 到底是控制环在追, 还是机械齿槽。

用法:  python tools/spd_analyze.py 8 6      # 给定 8 rad/s, 采 6 秒
"""
import sys
import time
import struct

PORT = r'\\.\COM12'
NCH = 16
TAIL = b'\x00\x00\x80\x7f'
FRAME = NCH * 4 + 4


def norm(p):
    p = p.strip().upper()
    if p.isdigit():
        p = 'COM' + p
    return p if p.startswith('\\\\.\\') else '\\\\.\\' + p


class L:
    def __init__(self):
        self.f = open(norm(PORT), 'r+b', buffering=0)
        self.buf = bytearray()

    def close(self):
        self.f.close()

    def send(self, s):
        if not s.endswith('\n'):
            s += '\n'
        self.f.write(s.encode('ascii'))
        self.f.flush()

    def read(self, secs):
        out, t0 = [], time.time()
        while time.time() - t0 < secs:
            c = self.f.read(8192)
            if not c:
                continue
            self.buf += c
            b, i, n = self.buf, 0, len(self.buf)
            while i < n:
                j = b.find(TAIL, i)
                if j < 0:
                    break
                st = j - NCH * 4
                if st < i:
                    break
                v = struct.unpack('<%df' % NCH, bytes(b[st:j]))
                if all(abs(x) < 1e12 for x in v):
                    out.append(v)
                    i = j + 4
                else:
                    i = st + 1
            self.buf = bytearray(b[i:]) if i < n else bytearray()
        return out


def dft_peak(x, fs):
    """找一个主频 (粗暴 DFT, 只为了看周期), 返回 (频率Hz, 幅度)"""
    import math
    n = len(x)
    if n < 16:
        return 0.0, 0.0
    mean = sum(x) / n
    x = [v - mean for v in x]
    best_f, best_a = 0.0, 0.0
    f = 0.5
    while f < fs / 2.5:
        re = im = 0.0
        w = 2 * math.pi * f / fs
        for k, v in enumerate(x):
            re += v * math.cos(w * k)
            im -= v * math.sin(w * k)
        a = math.hypot(re, im) * 2.0 / n
        if a > best_a:
            best_a, best_f = a, f
        f += 0.25
    return best_f, best_a


def main():
    target = float(sys.argv[1]) if len(sys.argv) > 1 else 8.0
    secs = float(sys.argv[2]) if len(sys.argv) > 2 else 6.0

    b = L()
    time.sleep(0.6)
    b.read(0.6)
    for c in ('RST', 'mode 2'):
        b.send(c)
        time.sleep(0.15)
    b.read(0.3)
    b.send('spd %.3f' % target)
    b.send('START')
    fs = b.read(secs)
    b.send('spd 0')
    time.sleep(0.2)
    b.send('STOP')
    b.close()

    if len(fs) < 100:
        print('数据太少 (%d 帧)' % len(fs))
        return 1

    # 丢掉前 1/3 (加速段)
    d = fs[len(fs) // 3:]
    rate = len(fs) / secs
    wr = [f[12] for f in d]
    iq = [f[9] for f in d]
    iqs = [f[10] for f in d]
    ia = [f[5] for f in d]
    n = len(d)

    def stat(name, v):
        av = sum(v) / len(v)
        pp = max(v) - min(v)
        sd = (sum((x - av) ** 2 for x in v) / len(v)) ** 0.5
        print('  %-12s 均 %9.3f   峰峰 %8.3f   标准差 %8.3f   (%.1f%%)'
              % (name, av, pp, sd, 100.0 * sd / abs(av) if av else 0))
        return av, pp, sd

    print('采样 %.0f Hz, %d 帧, 给定 %.2f rad/s' % (rate, n, target))
    print()
    print('=== 稳态统计 ===')
    stat('转速 wr', wr)
    stat('iq 实测', iq)
    stat('iq 给定', iqs)

    # 相关性: iq 是不是在跟着 wr 抖
    mw = sum(wr) / n
    mi = sum(iq) / n
    cov = sum((wr[k] - mw) * (iq[k] - mi) for k in range(n)) / n
    sw = (sum((v - mw) ** 2 for v in wr) / n) ** 0.5
    si = (sum((v - mi) ** 2 for v in iq) / n) ** 0.5
    corr = cov / (sw * si) if sw > 1e-9 and si > 1e-9 else 0.0

    print()
    print('=== 主频 ===')
    f1, a1 = dft_peak(wr, rate)
    f2, a2 = dft_peak(iq, rate)
    print('  转速主频   %6.2f Hz   幅度 %7.3f rad/s' % (f1, a1))
    print('  iq  主频   %6.2f Hz   幅度 %7.4f A' % (f2, a2))

    print()
    print('=== 判定 ===')
    print('  wr 与 iq 的相关系数 = %+.3f' % corr)
    if abs(corr) > 0.5:
        print('  => iq 明显跟着转速抖: 控制环在追, 是【环的问题】')
    else:
        print('  => iq 与转速基本无关: 是【机械齿槽】或测速噪声, 不是环在振')

    # 齿槽频率参考
    import math
    cog = 84.0 * (abs(mw) / (2 * math.pi))
    print('  (参考: 12N14P 齿槽 84 步/圈, 当前转速下齿槽频率 %.0f Hz)' % cog)
    return 0


if __name__ == '__main__':
    sys.exit(main())
