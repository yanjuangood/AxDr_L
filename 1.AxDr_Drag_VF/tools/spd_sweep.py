#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
速度环增益扫描: 试几组 spdkp/spdki, 看哪组响应最好。

用法:
    python tools/spd_sweep.py            # 默认扫几组
    python tools/spd_sweep.py 10         # 给定 10 rad/s
"""

import sys
import time
import struct

PORT = r'\\.\COM12'
NCH = 16
TAIL = b'\x00\x00\x80\x7f'
FRAME = NCH * 4 + 4

C_WR, C_IQ, C_IQSET = 12, 9, 10
C_VBUS, C_FAULT = 2, 4


def norm(p):
    p = p.strip().upper()
    if p.isdigit():
        p = 'COM' + p
    return p if p.startswith('\\\\.\\') else '\\\\.\\' + p


class Link:
    def __init__(self):
        self.f = open(norm(PORT), 'r+b', buffering=0)
        self.buf = bytearray()
        self.tacc = bytearray()

    def close(self):
        self.f.close()

    def send(self, s):
        if not s.endswith('\n'):
            s += '\n'
        self.f.write(s.encode('ascii'))
        self.f.flush()

    def read(self, seconds):
        out, t0 = [], time.time()
        while time.time() - t0 < seconds:
            c = self.f.read(8192)
            if not c:
                continue
            self.buf += c
            self._p(out)
        return out

    def _p(self, out):
        buf, i, n = self.buf, 0, len(self.buf)
        while i < n:
            j = buf.find(TAIL, i)
            if j < 0:
                break
            st = j - NCH * 4
            if st < i:
                break
            v = struct.unpack('<%df' % NCH, bytes(buf[st:j]))
            if all(abs(x) < 1e12 for x in v):
                out.append(v)
                i = j + 4
            else:
                i = st + 1
        self.buf = bytearray(buf[i:]) if i < n else bytearray()


def trial(L, kp, ki, target, secs=5.0):
    for c in ('STOP', 'spd 0'):
        L.send(c)
        time.sleep(0.1)
    L.read(0.3)

    L.send('spdkp %.6f' % kp)
    L.send('spdki %.6f' % ki)
    time.sleep(0.15)
    L.read(0.2)

    L.send('mode 2')
    time.sleep(0.1)
    L.send('spd %.3f' % target)

    fs = L.read(secs)
    L.send('spd 0')
    time.sleep(0.15)
    L.send('STOP')
    L.read(0.3)

    if len(fs) < 30:
        return None
    tail = fs[len(fs) // 2:]
    wr = [f[C_WR] for f in tail]
    iq = [f[C_IQSET] for f in tail]
    iqm = [f[C_IQ] for f in fs]
    flt = [f[C_FAULT] for f in fs]
    av = sum(wr) / len(wr)
    pk = max(wr) - min(wr)
    # 上升时间: 第一次达到给定 90% 的时刻
    rise = None
    for k, f in enumerate(fs):
        if f[C_WR] >= 0.9 * target:
            rise = k / (len(fs) / secs)
            break
    return dict(avg=av, err=av - target, ripple=pk,
                iq=sum(iq) / len(iq), iqmax=max(iq), rise=rise,
                iq_mea_max=max(iqm), fault=max(flt))


def main():
    target = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
    cases = [
        (0.0,      0.0,      '模型整定 (kp=wc*Js=0.0063)'),
        (0.010,    0.0005,   'kp=0.010 ki=0.0005  (1.6x)'),
        (0.020,    0.001,    'kp=0.020 ki=0.001   (3.2x)'),
        (0.040,    0.002,    'kp=0.040 ki=0.002   (6.3x)'),
    ]

    L = Link()
    time.sleep(0.5)
    L.read(0.6)

    print('速度环增益扫描, 给定 %.1f rad/s' % target)
    print()
    print('%-26s %9s %9s %9s %9s %8s %7s %7s' %
          ('kp / ki', '稳态', '误差', '峰峰', 'iq均值', '上升', 'iq峰值', '故障'))
    print('-' * 100)

    results = []
    for kp, ki, name in cases:
        r = trial(L, kp, ki, target)
        if r is None:
            print('%-26s   数据不足' % name)
            continue
        rs = '%.2fs' % r['rise'] if r['rise'] is not None else '  n/a'
        print('%-26s %9.3f %9.3f %9.3f %9.4f %8s %7.3f %7d'
              % (name, r['avg'], r['err'], r['ripple'], r['iq'], rs,
                 r['iq_mea_max'], int(r['fault'])))
        results.append((name, r))

    # 恢复默认
    L.send('spdkp 0')
    L.send('spdki 0')
    L.send('spd 0')
    L.send('STOP')
    L.read(0.4)
    L.close()

    if results:
        best = min(results, key=lambda x: abs(x[1]['ripple']))
        print()
        print('波动最小的是: %s  (峰峰 %.3f rad/s)' % (best[0], best[1]['ripple']))
    return 0


if __name__ == '__main__':
    sys.exit(main())
