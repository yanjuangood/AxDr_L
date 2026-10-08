#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
编码器偏心 / 安装误差测量。

原理
    匀速转动时, 真实机械角 theta_true 是线性增长的:
        theta_true = w0 * t
    但编码器读数是
        theta_meas = theta_true + eps(theta_true)
    两边求导:
        w_meas = w0 * (1 + eps'(theta))
    所以
        eps'(theta) = w_meas / w0 - 1
    对 theta 积分就得到角度误差 eps(theta), 再做谐波分解。

    偏心的典型特征是【一次谐波】占主导; 磁铁不对称还会有二次谐波。

怎么用
    1. 让电机空载, 用上位机或本脚本让它匀速转起来
    2. 跑本脚本, 记下"一次谐波"的度数
    3. 重新安装编码器磁铁 (对中 / 压平)
    4. 再跑一次对比 —— 度数明显变小就说明装好了

    python tools/ecc_test.py            # 默认 6 rad/s, 采 12 秒
    python tools/ecc_test.py 8 15

⚠️ 重装磁铁之后 e_off 必须重新标定 (上位机「标定 e_off」按钮, 或发 CAL),
   因为磁铁的角向位置变了。
"""
import sys
import time
import math
import struct

PORT = r'\\.\COM12'
NCH = 16
TAIL = b'\x00\x00\x80\x7f'
FRAME = NCH * 4 + 4

C_TOTAL, C_WR = 1, 12


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


def main():
    target = float(sys.argv[1]) if len(sys.argv) > 1 else 6.0
    secs = float(sys.argv[2]) if len(sys.argv) > 2 else 12.0
    pn = 7          # 极对数, 只用于把机械角度误差换算成电角度

    b = L()
    time.sleep(0.6)
    b.read(0.6)
    for c in ('RST', 'mode 2'):
        b.send(c)
        time.sleep(0.15)
    b.read(0.3)
    b.send('spd %.3f' % target)
    b.send('START')

    print('采集中 (%.1f rad/s, %.0f 秒)...' % (target, secs))
    fs = b.read(secs)
    b.send('spd 0')
    time.sleep(0.2)
    b.send('STOP')
    b.close()

    if len(fs) < 200:
        print('数据太少 (%d 帧)' % len(fs))
        return 1

    # 丢掉启动段
    d = fs[len(fs) // 4:]
    n = len(d)
    w0 = sum(f[C_WR] for f in d) / n
    if abs(w0) < 1.0:
        print('转速太低 (%.2f), 测不出来' % w0)
        return 1

    NB = 72                                  # 5 度一箱
    spd_sum = [0.0] * NB
    spd_cnt = [0] * NB
    for f in d:
        th = f[C_TOTAL] % (2 * math.pi)
        k = int(th / (2 * math.pi) * NB) % NB
        spd_sum[k] += f[C_WR]
        spd_cnt[k] += 1

    if min(spd_cnt) < 2:
        print('分箱样本太少, 多转几圈再测 (现在是 %.1f 圈)'
              % (abs(w0) * secs / (2 * math.pi)))
        return 1

    # eps'(theta) = w/w0 - 1
    dspd = []
    for k in range(NB):
        dspd.append(spd_sum[k] / spd_cnt[k] / w0 - 1.0)

    # 谐波分解。
    #
    # ⚠️ 不要对 dspd 积分去求 eps —— 积分会把分箱和测速的残余偏置一路累加,
    #    算出"角度误差峰峰 160 度"这种明显不合理的数字 (一次谐波才几度)。
    #
    #    直接用导数关系: 若 eps = A*sin(h*theta+phi), 则
    #        eps' = A*h*cos(h*theta+phi)
    #    => A = (eps' 的 h 次谐波幅度) / h
    dth = 2 * math.pi / NB
    amps = []
    for h in (1, 2, 3):
        re = im = 0.0
        for k in range(NB):
            re += dspd[k] * math.cos(h * k * dth)
            im += dspd[k] * math.sin(h * k * dth)
        a_deriv = 2.0 * math.hypot(re, im) / NB
        ph = math.degrees(math.atan2(im, re))
        amps.append((a_deriv / h, ph))

    # 用拟合出来的谐波重构 eps, 算峰峰 (这样不会被噪声带偏)
    eps = []
    for k in range(NB):
        v = 0.0
        for i, (a, ph) in enumerate(amps):
            h = i + 1
            v += a * math.sin(h * k * dth + math.radians(ph))
        eps.append(v)

    pp = max(eps) - min(eps)
    deg = math.degrees

    print()
    print('=== 结果 ===')
    print('  平均转速      %.3f rad/s   (%.2f 圈/秒)' % (w0, abs(w0) / (2 * math.pi)))
    print('  转了约        %.1f 圈' % (abs(w0) * secs / (2 * math.pi)))
    print('  角度误差峰峰  %.2f 度 (机械角)' % deg(pp))
    print()
    print('  谐波分解:')
    for i, (a, ph) in enumerate(amps):
        h = i + 1
        print('    %d 次谐波   %6.2f 度机械角   %7.2f 度电角度   相位 %+7.1f 度'
              % (h, deg(a), deg(a) * pn, ph))
    print()
    a1 = deg(amps[0][0])
    a2 = deg(amps[1][0])
    print('  判定:')
    if a1 > 3.0:
        print('    一次谐波 %.1f 度 —— 明显偏心。磁铁没对准传感器中心, 或与轴不垂直。' % a1)
    elif a1 > 1.0:
        print('    一次谐波 %.1f 度 —— 轻微偏心, 可以接受但不理想。' % a1)
    else:
        print('    一次谐波 %.1f 度 —— 装得很好。' % a1)
    if a2 > 2.0:
        print('    二次谐波 %.1f 度 —— 磁铁本身不对称 (充磁不均或形状不规整)。' % a2)
    print()
    print('  [注意] 重装磁铁后记得重新标定 e_off (它会跟着变)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
