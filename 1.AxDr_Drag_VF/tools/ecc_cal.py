#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
编码器角度误差谐波标定 —— 测出来, 算系数, 下发到板子。

用途
    编码器磁铁装不正时, 测出来的角度带一个和角度本身有关的误差:
        rad_meas = rad_true + eps(rad_true)
    eps 主要是 1 次 (偏心) + 2 次 (充磁不均/倾斜) + 3 次谐波。
    这个误差会让 FOC 的电角度偏掉, 转矩按 cos 打折还带脉动。

    硬件已经调到极限之后, 就用这个把剩下的误差用软件减掉。

原理
    让电机匀速转。真实角度线性增长, 而
        wr_meas = w0 * (1 + eps'(theta))
    所以 eps'(theta) = wr/w0 - 1, 对它做谐波分解:
        eps'(theta) = A_deriv * cos(h*theta - ph)
    积分回去:
        eps(theta)  = (A_deriv/h) * sin(h*theta - ph)
    要加给固件的是 -eps, 固件形式是 sum A_h*sin(h*theta + phi_h), 于是
        A_h   = A_deriv / h
        phi_h = -ph + pi

用法
    python tools/ecc_cal.py                # 标定并下发
    python tools/ecc_cal.py 6 15           # 6 rad/s, 采 15 秒
    python tools/ecc_cal.py 6 15 --dry     # 只算不下发

⚠️ 标定前后都不影响 e_off —— 角度零点的标定是独立的一件事。
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
PN = 7          # 极对数, 只用于打印电角度


def norm(p):
    p = p.strip().upper()
    if p.isdigit():
        p = 'COM' + p
    return p if p.startswith('\\\\.\\') else '\\\\.\\' + p


class L:
    def __init__(self):
        self.f = open(norm(PORT), 'r+b', buffering=0)
        self.buf = bytearray()
        self.texts = []

    def close(self):
        self.f.close()

    def send(self, s):
        if not s.endswith('\n'):
            s += '\n'
        self.f.write(s.encode('ascii'))
        self.f.flush()

    def pump(self, secs):
        out, t0 = [], time.time()
        while time.time() - t0 < secs:
            c = self.f.read(8192)
            if not c:
                continue
            self.buf += c
            self._parse(out)
        return out

    def _parse(self, out):
        b, i, n = self.buf, 0, len(self.buf)
        tacc = bytearray()
        while i < n:
            j = b.find(TAIL, i)
            if j < 0:
                break
            st = j - NCH * 4
            if st < i:
                break
            v = struct.unpack('<%df' % NCH, bytes(b[st:j]))
            if all(abs(x) < 1e12 for x in v):
                tacc += b[i:st]
                out.append(v)
                i = j + 4
            else:
                i = st + 1
        self.buf = bytearray(b[i:]) if i < n else bytearray()
        if tacc:
            d = bytes(tacc)
            k = 0
            while k < len(d):
                a = d.find(b'<<', k)
                if a < 0:
                    break
                bb = d.find(b'>>', a + 2)
                if bb < 0:
                    break
                for ln in d[a + 2:bb].decode('utf-8', 'replace').split('\n'):
                    ln = ln.strip()
                    if ln:
                        self.texts.append(ln)
                k = bb + 2


def measure(L, target, secs, NB=72, settle=3.0):
    """跑一段匀速, 返回 (谐波系数列表 [(A,phi)], 平均转速, 峰峰度数)

    ⚠️ settle 不能省: 从 START 立刻开始采的话会把加速段一起收进来,
       平均转速偏低、误差图案被污染, 标出来的系数就是错的
       (实测漏掉 settle 时, 同一工况两次测量平均转速一个 4.07 一个 5.97)。
    """
    for c in ('spd 0', 'STOP', 'RST', 'mode 2'):
        L.send(c)
        time.sleep(0.12)
    L.pump(0.4)
    L.send('spd %.3f' % target)
    L.send('START')
    L.pump(settle)              # 等转速稳下来, 这段时间的数据丢掉
    fs = L.pump(secs)
    L.send('spd 0')
    time.sleep(0.15)
    L.send('STOP')
    L.pump(0.3)

    if len(fs) < 200:
        print('    [measure] 帧数太少 (%d)' % len(fs))
        return None
    d = fs[len(fs) // 8:]                      # 再丢掉前 1/8 以防万一
    n = len(d)
    w0 = sum(f[C_WR] for f in d) / n
    if abs(w0) < 1.0:
        print('    [measure] 电机没转: 平均转速 %.3f rad/s, 故障 %d'
              % (w0, max(int(f[4]) for f in d)))
        return None
    # 转速波动太大说明没稳, 别拿这种数据标定
    wmin = min(f[C_WR] for f in d)
    wmax = max(f[C_WR] for f in d)
    if (wmax - wmin) > 0.6 * abs(w0):
        print('    [measure] 转速没稳: 均 %.3f, 范围 [%.2f, %.2f], 峰峰占 %.0f%%'
              % (w0, wmin, wmax, 100.0 * (wmax - wmin) / abs(w0)))
        return None

    ssum = [0.0] * NB
    scnt = [0] * NB
    for f in d:
        th = f[C_TOTAL] % (2 * math.pi)
        k = int(th / (2 * math.pi) * NB) % NB
        ssum[k] += f[C_WR]
        scnt[k] += 1
    if min(scnt) < 2:
        print('    [measure] 分箱样本不足 (最少 %d 个), 转速 %.2f rad/s 只转了 %.1f 圈'
              % (min(scnt), w0, abs(w0) * secs / (2 * math.pi)))
        return None

    dspd = [ssum[k] / scnt[k] / w0 - 1.0 for k in range(NB)]

    dth = 2 * math.pi / NB
    coef = []
    for h in (1, 2, 3):
        re = im = 0.0
        for k in range(NB):
            re += dspd[k] * math.cos(h * k * dth)
            im += dspd[k] * math.sin(h * k * dth)
        a_deriv = 2.0 * math.hypot(re, im) / NB
        ph = math.atan2(im, re)
        # dspd(theta) = a_deriv * cos(h*theta - ph)
        # eps  = (a_deriv/h) * sin(h*theta - ph)
        # 补偿 = -eps -> A*sin(h*theta + phi),  A = a_deriv/h,  phi = -ph + pi
        A = a_deriv / h
        phi = -ph + math.pi
        while phi > math.pi:
            phi -= 2 * math.pi
        while phi < -math.pi:
            phi += 2 * math.pi
        coef.append((A, phi))

    # 用系数重构 eps 的峰峰
    eps = []
    for k in range(NB):
        v = 0.0
        for i, (A, phi) in enumerate(coef):
            h = i + 1
            v += A * math.sin(h * k * dth + phi)
        eps.append(-v)                          # eps = -补偿
    pp = max(eps) - min(eps)
    return coef, w0, pp, len(fs)


def main():
    target = float(sys.argv[1]) if len(sys.argv) > 1 else 6.0
    secs = float(sys.argv[2]) if len(sys.argv) > 2 else 15.0
    dry = '--dry' in sys.argv

    L_ = L()
    time.sleep(0.6)
    L_.pump(0.6)

    # 先关掉补偿, 测"原始"误差
    L_.send('eccen 0')
    time.sleep(0.2)
    L_.pump(0.3)
    print('=== 第 1 步: 关掉补偿, 测原始误差 (%.1f rad/s, %.0f 秒) ===' % (target, secs))
    r1 = measure(L_, target, secs)
    if r1 is None:
        print('  测量失败')
        L_.close()
        return 1
    coef, w0, pp1, nf = r1
    print('  平均转速 %.3f rad/s, %d 帧' % (w0, nf))
    for i, (A, phi) in enumerate(coef):
        print('    %d 次谐波: %6.3f 度机械角  (%6.2f 度电角度)  相位 %+7.1f 度'
              % (i + 1, math.degrees(A), math.degrees(A) * PN, math.degrees(phi)))
    print('  角度误差峰峰 %.2f 度机械角' % math.degrees(pp1))

    if dry:
        print()
        print('  --dry 模式, 不下发')
        L_.close()
        return 0

    # 下发
    print()
    print('=== 第 2 步: 下发补偿系数 ===')
    for i, (A, phi) in enumerate(coef):
        h = i + 1
        L_.send('ecca%d %.6f' % (h, A))
        L_.send('eccp%d %.6f' % (h, phi))
    time.sleep(0.2)
    L_.send('eccen 1')
    time.sleep(0.3)
    L_.pump(0.3)
    for l in L_.texts[-8:]:
        if l.startswith('OK'):
            print('   ', l)
    L_.texts = []

    # 复测
    print()
    print('=== 第 3 步: 复测 ===')
    r2 = measure(L_, target, secs)
    if r2 is None:
        print('  复测失败')
        L_.close()
        return 1
    coef2, w02, pp2, nf2 = r2
    print('  平均转速 %.3f rad/s, %d 帧' % (w02, nf2))
    for i, (A, phi) in enumerate(coef2):
        print('    残余 %d 次谐波: %6.3f 度机械角  (%6.2f 度电角度)'
              % (i + 1, math.degrees(A), math.degrees(A) * PN))
    print('  残余角度误差峰峰 %.2f 度机械角' % math.degrees(pp2))

    print()
    print('=== 效果 ===')
    print('  峰峰 %.2f -> %.2f 度  (降低 %.0f%%)'
          % (math.degrees(pp1), math.degrees(pp2),
             100.0 * (1 - pp2 / pp1) if pp1 > 1e-9 else 0))
    L_.send('spd 0')
    time.sleep(0.15)
    L_.send('STOP')
    L_.pump(0.3)
    L_.close()
    print()
    print('  [注意] 补偿系数是运行时变量, 掉电就没了。')
    print('         要固化的话把上面第 2 步的数字写进 host_cmd.c 的初值。')
    return 0


if __name__ == '__main__':
    sys.exit(main())
