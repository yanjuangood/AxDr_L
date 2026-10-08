#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
固件命令协议快速验证脚本 (纯命令行, 不开 GUI)。

用法:
    python tools/cmd_test.py                  # 跑一遍预设测试
    python tools/cmd_test.py "GET?" "spd 5"   # 发指定命令

协议:
    下行 (板子->PC)  JustFloat 二进制, 12 通道, 每帧 52 字节, 结尾 00 00 80 7F
    上行 (PC->板子)  ASCII 文本, '\\n' 结尾
    文本回复用 << ... >> 包起来, 这样才能和二进制流分干净
"""

import sys
import time
import struct

PORT = r'\\.\COM12'
NCH = 12
TAIL = b'\x00\x00\x80\x7f'
FRAME = NCH * 4 + 4


class Board:
    """极简串口封装: 原始文件 I/O, 不依赖 pyserial"""

    def __init__(self, port=PORT):
        self.f = open(port, 'r+b', buffering=0)
        self.buf = bytearray()

    def close(self):
        self.f.close()

    def send(self, text):
        if not text.endswith('\n'):
            text += '\n'
        self.f.write(text.encode('ascii'))
        self.f.flush()

    def poll(self, seconds=0.4):
        """读一段时间, 返回 (文本行列表, 二进制帧数)"""
        t0 = time.time()
        while time.time() - t0 < seconds:
            chunk = self.f.read(4096)
            if chunk:
                self.buf += chunk
        return self.parse()

    def parse(self):
        """先摘出 << ... >> 之间的文本, 剩下的部分再解二进制帧"""
        lines = []
        keep = bytearray()
        buf = self.buf
        i = 0
        n = len(buf)

        while i < n:
            if buf[i:i + 2] == b'<<':
                j = buf.find(b'>>', i + 2)
                if j < 0:
                    # 回复还没收完, 留着下次
                    keep += buf[i:]
                    i = n
                    break
                text = buf[i + 2:j].decode('utf-8', 'replace')
                for ln in text.replace('\r', '\n').split('\n'):
                    ln = ln.strip()
                    if ln:
                        lines.append(ln)
                i = j + 2
            else:
                keep.append(buf[i])
                i += 1

        # 剩下的字节里数二进制帧
        nframes = 0
        i = 0
        last_end = 0
        m = len(keep)
        while i + FRAME <= m:
            if keep[i + NCH * 4:i + NCH * 4 + 4] == TAIL:
                vals = struct.unpack('<%df' % NCH, bytes(keep[i:i + NCH * 4]))
                if all(abs(v) < 1e12 for v in vals):
                    nframes += 1
                    i += FRAME
                    last_end = i
                    continue
            i += 1

        # 已经解析过的部分丢掉, 只留尾部不完整的
        self.buf = keep[last_end:] if last_end else keep
        return lines, nframes


def main():
    cmds = sys.argv[1:] if len(sys.argv) > 1 else [
        'HELP',
        'GET?',
        'spd 10',
        'spd?',
        'spd 9999',      # 应该被限幅到 200
        'spd?',
        'nosuchcmd',     # 应该报 unknown
        'mode?',
        'pn?',
        'eoff?',
        'encdir?',
        'spd 0',
    ]

    try:
        b = Board()
    except OSError as e:
        print('打开 %s 失败: %s' % (PORT, e))
        print('  - 板子插好了吗')
        print('  - 有别的程序占着这个口吗 (VOFA+ / host_gui.py)')
        return 1

    print('已连接 %s' % PORT)
    time.sleep(0.3)

    lines, nf = b.poll(0.5)
    print('清空积压: %d 帧数据 (文本回复 %d 行)' % (nf, len(lines)))

    for c in cmds:
        print('\n>>> %s' % c)
        b.send(c)
        lines, nf = b.poll(0.4)
        if lines:
            for l in lines[:45]:
                print('    ' + l)
            if len(lines) > 45:
                print('    ... 还有 %d 行' % (len(lines) - 45))
        else:
            print('    (没有文本回复)')

    b.close()
    print('\n完成')
    return 0


if __name__ == '__main__':
    sys.exit(main())
