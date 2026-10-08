#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AxDr FOC 上位机 —— 参数设置 + 实时波形

特点
  * 只依赖 Python 标准库 (tkinter + 原始文件 I/O), 不用装 pyserial / matplotlib
  * 参数面板是【自动生成】的: 启动时向板子发 HELP, 按回复里的参数表建控件。
    以后固件加一个参数, 上位机不用改代码就自动出现
  * 波形用 tkinter Canvas 手绘, 不依赖 matplotlib

协议 (见 User/moldue/host_cmd.c)
  下行  板子->PC : JustFloat 二进制, 12 通道, 52 字节/帧, 结尾 00 00 80 7F
  上行  PC->板子 : ASCII 文本, '\n' 结尾, 回复用 << ... >> 包起来

用法
  python tools/host_gui.py
  python tools/host_gui.py COM12
"""

import os
import re
import sys
import time
import queue
import struct
import threading
import tkinter as tk
from tkinter import ttk, messagebox

# ======================= 协议常量 =======================
NCH = 16
TAIL = b'\x00\x00\x80\x7f'
FRAME = NCH * 4 + 4
BAUD = 115200          # USB CDC 用不到, 但留着以防有人接 USART

# 和 main.c 的 vofa_send_data() 顺序严格对应
CH_NAMES = [
    '机械角(°)', '累计角(rad)', '母线(V)', 'vq给定(V)',
    '故障掩码', 'ia(A)', 'ib(A)', 'ic(A)',
    'id实测(A)', 'iq实测(A)', 'iq给定(A)', 'id给定(A)',
    '转速(rad/s)', 'MOS温度(°C)', '电流峰值(A)', '编码器CRC',
]
# 默认画这几条
CH_DEFAULT = [5, 6, 7, 9, 12]

PLOT_COLORS = ['#e6194b', '#3cb44b', '#4363d8', '#f58231', '#911eb4',
               '#46f0f0', '#f032e6', '#bcf60c', '#fabebe', '#008080',
               '#e6beff', '#9a6324']


# ======================= 串口/协议层 =======================

try:
    import serial
    from serial.tools import list_ports
    HAVE_SERIAL = True
except ImportError:
    HAVE_SERIAL = False
    list_ports = None


def norm_port(p):
    """Windows 上 COM10 及以上必须写成 \\\\.\\COMxx, 裸写 'COM12' 会
    FileNotFoundError。这里统一补上前缀, 顺便允许只填数字。"""
    p = p.strip().upper()
    if p.isdigit():
        p = 'COM' + p
    if not p.startswith('\\\\.\\'):
        p = '\\\\.\\' + p
    return p


def scan_ports():
    """列出可用串口。pyserial 没装就返回空"""
    out = []
    if HAVE_SERIAL:
        for p in list_ports.comports():
            out.append((p.device.replace('\\\\.\\', ''), p.description))
    return out


class BoardLink:
    """板子连接。读线程解析数据流, 结果塞进两个队列。

    优先用 pyserial (超时处理正确, 读写不会互相阻塞);
    没装就退回原始文件 I/O —— 那种方式在 Windows 上 read() 会一直阻塞,
    所以读线程是必须的。"""

    def __init__(self, port):
        self.port = norm_port(port)
        if HAVE_SERIAL:
            self.f = serial.Serial(self.port, BAUD, timeout=0.02)
        else:
            self.f = open(self.port, 'r+b', buffering=0)
        self.alive = True

        self.q_text = queue.Queue()      # 文本回复行
        self.q_frame = queue.Queue()     # 解出来的数据帧 (tuple of 12 float)
        self.q_raw = queue.Queue()       # 原始字节, 给波形用

        self.buf = bytearray()
        # 文本累积器。必须跨 _parse 调用保留:
        # 固件的回复是每 64 字节发一块, 块之间会插进来 VOFA 数据帧,
        # 所以一条 << ... >> 很可能横跨好几次 _parse。只在单次调用里配对的话,
        # 配不上就被整段丢掉 —— 这正是"收到几百帧但 0 个参数"的原因。
        self.text_acc = bytearray()
        self.t = threading.Thread(target=self._reader, daemon=True)
        self.t.start()

    def send(self, text):
        if not text.endswith('\n'):
            text += '\n'
        try:
            self.f.write(text.encode('ascii'))
            self.f.flush()
            return True
        except Exception as e:
            self.q_text.put('[发送失败] %s' % e)
            return False

    def close(self):
        self.alive = False
        try:
            self.f.close()
        except Exception:
            pass

    def _reader(self):
        while self.alive:
            try:
                chunk = self.f.read(4096)
            except Exception:
                break
            if not chunk:
                time.sleep(0.005)
                continue
            self.buf += chunk
            self._parse()

    def _parse(self):
        """解析数据流。

        关键: 先按【帧尾 00 00 80 7F】摘出二进制帧, 剩下的字节必然全是 ASCII 文本。

        为什么不能反过来先找 << >>:
            float 数据里随时可能出现 0x3C 0x3C 这两个字节, 会被误当成文本开头,
            然后把后面真正的回复一起吞掉 —— 这就是之前"收到 384 帧但 0 个参数"的原因。
            而帧尾含 0x00 和 0x80, ASCII 文本里绝不会有, 所以帧尾无歧义。
        """
        buf = self.buf
        text_bytes = bytearray()
        i = 0
        n = len(buf)

        while i < n:
            j = buf.find(TAIL, i)
            if j < 0:
                break
            start = j - NCH * 4
            if start < i:
                break                       # 这一帧还没收全, 等下次
            vals = struct.unpack('<%df' % NCH, bytes(buf[start:j]))
            if all(abs(v) < 1e12 for v in vals):
                text_bytes += buf[i:start]  # 帧前面的一定是文本
                self.q_frame.put(vals)
                i = j + 4
            else:
                # 假帧尾 (48 字节不是合法 float), 当成普通字节跳过
                i = start + 1

        if i < n:
            # 剩下的可能是一个没收完的帧。保守起见只保留最后 FRAME 字节,
            # 再往前的若一直凑不成帧就不该无限攒着
            if n - i > FRAME * 2:
                text_bytes += buf[i:n - FRAME]
                i = n - FRAME
            self.buf = bytearray(buf[i:])
        else:
            self.buf = bytearray()

        # ---- 从纯文本里摘 << ... >> (跨调用累积) ----
        if text_bytes:
            self.text_acc += text_bytes
            data = bytes(self.text_acc)
            k = 0
            m = len(data)
            while k < m:
                a = data.find(b'<<', k)
                if a < 0:
                    break
                b = data.find(b'>>', a + 2)
                if b < 0:
                    break                       # 后半截还没来, 留着下次
                text = data[a + 2:b].decode('utf-8', 'replace')
                for ln in text.replace('\r', '\n').split('\n'):
                    ln = ln.strip()
                    if ln:
                        self.q_text.put(ln)
                k = b + 2
            self.text_acc = bytearray(data[k:])
            # 万一一直配不上 (比如丢包) 别让它无限涨
            if len(self.text_acc) > 4096:
                self.text_acc = self.text_acc[-512:]


# ======================= 参数控件 =======================

class ParamRow:
    """一个参数对应一行: 名字 | 输入框 | 设置按钮 | 当前值"""

    def __init__(self, parent, name, ptype, vmin, vmax, desc, on_set, row):
        self.name = name
        self.on_set = on_set

        ttk.Label(parent, text=name, width=9, anchor='w').grid(
            row=row, column=0, sticky='w', padx=(2, 4), pady=1)

        self.var = tk.StringVar()
        e = ttk.Entry(parent, textvariable=self.var, width=11)
        e.grid(row=row, column=1, sticky='we', pady=1)
        e.bind('<Return>', lambda _e: self.apply())

        ttk.Button(parent, text='设', width=3,
                   command=self.apply).grid(row=row, column=2, padx=2, pady=1)

        self.lbl = ttk.Label(parent, text='-', width=11, anchor='e',
                             foreground='#0a0')
        self.lbl.grid(row=row, column=3, sticky='e', padx=(2, 2), pady=1)

        self.desc = ttk.Label(parent, text=desc, anchor='w',
                              foreground='#666')
        self.desc.grid(row=row, column=4, sticky='w', padx=(4, 2), pady=1)

        self.vmin, self.vmax = vmin, vmax

    def apply(self):
        self.on_set(self.name, self.var.get())

    def refresh(self, value):
        self.lbl.config(text=value)
        # 输入框是空的就自动填上当前值, 免得每次都要手打
        if not self.var.get():
            self.var.set(value)


# ======================= 主窗口 =======================

class App:
    def __init__(self, root, port):
        self.root = root
        self.root.title('AxDr FOC 上位机')
        self.root.geometry('1180x720')

        self.alive = True        # 关窗时置 False, 停掉 after 链
        self.link = None
        self.params = {}         # name -> ParamRow
        self.frames = []         # 波形缓冲 [(t, tuple12), ...]
        self.t0 = time.time()
        self.plot_ch = list(CH_DEFAULT)
        self.ch_vars = {}

        self._build_ui(port)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.after(200, self._tick_text)
        self.root.after(50, self._tick_plot)

    def _on_close(self):
        self.close()
        self.root.destroy()

    # ---------------- UI ----------------
    def _build_ui(self, port):
        # 顶部: 连接栏
        top = ttk.Frame(self.root, padding=6)
        top.pack(fill='x')

        ttk.Label(top, text='串口').pack(side='left')
        self.port_var = tk.StringVar(value=port)
        ttk.Entry(top, textvariable=self.port_var, width=10).pack(side='left', padx=4)

        self.btn_conn = ttk.Button(top, text='连接', command=self.toggle_conn)
        self.btn_conn.pack(side='left', padx=4)

        self.status = ttk.Label(top, text='未连接', foreground='#c00')
        self.status.pack(side='left', padx=10)

        for txt, cmd in [('START', 'START'), ('STOP', 'STOP'),
                         ('清故障', 'RST'), ('标定 e_off', 'CAL'),
                         ('回读全部', 'GET?')]:
            ttk.Button(top, text=txt,
                       command=lambda c=cmd: self.send(c)).pack(side='left', padx=2)

        # ===== 中部左: 参数表 =====
        mid = ttk.Frame(self.root)
        mid.pack(fill='both', expand=True, padx=6, pady=4)

        left = ttk.LabelFrame(mid, text='参数 (回车=设置, 空着点"设"=回读)')
        left.pack(side='left', fill='y')

        canvas = tk.Canvas(left, width=560, highlightthickness=0)
        sb = ttk.Scrollbar(left, orient='vertical', command=canvas.yview)
        self.param_frame = ttk.Frame(canvas)
        self.param_frame.bind(
            '<Configure>',
            lambda e: canvas.configure(scrollregion=canvas.bbox('all')))
        canvas.create_window((0, 0), window=self.param_frame, anchor='nw')
        canvas.configure(yscrollcommand=sb.set)
        canvas.pack(side='left', fill='both', expand=True)
        sb.pack(side='right', fill='y')

        # ===== 中部右: 波形 =====
        right = ttk.LabelFrame(mid, text='实时波形')
        right.pack(side='left', fill='both', expand=True, padx=(8, 0))

        chbar = ttk.Frame(right)
        chbar.pack(fill='x', padx=4, pady=3)
        ttk.Label(chbar, text='显示通道:').pack(side='left')
        for i, nm in enumerate(CH_NAMES):
            v = tk.BooleanVar(value=(i in CH_DEFAULT))
            self.ch_vars[i] = v
            ttk.Checkbutton(chbar, text=str(i), variable=v,
                            command=self._ch_changed).pack(side='left')

        self.plot = tk.Canvas(right, bg='white', height=380,
                              highlightthickness=1, highlightbackground='#ccc')
        self.plot.pack(fill='both', expand=True, padx=4, pady=4)
        self.plot.bind('<Configure>', lambda e: self._draw_plot())

        self.legend = ttk.Label(right, text='', foreground='#333')
        self.legend.pack(fill='x', padx=6)

        # ===== 底部: 日志 + 手打命令 =====
        bot = ttk.LabelFrame(self.root, text='日志 / 手动命令')
        bot.pack(fill='both', padx=6, pady=(0, 6))

        self.log = tk.Text(bot, height=7, wrap='none', bg='#111', fg='#0f0',
                           insertbackground='#0f0', font=('Consolas', 9))
        self.log.pack(fill='both', expand=True, padx=4, pady=(4, 2))

        bar = ttk.Frame(bot)
        bar.pack(fill='x', padx=4, pady=(0, 4))
        ttk.Label(bar, text='命令:').pack(side='left')
        self.cmd_var = tk.StringVar()
        ent = ttk.Entry(bar, textvariable=self.cmd_var)
        ent.pack(side='left', fill='x', expand=True, padx=4)
        ent.bind('<Return>', lambda _e: self._send_entry())
        ttk.Button(bar, text='发送', command=self._send_entry).pack(side='left')

        if self.port_var.get():
            self.root.after(300, self.toggle_conn)

    # ---------------- 连接 ----------------
    def toggle_conn(self):
        if self.link is not None:
            self.link.close()
            self.link = None
            self.btn_conn.config(text='连接')
            self.status.config(text='未连接', foreground='#c00')
            return

        port = self.port_var.get().strip()
        if not port:
            messagebox.showwarning('提示', '请填串口名, 例如 COM12')
            return

        try:
            self.link = BoardLink(port)
        except Exception as e:
            messagebox.showerror('连接失败', '%s\n\n%s' % (norm_port(port), e))
            return

        self.btn_conn.config(text='断开')
        self.status.config(text='已连接 %s' % port, foreground='#080')
        self._log('已连接 %s' % port)
        # 先问参数表 (HELP) 建控件, 再回读一次当前值 (GET?)
        self.root.after(200, lambda: self.send('HELP'))
        self.root.after(900, lambda: self.send('GET?'))
        self.root.after(1500, self._poll_values)

    def _poll_values(self):
        """每隔几秒回读一次当前值, 让面板上的数字保持新鲜。
        GET? 有 350 字节左右, 不能太频繁, 否则会挤占波形带宽。"""
        if self.link is not None and self.params:
            self.send('GET?')
        self.root.after(3000, self._poll_values)

    # ---------------- 发送 ----------------
    def send(self, text):
        if self.link is None:
            self._log('[未连接] %s' % text)
            return
        self.link.send(text)
        self._log('>> %s' % text)

    def _send_entry(self):
        t = self.cmd_var.get().strip()
        if t:
            self.send(t)
            self.cmd_var.set('')

    def on_set(self, name, value):
        value = value.strip()
        if value == '':
            self.send('%s?' % name)
        else:
            self.send('%s %s' % (name, value))

    # ---------------- 日志 ----------------
    def _log(self, s):
        self.log.insert('end', s + '\n')
        self.log.see('end')

    # ---------------- 文本回复处理 ----------------
    def _tick_text(self):
        if self.link is not None:
            n = 0
            while n < 200:
                try:
                    line = self.link.q_text.get_nowait()
                except queue.Empty:
                    break
                self._handle_line(line)
                n += 1
        self.root.after(120, self._tick_text)

    def _handle_line(self, line):
        # HELP 的参数行: "#   name     f    [min, max]  desc"
        m = re.match(r'^#\s+(\w+)\s+([fi])\s+\[([^,]+),\s*([^\]]+)\]\s*(.*)$', line)
        if m:
            name, ptype, vmin, vmax, desc = m.groups()
            self._add_param(name, ptype, float(vmin), float(vmax), desc.strip())
            return

        # 参数=值
        m = re.match(r'^(?:OK\s+)?(\w+)=(.+)$', line)
        if m:
            name, val = m.groups()
            row = self.params.get(name)
            if row is not None:
                row.refresh(val.strip())
            if name not in ('vbus', 'ia', 'ib', 'ic', 'id_mea', 'iq_mea',
                            'wr', 'pe', 'fault', 'ctrl'):
                pass
            return

        self._log('<< %s' % line)

    def _add_param(self, name, ptype, vmin, vmax, desc):
        if name in self.params:
            return
        row = len(self.params)
        r = ParamRow(self.param_frame, name, ptype, vmin, vmax, desc,
                     self.on_set, row)
        self.params[name] = r
        self._log('  发现参数 %s [%g, %g] %s' % (name, vmin, vmax, desc))

    # ---------------- 波形 ----------------
    def _ch_changed(self):
        self.plot_ch = [i for i, v in self.ch_vars.items() if v.get()]
        self._draw_plot()

    def _tick_plot(self):
        if not self.alive:
            return
        if self.link is not None:
            n = 0
            while n < 200:
                try:
                    fr = self.link.q_frame.get_nowait()
                except queue.Empty:
                    break
                self.frames.append((time.time() - self.t0, fr))
                n += 1
            # 只留最近 20 秒
            if self.frames:
                cut = self.frames[-1][0] - 20.0
                while self.frames and self.frames[0][0] < cut:
                    self.frames.pop(0)
            if n:
                self._draw_plot()
        self.root.after(100, self._tick_plot)

    def close(self):
        """退出前停掉 after 链和读线程, 否则 tkinter 会报
        'invalid command name ..._tick_plot'"""
        self.alive = False
        if self.link is not None:
            self.link.close()
            self.link = None

    def _draw_plot(self):
        c = self.plot
        c.delete('all')
        W = c.winfo_width()
        H = c.winfo_height()
        if W < 20 or H < 20:
            return

        pad_l, pad_r, pad_t, pad_b = 52, 8, 8, 18
        x0, x1 = pad_l, W - pad_r
        y0, y1 = pad_t, H - pad_b

        c.create_rectangle(x0, y0, x1, y1, outline='#bbb')

        if not self.frames or not self.plot_ch:
            c.create_text((x0 + x1) / 2, (y0 + y1) / 2,
                          text='等待数据 ...', fill='#999')
            self.legend.config(text='')
            return

        tmin = self.frames[0][0]
        tmax = self.frames[-1][0]
        if tmax - tmin < 0.5:
            tmax = tmin + 0.5

        # 自动量程
        lo, hi = 1e18, -1e18
        for _, fr in self.frames:
            for i in self.plot_ch:
                v = fr[i]
                if v < lo:
                    lo = v
                if v > hi:
                    hi = v
        if hi - lo < 1e-9:
            lo -= 1.0
            hi += 1.0
        margin = (hi - lo) * 0.08
        lo -= margin
        hi += margin

        def sx(t):
            return x0 + (t - tmin) / (tmax - tmin) * (x1 - x0)

        def sy(v):
            return y1 - (v - lo) / (hi - lo) * (y1 - y0)

        # 网格 + 刻度
        for k in range(5):
            v = lo + (hi - lo) * k / 4.0
            yy = sy(v)
            c.create_line(x0, yy, x1, yy, fill='#eee')
            c.create_text(x0 - 6, yy, text='%.4g' % v, anchor='e',
                          fill='#888', font=('Consolas', 8))
        for k in range(5):
            t = tmin + (tmax - tmin) * k / 4.0
            xx = sx(t)
            c.create_line(xx, y0, xx, y1, fill='#f4f4f4')
            c.create_text(xx, y1 + 8, text='%.1f' % t, fill='#888',
                          font=('Consolas', 8), anchor='n')

        # 曲线
        for i in self.plot_ch:
            col = PLOT_COLORS[i % len(PLOT_COLORS)]
            pts = []
            for t, fr in self.frames:
                pts.append(sx(t))
                pts.append(sy(fr[i]))
            if len(pts) >= 4:
                c.create_line(*pts, fill=col, width=1.4)

        txt = '   '.join('%d:%s' % (i, CH_NAMES[i]) for i in self.plot_ch)
        self.legend.config(text='  '.join(
            '%d:%s' % (i, CH_NAMES[i]) for i in self.plot_ch))


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM12'
    root = tk.Tk()
    try:
        root.call('tk', 'scaling', 1.2)
    except Exception:
        pass
    App(root, port)
    root.mainloop()
    return 0


if __name__ == '__main__':
    sys.exit(main())
