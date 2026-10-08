#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""模拟上位机"运行控制面板"的操作序列, 检查电机是否真的转起来。

对应 GUI 里的:
    _on_mode()  -> mode 2
    _on_slide() -> spd N
    _on_run()   -> mode 2 / spd N / START
"""
import sys
import time
import tkinter as tk

sys.path.insert(0, 'tools')
import host_gui

TARGET = float(sys.argv[1]) if len(sys.argv) > 1 else 8.0

root = tk.Tk()
app = host_gui.App(root, 'COM12')

def wr_iq():
    """从最近 1 秒的帧里取转速和 iq"""
    fs = [f for f in app.frames if isinstance(f, tuple) and len(f) == 2]
    if not fs:
        return None
    win = fs[-60:]
    w = [fr[12] for _, fr in win]
    i = [fr[9] for _, fr in win]
    return (sum(w) / len(w), min(w), max(w), sum(i) / len(i))

def step_reset():
    print('  [1] 清故障 + 停机')
    app.send('spd 0')
    app.send('STOP')
    app.send('RST')

def step_run():
    print('  [2] 模拟点「运行」: mode 2 / spd %.1f / START' % TARGET)
    app.mode_var.set(2)
    app.spd_var.set(TARGET)
    app._on_mode()
    root.after(200, lambda: (app._on_run(), None))

def step_look():
    r = wr_iq()
    print('  [3] 3 秒后:')
    print('      顶部读数:', app.lbl_live.cget('text'))
    if r:
        print('      最近 60 帧: 转速 均%.2f [%.2f, %.2f]   iq 均%.4f'
              % (r[0], r[1], r[2], r[3]))
    else:
        print('      没有数据帧!')
    print('      fault =', app._live.get('fault'))

def step_stop():
    print('  [4] 停止')
    app._on_stop()
    root.after(500, app._on_close)

root.after(900,  step_reset)
root.after(1600, step_run)
root.after(5000, step_look)
root.after(6200, step_stop)
root.mainloop()
