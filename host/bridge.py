#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""smartcar-bridge —— 上位机页面 ↔ 小车（HC05 蓝牙）之间的那一层。

    浏览器(smartcar-host.html) --HTTP--> bridge.py --串口--> HC05 --> 车

为什么要多这么一层（而不是让页面直接用 Web Serial）：

  * 页面用 file:// 双击打开时 navigator.serial 是 undefined（不是安全上下文），
    串口根本拿不到。要让页面摸得到串口，就得把它放在 http://localhost 下 ——
    既然都要起个本机服务了，串口也一起拿着最省事：只有一个进程占串口，
    不会出现"页面和串口助手抢 COM 口"。
  * 路径要从页面上的"米 + 浮点度"翻成车上那套整数协议（毫米 + 0.1 度）——
    车上的 printf 是 tiny vsnprintf，不支持 %f（见 speed_report.h）。
  * 下载要逐行等 ack、要控制节奏（车是一条一条同步处理的），
    而且跑动期间必须一直发心跳：失联急停的判据是"多久没收到字节"，
    而路径下载完之后页面本来就没话说了，不发心跳车会把自己急停掉。

用法：

    python bridge.py --list                    列出可用串口
    python bridge.py --port COM14              正常使用（默认 38400）
    python bridge.py --mock                    不接硬件，模拟一块车
    python bridge.py --port COM14 --http 8000 --link-ms 1000

然后浏览器打开 http://localhost:8000/ 。（也可以双击 HTML 用 file:// 打开，
桥的接口带了 CORS 头，页面照样能通过它控车，只是页面上那个"Web Serial 直连"
面板用不了。）

只依赖标准库。串口优先用 pyserial（装了就用），没装就退回用 ctypes 直接打
Win32 的串口 API —— 这台机器上没装 pyserial，而为了装它去动环境不值得。
"""

import argparse
import json
import os
import re
import socket
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

# ---------------------------------------------------------------- 协议常量 ----
# 和固件侧一一对应，改那边记得改这里：
MAX_POINTS = 128        # follow.h  FOLLOW_MAX_POINTS
COORD_LIMIT_MM = 20000  # follow.h  FOLLOW_COORD_MAX（航段长度要在 32 位里开方）
HDEG_LIMIT = 1800       # 0.1 度

ACK_TIMEOUT = 1.5       # 等一行 ok/err 的秒数（38400 下一行约 10ms，留足余量）
SETUP_TIMEOUT = 3.0     # 刚开串口时的头两条命令要等更久：模块唤醒/首包
                        # 排队常常 1~2s，用 ACK_TIMEOUT 会误报「没有回响应」
HEARTBEAT_S = 0.5       # 心跳周期，要明显小于车上的 bt link 超时（默认 1000ms）
RATE_MS = 500           # 车的 spd 遥测周期。默认 100ms 会把无线链路占满，
                        # 心跳排队 1~2s 才到，车上的失联急停就误刹（ab=2）。

# 串口断了以后自动重连的间隔（秒），连续失败就按这个阶梯退避。
RECONNECT_BACKOFF = (1.0, 2.0, 3.0, 5.0)

PAGE_NAME = 'smartcar-host.html'

FOL_RE = re.compile(
    r'^fol st=(?P<st>\w+) leg=(?P<leg>\d+)/(?P<seg>\d+) d=(?P<d>-?\d+) '
    r'yaw=(?P<yaw>-?\d+) err=(?P<err>-?\d+) ab=(?P<ab>\d+)$')
LOAD_RE = re.compile(r'^fol load n=(?P<n>\d+) seg=(?P<seg>\d+) mm=(?P<mm>\d+)$')
ODO_RE = re.compile(r'^fol odo l=(?P<l>-?\d+) r=(?P<r>-?\d+) mm=(?P<mm>-?\d+)$')


class BridgeError(Exception):
    """用户看得懂的错误，直接回给页面显示。"""


# ============================================================== 行日志 =======
class LineLog:
    """串口上收到/发出的每一行。页面用 since 增量取，所以给每行一个递增号。

    同时兼当"有回包了"的通知点：cmd() 就是在这个条件变量上等 ack 的。"""

    def __init__(self, cap=4000):
        self.cap = cap
        self.lines = []          # [{"i":n, "t":ts, "s":str, "dir":"in"/"out"}]
        self.next = 0
        self.cv = threading.Condition()

    def add(self, s, direction='in'):
        with self.cv:
            self.lines.append({'i': self.next, 't': time.time(),
                               's': s, 'dir': direction})
            self.next += 1
            if len(self.lines) > self.cap:
                del self.lines[:len(self.lines) - self.cap]
            self.cv.notify_all()

    def mark(self):
        with self.cv:
            return self.next

    def notify(self):
        """叫醒所有在 wait_ack 里等的线程（链路断了，别等到超时）。"""
        with self.cv:
            self.cv.notify_all()

    def since(self, start):
        with self.cv:
            return [l for l in self.lines if l['i'] >= start], self.next

    def wait_ack(self, start, deadline, abort=None):
        """等 start 之后的第一行 ok/err。

        abort：链路已经换过（或要退出）时返回 True，立刻放弃等待。不加它的话，
        重连时补发的 `ok rate` 会被当成上一条用户命令的回包 —— 假报成功。"""
        with self.cv:
            while True:
                if abort is not None and abort():
                    return None
                for l in self.lines:
                    if l['i'] >= start and l['dir'] == 'in' and \
                            (l['s'].startswith('ok ') or l['s'].startswith('err ')):
                        return l['s']
                remain = deadline - time.time()
                if remain <= 0:
                    return None
                self.cv.wait(remain)


# ============================================================ 串口后端 =======
class PySerial:
    """装了 pyserial 就走这条（最省心的那条）。"""

    def __init__(self, port, baud):
        import serial
        self.ser = serial.Serial(port, baud, timeout=0.1, write_timeout=2.0)
        self.ser.reset_input_buffer()

    def read(self, n):
        data = self.ser.read(n)
        return data or b''

    def write(self, data):
        n = self.ser.write(data)
        if n != len(data):
            raise BridgeError('写串口只发出 %s/%d 字节（链路可能断了）'
                              % (n, len(data)))

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass


class Win32Serial:
    """没装 pyserial 时直接用 Win32 的串口 API。

    【为什么用 create_string_buffer(28) 当 DCB】DCB 是个满是位域的结构体，
    在 ctypes 里把它逐位写对既啰嗦又容易错。BuildCommDCBW() 能从
    "baud=38400 parity=N data=8 stop=1" 这种字符串填出 DCB，而 DCB 的大小
    在所有 Windows 上都是固定的 28 字节 —— 给它 28 字节的缓冲就够了，
    不用声明结构体。
    """

    PURGE_RXCLEAR, PURGE_TXCLEAR = 0x0008, 0x0004
    GENERIC_READ, GENERIC_WRITE = 0x80000000, 0x40000000
    OPEN_EXISTING = 3

    def __init__(self, port, baud):
        if os.name != 'nt':
            raise BridgeError('没有 pyserial，而非 Windows 下没有备用的串口实现；'
                              '请先 pip install pyserial')
        import ctypes
        from ctypes import wintypes as wt

        self.ctypes = ctypes
        self.wt = wt
        self.k32 = ctypes.WinDLL('kernel32', use_last_error=True)
        k = self.k32

        k.CreateFileW.argtypes = [wt.LPCWSTR, wt.DWORD, wt.DWORD, ctypes.c_void_p,
                                  wt.DWORD, wt.DWORD, ctypes.c_void_p]
        k.CreateFileW.restype = ctypes.c_void_p
        k.BuildCommDCBW.argtypes = [wt.LPCWSTR, ctypes.c_void_p]
        k.BuildCommDCBW.restype = wt.BOOL
        k.SetCommState.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
        k.SetCommState.restype = wt.BOOL
        k.SetCommTimeouts.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
        k.SetCommTimeouts.restype = wt.BOOL
        k.ReadFile.argtypes = [ctypes.c_void_p, ctypes.c_void_p, wt.DWORD,
                               ctypes.POINTER(wt.DWORD), ctypes.c_void_p]
        k.ReadFile.restype = wt.BOOL
        k.WriteFile.argtypes = [ctypes.c_void_p, ctypes.c_void_p, wt.DWORD,
                                ctypes.POINTER(wt.DWORD), ctypes.c_void_p]
        k.WriteFile.restype = wt.BOOL
        k.CloseHandle.argtypes = [ctypes.c_void_p]

        path = '\\\\.\\' + port            # COM10 以上必须走 \\.\ 前缀
        h = k.CreateFileW(path, self.GENERIC_READ | self.GENERIC_WRITE,
                          0, None, self.OPEN_EXISTING, 0, None)
        if h is None or h == ctypes.c_void_p(-1).value:
            err = ctypes.get_last_error()
            raise BridgeError('打不开 %s（GetLastError=%d）：%s'
                              % (port, err, self._why(err)))
        self.h = h

        dcb = ctypes.create_string_buffer(28)
        spec = 'baud=%d parity=N data=8 stop=1' % baud
        if not k.BuildCommDCBW(spec, dcb) or not k.SetCommState(self.h, dcb):
            err = ctypes.get_last_error()
            self.close()
            raise BridgeError('%s 不支持 %s（GetLastError=%d）' % (port, spec, err))

        # ReadIntervalTimeout = MAXDWORD（其余为 0）是 Win32 上"有多少读多少、
        # 一个字节都没有就立刻返回"的标准写法 —— 我们要的就是非阻塞语义。
        to = ctypes.create_string_buffer(20)
        ctypes.memmove(to, ctypes.c_uint32(0xFFFFFFFF).value.to_bytes(4, 'little') +
                       b'\x00' * 16, 20)
        if not k.SetCommTimeouts(self.h, to):
            self.close()
            raise BridgeError('%s 设置超时失败' % port)

        k.PurgeComm(self.h, self.PURGE_RXCLEAR | self.PURGE_TXCLEAR)

    @staticmethod
    def _why(err):
        return {2: '没有这个串口（蓝牙没配对 / 小车没开机？）',
                3: '路径不对；COM10 以上要写 COM10 这样的完整名字',
                5: '被别的程序占着（串口助手、上一份 bridge 还在跑？）',
                6: '句柄无效（可能已经关过了）',
                22: '句柄失效：USB 模块重新枚举/拔插过，旧句柄作废',
                1167: '设备不在线：USB 模块被拔掉 / 还没枚举完',
                995: '等待被取消（关串口时打断了正在进行的读写）',
                }.get(err, '见 GetLastError')

    def read(self, n):
        h = self.h
        if not h:
            raise BridgeError('串口已经关了')
        buf = self.ctypes.create_string_buffer(n)
        got = self.wt.DWORD(0)
        if not self.k32.ReadFile(h, buf, n, self.ctypes.byref(got), None):
            err = self.ctypes.get_last_error()
            raise BridgeError('读串口失败 (GetLastError=%d)：%s'
                              % (err, self._why(err)))
        return buf.raw[:got.value]

    def write(self, data):
        h = self.h
        if not h:
            raise BridgeError('串口已经关了')
        data = bytes(data)
        # 走 create_string_buffer 而不是直接把 bytes 递进去：argtypes 是 c_void_p，
        # 传 bytes 其实也能过，但给个明确缓冲更稳，也不怕数据里混进 NUL。
        buf = self.ctypes.create_string_buffer(data)
        put = self.wt.DWORD(0)
        if not self.k32.WriteFile(h, buf, len(data),
                                  self.ctypes.byref(put), None):
            err = self.ctypes.get_last_error()
            raise BridgeError('写串口失败 (GetLastError=%d)：%s'
                              % (err, self._why(err)))
        if put.value != len(data):
            raise BridgeError('写串口只发出 %d/%d 字节（链路可能断了）'
                              % (put.value, len(data)))

    def close(self):
        # 先清 self.h 再关：关闭之后读线程/写线程再进来会看到 None 并干净地报错，
        # 而不是拿着一个已经归还给系统的句柄去调 ReadFile/WriteFile。
        h, self.h = getattr(self, 'h', None), None
        if h:
            try:
                self.k32.CloseHandle(h)
            except Exception:
                pass


class MockSerial:
    """--mock：不接硬件，在本地把固件的协议演一遍。

    它不是"假装成功"，而是照着 follow.c 的行为应答：收 begin/wp/end/start，
    回 ok/err，然后按航段推进持续吐 fol st=... 遥测。这样页面上的"进度/状态"
    那一整块（还有桥这一层）在没板子的时候也能真跑起来。
    """

    def __init__(self, log):
        self.log = log
        # 两个方向【必须分开】：read() 是"车 → 宿主"，write() 是"宿主 → 车"。
        # 塞进同一个缓冲的话，桥发出去的每条命令都会被它自己读回来当成车的
        # 应答 —— 变成自己跟自己对话（第一版就是这么写的，错的）。
        self.to_host = b''
        self.to_car = b''
        self.cv = threading.Condition()
        self.alive = True
        self.pts = []
        self.seg = 0
        self.leg = 0
        self.d = 0
        self.state = 'idle'
        self.ab = 0
        self.odo = 0
        self.t0 = 0.0
        self.leg_s = 3.0
        self.last_fol = time.time()
        threading.Thread(target=self._loop, daemon=True).start()

    # ---- 给 Bridge 用的串口接口 ----
    def read(self, n):
        with self.cv:
            if not self.to_host:
                self.cv.wait(0.05)      # 和真串口一样：没数据就等一会儿
            out, self.to_host = self.to_host[:n], self.to_host[n:]
            return out

    def write(self, data):
        with self.cv:
            self.to_car += data
            self.cv.notify_all()

    def close(self):
        self.alive = False
        with self.cv:
            self.cv.notify_all()

    # ---- 车的那一半 ----
    def _emit(self, s):
        with self.cv:
            self.to_host += (s + '\r\n').encode('ascii')
            self.cv.notify_all()

    def _loop(self):
        buf = b''
        while self.alive:
            with self.cv:
                if not self.to_car:
                    self.cv.wait(0.02)
                data, self.to_car = self.to_car, b''
            buf += data
            while b'\n' in buf:
                line, buf = buf.split(b'\n', 1)
                self._handle(line.decode('ascii', 'replace').strip('\r'))

            # 周期遥测，周期和真车一样：跑动时 200ms、闲着 2s
            # （follow.c 的 follow_tele_tick）。**这里不能图省事放宽**：
            # 原地转那一小段只有几百毫秒，吐得太稀就整个看不见了，
            # 页面上的"原地转向/稳定中"也就永远验不到。
            period = 0.2 if self.state == 'run' else 2.0
            if time.time() - self.last_fol >= period:
                self.last_fol = time.time()
                self._fol_line()

    def _handle(self, line):
        if not line.startswith('#'):
            return                      # 没有 '#' 的字节是遥控命令，模拟器不处理
        toks = line[1:].split()
        if not toks:
            return
        name = toks[0]
        if name == 'follow' and len(toks) > 1:
            sub = toks[1]
            if sub == 'begin' and len(toks) > 2:
                self.pts = []
                self.state = 'load'
                self._ok('follow')
            elif sub == 'wp' and len(toks) >= 5:
                self.pts.append((int(toks[2]), int(toks[3]), int(toks[4])))
                self._ok('follow')
            elif sub == 'end':
                self.seg = max(0, len(self.pts) - 1)
                self.leg = 0
                self.state = 'load'
                self._ok('follow')
                self._emit('fol load n=%d seg=%d mm=%d'
                           % (len(self.pts), self.seg,
                              sum(self._leg_mm(i)
                                  for i in range(1, len(self.pts)))))
            elif sub == 'start':
                self.state = 'run'
                self.t0 = time.time()
                self._ok('follow')
            elif sub == 'stop':
                self.state = 'idle'
                self._ok('follow')
                self._fol_line()
            elif sub == 'ping':
                pass                    # 心跳：不应答，和固件一致
            elif sub == 'status':
                self._ok('follow')
                self._fol_line()
            elif sub == 'odom':
                self._ok('follow')
                self._emit('fol odo l=%d r=%d mm=%d'
                           % (self.odo, self.odo, self.odo // 2))
            elif sub in ('spd', 'margin', 'tol', 'kp', 'lead', 'vmin', 'vmax', 'sign'):
                self._ok('follow')      # 整定旋钮：收下就行
            else:
                self._err('follow')
        elif name == 'bt':
            self._ok('bt')
        elif name in ('head', 'motor'):
            self._ok(name)
        elif name == 'on' or name == 'off':
            self._ok(name)
        elif name == 'rate':
            self._ok('rate')
        else:
            self._err(name)

    def _ok(self, n):
        self._emit('ok ' + n)

    def _err(self, n):
        self._emit('err ' + n)

    def _leg_mm(self, leg):
        """第 leg 段的长度（mm）。leg 从 1 数起，和遥测里的 leg= 对得上。"""
        if leg < 1 or leg >= len(self.pts):
            return 0
        (x0, y0, _), (x1, y1, _) = self.pts[leg - 1], self.pts[leg]
        return int(((x1 - x0) ** 2 + (y1 - y0) ** 2) ** 0.5)

    def _fol_line(self):
        # st 用真车那套名字（follow.c 的 follow_state_name）：idle/load/turn/
        # settle/drive/done/abort —— 页面的状态映射就是照着这几个词写的，
        # mock 要是自己编一套，验出来的就是假的。
        st = self.state
        if self.state == 'run':
            cycle = (time.time() - self.t0) / self.leg_s
            if cycle >= self.seg:               # 段走完了
                st = self.state = 'done'
                self.leg = self.seg
                self.d = 0
            else:
                self.leg = int(cycle) + 1
                frac = cycle - int(cycle)
                # 照着真车的节奏演：每段先原地转(25%)、稳一下(10%)、再走(65%)
                st = 'turn' if frac < 0.25 else ('settle' if frac < 0.35
                                                 else 'drive')
                self.d = int(max(0.0, (frac - 0.35) / 0.65)
                             * self._leg_mm(self.leg))
                self.odo = self.d + sum(self._leg_mm(i)
                                        for i in range(1, self.leg))
        yaw = self.pts[self.leg][2] if 0 <= self.leg < len(self.pts) else 0
        self._emit('fol st=%s leg=%d/%d d=%d yaw=%d err=%d ab=%d'
                   % (st, self.leg, self.seg, self.d, yaw, 12, self.ab))


# ================================================================ 桥 ========
class Bridge:
    def __init__(self, args):
        self.args = args
        self.mock = args.mock
        self.baud = args.baud
        self.port_name = args.port
        self.link_ms = args.link_ms
        self.ser = None
        self.name = None
        self.wlock = threading.RLock()
        self.log = LineLog()
        self.latest = {'fol': None, 'load': None, 'odo': None, 't': None}
        self.running = True
        self.feeder = None

        # ---- 串口健康状态（掉线自动重连用）----
        self.rlock = threading.Lock()          # 只护 ReadFile 和真正的 close
        self._connect_lock = threading.Lock()  # 串行化 connect / disconnect / 自动重连
        self._wake = threading.Event()
        self._want_open = False                # 用户是否要这个口一直开着
        self._target_port = args.port
        self._fail_count = 0
        self._reconnect_count = 0
        self._next_try = 0.0                   # time.monotonic()
        self._last_error = None
        self._last_logged_error = None
        self._dead = []                        # 等 supervisor 安全关闭的旧句柄
        self._link_gen = 0                     # 断链代数，给 cmd 丢弃旧链路的 ack

        # 【读线程必须先起来】connect() 里要发 rate / bt link 并等回包，回包是
        # 读线程收的；线程晚起会让这两条命令必然超时 —— 明明已经回 ok 了。
        threading.Thread(target=self._reader, daemon=True).start()
        threading.Thread(target=self._heartbeat, daemon=True).start()

        if self.mock:
            self._open_mock()
        elif args.port:
            try:
                self.connect(args.port)
            except BridgeError as e:
                # 【不能在这里退出】串口被别的程序占着（GetLastError=5）是常态。
                # 进程一死，页面就是 ERR_CONNECTION_REFUSED，用户连提示都看不到。
                # 记一条，supervisor 继续在后台试 —— 对方松手就自己连上。
                sys.stderr.write('串口 %s 暂时打不开：%s\n'
                                 '  bridge 继续运行，后台自动重试。\n'
                                 % (args.port, e))

        threading.Thread(target=self._supervisor, daemon=True).start()

    # ---------------------------------------------------------- 连接 ----
    def connect(self, port):
        """手动连接（/api/connect 和启动时的 --port 都走这里）。

        失败照旧抛给调用方 —— 页面要看到「为什么连不上」；但 _want_open 保持为真，
        supervisor 会继续在后台试，不需要用户再点一次。"""
        with self._connect_lock:
            with self.wlock:
                self._target_port = port
                self._want_open = True
            try:
                r = self._connect_once(port)
            except BridgeError as e:
                self._note_connect_fail(e)
                raise
            except Exception as e:
                err = BridgeError('%s: %s' % (type(e).__name__, e))
                self._note_connect_fail(err)
                raise err
            with self.wlock:
                self._fail_count = 0
                self._last_error = None
                self._last_logged_error = None
                self._next_try = 0.0
            self._wake.set()
            return r

    def _connect_once(self, port):
        # 【这里不能用 disconnect()】那会把 _want_open 清掉 —— 自动重连连上一次
        # 之后就不再重连了。另外旧句柄必须先真正关掉：CreateFileW 用 dwShareMode=0，
        # 同一个串口还开着的时候再开一次会报 GetLastError=5。
        self._close_serial()
        if self.mock:
            self._open_mock()
            return {'port': port, 'mock': True}

        errors = []
        for backend in (PySerial, Win32Serial):
            try:
                ser = backend(port, self.baud)
            except BridgeError as e:
                errors.append('%s: %s' % (backend.__name__, e))
                continue
            except Exception as e:                     # pyserial 不在就是 ImportError
                errors.append('%s 不可用: %s' % (backend.__name__, e))
                continue
            with self.wlock:
                self.ser = ser
                self.name = port
            self.log.add('# 已连接 %s @ %d（%s）'
                         % (port, self.baud, backend.__name__), 'out')
            if not self._apply_setup(ser):
                raise BridgeError('连上 %s 后链路又断了' % port)
            return {'port': port, 'mock': False}

        raise BridgeError('连不上 %s。%s' % (port, ' / '.join(errors)))

    def _apply_setup(self, ser):
        """每次（重）连成功都补发这两条。

        返回 False 表示这条链路在设置过程中就断了（_mark_dead 已经把 self.ser
        摘掉）—— 调用方要按「连接失败」处理，不能报成功。"""
        # 【先降遥测再开急停】车的 spd 推送默认 100ms 一次，这条无线链路
        # 上行跟不上：心跳（500ms 一条）会排队，板子侧超过 bt link 时限收
        # 不到字节就刹停（遥测里 ab=2）。降到 RATE_MS 后双向都够用。
        try:
            self.cmd('rate %d' % RATE_MS, timeout=SETUP_TIMEOUT)
            self.log.add('# 遥测周期已设为 %d ms（给心跳让带宽）'
                         % RATE_MS, 'out')
        except BridgeError as e:
            self.log.add('# 设置遥测周期失败：%s' % e, 'out')
            if self.ser is not ser:
                return False
        if self.link_ms:
            # 失联急停默认是关的，而且车跑起来之后是电池供电、插不了 USB ——
            # 只能从链路上开。见 bt_car.c 里 Bt_Cmd() 的注释。
            try:
                self.cmd('bt link %d' % self.link_ms, timeout=SETUP_TIMEOUT)
                self.log.add('# 已开启失联急停（%d ms 没有字节就刹车）'
                             % self.link_ms, 'out')
            except BridgeError as e:
                self.log.add('# 开失联急停失败：%s' % e, 'out')
                if self.ser is not ser:
                    return False
        return True

    # ------------------------------------------------ 掉线检测 / 自动重连 ----
    def _mark_dead(self, ser, why):
        """把一条失效的链路摘下来。

        【只标记，不关句柄】：关句柄要等 _reap_dead 拿到 rlock，
        否则可能和读线程正在进行的 ReadFile 撞上。"""
        with self.wlock:
            if self.ser is not ser:
                return False              # 别人已经处理过这条链路了
            self.ser = None
            self.name = None
            self._dead.append(ser)
            self._link_gen += 1           # 让还在等 ack 的 cmd 立刻失败
            self._last_error = str(why)
            self._last_logged_error = None
            auto = self._want_open and not self.mock
            if auto:
                self._fail_count = 1      # 断链本身算第一次失败
                self._next_try = time.monotonic() + RECONNECT_BACKOFF[0]
        self.log.add('# 串口链路断了：%s%s'
                     % (why, '，正在后台重连' if auto else '，不会自动重连'), 'out')
        self.log.notify()
        self._wake.set()
        return True

    def _reap_dead(self):
        """真正关闭已经摘下来的句柄。

        拿不到 rlock 说明读线程正卡在 ReadFile 里 —— 放回队列，下一轮再关。
        急着关会在另一个线程还在用这个句柄时把它从脚下抽走。"""
        while True:
            with self.wlock:
                if not self._dead:
                    return
                ser = self._dead.pop(0)
            if not self.rlock.acquire(timeout=1.0):
                with self.wlock:
                    self._dead.insert(0, ser)
                return
            try:
                ser.close()
            except Exception:
                pass
            finally:
                self.rlock.release()

    def _close_serial(self):
        with self.wlock:
            old, self.ser = self.ser, None
            self.name = None
        if old is not None:
            self._dead.append(old)
        self._reap_dead()

    def _note_connect_fail(self, err):
        msg = str(err)
        with self.wlock:
            self._last_error = msg
            self._fail_count += 1
            delay = RECONNECT_BACKOFF[min(self._fail_count - 1,
                                          len(RECONNECT_BACKOFF) - 1)]
            self._next_try = time.monotonic() + delay
            log_it = (self._fail_count == 1 or msg != self._last_logged_error)
            self._last_logged_error = msg
        if log_it:
            self.log.add('# %s 连接失败（第 %d 次，%.0f 秒后再试）：%s'
                         % (self._target_port, self._fail_count, delay, msg), 'out')

    def _supervisor(self):
        """唯一发起自动重连的地方，顺带负责真正关闭废弃句柄。

        单独一个线程（而不是挂在读线程里）：读线程可能正卡在 ReadFile 里，
        或者已经因为句柄失效退出了 —— 那两处都不可靠。"""
        while self.running:
            try:
                self._reap_dead()
                with self.wlock:
                    due = (self._want_open and not self.mock
                           and self.ser is None
                           and time.monotonic() >= self._next_try)
                if due:
                    self._auto_connect()
            except Exception:
                time.sleep(0.5)          # 监督线程自己不能死
            self._wake.wait(0.2)
            self._wake.clear()

    def _auto_connect(self):
        with self._connect_lock:
            with self.wlock:
                # 拿到锁之后再复查一次：手动 /api/connect 可能刚抢先连上。
                if (not self.running or not self._want_open or self.mock
                        or self.ser is not None):
                    return
                port = self._target_port
            if not port:
                return
            try:
                self._connect_once(port)
            except BridgeError as e:
                self._note_connect_fail(e)
                return
            except Exception as e:
                self._note_connect_fail(BridgeError('%s: %s'
                                                    % (type(e).__name__, e)))
                return
            with self.wlock:
                self._reconnect_count += 1
                n = self._reconnect_count
                self._fail_count = 0
                self._last_error = None
                self._last_logged_error = None
                self._next_try = 0.0
            self.log.add('# 已自动重连 %s（第 %d 次）' % (port, n), 'out')

    def _open_mock(self):
        self.ser = MockSerial(self.log)
        self.name = 'MOCK'
        self.log.add('# mock 模式：没有真串口，车上那一半是本地模拟的', 'out')

    def disconnect(self):
        """手动断开：连 _want_open 一起清掉，否则 supervisor 会立刻把口又抢回来。"""
        with self._connect_lock:
            with self.wlock:
                self._want_open = False
                self._fail_count = 0
                self._next_try = 0.0
                self._last_error = None
            self._close_serial()
        self._wake.set()
        return {'closed': True}

    # ---------------------------------------------------------- 收发 ----
    def write_line(self, text):
        data = (text + '\r\n').encode('ascii', 'replace')
        with self.wlock:
            ser = self.ser
            if ser is None:
                raise BridgeError('串口没打开' +
                                  ('，正在后台重连' if self._want_open else ''))
            self.log.add(text, 'out')
            try:
                ser.write(data)
            except BridgeError as e:
                # 写失败（尤其是 GetLastError=22/1167）说明句柄已经作废，
                # 必须立刻摘掉 —— 以前只抛错不清状态，页面会一直看到「已连接」。
                self._mark_dead(ser, e)
                raise
            except Exception as e:
                self._mark_dead(ser, e)
                raise BridgeError('写串口失败：%s' % e)

    def cmd(self, text, timeout=ACK_TIMEOUT):
        """发一条 '#' 命令并等它的 ok/err 回包。

        车上（speed_report.c 的 spd_run_line）每条命令只回一行 ok/err，
        所以"一条一发一等"就是最省事的流控 —— 不用管车的接收缓冲会不会满。
        """
        gen = self._link_gen
        start = self.log.mark()
        self.write_line('#' + text)
        ack = self.log.wait_ack(start, time.time() + timeout,
                                abort=lambda: self._link_gen != gen)
        if ack is None:
            if self._link_gen != gen:
                raise BridgeError('发「%s」时串口断了，命令没有被确认'
                                  '（正在重连，不会自动重发）' % text)
            raise BridgeError('车没有回应「%s」——检查蓝牙连接、波特率(%d)、'
                              '或者车上是否在复位' % (text, self.baud))
        if ack.startswith('err '):
            raise BridgeError('车拒绝了「%s」——原因看日志里上一条 fol 开头的行'
                              '（车上会把原因也发到链路上）' % text)
        return ack

    # -------------------------------------------------------- 后台线程 ----
    def _reader(self):
        buf = b''
        last_ser = None
        while self.running:
            ser = self.ser
            if ser is None:
                buf = b'' if last_ser is None else buf
                time.sleep(0.05)
                continue
            if ser is not last_ser:
                last_ser = ser
                buf = b''                 # 新链路：丢掉上一条残留的半行
            try:
                with self.rlock:
                    if self.ser is not ser:   # 已经被摘掉了，别再碰旧句柄
                        continue
                    data = ser.read(512)
            except Exception as e:
                self._mark_dead(ser, e)
                continue
            if not data:
                # 非阻塞读：空转别把 CPU 占满，也给 _reap_dead 留出拿 rlock 的机会
                time.sleep(0.005)
                continue
            buf += data
            while b'\n' in buf:
                raw, buf = buf.split(b'\n', 1)
                self._on_line(raw.decode('ascii', 'replace').strip('\r'))

    def _on_line(self, s):
        if not s:
            return
        self.log.add(s, 'in')
        for key, rx in (('fol', FOL_RE), ('load', LOAD_RE), ('odo', ODO_RE)):
            m = rx.match(s)
            if m:
                d = m.groupdict()
                for k in d:
                    if k != 'st':
                        d[k] = int(d[k])
                d['raw'] = s
                d['t'] = time.time()
                self.latest[key] = d
                break

    def _heartbeat(self):
        """跑动期间必须一直有字节发过去，否则车上的失联急停会把车刹掉。

        发的是 `#follow ping`：车上是个空操作，但**收到任何字节**都会刷新
        bt_car 的接收计时（见 follow.h 里 ping 那段注释）。"""
        while self.running:
            time.sleep(HEARTBEAT_S)
            with self.wlock:
                ser = self.ser
                if ser is None:
                    continue
                try:
                    ser.write(b'#follow ping\r\n')
                except Exception as e:
                    # 以前这里是 except: pass —— 死句柄最常在这里第一次暴露，
                    # 吞掉的话页面只会看到「写失败」而不知道整条链路已经没了。
                    self._mark_dead(ser, '心跳发送失败：%s' % e)
            # 心跳不进日志（不然日志全是它），但它是真的发出去了

    # ---------------------------------------------------------- 下载 ----
    def send_path(self, points, mode='stop', speed=None, autostart=True):
        if len(points) < 2:
            raise BridgeError('至少要 2 个航点')
        if len(points) > MAX_POINTS:
            raise BridgeError('这条轨迹采样出 %d 个点，超过车上的上限 %d 个。'
                              '把"采样间距"调大一点再画（点少不等于轨迹不准，'
                              '车上是靠航向走的，不是靠点密）'
                              % (len(points), MAX_POINTS))

        wps = []
        for i, p in enumerate(points):
            x = int(round(float(p['x']) * 1000.0))
            y = int(round(float(p['y']) * 1000.0))
            h = int(round(float(p.get('heading_deg', 0.0)) * 10.0))
            if abs(x) > COORD_LIMIT_MM or abs(y) > COORD_LIMIT_MM:
                raise BridgeError('第 %d 个点 (%.3f, %.3f) m 离起点太远，'
                                  '坐标要在 ±%d m 之内'
                                  % (i + 1, x / 1000.0, y / 1000.0,
                                     COORD_LIMIT_MM // 1000))
            if abs(h) > HDEG_LIMIT:
                raise BridgeError('第 %d 个点的航向 %.1f 不合法'
                                  % (i + 1, h / 10.0))
            wps.append((x, y, h))

        done = []
        self.cmd('follow begin %d' % len(wps))
        for i, (x, y, h) in enumerate(wps):
            self.cmd('follow wp %d %d %d' % (x, y, h))
            done.append(i + 1)
        self.cmd('follow end')

        if speed:
            self.cmd('follow spd %d' % int(speed))
        if autostart:
            self.cmd('follow start %s' % ('cont' if mode == 'cont' else 'stop'))

        return {'sent': len(done), 'mode': mode, 'autostart': bool(autostart),
                'load': self.latest.get('load')}

    # ---------------------------------------------------------- 状态 ----
    def state(self):
        with self.wlock:
            ser = self.ser
            name = self.name
            target = self._target_port
            reconnecting = bool(self._want_open and not self.mock and ser is None)
            retry_in_ms = int(max(0.0, self._next_try - time.monotonic()) * 1000) \
                if reconnecting else 0
            reconnect_count = self._reconnect_count
            last_error = self._last_error
        return {
            'ok': True,
            'open': ser is not None,      # 死句柄在这里永远是 False
            'port': name,
            'baud': self.baud,
            'mock': self.mock,
            'link_ms': self.link_ms,
            'lines': self.log.mark(),
            'fol': self.latest.get('fol'),
            'load': self.latest.get('load'),
            'odo': self.latest.get('odo'),
            # 重连状态（页面用来显示「正在后台重连」而不是「串口没打开」）
            'target_port': target,
            'reconnecting': reconnecting,
            'reconnect_count': reconnect_count,
            'retry_in_ms': retry_in_ms,
            'last_error': last_error,
        }


# ============================================================ HTTP ==========
def make_handler(bridge, page_path):
    class Handler(BaseHTTPRequestHandler):
        server_version = 'smartcar-bridge/1.0'
        protocol_version = 'HTTP/1.1'

        # 页面可能是 file:// 打开的（那样 origin 是 null），给个通吃的 CORS 头，
        # 双击打开也能通过这个桥控车，只是用不了 Web Serial。
        def _cors(self):
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Access-Control-Allow-Headers', 'Content-Type')
            self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')

        def _send(self, code, body, ctype='application/json; charset=utf-8'):
            if isinstance(body, str):
                body = body.encode('utf-8')
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self._cors()
            self.end_headers()
            self.wfile.write(body)

        def _json(self, obj, code=200):
            self._send(code, json.dumps(obj, ensure_ascii=False))

        def _body(self):
            n = int(self.headers.get('Content-Length') or 0)
            if n <= 0:
                return {}
            try:
                return json.loads(self.rfile.read(n).decode('utf-8'))
            except Exception as e:
                raise BridgeError('请求体不是合法 JSON：%s' % e)

        def log_message(self, fmt, *args):
            # 本机轮询 400ms 一次，全打出来会把控制台刷满，所以默认那行不要。
            # 但**来自别的机器**的请求必须留一行：手机连不上时，这一行是
            # "请求到底有没有走到 PC"唯一能证的东西 —— 没有这一行就说明包
            # 根本没到（无线网做了客户端隔离 / 不在同一个网段），
            # 有这一行才轮到查 PC 这边。
            if self.client_address[0] not in ('127.0.0.1', '::1'):
                sys.stderr.write('[远端 %s] %s\n'
                                 % (self.client_address[0], fmt % args))

        # ---- 路由 ----
        def do_OPTIONS(self):
            self._send(204, b'')

        def do_GET(self):
            path = self.path.split('?', 1)[0]
            query = {}
            if '?' in self.path:
                for kv in self.path.split('?', 1)[1].split('&'):
                    if '=' in kv:
                        k, v = kv.split('=', 1)
                        query[k] = v

            if path in ('/', '/index.html', '/' + PAGE_NAME):
                try:
                    with open(page_path, 'rb') as f:
                        self._send(200, f.read(), 'text/html; charset=utf-8')
                except OSError as e:
                    self._send(404, '打不开 %s：%s' % (page_path, e),
                               'text/plain; charset=utf-8')
                return

            if path == '/api/state':
                self._json(bridge.state())
                return

            if path == '/api/lines':
                try:
                    since = int(query.get('since', '0'))
                except ValueError:
                    since = 0
                lines, nxt = bridge.log.since(since)
                self._json({'ok': True, 'next': nxt, 'lines': lines})
                return

            if path == '/api/ports':
                self._json({'ok': True, 'ports': list_ports()})
                return

            self._json({'ok': False, 'error': '没有这个路径：%s' % path}, 404)

        def do_POST(self):
            path = self.path.split('?', 1)[0]
            try:
                body = self._body()
                if path == '/api/connect':
                    port = body.get('port') or bridge.args.port
                    if not port and not bridge.mock:
                        raise BridgeError('没有指定串口。用 --list 看有哪些，'
                                          '或者启动时加 --port COM14')
                    if bridge.mock:
                        bridge._open_mock()
                        self._json({'ok': True, 'state': bridge.state()})
                        return
                    if body.get('baud'):
                        bridge.baud = int(body['baud'])
                    r = bridge.connect(port)
                    self._json({'ok': True, 'result': r, 'state': bridge.state()})
                    return

                if path == '/api/disconnect':
                    r = bridge.disconnect()
                    self._json({'ok': True, 'result': r, 'state': bridge.state()})
                    return

                if path == '/api/path':
                    pts = body.get('points') or []
                    r = bridge.send_path(pts, body.get('mode', 'stop'),
                                         body.get('speed'), body.get('start', True))
                    self._json({'ok': True, 'result': r})
                    return

                if path == '/api/follow':
                    cmd = body.get('cmd', 'start')
                    if cmd in ('start', 'cont'):
                        extra = body.get('speed')
                        if extra:
                            bridge.cmd('follow spd %d' % int(extra))
                        ack = bridge.cmd('follow start %s'
                                         % ('cont' if cmd == 'cont' else 'stop'))
                    elif cmd == 'stop':
                        ack = bridge.cmd('follow stop')
                    elif cmd == 'status':
                        ack = bridge.cmd('follow status')
                    elif cmd == 'odom':
                        ack = bridge.cmd('follow odom')
                    else:
                        raise BridgeError('不认识的 follow 子命令：%s' % cmd)
                    self._json({'ok': True, 'ack': ack})
                    return

                if path == '/api/send':
                    line = (body.get('line') or '').strip()
                    if not line:
                        raise BridgeError('要发什么？')
                    ack = bridge.cmd(line.lstrip('#'))
                    self._json({'ok': True, 'ack': ack})
                    return

                self._json({'ok': False, 'error': '没有这个路径：%s' % path}, 404)
            except BridgeError as e:
                self._json({'ok': False, 'error': str(e)}, 200)
            except Exception as e:                  # 别让一个异常把连接弄死
                self._json({'ok': False, 'error': '%s: %s'
                            % (type(e).__name__, e)}, 200)

    return Handler


# ============================================================ 串口枚举 ======
def list_ports():
    try:
        from serial.tools import list_ports as lp
        return [{'port': p.device, 'desc': p.description or ''}
                for p in lp.comports()]
    except Exception:
        pass

    if os.name != 'nt':
        import glob
        return [{'port': p, 'desc': ''} for p in sorted(glob.glob('/dev/tty*'))]

    out = []
    try:
        import winreg
        key = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE,
                             r'HARDWARE\DEVICEMAP\SERIALCOMM')
        i = 0
        while True:
            try:
                name, val, _ = winreg.EnumValue(key, i)
            except OSError:
                break
            out.append({'port': val, 'desc': name})
            i += 1
    except OSError:
        pass
    return sorted(out, key=lambda d: d['port'])


def local_ips():
    """本机的局域网 IPv4，用来告诉人"手机该开哪个地址"。

    【为什么要自动查】这台笔记本在校园网里换个 AP 就重新拿一个 DHCP 租约 ——
    实测二十分钟内就从 10.151.50.198 变成 10.150.240.239。地址一漂，
    手机上原来那个 URL 就指向别的设备了，现象是"同一个网就是打不开"。
    所以启动时直接把当前地址打出来，省得去猜。
    """
    out = []
    try:
        # UDP connect 不会真的发包，只是让内核按路由表挑一张网卡出来
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(('8.8.8.8', 80))
        out.append(s.getsockname()[0])
        s.close()
    except OSError:
        pass
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            ip = info[4][0]
            if ip not in out and not ip.startswith('127.'):
                out.append(ip)
    except OSError:
        pass
    return out


# ============================================================ main ==========
def main(argv=None):
    ap = argparse.ArgumentParser(
        description='smartcar bridge: 把上位机页面接到小车（HC05 蓝牙）上')
    ap.add_argument('--port', help='串口名，例如 COM14（蓝牙配对出来的那个）')
    ap.add_argument('--baud', type=int, default=38400,
                    help='波特率，默认 38400（实测这块 HC-05 就是 38400）')
    ap.add_argument('--http', type=int, default=8000, help='HTTP 端口，默认 8000')
    ap.add_argument('--host', default='127.0.0.1',
                    help='监听地址，默认 127.0.0.1（只给本机用）')
    ap.add_argument('--link-ms', type=int, default=1000,
                    help='连上后自动下发的失联急停时限（ms），0 = 不动它')
    ap.add_argument('--mock', action='store_true', help='不接硬件，模拟一块车')
    ap.add_argument('--list', action='store_true', help='列出可用串口然后退出')
    ap.add_argument('--page', default=None, help='页面文件路径，默认同目录下的 %s'
                    % PAGE_NAME)
    args = ap.parse_args(argv)

    if args.list:
        ports = list_ports()
        if not ports:
            print('没找到任何串口。蓝牙要先在系统里配对好（Windows 的'
                  '「蓝牙和其他设备」里会出现两个 COM 口：一个"传出"、一个'
                  '"传入"，连车要用"传出"那个）。')
        for p in ports:
            print('  %-8s %s' % (p['port'], p['desc']))
        return 0

    here = os.path.dirname(os.path.abspath(__file__))
    page = args.page or os.path.join(here, PAGE_NAME)
    if not os.path.exists(page):
        print('警告：找不到页面 %s（接口照样能用，但 / 会 404）' % page)

    bridge = Bridge(args)

    if args.mock:
        print('mock 模式：车上那半边是本地模拟的，可以直接测页面和协议。')

    httpd = ThreadingHTTPServer((args.host, args.http), make_handler(bridge, page))
    url = 'http://%s:%d/' % (args.host, args.http)
    print('bridge 已启动：%s' % url)
    if args.host in ('0.0.0.0', '::', ''):
        for ip in local_ips():
            print('  手机/别的设备开这个： http://%s:%d/' % (ip, args.http))
        print('        （校园网里换个 AP 就会重新拿租约、地址会变；'
              '连不上就重跑一次本命令看新地址）')
        print('        同一网段里谁都能连上并控车 —— 跑完就把这个窗口 Ctrl-C 掉。')
    else:
        print('  只听 %s：别的设备连不上。要让手机访问，用 --host 0.0.0.0' % args.host)
    print('  页面   %s' % page)
    if bridge.name:
        print('  串口   %s' % bridge.name)
    elif args.port:
        print('  串口   %s（暂时打不开，后台自动重试）' % args.port)
    else:
        print('  串口   （未连接，页面上点"连接"）')
    print('  失联急停 %s' % ('%d ms' % args.link_ms if args.link_ms else '不干预'))
    print('Ctrl-C 退出。注意：这个进程拿着串口的时候，别的串口工具打不开它。')
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print('\n退出中……')
    finally:
        bridge.running = False
        bridge.disconnect()
        httpd.server_close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
