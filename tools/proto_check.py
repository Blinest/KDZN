#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
proto_check.py —— 上下位机通信协议对接自检

按《上下位机通信协议（指令集）》的检查清单逐项自动验证，输出 PASS/FAIL 报告。

检查项:
  0  静默监听      —— 判断连接/波特率是否正常
  1  电机使能      —— 可选, 会驱动电机 (--enable-motor)
  2  V2 协议协商   —— AA 09 01 02  →  期待 55 00 00 04 ...
  3  周期收帧      —— 统计 55 01(状态) / 55 02(感知) 的实际到达间隔
  4  状态帧解析    —— 打印 6 路 force(滤波+平衡) 与 raw(原始)
  5  安装方向切换  —— 可选 (--mount-test), 验证命令后数据流不中断
  6  标定事件监听  —— 上电自动标定若失败会主动上报 55 03

用法:
    python proto_check.py --port COM7
    python proto_check.py --port COM7 --enable-motor
    python proto_check.py --port COM7 --mount-test --dump

依赖:  pip install pyserial
"""

import argparse
import struct
import sys
import time

# 说明: 串口库惰性导入 —— 协议层(帧构造/解析/解码)不依赖 pyserial,
#       便于单独做单元测试或嵌入其他程序。
serial = None


def _ensure_serial():
    global serial
    if serial is None:
        try:
            import serial as _s
            serial = _s
        except ImportError:
            print("[FAIL] 缺少 pyserial, 请先执行:  pip install pyserial")
            sys.exit(2)
    return serial


# ==================== 协议常量 ====================
MAGIC_DOWN  = 0xAA
MAGIC_UP_V1 = 0xBB
MAGIC_UP_V2 = 0x55

DEV_SYS, DEV_DRIVE, DEV_TENSION, DEV_CALIB = 0x00, 0x01, 0x02, 0x03

C_SYS_HANDSHAKE = 0x09
C_SET_MOUNT     = 0x30

HEALTH    = ["正常", "松弛", "断丝", "过张", "过载"]
CALIB_ST  = ["未运行", "进行中", "成功", "失败"]
CALIB_ERR = {0: "无", 1: "标定被禁用", 2: "电机无响应(未使能/未上电/CAN不通)",
             3: "Δτ过小", 4: "Δτ为负(方向反)", 5: "离散过大",
             6: "电机未到位", 7: "信号未稳定"}


# ==================== 帧构造 ====================
def checksum(buf: bytes) -> int:
    return sum(buf) & 0xFF


def build(func: int, data: bytes = b"") -> bytes:
    assert len(data) <= 255
    body = bytes([MAGIC_DOWN, func, len(data)]) + data
    return body + bytes([checksum(body)])


def cmd_motor_enable() -> bytes:
    return build(0x01)


def cmd_handshake(ver: int = 2) -> bytes:
    return build(C_SYS_HANDSHAKE, bytes([ver]))


def cmd_set_mount(vec) -> bytes:
    d = b""
    for v in vec:
        d += struct.pack(">h", max(-32768, min(32767, int(round(v * 1000)))))
    return build(C_SET_MOUNT, d)


def i16(v, scale=1.0):
    return max(-32768, min(32767, int(round(v * scale))))


# ==================== 解析 ====================
class Frame:
    __slots__ = ("ver", "dev", "func", "data")

    def __init__(self, ver, dev, func, data):
        self.ver, self.dev, self.func, self.data = ver, dev, func, data

    def __repr__(self):
        return f"<{'V2' if self.ver == 2 else 'V1'} dev=0x{self.dev:02X} func=0x{self.func:02X} len={len(self.data)}>"


class Parser:
    """帧头同步 + 长度收帧 + 校验; 同时兼容 V1(0xBB) 与 V2(0x55), 可混流"""

    def __init__(self):
        self.buf = bytearray()
        self.bad_crc = 0
        self.dropped = 0

    def feed(self, chunk: bytes):
        self.buf.extend(chunk)
        out = []
        while True:
            while self.buf and self.buf[0] not in (MAGIC_UP_V1, MAGIC_UP_V2):
                self.buf.pop(0)
                self.dropped += 1
            if not self.buf:
                break

            magic = self.buf[0]
            hdr = 3 if magic == MAGIC_UP_V1 else 4
            if len(self.buf) < hdr:
                break

            if magic == MAGIC_UP_V1:
                dev, func, ln = 0, self.buf[1], self.buf[2]
            else:
                dev, func, ln = self.buf[1], self.buf[2], self.buf[3]

            total = hdr + ln + 1
            if len(self.buf) < total:
                break

            raw = bytes(self.buf[:total])
            if checksum(raw[:-1]) != raw[-1]:
                self.bad_crc += 1
                self.buf.pop(0)
                continue

            out.append(Frame(2 if magic == MAGIC_UP_V2 else 1, dev, func, raw[hdr:-1]))
            del self.buf[:total]
        return out


def decode_status(d: bytes):
    if len(d) != 73:
        return None
    motors = []
    for i in range(6):
        pos, vel, acc, st = struct.unpack_from(">hhhB", d, 2 + i * 7)
        motors.append((pos / 100, vel / 100, acc / 100, st))
    sensors = []
    for i in range(6):
        x, y = struct.unpack_from(">hh", d, 44 + i * 4)
        sensors.append((x / 100, y / 100))
    scale, armbend = struct.unpack_from(">hh", d, 68)
    return dict(motors=motors, sensors=sensors,
                scale=scale / 100, armbend=armbend / 100, state=d[72])


def decode_percept(d: bytes):
    if len(d) != 50:
        return None
    s_norm, dist, fx, fy, fmag = struct.unpack_from(">hhhhh", d, 2)
    return dict(
        contact=bool(d[0]), seg=d[1],
        s_norm=s_norm / 1e4, disturbance=dist / 1e4,
        F=(fx / 1e4, fy / 1e4), F_mag=fmag / 100,
        health=[HEALTH[h] if h < len(HEALTH) else str(h) for h in d[12:18]],
        share_err=[v / 1e4 for v in struct.unpack_from(">6h", d, 18)],
        force=[v / 100 for v in struct.unpack_from(">6h", d, 30)],
        balance_gain=[g / 100 for g in d[42:48]],
        calib_status=CALIB_ST[d[48]] if d[48] < len(CALIB_ST) else str(d[48]),
        calib_err=CALIB_ERR.get(d[49], str(d[49])),
    )


def decode_calib(d: bytes):
    if len(d) != 3:
        return None
    mask = d[2]
    return dict(stage={1: "PRE_SETTLE", 2: "POST_MOVE", 3: "POST_SETTLE"}.get(d[0], str(d[0])),
                err=CALIB_ERR.get(d[1], str(d[1])),
                mask=mask, bits=[i + 1 for i in range(6) if mask >> i & 1])


# ==================== 检查框架 ====================
class Ctx:
    def __init__(self, ser, dump=False):
        self.ser = ser
        self.p = Parser()
        self.dump = dump
        self.results = []

    def send(self, frame: bytes, note=""):
        self.ser.write(frame)
        if self.dump:
            print(f"  TX  {' '.join(f'{b:02X}' for b in frame)}   {note}")

    def pump(self, seconds, collect=None):
        """读取 seconds 秒, 返回所有帧; collect(frame) 可实时回调"""
        deadline = time.time() + seconds
        got = []
        while time.time() < deadline:
            chunk = self.ser.read(4096)
            if chunk:
                for fr in self.p.feed(chunk):
                    got.append(fr)
                    if self.dump:
                        print(f"  RX  {' '.join(f'{b:02X}' for b in fr.data[:12])}"
                              f"{' ...' if len(fr.data) > 12 else ''}   {fr}")
                    if collect:
                        collect(fr)
            else:
                time.sleep(0.005)
        return got

    def report(self, ok, name, detail=""):
        self.results.append((ok, name, detail))
        tag = "[ OK ]" if ok else "[FAIL]"
        print(f"{tag} {name}" + (f"   {detail}" if detail else ""))


# ==================== 各检查项 ====================
def step0_silent(ctx, seconds=3.0):
    print("\n--- 0. 静默监听 (判断连接/波特率) ---")
    frames = ctx.pump(seconds)
    n = len(frames)
    kinds = {}
    for f in frames:
        key = (f.ver, f.dev, f.func)
        kinds[key] = kinds.get(key, 0) + 1
    ctx.report(n > 0, "接收到上行帧",
               f"{seconds:.0f}s 内 {n} 帧, 校验错 {ctx.p.bad_crc}, 丢弃字节 {ctx.p.dropped}")
    if n == 0:
        print("      可能原因: 端口错 / 波特率错 / 下位机未上电 / 未发过任何指令")
    for k, v in sorted(kinds.items()):
        ver, dev, func = k
        print(f"      {'V2' if ver == 2 else 'V1'} dev=0x{dev:02X} func=0x{func:02X}  ×{v}")
    return frames


def step1_motor_enable(ctx):
    print("\n--- 1. 电机使能 (会驱动电机) ---")
    ctx.send(cmd_motor_enable(), "AA 01 00 01")
    ctx.pump(0.5)
    print("      该指令无应答帧; 请人工确认电机是否变刚性(有保持力)")
    ctx.report(True, "已发送使能指令", "无应答, 需人工确认")


def step2_handshake(ctx, timeout=2.0):
    print("\n--- 2. V2 协议协商 ---")
    for attempt in range(1, 4):
        ctx.send(cmd_handshake(2), "AA 09 01 02")
        deadline = time.time() + timeout
        while time.time() < deadline:
            for fr in ctx.pump(0.1):
                if fr.ver == 2 and fr.dev == DEV_SYS and fr.func == 0x00 and len(fr.data) >= 4:
                    ver, mn, sn, flags = fr.data[0], fr.data[1], fr.data[2], fr.data[3]
                    ctx.report(True, "收到 V2 握手应答",
                               f"ver={ver} motor_n={mn} sensor_n={sn} flags={flags}")
                    return True
        print(f"      第 {attempt} 次未收到, 重试…")
    ctx.report(False, "收到 V2 握手应答", f"{timeout}s × 3 次超时")
    print("      排查: 握手帧的校验和 / 下位机是否已烧入含 proto.c 的固件")
    return False


def step3_periodic(ctx, seconds=4.0):
    print(f"\n--- 3. 周期收帧 ({seconds:.0f}s) ---")
    stamps = {"status": [], "percept": []}

    def cb(fr):
        if fr.ver == 2 and fr.dev == DEV_DRIVE and fr.func == 0x00:
            stamps["status"].append(time.time())
        elif fr.ver == 2 and fr.dev == DEV_TENSION and fr.func == 0x00:
            stamps["percept"].append(time.time())

    ctx.pump(seconds, cb)

    for key, expect in (("status", 200), ("percept", 500)):
        ts = stamps[key]
        if len(ts) >= 2:
            gaps = [(ts[i + 1] - ts[i]) * 1000 for i in range(len(ts) - 1)]
            avg = sum(gaps) / len(gaps)
            ok = abs(avg - expect) < expect * 0.5
            ctx.report(ok, f"{key} 周期",
                       f"{len(ts)} 帧, 平均 {avg:.0f} ms (期望 ≈{expect} ms)")
        else:
            ctx.report(False, f"{key} 周期", f"仅 {len(ts)} 帧")
    return stamps


def step4_status(ctx, frames):
    print("\n--- 4. 状态帧解析 ---")
    ok_frame = None
    for f in frames:
        if f.ver == 2 and f.dev == DEV_DRIVE and f.func == 0x00:
            d = decode_status(f.data)
            if d:
                ok_frame = d
                break
    if not ok_frame:
        ctx.report(False, "状态帧解析", "未取到有效状态帧")
        return
    print("      序号   位置(mm)    速度(mm/s)   state    force(N)    raw(N)")
    for i in range(6):
        pos, vel, acc, st = ok_frame["motors"][i]
        fx, fy = ok_frame["sensors"][i]
        print(f"        {i + 1}   {pos:9.3f}  {vel:9.3f}      {st:3d}   {fx:9.3f}  {fy:9.3f}")
    print(f"      scale={ok_frame['scale']}  armbend={ok_frame['armbend']}  state={ok_frame['state']}")
    ctx.report(True, "状态帧解析", "6 路电机 + 6 路传感器字段齐全")


def step5_mount(ctx):
    print("\n--- 5. 安装方向切换 (AA 30) ---")
    before = ctx.pump(1.0)
    before_n = len(before)
    ctx.send(cmd_set_mount((1.0, 0.0, 0.0)), "AA 30 06 03E8 0000 0000  (水平沿 X)")
    after = ctx.pump(2.0)
    ctx.report(len(after) > 0, "切换后数据流未中断",
               f"切换前 {before_n} 帧 → 切换后 {len(after)} 帧")
    print("      提示: 重力基准切换的效果需人工判读感知帧 disturbance / share_err 变化")


def step6_calib(ctx, seconds=5.0):
    print(f"\n--- 6. 标定事件监听 ({seconds:.0f}s) ---")
    found = []
    ctx.pump(seconds, lambda fr: found.append(fr) if (fr.ver == 2 and fr.dev == DEV_CALIB) else None)
    if found:
        for f in found:
            e = decode_calib(f.data)
            if e:
                ctx.report(e["err"] == "无", "收到标定事件帧",
                           f"stage={e['stage']} err={e['err']} 未到位电机={e['bits']}")
                if e["bits"]:
                    print(f"      未到位: 电机 {e['bits']}")
        return
    ctx.report(True, "标定事件监听", "未收到事件 (标定成功或尚未触发均属正常)")


# ==================== 主流程 ====================
def main():
    ap = argparse.ArgumentParser(description="上下位机协议对接自检")
    ap.add_argument("--port", required=True, help="串口, 如 COM7")
    ap.add_argument("--baud", type=int, default=9600)
    ap.add_argument("--enable-motor", action="store_true", help="执行电机使能(会驱动电机)")
    ap.add_argument("--mount-test", action="store_true", help="执行安装方向切换测试")
    ap.add_argument("--dump", action="store_true", help="打印每一帧原始字节")
    ap.add_argument("--no-handshake", action="store_true", help="跳过 V2 协商(仅看 V1)")
    args = ap.parse_args()

    print("=" * 62)
    print(" 上下位机通信协议对接自检")
    print(f" 端口 {args.port} @ {args.baud} 8N1   (Ctrl+C 中止)")
    print("=" * 62)
    print(" 注意: 本脚本只读监听 + 发送协议指令, 不会写任何配置到磁盘。")
    if not args.enable_motor:
        print(" 电机使能步骤已跳过 (如需执行请加 --enable-motor)")

    _ensure_serial()
    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.05)
    except Exception as e:
        print(f"[FAIL] 打开串口失败: {e}")
        return 2

    ctx = Ctx(ser, dump=args.dump)
    try:
        step0_silent(ctx)
        if args.enable_motor:
            step1_motor_enable(ctx)
        v2 = True
        if not args.no_handshake:
            v2 = step2_handshake(ctx)
        frames = step3_periodic(ctx)
        if v2:
            step4_status(ctx, frames)
        else:
            print("\n--- 4. 状态帧解析 (跳过: V1 模式) ---")
        if args.mount_test:
            step5_mount(ctx)
        step6_calib(ctx)
    except KeyboardInterrupt:
        print("\n(用户中止)")
    finally:
        ser.close()

    print("\n" + "=" * 62)
    print(" 汇总")
    print("=" * 62)
    npass = sum(1 for ok, _, _ in ctx.results if ok)
    for ok, name, detail in ctx.results:
        print(f"  [{'OK' if ok else 'NG'}] {name:<22} {detail}")
    print(f"\n  通过 {npass}/{len(ctx.results)}")
    print(f"  解析统计: 校验错 {ctx.p.bad_crc} 帧, 丢弃无效字节 {ctx.p.dropped}")
    return 0 if npass == len(ctx.results) else 1


if __name__ == "__main__":
    sys.exit(main())
