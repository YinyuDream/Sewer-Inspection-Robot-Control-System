#!/usr/bin/env python3
"""
FreeRTOS CAN 通信模拟测试脚本
使用 python-can 库与 vcan0 接口通信，模拟多任务 CAN 数据处理
"""

import can
import threading
import queue
import time
import struct
import signal
import sys
from typing import List
import math

# ---------- 全局配置 ----------
CAN_INTERFACE = 'vcan0'
CAN_BUSTYPE = 'socketcan'

# 任务状态数组（模拟 FreeRTOS 中的 status 数组）
motion_status = [0.0] * 20      # ID 0x100 ~ 0x1FF
power_status = [0.0] * 10       # ID 0x000 ~ 0x0FF
sensor_data = [0.0] * 32        # ID 0x200 ~ 0x21F  (更多传感器槽位)

# 控制标志
running = True
control_trigger = threading.Event()

# 帧序列诊断
_frame_seq = []          # 当前周期收到的帧 ID 列表
_frame_seq_lock = threading.Lock()
_cycle_count = 0

# 发送锁（防止多个线程同时发送）
send_lock = threading.Lock()

# ---------- CAN 发送函数 ----------
def send_can(can_id: int, data_bytes: bytes):
    """发送 CAN 消息（标准帧）"""
    msg = can.Message(
        arbitration_id=can_id,
        data=data_bytes,
        is_extended_id=False
    )
    with send_lock:
        bus.send(msg)
    # print(f"[TX] ID=0x{can_id:03X}, data={data_bytes.hex()}")

# ---------- 各任务线程 ----------

def motion_control_task():
    """运动控制任务：实现Stanley轨迹+差速速度分配+电机电压PID。
    周期 20ms，读取 motion_queue/ekf_queue 的最新数据并输出 CAN 控制帧 0x180-0x183。
    """
    print("MotionControlTask 启动")
    # 控制参数 (与 firmware/control.c 对应)
    DT = 0.01  # 20ms
    WHEEL_RADIUS = 0.16
    L = 0.6
    W = 0.68
    K_STANLEY = 2.5
    K_SOFT = 2.0
    K_ROLL_P = 2.5; K_ROLL_I = 0.5; K_ROLL_D = 1.5
    KP = 2.8; KI = 1.0; KD = 0.05; KF = 0.5
    BATTERY_VOLTAGE = 12.0

    # 内部状态
    target_speed = 0.0
    current_delta = 0.0
    filtered_delta = 0.0
    voltage_i = [0.0, 0.0]
    last_error = [0.0, 0.0]
    roll_i = 0.0

    _last_ctrl_time = None
    _ctrl_count = 0
    _ctrl_dt_sum = 0.0

    while running:
        # 等待 0x108 触发
        if not control_trigger.wait(timeout=0.1):
            continue
        control_trigger.clear()

        if not running:
            break

        # 获取并清空帧序列
        global _cycle_count
        with _frame_seq_lock:
            seq = _frame_seq.copy()
            _frame_seq.clear()
        _cycle_count += 1
        if _cycle_count <= 20 or _cycle_count % 200 == 0:
            print(f"[SEQ#{_cycle_count}] {len(seq)} frames: {[f'0x{x:03X}' for x in seq]}")

        # 测量实际控制周期
        _now = time.perf_counter()
        if _last_ctrl_time is not None:
            _actual_dt = _now - _last_ctrl_time
            _ctrl_dt_sum += _actual_dt
            _ctrl_count += 1
            if _ctrl_count % 100 == 0:
                _avg_dt = _ctrl_dt_sum / _ctrl_count
                print(f"[DIAG] 实际平均控制周期: {_avg_dt*1000:.2f}ms ({1.0/_avg_dt:.1f}Hz), 样本数: {_ctrl_count}")
        _last_ctrl_time = _now

        # 获取最新的 dt (从 motion_status[16] 获取，对应 0x108 的第一个 float)
        motion_status_copy = motion_status.copy()  # 避免在处理过程中被更新
        raw_DT = motion_status_copy[16] if motion_status_copy[16] > 0 else 0.01
        # 对 DT 做指数平滑，消除单帧波动对微分项的冲击
        if not hasattr(motion_control_task, '_smooth_dt'):
            motion_control_task._smooth_dt = raw_DT
        motion_control_task._smooth_dt = 0.8 * motion_control_task._smooth_dt + 0.2 * raw_DT
        DT = motion_control_task._smooth_dt
        if _ctrl_count % 100 == 0:
            print(f"[DIAG] DT={DT*1000:.2f}ms | pos=({motion_status_copy[0]:.3f},{motion_status_copy[1]:.3f},{motion_status_copy[2]:.3f}) | "
                  f"quat=({motion_status_copy[3]:.4f},{motion_status_copy[4]:.4f},{motion_status_copy[5]:.4f},{motion_status_copy[6]:.4f}) | "
                  f"vel=({motion_status_copy[7]:.3f},{motion_status_copy[8]:.3f},{motion_status_copy[9]:.3f}) | "
                  f"ang=({motion_status_copy[10]:.3f},{motion_status_copy[11]:.3f}) | "
                  f"[12-13]=({motion_status_copy[12]:.3f},{motion_status_copy[13]:.3f}) | "
                  f"wheel=({motion_status_copy[14]:.3f},{motion_status_copy[15]:.3f})")
        # 解析状态：根据 can_transceiver.py 的映射关系提取变量
        # 0x100: x, y | 0x101: z, qx | 0x102: qy, qz | 0x103: qw, vx | 0x104: vy, vz
        x = motion_status_copy[0]
        y = motion_status_copy[1]
        z = motion_status_copy[2]
        qx = motion_status_copy[3]
        qy = motion_status_copy[4]
        qz = motion_status_copy[5]
        qw = motion_status_copy[6]
        vx = motion_status_copy[7]
        vy = motion_status_copy[8]
        vz = motion_status_copy[9]
        v_linear = math.sqrt(vx*vx + vy*vy + vz*vz)

        # 0x105: vel_ang_x, vel_ang_y (roll_rate)
        roll_rate = motion_status_copy[10] 
        
        # 0x107: wheel_speed_rl, wheel_speed_rr
        w_act_l = motion_status_copy[14]
        w_act_r = motion_status_copy[15]

        # 计算 yaw, roll
        # euler from quaternion (simplified, only roll & yaw needed)
        def euler_from_quat(xq,yq,zq,wq):
            t0 = +2.0 * (wq * xq + yq * zq)
            t1 = +1.0 - 2.0 * (xq*xq + yq*yq)
            roll = math.atan2(t0, t1)
            t3 = +2.0 * (wq * zq + xq * yq)
            t4 = +1.0 - 2.0 * (yq*yq + zq*zq)
            yaw = math.atan2(t3, t4)
            return roll, yaw

        roll, yaw = euler_from_quat(qx,qy,qz,qw)

        # 前轴位置
        CAR_LENGTH = 0.3
        front_x = x + CAR_LENGTH * math.cos(yaw)
        front_y = y + CAR_LENGTH * math.sin(yaw)

        # 简化 corner geometric calculation：与 firmware 中对应
        CORNER_CENTER = 8.0
        RADIUS = 1.6
        def geometric_calc(px, py, yaw):
            d_path = CORNER_CENTER + RADIUS
            psi = 0.0
            e = 0.0
            # corners
            if px > CORNER_CENTER and py > CORNER_CENTER:
                xc, yc = CORNER_CENTER, CORNER_CENTER
                dist = math.hypot(px-xc, py-yc)
                e = dist - RADIUS
                angle_to_center = math.atan2(py-yc, px-xc)
                psi = angle_to_center + math.pi/2.0
            elif px < -CORNER_CENTER and py > CORNER_CENTER:
                xc, yc = -CORNER_CENTER, CORNER_CENTER
                dist = math.hypot(px-xc, py-yc)
                e = dist - RADIUS
                angle_to_center = math.atan2(py-yc, px-xc)
                psi = angle_to_center + math.pi/2.0
            elif px < -CORNER_CENTER and py < -CORNER_CENTER:
                xc, yc = -CORNER_CENTER, -CORNER_CENTER
                dist = math.hypot(px-xc, py-yc)
                e = dist - RADIUS
                angle_to_center = math.atan2(py-yc, px-xc)
                psi = angle_to_center + math.pi/2.0
            elif px > CORNER_CENTER and py < -CORNER_CENTER:
                xc, yc = CORNER_CENTER, -CORNER_CENTER
                dist = math.hypot(px-xc, py-yc)
                e = dist - RADIUS
                angle_to_center = math.atan2(py-yc, px-xc)
                psi = angle_to_center + math.pi/2.0
            elif py > CORNER_CENTER:
                e = py - d_path; psi = math.pi
            elif py < -CORNER_CENTER:
                e = -d_path - py; psi = 0.0
            elif px < -CORNER_CENTER:
                e = -d_path - px; psi = -math.pi/2.0
            elif px > CORNER_CENTER:
                e = px - d_path; psi = math.pi/2.0
            theta_e = math.atan2(math.sin(psi-yaw), math.cos(psi-yaw))
            return e, theta_e

        e_geo, theta_e = geometric_calc(front_x, front_y, yaw)

        # Stanley + roll correction
        delta_geo = theta_e + math.atan2(K_STANLEY * e_geo, v_linear + K_SOFT)
        roll_i += roll * DT
        delta_roll = -K_ROLL_P * roll - K_ROLL_D * roll_rate - K_ROLL_I * roll_i
        raw_delta = delta_geo + delta_roll
        # clamp +/-45deg
        max45 = math.radians(45.0)
        raw_delta = max(-max45, min(max45, raw_delta))

        # low-pass filtered + slew
        FILTER_ALPHA = 0.7
        DELTA_SLEW = 100.0
        filtered_delta = filtered_delta * (1.0 - FILTER_ALPHA) + raw_delta * FILTER_ALPHA
        max_delta_change = DELTA_SLEW * DT
        delta_change = filtered_delta - current_delta
        delta_change = max(-max_delta_change, min(max_delta_change, delta_change))
        current_delta += delta_change
        delta = current_delta

        # Ackermann -> left/right steer
        if abs(delta) > 1e-6:
            tan_delta = math.tan(delta)
            delta_l = math.atan2(2.0 * L * tan_delta, (2.0 * L - W * tan_delta))
            delta_r = math.atan2(2.0 * L * tan_delta, (2.0 * L + W * tan_delta))
        else:
            delta_l = 0.0; delta_r = 0.0

        # target speed logic (simple ramp towards 1.0 m/s)
        DESIRED_MAX = 1.0
        SAFE_FACTOR = 2.0
        expected_speed = DESIRED_MAX / (1.0 + SAFE_FACTOR * abs(delta))
        expected_speed = max(0.4, expected_speed)
        if target_speed < expected_speed:
            target_speed += 0.5 * DT
        else:
            target_speed -= 1.0 * DT
        target_speed = max(0.0, min(1.2, target_speed))

        # differential wheel speeds
        omega = target_speed * math.tan(delta) / L
        v_left = target_speed - (omega * W / 2.0)
        v_right = target_speed + (omega * W / 2.0)
        w_target_l = v_left / WHEEL_RADIUS
        w_target_r = v_right / WHEEL_RADIUS

        # PID for voltage
        def calc_voltage(target_w, cur_w, idx):
            err = target_w - cur_w
            voltage_i[idx] += err * DT
            # anti-windup
            voltage_i[idx] = max(-12.0, min(12.0, voltage_i[idx]))
            d_err = (err - last_error[idx]) / DT
            last_error[idx] = err
            voltage = KP * err + KI * voltage_i[idx] + KF * target_w + KD * d_err
            voltage = max(-BATTERY_VOLTAGE, min(BATTERY_VOLTAGE, voltage))
            return voltage

        volt_l = calc_voltage(w_target_l, w_act_l, 0)
        volt_r = calc_voltage(w_target_r, w_act_r, 1)

        # pack and send (firmware expects 4 uint16 in ID 0x180: volt_l, volt_r, delta_l, delta_r)
        def to_uint16_offset(v):
            return max(0, min(0xFFFF, int(v * 1000.0 + 0x8000)))

        combined_data = struct.pack('<HHHH', 
            to_uint16_offset(volt_l), 
            to_uint16_offset(volt_r), 
            to_uint16_offset(delta_l), 
            to_uint16_offset(delta_r)
        )
        send_can(0x180, combined_data)
        if _ctrl_count % 100 == 0:
            print(f"[OUT] volt=({volt_l:.3f},{volt_r:.3f}) delta=({math.degrees(delta_l):.2f}°,{math.degrees(delta_r):.2f}°) "
                  f"target_spd={target_speed:.3f} delta={math.degrees(delta):.2f}° roll={roll:.4f} roll_rate={roll_rate:.4f} roll_i={roll_i:.4f}")


# NOTE: EKF task removed — sensor data still accepted by can_receiver and stored in `sensor_data` for the controller.

# NOTE: Monitor task removed to keep script focused on control + CAN I/O.

# ---------- CAN 接收与分发 ----------
def can_receiver():
    """接收 CAN 消息并分发到对应队列"""
    print("CAN 接收线程启动")
    while running:
        try:
            msg = bus.recv(timeout=1.0)
            if msg is None:
                continue
            can_id = msg.arbitration_id
            data_bytes = msg.data
            
            # Determine interpretation: many frames use float32 (4 bytes) or uint16 (2 bytes)
            if len(data_bytes) >= 8:
                # New logic: 0x100-0x202 frames contain two float32 values
                try:
                    v1, v2 = struct.unpack('<ff', data_bytes[:8])
                    
                    # For the purpose of the motion_control_task, we'll just store them
                    # into the status arrays. Since 0x100 sends (x, y), 0x101 sends (z, qx), etc.
                    if 0x100 <= can_id <= 0x107:
                        base_idx = (can_id - 0x100) * 2
                        if base_idx + 1 < len(motion_status):
                            motion_status[base_idx] = v1
                            motion_status[base_idx + 1] = v2
                    elif 0x200 <= can_id <= 0x202:
                        base_idx = (can_id - 0x200) * 2
                        if base_idx + 1 < len(sensor_data):
                            sensor_data[base_idx] = v1
                            sensor_data[base_idx + 1] = v2
                    
                    # 记录帧序列
                    if 0x100 <= can_id <= 0x300:
                        with _frame_seq_lock:
                            _frame_seq.append(can_id)
                    
                    if can_id == 0x300:
                        # 收到同步帧，触发控制
                        motion_status[16] = v1  # dt
                        control_trigger.set()
                    
                    value = v1 # for logging
                except struct.error:
                    value = 0.0
            elif len(data_bytes) >= 4:
                # most telemetry frames are float32
                try:
                    value = struct.unpack('<f', data_bytes[:4])[0]
                except struct.error:
                    value = 0.0
            elif len(data_bytes) >= 2:
                # often control frames are uint16 (e.g., PWM or encoded values)
                value = struct.unpack('<H', data_bytes[:2])[0]
            else:
                value = 0.0

            # print(f"[RX] ID=0x{can_id:03X}, data={data_bytes.hex()}, value={value}")

            # 根据 ID 范围分发
            if 0x000 <= can_id <= 0x0FF:
                # 电源管理数据
                idx = can_id - 0x000
                if 0 <= idx < len(power_status):
                    power_status[idx] = value
            elif can_id == 0x700:
                # 环回测试：回复相同数据
                send_can(0x701, data_bytes)
            # 其他 ID 忽略
        except Exception as e:
            print(f"接收错误: {e}")

# ---------- 主程序 ----------
def main():
    global bus, running
    # 设置信号处理
    def signal_handler(sig, frame):
        global running
        print("\n收到退出信号，正在停止...")
        running = False
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # 初始化 CAN 总线
    try:
        bus = can.interface.Bus(channel=CAN_INTERFACE, bustype=CAN_BUSTYPE)
        print(f"CAN 接口 {CAN_INTERFACE} 已打开")
    except Exception as e:
        print(f"打开 CAN 接口失败: {e}")
        print("请确保 vcan0 已创建并启用：")
        print("  sudo ip link add dev vcan0 type vcan")
        print("  sudo ip link set up vcan0")
        sys.exit(1)

    # 创建并启动线程（仅保留 CAN 接收与运动控制）
    threads = [
        threading.Thread(target=can_receiver, name="CANReceiver"),
        threading.Thread(target=motion_control_task, name="MotionControl"),
    ]

    for t in threads:
        t.daemon = True
        t.start()

    print("所有任务已启动，按 Ctrl+C 退出")

    # 主线程等待
    try:
        while running:
            time.sleep(1)
    except KeyboardInterrupt:
        running = False
        print("\n程序退出")

if __name__ == "__main__":
    main()