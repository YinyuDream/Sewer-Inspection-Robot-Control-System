#!/usr/bin/env python3
"""
HIL Stress Test & Performance Monitoring Script
用于测试 STM32 固件在极高并发总线负载下的 RTOS 任务调度稳定性和丢包率。
测试数据将自动保存为 CSV，用于论文 4.2 章节的图表绘制。
"""

import can
import time
import struct
import csv
import threading
import os
import signal
import sys
import math

# 【配置项】
# 如果是真车联调请换成 'can0'；若是虚拟仿真测试使用 'vcan0'
CAN_INTERFACE = 'can0'
TEST_DURATION = 15  # 测试持续时间(秒)

print(f"======================================================")
print(f"🚀 Starting HIL Stress Test on {CAN_INTERFACE}")
print(f"⏱️  Duration: {TEST_DURATION} seconds")
print(f"======================================================")

try:
    # 更新 API 以匹配 python-can 4.x+ 的用法，解决 DeprecationWarning
    bus = can.interface.Bus(channel=CAN_INTERFACE, interface='socketcan')
except OSError:
    print(f"❌ Error: Could not open {CAN_INTERFACE}. Make sure it is set up.")
    print("If testing locally, run: sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0")
    exit(1)

# 数据存储
results_time = []
results_exec_ms = []
stack_watermarks = {
    "MotionControl": 0,
    "CanData": 0,
    "EkfAlgo": 0,
    "Power": 0
}

# 环回测试数据存储
ping_sent_times = {}  # 存储发送序号和时间戳 {seq_num: send_time}
ping_received_times = []  # 存储接收延迟数据 [(seq_num, latency_ms)]
echo_received = 0

start_time = time.time()
seq_num = 0
running = True
now_time = 0

def rx_thread_loop():
    """ 独立接收线程：监听性能数据和心跳环回 """
    global echo_received, running, ping_received_times
    while running:
        msg = bus.recv(0.1) # 100ms timeout
        if msg:
            if msg.arbitration_id == 0x701:
                # 收到心跳回环包
                echo_received += 1
                # 从数据中提取序号（前2字节，小端序）
                if len(msg.data) >= 2:
                    seq_short = struct.unpack('<H', bytes(msg.data[:2]))[0]
                    receive_time = time.time()
                    # 查找对应的发送时间
                    if seq_short in ping_sent_times:
                        latency = (receive_time - ping_sent_times[seq_short]) * 1000  # 转换为毫秒
                        ping_received_times.append((seq_short, latency))
                        # 移除已处理的发送时间，防止内存泄漏
                        del ping_sent_times[seq_short]
                    else:
                        # 可能序号已处理过或序号回绕，静默忽略
                        pass
            elif msg.arbitration_id == 0x400:
                # 收到运动控制算法执行耗时性能数据（单位：毫秒）
                exec_time = struct.unpack('<f', bytes(msg.data[:4]))[0]
                results_time.append(time.time() - start_time)
                results_exec_ms.append(exec_time)
            elif msg.arbitration_id == 0x500:
                stack_watermarks["MotionControl"] = struct.unpack('<f', bytes(msg.data[:4]))[0]
            elif msg.arbitration_id == 0x501:
                stack_watermarks["CanData"] = struct.unpack('<f', bytes(msg.data[:4]))[0]
            elif msg.arbitration_id == 0x502:
                stack_watermarks["EkfAlgo"] = struct.unpack('<f', bytes(msg.data[:4]))[0]
            elif msg.arbitration_id == 0x503:
                stack_watermarks["Power"] = struct.unpack('<f', bytes(msg.data[:4]))[0]
            elif msg.arbitration_id == 0x301:
                print(time.time() - now_time, struct.unpack('<I', bytes(msg.data[:4]))[0])

# 启动接收线程
rx_thread = threading.Thread(target=rx_thread_loop)
rx_thread.start()

print("⚡ Injecting high-frequency traffic...")

def signal_handler(sig, frame):
    global running
    print("\n⚠️ Test interrupted by user.")
    running = False
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

cnt = 0

try:
    while time.time() - start_time < TEST_DURATION:
        cnt += 1
        now_time = time.time()
        # 严格对照 can_transceiver.py 的发送逻辑和顺序
        # 位置和姿态数据 (0x100-0x107)
        bus.send(can.Message(arbitration_id=0x100, data=struct.pack('<ff', 1.0, 2.0), is_extended_id=False))
        bus.send(can.Message(arbitration_id=0x101, data=struct.pack('<ff', 3.0, 4.0), is_extended_id=False))
        bus.send(can.Message(arbitration_id=0x102, data=struct.pack('<ff', 5.0, 6.0), is_extended_id=False))
        bus.send(can.Message(arbitration_id=0x103, data=struct.pack('<ff', 7.0, 8.0), is_extended_id=False))
        bus.send(can.Message(arbitration_id=0x104, data=struct.pack('<ff', 9.0, 10.0), is_extended_id=False))
        bus.send(can.Message(arbitration_id=0x105, data=struct.pack('<ff', 11.0, 12.0), is_extended_id=False))
        bus.send(can.Message(arbitration_id=0x106, data=struct.pack('<ff', 13.0, 0.0), is_extended_id=False))
        bus.send(can.Message(arbitration_id=0x107, data=struct.pack('<ff', 14.0, 15.0), is_extended_id=False))
        
        # IMU 数据 (0x200-0x202)
        bus.send(can.Message(arbitration_id=0x200, data=struct.pack('<ff', 16.0, 17.0), is_extended_id=False))
        bus.send(can.Message(arbitration_id=0x201, data=struct.pack('<ff', 18.0, 19.0), is_extended_id=False))
        bus.send(can.Message(arbitration_id=0x202, data=struct.pack('<ff', 20.0, 21.0), is_extended_id=False))
        
        # 同步帧 0x300（包含 dt 和 0.0），最后发送以触发固件任务执行
        dt = 0.02  # 模拟 20ms 间隔
        bus.send(can.Message(arbitration_id=0x300, data=struct.pack('<fI', dt, cnt), is_extended_id=False))
        
        # 环回丢包测试 (Ping-Pong Packet) 
        # 发送2字节序号（小端序），确保15秒测试有足够唯一序号
        seq_short = seq_num % 65536  # 2字节范围
        # 记录发送时间
        ping_sent_times[seq_short] = time.time()
        # 打包2字节序号到前2个字节，后6字节补零，总共8字节
        seq_data = struct.pack('<H', seq_short) + b'\x00' * 6
        bus.send(can.Message(arbitration_id=0x700, data=seq_data, is_extended_id=False))
        
        seq_num += 1
        
        # 发送间隔设为 0.02s (50Hz)，在高频压测与系统稳定性间取得平衡
        time.sleep(0.01)
        
except can.CanError as e:
    # 针对 Error Code 105: No buffer space available 的优雅处理
    if "No buffer space available" in str(e):
        print("⚠️  Warning: CAN transmit buffer full, wait and retry...")
        time.sleep(0.1) 
    else:
        print(f"❌ CAN Transmit Error: {e}")
except Exception as e:
    print(f"❌ Exception during test: {e}")

running = False
rx_thread.join()

# ----- 统计与输出 -----
lost_packets = seq_num - echo_received
drop_rate = (lost_packets / seq_num) * 100 if seq_num > 0 else 0
avg_exec_time = sum(results_exec_ms) / len(results_exec_ms) if results_exec_ms else 0
max_exec_time = max(results_exec_ms) if results_exec_ms else 0

# 延迟统计
latencies = [latency for _, latency in ping_received_times]
if latencies:
    avg_latency = sum(latencies) / len(latencies)
    min_latency = min(latencies)
    max_latency = max(latencies)
    # 计算标准差
    variance = sum((x - avg_latency) ** 2 for x in latencies) / len(latencies)
    std_latency = math.sqrt(variance)
    # 计算抖动（延迟变化），使用相邻延迟的差值绝对值的平均值
    jitter = 0
    if len(latencies) > 1:
        jitter = sum(abs(latencies[i] - latencies[i-1]) for i in range(1, len(latencies))) / (len(latencies) - 1)
    # 统计超过特定阈值的延迟
    latency_threshold_ms = 10.0  # 10ms 阈值
    high_latency_count = sum(1 for x in latencies if x > latency_threshold_ms)
    high_latency_percent = (high_latency_count / len(latencies)) * 100 if latencies else 0
else:
    avg_latency = min_latency = max_latency = std_latency = jitter = 0
    high_latency_percent = 0

# 清理未匹配的发送时间（丢包）
unmatched_packets = len(ping_sent_times)
if unmatched_packets > 0:
    print(f"⚠️  Warning: {unmatched_packets} packets were sent but no echo received (potential lost packets)")
    # 显示丢包序列号范围
    if unmatched_packets <= 20:
        lost_sequences = sorted(ping_sent_times.keys())
        print(f"   Lost sequence numbers: {lost_sequences}")
    else:
        lost_min = min(ping_sent_times.keys())
        lost_max = max(ping_sent_times.keys())
        print(f"   Lost sequence range: {lost_min} to {lost_max}")

print("\n================ TEST COMPLETED ================")
print(f"📨 Total Ping Messages Sent: {seq_num}")
print(f"📥 Total Echo Received:      {echo_received}")
print(f"📉 Packet Drop Rate:         {drop_rate:.2f}%")
print(f"⏱️  Motion Control Avg Time:  {avg_exec_time:.2f} ms")
print(f"📈 Motion Control Max Time:  {max_exec_time:.2f} ms (Jitter/Spike)")
print("\n=========== PING-PONG LATENCY STATS ===========")
print(f"📊 Average Latency:         {avg_latency:.2f} ms")
print(f"📉 Minimum Latency:         {min_latency:.2f} ms")
print(f"📈 Maximum Latency:         {max_latency:.2f} ms")
print(f"📐 Latency Std Dev:         {std_latency:.2f} ms")
print(f"📊 Latency Jitter:          {jitter:.2f} ms")
print(f"⚠️  High Latency (>10ms):    {high_latency_percent:.1f}%")
print("\n========== RTOS TASK STACK WATERMARKS ==========")
print("(Words remaining. Closer to 0 means higher overflow risk)")
print(f"🛠️  Motion Control Task: {int(stack_watermarks['MotionControl'])} words")
print(f"📡  CAN Data Task:       {int(stack_watermarks['CanData'])} words")
print(f"🧮  EKF Algorithm Task:  {int(stack_watermarks['EkfAlgo'])} words")
print(f"🔋  Power Mgmt Task:     {int(stack_watermarks['Power'])} words")
print("================================================\n")

# 保存结果到 CSV 供后续画图分析
# 1. 运动控制执行时间数据
output_csv = os.path.join(os.path.dirname(__file__), 'hil_stress_results.csv')
with open(output_csv, 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(['timestamp_s', 'exec_time_ms'])
    for t, e in zip(results_time, results_exec_ms):
        writer.writerow([round(t, 4), round(e, 2)])
        
# 2. 延迟数据
if ping_received_times:
    latency_csv = os.path.join(os.path.dirname(__file__), 'hil_stress_latency.csv')
    with open(latency_csv, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['sequence_num', 'latency_ms'])
        for seq, latency in ping_received_times:
            writer.writerow([seq, round(latency, 2)])
    print(f"💾 Latency results saved successfully to:\n   {latency_csv}")
        
print(f"💾 Motion control results saved successfully to:\n   {output_csv}")