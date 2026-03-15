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

start_time = time.time()
seq_num = 0
echo_received = 0
running = True

def rx_thread_loop():
    """ 独立接收线程：监听性能数据和心跳环回 """
    global echo_received, running
    while running:
        msg = bus.recv(0.1) # 100ms timeout
        if msg:
            if msg.arbitration_id == 0x701:
                # 收到心跳回环包
                echo_received += 1
            elif msg.arbitration_id == 0x400:
                # 收到运动控制算法执行耗时性能数据
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

try:
    while time.time() - start_time < TEST_DURATION:
        # 1. 触发运动控制负载 (100Hz+)
        data_f = struct.pack('<f', 1.0) # 伪造的浮动指令数据
        bus.send(can.Message(arbitration_id=0x100, data=data_f, is_extended_id=False))
        
        # 2. 强插 EKF 任务负载，占用 CPU Cycle
        bus.send(can.Message(arbitration_id=0x200, data=data_f, is_extended_id=False))
        
        # 3. 环回丢包测试 (Ping-Pong Packet) 
        # 只发送前1个字节作为序号，其余留空
        seq_byte = seq_num % 256
        bus.send(can.Message(arbitration_id=0x700, data=[seq_byte, 0, 0, 0], is_extended_id=False))
        
        seq_num += 1
        
        # 发送间隔设为 0.02s (50Hz)，在高频压测与系统稳定性间取得平衡
        time.sleep(0.02)
        
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

print("\n================ TEST COMPLETED ================")
print(f"📨 Total Ping Messages Sent: {seq_num}")
print(f"📥 Total Echo Received:      {echo_received}")
print(f"📉 Packet Drop Rate:         {drop_rate:.2f}%")
print(f"⏱️  Motion Control Avg Time:  {avg_exec_time:.2f} ms")
print(f"📈 Motion Control Max Time:  {max_exec_time:.2f} ms (Jitter/Spike)")
print("\n========== RTOS TASK STACK WATERMARKS ==========")
print("(Words remaining. Closer to 0 means higher overflow risk)")
print(f"🛠️  Motion Control Task: {int(stack_watermarks['MotionControl'])} words")
print(f"📡  CAN Data Task:       {int(stack_watermarks['CanData'])} words")
print(f"🧮  EKF Algorithm Task:  {int(stack_watermarks['EkfAlgo'])} words")
print(f"🔋  Power Mgmt Task:     {int(stack_watermarks['Power'])} words")
print("================================================\n")

# 保存结果到 CSV 供后续画图分析
output_csv = os.path.join(os.path.dirname(__file__), 'hil_stress_results.csv')
with open(output_csv, 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(['timestamp_s', 'exec_time_ms'])
    for t, e in zip(results_time, results_exec_ms):
        writer.writerow([round(t, 4), round(e, 2)])
        
print(f"💾 Results saved successfully to:\n   {output_csv}")