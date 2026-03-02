#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CAN总线数据发送脚本 - 基础版
支持SocketCAN（Linux）和PCAN接口
"""

import can
import time
import random

def send_single_message(bus, can_id, data, is_extended=False):
    """
    发送单个CAN消息
    
    参数:
        bus: CAN总线实例
        can_id: CAN ID
        data: 数据列表（最多8字节）
        is_extended: 是否使用扩展帧
    """
    try:
        # 创建CAN消息
        msg = can.Message(
            arbitration_id=can_id,
            data=data,
            is_extended_id=is_extended,
            is_remote_frame=False
        )
        
        # 发送消息
        bus.send(msg)
        print(f"发送成功: ID=0x{can_id:X} Data={data}")
        return True
        
    except can.CanError as e:
        print(f"发送失败: {e}")
        return False

def send_periodic_messages(bus, can_id, interval=0.1, count=10):
    """
    周期性发送CAN消息
    
    参数:
        bus: CAN总线实例
        can_id: CAN ID
        interval: 发送间隔（秒）
        count: 发送次数
    """
    try:
        for i in range(count):
            # 生成递增数据
            data = [i % 256, (i+1) % 256, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
            
            msg = can.Message(
                arbitration_id=can_id,
                data=data,
                is_extended_id=False
            )
            
            bus.send(msg)
            print(f"[{i+1}/{count}] 发送: ID=0x{can_id:X} Data={data}")
            
            time.sleep(interval)
            
        return True
        
    except can.CanError as e:
        print(f"发送失败: {e}")
        return False

def main():
    # 非参数式配置：直接在代码中定义参数
    # Windows下常用接口: 'pcan', 'ixxat', 'vector', 'serial' (slcan), 'canalystii' (周立功/创芯科技)
    # Linux下常用接口: 'socketcan'
    interface = 'socketcan'  # Linux 默认使用 socketcan
    channel = 'can0'         # socketcan 通道通常为 can0
    bitrate = 1000000        # 比特率 (1M)
    id_arg = '123'           # CAN ID (十六进制字符串)
    data_arg = '01 02 03 04' # 要发送的数据 (十六进制字节，空格分隔)
    periodic = True          # 是否周期性发送
    count = 1000000          # 周期性发送次数
    interval = 2.5           # 发送间隔（秒）
    
    # 配置总线
    bus_config = {
        'interface': interface,
        'channel': channel,
        'bitrate': bitrate
    }
    
    try:
        # 创建CAN总线实例
        bus = can.Bus(**bus_config)
        print(f"CAN总线初始化成功: {interface} - {channel}")
        
        # 解析CAN ID
        can_id = int(id_arg, 16)
        
        # 解析数据
        data_bytes = [int(x, 16) for x in data_arg.split()]
        
        if periodic:
            # 周期性发送
            send_periodic_messages(bus, can_id, interval, count)
        else:
            # 单次发送
            send_single_message(bus, can_id, data_bytes)
            
    except Exception as e:
        print(f"错误: {e}")
    finally:
        if 'bus' in locals():
            bus.shutdown()
            print("CAN总线已关闭")

if __name__ == "__main__":
    main()