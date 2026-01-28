import math
import os
import re

def calculate_speed_ratio(D, R, W, L, r):
    """
    计算履带式管道机器人在 45度 姿态角下的过弯速度比。
    基于论文公式 (3), (12), (13), (14)。
    
    参数:
    D : 管道内径 (mm)
    R : 弯管曲率半径 (mm)
    W : 履带宽度 (mm)
    L : 履带长度 (mm)
    r : 机器人本体半径 (mm)
    
    返回:
    ratio : 速度比 (外侧速度 / 内侧速度)
    details : 计算过程中的中间变量 (字典)，如果干涉则返回 None
    """
    
    # 1. 基础常数转换
    angle_45 = math.radians(45)  # 将45度转换为弧度
    
    # 2. 计算模长 M (公式 3)
    # 机器人中心到履带边缘角的距离
    M = math.sqrt(r**2 + (0.5 * W)**2)
    
    # 3. 计算角度 alpha (公式 3)
    # 履带角相对于中心的偏角
    alpha = math.atan((0.5 * W) / r)
    
    # 4. 计算位移量 s (公式 3)
    # 判断是否干涉的核心步骤
    # s 公式中有根号: sqrt((0.5D)^2 - (M * sin(45 - alpha))^2)
    
    sin_val_minus = math.sin(angle_45 - alpha)
    cos_val_minus = math.cos(angle_45 - alpha)
    
    term_inside_sqrt = (0.5 * D)**2 - (M * sin_val_minus)**2
    
    # --- 干涉检测 ---
    if term_inside_sqrt < 0:
        return None, "干涉警告: 机器人半径过大，无法以45度姿态进入该弯管。"
    
    s = math.sqrt(term_inside_sqrt) - M * cos_val_minus
    
    # 5. 计算内外侧路径半径 (公式 13 & 14)
    sin_val_plus = math.sin(angle_45 + alpha)
    
    # d_Ob: 内侧（慢速侧）接触点到弯管中心的距离
    d_Ob = R + D - s - M * sin_val_plus
    
    # d_Og: 外侧（快速侧）接触点到弯管中心的距离
    # 注意：d_Og 是空间距离，包含履带长度分量
    vertical_term = 0.5 * D + R - s + M * sin_val_plus
    d_Og = math.sqrt((0.5 * L)**2 + vertical_term**2)
    
    # 6. 计算速度比 (公式 12)
    if d_Ob <= 0:
        return None, "错误: 计算出的内侧半径无效 (<=0)。"
        
    ratio = d_Og / d_Ob
    
    details = {
        "M": M,
        "alpha_deg": math.degrees(alpha),
        "s": s,
        "d_Og": d_Og,
        "d_Ob": d_Ob
    }
    
    return ratio, details

def get_auto_params():
    """从 create_pipe_world.py 和 robot.urdf.xacro 自动读取参数"""
    print("正在尝试从 create_pipe_world.py 和 robot.urdf.xacro 读取参数...")
    
    params = {}
    
    # Path to files
    pipe_world_path = os.path.join(os.getcwd(), 'create_pipe_world.py')
    robot_urdf_path = os.path.join(os.getcwd(), 'src/simple_car_sim/urdf/robot.urdf.xacro')
    
    # Parse create_pipe_world.py
    if not os.path.exists(pipe_world_path):
        print(f"Error: {pipe_world_path} not found.")
        return None
        
    with open(pipe_world_path, 'r', encoding='utf-8') as f:
        content = f.read()
        r_corner_match = re.search(r'R_corner\s*=\s*([\d\.]+)', content)
        r_pipe_match = re.search(r'R_pipe\s*=\s*([\d\.]+)', content)
        
        if r_corner_match: params['R_corner'] = float(r_corner_match.group(1))
        if r_pipe_match: params['R_pipe'] = float(r_pipe_match.group(1))

    # Parse robot.urdf.xacro
    if not os.path.exists(robot_urdf_path):
        print(f"Error: {robot_urdf_path} not found.")
        return None

    with open(robot_urdf_path, 'r', encoding='utf-8') as f:
        content = f.read()
        def get_xacro_val(name):
             match = re.search(r'<xacro:property\s+name="' + name + r'"\s+value="([\d\.]+)"', content)
             return float(match.group(1)) if match else None
             
        params['wheel_width'] = get_xacro_val('wheel_width')
        params['wheel_interval'] = get_xacro_val('wheel_interval')
        params['wheel_radius'] = get_xacro_val('wheel_radius')
        params['wheel_offset'] = get_xacro_val('wheel_offset')

    # Validate
    required = ['R_corner', 'R_pipe', 'wheel_width', 'wheel_interval', 'wheel_radius', 'wheel_offset']
    if any(k not in params or params[k] is None for k in required):
        print("Error: 缺少必要参数")
        return None

    print("-" * 30)
    print(f"读取到的参数 (m):")
    for k, v in params.items():
        print(f"  {k}: {v}")

    # Convert to mm
    try:
        D_in = params['R_pipe'] * 2 * 1000
        R_in = params['R_corner'] * 1000
        W_in = params['wheel_width'] * 1000
        L_in = (params['wheel_interval'] + params['wheel_radius']) * 2 * 1000
        # Formula: r = wheel_offset * Sqrt(2) + wheel_radius + 0.1
        # Assuming 0.1 is in meters
        r_in = (params['wheel_offset'] * math.sqrt(2) + params['wheel_radius'] + 0.1) * 1000
        
        return D_in, R_in, W_in, L_in, r_in
    except Exception as e:
        print(f"参数计算错误: {e}")
        return None

# --- 主程序入口 ---
if __name__ == "__main__":
    print("=== 管道机器人 45度过弯 速度比计算器 ===")
    
    try:
        # 尝试自动获取
        auto_params = get_auto_params()
        
        if auto_params:
            D_in, R_in, W_in, L_in, r_in = auto_params
            print(f"自动获取并计算的参数 (mm): ")
            print(f"  管道直径 D: {D_in:.2f}")
            print(f"  弯管半径 R: {R_in:.2f}")
            print(f"  履带宽度 W: {W_in:.2f}")
            print(f"  履带长度 L: {L_in:.2f}")
            print(f"  机器人半径 r: {r_in:.2f}")
        else:
            print("自动获取失败，请手动输入 (单位: mm):")
            D_in = float(input("管道直径 D = R_pipe * 2: "))
            R_in = float(input("弯管半径 R = R_corner: "))
            W_in = float(input("履带宽度 W = wheel_width: "))
            L_in = float(input("履带长度 L = (wheel_interval + wheel_radius) * 2: "))
            r_in = float(input("机器人半径 r = wheel_offset * Sqrt(2) + wheel_radius + 0.1: "))
        
        print("-" * 30)
        
        # 调用计算
        ratio, info = calculate_speed_ratio(D_in, R_in, W_in, L_in, r_in)
        
        if ratio is None:
            print(f"FAILED: {info}")
        else:
            print(f"计算成功!")
            print(f"位移补偿量 s: {info['s']:.2f} mm")
            print(f"外侧路径半径 d_Og: {info['d_Og']:.2f} mm")
            print(f"内侧路径半径 d_Ob: {info['d_Ob']:.2f} mm")
            print("-" * 30)
            print(f"★ 推荐速度比 (外 : 内) = {ratio:.4f} : 1")
            print("-" * 30)
            
            # 简单的控制指令建议
            base_speed = 0.1 # 假设基准速度 m/s
            print(f"示例控制指令 (基准速度 {base_speed} m/s):")
            print(f"  内侧履带速度: {base_speed:.3f} m/s")
            print(f"  外侧履带速度: {base_speed * ratio:.3f} m/s")

    except ValueError:
        print("输入错误: 请确保输入的是数字。")