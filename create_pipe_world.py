
import numpy as np
import os

def generate_pipe_obj(filename):
    """
    生成管道的3D网格模型(.obj文件)
    算法思路：
    1. 定义中心路径（直线段和圆弧段）。
    2. 沿着路径生成一系列截面圆环。
    3. 连接相邻截面圆环上的点，形成三角面或四边面。
    """
    # 參數设置
    # 根据小车尺寸(~0.5m)进行调整
    L_straight = 8.0  # 直线段长度
    R_corner = 1.6    # 拐角半径
    R_pipe = 0.8      # 管道半径
    N_circle = 32     # 管道截面圆的分段数（圆的平滑度）
    N_corner_steps = 16 # 拐角处的路径分段数
    N_straight_steps = 5 # 直线段的路径分段数（为了保持网格密度均匀）
    
    # 路径定义
    # S = L_straight / 2 = 1.0 (直线的半长)
    # R = R_corner = 1.5 (拐角半径)
    # 这里的路径是一个闭合的圆角矩形或者是类似的环形结构
    # 中心点大致在: (+/- 1.0, +/- 1.0)
    
    path_points = []   # 存储路径上的中心点坐标
    path_tangents = [] # 存储路径上每个点的切线方向（用于确定截面朝向）
    
    def add_corner(center, start_ang, end_ang):
        """
        添加圆弧拐角路径
        :param center: 圆弧中心坐标 (x, y)
        :param start_ang: 起始角度 (度)
        :param end_ang: 结束角度 (度)
        """
        # 生成角度序列，不包含终点，以免与下一段起点重合
        angles = np.linspace(np.radians(start_ang), np.radians(end_ang), N_corner_steps, endpoint=False)
        for ang in angles:
            # 计算圆弧上的点坐标
            x = center[0] + R_corner * np.cos(ang)
            y = center[1] + R_corner * np.sin(ang)
            path_points.append(np.array([x, y, 0]))
            
            # 计算切线方向: 
            # 位置导数 (cos, sin)' = (-sin, cos)
            # 方向取决于是在顺时针还是逆时针，这里假设角度增加方向(逆时针CCW)
            tx = -np.sin(ang)
            ty = np.cos(ang)
            path_tangents.append(np.array([tx, ty, 0]))

    def add_straight(start, end):
        """
        添加直线路径
        :param start: 起点坐标 (x, y)
        :param end: 终点坐标 (x, y)
        """
        # 确保坐标是3D的 (x, y, 0)
        if len(start) == 2:
            start = (*start, 0)
        if len(end) == 2:
            end = (*end, 0)
            
        # 我们不包含终点，以避免与下一段的起点重复
        # (除非是最后一段且没有手动闭合回路，但这里我们后面会手动闭合)
        vec = np.array(end) - np.array(start) # 向量
        length = np.linalg.norm(vec)          # 长度
        direction = vec / length              # 单位方向向量
        
        # 生成直线上的插值点
        steps = np.linspace(0, length, N_straight_steps, endpoint=False)
        for s in steps:
            p = np.array(start) + direction * s
            path_points.append(p)
            path_tangents.append(direction)

    # 下面按顺序构建闭合路径
    # 按照用户要求：圆心改为 (+-L_straight/2, +-L_straight/2)
    # 1. Corner BR: Center (L_straight/2, -L_straight/2)
    add_corner((L_straight/2, -L_straight/2), -90, 0)
    
    # 2. Straight R:
    add_straight((L_straight/2 + R_corner, -L_straight/2), (L_straight/2 + R_corner, L_straight/2))
    
    # 3. Corner TR: Center (L_straight/2, L_straight/2)
    add_corner((L_straight/2, L_straight/2), 0, 90)
    
    # 4. Straight T:
    add_straight((L_straight/2, L_straight/2 + R_corner), (-L_straight/2, L_straight/2 + R_corner))
    
    # 5. Corner TL: Center (-L_straight/2, L_straight/2)
    add_corner((-L_straight/2, L_straight/2), 90, 180)
    
    # 6. Straight L:
    add_straight((-L_straight/2 - R_corner, L_straight/2), (-L_straight/2 - R_corner, -L_straight/2))
    
    # 7. Corner BL: Center (-L_straight/2, -L_straight/2)
    add_corner((-L_straight/2, -L_straight/2), 180, 270)
    
    # 8. Straight B:
    add_straight((-L_straight/2, -L_straight/2 - R_corner), (L_straight/2, -L_straight/2 - R_corner))
    
    # 闭合回路：将第一个点（和切线）再次添加到末尾，确保首尾相接
    path_points.append(path_points[0])
    path_tangents.append(path_tangents[0])
    
    vertices = [] # 存储所有顶点的列表
    normals = []  # 存储所有顶点法线的列表
    
    # Generate vertices
    # 生成网格顶点
    for i, (P, T) in enumerate(zip(path_points, path_tangents)):
        # 构建局部坐标系帧 (Frenet Frame 的简化版)
        # Frame
        # T is tangent
        # B is Up (0, 0, 1)
        # N = B x T (Horizontal normal)
        B_frame = np.array([0, 0, 1])
        N_frame = np.cross(B_frame, T)
        N_frame = N_frame / np.linalg.norm(N_frame)
        
        # Generate circle
        # We want normals pointing INWARD.
        # Standard cylinder normals point outward (away from axis).
        # So we will use -Normal for lighting.
        # 生成当前截面上的圆形顶点
        
        for j in range(N_circle):
            phi = 2 * np.pi * j / N_circle
            # Circle in N_frame, B_frame plane
            # v = P + R * (cos(phi)*N_frame + sin(phi)*B_frame)
            
            cos_phi = np.cos(phi)
            sin_phi = np.sin(phi)
            
            v_offset = R_pipe * (cos_phi * N_frame + sin_phi * B_frame)
            v = P + v_offset
            vertices.append(v)
            
            # Normal pointing inward: - (cos*N + sin*B)
            # 法线方向 (指向圆心内部)
            n = -(cos_phi * N_frame + sin_phi * B_frame)
            normals.append(n)

    # Generate faces
    # Grid of size (NumPathSteps) x (N_circle)
    # i is path index, j is circle index
    # Vertices are stored linearly: index = i * N_circle + j
    
    faces = []
    num_path_steps = len(path_points) - 1
    
    for i in range(num_path_steps):
        for j in range(N_circle):
            # Current ring: i
            # Next ring: i+1
            # Current angle: j
            # Next angle: (j+1) % N_circle
            
            v1 = i * N_circle + j
            v2 = i * N_circle + (j + 1) % N_circle
            v3 = (i + 1) * N_circle + (j + 1) % N_circle
            v4 = (i + 1) * N_circle + j
            
            # OBJ indices are 1-based
            # We want inward facing normals.
            # Standard CCW (v1, v2, v3) points outward?
            # Let's check.
            # v1 is (i, j), v2 is (i, j+1). v1->v2 is along circle CCW.
            # v1->v4 is along path.
            # Cross product (CircleTangent) x (PathTangent) = Outward Normal.
            # So (v1, v2, v4) would be outward.
            # We want inward. So we should reverse winding.
            # (v1, v4, v2)
            
            # Quad: v1, v4, v3, v2
            faces.append([v1 + 1, v4 + 1, v3 + 1, v2 + 1])

    with open(filename, 'w') as f:
        f.write("# Pipe World OBJ\n")
        # 写入顶点 v x y z
        for v in vertices:
            f.write(f"v {v[0]:.4f} {v[1]:.4f} {v[2]:.4f}\n")
        # 写入法线 vn x y z
        for n in normals:
            f.write(f"vn {n[0]:.4f} {n[1]:.4f} {n[2]:.4f}\n")
        # 写入面 f v1//vn1 v2//vn2 ...
        for face in faces:
            # f v1//vn1 v2//vn2 ...
            # We assume vn index matches v index
            f.write(f"f {face[0]}//{face[0]} {face[1]}//{face[1]} {face[2]}//{face[2]} {face[3]}//{face[3]}\n")

def generate_sdf(obj_path, sdf_path):
    """
    生成 SDF (Simulation Description Format) 文件
    SDF用于在Gazebo仿真环境中描述世界、模型、光照等。
    """
    sdf_content = f"""<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="pipe_world">
    <!-- 物理引擎设置 -->
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size> <!-- 仿真步长 1ms -->
      <real_time_factor>1.0</real_time_factor> <!-- 实时因子，1.0表示并通过真实时间流逝 -->
    </physics>
    
    <!-- 必要的插件 -->
    <plugin
      filename="ignition-gazebo-physics-system"
      name="ignition::gazebo::systems::Physics">
    </plugin>
    <plugin
      filename="ignition-gazebo-user-commands-system"
      name="ignition::gazebo::systems::UserCommands">
    </plugin>
    <plugin
      filename="ignition-gazebo-scene-broadcaster-system"
      name="ignition::gazebo::systems::SceneBroadcaster">
    </plugin>

    <!-- 光照设置 (太阳光) -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- 管道模型 -->
    <model name="pipe_track">
      <static>true</static> <!-- 静态模型，不受重力影响，不动 -->
      <pose>0 0 0 0 0 0</pose>
      <link name="link">
        <!-- 碰撞几何体 (Collision) - 用于物理计算 -->
        <collision name="collision">
          <geometry>
            <mesh>
              <uri>{obj_path}</uri> <!-- 引用生成的 OBJ 文件 -->
              <scale>1 1 1</scale>
            </mesh>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>100.0</mu> <!-- 摩擦系数 -->
                <mu2>100.0</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <!-- 可视化几何体 (Visual) - 用于渲染 -->
        <visual name="visual">
          <geometry>
            <mesh>
              <uri>{obj_path}</uri>
              <scale>1 1 1</scale>
            </mesh>
          </geometry>
          <material>
            <ambient>0.6 0.6 0.6 1</ambient>
            <diffuse>0.6 0.6 0.6 1</diffuse>
            <specular>0.5 0.5 0.5 1</specular>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
"""
    with open(sdf_path, 'w') as f:
        f.write(sdf_content)

if __name__ == "__main__":
    # 设置输出目录
    # 注意：这里我们使用硬编码路径，假设用户通过 robot 工作区工作
    # 但為了更健壯，最好使用脚本所在位置
    
    # 原始路径: base_dir = "/home/yinyudream/Desktop/final_design/src/simple_car_sim/worlds"
    # 如果不存在則修正為当前脚本子目录
    script_dir = os.path.dirname(os.path.abspath(__file__))
    potential_dir = base_dir = "/home/yinyudream/Desktop/robot/src/simple_car_sim/worlds"
    
    if not os.path.exists(os.path.dirname(base_dir)): # 检查上级目录
         base_dir = os.path.join(script_dir, "src/simple_car_sim/worlds")

    if not os.path.exists(base_dir):
        try:
            os.makedirs(base_dir)
        except:
             base_dir = script_dir # 最后回退

    obj_filename = os.path.join(base_dir, "pipe.obj")
    sdf_filename = os.path.join(base_dir, "pipe_world.sdf")
    
    print(f"Generating {obj_filename}...")
    generate_pipe_obj(obj_filename)
    print(f"Generating {sdf_filename}...")
    generate_sdf(f"file://{obj_filename}", sdf_filename)
    print("Done.")
