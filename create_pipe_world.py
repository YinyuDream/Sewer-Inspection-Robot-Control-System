
import numpy as np
import os

def generate_pipe_obj(filename):
    # Parameters
    # Adjusted for car size ~0.5m
    L_straight = 2.0
    R_corner = 1.5
    R_pipe = 0.8
    N_circle = 32
    N_corner_steps = 16
    N_straight_steps = 5 # A few steps for straights just in case
    
    # Path definition
    # S = L_straight / 2 = 1.0
    # R = R_corner = 1.5
    # Centers: (+/- 1.0, +/- 1.0)
    
    path_points = []
    path_tangents = []
    
    def add_corner(center, start_ang, end_ang):
        angles = np.linspace(np.radians(start_ang), np.radians(end_ang), N_corner_steps, endpoint=False)
        for ang in angles:
            x = center[0] + R_corner * np.cos(ang)
            y = center[1] + R_corner * np.sin(ang)
            path_points.append(np.array([x, y, 0]))
            # Tangent: derivative of (cos, sin) is (-sin, cos)
            # Direction depends on CCW. increasing angle is CCW.
            tx = -np.sin(ang)
            ty = np.cos(ang)
            path_tangents.append(np.array([tx, ty, 0]))

    def add_straight(start, end):
        # Ensure 3D
        if len(start) == 2:
            start = (*start, 0)
        if len(end) == 2:
            end = (*end, 0)
            
        # We don't include the end point to avoid duplication with next segment start
        # except for the very last segment if we weren't closing the loop manually
        vec = np.array(end) - np.array(start)
        length = np.linalg.norm(vec)
        direction = vec / length
        
        steps = np.linspace(0, length, N_straight_steps, endpoint=False)
        for s in steps:
            p = np.array(start) + direction * s
            path_points.append(p)
            path_tangents.append(direction)

    # 1. Corner BR: Center (1.0, -1.0)
    add_corner((1.0, -1.0), -90, 0)
    # 2. Straight R: (2.5, -1.0) -> (2.5, 1.0)
    add_straight((2.5, -1.0), (2.5, 1.0))
    # 3. Corner TR: Center (1.0, 1.0)
    add_corner((1.0, 1.0), 0, 90)
    # 4. Straight T: (1.0, 2.5) -> (-1.0, 2.5)
    add_straight((1.0, 2.5), (-1.0, 2.5))
    # 5. Corner TL: Center (-1.0, 1.0)
    add_corner((-1.0, 1.0), 90, 180)
    # 6. Straight L: (-2.5, 1.0) -> (-2.5, -1.0)
    add_straight((-2.5, 1.0), (-2.5, -1.0))
    # 7. Corner BL: Center (-1.0, -1.0)
    add_corner((-1.0, -1.0), 180, 270)
    # 8. Straight B: (-1.0, -2.5) -> (1.0, -2.5)
    add_straight((-1.0, -2.5), (1.0, -2.5))
    
    # Close the loop by adding the first point at the end
    path_points.append(path_points[0])
    path_tangents.append(path_tangents[0])
    
    vertices = []
    normals = [] # Vertex normals
    
    # Generate vertices
    for i, (P, T) in enumerate(zip(path_points, path_tangents)):
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
        for v in vertices:
            f.write(f"v {v[0]:.4f} {v[1]:.4f} {v[2]:.4f}\n")
        for n in normals:
            f.write(f"vn {n[0]:.4f} {n[1]:.4f} {n[2]:.4f}\n")
        for face in faces:
            # f v1//vn1 v2//vn2 ...
            # We assume vn index matches v index
            f.write(f"f {face[0]}//{face[0]} {face[1]}//{face[1]} {face[2]}//{face[2]} {face[3]}//{face[3]}\n")

def generate_sdf(obj_path, sdf_path):
    sdf_content = f"""<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="pipe_world">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
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

    <model name="pipe_track">
      <static>true</static>
      <pose>0 0 0 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <mesh>
              <uri>{obj_path}</uri>
              <scale>1 1 1</scale>
            </mesh>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>100.0</mu>
                <mu2>100.0</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
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
    base_dir = "/home/yinyudream/Desktop/final_design/src/simple_car_sim/worlds"
    if not os.path.exists(base_dir):
        os.makedirs(base_dir)
        
    obj_filename = os.path.join(base_dir, "pipe.obj")
    sdf_filename = os.path.join(base_dir, "pipe_world.sdf")
    
    print(f"Generating {obj_filename}...")
    generate_pipe_obj(obj_filename)
    print(f"Generating {sdf_filename}...")
    generate_sdf(obj_filename, sdf_filename)
    print("Done.")
