#!/usr/bin/env python3
"""
map2world.py — 把 ROS 2 栅格地图 PNG 转成 Gazebo .world 文件

用法：
    python3 map2world.py

输出：
    Btraj/worlds/maze.world   （直接放进项目目录）

依赖：
    pip install pillow numpy   （或 pip3 install pillow numpy）
"""

import os
import numpy as np
from PIL import Image

# ── 配置 ─────────────────────────────────────────────────────────────────────
MAP_PNG        = "Btraj/maps/map_maze.png"   # 相对于项目根目录
OUTPUT_WORLD   = "Btraj/worlds/maze.world"
RESOLUTION     = 1.0    # 每格多少米（和 map.yaml 一致）
WALL_HEIGHT    = 1.5    # 障碍物高度（米）
WALL_THICKNESS = 0.95   # 稍小于 resolution，避免重叠（米）
# ─────────────────────────────────────────────────────────────────────────────


def png_to_obstacles(png_path: str, resolution: float):
    """返回障碍物中心坐标列表 [(x, y), ...]，使用 ROS 地图坐标系"""
    img = Image.open(png_path).convert("RGB")
    arr = np.array(img)
    height, width = arr.shape[:2]

    obstacles = []
    for row in range(height):
        for col in range(width):
            if arr[row, col, 0] < 128:          # 黑色 = 障碍物
                # PNG row 0 = 地图最上方，对应 map_y 最大值
                map_x = col * resolution + resolution / 2.0
                map_y = (height - 1 - row) * resolution + resolution / 2.0
                obstacles.append((map_x, map_y))

    return obstacles, width, height


def make_box_sdf(idx: int, x: float, y: float,
                 size: float, height: float) -> str:
    """生成单个方块的 SDF XML 片段"""
    z = height / 2.0
    return f"""
    <model name="wall_{idx}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>{size:.3f} {size:.3f} {height:.3f}</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>{size:.3f} {size:.3f} {height:.3f}</size></box>
          </geometry>
          <material>
            <ambient>0.2 0.2 0.2 1</ambient>
            <diffuse>0.3 0.3 0.3 1</diffuse>
          </material>
        </visual>
      </link>
    </model>"""


def generate_world(obstacles, map_w, map_h, resolution,
                   wall_height, wall_thickness) -> str:
    """生成完整的 Gazebo world 文件内容"""

    boxes = "\n".join(
        make_box_sdf(i, x, y, wall_thickness, wall_height)
        for i, (x, y) in enumerate(obstacles)
    )

    # 地面平面大小和地图一致
    ground_size_x = map_w * resolution
    ground_size_y = map_h * resolution
    ground_cx     = ground_size_x / 2.0
    ground_cy     = ground_size_y / 2.0

    return f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="maze_world">

    <!-- 光源 -->
    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- 物理引擎 -->
    <physics name="default_physics" default="0" type="ode">
      <real_time_update_rate>1000.0</real_time_update_rate>
      <max_step_size>0.001</max_step_size>
    </physics>

    <!-- 地面（覆盖整个地图范围） -->
    <model name="ground_plane">
      <static>true</static>
      <pose>{ground_cx:.1f} {ground_cy:.1f} 0 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>{ground_size_x:.1f} {ground_size_y:.1f}</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>{ground_size_x:.1f} {ground_size_y:.1f}</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- 障碍物方块（共 {len(obstacles)} 个，由 map_maze.png 生成） -->
{boxes}

  </world>
</sdf>
"""


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    # 脚本放在项目根目录时使用相对路径；否则用绝对路径
    base = script_dir

    png_path    = os.path.join(base, MAP_PNG)
    output_path = os.path.join(base, OUTPUT_WORLD)

    print(f"Reading map: {png_path}")
    obstacles, map_w, map_h = png_to_obstacles(png_path, RESOLUTION)
    print(f"Found {len(obstacles)} obstacle cells ({map_w}x{map_h} map)")

    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    world_content = generate_world(
        obstacles, map_w, map_h,
        RESOLUTION, WALL_HEIGHT, WALL_THICKNESS
    )

    with open(output_path, "w") as f:
        f.write(world_content)

    print(f"World file written to: {output_path}")
    print(f"Next step: update launch/test.launch.py to load this world")


if __name__ == "__main__":
    main()