# Bezier 轨迹优化 — ROS 2 Humble 完整复现指南

基于贝塞尔多项式的轨迹优化运动规划，前端 A* 搜索 + 后端运动走廊膨胀 + QP 最小 jerk 优化。

原项目 `hourenyu/Trajectory-optimization-based-on-Bezier-polynomial-motion-planning-` 的 ROS 2 Humble 移植版。

---

## 环境要求

| 项目 | 版本 |
|------|------|
| 系统 | Ubuntu 22.04（原生或 WSL 均可） |
| ROS 2 | Humble |
| 编译器 | g++ 11.4.0+ |
| CMake | 3.10+ |

---

## 一、安装 ROS 2 Humble（如已安装跳过）

```bash
sudo apt update && sudo apt install -y ros-humble-desktop python3-colcon-common-extensions
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 二、安装项目依赖

```bash
# nav2 地图服务器 + 生命周期管理
sudo apt install -y ros-humble-nav2-map-server ros-humble-nav2-lifecycle-manager

# tf2
sudo apt install -y ros-humble-tf2-ros ros-humble-tf2-geometry-msgs

# Eigen3 线性代数库
sudo apt install -y libeigen3-dev

# rviz2 可视化
sudo apt install -y ros-humble-rviz2

# Gazebo 仿真（用于机器人仿真）
sudo apt install -y ros-humble-gazebo-ros ros-humble-gazebo-plugins

# xacro（URDF 处理）
sudo apt install -y ros-humble-xacro
```

---

## 三、安装 qpOASES（必须源码编译，仅需一次）

```bash
cd ~
git clone https://github.com/coin-or/qpOASES.git
cd qpOASES
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```

验证安装成功：
```bash
ls /usr/local/lib/libqpOASES.a
ls /usr/local/include/qpOASES.hpp
```
两个文件都存在即可。

---

## 四、克隆项目并编译

```bash
# 克隆仓库
cd ~
git clone https://github.com/7owk/motion-planning.git
cd motion-planning

# 创建 colcon 工作空间结构（colcon 要求包在 src/ 目录下）
mkdir -p src
ln -sf ../Btraj src/btraj

# 编译
source /opt/ros/humble/setup.bash
colcon build --packages-select btraj --symlink-install
```

编译成功输出：
```
Starting >>> btraj
Finished <<< btraj [XXs]
Summary: 1 package finished [XXs]
```

`--symlink-install` 的好处：修改 launch、rviz、maps 等非编译文件后不需要重新编译，直接生效。

---

## 五、运行

```bash
source install/setup.bash
ros2 launch btraj test.launch.py
```

启动后终端会输出各节点日志，同时弹出 rviz2 窗口，显示 50×50 的栅格地图。

### 交互操作

1. 点击 rviz2 顶部工具栏 **2D Pose Estimate**（绿色箭头），在地图空白处点击并拖动方向，设起点。
2. 点击 **2D Goal Pose**（红色箭头），在另一个空白处点击，设终点。
3. 观察终端输出 `Find Goal!`，rviz2 中依次出现：
   - 绿色点：A* 搜索扩展节点
   - 红色点：A* 最短路径
   - 蓝色半透明方块：膨胀后的运动走廊
   - 红色轨迹：贝塞尔 QP 优化后的最终轨迹

---

## 六、常见问题

### rviz2 报错 "No map received"

在左侧面板点开 **Map** 属性，找到 **Durability**，改成 **Transient Local**。`nav2_map_server` 用持久化 QoS 发布地图，rviz2 必须匹配。

### rviz2 地图收到但不显示（GLSL shader 报错）

WSL 或部分显卡驱动下 rviz2 的 Map 渲染器会报 GLSL 链接错误。解决方法：

1. Map 属性中把 **Color Scheme** 从 `map` 改成 `costmap`。
2. 如果还不行，强制软件渲染启动 rviz2：
   ```bash
   LIBGL_ALWAYS_SOFTWARE=1 rviz2
   ```

### rviz2 每次都要手动加显示项

第一次配好后，点 **File → Save Config As**，保存到：
```
<项目根目录>/Btraj/rviz/rviz.rviz
```
覆盖旧文件。之后 `ros2 launch btraj test.launch.py` 会自动加载你的配置。

### colcon build 报 qpOASES 找不到

确认 `/usr/local/lib/libqpOASES.a` 存在。如果装到了其他路径，修改 `Btraj/CMakeLists.txt` 中 `find_library` 和 `find_path` 的 `PATHS`。

### colcon build 报 rclcpp 等 ROS 包找不到

确认已 source ROS 2 环境：
```bash
source /opt/ros/humble/setup.bash
```

### 起点/目标点设了没反应

检查终端日志。如果输出 `Invalid start!` 或 `Invalid goal!`，说明点击位置在障碍物上或地图范围外。在空白区域重新设置。

---

## 七、项目结构

```
motion-planning/
├── Btraj/
│   ├── CMakeLists.txt          # ament_cmake 构建配置
│   ├── package.xml             # ROS 2 包描述
│   ├── include/
│   │   ├── Asearch.h           # A* 类声明
│   │   └── b_traj.h            # Bezier 走廊 + QP 类声明
│   ├── src/
│   │   ├── astar.cpp           # A* 节点 main
│   │   ├── Asearch.cpp         # A* 搜索实现
│   │   ├── b_traj.cpp          # 走廊膨胀 + QP 轨迹优化
│   │   └── tf_br.cpp           # 静态 TF 发布 (map→odom)
│   ├── launch/
│   │   └── test.launch.py      # ROS 2 启动文件
│   ├── maps/
│   │   ├── map.yaml            # 地图配置
│   │   └── map_maze.png        # 50×50 栅格地图
│   └── rviz/
│       └── rviz.rviz           # rviz2 配置
├── src/                        # colcon 工作空间目录（编译时创建）
├── build/                      # 编译产物（自动生成）
├── install/                    # 安装产物（自动生成）
└── log/                        # 编译日志（自动生成）
```

---

## 八、调试命令

```bash
# 查看运行中的节点
ros2 node list

# 查看话题
ros2 topic list

# 手动发送目标点（不用 rviz）
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 20.0, y: 20.0, z: 0.0}, orientation: {w: 1.0}}}"

# 查看 b_traj 参数
ros2 param list /b_traj

# 单独启动某个节点
ros2 run btraj astar
ros2 run btraj b_traj
ros2 run btraj tf_br
```

---

## 九、规划流程概览

```
起点/终点 → A* 搜索最短路径
                ↓
        沿路径膨胀障碍物生成运动走廊（bounding box）
                ↓
        走廊重叠区域简化 + 中心点提取
                ↓
        梯形速度曲线做粗略时间分配
                ↓
        构建 QP：最小化 jerk（最小化加加速度的积分）
              - 等式约束：起点/终点 p,v,a + 段间连续性
              - 不等式约束：走廊边界 + 速度/加速度限幅
                ↓
        qpOASES 求解 → 贝塞尔曲线控制点
                ↓
        输出平滑轨迹
```
