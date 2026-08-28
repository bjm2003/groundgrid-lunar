# GroundGrid 月面感知与规划系统部署测试指南

本文档用于在一个新终端中，从零完成依赖安装、工作空间构建、内置月面仿真启动、规划控制验证、自动化测试，以及外部仿真器接入。

## 1. 系统要求

本项目当前是 ROS 1 工程，推荐环境如下：

- Ubuntu 20.04
- ROS Noetic Desktop Full
- Python 3
- GCC/G++ 支持 C++17
- 至少 8 GB 内存；运行 RViz 时建议 16 GB

后续命令按当前已知 Ubuntu 工作空间编写，项目源码位于：

```bash
$HOME/lunar_ws/src/groundgrid
```

如果你的路径不同，请相应修改 `PROJECT_DIR` 和 `WS_DIR`。

## 2. 安装 ROS Noetic

已安装 ROS Noetic 的机器可跳过本节。先确认：

```bash
source /opt/ros/noetic/setup.bash
rosversion -d
```

输出应为：

```text
noetic
```

如果 `/opt/ros/noetic/setup.bash` 不存在，请先按照 ROS Noetic 官方安装流程安装 `ros-noetic-desktop-full`，然后执行：

```bash
sudo rosdep init
rosdep update
```

`sudo rosdep init` 如果提示已经初始化，可以忽略。

## 3. 安装项目依赖

打开新终端，执行：

```bash
sudo apt update
sudo apt install -y \
  python3-catkin-tools \
  python3-nose \
  python3-rosdep \
  python3-numpy \
  ros-noetic-grid-map \
  ros-noetic-grid-map-rviz-plugin \
  ros-noetic-pcl-ros \
  ros-noetic-pcl-conversions \
  ros-noetic-cv-bridge \
  ros-noetic-image-transport \
  ros-noetic-tf2-ros \
  ros-noetic-tf2-geometry-msgs \
  ros-noetic-rostest
```

这里最关键的额外依赖是 `grid_map`。如果构建时出现 `Could not find grid_map_core`，通常表示 `ros-noetic-grid-map` 未正确安装。

## 4. 准备 catkin 工作空间

已在 `lyq@bjm` 上使用现有工作空间时：

```bash
export WS_DIR=$HOME/lunar_ws
export PROJECT_DIR=$WS_DIR/src/groundgrid
test -f "$PROJECT_DIR/package.xml"
```

然后用 `rosdep` 检查并补齐依赖：

```bash
source /opt/ros/noetic/setup.bash
cd "$WS_DIR"
rosdep install --from-paths src --ignore-src -r -y
```

检查 ROS 是否能识别源码包：

```bash
cd "$WS_DIR"
catkin config --extend /opt/ros/noetic
catkin list
```

列表中应出现 `groundgrid`。

## 5. 编译

执行 Release 构建：

```bash
source /opt/ros/noetic/setup.bash
cd "$WS_DIR"
catkin build -DCMAKE_BUILD_TYPE=Release groundgrid
```

成功后加载工作空间：

```bash
source "$WS_DIR/devel/setup.bash"
rospack find groundgrid
```

最后一条命令应输出项目路径。检查新增节点是否生成：

```bash
ls "$WS_DIR/devel/lib/groundgrid" | grep -E \
  'lunar_traversability_node|state_lattice_planner_node|lunar_path_follower_node|lunar_surface_sim.py'
```

应能看到四个程序。

### 5.1 使用 catkin_make 的替代方法

如果机器没有 `catkin_tools`，也可以使用：

```bash
source /opt/ros/noetic/setup.bash
cd "$WS_DIR"
catkin_make -DCMAKE_BUILD_TYPE=Release
source "$WS_DIR/devel/setup.bash"
```

同一个工作空间不要交替使用 `catkin build` 和 `catkin_make`。如需切换，请新建工作空间。

## 6. 启动内置月面仿真

每个新终端都必须先执行：

```bash
source /opt/ros/noetic/setup.bash
source "$HOME/lunar_ws/devel/setup.bash"
```

带 RViz 启动完整系统：

```bash
roslaunch groundgrid LunarDemo.launch
```

无图形界面或通过 SSH 测试时：

```bash
roslaunch groundgrid LunarDemo.launch rviz:=false
```

该启动文件会同时运行：

1. `lunar_surface_sim.py`：生成确定性的起伏地形、陨石坑、岩石、点云、里程计和 TF。
2. `groundgrid_node`：点云地面分割与状态栅格构建。
3. `lunar_traversability_node`：生成连续地形代价和占据栅格。
4. `state_lattice_planner_node`：在 `(x, y, yaw)` 状态空间规划。
5. `lunar_path_follower_node`：跟踪路径并输出差速底盘速度。

默认初始状态为：

```text
x = -10.0 m
y =  -6.0 m
yaw = 0 rad
```

仿真点云默认以 5 Hz 发布。点数较多，启动后等待几秒再发送目标。

## 7. 基础运行检查

保留启动终端，再打开第二个终端并加载环境：

```bash
source /opt/ros/noetic/setup.bash
source "$HOME/lunar_ws/devel/setup.bash"
```

检查核心节点：

```bash
rosnode list | grep -E \
  'lunar_surface_sim|groundgrid|lunar_traversability|state_lattice_planner|lunar_path_follower'
```

检查核心话题：

```bash
rostopic list | grep -E \
  'velodyne_points|odometry/filtered_map|groundgrid/grid_map|terrain/grid_map|terrain/costmap|lunar_planner|lunar_path_follower/status|cmd_vel'
```

检查输入点云和里程计是否持续更新：

```bash
rostopic hz /sensors/velodyne_points
```

按 `Ctrl+C` 停止统计，再执行：

```bash
rostopic hz /localization/odometry/filtered_map
```

模拟器的点云定时器名义周期为 0.20 s，里程计约 50 Hz；但点云生成和
GroundGrid 处理均可能成为瓶颈。历史 Ubuntu 实测 `/groundgrid/grid_map` 和
`/terrain/grid_map` 约 1.05–1.10 s 一帧，应以 `rostopic hz` 实测值为准。

检查地图消息：

```bash
rostopic echo -n 1 /terrain/costmap/info
rostopic echo -n 1 /lunar_planner/status
```

尚未发送目标时，规划状态话题可能没有消息，这是正常现象。

## 8. 发送目标并观察运动

### 8.1 使用 RViz

RViz 的 Fixed Frame 应为 `map`。等待地图显示后：

1. 点击工具栏中的 **2D Nav Goal**。
2. 在车辆附近已观测的可通行区域按下鼠标并拖动，箭头方向表示目标朝向。
3. 第一次测试建议目标设在 `(-7, -6)` 附近，即初始位置前方约 3 m。

规划器收到的目标话题是 `/move_base_simple/goal`。

### 8.2 使用命令行

不启动 RViz 时，可直接发布目标：

```bash
rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped \
"header:
  frame_id: 'map'
pose:
  position: {x: -7.0, y: -6.0, z: 0.0}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}"
```

查看规划结果：

```bash
rostopic echo -n 1 /lunar_planner/status
rostopic echo -n 1 /lunar_planner/path
rostopic echo -n 1 /lunar_planner/trajectory
rostopic echo -n 1 /lunar_path_follower/status
```

正常状态应为：

```text
data: "success"
```

观察底盘速度：

```bash
rostopic echo /cmd_vel
```

观察车辆位置：

```bash
rostopic echo /localization/odometry/filtered_map/pose/pose/position
```

车辆接近目标后，`/cmd_vel` 的线速度和角速度都应回到零。

## 9. RViz 推荐显示项

现有 `rviz/groundgrid.rviz` 已包含 GroundGrid、坡度和分割点云显示。为观察完整规划链，可在 RViz 中继续添加：

| Display 类型 | Topic | 用途 |
| --- | --- | --- |
| PointCloud2 | `/groundgrid/segmented_cloud` | 地面/非地面分割点云 |
| GridMap | `/groundgrid/grid_map` | 原始状态栅格及地形层 |
| Map | `/groundgrid/slope_grid` | 坡度栅格 |
| Map | `/groundgrid/traversability_grid` | GroundGrid 基础可通行性 |
| GridMap | `/terrain/grid_map` | 规划使用的地形代价图 |
| Map | `/terrain/costmap` | 未知、可通行和致命区域 |
| Path | `/lunar_planner/path` | 状态栅格规划路径 |
| TF | 无需指定 | `map -> base_link -> velodyne` 坐标关系 |

`/lunar_planner/trajectory` 是跟踪器唯一控制输入，其中 `path.poses[i]` 与
`twists[i]` 必须一一对应。`/lunar_planner/path` 和
`/lunar_planner/velocity_profile` 继续用于 RViz 和外部兼容工具。

`/terrain/costmap` 的语义是：

- `-1`：未知或观测已过期，规划器不可进入。
- `0..99`：可通行但代价不同。
- `100`：不可通行区域。

未知区域被有意视为不可规划区域，而不是默认自由空间。

## 10. 查看状态栅格数据层

查看 GroundGrid 输出的 layer 名称：

```bash
rostopic echo -n 1 /groundgrid/grid_map/layers
```

重点层包括：

- `observed`：是否由实际传感器观测覆盖。
- `observation_age`：观测年龄。
- `elevation_raw`：原始高程。
- `elevation_filtered`：滤波高程。
- `elevation_residual`：滤波残差。
- `roughness`：粗糙度。
- `step_height`：局部台阶高度。
- `obstacle_height`：障碍物高度。
- `obstacle_confidence`：时间滤波后的障碍置信度。
- `slope_x`、`slope_y`：坡度方向分量。

查看规划地图中的新增 layer：

```bash
rostopic echo -n 1 /terrain/grid_map/layers
```

重点关注 `terrain_cost`。规划器还会结合 `slope_x`、`slope_y` 和车辆朝向判断纵向、横向坡度。

## 11. 快速统计代价地图

下列脚本统计未知、可通行和致命栅格数量：

```bash
python3 - <<'PY'
import rospy
from nav_msgs.msg import OccupancyGrid

rospy.init_node('inspect_lunar_costmap', anonymous=True)
msg = rospy.wait_for_message('/terrain/costmap', OccupancyGrid, timeout=10.0)
data = list(msg.data)
print('size       :', msg.info.width, 'x', msg.info.height)
print('resolution :', msg.info.resolution)
print('unknown    :', sum(v < 0 for v in data))
print('traversable:', sum(0 <= v < 100 for v in data))
print('lethal     :', sum(v == 100 for v in data))
PY
```

默认地图为 60 m × 60 m，分辨率为 0.15 m。

## 12. 安全停止验证

路径跟踪器采用失效安全策略：路径过期、地图过期或 TF 缺失时发布零速度。

最简单的手动检查方式是：车辆开始运动后，在启动终端按 `Ctrl+C` 关闭整套系统。对于外部底盘接入，还必须在底盘驱动层配置 `/cmd_vel` 超时制动，不能只依赖上层节点。

单独停止规划器后观察 follower 超时归零，可在另一个终端执行：

```bash
rosnode kill /state_lattice_planner
rostopic echo /cmd_vel
```

轨迹超时固定为 3.0 s，因此速度应在约 3 s 内变为零，状态变为
`stale_trajectory`。此操作结束后，重新运行 `LunarDemo.launch` 恢复系统。

## 13. 运行端到端自动化测试

先停止手工启动的 `LunarDemo.launch`，避免同名节点和话题相互干扰。然后执行：

```bash
if command -v conda >/dev/null 2>&1; then
  while [[ -n "${CONDA_PREFIX:-}" ]]; do conda deactivate; done
fi
source /opt/ros/noetic/setup.bash
cd "$HOME/lunar_ws"
git -C src/groundgrid checkout codex/p0-trajectory-fixes
git -C src/groundgrid pull --ff-only origin codex/p0-trajectory-fixes
catkin clean groundgrid -y
catkin config --extend /opt/ros/noetic
catkin build groundgrid --no-deps -DCMAKE_BUILD_TYPE=Release
source "$HOME/lunar_ws/devel/setup.bash"
catkin run_tests groundgrid
catkin_test_results build/groundgrid/test_results
```

如果使用 `catkin_make` 构建，则执行：

```bash
source /opt/ros/noetic/setup.bash
source "$HOME/lunar_ws/devel/setup.bash"
cd "$HOME/lunar_ws"
catkin_make
catkin_make run_tests_groundgrid
catkin_test_results build/test_results
```

成功标准为：

```text
0 failures
0 errors
```

该测试会启动月面模拟器和完整规划链，在车辆前方发送一个 2 m 目标，并验证车辆实际移动且最终接近目标。

也可以直接运行 rostest：

```bash
rostest groundgrid lunar_pipeline.test
```

滑移辨识的注入参数恢复测试建议把结果写入日志目录，避免改动源码配置：

```bash
roslaunch groundgrid IdentifySkidSteer.launch \
  output:="$HOME/.ros/skid_steer_model.yaml"
```

日志中的 `max abs recovery error` 必须不超过 0.02；辨识失败、激励不足或
矩阵退化时节点应以非零状态退出，且不得生成参数文件。

正式验收前先运行 `mixed` 三次冒烟，再运行五场景十次：

```bash
rosrun groundgrid run_planner_experiments.py \
  --scenarios mixed --n-trials 3 --out-dir "$HOME/.ros/p0-smoke"
rosrun groundgrid run_planner_experiments.py \
  --n-trials 10 --out-dir "$HOME/.ros/p0-five-scenarios"
```

保留两处 JSON、`~/.ros/log` 中完整日志、rostest XML、辨识参数、运行配置和
`git rev-parse HEAD`。五场景任何一次返回非零都按回归处理，不能只汇总成功项。

P0 通过条件是 `output_conformance=1.0`、碰撞数为 0、跟踪期间没有
`stale_trajectory`/`stale_terrain`，并且五个场景的 rostest 都返回 0。
`mixed` 和 `flat` 的巡回成功率应至少为 99%；其他场景低于任务书 99% 时，
必须在验收记录中列为缺口，不能通过下调阈值改写为通过。

## 14. 接入外部仿真器或真实传感器

外部系统接入时不要启动内置 `lunar_surface_sim.py`，只启动感知规划系统：

```bash
roslaunch groundgrid LunarSystem.launch \
  point_cloud_topic:=/lidar/points \
  odometry_topic:=/odometry/filtered \
  cmd_vel_topic:=/rover/cmd_vel
```

外部系统必须提供：

| 接口 | 类型 | 必要条件 |
| --- | --- | --- |
| 点云 | `sensor_msgs/PointCloud2` | 含 `x/y/z`；当前 GroundGrid 最好同时具有 `intensity` 和 `ring` 字段 |
| 里程计 | `nav_msgs/Odometry` | 位姿应处于 `map` 坐标系 |
| TF | `map -> base_link` | 时间戳必须覆盖点云时间 |
| TF | `base_link -> velodyne` | `velodyne` 可在参数中换成真实雷达 frame |
| 目标 | `geometry_msgs/PoseStamped` | 发布到 `/move_base_simple/goal`，frame 为 `map` |
| 速度输入 | `geometry_msgs/Twist` | 默认 `/cmd_vel`，可通过 launch 参数重映射 |

必须保证点云时间、TF 时间和里程计时间使用同一时钟。如果仿真器使用 `/clock`，需要在所有节点启动前设置：

```bash
rosparam set use_sim_time true
```

注意：`LunarDemo.launch` 会主动设置 `/use_sim_time=false`，因此使用外部仿真时应启动 `LunarSystem.launch`，不要启动 `LunarDemo.launch`。

### 14.1 修改坐标系名称

默认 frame 在 `config/lunar_system.yaml` 中定义：

```yaml
groundgrid:
  map_frame: map
  base_frame: base_link
  sensor_frame: velodyne
```

接入自己的系统时，建议复制配置文件后修改：

```bash
cp "$HOME/lunar_ws/src/groundgrid/config/lunar_system.yaml" \
   "$HOME/lunar_ws/src/groundgrid/config/my_lunar_system.yaml"
```

编辑 `my_lunar_system.yaml` 后运行：

```bash
roslaunch groundgrid LunarSystem.launch \
  config:="$HOME/lunar_ws/src/groundgrid/config/my_lunar_system.yaml" \
  point_cloud_topic:=/lidar/points \
  odometry_topic:=/odometry/filtered \
  cmd_vel_topic:=/rover/cmd_vel
```

`state_lattice_planner` 和 `lunar_path_follower` 的 `map_frame`、`base_frame` 也要与 GroundGrid 保持一致。

### 14.2 修改车辆尺寸和地形限制

中型月球车的默认规划参数为：

```yaml
state_lattice_planner:
  footprint_length: 1.8
  footprint_width: 1.5
  max_longitudinal_slope_deg: 20.0
  max_lateral_slope_deg: 15.0
```

这些值必须根据真实车辆外廓、质心、轮距、轴距、稳定裕量和轮地附着能力重新标定。不要直接把演示参数用于真实车辆。

地形判定参数位于：

```yaml
lunar_traversability:
  max_longitudinal_slope_deg: 20.0
  max_roughness: 0.08
  max_step_height: 0.20
  max_observation_age: 3.0
  min_obstacle_height: 0.10
  min_obstacle_confidence: 0.45
```

建议先用保守阈值进行离线回放，再根据漏检、误检和车辆动力学试验逐项调整。

## 15. 常见问题

### 15.1 找不到 groundgrid 包

错误：

```text
Resource not found: groundgrid
```

处理：

```bash
source /opt/ros/noetic/setup.bash
source "$HOME/lunar_ws/devel/setup.bash"
rospack profile
rospack find groundgrid
```

仍找不到时，检查链接：

```bash
ls -l "$HOME/lunar_ws/src/groundgrid"
```

### 15.2 找不到 grid_map

错误通常包含：

```text
Could not find a package configuration file provided by "grid_map_core"
```

处理：

```bash
sudo apt update
sudo apt install --reinstall ros-noetic-grid-map ros-noetic-grid-map-rviz-plugin
```

### 15.3 TF extrapolation 或 lookupTransform 失败

检查 TF 树：

```bash
rosrun tf tf_echo map base_link
rosrun tf tf_echo base_link velodyne
```

如果 TF 存在但仍报时间外推，检查点云时间戳：

```bash
rostopic echo -n 1 /sensors/velodyne_points/header
rostopic echo -n 1 /localization/odometry/filtered_map/header
```

外部模拟器中最常见原因是 `/use_sim_time` 配置不一致，或 TF 发布频率太低。

### 15.4 规划状态为 no_path

依次检查：

1. 目标是否位于当前 60 m × 60 m 地图内。
2. 目标是否落在已观测区域，而不是 `-1` 未知区域。
3. 目标周围是否给 1.8 m × 1.5 m 车体留出足够空间。
4. 坡度、粗糙度、台阶或障碍阈值是否过于严格。
5. 起点或目标朝向是否导致完整 footprint 与障碍重叠。

先用 `(-7, -6)` 作为基准目标确认管线工作，再测试复杂目标。

### 15.5 一直没有 /terrain/costmap

沿数据链逐级检查：

```bash
rostopic hz /sensors/velodyne_points
rostopic hz /groundgrid/grid_map
rostopic hz /terrain/grid_map
rostopic hz /terrain/costmap
```

第一个停止更新的话题之前的节点通常就是故障位置。再用：

```bash
rosnode info /groundgrid
rosnode info /lunar_traversability
```

检查订阅和发布连接。

### 15.6 RViz 空白或卡顿

- 确认 Fixed Frame 为 `map`。
- 暂时关闭 PointCloud2 或 GridMap 显示，降低渲染负载。
- 远程机器使用 `rviz:=false`。
- 检查 `/groundgrid/grid_map` 是否真的在发布，而不是只排查 RViz。

### 15.7 点云 CPU 占用高

内置模拟器使用 64 线、0.2° 方位分辨率的射线步进点云，默认半径 25 m。
需要降低负载时，优先缩小点云半径：

```bash
roslaunch groundgrid LunarDemo.launch rviz:=false cloud_radius:=20.0
```

不要把 `azimuth_res_deg` 调粗到 0.2° 以上：GroundGrid 的地面候选密度门槛
依赖这一采样密度，调粗后会使代价图退化。缩小半径也会减少远场可规划范围，
因此性能数据必须记录该参数。

## 16. 最短验证流程

依赖已经安装、项目已经编译时，只需要三个终端。

终端 1：

```bash
source /opt/ros/noetic/setup.bash
source "$HOME/lunar_ws/devel/setup.bash"
roslaunch groundgrid LunarDemo.launch rviz:=false
```

终端 2：

```bash
source /opt/ros/noetic/setup.bash
source "$HOME/lunar_ws/devel/setup.bash"
rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped \
"header: {frame_id: 'map'}
pose:
  position: {x: -7.0, y: -6.0, z: 0.0}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}"
```

终端 3：

```bash
source /opt/ros/noetic/setup.bash
source "$HOME/lunar_ws/devel/setup.bash"
rostopic echo /lunar_planner/status
```

看到 `success` 后执行：

```bash
rostopic echo /localization/odometry/filtered_map/pose/pose/position
```

车辆从 `(-10, -6)` 向 `(-7, -6)` 移动并最终停车，即表示从点云、状态栅格、地形评价、状态栅格规划到路径跟踪的完整链路工作正常。
