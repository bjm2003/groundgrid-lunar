---
name: Ubuntu build/run environment
description: 在 Ubuntu 上 catkin 编译与 roslaunch 验证本项目的环境要点与已知构建坑
type: reference
originSessionId: 3b2a6700-56df-4faf-b739-389c28584b50
---
项目在 Windows 开发（`D:\groundgrid-main`），在 **Ubuntu 机器上编译/运行**（用户 `lyq@bjm`，workspace `~/lunar_ws`，源码在 `~/lunar_ws/src/groundgrid`）。

- **必须用 ROS 1 Noetic**（catkin）。该机同时装了 ROS2，但本项目是 ROS1 包，ROS2 编不了；两者可共存，跑本项目的终端只 `source /opt/ros/noetic/setup.bash`。workspace extend 链含 `/home/lyq/AD2025/devel:/home/lyq/truck_ws/devel:/opt/ros/noetic`。
- **忌 conda**：Noetic 的 rospy/catkin 绑定系统 Python 3.8，conda 里常炸。本轮脚本唯一非 ROS 第三方依赖是 numpy，用 `apt install python3-numpy` 即可，不必装整包 requirements.txt（后者大半是 KITTI 评估/坡度导出用的，与规划链路无关）。
- **已知构建坑 — OpenCV**：`grid_map_ros` 间接 `find_package(grid_map_cv)`，后者硬要 `/usr/include/opencv4` 存在。该机 OpenCV 装得乱（`/usr/include/opencv2`、`/usr/local/opencv3`、`/usr/local/include/opencv4` 多套），`/usr/include/opencv4` 不存在导致 cmake 报错。解法：`sudo ln -s /usr/local/include/opencv4 /usr/include/opencv4`。注：我们的规划模块不用 grid_map_cv 功能，软链只为过 cmake 目录存在性检查。
- 改完代码重编前先删缓存：`rm -rf build/groundgrid devel/.private/groundgrid` 再 `catkin build`。`catkin clean groundgrid` 在从未成功构建过时会报 "No devel manifest"，无害。

**Why:** 每次换机/重装都会重复踩这些坑，尤其 OpenCV 软链和 ROS1/ROS2 混装。
**How to apply:** 用户在 Ubuntu 上编译报错时先对照这几条；报 `/usr/include/opencv4 not found` 直接给软链方案。
