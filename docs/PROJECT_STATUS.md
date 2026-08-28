# GroundGrid 月球车项目状态

最后核对日期：2026-08-28

核对基线：`main` / `14e0fde`

状态：可进行 ROS 仿真闭环的月球车感知与局部规划原型，尚未达到任务书整体验收或真实车辆交付状态。

## 1. 项目目标与当前边界

项目从 GroundGrid LiDAR 地形分割代码扩展而来，目标是实现月球车环境感知、风险建图、局部路径规划、速度规划、跟踪控制和异常恢复，最终面向 Atlas 200i 平台部署。

当前代码已经覆盖仿真环境中的局部闭环导航，但尚未覆盖完整的公里级任务规划、多传感器感知、长期地图记忆和目标硬件部署。因此，对外应称为“仿真闭环原型”或“局部导航原型”，不能称为验收完成版本。

## 2. 当前数据链路

```text
lunar_surface_sim.py / 真实 PointCloud2、Odometry、TF
    -> GroundGridNodelet + GroundSegmentation
    -> /groundgrid/grid_map
    -> LunarTraversabilityNode
    -> /terrain/grid_map + /terrain/costmap
    -> StateLatticePlannerNode
    -> /lunar_planner/path + velocity_profile + status/diagnostics
    -> LunarPathFollowerNode
    -> /cmd_vel
```

## 3. 已实现能力

### 感知与地形表达

- GroundGrid 点云栅格化、地面候选判定、插值和分割。
- 高程、坡度及方向、粗糙度、台阶高度、障碍高度/置信度、观测状态和观测年龄图层。
- 60 m 滚动 `grid_map`，当前月面配置分辨率为 0.15 m。
- 真实射线式多线激光雷达仿真，而不是旧的笛卡尔均匀点阵。
- 观测空隙填充、LiDAR 近场盲区处理和随时间衰减的可通行性判断。

### 规划与控制

- 滑移转向动力学模型、逆命令补偿和运动基元库。
- 16 航向状态格、理想弧线或动力学运动基元、加权 A* 搜索。
- 坡度、风险、倒车、转向和路径长度等多项代价。
- 1.8 m × 1.5 m 矩形车体碰撞检测，以及原地旋转的扫掠检测。
- 目标吸附、吸附安全边界、路径和线角速度曲线输出。
- Relax、Rotate、BackOut、Abort 恢复阶梯及状态/诊断话题。
- 路径跟踪、地形降速和基础滑移补偿。

### 仿真与评估

- `mixed`、`flat`、`dense`、`slope`、`negative` 五类场景。
- 闭环到达、规划成功率、避障、近障恢复、规划耗时、绕行率、跟踪 RMSE、安全距离、CPU、RSS、异常率和输出一致性统计。
- 不可达陡坡目标与可达目标分开计分；碰撞按矩形车体到真值障碍计算。

## 4. 已确认的当前问题

### P0：重新建立可信基线

仓库中的两份 `roslog/rostest-*.log` 均为 2026-07-31 的旧失败记录：`0 errors, 1 failures, 2 tests`。Claude 历史笔记记载后续 Ubuntu 端曾修到通过，但最终五场景、每场景 `n_trials=10` 的 JSON 和 rostest XML 没有随当前提交保存。

在重新运行前，不应把历史数字当作当前提交的正式验收结果。尤其不要引用 Claude 笔记中标为“作废”的旧 LiDAR 仿真指标。

### P0：跟踪超时小于感知周期

Ubuntu 历史实测 GroundGrid/地形图约 1.05–1.10 s 一帧，而 `lunar_path_follower.path_timeout` 当前为 1.0 s。控制器同时用它判断路径和地形图新鲜度，可能在正常帧间隔内停车。需要拆分路径/地形超时，并根据实测周期留出裕量。

### P0：规划角速度未完整闭环

规划器为每个路径点发布 `(v, w)`，但 `LunarPathFollowerNode::plannedSpeedAt` 只读取线速度 `v`；角速度由跟踪器根据前视曲率重新计算。因此当前只能证明速度数组格式正确，不能证明规划器给出的动力学可行角速度真正执行到了轮端命令。

### P0：滑移辨识脚本缺陷

`scripts/identify_skidsteer.py` 把 `LunarSurfaceSim.terrain_gradient` 实例方法作为普通函数调用，导入成功时参数绑定不正确。脚本还声称将换挡后的 settle-window 样本标为 NaN，实际代码仍写入正常命令值。应先修复并增加不依赖真实 ROS 运行的求解器单元测试。

### P1：测试门槛与任务书不完全一致

- 只有 `mixed` 和 `flat` 属于严格成功率场景。
- `plan_success_all` 当前断言阈值为 90%，任务书要求规划成功率至少 99%。
- CPU 指标只覆盖规划器进程，任务书要求的是目标平台上感知与规划整体占用低于 40%。
- 默认 `n_trials=3` 只适合冒烟检查；正式数据至少使用 10 次重复试验。
- JSON 允许写出 Python 的 `NaN`，它不是严格标准 JSON。

### P1：构建与文档债务

- CMake 依赖 `tf2_msgs`，`package.xml` 尚未声明。
- `skidsteer_selfcheck` 会构建但未加入安装目标。
- `LUNAR_DEPLOYMENT_GUIDE.md` 含旧工作区路径、旧观测年龄、已删除的 `cloud_spacing` 参数和过时测试说明。
- 当前 RViz 配置主要显示感知图层，规划路径、吸附目标和诊断仍需手动添加。
- `GroundGridNodelet.cpp` 的旧图像发布路径包含硬编码索引 `data(181,181)`，不应视为通用地图中心。

## 5. 尚未完成的任务书能力

- 三维占据栅格或等价的完整三维环境表达。
- 公里级兴趣点/航点任务规划，以及全局与局部规划衔接。
- 视野外与历史障碍物的持久化记忆。
- 地图分辨率、范围和存储形式对精度与实时性的系统实验。
- 相机、TOF 与 LiDAR 的融合感知，以及任务书中的识别、尺寸、位置和坡度精度指标。
- 模型压缩、NPU 推理、Atlas 200i C++ 部署与全系统资源测试。
- 真实月面类环境、真实车辆或外部数据集验证。

## 6. 验证记录

2026-08-28 在 Windows 接手审计中完成：

- 9 个 Python 文件通过纯语法编译检查。
- 9 个 package/launch/rostest XML 文件通过解析。
- 纯 C++ `SkidSteerModel`/`MotionPrimitiveLibrary` 成功使用 GCC 13.2 编译。
- `skidsteer_selfcheck` 的零滑移、ICR、坡度、横向漂移和逆命令 5 项检查全部通过。
- 运动基元生成器可编译，默认生成结果与跟踪文件无内容差异。

Windows 环境没有 ROS1 Noetic，因此以上结果不是 ROS 闭环通过证明。

## 7. 推荐执行顺序

1. 修复跟踪超时、规划角速度闭环和滑移辨识脚本，并补定向测试。
2. 补齐 `package.xml`、安装目标、日志忽略规则和部署文档。
3. 在 Ubuntu `~/lunar_ws/src/groundgrid` 重新构建并运行冒烟测试。
4. 运行五场景、每场景 `n_trials=10`，归档 JSON、rostest XML、完整日志、参数和 commit SHA。
5. 基于新基线优化 GroundGrid 处理周期、规划超时和全系统 CPU。
6. 增加公里级全局航点规划和历史障碍地图。
7. 最后进入多传感器融合、Atlas/NPU 和真实车辆验证。

## 8. 关键资料

- `任务书.pdf`：合同/验收要求。
- `技术要点清单（总）(改)(6).docx`：16 项技术路线与 11 项软件设计要求。
- `[21]_GroundGrid_LiDAR_Terrain_2405.15664.pdf`：上游 GroundGrid 论文；论文性能不能当作本月球车分支性能。
- `docs/project-memory/claude/`：Claude 阶段历史记录；使用前必须与当前代码和新测试交叉核对。
