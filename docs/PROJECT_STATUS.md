# GroundGrid 月球车项目状态

最后核对日期：2026-08-28

核对基线：`main` / `14e0fde`；P0 实现分支：`codex/p0-trajectory-fixes`

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
    -> /lunar_planner/trajectory（Path + 一一对应的 Twist）
    -> LunarPathFollowerNode
    -> /cmd_vel
```

`/lunar_planner/path` 和 `/lunar_planner/velocity_profile` 仍锁存发布，供 RViz
和旧工具兼容，但不再作为跟踪器控制输入。

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
- 原子轨迹跟踪、规划角速度前馈与几何反馈混合，以及一次性滑移逆补偿。
- 轨迹和地形独立新鲜度门控，以及锁存的跟踪器状态话题。

### 仿真与评估

- `mixed`、`flat`、`dense`、`slope`、`negative` 五类场景。
- 闭环到达、规划成功率、避障、近障恢复、规划耗时、绕行率、跟踪 RMSE、安全距离、CPU、RSS、异常率和输出一致性统计。
- 不可达陡坡目标与可达目标分开计分；碰撞按矩形车体到真值障碍计算。

## 4. 当前风险、P0 修复状态与待验证项

### P0：重新建立可信基线

仓库中的两份 `roslog/rostest-*.log` 均为 2026-07-31 的旧失败记录：`0 errors, 1 failures, 2 tests`。Claude 历史笔记记载后续 Ubuntu 端曾修到通过，但最终五场景、每场景 `n_trials=10` 的 JSON 和 rostest XML 没有随当前提交保存。

在重新运行前，不应把历史数字当作当前提交的正式验收结果。尤其不要引用 Claude 笔记中标为“作废”的旧 LiDAR 仿真指标。

### P0 实现完成、待 Ubuntu 验证：轨迹竞态与跟踪新鲜度

规划器新增锁存的 `groundgrid/LunarTrajectory`，原子携带等长 Path/Twist；跟踪器只订阅该话题。路径/轨迹和地形新鲜度已拆分，超时均为 3.0 s，并发布 `tracking`、`goal_reached`、空/非法/陈旧轨迹、陈旧地形和 TF 不可用状态。仍需在 Ubuntu 正常闭环中证明不会出现 `stale_trajectory` 或 `stale_terrain`。

### P0 实现完成、待 Ubuntu 验证：规划速度语义和角速度闭环

理想弧线和动力学基元现在都输出滑移补偿前的期望有效车体速度。动力学基元原始轮端命令先通过 `effectiveTwist()` 转换；地形缩放和速度/加速度包络统一在规划器端执行。跟踪器直接采用规划线速度，以固定 0.5 权重混合规划角速度和几何反馈，再调用一次 `inverseCommand()`。纯 C++ 核心已覆盖正向、倒车、原地旋转、角速度前馈、限幅和非有限输入；ROS 闭环仍需验证。

### P0 实现完成、待 Ubuntu 验证：滑移辨识

月面解析地形已提取为仿真器和辨识器共用的无 ROS 模块，最小二乘核心也已拆为纯 NumPy。辨识器使用场景实例、时间戳和锁丢弃命令切换后的 settle-window 样本；NaN、无效区间、激励不足和退化矩阵不会进入或伪装成成功求解，失败时不写参数文件。合成数据测试最大误差门槛为 0.02；仍需用 `IdentifySkidSteer.launch` 验证注入参数恢复。

### P1：测试门槛与任务书不完全一致

- 只有 `mixed` 和 `flat` 属于严格成功率场景。
- `plan_success_all` 当前断言阈值为 90%，任务书要求规划成功率至少 99%。
- CPU 指标只覆盖规划器进程，任务书要求的是目标平台上感知与规划整体占用低于 40%。
- 默认 `n_trials=3` 只适合冒烟检查；正式数据至少使用 10 次重复试验。
- JSON 允许写出 Python 的 `NaN`，它不是严格标准 JSON。

### P1：剩余构建与文档债务

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

- 12 个 Python 文件通过纯语法编译检查。
- 10 个 package/launch/rostest XML 文件通过解析。
- 纯 C++ `SkidSteerModel`/`MotionPrimitiveLibrary` 成功使用 GCC 13.2 编译。
- `skidsteer_selfcheck` 的动力学 5 项和轨迹控制 5 项检查全部通过。
- 运动基元生成器可编译，默认生成结果与跟踪文件无内容差异。
- P0 分支的 5 项纯 NumPy 辨识测试覆盖已知五参数恢复、settle/NaN 样本、退化激励、解析地形和提取前后数组结果一致性。
- 合成五参数恢复最大绝对误差为 0.00344，加入 settle/NaN 样本后结果最大变化为 0。

Windows 环境没有 ROS1 Noetic，因此以上结果不是 ROS 闭环通过证明。

## 7. 推荐执行顺序

1. 在 Ubuntu `~/lunar_ws/src/groundgrid` 清理 groundgrid 包缓存并执行 Release 构建、包测试和 `catkin_test_results`。
2. 运行 `IdentifySkidSteer.launch`，确认五参数注入恢复最大绝对误差不超过 0.02。
3. 先运行 `mixed, n_trials=3` 冒烟，再运行五场景、每场景 `n_trials=10`。
4. 归档 JSON、rostest XML、完整日志、辨识参数、配置和 commit SHA；验证完成前不合并 `main`。
5. 如出现回归，在同一 P0 分支修复并复跑；不得降低 99% 任务书门槛掩盖缺口。
6. 基于新基线优化 GroundGrid 处理周期和全系统 CPU，然后进入公里级规划、历史地图、多传感器融合与 Atlas/NPU 工作。

## 8. 关键资料

- `任务书.pdf`：合同/验收要求。
- `技术要点清单（总）(改)(6).docx`：16 项技术路线与 11 项软件设计要求。
- `[21]_GroundGrid_LiDAR_Terrain_2405.15664.pdf`：上游 GroundGrid 论文；论文性能不能当作本月球车分支性能。
- `docs/project-memory/claude/`：Claude 阶段历史记录；使用前必须与当前代码和新测试交叉核对。
