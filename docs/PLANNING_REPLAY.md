# 规划输入快照与重放

快照不是控制命令，也不是实时地图闭环的替代品。它用于比较同一组实际输入下的
目标选择、搜索、输出和耗时。控制链、停车确认、恢复阶段仍由 ROS 节点负责。

## 记录

给 `lunar_pipeline.test` 或 `LunarSystem.launch` 传入
`planning_snapshot_directory:=/绝对路径/新的目录`。父目录必须存在，记录目录必须
不存在，避免不同进程重用 `attempt_id` 覆盖旧记录。空字符串（默认）关闭记录。

实际 `plan()` 调用在原节点互斥锁下捕获当前地图、缓存、起点、请求目标、时间戳、
实际生效的 Relax 参数和已加载的基元。记录不从后来收到的 TF/地图拼接。
每个 `attempt-N.ggsnap` 是显式小端、带版本和校验和的完整输入，`attempt-N.json`
是原运行输出摘要。浮点未知值及纳秒时间戳无损；基元不经旧文本格式舍入。
v2 另保存 `attempt-N-trajectory.json` 的完整原输出，新增策略字段；读取器仍接受
v1 输入并将其解释为旧吸附策略。默认旧策略保留为 Ubuntu 等价采集的对照入口。
计算过程只负责捕获和入队，独立线程写盘。`writer-summary.json` 中必须满足
`submitted == written`、`dropped == 0`、`failed == 0`，否则记录不完整。
归档工具另算各文件 SHA-256；二进制自身校验和仅用于检测损坏，不用于认证来源。

## 重放

普通计时重放（无需启动 roscore；已编译安装/加载包环境）：

```bash
rosrun groundgrid replay_planning_snapshot /绝对路径/attempt-42.ggsnap --repeat 5
```

固定扩展预算用于确定性对照，不能作为低于 1 秒的性能证明：

```bash
rosrun groundgrid replay_planning_snapshot /绝对路径/attempt-42.ggsnap --expansions 50000 --trajectory
```

同一输入切换策略，不改地图、起点、限速或安全参数：

```bash
rosrun groundgrid replay_planning_snapshot /绝对路径/attempt-42.ggsnap --strategy reachable_cost --repeat 5
```

运行时通过 launch 参数 `snap_strategy:=reachable_cost` 显式选择新策略。
在首轮真实快照采集及对照完成前，默认仍为 `legacy_nearest`。
新策略把范围内安全终点作为一个目标集合，使用共享加权 A* 队列，路线代价加
原吸附偏移/朝向/地形代价作为目标代价。候选按需检查，不把几何启发式当安全证明。
它不保证全局最优执行时间；状态格量化、加权启发式和统一时间预算仍存在。

重放工具返回 0 表示文件有效且计算执行成功；规划器本身是否找到路线在 JSON 的
`result.ok` 中。原输入本来就可能无解，不能把所有重放返回 0 当成全部任务成功。
`total_ms` 在 ROS 记录中包含捕获/适配开销，离线重放只计核心；两者需要分列。

## 发布前检查

Windows 运行纯 C++ 核心、快照和现有安全自检。Ubuntu 另运行
`planning_grid_parity_selfcheck`，对照安装的 grid_map 在地图边界、分辨率和循环
索引上的行为。映射语义已核对 [grid_map 原实现](https://github.com/ANYbotics/grid_map/blob/master/grid_map_core/src/GridMapMath.cpp)，运行机器上的对照测试仍是必要条件。
对 ROS 接口、停车/恢复行为及实际任务完成率的验证必须跑原闭环测试。
