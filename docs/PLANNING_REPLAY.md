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

## 一终端运行与归档

新版批量脚本必须指定**尚不存在**的 --out-dir。每轮独立 ROS 进程、ROS_HOME、
ROS_LOG_DIR、ROS_TEST_RESULTS_DIR 和 run_id；目录中保存完整日志、参数、JSON、
断言 XML、源配置、commit、SHA-256 清单。结束自动生成同名 .tar.gz，失败也归档。
原始日志不删除。控制日志较多时直接回传结果包，无需复制终端长输出。

首轮只采集旧策略真实输入（不是新策略验收；剩余困难目标仍可能失败）：

    rosrun groundgrid run_planner_experiments.py --scenarios mixed --n-trials 3 --repeat 1 --snap-strategy legacy_nearest --capture-inputs --debug-control --out-dir "$HOME/p0-capture-$(date +%Y%m%d-%H%M%S)"

此命令先运行 8 个自检，包括 Ubuntu grid_map 对照；自检失败便停止启动闭环并归档。
SUITE_RC=0 要求 rostest、catkin_test_results 和归档检查均通过；旧策略首轮
SUITE_RC=1 时仍应回传归档，不要自行重跑。注意使用普通 rostest，不加 --text：
[ROS Noetic 的实现](https://github.com/ros/ros_comm/blob/noetic-devel/tools/rostest/src/rostest/runner.py)
在 text 模式下不按正常流程汇总子测试 XML，不能用它采集正式断言结果。

收到首轮快照后，选择归档目录进行同输入对照；以下 CAPTURE_DIR 须设为实际目录：

    rosrun groundgrid compare_planning_snapshots.py --inputs "$CAPTURE_DIR" --out-dir "$HOME/p0-replay-$(date +%Y%m%d-%H%M%S)" --expansions 5000 --repeat 3

默认包含所有输入与所有重复，保存旧/新策略的固定扩展预算和真实时钟两组输出。
需要仅诊断某项时显式传 --goal-id 16，报告保留该筛选条件，不能把子集当全量结果。
重放另外调用生产扫掠检查器审核实际导出折线（含起步间距恢复），
export_safety_ok=false 会使对照归档失败。检查时间不混入搜索时长；
此离线审核不替代 ROS 动态重检，更不是物理制动证明。

新策略获得同输入审核通过后，才执行以下后续组次（每行一个独立结果包）：

    rosrun groundgrid run_planner_experiments.py --scenarios mixed --n-trials 3 --repeat 3 --snap-strategy reachable_cost --identify-first --out-dir "$HOME/p0-smoke-arcs-$(date +%Y%m%d-%H%M%S)"

    rosrun groundgrid run_planner_experiments.py --scenarios mixed --n-trials 3 --repeat 1 --snap-strategy reachable_cost --primitive-mode dynamics --out-dir "$HOME/p0-smoke-dynamics-$(date +%Y%m%d-%H%M%S)"

    rosrun groundgrid run_planner_experiments.py --scenarios mixed flat dense slope negative --n-trials 10 --snap-strategy reachable_cost --out-dir "$HOME/p0-baseline-$(date +%Y%m%d-%H%M%S)"

--identify-first 使用独立 master 启动已知参数仿真，参数文件写入该结果目录，
不覆盖源配置；五参数最大误差必须 ≤0.02。动力学组显式指定已跟踪的基元文件；
每次尝试中的实际模式与所请求模式必须相同，不能接受静默回退。
构建日志和包测试仍需单独归档；批量脚本不会替用户编译或伪造 catkin 包测试通过。

各结果包的 suite.json / 每轮 run.json 列出阶段返回码、唯一规划尝试统计、
任务总耗时和任务书缺口。任何成功包都不自动标记完整验收，需同时审阅三次弧线
冒烟、一次动力学冒烟、五场景 n=10、辨识和构建/包测试。所有安全阈值及测试时限不变。
实际地图重放前不根据合成墙体案例进行状态缓存或经验参数调优。

2026-09-06 已取得第一包真实输入，Windows 对照发现并修复“空安全目标集合仍
进行满预算搜索”的开销。新策略增加同一预算内的候选存在性探测，见到一个安全
候选即停止；它不是预先选定该终点，后续共享队列仍比较任意可达候选。只有集合
被完整检查且为空才报 goal_invalid；超时为 snap_timeout。安全结果缓存仅存在于
一次调用内，不跨地图、请求或安全策略。

同日 fabd8a0 的 Ubuntu 39 份输入、468 次重放已经完成完整性、安全性及
确定性审核，旧策略输出逐值复现原记录。见
[Ubuntu 重放记录](validation/2026-09-06-ubuntu-replay.md)。当前应推进包测试、
辨识及显式 reachable_cost 闭环，不重新采集或重放；默认策略暂不改变。

随后旧策略包测试复现 (8,-5) Abort，47 项 Python 测试已通过。详见
[包测试记录及执行顺序调整](validation/2026-09-06-package-test.md)。下一步先用
上述 --identify-first、--repeat 3、--snap-strategy reachable_cost 入口，增加
--capture-inputs --debug-control 保留新策略失败现场；这不是重复旧输入采集。
完整包测试全绿仍是后续交付条件。

注意：若手动设置 CATKIN_TEST_RESULTS_DIR 到默认构建树以外，catkin test 的
内置无参数 catkin_test_results 可能只汇总到0项而返回0。必须再显式指定真实
XML 目录，并确认预期文件/测试数量齐全。批量脚本已经显式指定每轮目录并检查
pipeline 断言 XML，勿用裸 TEST_RC 或“0 tests”替代真实测试通过。
归档验证器还会独立检查 XML 失败计数/元素和 pipeline 至少2项测试；即使两个
外层命令均错误返回0，也不能将实际失败或空结果记为成功。

df032de 已通过五参数辨识和三轮独立 reachable_cost / arcs / mixed / n_trials=3，
详情见[三轮弧线记录](validation/2026-09-06-arcs-smoke.md)。后续动力学回归暴露
起步门槛和基元接缝故障，尚未通过；当前需更新并构建修复版，再跑 dynamics
同规格一次，增加 --capture-inputs --debug-control；无需重复 --identify-first。
动力学搜索现在从实际积分终点续接，不能将量化格心视作额外可执行动作。
吸附结果记录实际导出终点，精确目标可在已有积分采样点结束；参数和预算不变。
见[连续接缝记录](validation/2026-09-07-dynamics-continuity.md)。两模式审核后统一
决定默认启用及复核包测试，再进入五场景基线；旧提交通过不自动认证新提交。
