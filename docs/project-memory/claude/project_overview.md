---
name: Project overview
description: 月球车项目目标、部署目标平台、感知与规划模块的关系
type: project
originSessionId: a7705bb8-c2fe-4c8f-b882-44fcb85ff798
---
月球车项目（D:\groundgrid-main），最终目标是部署到月球车上（目标算力平台 Atlas 200i / NPU）。

- 感知模块基于论文 GroundGrid (LiDAR Terrain, arXiv 2405.15664) 的开源代码开发，产出增强的 grid_map（含 slope/slope_x/slope_y/roughness/step_height/obstacle_height/obstacle_confidence/observed/observation_age/groundpatch 等图层）。
- 当前任务：在感知基础上继续增加**规划模块**。用户此前已在论文开源码基础上加过部分规划代码。
- ROS 1 (catkin)，C++17，差速/滑移转向 rover。Windows 开发、Ubuntu 编译运行的双机工作流：经 **GitHub 仓库**同步（Windows 端 `git add -A && commit && push` → Ubuntu 端 `git -C src/groundgrid pull`）。
- **验证进展（2026-07 首次在 Ubuntu 端端到端跑）**：滑移动力学/辨识（要点1）已通过——`IdentifySkidSteer.launch` 复原注入真值最大误差 ~1%。skidsteer_selfcheck、基元库生成均通过。
- **no_path 根因已定位（2026-07-28 静态分析）**：`LunarTraversabilityNode.cpp` 把未观测栅格的 `terrain_cost` 留为 NaN（只有 observed+age≤1s+groundpatch≥0.2+slope/rough/step 有限的格子才赋值）；`StateLatticePlannerNode.footprintValid` 对任一 NaN 格子 return false。车身正下方/紧邻栅格被自身遮挡永远未观测 → 起点 1.8×1.5m footprint 必挂 → 与目标点无关的系统性 no_path。
- **已修（最小方案「只修起点」，2026-07-28，待 push+Ubuntu 验证）**：`footprintValid` 加 `allow_unknown` 开关，仅起点校验传 true——未观测(NaN)/出图格子跳过，仍拒绝致命(≥100)与超坡；目标校验、`transition`/`primitiveValid` 路径校验保持严格。起点诊断日志相应改写。
- **第二个卡点已定位并修复（2026-07-28 Ubuntu 端实测）**：发目标后一条 `plan:` 都不打印——因为 `plan()` 前有三道门(have_map_/stale_map/tf_unavailable)，状态只发 `/lunar_planner/status` 话题不打屏。实测 `rostopic echo /lunar_planner/status` 恒为 `stale_map`：感知 groundgrid ~1.08s/帧，`/terrain/grid_map` 间隔 ~1.1s，而规划器硬编码「地图 stamp 超 1.0s 判过期」→ 每帧必过期 → plan() 从不执行。**已修（commit ce5180c，待 push+验证）**：新增参数 `max_map_age`(默认 3.0s)替换硬编码 1.0s，`config/lunar_system.yaml` 的 `state_lattice_planner:` 段已加。
- **调试要诀**：规划器三道门(stale_map/tf_unavailable/have_map_未成立)不打屏，只发 `/lunar_planner/status`；诊断先 `rostopic echo /lunar_planner/status`。Demo 启动命令：`roslaunch groundgrid LunarDemo.launch`(含 sim+系统+RViz)，RViz 用 "2D Nav Goal" 发 `/move_base_simple/goal`。仿真脚本 `lunar_surface_sim.py` 正常发 `map→base_link`、`velodyne→base_link` TF，初始位姿 (-10,-6,yaw0)。
- **端到端规划已打通（2026-07-28 Ubuntu 实测里程碑）**：命令行发近目标 `rostopic pub -1 /move_base_simple/goal ...`（如 (-7.5,-6) 朝+x，车初始 (-10,-6) yaw0）→ `/lunar_planner/status`=`success`、发布出 `/lunar_planner/path`。三关（起点 footprint 卡死→stale_map→现在）全过。**注意语义坑**：`plan:` 全是 ROS_WARN，仅失败打印；成功时终端无任何 plan 输出，别误判为没跑——看 `status=success` 和 `/lunar_planner/path` 才对。RViz 默认配置无车体显示，看车/路径要手动 Add：`/lunar_planner/path`→Path、`/localization/odometry/filtered_map`→Odometry。
- **搜索提速阶段一已实现（commit d79bbb0，2026-07-28，待 Ubuntu 量化验证）**：正法而非快法（快法=调大 max_planning_time 被否，因与要点11 公里级/指标10 实时/NPU 部署冲突）。根因：footprintValid 每个 ~143 采样点做 grid_map 字符串图层查找+2次 atan，每节点~8000次 → 仅 844 节点/s。做法：mapCallback 里 `buildTraversabilityCache()` 把 terrain_cost/slope_x/slope_y+预算 slope 幅值展平成连续数组；footprintValid 改读数组，坡度幅值≤max_lat(15°)的缓格跳过方向性 atan（数学等价：|g·u|≤|g|），仅陡格回退。行为严格不变、纯提速。**分阶段**：阶段一低风险行为不变先上；量化(看 plan: expanded 相对 844 的倍数)后再决定阶段二(单次 plan 内 footprint 记忆化/降采样)——因本机无法跑 ROS 验证，靠 Ubuntu 实测数据驱动。
- **阶段一实测**：expanded 844→2251(2.7×)，行为不变；但远目标仍在 1s 预算处 search exhausted，故上阶段二。
- **搜索提速阶段二已实现（commit 79029d8，2026-07-28，待 push+验证）**：①arc transition 弧上 footprint 校验从固定 6 次降为「半车长间距+转角>~11°才加点」→ 直行 1 次/转弯 2 次（footprint 1.8m≫步长 0.45m，父+子 footprint 连续覆盖，安全性不变）；②`heuristic_weight` 1.2→2.0（加权 A* 更贪心、到达远目标扩展数大减，属搜索效率正法非「干等」，代价路径次优≤~2×）。dynamics 模式的 primitiveValid 未改（当前 use_dynamics_primitives=false，非活跃路径）。**命令行 rostopic pub 报 XmlRpcClient connection reset 是环境/master 繁忙问题，与代码无关；测量改用 RViz 2D Nav Goal。**
- **阶段二实测通过（2026-07-29）**：干净已观测地形的远目标现在都能 `success`，`search exhausted` 已消失——性能问题解决。剩余失败均为 `GOAL footprint invalid`（正确行为，非 bug）：目标是 1.8×1.5m 车身框按朝向旋转，整框须落在已观测+非致命+达标坡度上，中心绿不够；未观测 NaN 格在 Map 显示里非绿易误判。**用户指引**：目标中心离非绿/空白留 ~1.2m(车身半对角)、往观测区里侧点、朝向拖自然前进向。**规划链路+性能均已达标。可选增强：目标不合法时吸附到最近合法位姿(对应要点13 近障/恢复)，用户未定是否做。**

- **速度剖面 + 吸附 + 恢复 + 指标框架已实现（commits 50a6048 / deaf386 / 80e400d / 485bb3f / 94e4364，2026-07-29，全部待 push+Ubuntu 验证）**：定性纠正——「目标吸附」不是可选增强，它属要点13 + 软件设计9 + 任务书3.2 恢复要求 + 避障恢复率指标。①**圆弧模式速度剖面**：`/lunar_planner/velocity_profile` 此前只在 `use_dynamics_primitives=true` 分支填，而出厂默认是 false → 该话题恒空、违反任务书3.2 输出要求、a_max/alpha_max 从未生效。新增 `buildVelocityProfile()` 前后向梯形规划挂到圆弧分支。②**目标吸附** `snapGoal()` 有界螺旋（世界坐标步进，**不可用 index 算术**——grid_map 是环形缓冲会在接缝错格），status 新增 `success_snapped`，新话题 `/lunar_planner/snapped_goal`。③**卡死检测+恢复阶梯** Relax→Rotate→BackOut→Abort，双触发（失败连击 / 无进展超时抓振荡），新话题 `/lunar_planner/diagnostics`（key=value，`recovery_events/successes` 是单调计数器供测试差分，`plan_ms` 是唯一可外部观测的规划耗时）。**同时把重规划从 0.5s 定时器改为地图到达触发**——0.5s tick vs ~1.1s 感知周期会对同一份地图规划两次，是振荡真因。④**测试指标框架** `test_lunar_pipeline.py` 拆出 `_run_trial`，确定性目标巡回（4 易目标闭环×n_trials + 3 难目标），报规划成功率/避障恢复率/规划耗时均值方差极值。**关键坑**：goal 每 trial 只发一次——反复发会每 tick 重置卡死状态，恢复永远触发不了、恢复率恒为 0。

**Why:** 用户在推进规划模块的实现，需要清楚已完成 vs 待做。
**How to apply:** 涉及规划时对照技术路线清单判断进度；感知层图层名是规划输入契约，改动需保持一致；Ubuntu 构建/运行见单独的 build 环境 memory。
