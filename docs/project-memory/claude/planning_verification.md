---
name: Planning module verification results
description: 规划模块在 Ubuntu 端的实测结论与踩过的坑；2026-08-03 mixed 场景全绿，指标待重测
type: project
originSessionId: b4d9fbc1-08f5-4fcf-865c-1871d0c38723
---
**2026-08-03：mixed 场景 rostest 全绿（碰撞 0、巡回 12/12），2026-07-29 那批作废指标的重测通道打通。** 从 4 次碰撞到全绿一共五跳，根因互不相同、且**每一跳都是被前一跳的修复暴露出来的**，顺序不可颠倒：感知近场失明(`min_point_distance`) → 原地旋转不查扫掠(`rotationValid`) → 吸附位姿零余量(`goal_snap_clearance`) + 石头真值半径用了形状半径 → 余量误用在「未观测」上。最后一跳的教训单独记：**`goal_snap_clearance` 的余量带必须 `allow_unknown=true`**，车体矩形本身才严格校验。用严格校验会要求「车体外 0.25 m 也已观测」，而遮挡阴影里的目标（(0.9,-1.6) 就是专门设计成这样的）按定义给不出，表现为 `driven=0.00` + 只有 `aborted`——**`plan()` 在 `snapGoal` 失败时直接 return，一个节点都不展开，所以「一步没动」是「吸附失败」而不是「搜索失败」**，这是最快的分诊信号。commit `5530339`。

**2026-08-03（指标口径）：全绿之后第一批数字里有两个指标是假的，都是「测量口径」问题不是规划器问题。** (1) `detour_ratio` mean **−0.31**——几何上不可能，路径不可能比它被比较的直线还短。根因：`_reset_trial_state()` 比发目标早 0.5 s，而规划器每 ~1.1 s 按地图重规划一次，于是「目标发出后第一条路径」经常是**上一个目标的滞后重规划**或**恢复动作**（两者都短）。修法：只接受「起点在本 trial 起点 2.0 m 内 且 终点在目标 3.5 m 内 且 终点比起点更靠近目标」的路径；`输出达标率` 另用 `path_seen` 标志保留原来的宽口径（它管的是 follower 的 2 浮点/位姿不变量，与路线质量无关）。(2) `obstacle_avoidance` **0.75 恰好等于 `reach_rate`**——碰撞为 0 时「到达且无碰撞/可测」退化成「到达」，而 4 个 hard 目标本就设计成部分不可达，于是这个指标在测目标难度。修法：把「正确 abort」的 trial 移出分母（沿用 `recoverable = events - aborts` 的论证），但**碰撞过的 trial 无论怎么结束都留在分母里**。commit `d82c03d`。**通用教训：任何比率指标算出来恰好等于另一个比率，就是它退化了。**

**2026-08-03（首批有效数字，n_trials=3，mixed）**：`plan_success_tour` 1.000(12/12)、`plan_success_all` 0.9375、`near_obstacle_recovery` 1.000、碰撞 0、`output_conformance` 1.000、`anomaly` 0.25；`plan_ms` mean **49.99** / max **1000.30**（14.7→50 是 `rotationValid` 的代价；max 精确等于 `max_planning_time`=1.0 ⇒ 至少一次搜索耗尽预算，属正常超时后转恢复）；`cpu_pct` mean 8.71 / max **101.40**（99 样本中 3 个超 40% 的 Atlas 预算，与那次 1 s 满算自洽）；`rss_mb` 165；`tracking_rmse` 0.15。`detour_ratio`/`obstacle_avoidance` 这两项按上条修复后需重测。

**2026-08-05（指标口径第二跳）：路径长度偏差必须除以「路径自己的弦」，不是 trial 起点到目标的直线。** 加了「只收本 trial 的路径」过滤后仍有 min −0.87：过滤解决不了这个，因为被相减的两个长度**端点不同**——目标在途中飞行时车已开出 1~2 m，目标本身还可能被吸附挪 3 m，两者都缩短了路径却没缩短分母那条线。改成 `(path_len − hypot(last−first))/hypot(last−first)`，按构造 ≥0，直行为 0，只有绕行才为正。**另一条：`规划成功率` 分母里不能放「正确答案是失败」的目标。** (3,0) 在陨石坑 3.2 m 碗内、壁面 40° 对 20° 限，Abort 才对；把它算作失败使指标上限锁死在 15/16=0.9375 而阈值是 0.90，设计内的结果就吃光了全部余量，随便一次抖动就挂测（实测 0.875 挂）。改为移出比率、**改成正面断言**（必须 abort、且绝不能到达）——这是更严不是更松：旧写法根本测不出「本该拒绝的目标被开过去了」。commit `14e0fde`。

**2026-08-05：`obstacle_avoidance` 修复已验证** = 1.0 且不再等于 `reach_rate`(0.75)。同一次运行 `plan_success_all` 0.9375→0.875、`near_obstacle_recovery` 1.0→nan（`events−aborts=0`，即恢复入口全部以 abort 收场）、`anomaly` 0.25→0.3125，而本轮**只动了测试脚本、规划器二进制未变**，所以这是 mixed hard 目标的**跑间抖动**，不是回归。结论：mixed 的 hard 目标处在判定边界上，单次 n_trials=3 的 hard 结果不可作为结论，报告数字必须来自 n_trials=10。

**诊断口诀（这几轮反复用到）**：先看 `driven`。`driven=0.00` ⇒ 没发出路径 ⇒ 查 snapGoal/目标校验，不是搜索。`driven>0` 但 `clr_min<0` ⇒ 走位问题，看 `at` 和朝向能否复现（支持函数 `0.9|cosφ|+0.75|sinφ|`，车角 1.1715 m、正面 0.9 m、侧面 0.75 m——三个数一算就知道是不是转弯扫掠）。多条碰撞的 `at` 完全相同 ⇒ 是同一次事件被后继 trial 的起点继承，不是多次。

**2026-07-31（第五跳）：3 条碰撞是同一个位姿被记了 3 次——吸附后的目标位姿零余量。** 断言里三行的 `at` 完全相同 (-1.58,-2.20)，后两条目标只是「从那儿起步」，clearance 取的是全程最小值即起点样本。根因两条，**都要改才够**：(1) `snapGoal` 按「距离最近且 footprintValid 通过」选点，**天然落在致命边界上**，而两项误差必然存在——代价图量化到 1 格（车角可能落在最后一个被查格心外 0.106 m）、follower 按 `goal_yaw_tolerance` 10° 停车（车角比被校验的朝向再外伸 0.12 m）。已加 `goal_snap_clearance`(0.25 m)，只在吸附校验时把 footprint 膨胀（`footprintValid` 新增 `margin` 默认 0，**不动全局规划**，否则 dense 场景 1.92 m 通道会被封死）。(2) **仿真发的石头真值半径是形状半径，不是危险半径**——石头是圆锥 `h*(1-d/r)`，形状半径处高度为 0，而系统自己的障碍定义是 `min_obstacle_height`=0.10；(0,-2) 那块差 0.118 m（<1 格），任何基于 0.15 m 栅格的规划器都不可能满足。已改为 `radius*(1-thresh/h)`，与陨石坑早就分开发 `hazard_radius` 的做法一致。commit `b3c697f`。**验算**：margin 0.25 后吸附中心距石心 1.68 m，被校验朝向 +0.25、偏 10° +0.13、偏 16° +0.08、车角正对 −0.02（此时 `rotationValid` 也判致命会拒绝旋转，两者自洽）。

**2026-07-31（第四跳）：原地旋转只校验终点朝向，车角会在转弯途中扫进障碍。** `min_point_distance` 修好后碰撞从 4 降到 1，剩下那条的数字精确指向旋转扫掠：车停在 (-1.67,-2.07)，(0,-2) r=0.65 的石头，圆心距 1.6715；半对角 hypot(0.9,0.75)=1.1715，1.6715−1.1715−0.65 = **−0.150**，实测 clearance −0.14。同一位姿在 yaw=0 是 +0.12、yaw=90° 是 +0.27——**只有角对着石头时为负**，所以一定是转过去了。三处漏检（都已修，commit `1763eef`）：`transition()` 的 `direction==0` 分支只查 `yawForBin(to.t)` 一个朝向；`recoveryRotate` 只查 `target_yaw`；`primitiveValid` 的 `stride` 由 `prim.length` 推，原地基元 length≈0 ⇒ stride 巨大 ⇒ 只查端点。**弧线基元一直是采样扫掠的，唯独扫得最宽的纯旋转没查。** 修法：新增 `rotationValid(x,y,yaw0,yaw1,...)`，按「车角走过 ≤1 格(0.15m)」定采样数（一个 bin 0.3927 rad ⇒ n=4），两处调用换掉；`primitiveValid` 的 stride 改为 `r/(step_len + 每样本转角×半对角)`。代价：旋转边的 footprint 检查从 1 次变 ~4 次，plan_ms 预计翻倍（原 mean 14.7/max 161.9，断言阈值 mean<1000/max<1200，有余量）。**别把 snapGoal 也改成要求整圈可转**——那会要求目标离石心 ≥1.95 m，属于没有实测支撑的加码。

**通用教训（这四跳的共性）**：碰撞判据用真值圆、规划器用 0.15 m 栅格，两者**天然差一格**。凡是「差值 < 0.15 m」的失败，先问「这是不是量化误差」，而不是继续收紧规划器；反过来，凡是要求规划器贴边通过的设计（吸附到最近合法位姿），都必须显式留出 ≥1 格的余量。

**2026-07-31（第三跳）：GroundGrid 对 3.46 m 以内的障碍完全失明，是碰撞的根因。** `minDistSquared=12.0f`（原 `GroundSegmentation.h:93` 硬编码）在 `GroundSegmentation.cpp:296` 把近场点 `continue` 掉，而这一句在 `maxGroundHeight/minGroundHeight/points/m2` 更新之前、只在 `pointsRaw`(:293) 之后。连锁：`points=0` → `detect_ground_patch` 永远早退；`maxGroundHeight` 停在 `lowest()` **但它是有限值**，于是 `:647` 分支照进，`obstacle_height=max(0,lowest()-center)=0`、`obstacle_confidence*=0.7` 两帧跌破 0.45；只剩 `step_height` 撑着，而 `interpolate_cell:516` 每帧把 `ground` 往邻域均值抹。3.46 m 是轿车车顶雷达的尺度常数，我们整车才 1.8 m。已提为 `min_point_distance` 配置项（默认仍 sqrt(12)，`config/lunar_system.yaml` 覆盖为 1.2 m = 车体半对角），commit `e683d20`。**耦合警告**：代价图的 `sensor_blind_radius`(2.5 m) 把车周圆盘判为已知，只有在 `min_point_distance < 2.5` 时才安全；只改其一 = 放心撞看不见的石头。

**取数路径（反复咬人，记住）**：rostest 终端只回显断言消息，指标表和逐 trial 明细都不在里面；`roslog/rostest-bjm-*.log` 是 **rosmaster** 日志，只有 `N errors, M failures` 一行。真东西在 `~/.ros/planner_metrics_<scenario>.json` 和 `~/.ros/log/latest/lunar_pipeline_test*.log`。碰撞断言已改成自带肇事目标/位置/石头坐标（commit `e0b40e8`，`_describe_collision`），以后不用翻文件。

**2026-07-31（第二跳）：换成真射线雷达后规划全线失败，车一步没动，原因在代价图不在感知。** 旋转雷达按环采样，环间地面间距 ≈ (h + r²/h)·环间角；h=1.0 m、64 线 0.43° 时 5 m 处 0.2 m、8 m 处 0.48 m、14 m 处 1.5 m、20 m 处 3.0 m，而栅格是 0.15 m。`GroundSegmentation.cpp:132` 的 `observed` 是 `pointsRaw` 的 3×3 膨胀（±0.15 m），过 ~8 m 之后环间格全部 `observed=0`；`LunarTraversabilityNode` 的 `known` 一票否决 → `terrain_cost` 保持 NaN → 严格 `footprintValid` 拒绝所有目标位姿（`no valid pose within 3.00m`）。旧的笛卡尔均匀采样每格都有点，把这个假设一直遮着。修法（commit `5bc0c21`）：`known` 改为「`observation_fill_radius`(1.0 m) 邻域内存在实测且新鲜的格」，用 GroundGrid 本来就插值好的 `ground` 填环间空隙；另 `max_observation_age` 1.0→3.0（感知 ~1.05 s/帧，1.0 s 意味着漏一帧就作废）。**这不是感知侧的问题**：`observed` 层与整个可通行性节点都是规划侧加的，感知算法本身没退化。另一半是雷达几何盲区：最低束 -24.8° + 安装高 1.0 m ⇒ 车周 2.16 m 内永无回波，后继位姿的车体框全泡在洞里导致 `expanded=1 nodes`，加 `sensor_blind_radius`(2.5 m) 把该圆盘判为已知解决（commit `eab2bdf`）。

**2026-07-31：2026-07-29 那批「全绿」指标全部作废，必须重测。** 原因是仿真雷达而非规划器：`lunar_surface_sim` 原来发的是 0.20 m 笛卡尔均匀网格 + 伪造 `ring` 字段，对 0.15 m 地图只有 0.56 点/格。GroundSegmentation 接纳地面片要求点数超过 `0.25*patchSize*atan(1/格距)/0.2°`（`GroundSegmentation.cpp:420`，0.2° 定死在 `GroundSegmentation.h:92`），置信度分支（`:438`）还要求格内方差非零即至少 2 点——两条都从未满足。后果实测：`groundpatch` 全图 0.001，不确定项单独把每格代价顶到 ~20，98.4% 的致命格来自 `obstacle` 判据且排成周期 0.60 m = LCM(0.20,0.15) 的点阵；车开上去的那块石头实测 `step_height` 0.037 m，真值 0.55 m。已改为 64 线 × 0.2° 逐束步进求交（commit `eef3f00`），顺带拿到真遮挡。

**排除掉的错误假设**：`ring` 字段不是原因——它只在 `GroundSegmentation.cpp:296` 当过滤器用，`max_ring` 默认 1024，伪造的 0-31 全部通过。别再往 ring 上查。

**作废的旧数字（勿引用）**：规划成功率 开阔地 1.000(12/12)、含难目标 0.938(15/16)、避障恢复率 1.000、规划耗时 mean 14.7 / max 161.9 ms。**指标块只写进 rostest log 不上终端**，取法：`grep -A 20 "planner metrics over" ~/.ros/log/latest/lunar_pipeline_test*.log`；现在也会落盘 `~/.ros/planner_metrics_<scenario>.json`。

**仍然成立的结论**：`rostopic hz /lunar_planner/status` ~0.9Hz 是地图触发重规划的证据；感知 groundgrid 原 ~1053-1070ms/帧（点数从 4.9 万涨到 9.7 万后须重测）。

**恢复分母的实际来源**：设计时押注 (8,-5) 大岩石正心提供「难但可达」样本，实测它靠常规吸附就过了；真正走完整恢复链（`no_path → recovery_relax → success → success_snapped`）的是遮挡阴影目标 (0.9,-1.6)。改难目标清单时别把它删了。

**踩过的坑（都已修，避免重犯）**：
- `diag_pub_` 只 advertise 没声明 → 编译失败。新增 publisher 记得改成员行。
- **Rotate/BackOut 产出路径 ≠ 规划成功**：起初把恢复动作的 `true` 当成规划成功，导致清零失败计数、满足确认计数、阶梯永远在 Abort 前一级复位（绝望目标无限循环），且恢复率虚高。修法：`runRecovery` 用出参 `manoeuvre` 区分「到得了目标的路径」和「只是挪位置的路径」。
- **`recovery_none` 状态**：退出恢复时先清 `action_` 再拿它拼状态串。
- **避障恢复率分母**：正确判定不可达而 Abort 的条目既不是恢复成功也不是恢复失败，必须从分母剔除（新增 `recovery_aborts` 计数器）。不可 gaming——每次 abort 同时在规划成功率里记一次失败。另加断言「开阔地目标绝不允许 abort」兜底。
- **安全距离要按车体矩形算，不能用外接圆**：外接圆在车侧比车体宽 0.42 m，会把规划器正确认为安全的位姿报成碰撞（commit `579c9a9`）。
- **`test_*.py` 需要可执行位**，否则 rostest 报 "does not exist or is not executable"。Windows 端用 `git update-index --chmod=+x`。Ubuntu 端本地 `chmod +x` 会变成 git 修改导致 pull 冲突，`git checkout --` 丢弃即可。
- **`vprofile: v_peak=0.00` 不是 bug**：arc 模式有原地旋转基元（`StateLatticePlannerNode.cpp` transition 里 `to = from; to.t = ...`），纯旋转路径 v 本就该是 0，靠 w。`n=2 v_peak=0.00` 是 Rotate 恢复发出的两点路径。
- **地图是以车为中心的 60m 窗口**（`GroundGrid.cpp:57`，±30m），车开远后固定坐标目标会 `goal pose not in map`，属正确行为。
- **陨石坑 (5,2) 碗内目标真的无解**（坡度超限），正确行为是升级到 Abort。

**Why:** 这些结论来自昂贵的双机实测循环，重犯代价高；尤其那次指标作废是因为整条链路的输入端没被验证过，只验证了规划器自身。
**How to apply:** 改规划器恢复逻辑或指标定义时先对照本条；新增 hard 目标要区分「难但可达」（喂恢复率分母）和「真不可达」（验证 Abort）。看到「感知输出可疑」时，先查仿真传感器是否满足感知算法的密度假设，再怀疑算法。
