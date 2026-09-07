# 7bdc5af：独立完整包验证通过，五场景基线仍待执行

## 证据与完整性

- 收到 `p0-package-7bdc5af-20260907-163147.tar.gz`，76776019字节。
  SHA-256：`7d1d6097f21a34c5d0171406ef43620ae54f1574b28a6291e24be4eee0e33c5c`。
- 安全检查成员路径、重复名、类型及展开大小后，仅解普通文件到
  `build/received-p0-package-7bdc5af-20260907-163147`；不跟随1个ROS便利软链接。
  156份清单文件SHA-256全部匹配，文件集合与清单相同。
- 提交为 `7bdc5afeae7cc212b3be2a6152792860d7c7926e`，Ubuntu跟踪工作区干净。
  归档源配置与该提交逐字节一致；环境、套件、运行及指标的身份相符。
  本次 `run_id=b238fc0b661f45beb4d04ec3b285c8a6`。
- Ubuntu Noetic、Python 3.8.10、CONDA_PREFIX为空。CMakeCache确认为Release、
  CATKIN_ENABLE_TESTING=ON、/usr/bin/python3、此次独立XML目录。增量构建成功，
  1.5秒；保留已有PCL/CMake策略兼容警告，不把带警告称为无警告构建。

## 包门槛

本次自动入口恰好执行：构建、标签清单、标签C++检查、一次catkin包测试、独立XML汇总。
五个命令均返回0。只存在一轮pipeline ROS启动记录（rostest-bjm-1606168），
未在裸CTest中重复执行ROS；没有从旧的 ~/.ros 复制指标。package/run.json与
suite.json内的记录一致，archive_errors为空，SUITE_RC=0。

- 标签 `groundgrid_selfcheck` 实际执行8项C++检查，全部通过；含安装版grid_map
  的循环索引对照，完整CTest输出保留。耗时0.63秒，不混入ROS或Python注册项。
- 6份Python XML共63项通过：辨识5、仿真时序5、任务指标7、观测18、归档16、
  包验证12。每份XML均达到按当前源代码测试方法得出的最低数量。
- 两项ROS断言均通过，另有1项启动器XML；独立catkin_test_results汇总66项、
  0 errors、0 failures、0 skipped。该66项不包含前面的8项C++自检。
- 包阶段总计225.272秒；pipeline两项断言合计218.275秒。
- writer-summary：submitted=written=39，dropped=failed=0；每个任务关联尝试
  都有二进制输入、摘要及轨迹文件，无partial载荷。此处审核了完整性，未将它
  描述为新的39份同输入重放性能认证。

日志中三个rosmaster XML-RPC ConnectionRefusedError出现在16:35:32注销节点阶段，
目标服务已下线；断言及正常运行结果不受影响。原日志保留，不声称全程没有错误文本。

## 此次默认弧线闭环

实际使用 `reachable_cost / arcs / mixed / n_trials=3`，原矩形足迹、安全间距、
吸附范围与测试超时不变。包入口虽在顶层参数中保留五场景枚举默认值，但其实际
只运行配置内mixed短套件；不能误认成五场景全通过。

- 12/12巡回进入原请求半径，且12/12任务完成。
- 3/3可解困难任务完成所选安全吸附终点。按顺序耗时11.823、6.023、14.613秒；
  到原请求坐标的最终距离分别1.415、3.166、2.846米，三项原始坐标reached均为false。
- 无解(3,0)在12.823秒内按恢复流程进入Abort；不计作可解任务完成。
- 碰撞0、任务跟踪陈旧0、目标未确认0；原子输出合规率1.0，角速度曲线覆盖1.0。
- 可解任务规划成功率1.0，巡回到达与完成率均1.0；总体完成15/16，含故意无解项。
  不能用整体reach_rate=0.75误判三项吸附任务未执行，也不能将其标记为原点到达。

性能去重口径：38次任务关联planning_attempt平均32.697ms、p95=115.961ms、
最大157.048ms。加上普通闭环预检目标为39次，平均37.044ms、p95=157.048ms、
最大202.196ms；两种口径明确分列，均为Ubuntu记录的完整尝试而非离线计时。
任务耗时包含规划、恢复与执行，不能拿几十毫秒搜索耗时代替任务完成时间。
规划器进程CPU样本最大32.9%，不代表目标平台感知加规划CPU<40%达标。

## 下一步与保留的问题

这是新包验证流程的Ubuntu通过证据，不是c72eb41偶发失败的根因修复。
上一份无快照的弧线Abort仍是真实失败，见
[重复执行及失败记录](2026-09-07-package-isolation.md)；不能拿本次成功抵消它。
本批没有运行代码改动，不延长超时、不扩大吸附范围、不降低安全或断言门槛。

继续保持已构建的7bdc5af采集五场景正式基线候选；不要求为了本文档变动拉取或重编译。
每场景n_trials=10、默认弧线、独立进程，保存全部结果，失败时先分析本轮完整快照，
不反复启动只挑通过样本。整组采集结束后统一审核，当前formal_baseline仍为false。

```bash
source "$HOME/lunar_ws/devel/setup.bash" && rosrun groundgrid run_planner_experiments.py --scenarios mixed flat dense slope negative --n-trials 10 --repeat 1 --snap-strategy reachable_cost --primitive-mode arcs --capture-inputs --debug-control --out-dir "$HOME/p0-baseline-arcs-7bdc5af-$(date +%Y%m%d-%H%M%S)"
```

五场景尚无本阶段完整结果；其他场景若低于任务书99%必须保留缺口，即便已有分级
rostest门槛通过也不代表任务书达标。此前三轮弧线、一次动力学与辨识按各自提交
保留，不冒充在7bdc5af上重新完成。目标平台CPU、速度、公里级规划、历史地图、
融合感知、Atlas部署与实地验证仍未完成。验证结束前不合并main。
