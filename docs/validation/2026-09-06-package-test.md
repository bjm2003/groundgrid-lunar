# 2026-09-06 fabd8a0 包测试：旧策略闭环仍失败

## 证据与归属

- 包：p0-package-fabd8a0-20260906-210359.tar.gz。
  SHA-256：1916f8fc03f1510f24d6ac0ded1ea08b238ed4c86ce978ddf7a8e005e48eae83。
- 解包至 build/received-package-20260906-210359，只解普通文件、跳过软链接。
  此包由用户手工 tar，不含逐文件散列清单；上面的散列标识收到的整个归档。
- commit.txt 与指标 commit 均为 fabd8a0d11b1fb6ba75be60d12823f2039da99ba；
  指标 run_id 为 p0-package-fabd8a0-20260906-210359，确认为本轮而非旧 JSON。
- CMakeCache：Release、/usr/bin/python3、CATKIN_ENABLE_TESTING=ON。
  完整 build.log 显示构建成功，1.5 秒；警告为依赖包的旧 CMake 版本/策略兼容提示，
  不是本次闭环失败的证据。未为消除警告改动系统依赖。
- 指标有效参数及 planning_attempt 均为 legacy_nearest / arcs。
  新策略已编译存在，但这次 catkin 包测试没有选择 reachable_cost。

## 两个返回码为什么不同

本次命令将 CATKIN_TEST_RESULTS_DIR 指向独立归档目录。package-test.log 中
catkin test 的 results 阶段仍在 build/groundgrid 执行无参数 catkin_test_results，
输出为 `Summary: 0 tests, 0 errors, 0 failures`，最后返回 TEST_RC=0。

显式指定归档 test_results 后，正确统计到 50 tests、0 errors、1 failure，
RESULTS_RC=1。7 份 XML 及对应断言日志确认这一失败真实存在。
这是助手之前的目录隔离命令与 catkin 内置汇总路径不一致，不能把前者当通过。
今后必须以明确目录、非空且齐全的 XML 和实际断言为准；单一 shell 返回码不够。

本次补强现有批量归档验证器：直接读取 XML 的 errors/failures 及失败元素，
pipeline 不足2项测试也判失败，不依赖 subprocess 返回码。新增零/非法计数、
隐藏失败元素、两级进程返回0但实际断言失败的回归测试。50 项纯 Python 测试、
22 个 Python 语法及10份 XML 检查在 Windows 通过；将本次真实归档送入验证器，
也准确拒绝了该 pipeline XML。没有修改规划器、跟踪器或阈值。

## 实际结果

- 47 项 Python 单元测试全部通过：辨识 5、仿真测量时间 5、指标 7、
  任务观测 18、归档 12。
- 两个 ROS pipeline 测试中，普通闭环通过，多目标闭环 1 项失败。
  50 总数还包含 1 个 rostest 启动器结果；不将其算作独立功能覆盖。
  本包没有八个纯 C++ 自检的完整日志，不能据此补记它们已执行。
- mixed / n_trials=3：12/12 巡回到达与任务完成，前两个困难目标完成，
  第三个困难目标 (8,-5) 在 12.815 秒后 Abort。无解目标 (3,0) 正确 Abort。
- 碰撞 0、follower_stale=0、output_conformance=1.0。
  不能用这些已达标项覆盖可解困难任务失败；包测试总体仍为失败。

第三困难目标从 (-0.841,-0.153,yaw=0.490) 开始；连续尝试出现 goal_invalid、
start_no_successor，退回后一次 search_timeout。最后一次搜索虽导出
13.409 m 路线（倒车 10.763 m），但未成为恢复确认后的名义任务成功，随后 Abort。
仅实际驱动 0.231 m。本轮是搜索/恢复中终止，不是跑满35秒的行驶超时。
两次完整尝试分别约 1000.792、1000.536 ms，<1秒要求也未满足。
本包未启用快照，不能仅用文本记录重新构造该地图输入来断言新策略必然解决。

## 后续决定

保留此旧策略失败，不挑选先前成功轮替代，不改验收阈值或延长恢复时限。
不再盲目重复旧策略包测试；已通过 Ubuntu 重放关的新策略需要真实闭环验证。

下一步用现有 run_planner_experiments.py 显式指定 reachable_cost，先进行
五参数辨识，随后三个独立 arcs / mixed / n_trials=3 进程，保留全部成功/失败、
控制日志和精确输入快照。该脚本显式汇总每轮新 XML 目录并核对完整性，
不会使用上面的零测试汇总作为通过依据。

这调整了原先“包测试全绿再开始新策略闭环”的执行顺序：47 项单元测试已过，
剩下的正是旧策略目标选择/搜索问题，需先验证为该问题实现的新策略。
完整包测试零失败这一交付条件没有取消；新策略闭环审核后再决定默认启用，
并重新核验完整包测试，然后继续动力学回归及五场景基线。当前不合并 main。
