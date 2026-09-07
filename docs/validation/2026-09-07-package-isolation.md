# c72eb41包验证：重复执行导致证据覆盖，默认弧线仍有失败

## 接收证据

- p0-package-c72eb41-Au9N2Q.tar.gz，239561字节，SHA-256
  3cffc9ffbd4948d915d96ad03bdbedc16884aea3df0d436ffa8a0ac6c2828340。
  此为手工tar，无逐文件清单；安全检查路径后解普通文件于
  build/received-p0-package-c72eb41-Au9N2Q，便利软链接不跟随。
- commit.txt和最终指标均为c72eb4131c18f574d42ff6dff2e90a0e91ec1873，
  跟踪工作区干净；JSON的run_id匹配p0-package-c72eb41-Au9N2Q。
- CMakeCache确认Release、CATKIN_ENABLE_TESTING=ON、/usr/bin/python3，
  CATKIN_TEST_RESULTS_DIR为此次新目录，构建11.9秒成功。构建警告是已有
  CMake策略/依赖兼容提示，未发现本包编译错误。

## 两次运行不能混为一轮

1. package-test.log：rostest-bjm-1467124，两个ROS断言均passed，TEST_RC=0。
   catkin内置汇总仍因结果目录在构建树外而显示0 tests，不构成独立通过依据。
2. 助手随后提供的ctest --output-on-failure没有筛选条件。实际执行14个注册项：
   1个ROS套件、5个Python套件、8个纯C++检查，不是预期的8个C++检查。
   rostest-bjm-1493935是第二个独立ROS进程，(8,-5) Abort，因此CTEST_RC=8。
3. 两次共用输出目录及run_id。第二次覆写原JSON和XML，最终RESULTS_RC=1对应
   第二次失败。最终XML合计54项：51项Python通过、2项pipeline中1项失败，另有
   1个启动器结果。8个C++目标全部通过，ctest-output中保留其完整输出。

不能称“C++失败”，也不能声称第二次失败只是统计误报。第一次成功日志保留，
但其完整JSON/XML已被覆盖，无法重建独立指标基线；不挑选第一遍替代第二遍。
这次流程疏漏来自助手命令，用户执行与给定命令一致。

## 第二次真实导航失败

实际配置reachable_cost/arcs。12/12巡回进入请求半径且完成，前两个困难任务
完成吸附终点。第三困难目标goal_id=16，从
(-1.691325795,-0.923672749,yaw=0.953943048)开始，8.613秒后Abort，位移0。
attempt24–26为goal_invalid；Relax后attempt27–30为start_no_successor，
均只扩展根节点、root_successors=0，计算约5.5–7.5ms。不是1秒搜索耗尽或
35秒走不完路线的问题，也未产生可执行名义路线。

BackOut检查日志拒绝当前/候选扫掠：cell=(-0.625,-2.025)，cost=100，
gx=-0.6072、gy=-0.0169、slope=31.277度，在margin=0.25m下拒绝。
这些是地图记录的已知危险，不允许为了通过测试直接放宽或跳过。
归档未启用精确规划输入快照，不能用这些少量单元值重建全图、判断拒绝是否
来自地图更新/起点姿态/历史退路交互，或宣称已找到可安全执行的修复路线。

无解(3,0)也正确Abort；碰撞0、陈旧0、原子合规率1.0。这些安全指标不能抵消
可解任务失败。此前425205a动力学通过是另一模式的有限样本，仍保留但不互相覆盖。

## 本批限定改动

新增run_planner_experiments.py --package-only自动入口：

- 同一新结果目录中Release构建、固定系统Python和独立XML目录。
- CMake只给八项C++检查添加groundgrid_selfcheck标签；先核对标签清单，再执行，
  核对八项实际Passed和完整日志，禁止混入ROS/Python或接受0 tests。
- catkin包测试只运行一次；JSON直接进入新目录，XML和输入快照同轮保存。
  调试日志和快照默认在此入口开启；不改普通运行的控制参数或验收时限。
- 直接审核所有登记的Python XML和pipeline XML、目标/提交/运行身份、快照
  写盘计数及每个尝试对应的载荷。即使命令错误返回0，断言失败仍是失败。
- 每阶段保存命令、工作目录和返回码；成功/失败/中断归档均有SHA-256清单。
  新实验不继承包测试的输出位置，禁止从旧~/.ros补一个貌似成功的JSON。

Windows验证：新增12项包验证回归，连同现有测试共63项通过；覆盖成功单次调用、
错误返回码伪通过、缺失Python XML、旧JSON、快照丢失、混入ROS的CTest清单、
构建/C++失败、中断、0项CTest伪通过、CMake标签/launch输出入口及环境隔离。
24份Python语法、10份XML解析、现有七个纯C++检查通过，六份Claude散列不变。
这些是无ROS回归，不宣称新自动入口已在Ubuntu成功执行。

导航算法、吸附范围、安全间距、恢复窗口和测试阈值本批未变。下一步仅运行
一次新自动入口，既验证修复后的收集流程，也采集后续定位所需的精确输入；
即使下一次通过，也不能抹去本轮的弧线偶发失败证据。正式五场景基线暂缓，
不合并main。
