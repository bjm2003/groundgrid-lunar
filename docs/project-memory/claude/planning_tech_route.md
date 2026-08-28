---
name: Planning tech route
description: 规划模块技术路线来源文档，及各技术要点当前在代码中的落地情况
type: reference
originSessionId: a7705bb8-c2fe-4c8f-b882-44fcb85ff798
---
规划模块技术路线权威来源：`技术要点清单（总）(改)(6).docx`（含16项技术要点、11项软件设计、对比验证实验清单）。中文部署流程见 `LUNAR_DEPLOYMENT_GUIDE.md`。

当前已落地的规划代码（src/，均在 CMakeLists 中构建，LunarSystem.launch 串联，config/lunar_system.yaml 配参）：
- `LunarTraversabilityNode.cpp` — 通行风险评估(要点5)+局部代价地图(要点6)。消费感知 grid_map，产出 terrain_cost 图层与 OccupancyGrid costmap，含 lethal/unknown 标记。
- `StateLatticePlannerNode.cpp` — 状态栅格(7)+多目标代价(8)+碰撞检测(9)+混合A*(10)。(x,y,yaw) 搜索，前进/后退/转弯/原地运动基元，整车 footprint + 纵/横向坡度校验。
- `LunarPathFollowerNode.cpp` — 路径提取/轨迹跟踪(软件设计8)。纯追踪式差速控制器，坡度降速，失效安全停车。订阅 `/lunar_planner/velocity_profile` 用规划 v 作前馈 + `SkidSteerModel::inverseCommand` 滑移补偿。
- `scripts/lunar_surface_sim.py` — 轻量陨石坑/岩石地形仿真，用于端到端测试。速度上限参数化(默认 v±1.5/w±0.8)，可注入 ICR+滑移"标称真值"(默认恒等)。

**要点1/2/3 已落地（滑移动力学 + 运动基元库，纯 C++ 无 ROS，可剥离到 Atlas）：**
- `include/groundgrid/{SkidSteerModel,MotionPrimitive,MotionPrimitiveLibrary}.h` + `src/{SkidSteerModel,MotionPrimitiveLibrary}.cpp` — ICR 滑移模型 + 离线基元库(生成/save/load/O(1)查询)。selfcheck 全 PASS。
- `src/generate_motion_primitives.cpp`(离线生成器) + `src/skidsteer_selfcheck.cpp`(自检)，均 g++ 编译通过。`config/motion_primitives.dat` 已生成(256基元/16 bins)。
- `StateLatticePlannerNode.cpp` — A/B 开关 `use_dynamics_primitives`：真则加载基元库、`primitiveValid()` 沿基元样点复用 footprint 校验、`plan()` 输出稠密路径 + `/lunar_planner/velocity_profile`(v,w 序列)；假则走原理想圆弧。
- `scripts/identify_skidsteer.py` + `launch/IdentifySkidSteer.launch` — 最小二乘辨识 alpha_v/alpha_w/x_icr/slope_slip_gain/slope_grade_gain，对注入真值报复原误差。
- `config/lunar_system.yaml` 新增 `skid_steer_model:` 段；速度提到 1.39 m/s(5km/h,指标10)。`test/test_lunar_pipeline.py` 扩展记录规划耗时/跟踪RMSE/速度剖面(A/B)。
- **注意**：ROS 节点(.cpp)本机无法 catkin 构建/roslaunch 验证，需用户在 Ubuntu/ROS Noetic 上跑；纯 C++ 部分已本机 g++ 验证。

主要**未做/部分**：3D占据栅格(4)、公里级路点约束全局规划(11)、近障恢复逻辑(13/软件9)、视场外已路过障碍记忆(15)、地图分辨率影响实验(14)、模型压缩/NPU加速(16)、运动基元管理模块/内存池/导航地图文件读写(软件2/5/6)。

**Why:** 是判断规划进度与下一步的基准。
**How to apply:** 用户问“进度/下一步做什么”时对照此清单；文档随实现推进会变，动手前重新核对 docx 与 src 现状。
