#!/usr/bin/env python3
"""Turn the per-scenario metric JSON into one Markdown table for the report.

Reads whatever `run_planner_experiments.py` left in --in-dir and prints two tables --
rates, then distributions as `mean ± sd [min, max]` -- plus the limitations that have to
travel with the numbers. Nothing is recomputed here; the test node is the single place
where a metric is defined, so the table cannot drift from what was asserted.

    rosrun groundgrid summarize_planner_metrics.py > planner_metrics.md
"""

import argparse
import glob
import json
import math
import os
import sys

SCENARIO_ORDER = ["mixed", "flat", "dense", "slope", "negative"]

SCENARIO_LABEL = {"mixed": "mixed 综合", "flat": "flat 平坦区域",
                  "dense": "dense 密集障碍区", "slope": "slope 大坡度起伏区",
                  "negative": "negative 负障碍集中区"}

RATE_ROWS = [("规划成功率 (巡回)", "plan_success_tour"),
             ("规划成功率 (含难目标)", "plan_success_all"),
             ("进入目标距离阈值率（全部）", "reach_rate"),
             ("进入目标距离阈值率（巡回）", "reach_tour"),
             ("任务完成率（全部，双方确认）", "completion_rate"),
             ("任务完成率（巡回，双方确认）", "completion_tour"),
             ("避障成功率", "obstacle_avoidance"),
             ("近障恢复率", "near_obstacle_recovery"),
             ("资源占用超标率", "cpu_overrun"),
             ("异常发生率", "anomaly"),
             ("输出达标率", "output_conformance")]

STAT_ROWS = [("规划耗时 (ms)", "plan_ms"),
             ("路径长度偏差", "detour_ratio"),
             ("规划路径长度 (m)", "path_len_m"),
             ("实际行驶长度 (m)", "driven_m"),
             ("轨迹跟踪误差 RMSE (m)", "tracking_rmse_m"),
             ("安全距离均值 (m)", "clearance_mean_m"),
             ("安全距离最小值 (m)", "clearance_min_m"),
             ("CPU 占用 (%)", "cpu_pct"),
             ("常驻内存 RSS (MB)", "rss_mb")]

LIMITATIONS = """\
## 已知局限

- **进入距离阈值不等于完成任务**：`reach_rate` / `reach_tour` 记录曾进入请求目标
  0.5 m 范围；`completion_rate` / `completion_tour` 才要求规划器和跟踪器双方确认完成，
  并排除超时或中止。吸附目标可能完成而未进入原始目标范围。旧报告缺少完成率时显示
  `n/a`，不能从旧到达率推断任务完成率。
- **感知无法区分坑与石**：`GroundSegmentation` 的 `step_height` 是窗口内的无符号极差，
  `obstacle_height` 只取正向，所以负障碍场景测的是「凹陷能否被当成障碍避开」，不是
  「能否被识别为凹陷」。
- **可通行区是「扫过」而非「逐格测到」**：旋转式雷达按环采样，1.0 m 安装高度下环间地面
  间距在 5 m 处 0.2 m、14 m 处已达 1.5 m，远大于 0.15 m 栅格。`LunarTraversabilityNode`
  因此把「1.0 m 邻域内存在实测格」判为已知，用 GroundGrid 插值出的 `ground` 填环间空隙。
  代价是遮挡阴影边缘最多 1.0 m 会被误判为已知；超出约 16 m（环距 > 2×1.0 m）之后地图
  如实转为未知，公里级路点必须靠边走边看，不能一次性给远目标。
- **车下 2.2 m 是雷达盲区**：最低一束 -24.8°、安装高 1.0 m，落点在 2.16 m 外，盲区内地面
  永远无回波。代价图把这个圆盘直接判为已知可通行——车确实站在上面。立在盲区里的正障碍
  仍然可见（0.5 m 的石头在 2 m 处会被更高的束打到），**前提是 `min_point_distance` 小于
  这个半径**；该参数上游默认 3.46 m（轿车尺度），本项目按车体半对角改为 1.2 m，两者是
  耦合的，只改其一会让车放心撞上看不见的石头。
- **雷达模型是理想单回波**：`lunar_surface_sim.publish_cloud` 按 HDL-64e 的 64 线 ×
  0.2° 方位角逐束步进求交，遮挡是真的，但没有月尘散射、多回波、强度衰减与测距噪声
  随距离的增长。掠射角下真实雷达的失效率会高于这里。
- **资源占用只含规划器**：CPU/RSS 由规划节点读 `/proc/self/*` 自报。感知节点（每帧
  处理约 9.7 万点，才是 CPU 大头）不在这组数字里。
- **地图是以车为中心的 60 m 滚动窗口**，固定坐标的远距目标离开窗口后会被正确拒绝；
  公里级路点需要全局地图落地后重测。
- **上坡方向的感知距离由地形决定**：1.0 m 的传感器高度在 8.5° 上坡上，最平的一束
  （+2.0°）也只能打到约 9 m 外。`slope` 场景 24 m 处的点密度低于 GroundGrid 的地面
  片门限，那一带的可通行性判定不可信——这是几何事实，不是仿真缺陷。
"""


def load(in_dir):
    runs = {}
    for path in sorted(glob.glob(os.path.join(in_dir, "planner_metrics_*.json"))):
        try:
            with open(path) as handle:
                report = json.load(handle)
        except (OSError, ValueError) as exc:
            print("skipping %s: %s" % (path, exc), file=sys.stderr)
            continue
        runs[report.get("scenario", os.path.basename(path))] = report
    return runs


def fmt_rate(value):
    if value is None or (isinstance(value, float) and math.isnan(value)):
        return "n/a"
    return "%.3f" % value


def fmt_stat(stat):
    if not stat or not stat.get("n"):
        return "n/a"
    return "%.2f ± %.2f [%.2f, %.2f]" % (stat["mean"], stat["sd"], stat["min"], stat["max"])


def table(header, rows, columns):
    lines = ["| %s | %s |" % (header, " | ".join(columns)),
             "|---|%s" % ("---|" * len(columns))]
    lines += ["| %s | %s |" % (label, " | ".join(cells)) for label, cells in rows]
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--in-dir", default=os.path.expanduser("~/.ros"))
    args = parser.parse_args()

    runs = load(args.in_dir)
    if not runs:
        print("no planner_metrics_*.json in %s; run run_planner_experiments.py first"
              % args.in_dir, file=sys.stderr)
        return 1
    order = [s for s in SCENARIO_ORDER if s in runs] + \
            [s for s in sorted(runs) if s not in SCENARIO_ORDER]
    columns = [SCENARIO_LABEL.get(s, s) for s in order]

    print("# 规划模块对比验证实验\n")
    print("| 场景 | %s |" % " | ".join(columns))
    print("|---|%s" % ("---|" * len(columns)))
    print("| 试验次数 | %s |" % " | ".join(
        str(runs[s].get("counts", {}).get("trials", "?")) for s in order))
    print("| 碰撞次数 | %s |" % " | ".join(
        str(runs[s].get("counts", {}).get("collisions", "?")) for s in order))
    print("\n## 比率指标\n")
    print(table("指标",
                [(label, [fmt_rate(runs[s].get("rates", {}).get(key)) for s in order])
                 for label, key in RATE_ROWS], columns))
    print("\n## 分布指标（均值 ± 标准差 [最小, 最大]）\n")
    print(table("指标",
                [(label, [fmt_stat(runs[s].get("metrics", {}).get(key)) for s in order])
                 for label, key in STAT_ROWS], columns))
    budgets = {runs[s].get("cpu_budget_pct") for s in order}
    print("\nCPU 预算：%s%%（超标率的判定阈值）\n"
          % "/".join(str(b) for b in sorted(b for b in budgets if b is not None)))
    print(LIMITATIONS)
    return 0


if __name__ == "__main__":
    sys.exit(main())
