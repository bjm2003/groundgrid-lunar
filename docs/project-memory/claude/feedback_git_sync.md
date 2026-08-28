---
name: Git sync + dual-machine workflow
description: 用户要求代码改动在合适时机及时同步 GitHub，并每次明确给出两台机器各自要做什么
type: feedback
originSessionId: b4d9fbc1-08f5-4fcf-865c-1871d0c38723
---
改完代码、达到一个可验证节点时，主动 `git add`(只加相关源文件，勿带 `.claude/settings.local.json` 等本机文件) + `commit` + `push` 到 GitHub，不用每次等用户开口。每次同步后，明确分点告诉用户**两台机器分别要做什么**：
- Windows 开发机（`D:\groundgrid-main`）：我这边负责 commit/push；若 push 失败给出重试/代理方案。
- Ubuntu 编译运行机（`lyq@bjm`，`~/lunar_ws/src/groundgrid`）：`git pull` + 重编(`rm -rf build/groundgrid devel/.private/groundgrid && catkin build groundgrid`) + roslaunch 验证，并说明看哪个日志/现象。

**Why:** 双机工作流经 GitHub 中转，用户在 Windows 写、Ubuntu 跑；及时同步能减少「改了没推、跑的是旧码」的空耗。用户 2026-07-28 明确提出此要求。
**How to apply:** 每完成一处可验证改动就走这套流程；网络在 China 区连 github.com 常超时(443)，push 失败时提示代理/VPN，不盲目重试。提交说明聚焦「为什么改」。
