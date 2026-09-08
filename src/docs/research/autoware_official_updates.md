# Autoware 官方更新对比：本地 1.7.0 基线至官方 1.9.0

> 调查日期：2026-09-07  
> 本地仓库：`git@github.com:LeslieJay/new_car_autoware.git`  
> 本地 HEAD：`f5271ffd072b4c4b6f4bf0850ef416514a04f6e7`

## 结论摘要

本地仓库不是 `autowarefoundation/autoware` 的普通 fork，而是将多个上游仓库平铺到 `src/` 后继续开发的内部仓库。因此不能用单一 `git merge-base` 得到全部差异。被根目录 `.gitignore` 忽略、但实际存在的 `repositories/autoware.repos` 是最可靠的导入版本证据：本地来自官方 **Autoware 1.7.0 版本组**，其中 Universe 和 Launch 均为 0.50.0，Core 为 1.7.0。

本文以官方最新稳定 **Autoware 1.9.0** 为可复现终点，归纳本地基线之后的主要新功能和 bug 修复。官方 `main` 在各稳定标签之后仍持续前进，不适合作为固定升级验收线；若要跟随 `main`，应在实际升级日重新生成差异。

官方 GitHub compare 给出的版本跨度量级为：Universe 从 0.50.0 到 0.52.1 共前进 **597 commits**，Core 从 1.7.0 到 1.9.0 共前进 **304 commits**，Launch 从 0.50.0 到 0.52.0 共前进 **132 commits**。本文不是逐提交抄录，而是从这些提交中筛选影响功能、安全、稳定性和实际集成的高价值项目。

高优先级关注点如下：

1. 反向机动、`direction_change`、area primitive 路由已形成跨 Core、Universe、Launch 的完整功能链。
2. 轨迹验证/安全过滤器、行为规划超时诊断、静态避障和换道可靠性均有显著增强。
3. 感知侧新增 polar voxel filter，并扩展多目标跟踪、BEVFusion、PTv3 和交通灯能力。
4. Agnocast/CIE 的官方接入范围大幅扩大，Core 1.9.0 又补充 service/client、Timer、tf2、diagnostics 等基础能力。
5. 0.50.0 之后修复了多项卡死、空数据崩溃、空指针、数据竞争、轨迹顺序和碰撞漏检问题，建议优先回移安全相关修复。

## 基线证据

| 组件 | 本地锁定版本 | 本地基线 tag 提交 | 官方 1.9.0 版本组 | 官方 tag 提交 |
|---|---:|---|---:|---|
| Autoware meta | 1.7.0 版本组 | [1.7.0 release task](https://github.com/autowarefoundation/autoware/issues/6803) | 1.9.0 | [`1071878`](https://github.com/autowarefoundation/autoware/releases/tag/1.9.0) |
| autoware_universe | 0.50.0 | [`56e3d27`](https://github.com/autowarefoundation/autoware_universe/tree/0.50.0) | 0.52.0 | [`6e477c6`](https://github.com/autowarefoundation/autoware_universe/releases/tag/0.52.0) |
| autoware_core | 1.7.0 | [`6eb2c14`](https://github.com/autowarefoundation/autoware_core/tree/1.7.0) | 1.9.0 | [`f25f83c`](https://github.com/autowarefoundation/autoware_core/releases/tag/1.9.0) |
| autoware_launch | 0.50.0 | [`0f3946a`](https://github.com/autowarefoundation/autoware_launch/tree/0.50.0) | 0.52.0 | [`f942598`](https://github.com/autowarefoundation/autoware_launch/releases/tag/0.52.0) |

本地还锁定了 `autoware_msgs 1.11.0`、`autoware_adapi_msgs 1.9.1`、`autoware_internal_msgs 1.12.1`、`autoware_utils 1.5.0`、`autoware_lanelet2_extension 0.12.0` 和 `autoware_rviz_plugins 0.4.0`。官方 [Autoware 1.9.0 manifest](https://github.com/autowarefoundation/autoware/blob/1.9.0/repositories/autoware.repos) 分别升级至 1.13.0、1.9.2、1.13.0、1.9.0、1.2.0 和 0.6.0，并纳入更新的 Agnocast、System Designer 与独立的 Simple Planning Simulator。

调查时官方 meta `main` HEAD 为 [`47569464`](https://github.com/autowarefoundation/autoware/commit/475694641bbe9126f5776999427d7f8ead7a82a9)，其 manifest 已指向 Universe 0.52.1、Launch 0.52.0、Core 1.9.0。各源码仓库 `main` HEAD 为：Universe `6227b872cc59784e57ef37dcf34d9c04ccc1e472`、Core `4ef3d69a17a1a7a0a4ee02dbde1cbe84959bf030`、Launch `03740bc9782276a64610b491c225865c137124c3`。这些值仅记录调查时状态，不作为稳定升级基线。

### 统计口径与完整 compare

| 仓库 | 固定对比区间 | commits ahead | 官方 compare |
|---|---|---:|---|
| Universe | 0.50.0 → 0.52.1 | 597 | [完整 compare](https://github.com/autowarefoundation/autoware_universe/compare/0.50.0...0.52.1) |
| Core | 1.7.0 → 1.9.0 | 304 | [完整 compare](https://github.com/autowarefoundation/autoware_core/compare/1.7.0...1.9.0) |
| Launch | 0.50.0 → 0.52.0 | 132 | [完整 compare](https://github.com/autowarefoundation/autoware_launch/compare/0.50.0...0.52.0) |

Universe 0.52.1 是调查日 meta `main` 的 manifest 版本，略新于稳定 Autoware 1.9.0 所锁定的 Universe 0.52.0。因此，正文功能清单以 1.9.0 稳定组为主，统计则额外覆盖 0.52.1，以显示本地到调查日官方整合线的真实距离。

0.52.1 相对 0.52.0 仅包含一个补丁：移除 Diffusion Planner / Trajectory Ranker 的 builder pattern，以修复该组合的构建/集成问题（[commit `02a58920`](https://github.com/autowarefoundation/autoware_universe/commit/02a589200c1af644ca4b4cb3ed98695b4b62118b)）。

## 主要新功能

### 平台与中间件

| 功能 | 官方实现证据 | 对本地的意义 |
|---|---|---|
| Autoware System Designer | [Universe PR #12070](https://github.com/autowarefoundation/autoware_universe/pull/12070)，[Autoware 1.8 公告](https://github.com/orgs/autowarefoundation/discussions/7095) | 提供节点/系统的可视化设计与设计文件；本地 `src/core/autoware_system_designer` 不存在。 |
| Agnocast/CIE 大规模接入 | [Core 1.9.0 release](https://github.com/autowarefoundation/autoware_core/releases/tag/1.9.0) | 官方将零拷贝 IPC 接入数十个节点，并增加 service/client、Timer、tf2、diagnostic updater、ExactTime 和 launch wrapper 等能力。本地 manifest 仅锁定较早的 `backport-jazzy-support-v2.1.2`。 |
| 新 `autoware_command_gate` | [Core PR #1012](https://github.com/autowarefoundation/autoware_core/pull/1012) | 新包及不依赖 ROS 的 mode-dispatch helper/tests；迁移时需与本地定制的 `control_command_gate` 参数和 BYD 控制链做三方合并。 |
| 独立 Simple Planning Simulator 与 scenario demo | [Autoware 1.8 公告](https://github.com/orgs/autowarefoundation/discussions/7095) | 官方把简单规划仿真器独立并补齐场景仿真演示；本地没有相应独立 Core 仓库目录。 |

### 规划与控制

| 功能 | 官方实现证据 | 说明 |
|---|---|---|
| 反向机动完整链路 | [#12572](https://github.com/autowarefoundation/autoware_universe/pull/12572)、[#12609](https://github.com/autowarefoundation/autoware_universe/pull/12609)、[#12638](https://github.com/autowarefoundation/autoware_universe/pull/12638)、[#12668](https://github.com/autowarefoundation/autoware_universe/pull/12668) | 增加 area primitive 路由、VTL reverse、reverse goal、`direction_change` 模块以及 `allow_area` 下传。本地未发现 `direction_change` package。 |
| 车辆约束轨迹安全过滤 | [#12197](https://github.com/autowarefoundation/autoware_universe/pull/12197) | 在轨迹输出链增加车辆约束检查/过滤，提高不可执行轨迹的拦截能力。 |
| 不可跨越边界 departure filter | [#12587](https://github.com/autowarefoundation/autoware_universe/pull/12587) | 轨迹验证器可根据不可跨越边界过滤驶离可行驶区的轨迹。 |
| Trajectory validator shadow mode、评价表与耗时/debug marker | [#12478](https://github.com/autowarefoundation/autoware_universe/pull/12478) | 支持旁路评估新检查器，并提高验证结果和性能的可观测性。 |
| 行为规划与轨迹跟随输入超时诊断 | [#12075](https://github.com/autowarefoundation/autoware_universe/pull/12075)、[#12082](https://github.com/autowarefoundation/autoware_universe/pull/12082) | 为 BPP 和 trajectory follower 增加输入消息超时诊断，降低静默使用陈旧数据的风险。 |
| 静态避障策略增强 | [#12105](https://github.com/autowarefoundation/autoware_universe/pull/12105) | 可从 obstacle stop distance 开始避障，并完善转向灯与审批策略。 |
| 换道路径重建 | [#12280](https://github.com/autowarefoundation/autoware_universe/pull/12280) | 当车辆相对计划路径偏差过大时重建 lane-change path。 |
| 新优化器与学习型规划升级 | [#12300](https://github.com/autowarefoundation/autoware_universe/pull/12300)、[#12394](https://github.com/autowarefoundation/autoware_universe/pull/12394)、[#12348](https://github.com/autowarefoundation/autoware_universe/pull/12348) | 包括 acados MPT path optimizer、temporal trajectory optimizer，以及 diffusion planner v4 / TensorRT pipeline 和 planning factor 改进。 |

### 感知与定位

| 功能 | 官方实现证据 | 说明 |
|---|---|---|
| Polar voxel noise filter | [#12496](https://github.com/autowarefoundation/autoware_universe/pull/12496) | 新增极坐标体素噪声过滤器；本地未找到对应 package。 |
| 新 cluster/merger 能力 | [#12682](https://github.com/autowarefoundation/autoware_universe/pull/12682) | 扩展聚类与融合管线。 |
| Multi-object tracker 关联与形状增强 | [#12475](https://github.com/autowarefoundation/autoware_universe/pull/12475)、[#12672](https://github.com/autowarefoundation/autoware_universe/pull/12672)、[#12698](https://github.com/autowarefoundation/autoware_universe/pull/12698) | 增加极坐标关联、shape-aware spawn/association、bounding-box/polygon，以及 wheel/lateral anchor 等能力。 |
| ANIMAL/HAZARD 与静态 tracker 全链路 | [#12818](https://github.com/autowarefoundation/autoware_universe/pull/12818) | 从分类、过滤到跟踪扩展新类别；Launch 侧也有配套配置，不能只升级源码。 |
| BEVFusion 图像和类别处理增强 | [#12279](https://github.com/autowarefoundation/autoware_universe/pull/12279)、[#12637](https://github.com/autowarefoundation/autoware_universe/pull/12637)、[#12732](https://github.com/autowarefoundation/autoware_universe/pull/12732) | GPU 图像去畸变、按类别/距离过滤，以及 TRAFFIC_CONE/BARRIER 支持。 |
| PTv3 管线增强 | [#12362](https://github.com/autowarefoundation/autoware_universe/pull/12362)、[#12547](https://github.com/autowarefoundation/autoware_universe/pull/12547) | 多输入输出、source-cloud reconstruction/entropy，以及 backbone/head 拆分。 |
| 交通灯识别增强 | [#12076](https://github.com/autowarefoundation/autoware_universe/pull/12076)、[#12266](https://github.com/autowarefoundation/autoware_universe/pull/12266)、[#12492](https://github.com/autowarefoundation/autoware_universe/pull/12492)、[#12302](https://github.com/autowarefoundation/autoware_universe/pull/12302) | amber 支持、黄灯箭头处理、多相机一致性检查和新的 regression classifier。 |

## 主要 bug 修复

以下项目均位于本地 0.50.0 / 1.7.0 基线之后。它们是升级或定向回移时应优先复核的安全性、稳定性修复。

| 子系统 | 修复 | 官方证据 |
|---|---|---|
| Goal planner | 修复 node 卡死；避免空输出路径导致崩溃 | [#12177](https://github.com/autowarefoundation/autoware_universe/pull/12177)、[#12284](https://github.com/autowarefoundation/autoware_universe/pull/12284) |
| 静态避障 | ego 通过所有 shift lines 后正确转为 `SUCCEEDED` | [#12199](https://github.com/autowarefoundation/autoware_universe/pull/12199) |
| A* / freespace planning | 修复非单调轨迹点顺序；补上扩展圆弧中间点的碰撞检查 | [#11789](https://github.com/autowarefoundation/autoware_universe/pull/11789)、[#12403](https://github.com/autowarefoundation/autoware_universe/pull/12403) |
| Behavior path planner | early-return 时不再漏发诊断；manual override 后恢复 route lanelet | [#12214](https://github.com/autowarefoundation/autoware_universe/pull/12214)、[#12576](https://github.com/autowarefoundation/autoware_universe/pull/12576) |
| Crosswalk | 修复卡车判定和剩余通行宽度逻辑 | [#12224](https://github.com/autowarefoundation/autoware_universe/pull/12224)、[#12248](https://github.com/autowarefoundation/autoware_universe/pull/12248)、[#12249](https://github.com/autowarefoundation/autoware_universe/pull/12249) |
| Multi-object tracker | 改善 EKF 数值稳定性并修复 bicycle model、merge algorithm | [#12347](https://github.com/autowarefoundation/autoware_universe/pull/12347)、[#12628](https://github.com/autowarefoundation/autoware_universe/pull/12628)、[#12754](https://github.com/autowarefoundation/autoware_universe/pull/12754) |
| AEB / shape estimation | 空点云崩溃保护；AEB/collision detector 跳过空云，避免 PCL 告警刷屏 | [#12569](https://github.com/autowarefoundation/autoware_universe/pull/12569)、[#12670](https://github.com/autowarefoundation/autoware_universe/pull/12670) |
| No stopping area | 补声明 `predicted_objects` subscription 并修复空指针 | [#12860](https://github.com/autowarefoundation/autoware_universe/pull/12860) |
| External command selector | 修复数据竞争 | [#12887](https://github.com/autowarefoundation/autoware_universe/pull/12887) |
| Path generator | 防止当前 lanelet 之前的路径被截断 | [Core #979](https://github.com/autowarefoundation/autoware_core/pull/979) |
| BEVFusion Camera | 修复 `bev_pool` 投影错误 | [#12206](https://github.com/autowarefoundation/autoware_universe/pull/12206) |
| Euclidean cluster | 空输入仍连续发布；修复 invalid type | [#12257](https://github.com/autowarefoundation/autoware_universe/pull/12257)、[#12724](https://github.com/autowarefoundation/autoware_universe/pull/12724) |
| Traffic light | 多相机融合的信号元素置信度取 minimum；map detector 静默 fallback 改为显式异常 | [#12566](https://github.com/autowarefoundation/autoware_universe/pull/12566)、[#12434](https://github.com/autowarefoundation/autoware_universe/pull/12434) |
| Planning evaluator | 修复 DRAC 公式错误 | [#12124](https://github.com/autowarefoundation/autoware_universe/pull/12124) |

## Launch 0.52.0 的配套变化

这些变化决定新源码是否真正被启动和正确连线，升级 Universe/Core 时不可遗漏。完整清单见 [autoware_launch 0.52.0 release](https://github.com/autowarefoundation/autoware_launch/releases/tag/0.52.0)（tag `f942598d44b5769353167c76b784323d5c14c8c7`）。

### 新配置/功能

- side-shift 防驶出可行驶区域参数：[Launch #1828](https://github.com/autowarefoundation/autoware_launch/pull/1828)
- NDT `publish_loaded_map`：[Launch #1843](https://github.com/autowarefoundation/autoware_launch/pull/1843)
- `direction_change` 配置：[Launch #1857](https://github.com/autowarefoundation/autoware_launch/pull/1857)
- 仿真 dummy traffic-light publisher：[Launch #1859](https://github.com/autowarefoundation/autoware_launch/pull/1859)
- semantic-segmentation pipeline option：[Launch #1865](https://github.com/autowarefoundation/autoware_launch/pull/1865)
- `get_selected_lanelet2_map` 参数：[Launch #1813](https://github.com/autowarefoundation/autoware_launch/pull/1813)
- ANIMAL/HAZARD 的 tracker、filter、YOLOX、cluster 配置：[Launch #1864](https://github.com/autowarefoundation/autoware_launch/pull/1864)、[#1866](https://github.com/autowarefoundation/autoware_launch/pull/1866)、[#1867](https://github.com/autowarefoundation/autoware_launch/pull/1867)、[#1876](https://github.com/autowarefoundation/autoware_launch/pull/1876)、[#1884](https://github.com/autowarefoundation/autoware_launch/pull/1884)

### Launch bug 修复

- 修复 e2e simulator 的 CARLA launch 路径：[Launch #1841](https://github.com/autowarefoundation/autoware_launch/pull/1841)
- 修复 motion-planning container 的 acados `LD_LIBRARY_PATH`：[Launch #1842](https://github.com/autowarefoundation/autoware_launch/pull/1842)
- map launcher include 支持 substitutions：[Launch #1821](https://github.com/autowarefoundation/autoware_launch/pull/1821)
- 修正 PTv3 occupancy-grid input topic：[Launch #1871](https://github.com/autowarefoundation/autoware_launch/pull/1871)
- 修复启用 semantic segmentation、但未启用 multi-channel tracker 时的 merger topic 错位：[Launch #1895](https://github.com/autowarefoundation/autoware_launch/pull/1895)

## 建议的回移优先级

1. **P0：崩溃与并发问题。** Goal planner 空路径、AEB/shape estimation 空云、no-stopping-area 空指针、external command selector 数据竞争。
2. **P0：规划安全正确性。** A* 碰撞漏检/轨迹顺序、DRAC 公式、path generator 截断、crosswalk 通行宽度。
3. **P1：可观测性与陈旧数据保护。** BPP/trajectory follower 超时诊断、trajectory validator shadow mode 和处理耗时。
4. **P1：反向机动。** 作为 Core、Universe、Launch、消息接口的整体功能迁移，不建议只 cherry-pick 单一 package。
5. **P2：感知模型与类别扩展。** BEVFusion、PTv3、ANIMAL/HAZARD 需要同步模型文件、类别表、Launch 配置和消息版本。

## 限定与升级方法

- 本地工作树在调查时已有多处未提交修改；本次调查未修改这些文件，也未把它们作为官方差异。
- 版本 manifest 能证明初始上游快照，但本地随后有大量 BYD 私有提交；某项上游修复可能已被独立实现、部分回移或以不同方式替代。因此本文使用“本地基线之后的官方更新”，而不是声称每个补丁都百分之百缺失。
- 目录存在性已用于确认部分整包新功能（例如 `direction_change`、polar voxel filter、System Designer）未纳入；其余项目在回移前仍应按 package 做 `base(官方旧 tag) / local / upstream(官方新 tag)` 三方 diff。
- 不建议直接覆盖 `src/launcher/autoware_launch`、行为规划或 command-gate 配置；这些位置存在 BYD 参数和逻辑定制，应逐项合并并执行场景回归。

## 官方总清单

- [Autoware 1.9.0](https://github.com/autowarefoundation/autoware/releases/tag/1.9.0)
- [Autoware Universe 0.51.0](https://github.com/autowarefoundation/autoware_universe/releases/tag/0.51.0)（tag `d4d260983d357e1b2b34291d91933f9f4b53bf94`）
- [Autoware Universe 0.52.0](https://github.com/autowarefoundation/autoware_universe/releases/tag/0.52.0)（tag `6e477c645efec33f7909095eea684474e97f5e3d`）
- [Autoware Core 1.8.0](https://github.com/autowarefoundation/autoware_core/releases/tag/1.8.0)（tag `16045061a9c10da468b60e190b8fab02110fa501`）
- [Autoware Core 1.9.0](https://github.com/autowarefoundation/autoware_core/releases/tag/1.9.0)（tag `f25f83c632c1984ec276c894c41857d4abc0dad8`）
- [Autoware Launch 0.51.0](https://github.com/autowarefoundation/autoware_launch/releases/tag/0.51.0)（tag `3ba12ba6a47abca105e755e6d21cf66a0cdf743f`）
- [Autoware Launch 0.52.0](https://github.com/autowarefoundation/autoware_launch/releases/tag/0.52.0)（tag `f942598d44b5769353167c76b784323d5c14c8c7`）

## 调查方法

本地证据来自 `git status --short --branch`、`git remote -v`、`git log -1`、`repositories/autoware.repos`、目录/符号搜索；官方版本和提交来自 GitHub release、tag、PR、compare API 及 `git ls-remote`。所有官方功能与修复链接均指向 Autoware Foundation 的一手 GitHub 资料。
