# RoboSense Airy “No Leap Second” 与 linuxptp 时间尺度

> 调查日期：2026-09-18  
> 设备：左右 RoboSense Airy（`192.168.1.201`、`192.168.1.202`）  
> 场景：工控机作为 PTP Grandmaster，Airy 使用 PTP-E2E-L2

## 结论

1. **不要为适配 `No Leap Second=ON` 而修改当前 linuxptp 的时间尺度。** linuxptp 在硬件时间戳模式下要求 PHC 使用连续的 PTP/TAI 时间尺度、`CLOCK_REALTIME` 使用 UTC，两者的整数秒差由 `phc2sys` 维护。官方给出的 GM 方向示例正是 `phc2sys -c /dev/ptp0 -s CLOCK_REALTIME -w`。[linuxptp `phc2sys(8)`：TIME SCALE USAGE 与 EXAMPLES](https://github.com/richardcochran/linuxptp/blob/v3.1.1/phc2sys.8)
2. **建议三颗雷达统一使用 `No Leap Second=OFF`（出厂默认），保留现有 `phc2sys -s CLOCK_REALTIME -c eth1 -w`。** Airy 官方手册称该开关决定是否响应 Announce 报文中的“闰秒偏差设置”，并明确默认值为 OFF。[RoboSense Airy 官方手册：Appendix A.2.1](https://robosense-robotics.github.io/product-manual/en/Airy/)
3. **当前实机 A/B 测试已经确认：`No Leap Second=ON` 时，左右 Airy 原始 MSOP 包内时间均比主机 UTC 快约 36.986 秒；恢复 `OFF` 后，左右分别落后系统接收时刻约 12.8 ms 和 13.8 ms。** 当前驱动的 Airy 解码路径把该字段直接解析为 Unix 秒，因此 ON 时 ROS 时间戳也会快约 37 秒。
4. 这项现场结果确认了当前固件中 `ON=不应用 Announce 的累计闰秒偏移、输出 PTP/TAI 秒值`；但 RoboSense 公开手册没有明确写出 ON/OFF 的映射，因此仍建议向 `support@robosense.cn` 获取对应固件版本的书面定义。

## 为什么当前 GM 不应改成 UTC 时间尺度

linuxptp 官方文档明确区分两套时间：

- `CLOCK_REALTIME`：UTC，包含闰秒；
- 硬件时间戳模式下的 PHC/PTP：连续时间尺度，不插入闰秒；
- `currentUtcOffset`：两者之间的整数秒差，由 `phc2sys -w` 从 `ptp4l` 获取并维护。

linuxptp 3.1.1 源码将 `CURRENT_UTC_OFFSET` 定义为 37 秒，并注明自 2017-01-01 起生效。[linuxptp 3.1.1 `ds.h`](https://github.com/richardcochran/linuxptp/blob/v3.1.1/ds.h#L88) IERS Bulletin C 72 也确认当前 `UTC−TAI = −37 s`，且 2026 年 12 月末不引入闰秒。[IERS Bulletin C 72](https://datacenter.iers.org/data/html/bulletinc-072.html)

因此标准链路应为：

```text
CLOCK_REALTIME (UTC)
        │ phc2sys 加 currentUtcOffset=37
        ▼
eth1 PHC / PTP (TAI-like continuous timescale)
        │ Announce: ptpTimescale=1, currentUtcOffset=37
        ▼
Airy 内部时钟
        │ 应用 37 秒偏移后输出 UTC 时间戳
        ▼
ROS header.stamp ≈ CLOCK_REALTIME
```

如果把 `phc2sys` 改为 `-O 0`、让 PHC 直接保存 UTC，或令 GM 在硬件时间戳模式下谎报 UTC 时间尺度，虽可能暂时抵消 Airy 的 `ON` 行为，却违反 linuxptp 官方时间尺度模型；其他合规 PTP 从钟会产生 37 秒错误，未来累计闰秒变化也容易再次出错。[linuxptp `phc2sys(8)`](https://github.com/richardcochran/linuxptp/blob/v3.1.1/phc2sys.8)

## 当前主机配置与建议

调查时主机运行 linuxptp 3.1.1：

```text
ptp4l:    network_transport L2
          delay_mechanism E2E
          time_stamping hardware
phc2sys:  -s CLOCK_REALTIME -c eth1 -w -m
domain:   0
```

这组方向和传输设置适合当前 GM 角色，不需要因 Airy 开关改动。建议只做以下稳健性增强：

- 在 `/etc/linuxptp/ptp4l.conf` 中显式写入 `utc_offset 37`，而不是依赖 3.1.1 编译期默认值；未来 IERS 宣布新的闰秒后需要同步更新。linuxptp 默认配置也列出该项。[linuxptp `default.cfg`](https://github.com/richardcochran/linuxptp/blob/v3.1.1/configs/default.cfg)
- 用 `pmc` 检查 GM 发出的 `TIME_PROPERTIES_DATA_SET`，至少确认 `ptpTimescale=1`、`currentUtcOffset=37`。`pmc` 官方实现会显示 `currentUtcOffset`、`currentUtcOffsetValid`、`ptpTimescale` 等字段。[linuxptp 3.1.1 `pmc.c`](https://github.com/richardcochran/linuxptp/blob/v3.1.1/pmc.c)
- 若 `currentUtcOffsetValid=0`，可在 `ptp4l` 启动后通过 `SET GRANDMASTER_SETTINGS_NP` 将它设为 1；linuxptp 官方 `pmc` 源码定义了该管理消息的完整字段。[linuxptp 3.1.1 `pmc_common.c`](https://github.com/richardcochran/linuxptp/blob/v3.1.1/pmc_common.c) 不应在没有可靠证据时把 `timeTraceable`/`frequencyTraceable` 也宣告为 1。
- 三颗雷达的 PTP Domain、E2E-L2 和 No Leap Second 必须一致；Web Diagnostic 中 `Time Sync Status=Lock` 才表示同步成功。[RoboSense Airy 官方手册](https://robosense-robotics.github.io/product-manual/en/Airy/)

当前 `/run/ptp4l` 管理 socket 权限为 `root:root 0660`，普通用户不能直接执行下面的只读查询，需要使用 sudo：

```bash
sudo pmc -u -b 0 'GET TIME_PROPERTIES_DATA_SET'
sudo pmc -u -b 0 'GET GRANDMASTER_SETTINGS_NP'
```

## 当前固件的现场证据

2026-09-18 在 `No Leap Second=ON` 状态下，直接抓取并解析两颗 Airy 的原始 MSOP 数据：

| 设备 | 地址 | MSOP 时间相对主机 UTC |
|---|---|---:|
| 左 Airy | `192.168.1.201` | 约 `+36.986 s` |
| 右 Airy | `192.168.1.202` | 约 `+36.986 s` |

该偏差与当前 `TAI−UTC=37 s` 一致，排除了普通网络/ROS 发布延迟。驱动 Airy 解码使用时间戳偏移 20，并通过 `parseTimeUTCWithUs` 将包内字段直接构造成 Unix 时间，因此不会替雷达再扣除 37 秒。由此可确定当前设备/固件组合的实际行为：

```text
No Leap Second=ON → 保留 PTP/TAI 秒值 → ROS 比 UTC 快约 37 秒
No Leap Second=OFF → 应用 Announce 的累计闰秒偏移 → ROS 与 UTC 对齐
```

将左右 Airy 恢复为 `OFF` 后再次直接解析原始 MSOP，测得系统接收时刻减包内时间分别约为 `+0.0128 s` 和 `+0.0138 s`；同一时刻顶部 Helios 约为 `+0.0079 s`。三颗雷达已经处于同一 UTC 时间尺度，剩余差值是网络接收与包处理延迟。

## OFF/ON A/B 验证

一次只修改一颗雷达，保存设置并等待 Web 页面显示 `Lock`，然后重启/重新连接驱动，分别采样：

```bash
ros2 topic echo /rslidar_left/points --field header.stamp --once
date +%s.%N
```

计算：

```text
delta = LiDAR header.stamp - CLOCK_REALTIME
```

当前固件的 ON/OFF 行为均已通过原始 MSOP 实测确认：

| No Leap Second | 预期 delta | 判断 |
|---|---:|---|
| OFF | 约 `-0.05 ~ -0.3 s` | 正常的扫描/发布延迟，输出 UTC |
| ON | 约 `+36.7 ~ +37.0 s` | 未扣除累计闰秒，输出接近 PTP/TAI |

除绝对偏差外，还应连续采样并确认时间戳严格递增、相邻帧约 0.1 秒，且三路雷达彼此处于同一时间尺度。

## 官方资料没有回答的问题

- Airy 手册没有明确给出 ON/OFF 到“应用/忽略 Announce 偏移”的映射；当前映射来自现场原始包实测。
- 手册用“闰秒偏差设置”描述该行为，但没有明确指出它只指 `currentUtcOffset`，还是也包括 `leap59`/`leap61` 标志。
- 不同 Airy 固件版本是否保持完全一致的行为，公开手册没有说明。

因此，生产配置应把三台都设为 OFF，并以三台实机复测结果为验收依据；更换固件后应重新验证，必要时向 RoboSense 获取对应版本的书面定义。
