# autoware_lidar_centerpoint 配置参数详解

本文档详细解析 `/home/nvidia/autoware/src/universe/autoware_universe/perception/autoware_lidar_centerpoint/config` 目录下所有配置文件的参数含义、物理意义及算法背景。

---

## 目录
- [一、配置体系概览与模型变体对比](#一配置体系概览与模型变体对比)
- [二、模型部署与后处理参数 (`centerpoint*.param.yaml`)](#二模型部署与后处理参数-centerpointparamyaml)
- [三、深度学习模型超参数 (`centerpoint*_ml_package.param.yaml`)](#三深度学习模型超参数-centerpoint_ml_packageparamyaml)
- [四、目标类别面积重映射 (`detection_class_remapper.param.yaml`)](#四目标类别面积重映射-detection_class_remapperparamyaml)
- [五、节点运行诊断与性能监控 (`centerpoint_common.param.yaml`)](#五节点运行诊断与性能监控-centerpoint_commonparamyaml)
- [六、各模型变体参数差异对比表](#六各模型变体参数差异对比表)

---

## 一、配置体系概览与模型变体对比

在节点启动时（详见 [launch/lidar_centerpoint.launch.xml](launch/lidar_centerpoint.launch.xml)），节点会组合加载四类参数文件：
1. **模型部署与后处理配置 (`model_param_path`)**：如 `centerpoint.param.yaml`。
2. **深度学习模型超参数配置 (`ml_package_param_path`)**：如 `centerpoint_ml_package.param.yaml`。
3. **目标类别重映射配置 (`class_remapper_param_path`)**：固定为 `detection_class_remapper.param.yaml`。
4. **通用诊断参数配置 (`common_param_path`)**：固定为 `centerpoint_common.param.yaml`。

### 预设模型变体概述

| 模型变体名称 (`model_name`) | 部署配置文件 | 神经网络超参数文件 | 适用场景与核心特点 |
| :--- | :--- | :--- | :--- |
| **centerpoint (标准版)** | `centerpoint.param.yaml` | `centerpoint_ml_package.param.yaml` | 在 nuScenes 与 TIER IV 数据集上训练的标准模型，感知范围 $\pm 76.8\text{ m}$，特征降采样倍数 1。 |
| **centerpoint_tiny (轻量版)** | `centerpoint_tiny.param.yaml` | `centerpoint_tiny_ml_package.param.yaml` | 特征图降采样倍数设为 2，大幅减少 Backbone 计算量与显存占用，适合算力有限的边缘芯片（如 Jetson）。 |
| **centerpoint_short_range (近距离版)** | `centerpoint_short_range.param.yaml` | `centerpoint_short_range_ml_package.param.yaml` | 范围收缩至 $\pm 51.2\text{ m}$，体素细化为 $0.16\text{ m}$，特征加入 $Z$ 偏移（10 维），专为近场与盲区高精度检测设计。 |
| **centerpoint_sigma (不确定度输出版)** | `centerpoint_sigma.param.yaml` | `centerpoint_sigma_ml_package.param.yaml` | 支持预测检测框位置、尺寸、航向及速度的高斯方差/协方差（Variance/Covariance）。 |

---

## 二、模型部署与后处理参数 (`centerpoint*.param.yaml`)

以 `centerpoint.param.yaml` 为例，主要控制 TensorRT 引擎路径、推理精度、多帧点云融合以及两级非极大值抑制（NMS）。

### 参数文件内容示例
```yaml
/**:
  ros__parameters:
    # 权重与引擎路径
    encoder_onnx_path: "$(var model_path)/pts_voxel_encoder_$(var model_name).onnx"
    encoder_engine_path: "$(var model_path)/pts_voxel_encoder_$(var model_name).engine"
    head_onnx_path: "$(var model_path)/pts_backbone_neck_head_$(var model_name).onnx"
    head_engine_path: "$(var model_path)/pts_backbone_neck_head_$(var model_name).engine"
    trt_precision: fp16
    cloud_capacity: 2000000

    post_process_params:
      circle_nms_dist_threshold: 0.5
      iou_nms_search_distance_2d: 10.0
      iou_nms_threshold: 0.1
      yaw_norm_thresholds: [0.3, 0.3, 0.3, 0.3, 0.0]

    densification_params:
      world_frame_id: map
      num_past_frames: 1
```

### 详细参数说明

| 参数字段 | 类型 | 默认值 | 单位 | 说明 |
| :--- | :--- | :--- | :--- | :--- |
| `encoder_onnx_path` | `string` | - | - | Voxel Feature Encoder (VFE) 的 ONNX 模型文件路径。 |
| `encoder_engine_path` | `string` | - | - | VFE 编译生成的 TensorRT Engine 二进制文件路径。 |
| `head_onnx_path` | `string` | - | - | Backbone-Neck-Head 网络的 ONNX 模型文件路径。 |
| `head_engine_path` | `string` | - | - | Backbone-Neck-Head 编译生成的 TensorRT Engine 文件路径。 |
| `trt_precision` | `string` | `fp16` | - | TensorRT 推理精度，可选 `fp16`（半精度推荐）或 `fp32`。 |
| `cloud_capacity` | `int` | `2000000` | 个 | GPU 端点云最大容量缓冲区大小。 |
| **`post_process_params`** | `object` | - | - | **后处理参数组** |
| `circle_nms_dist_threshold` | `float` | `0.5` | 米 (m) | **第一级 GPU Circle NMS 距离阈值**：抑制两个候选目标中心在 BEV 平面上欧氏距离小于该值的低分框。 |
| `iou_nms_search_distance_2d` | `double`| `10.0` | 米 (m) | **第二级 CPU BEV IoU NMS 搜索半径**：仅对距离小于此值的候选框计算精确的旋转多边形 IoU，降低计算复杂度。 |
| `iou_nms_threshold` | `double`| `0.1` | - | **BEV IoU 抑制阈值**：两个目标框的 2D 鸟瞰图 IoU 超过此门限时抑制置信度较低的框。 |
| `yaw_norm_thresholds` | `list` | `[0.3, 0.3, 0.3, 0.3, 0.0]` | - | 各类别航向向量模长阈值 $\sqrt{\sin^2\theta + \cos^2\theta}$。CenterPoint 预测的朝向置信度若低于此阈值，则直接将得分置零弃用。最后一个元素对应 `PEDESTRIAN`，设为 `0.0` 表示行人不按朝向置信度过滤。 |
| **`densification_params`** | `object` | - | - | **点云时间序列稠密化（融合）参数** |
| `world_frame_id` | `string` | `map` | - | 多帧点云坐标变换对齐的参考世界坐标系（通过 TF 查询位姿并补偿自车运动）。 |
| `num_past_frames` | `int` | `1` | 帧 | 累积的历史点云帧数。设为 `1` 表示累加过去 1 帧与当前帧（共 2 帧点云），相对时间差填入第 4 通道 `time_lag`。 |

---

## 三、深度学习模型超参数 (`centerpoint*_ml_package.param.yaml`)

该文件定义与模型结构直接绑定的超参数，**必须与训练导出 ONNX 时的设定保持完全一致**。

### 参数文件内容示例 (`centerpoint_ml_package.param.yaml`)
```yaml
/**:
  ros__parameters:
    model_params:
      class_names: ["CAR", "TRUCK", "BUS", "BICYCLE", "PEDESTRIAN"]
      point_feature_size: 4
      max_voxel_size: 40000
      point_cloud_range: [-76.8, -76.8, -4.0, 76.8, 76.8, 6.0]
      voxel_size: [0.32, 0.32, 10.0]
      downsample_factor: 1
      encoder_in_feature_size: 9
      has_variance: false
      has_twist: false
      detection_score_thresholds:
        distance_bin_upper_limits: [50.0, 90.0, 121.0, 200.0]
        min_confidence_scores:
          CAR: [0.35, 0.35, 0.35, 0.35]
          TRUCK: [0.35, 0.35, 0.35, 0.35]
          BUS: [0.35, 0.35, 0.35, 0.35]
          BICYCLE: [0.35, 0.35, 0.35, 0.35]
          PEDESTRIAN: [0.35, 0.35, 0.35, 0.35]
```

### 详细参数说明

| 参数字段 | 类型 | 说明与物理含义 |
| :--- | :--- | :--- |
| `class_names` | `list[string]` | 预测的目标类别名称数组，决定输出通道顺序。 |
| `point_feature_size` | `int` | 输入点云单点特征通道数。通常为 `4` ($x, y, z, \text{time\_lag}$)；若包含反射强度则为 `5` ($x, y, z, \text{intensity}, \text{time\_lag}$)。 |
| `max_voxel_size` | `int` | GPU 显存预分配的最大有效体素（Pillars）数量（默认 40,000）。 |
| `point_cloud_range` | `list[double]` | 点云检测三维截断边界 $[x_{min}, y_{min}, z_{min}, x_{max}, y_{max}, z_{max}]$（单位：米）。 |
| `voxel_size` | `list[double]` | 单个体素在 $X, Y, Z$ 轴的三维尺寸 $[v_x, v_y, v_z]$（单位：米）。$Z$ 轴通常为 8~10m，将整个垂直空间视作单一柱体 (Pillar)。 |
| `downsample_factor` | `int` | Backbone 特征提取网络的降采样步长（倍数）。标准版为 `1`，Tiny 版为 `2`，Short-range 版为 `4`。 |
| `encoder_in_feature_size` | `int` | 体素特征提取阶段输入给全连接层的特征维度：<br>• **9**：$[x, y, z, \Delta t, \bar{x}_v, \bar{y}_v, \bar{z}_v, x-x_c, y-y_c]$<br>• **10**：额外加入 $z-z_c$<br>• **11**：额外加入 $intensity$ |
| `has_variance` | `bool` | 模型是否输出高斯方差（不确定度），开启后输出目标的位置和速度协方差。 |
| `has_twist` | `bool` | 检测头是否直接预测物体运动速度线速度（Twist）。 |
| **`detection_score_thresholds`** | `object` | **分距离区间的分类置信度过滤门限** |
| `distance_bin_upper_limits` | `list[double]` | 径向距离区间上限列表（例如 `[50.0, 90.0, 121.0, 200.0]` 表示划分为 $[0, 50), [50, 90), [90, 121), [121, 200)$ 四个区间）。 |
| `min_confidence_scores` | `map` | 字典形式定义各目标类别在每个距离区间上的最低分类置信度阈值。可配置近处低门限保召回、远处高门限抑虚警。 |

---

## 四、目标类别面积重映射 (`detection_class_remapper.param.yaml`)

### 作用与背景
由于远距离或遮挡下，大货车（Truck）、大客车（Bus）和挂车（Trailer）在点云上可能只有局部反射，神经网络容易将其误分类为普通轿车（Car）。  
本节点通过后处理阶段计算预测框在鸟瞰图（BEV）上的投影面积 $\text{Area} = \text{length} \times \text{width}$，根据面积大小执行后验重映射。

### 类别索引映射
```
0: UNKNOWN,  1: CAR,      2: TRUCK,       3: BUS,
4: TRAILER,  5: MOTORBIKE, 6: BICYCLE,    7: PEDESTRIAN
```

### 参数矩阵定义
```yaml
/**:
  ros__parameters:
    # 8x8 允许重映射开关矩阵 (行: 原始类别, 列: 目标重映射类别)
    allow_remapping_by_area_matrix:
      [0, 0, 0, 0, 0, 0, 0, 0,   # UNKNOWN
       0, 0, 1, 0, 1, 0, 0, 0,   # CAR -> 允许重映射为 TRUCK(列2) 或 TRAILER(列4)
       0, 0, 0, 0, 1, 0, 0, 0,   # TRUCK -> 允许重映射为 TRAILER(列4)
       0, 0, 0, 0, 1, 0, 0, 0,   # BUS -> 允许重映射为 TRAILER(列4)
       0, 0, 0, 0, 0, 0, 0, 0,   # TRAILER
       0, 0, 0, 0, 0, 0, 0, 0,   # MOTORBIKE
       0, 0, 0, 0, 0, 0, 0, 0,   # BICYCLE
       0, 0, 0, 0, 0, 0, 0, 0]   # PEDESTRIAN

    # 8x8 对应面积下限矩阵 (单位: m^2)
    min_area_matrix:
      [ 0.0, 0.0,  0.0,  0.0,  0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 12.1,  0.0, 36.0, 0.0, 0.0, 0.0,   # CAR: >= 12.1m^2 转 TRUCK; >= 36.0m^2 转 TRAILER
        0.0, 0.0,  0.0,  0.0, 36.0, 0.0, 0.0, 0.0,   # TRUCK: >= 36.0m^2 转 TRAILER
        0.0, 0.0,  0.0,  0.0, 36.0, 0.0, 0.0, 0.0,   # BUS: >= 36.0m^2 转 TRAILER
        ...]

    # 8x8 对应面积上限矩阵 (单位: m^2)
    max_area_matrix:
      [ 0.0, 0.0,  0.0,  0.0,   0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 36.0,  0.0, 999.9, 0.0, 0.0, 0.0,   # CAR: <= 36.0m^2 转 TRUCK
        0.0, 0.0,  0.0,  0.0, 999.9, 0.0, 0.0, 0.0,   # TRUCK: <= 999.9m^2 转 TRAILER
        0.0, 0.0,  0.0,  0.0, 999.9, 0.0, 0.0, 0.0,   # BUS: <= 999.9m^2 转 TRAILER
        ...]
```

### 映射业务规则示例
1. **$12.1\text{ m}^2 \le \text{Area} \le 36.0\text{ m}^2$**（即 $2.2\text{m} \times 5.5\text{m} \sim 3.0\text{m} \times 12.0\text{m}$）：
   原先识别为 `CAR` 的物体将被修正重映射为 `TRUCK`。
2. **$\text{Area} > 36.0\text{ m}^2$**（即超过 $3.0\text{m} \times 12.0\text{m}$）：
   原先识别为 `CAR`、`TRUCK` 或 `BUS` 的超大物体将被统一修正为 `TRAILER`（拖挂车）。

---

## 五、节点运行诊断与性能监控 (`centerpoint_common.param.yaml`)

用于配置 ROS 2 Diagnostics 对该节点计算耗时的实时监控与状态告警。

```yaml
/**:
  ros__parameters:
    diagnostics:
      max_allowed_processing_time_ms: 200.0
      max_acceptable_consecutive_delay_ms: 1000.0
      validation_callback_interval_ms: 100.0
```

### 参数说明

| 参数字段 | 类型 | 默认值 | 单位 | 说明 |
| :--- | :--- | :--- | :--- | :--- |
| `max_allowed_processing_time_ms` | `double` | `200.0` | 毫秒 (ms) | **单帧最大允许处理耗时**。若单帧点云完整推理时间超过该阈值，发布 `WARN` 警告。 |
| `max_acceptable_consecutive_delay_ms` | `double` | `1000.0` | 毫秒 (ms) | **最大容忍连续超时时间**。若持续发生延迟且自上次正常处理以来的时间超过该阈值，诊断状态升级为 `ERROR` 错误。 |
| `validation_callback_interval_ms` | `double` | `100.0` | 毫秒 (ms) | 定时器调用诊断检查并发布 `processing_time_status` 的周期频率（默认 100ms / 10Hz）。 |

---

## 六、各模型变体参数差异对比表

| 参数项 | `centerpoint` (默认标准版) | `centerpoint_tiny` (轻量化版) | `centerpoint_short_range` (近距离版) | `centerpoint_sigma` (方差输出版) |
| :--- | :--- | :--- | :--- | :--- |
| **检测范围 `point_cloud_range`** | $[-76.8, -76.8, -4.0, 76.8, 76.8, 6.0]$ | $[-76.8, -76.8, -4.0, 76.8, 76.8, 6.0]$ | $[-51.2, -51.2, -3.0, 51.2, 51.2, 5.0]$ | $[-76.8, -76.8, -4.0, 76.8, 76.8, 6.0]$ |
| **体素尺寸 `voxel_size`** | $[0.32, 0.32, 10.0]$ | $[0.32, 0.32, 10.0]$ | $[0.16, 0.16, 8.0]$ | $[0.32, 0.32, 10.0]$ |
| **降采样倍数 `downsample_factor`** | **1** | **2** | **4** | **1** |
| **输入特征维数 `encoder_in_feature_size`** | **9** | **9** | **10** | **9** |
| **不确定度方差 `has_variance`** | `false` | `false` | `false` | `false` (支持切换为 `true`) |
| **部署模型名称 `model_name`** | `centerpoint` | `centerpoint_tiny` | `centerpoint_short_range` | `centerpoint_sigma` |
