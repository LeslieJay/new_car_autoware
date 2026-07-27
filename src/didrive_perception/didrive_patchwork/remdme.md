## 你这份 patchworkpp.launch.py 里参数含义

### ROS 相关
- `input_pointcloud`
  - 输入点云 Topic。
- `output_ground`
  - 输出地面点云 Topic。
- `output_nonground`
  - 输出非地面点云 Topic。
- `base_frame`
  - 点云所在坐标系的参考 frame，一般是 `base_link`。
- `mask_frame`
  - 图像 mask 所在坐标系，一般是相机光学坐标系。
- `use_mask`
  - 是否启用图像 mask 对地面点进行二次过滤。

### Patchwork++ 算法参数
- `sensor_height`
  - 传感器到地面的垂直距离（LiDAR 相对地面的高度）。
  - 这个值用于低点种子选择和噪声过滤，必须与当前点云坐标系一致。
- `num_iter`
  - PCA 地面估计迭代次数。
  - 越大可能更稳，但计算开销也更大。
- `num_lpr`
  - 低点代表点（Lowest Point Representative）的数量。
  - 用于计算初始地面高度 `LPR`，点越多对远处稀疏地面更稳定。
- `num_min_pts`
  - 每个 patch 最少点数量阈值。
  - 小于这个值的 patch 直接认为非地面，不参与平面估计。
- `th_seeds`
  - 地面种子点高度阈值。
  - `z < LPR + th_seeds` 的点被选为地面初始种子，值越大地面范围越宽松。
- `th_dist`
  - 地面厚度阈值。
  - PCA 拟合后，点到估计地面距离小于该值的被认为是地面。值越大允许的地面“厚度”更大。
- `th_seeds_v`
  - 垂直结构点初始种子阈值。
  - 用于分离垂直物体，和 ground 初始化的 `th_seeds` 相似，但针对垂直结构。
- `th_dist_v`
  - 垂直结构距离阈值。
  - 用于判别垂直区域是否属于垂直结构。
- `max_range`
  - 参与地面估计的最大距离。
  - 超过该距离的点不会参与 Patchwork++ 的地面划分。
- `min_range`
  - 参与地面估计的最小距离。
  - 低于该距离的近距离点不参与分区估计。
- `uprightness_thr`
  - 斜度（法向量竖直分量）阈值。
  - `normal.z > uprightness_thr` 时认为 patch 足够“水平”，值越高对远处倾斜地面越严格。

### 其他参数
- `verbose`
  - 是否打印调试/日志信息。
- `max_time_diff`
  - 点云与图像时间戳差异最大值，超过则不匹配 mask。
- `camera_intrinsic`
  - 相机内参矩阵，用于将点云投影到图像上做 mask 过滤。

---

## 额外说明

- `sensor_height` 不是“base_link 到激光雷达”的距离，而是算法里用于地面高度判断的传感器高度。
- `use_mask=true` 时，算法会先通过 Patchwork++ 做一次分割，再用图像 mask 进一步过滤 “被标记为地面” 的点。
- 如果远处地面分割不干净，最常调的是：
  - `sensor_height`
  - `th_seeds`
  - `th_dist`
  - `uprightness_thr`
  - `num_lpr`
  - `num_min_pts`

如果你要，我也可以继续帮你逐个参数给出推荐调参范围。