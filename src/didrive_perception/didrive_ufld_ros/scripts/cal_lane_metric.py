import os
import numpy as np
import matplotlib.pyplot as plt
import cv2
def ipm_2d_to_3d_advanced(out_j, K, camera_height=1.55, pitch_angle=3):
    """
    将车道线二维点序列通过IPM转换为车辆坐标系下的三维点序列
    
    参数:
    out_j: 车道线二维点序列，格式如 [[u1,v1,u2,v2,...], ...]
    K: 相机内参矩阵
    camera_height: 相机相对于地面的高度(米)，默认1.55m
    pitch_angle: 相机俯仰角(度)，默认3度
    
    返回:
    out_3d: 车辆坐标系下的三维点序列，格式如 [[x1,y1,z1,x2,y2,z2,...], ...]
    """
    
    # 将俯仰角转换为弧度
    pitch_rad = np.radians(pitch_angle)
    
    # 相机坐标系到车辆坐标系的旋转矩阵
    # 相机坐标系: x右, y下, z前 (相对于地面)
    # 车辆坐标系: x前, y左, z上 (相对于地面)
    # 转换关系: 相机z→车辆x, 相机-x→车辆y, 相机-y→车辆z
    R_camera_to_vehicle = np.array([
        [0, 0, 1],    # 相机z → 车辆x (前)
        [-1, 0, 0],   # 相机-x → 车辆y (左)  
        [0, -1, 0]    # 相机-y → 车辆z (上)
    ])
    
    # 考虑俯仰角的旋转矩阵（绕相机x轴旋转）
    cos_pitch = np.cos(pitch_rad)
    sin_pitch = np.sin(pitch_rad)
    R_pitch = np.array([
        [1, 0, 0],
        [0, cos_pitch, -sin_pitch],
        [0, sin_pitch, cos_pitch]
    ])
    
    # 组合旋转矩阵：先进行俯仰角补偿，再进行坐标系转换
    R_total = R_camera_to_vehicle @ R_pitch
    
    out_3d = []
    
    for lane_points in out_j:
        lane_3d = []
        points_2d = np.array(lane_points).reshape(-1, 2)
        
        for u, v in points_2d:
            # 1. 将像素坐标转换到相机归一化坐标
            uv1 = np.array([u, v, 1.0])
            K_inv = np.linalg.inv(K)
            point_normalized = K_inv @ uv1
            
            # 2. 计算射线与地面的交点（考虑俯仰角）
            # 在相机坐标系中，考虑俯仰角后的地面平面方程
            # 地面法向量在相机坐标系中为: (0, -cos_pitch, sin_pitch)
            normal_camera = np.array([0, -cos_pitch, sin_pitch])
            
            # 地面上的一点（车辆坐标系原点）：在相机坐标系中的位置
            # 在无俯仰角时位置为 (0, camera_height, 0)
            # 考虑俯仰角后，需要找到相机正下方地面点在相机坐标系中的位置
            point_on_ground_camera = np.array([0, camera_height, 0])
            
            # 计算射线参数 t
            # 射线方程: P = t * point_normalized
            # 平面方程: normal_camera · (P - point_on_ground_camera) = 0
            denominator = np.dot(normal_camera, point_normalized)
            
            if abs(denominator) < 1e-6:
                # 避免除零，如果射线与地面平行，跳过该点
                continue
                
            t = np.dot(normal_camera, point_on_ground_camera) / denominator
            
            # 3. 在相机坐标系中的3D点
            point_camera = t * point_normalized
            
            # 4. 转换到车辆坐标系
            # 车辆坐标系原点在相机正下方地面处
            # 在相机坐标系中，车辆坐标系原点位置为 point_on_ground_camera
            point_vehicle = R_total @ (point_camera - point_on_ground_camera)
            
            # 5. 添加到当前车道线的3D点序列
            lane_3d.extend(point_vehicle.tolist())
        
        out_3d.append(lane_3d)
    
    return out_3d

def load_lane_data(file_path):
    """
    从txt文件加载车道线数据。
    每行代表一条车道线，由x1, y1, x2, y2...组成（像素坐标）。
    返回一个列表，每个元素是一个NumPy数组，形状为(N, 2)，代表一条车道线的所有像素点。
    """
    lanes = []
    with open(file_path, 'r') as f:
        for line in f:
            points = line.strip().split()
            if len(points) % 2 != 0 or len(points) < 4:
                print(f"Warning: Invalid data line in {file_path}: {line}")
                continue
            # 将数据转换为点对
            points = [float(p) for p in points]
            points = np.array(points).reshape(-1, 2)  # 转换为(N, 2)的数组
            lanes.append(points)
    return lanes

def apply_ipm_to_lanes(lanes_2d, K, camera_height=1.55, pitch_angle=3):
    """
    将2D车道线点集应用IPM变换到3D车辆坐标系
    """
    # 将点集转换为IPM函数需要的格式 [[u1,v1,u2,v2,...], ...]
    ipm_input = []
    for lane in lanes_2d:
        flat_points = lane.flatten().tolist()
        ipm_input.append(flat_points)
    
    # 应用IPM变换
    lanes_3d = ipm_2d_to_3d_advanced(ipm_input, K, camera_height, pitch_angle)
    
    # 将结果转换回点集格式
    lanes_3d_points = []
    for lane_3d in lanes_3d:
        points_3d = np.array(lane_3d).reshape(-1, 3)
        lanes_3d_points.append(points_3d)
    
    return lanes_3d_points

def fit_lane_curve_3d(points_3d, y_min=None, y_max=None):
    """
    用3D车道线点拟合一条在XY平面上的曲线 x = f(y)。
    由于车道线在地面上，我们忽略z坐标，只在x-y平面上拟合。
    返回一个函数 f(y)，输入y，返回预测的x。
    如果点少于2个，无法拟合，返回None。
    """
    if len(points_3d) < 2:
        return None
    
    # 提取x,y坐标（车辆坐标系：x前进，y向左）
    y_data = points_3d[:, 0]  # 车辆前进方向
    x_data = points_3d[:, 1]  # 横向位置（向左为正）
    
    # 如果指定了y范围，只拟合范围内的点
    if y_min is not None and y_max is not None:
        mask = (y_data >= y_min) & (y_data <= y_max)
        y_data = y_data[mask]
        x_data = x_data[mask]
        if len(y_data) < 2:
            return None
    
    try:
        # 使用二次多项式拟合，polyfit的参数是 (y, x)，因为我们要求 x = f(y)
        coeffs = np.polyfit(y_data, x_data, deg=2)  # coeffs = [A, B, C]
        # 创建一个函数 f(y) = A*y^2 + B*y + C
        poly_func = np.poly1d(coeffs)
        return poly_func
    except (np.RankWarning, np.linalg.LinAlgError):
        # 如果二次拟合出现问题，尝试降阶到1次线性拟合
        try:
            coeffs = np.polyfit(y_data, x_data, deg=1)
            poly_func = np.poly1d(coeffs)
            print(f"Warning: Quadratic fit failed, using linear fit instead.")
            return poly_func
        except:
            print(f"Warning: Fit failed for points with y-range: {y_data.min():.1f} to {y_data.max():.1f}")
            return None
def match_lanes_by_type(pred_lanes, gt_lanes, y_reference=20.0):
    """
    如果知道车道线类型（左、右、ego左、ego右等），按类型匹配
    """
    # 假设我们知道每条车道线的类型
    # 这里需要额外的类型信息
    pred_left = []
    pred_right = []
    gt_left = []
    gt_right = []
    
    # 根据横向位置判断类型（假设车辆在中心，左侧为负，右侧为正）
    for i, lane in enumerate(pred_lanes):
        func = fit_lane_curve_3d(lane, y_reference-5, y_reference+5)
        if func is not None:
            try:
                x_pos = func(y_reference)
                if x_pos < 0:  # 左侧车道线
                    pred_left.append((i, x_pos))
                else:  # 右侧车道线
                    pred_right.append((i, x_pos))
            except:
                pass
    
    for j, lane in enumerate(gt_lanes):
        func = fit_lane_curve_3d(lane, y_reference-5, y_reference+5)
        if func is not None:
            try:
                x_pos = func(y_reference)
                if x_pos < 0:  # 左侧车道线
                    gt_left.append((j, x_pos))
                else:  # 右侧车道线
                    gt_right.append((j, x_pos))
            except:
                pass
    
    # 分别匹配左右车道线
    matched_pairs = []
    
    # 匹配左侧车道线（按位置远近排序）
    pred_left.sort(key=lambda x: x[1])
    gt_left.sort(key=lambda x: x[1])
    for (p_idx, _), (g_idx, _) in zip(pred_left, gt_left):
        matched_pairs.append((p_idx, g_idx))
    
    # 匹配右侧车道线（按位置远近排序）
    pred_right.sort(key=lambda x: x[1])
    gt_right.sort(key=lambda x: x[1])
    for (p_idx, _), (g_idx, _) in zip(pred_right, gt_right):
        matched_pairs.append((p_idx, g_idx))
    
    return matched_pairs
def match_lanes_by_distance(pred_lanes, gt_lanes, y_samples=None):
    """
    基于点集之间的平均距离进行车道线匹配
    """
    if y_samples is None:
        y_samples = np.linspace(5, 40, 8)
    
    matched_pairs = []
    used_gt_indices = set()
    
    for pred_idx, pred_lane in enumerate(pred_lanes):
        best_gt_idx = -1
        min_distance = float('inf')
        
        pred_func = fit_lane_curve_3d(pred_lane, min(y_samples), max(y_samples))
        if pred_func is None:
            continue
            
        for gt_idx, gt_lane in enumerate(gt_lanes):
            if gt_idx in used_gt_indices:
                continue
                
            gt_func = fit_lane_curve_3d(gt_lane, min(y_samples), max(y_samples))
            if gt_func is None:
                continue
            
            # 计算在采样点上的平均距离
            distances = []
            for y in y_samples:
                try:
                    x_pred = pred_func(y)
                    x_gt = gt_func(y)
                    distance = abs(x_pred - x_gt)
                    distances.append(distance)
                except:
                    distances.append(float('inf'))
            
            avg_distance = np.mean(distances)
            if avg_distance < min_distance:
                min_distance = avg_distance
                best_gt_idx = gt_idx
        
        if best_gt_idx != -1 and min_distance < 3.0:  # 3米阈值
            matched_pairs.append((pred_idx, best_gt_idx))
            used_gt_indices.add(best_gt_idx)
    
    return matched_pairs
def match_lanes_by_position(pred_lanes, gt_lanes, y_reference=20.0):
    """
    基于在参考纵向距离处的横向位置进行车道线匹配
    """
    matched_pairs = []
    
    # 为每条预测和真实车道线计算在参考点的横向位置
    pred_positions = []
    for i, lane in enumerate(pred_lanes):
        func = fit_lane_curve_3d(lane, y_reference-5, y_reference+5)
        if func is not None:
            try:
                x_pos = func(y_reference)
                pred_positions.append((i, x_pos))
            except:
                pred_positions.append((i, float('inf')))
    
    gt_positions = []
    for j, lane in enumerate(gt_lanes):
        func = fit_lane_curve_3d(lane, y_reference-5, y_reference+5)
        if func is not None:
            try:
                x_pos = func(y_reference)
                gt_positions.append((j, x_pos))
            except:
                gt_positions.append((j, float('inf')))
    
    # 按横向位置排序
    pred_positions.sort(key=lambda x: x[1])
    gt_positions.sort(key=lambda x: x[1])
    
    # 一对一匹配
    for (pred_idx, pred_x), (gt_idx, gt_x) in zip(pred_positions, gt_positions):
        if pred_x != float('inf') and gt_x != float('inf'):
            matched_pairs.append((pred_idx, gt_idx))
    
    return matched_pairs
def calculate_lane_error(pred_func, gt_func, y_values):
    """
    在给定的y_values上计算预测函数和真实函数之间的横向误差。
    返回每个y点的误差列表。
    """
    errors = []
    for y in y_values:
        try:
            x_pred = pred_func(y)
            x_gt = gt_func(y)
            error = abs(x_pred - x_gt)
            errors.append(error)
        except:
            # 如果某个y值超出拟合范围或函数计算失败，记录为NaN
            errors.append(np.nan)
    return errors
def save_3d_lanes_plot(image_path,lanes_pixel,out_3d, save_path, title="车道线3D可视化（车辆坐标系）"):
    """
    保存3D车道线可视化图片（车辆坐标系）
    """
    # 定义不同颜色
    colors = ['red', 'blue', 'green', 'orange', 'purple', 'brown', 'pink', 'gray']
    # 创建包含4个子图的画布：3D图 + 三个投影图
    fig = plt.figure(figsize=(40, 30))
    

    #原图
    img = plt.imread(image_path)
    print("数值范围: min={}, max={}".format(img.min(), img.max()))
    print("图片形状:", img.shape)
    ax0 = fig.add_subplot(121)
    plt.imshow(img)
    for i, lane_points in enumerate(lanes_pixel):
        points = np.array(lane_points).reshape(-1, 2)
        color = colors[i % len(colors)]
        ax0.scatter(points[:, 0], points[:, 1], 
                   color=color, s=30, alpha=0.8)

    
    # 2. 三个投影图
    ax2 = fig.add_subplot(122)  # X-Y平面投影（俯视图）

    


    for i, lane_points in enumerate(out_3d):
        # 将一维数组转换为三维点
        points = np.array(lane_points).reshape(-1, 3)
        
        # 选择颜色
        color = colors[i % len(colors)]
        label = f'lane {i+1}'

        
        # 2. X-Y平面投影（俯视图）
        ax2.plot(points[:, 0], points[:, 1], 
                marker='o', linestyle='-', linewidth=2, markersize=4,
                color=color, label=label)
        ax2.scatter(points[:, 0], points[:, 1], 
                   color=color, s=30, alpha=0.8)
        

    

    
    # 设置X-Y投影图（俯视图）
    ax2.set_xlabel('X')
    ax2.set_ylabel('Y')
    ax2.set_title('X-Y')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    ax2.set_aspect('equal', adjustable='box')
    


    
    # 设置所有子图的坐标轴范围一致
    if len(out_3d) > 0:
        all_points = np.vstack([np.array(lane).reshape(-1, 3) for lane in out_3d])
        
        # 3D图范围设置
        max_range = np.max(np.ptp(all_points, axis=0))
        mid_x = np.mean(all_points[:, 0])
        mid_y = np.mean(all_points[:, 1])
        mid_z = np.mean(all_points[:, 2])
        
        
        # 投影图范围设置
        margin = max_range * 0.1  # 10%的边距
        ax2.set_xlim(mid_x - max_range/2 - margin, mid_x + max_range/2 + margin)
        ax2.set_ylim(mid_y - max_range/2 - margin, mid_y + max_range/2 + margin)
        

    
    plt.tight_layout()
    
    # 保存图片
    plt.savefig(save_path, dpi=300, bbox_inches='tight', facecolor='white')
    plt.close()  # 关闭图形，释放内存
    
    print(f"3D可视化图片（含投影）已保存到: {save_path}")
def process_folder_with_ipm(img_folder,pred_folder, gt_folder, K, y_values, 
                               matching_strategy='position',  # 'position', 'distance', 'type'
                               camera_height=1.55, pitch_angle=3,output_file="lane_errors.csv"):
    """
    使用改进的匹配策略处理整个文件夹
    """
    all_errors = []
    img_files = sorted([f for f in os.listdir(img_folder) if f.endswith('.jpg')])
    pred_files = sorted([f for f in os.listdir(pred_folder) if f.endswith('.txt')])
    gt_files = sorted([f for f in os.listdir(gt_folder) if f.endswith('.txt')])
    
    print(f"Found {len(pred_files)} prediction files and {len(gt_files)} ground truth files.")
    print(f"Using matching strategy: {matching_strategy}")
    
    y_min, y_max = min(y_values), max(y_values)
    
    for idx, (img_file,pred_file, gt_file) in enumerate(zip(img_files,pred_files, gt_files)):
        image_path=os.path.join(img_folder, img_file)
        pred_path = os.path.join(pred_folder, pred_file)
        gt_path = os.path.join(gt_folder, gt_file)

        # 加载数据并应用IPM
        pred_lanes_pixel = load_lane_data(pred_path)
        gt_lanes_pixel = load_lane_data(gt_path)
        
        pred_lanes_3d = apply_ipm_to_lanes(pred_lanes_pixel, K, camera_height, pitch_angle)
        gt_lanes_3d = apply_ipm_to_lanes(gt_lanes_pixel, K, camera_height, pitch_angle)
        save_path_gt="/mnt/qzz/Ultra-Fast-Lane-Detection-master/predlane/gt_"+pred_path.split('/')[-1].split('.')[0]+".png"
        save_path_pred="/mnt/qzz/Ultra-Fast-Lane-Detection-master/predlane/predlane_"+pred_path.split('/')[-1].split('.')[0]+".png"
    
        save_3d_lanes_plot(image_path,gt_lanes_pixel,gt_lanes_3d,save_path_gt)
        # print(save_path)
        save_3d_lanes_plot(image_path,pred_lanes_pixel,pred_lanes_3d,save_path_pred)

        # 根据选择的策略进行车道线匹配
        if matching_strategy == 'position':
            matched_pairs = match_lanes_by_position(pred_lanes_3d, gt_lanes_3d, y_reference=20.0)
        elif matching_strategy == 'distance':
            matched_pairs = match_lanes_by_distance(pred_lanes_3d, gt_lanes_3d, y_samples=y_values)
        elif matching_strategy == 'type':
            matched_pairs = match_lanes_by_type(pred_lanes_3d, gt_lanes_3d, y_reference=20.0)
        else:
            # 默认使用顺序匹配
            min_lanes = min(len(pred_lanes_3d), len(gt_lanes_3d))
            matched_pairs = [(i, i) for i in range(min_lanes)]
        
        frame_errors = []
        for pred_idx, gt_idx in matched_pairs:
            pred_func = fit_lane_curve_3d(pred_lanes_3d[pred_idx], y_min, y_max)
            gt_func = fit_lane_curve_3d(gt_lanes_3d[gt_idx], y_min, y_max)
            
            if pred_func is None or gt_func is None:
                continue
            
            lane_errors = calculate_lane_error(pred_func, gt_func, y_values)
            frame_errors.extend(lane_errors)
        
        all_errors.extend(frame_errors)
        
        if (idx + 1) % 100 == 0:
            print(f"Processed {idx + 1}/{len(pred_files)} files, "
                  f"current frame matched {len(matched_pairs)} lane pairs")
    
    # 转换为numpy数组并移除NaN值
    all_errors = np.array(all_errors)

    valid_errors = all_errors[~np.isnan(all_errors)]
    
    if len(valid_errors) == 0:
        print("Error: No valid errors calculated! Please check your data and parameters.")
        return
    
    # 输出统计结果
    print("\n=== 车道线检测误差统计 (IPM变换后) ===")
    print(f"总有效测量点数: {len(valid_errors)}")
    print(f"平均误差: {np.mean(valid_errors):.4f} 米")
    print(f"误差标准差: {np.std(valid_errors):.4f} 米")
    print(f"最大误差: {np.max(valid_errors):.4f} 米")
    print(f"最小误差: {np.min(valid_errors):.4f} 米")
    print(f"中位数误差: {np.median(valid_errors):.4f} 米")
    
    # 计算百分位数
    print("\n--- 误差分位数统计 ---")
    for percentile in [50, 75, 90, 95, 99]:
        p_value = np.percentile(valid_errors, percentile)
        print(f"{percentile}% 分位数误差: {p_value:.4f} 米")
    
    # 保存详细误差结果
    np.savetxt(output_file, valid_errors, delimiter=',', header='Lane_Detection_Errors(m)')
    print(f"\n详细误差数据已保存到: {output_file}")
    
    # 绘制误差分布直方图
    plt.figure(figsize=(12, 8))
    
    plt.subplot(2, 1, 1)
    plt.hist(valid_errors, bins=50, alpha=0.7, edgecolor='black', density=True)
    plt.xlabel('车道线检测误差 (米)')
    plt.ylabel('概率密度')
    plt.title('车道线检测误差分布 (IPM变换后)')
    plt.grid(True, alpha=0.3)
    
    # 标记关键统计量
    mean_error = np.mean(valid_errors)
    plt.axvline(mean_error, color='red', linestyle='--', label=f'平均误差: {mean_error:.3f}m')
    plt.axvline(0.2, color='orange', linestyle='-', label='需求阈值: 0.2m', linewidth=2)
    plt.legend()
    
    plt.subplot(2, 1, 2)
    # 绘制累积分布函数
    sorted_errors = np.sort(valid_errors)
    cdf = np.arange(1, len(sorted_errors) + 1) / len(sorted_errors)
    plt.plot(sorted_errors, cdf, linewidth=2)
    plt.xlabel('车道线检测误差 (米)')
    plt.ylabel('累积概率')
    plt.title('误差累积分布函数 (CDF)')
    plt.grid(True, alpha=0.3)
    plt.axvline(0.2, color='orange', linestyle='-', label='0.2m阈值', linewidth=2)
    plt.axhline(0.95, color='green', linestyle='--', label='95%概率', linewidth=1)
    plt.legend()
    
    plt.tight_layout()
    plt.savefig('lane_error_distribution_ipm.png', dpi=300)
    plt.show()
    
    # 检查是否满足需求
    mean_ok = np.mean(valid_errors) <= 0.3
    p95_ok = np.percentile(valid_errors, 95) <= 0.3
    
    print("\n=== 需求验证结果 ===")
    
    if mean_ok:
        print(f"{min(y_values)}m--{max(y_values)}m范围内平均误差为 {np.mean(valid_errors):.4f}m"+"✅ 满足需求：平均误差不大于0.3米")
    else:
        print("❌ 不满足需求：")
        if not mean_ok:
            print(f" {min(y_values)}m--{max(y_values)}m范围内平均误差 {np.mean(valid_errors):.4f}m > 0.3m")
        # if not p95_ok:
        #     print(f"   - 95%分位数误差 {np.percentile(valid_errors, 95):.4f}m > 0.2m")

# ====== 在这里设置您的参数 ======
if __name__ == "__main__":
    # 文件夹路径
    img_folder='/mnt/qzz/Ultra-Fast-Lane-Detection-master/tmp/culane_eval_tmp/img'
    prediction_folder = "/mnt/qzz/Ultra-Fast-Lane-Detection-master/tmp/culane_eval_tmp/select_to_calculate"  # 请替换为您的预测结果文件夹路径
    ground_truth_folder = "/mnt/qzz/Ultra-Fast-Lane-Detection-master/tmp/culane_eval_tmp/select_to_calculate_gt"  # 请替换为您的标签文件夹路径
    

    K = np.array([[1907.3244009283,0.0,1919.4205556116],[0.0,1906.8286514328,1079.8599665278],[0.0,0.0,1.0]])
    
    # 相机参数
    camera_height = 1.95  # 相机高度(米)
    pitch_angle = 5.96     # 相机俯仰角(度)
    
    # 定义测量点（车辆前方的纵向距离，单位：米）
    # 注意：在车辆坐标系中，x方向是前进方向
    measurement_points = [5, 10, 15, 20, 25, 30, 35, 40]
    for dis in [26,31,36,41,46]:
        measurement_points = [i for i in range(5,dis)]
        # 执行处理
        process_folder_with_ipm(
            img_folder,
            prediction_folder, 
            ground_truth_folder, 
            K, 
            measurement_points,
            matching_strategy='distance',
            camera_height=camera_height,
            pitch_angle=pitch_angle
        )