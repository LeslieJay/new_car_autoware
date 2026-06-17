import json
import numpy as np
from scipy.optimize import minimize_scalar
import matplotlib.pyplot as plt


def read_lane_points_from_json(json_file_path):
    """
    从JSON文件中读取车道线点序列
    """
    with open(json_file_path, 'r', encoding='utf-8') as f:
        data = json.load(f)

    lane_points = []
    for shape in data['shapes']:
        if shape['label'] in ['yellow-solid', 'yellow-dash']:
            # 去除重复点（如果有的话）
            points = []
            for point in shape['points']:
                point_tuple = tuple(point)
                if point_tuple not in points:
                    points.append(point_tuple)
            lane_points.append(points)

    return lane_points


def ipm_2d_to_3d_advanced(out_j, K, camera_height=1.55, pitch_angle=3):
    """
    将车道线二维点序列通过IPM转换为车辆坐标系下的三维点序列
    """
    pitch_rad = np.radians(pitch_angle)

    R_camera_to_vehicle = np.array([
        [0, 0, 1],
        [-1, 0, 0],
        [0, -1, 0]
    ])

    cos_pitch = np.cos(pitch_rad)
    sin_pitch = np.sin(pitch_rad)
    R_pitch = np.array([
        [1, 0, 0],
        [0, cos_pitch, -sin_pitch],
        [0, sin_pitch, cos_pitch]
    ])

    R_total = R_camera_to_vehicle @ R_pitch
    out_3d = []

    for lane_points in out_j:
        lane_3d = []
        points_2d = np.array(lane_points).reshape(-1, 2)

        for u, v in points_2d:
            uv1 = np.array([u, v, 1.0])
            K_inv = np.linalg.inv(K)
            point_normalized = K_inv @ uv1

            normal_camera = np.array([0, -cos_pitch, sin_pitch])
            point_on_ground_camera = np.array([0, camera_height, 0])

            denominator = np.dot(normal_camera, point_normalized)
            if abs(denominator) < 1e-6:
                continue

            t = np.dot(normal_camera, point_on_ground_camera) / denominator
            point_camera = t * point_normalized
            point_vehicle = R_total @ (point_camera - point_on_ground_camera)
            lane_3d.append(point_vehicle.tolist())

        out_3d.append(lane_3d)

    return out_3d


def fit_lane_lines_3d(lane_3d):
    """
    对三维车道线进行直线拟合
    返回每条车道线的方向向量
    """
    direction_vectors = []

    for lane in lane_3d:
        if len(lane) < 2:
            continue

        points = np.array(lane)
        # 使用主成分分析(PCA)找到主要方向
        points_centered = points - np.mean(points, axis=0)
        cov_matrix = points_centered.T @ points_centered
        eigenvalues, eigenvectors = np.linalg.eigh(cov_matrix)

        # 最大特征值对应的特征向量就是主要方向
        direction = eigenvectors[:, np.argmax(eigenvalues)]
        direction_vectors.append(direction)

    return direction_vectors


def calculate_parallel_score(direction_vectors):
    """
    计算两条车道线的平行程度
    使用方向向量之间的夹角作为评分标准
    夹角越小，平行程度越高，得分越高
    """
    if len(direction_vectors) != 2:
        return 0

    v1, v2 = direction_vectors
    # 计算两个方向向量的夹角（弧度）
    cos_angle = np.abs(np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2)))
    cos_angle = np.clip(cos_angle, -1, 1)  # 防止数值误差
    angle = np.arccos(cos_angle)

    # 将夹角转换为评分：夹角越小，评分越高
    # 使用指数衰减函数，夹角为0时得分为1，夹角增大时得分迅速下降
    score = np.exp(-10 * angle)

    return score


def evaluate_pitch_angle(pitch_angle, lane_points_2d, K, camera_height=1.55):
    """
    评估给定pitch_angle下的车道线平行程度
    """
    try:
        # 进行IPM变换
        lane_points_3d = ipm_2d_to_3d_advanced(lane_points_2d, K, camera_height, pitch_angle)

        # 拟合车道线并获取方向向量
        direction_vectors = fit_lane_lines_3d(lane_points_3d)

        if len(direction_vectors) != 2:
            return 0

        # 计算平行程度得分
        parallel_score = calculate_parallel_score(direction_vectors)

        return parallel_score

    except Exception as e:
        print(f"Error evaluating pitch_angle {pitch_angle}: {e}")
        return 0


def find_optimal_pitch_angle(json_file_path, K, camera_height=1.95, pitch_range=(-20, 20), step=0.01):
    """
    寻找最优的pitch_angle
    """
    # 读取车道线数据
    lane_points_2d = read_lane_points_from_json(json_file_path)
    print(f"找到 {len(lane_points_2d)} 条车道线")

    # 遍历pitch_angle
    pitch_angles = np.arange(pitch_range[0], pitch_range[1] + step, step)
    scores = []

    best_pitch = pitch_range[0]
    best_score = 0

    for pitch in pitch_angles:
        score = evaluate_pitch_angle(pitch, lane_points_2d, K, camera_height)
        scores.append(score)

        if score > best_score:
            best_score = score
            best_pitch = pitch

        if len(scores) % 100 == 0:
            print(f"进度: {len(scores)}/{len(pitch_angles)}, 当前最佳pitch: {best_pitch:.2f}°, 得分: {best_score:.6f}")

    # 可视化结果
    plt.figure(figsize=(12, 6))
    plt.plot(pitch_angles, scores, 'b-', linewidth=2)
    plt.axvline(x=best_pitch, color='r', linestyle='--', label=f'最佳pitch: {best_pitch:.2f}°')
    plt.xlabel('Pitch Angle (°)')
    plt.ylabel('平行程度得分')
    plt.title('Pitch Angle 优化结果')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.show()

    # 使用最佳pitch进行最终变换
    best_lane_3d = ipm_2d_to_3d_advanced(lane_points_2d, K, camera_height, best_pitch)
    best_directions = fit_lane_lines_3d(best_lane_3d)

    print(f"\n最优结果:")
    print(f"最佳pitch_angle: {best_pitch:.2f}°")
    print(f"平行程度得分: {best_score:.6f}")

    if len(best_directions) == 2:
        v1, v2 = best_directions
        angle_deg = np.degrees(np.arccos(np.abs(np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2)))))
        print(f"两条车道线夹角: {angle_deg:.2f}°")

    return best_pitch, best_lane_3d


# 使用示例
if __name__ == "__main__":
    # 假设的相机内参矩阵（需要替换为实际值）
    K = np.array([[1907.3244009283, 0.0, 1919.4205556116],
             [0.0, 1906.8286514328, 1079.8599665278],
             [0.0, 0.0, 1.0]])

    # JSON文件路径（需要替换为实际路径）
    json_file_path = "./1746689220133pingxing.json"



    # 寻找最优pitch_angle
    optimal_pitch, optimal_lane_3d = find_optimal_pitch_angle(json_file_path, K, camera_height=1.95, pitch_range=(-20, 20), step=0.01)