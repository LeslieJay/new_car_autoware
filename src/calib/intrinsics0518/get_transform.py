import numpy as np
from scipy.spatial.transform import Rotation as R

# =====================================================================
# 相机标定参数定义
# =====================================================================

#前视相机 (FW - Front) cam0 1920x1200
# 这是通过标定获得的激光雷达到前视相机光学坐标系的变换矩阵
# 表示：点在lidar坐标系中的坐标如何变换到camera_optical_link坐标系

#前视 cam0 1920x1200
ext_lidar_cam_fw = np.array([[-0.0304711,  -0.995002, -0.0950903, -0.0628741],
							 [  -0.521051,  0.0969979,  -0.847996,  -0.226249],
							 [   0.852981,  0.0237075,  -0.521403,  -0.290832],
							 [ 0.,          0.,          0.,      1.        ]])                          


#侧视右前 cam1 1920x1200
ext_lidar_cam_rf = np.array([[ -0.980004,  -0.198389, -0.0152913,  -0.579979],
							 [ -0.0573247,   0.355091,  -0.933072, -0.0293776],
							 [   0.190541,  -0.913538,  -0.359363,  -0.456639],
							 [ 0.,          0.,          0.,          1.        ]])

#侧视左前 cam2 1920x1200
ext_lidar_cam_lf = np.array([[  0.976277,  -0.216125,  0.0131855,   0.513147],
							 [ -0.0699837,  -0.372584,  -0.925356, -0.0728838],
							 [   0.204905,   0.902481,   -0.37887,  -0.453318],
							 [ 0.,          0.,          0.,          1.        ]])
                           

# =====================================================================
# 相机内参矩阵定义 (4x4格式)
# =====================================================================
# 格式: [[fx,  0, cx, 0],
#        [ 0, fy, cy, 0],
#        [ 0,  0,  1, 0],
#        [ 0,  0,  0, 1]]
# 其中: fx, fy = 焦距 (像素单位)
#       cx, cy = 主点 (图像中心)

# 前视相机内参 (已去畸变)
intr_cam_fw = np.array([[1002.010255,    0.000000,  960.626682, 0.0000000e+00],
                        [0, 1001.998804,  603.335471, 0.0000000e+00],
                        [0.0000000e+00, 0.0000000e+00, 1.0000000e+00, 0.0000000e+00],
                        [0.0000000e+00, 0.0000000e+00, 0.0000000e+00, 1.0000000e+00]])
                       

intr_cam_lf = np.array([[1007.814212,    0.000000,  969.794584, 0.0000000e+00],
                    [0.00000000e+00, 1007.364940,  604.616891, 0.0000000e+00],
                    [0.0000000e+00, 0.0000000e+00, 1.0000000e+00, 0.0000000e+00],
                    [0.0000000e+00, 0.0000000e+00, 0.0000000e+00, 1.0000000e+00]])
                    

intr_cam_rf = np.array([[1004.016101,    0.000000,  959.065059, 0.000000],
                      [0.00000000e+00, 1003.853272,  598.919680, 0.000000],
                      [0.00000000e+00, 0.00000000e+00, 1.00000000e+00, 0.000000],
                      [0.000000,0.000000,0.000000,1.0]])
                      
                                                         
# =====================================================================
# 辅助函数：计算变换矩阵的逆
# =====================================================================
             
def inverse_transform(T):
    """
    计算给定变换矩阵的逆变换矩阵。
    
    参数:
    T (numpy.ndarray): 4x4 的齐次变换矩阵
    
    返回:
    numpy.ndarray: 变换矩阵的逆矩阵
    """
    R = T[:3, :3]  # 旋转矩阵
    t = T[:3, 3]   # 平移向量
    
    # 计算逆变换矩阵
    inv_T = np.eye(4)
    inv_T[:3, :3] = R.T  # 旋转矩阵的转置即为其逆
    inv_T[:3, 3] = -R.T @ t  # 平移部分需要反向旋转再取相反数
    
    return inv_T

# =====================================================================
# 核心函数：计算激光雷达到相机坐标系的变换（用于URDF）
# =====================================================================

def get_lidar2camera_link(ext_lidar_cam):
    """
    将标定得到的激光雷达到相机光学坐标系(camera_optical_link)的变换
    转换为激光雷达到相机机械坐标系(camera_link)的变换。
    
    背景:
    - camera_optical_link: 相机内部的光学坐标系（与OpenCV对齐）
      * X轴: 向右
      * Y轴: 向下
      * Z轴: 向前（沿视线方向）
    
    - camera_link: 相机机械坐标系（用于URDF中的TF变换定义）
      * X轴: 向前
      * Y轴: 向左
      * Z轴: 向上
    
    两个坐标系之间的固定变换:
    T(camera_link -> optical_link) = 90度旋转矩阵
    
    参数:
        ext_lidar_cam (numpy.ndarray): 标定得到的4x4变换矩阵
                                       表示 lidar -> camera_optical_link
    
    返回:
        无返回值，但打印输出相机相对于激光雷达的:
        - 平移量: x, y, z (单位: 米)
        - 欧拉角: roll, pitch, yaw (单位: 弧度)
    """
    
    # 你标定得到的外参：lidar -> camera_optical_link
    T_lidar_to_optical = inverse_transform(ext_lidar_cam)  # 4x4 矩阵

    # camera_link -> optical_link 的旋转矩阵
    R_opt = np.array([
        [ 0,  0, 1],
        [-1,  0, 0],
        [ 0, -1, 0]
    ])
    R_opt_inv = R_opt.T  # 转置即逆

    # 计算 lidar -> camera_link 的变换（用于 URDF）
    T_lidar_to_camera_link = T_lidar_to_optical.copy()
    T_lidar_to_camera_link[:3, :3] = T_lidar_to_optical[:3, :3] @ R_opt_inv

    # 提取 xyz 和 rpy
    trans = T_lidar_to_camera_link[:3, 3]
    rot = R.from_matrix(T_lidar_to_camera_link[:3, :3])
    rpy = rot.as_euler('xyz')  # 用于 URDF 的 roll, pitch, yaw（弧度）

    print(f"x: {trans[0]:.6f}")
    print(f"y: {trans[1]:.6f}")
    print(f"z: {trans[2]:.6f}")
    print(f"roll: {rpy[0]:.6f}")
    print(f"pitch: {rpy[1]:.6f}")
    print(f"yaw: {rpy[2]:.6f}")
    
print (inverse_transform(ext_lidar_cam_fw))
print (inverse_transform(ext_lidar_cam_lf))
print (inverse_transform(ext_lidar_cam_rf))
# get_lidar2camera_link(ext_lidar_cam_fw)
exit()

lidar2image_fw = intr_cam_fw @ ext_lidar_cam_fw
lidar2image_rf = intr_cam_rf @ ext_lidar_cam_rf
# lidar2image_rb = intr_cam_rb @ ext_lidar_cam_rb
lidar2image_lf = intr_cam_lf @ ext_lidar_cam_lf
# lidar2image_lb = intr_cam_lb @ ext_lidar_cam_lb
# lidar2image_back = intr_cam_back @ ext_lidar_cam_back

# print (lidar2image_back)
# exit()

# points = np.array([[11.662502, 3.187502, 1.133679],
#                    [9.187503, 6.937502, 0.974524],
#                    [4.125002, 6.450002, 0.746609],
#                    [26.700003, -1.424998, 3.672008],
#                    [7.4468894, 1.6946255, 1.09238505]])
                   
# # 添加齐次坐标，将点云转换为 (N, 4)
# homogeneous_points = np.hstack((points, np.ones((points.shape[0], 1))))

# # 应用 lidar2image 矩阵进行变换
# transformed_points = homogeneous_points @ lidar2image2.T

# # 归一化图像坐标
# u = transformed_points[:, 0] / transformed_points[:, 2]
# v = transformed_points[:, 1] / transformed_points[:, 2]
# print (u,v)


# transformed_points = np.dot(ext_lidar_cam2, homogeneous_points.T).T
# transformed_points = transformed_points[:, :3]

# # 应用内参矩阵
# projected_points = np.dot(intr_cam2[:3,:3], transformed_points.T).T

# # 归一化
# projected_points = projected_points[:, :2] / projected_points[:, 2].reshape(-1, 1)

# print (projected_points)
