#ifndef PATCHWORKPP_H
#define PATCHWORKPP_H

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <time.h>
#include <chrono>
#include <iostream>
#include <numeric>
#include <vector>

#include <Eigen/Dense>

using namespace std;

#define MAX_POINTS 5000  // 单个补丁中的最大点数

namespace patchwork {

/**
 * @struct PointXYZ
 * @brief 三维点云中的单个点，包含空间坐标和索引信息
 */
struct PointXYZ {
  float x;      // 点的X坐标
  float y;      // 点的Y坐标
  float z;      // 点的Z坐标
  int idx;      // 点在原始点云中的索引，-1表示无效

  /**
   * @brief 构造函数
   * @param _x X坐标
   * @param _y Y坐标
   * @param _z Z坐标
   * @param _idx 点在原始点云中的索引，默认为-1
   */
  PointXYZ(float _x, float _y, float _z, int _idx = -1) : x(_x), y(_y), z(_z), idx(_idx) {}
};

/**
 * @struct RevertCandidate
 * @brief 用于时间域地面回退的候选点信息，存储被判定为非地面但实际可能是地面的点集信息
 */
struct RevertCandidate {
  int concentric_idx;              // 同心圆环模型中的区域索引
  int sector_idx;                  // 扇形分割中的扇形索引
  double ground_flatness;          // 该区域地面平整度（平面度），用于判断是否为平面
  double line_variable;            // 线性变量，用于平面拟合评估
  Eigen::VectorXf pc_mean;         // 该区域点云的平均值中心
  vector<PointXYZ> regionwise_ground;  // 该区域内的地面点集

  /**
   * @brief 构造函数
   * @param _c_idx 同心圆环区域索引
   * @param _s_idx 扇形区域索引
   * @param _flatness 地面平整度值
   * @param _line_var 线性变量值
   * @param _pc_mean 点云平均值
   * @param _ground 地面点集
   */
  RevertCandidate(int _c_idx,
                  int _s_idx,
                  double _flatness,
                  double _line_var,
                  Eigen::VectorXf _pc_mean,
                  vector<PointXYZ> _ground)
      : concentric_idx(_c_idx),
        sector_idx(_s_idx),
        ground_flatness(_flatness),
        line_variable(_line_var),
        pc_mean(_pc_mean),
        regionwise_ground(_ground) {}
};

/**
 * @struct Params
 * @brief PatchWork++算法的所有配置参数
 * 
 * 包括算法开关、迭代参数、距离阈值、区域划分参数等
 */
struct Params {
  bool verbose;      // 是否输出详细的调试信息
  bool enable_RNR;   // 启用反射噪声移除 (Reflected Noise Removal)
  bool enable_RVPF;  // 启用路面垂直平面拟合 (Road Vertical Plane Fitting)
  bool enable_TGR;   // 启用时间域地面回退 (Temporal Ground Revert)

  // ===== 算法迭代和采样参数 =====
  int num_iter;              // PCA地面平面估计的迭代次数
  int num_lpr;               // 作为最低点代表的最大选择点数
  int num_min_pts;           // 每个补丁估计地面平面所需的最小点数
  int num_zones;             // 同心圆环模型的区域数量
  int num_rings_of_interest; // 需要检查高度值和平整度值的环数

  // ===== 反射噪声移除参数 =====
  double RNR_ver_angle_thr;  // 噪声点垂直角度阈值。雷达向下的射线更可能产生严重噪声
  double RNR_intensity_thr;   // 噪声点反射强度阈值。反射点的强度通常较小

  // ===== 传感器和距离阈值参数 =====
  double sensor_height;  // 传感器（雷达）相对地面的安装高度
  double th_seeds;       // 用于初始地面点代表选择的最低点代表阈值
  double th_dist;        // 地面厚度阈值，用于判断点是否属于地面
  double th_seeds_v;     // 用于初始竖直结构点选择的最低点代表阈值
  double th_dist_v;      // 竖直结构厚度阈值
  double max_range;      // 地面估计区域的最大范围（米）
  double min_range;      // 地面估计区域的最小范围（米）
  double uprightness_thr;// 竖直性阈值，用于地面似然估计 (GLE)，详见论文
  double adaptive_seed_selection_margin;  // 初始种子选择中使用的参数
  double intensity_thr;  // 反射强度阈值

  // ===== 同心圆环模型配置 =====
  vector<int> num_sectors_each_zone;  // 每个区域的扇形数量 {16, 32, 54, 32}
  vector<int> num_rings_each_zone;    // 每个区域的环数 {2, 4, 4, 4}

  // ===== 自适应阈值存储参数 =====
  int max_flatness_storage;   // 平整度值的最大存储数量
  int max_elevation_storage;  // 高度值的最大存储数量

  // ===== 自适应阈值（动态更新） =====
  vector<double> elevation_thr;  // 每个环的高度阈值（在地面似然估计中使用），动态更新
  vector<double> flatness_thr;   // 每个环的平整度阈值（在地面似然估计中使用），动态更新

  /**
   * @brief 构造函数 - 初始化所有参数为默认值
   * 
   * 默认参数基于Hyun-Jae Lin等人的论文调优
   * 参考: "Patchwork++: Efficient Ground Segmentation of Large-scale Point Clouds"
   */
  Params() {
    verbose     = false;  // 默认关闭详细输出
    enable_RNR  = true;   // 启用噪声移除
    enable_RVPF = true;   // 启用路面拟合
    enable_TGR  = true;   // 启用时间域地面回退

    // PCA迭代参数
    num_iter    = 3;      // 3次迭代通常能获得稳定的平面估计
    num_lpr     = 20;     // 选择每个环中最低的20个点作为代表
    num_min_pts = 10;     // 少于10个点的补丁不可靠，不参与地面估计
    num_zones   = 4;      // 将点云分为4个同心圆环区域
    num_rings_of_interest = 4;  // 检查每个环的4条环线

    // 反射噪声移除参数
    RNR_ver_angle_thr = -25.0;  // 垂直角度小于-15°的射线产生的点可能是噪声
    RNR_intensity_thr = 0.05;    // 反射强度小于0.2的点被认为是噪声

    // 传感器和距离参数
    sensor_height = 1.723;      // 典型的车载LiDAR安装高度（单位：米）
    th_seeds      = 0.125;      // 种子选择的降低距离阈值（单位：米）
    th_dist       = 0.125;      // 地面厚度阈值（单位：米）
    th_seeds_v    = 0.25;       // 竖直结构的种子选择阈值
    th_dist_v     = 0.1;        // 竖直结构的厚度阈值
    max_range     = 80.0;       // 处理范围内最大距离160米内的点
    min_range     = 2.7;        // 距离传感器太近（<2.7m）的点通常为噪声
    uprightness_thr = 0.707;    // cos(45°) ≈ 0.707，用于地面似然估计
    adaptive_seed_selection_margin = -1.2;  // 自适应种子选择的偏移参数

    // 同心圆环模型配置：4个区域，逐渐增加分辨率
    num_sectors_each_zone = {16, 24, 24, 32};  // 第1区: 16个扇形, 第2区: 32个, 第3区: 54个, 第4区: 32个
    num_rings_each_zone   = {2, 4, 4, 4};      // 第1区: 2个环, 第2-4区: 各4个环

    // 自适应阈值存储
    max_flatness_storage  = 1000;  // 保存最近1000个平整度值用于自适应更新
    max_elevation_storage = 1000;  // 保存最近1000个高度值用于自适应更新
    
    // 初始阈值为0，运行时根据点云数据自动调整
    elevation_thr = {0, 0, 0, 0};  // 4个环的高度阈值，会动态更新
    flatness_thr = {0, 0, 0, 0};   // 4个环的平整度阈值，会动态更新
  }
};

/**
 * @class PatchWorkpp
 * @brief PatchWork++地面分割算法的核心实现类
 * 
 * 基于同心圆环模型(CZM)的快速鲁棒地面分割算法。
 * 将点云分批处理，使用局部平面特征进行实时地面检测。
 * 
 * 算法流程:
 * 1. 将点云按照极坐标划分为4个同心圆环区域
 * 2. 每个区域进一步分割为多个扇形补丁
 * 3. 对每个补丁进行PCA分析，估计局部地面平面
 * 4. 利用自适应阈值判断点是否属于地面
 * 5. 使用时间域信息进行地面回退，提高时序稳定性
 */
class PatchWorkpp {
 public:
  typedef std::vector<vector<PointXYZ>> Ring;  // 一个环 = 多个扇形
  typedef std::vector<Ring> Zone;              // 一个区域 = 多个环

  /**
   * @brief 构造函数 - 初始化同心圆环模型的结构
   * @param _params PatchWork++算法的参数配置
   * 
   * 初始化过程:
   * 1. 计算4个区域的范围边界
   * 2. 计算每个环和扇形的大小
   * 3. 构建嵌套的同心圆环模型数据结构
   */
  PatchWorkpp(patchwork::Params _params) : params_(_params) {
    // 计算4个同心圆环区域的最小范围边界
    // 区域之间以不同的间隔划分，使得较近的区域分辨率更高
    double min_range_z2_ = (7 * params_.min_range + params_.max_range) / 8.0;  // 区域2的最小范围
    double min_range_z3_ = (3 * params_.min_range + params_.max_range) / 4.0;  // 区域3的最小范围
    double min_range_z4_ = (params_.min_range + params_.max_range) / 2.0;      // 区域4的最小范围
    min_ranges_          = {params_.min_range, min_range_z2_, min_range_z3_, min_range_z4_};

    // 计算每个区域内每条环线的径向宽度（米）
    ring_sizes_   = {(min_range_z2_ - params_.min_range) / params_.num_rings_each_zone.at(0),
                     (min_range_z3_ - min_range_z2_) / params_.num_rings_each_zone.at(1),
                     (min_range_z4_ - min_range_z3_) / params_.num_rings_each_zone.at(2),
                     (params_.max_range - min_range_z4_) / params_.num_rings_each_zone.at(3)};
    
    // 计算每个区域内每个扇形的角度宽度（弧度）
    sector_sizes_ = {2 * M_PI / params_.num_sectors_each_zone.at(0),  // 区域1的扇形宽度
                     2 * M_PI / params_.num_sectors_each_zone.at(1),  // 区域2的扇形宽度
                     2 * M_PI / params_.num_sectors_each_zone.at(2),  // 区域3的扇形宽度
                     2 * M_PI / params_.num_sectors_each_zone.at(3)}; // 区域4的扇形宽度

    // 构建同心圆环模型: 4个区域，每个区域由多个环组成，每个环包含多个扇形
    for (int k = 0; k < params_.num_zones; k++) {
      Ring empty_ring;  // 创建一个空的环
      empty_ring.resize(params_.num_sectors_each_zone[k]);  // 该环包含的扇形数

      Zone z;  // 创建一个新的区域
      for (int i = 0; i < params_.num_rings_each_zone[k]; i++) {
        z.push_back(empty_ring);  // 向区域中添加环
      }

      ConcentricZoneModel_.push_back(z);  // 将区域加入模型
    }

    std::cout << "PatchWorkpp::PatchWorkpp() - INITIALIZATION COMPLETE" << std::endl;
  }

  /**
   * @brief 主算法接口 - 对输入点云进行地面分割
   * @param cloud_in 输入点云，大小为 (N, 3) 的矩阵，其中N为点数
   * 
   * 该函数执行完整的地面分割流程，结果存储在内部成员变量中
   */
  void estimateGround(Eigen::MatrixXf cloud_in);

  /**
   * @brief 获取传感器高度
   * @return 传感器相对地面的安装高度（米）
   */
  double getHeight() { return params_.sensor_height; }
  
  /**
   * @brief 获取地面分割耗时
   * @return 执行地面分割所用的时间（秒）
   */
  double getTimeTaken() { return time_taken_; }

  /**
   * @brief 获取分割出的地面点
   * @return 地面点集，大小为 (M, 3) 的Eigen矩阵，M为地面点数
   */
  Eigen::MatrixX3f getGround() { return toEigenCloud(cloud_ground_); }
  
  /**
   * @brief 获取分割出的非地面点
   * @return 非地面点集，大小为 (K, 3) 的Eigen矩阵，K为非地面点数
   */
  Eigen::MatrixX3f getNonground() { return toEigenCloud(cloud_nonground_); }
  
  /**
   * @brief 获取地面点在原始点云中的索引
   * @return 地面点的原始索引向量
   */
  Eigen::VectorXi getGroundIndices() { return toIndices(cloud_ground_); }
  
  /**
   * @brief 获取非地面点在原始点云中的索引
   * @return 非地面点的原始索引向量
   */
  Eigen::VectorXi getNongroundIndices() { return toIndices(cloud_nonground_); }

  /**
   * @brief 获取每个补丁的中心点
   * @return 补丁中心点的坐标矩阵
   */
  Eigen::MatrixX3f getCenters() { return toEigenCloud(centers_); }
  
  /**
   * @brief 获取每个补丁的法向量
   * @return 补丁法向量的坐标矩阵
   */
  Eigen::MatrixX3f getNormals() { return toEigenCloud(normals_); }

 private:
  // ===== 命名规范：所有私有成员变量均以下划线"_"结尾 =====

  /**
   * @brief 算法参数配置
   */
  patchwork::Params params_;

  // ===== 时间统计 =====
  time_t timer_;       // 计时器
  double time_taken_;  // 最后一次分割耗时（秒）

  // ===== 自适应阈值更新数据 =====
  std::vector<double> update_flatness_[4];   // 4个区域的平整度历史记录
  std::vector<double> update_elevation_[4];  // 4个区域的高度值历史记录

  // ===== PCA计算的中间结果 =====
  double d_;  // 平面到原点的有向距离

  Eigen::VectorXf normal_;          // 估计的地面平面法向量 (3x1)
  Eigen::VectorXf singular_values_; // PCA的奇异值 (3x1)，用于判断平面质量
  Eigen::Matrix3f cov_;             // 点云的协方差矩阵 (3x3)
  Eigen::VectorXf pc_mean_;         // 点云的平均值中心

  // ===== 同心圆环模型配置 =====
  vector<double> min_ranges_;   // 4个区域的最小范围
  vector<double> sector_sizes_; // 4个区域的扇形角度宽度
  vector<double> ring_sizes_;   // 4个区域的环径向宽度

  // ===== 同心圆环模型数据结构 =====
  vector<Zone> ConcentricZoneModel_;  // 完整的同心圆环模型: Zone -> Ring -> 扇形中的点

  // ===== 中间处理结果（区域级别） =====
  vector<PointXYZ> ground_pc_, non_ground_pc_;              // 从当前区域分割出的地面和非地面点
  vector<PointXYZ> regionwise_ground_, regionwise_nonground_;  // 按区域逐个处理的中间结果

  // ===== 最终分割结果 =====
  vector<PointXYZ> cloud_ground_, cloud_nonground_;  // 最终的地面点和非地面点集

  // ===== 补丁特征 =====
  vector<PointXYZ> centers_, normals_;  // 每个补丁的中心和法向量

  // ===== 数据格式转换 =====
  /**
   * @brief 将内部PointXYZ向量转换为Eigen矩阵格式
   * @param cloud 输入的PointXYZ向量
   * @return (N, 3)的Eigen矩阵
   */
  Eigen::MatrixX3f toEigenCloud(const vector<PointXYZ> &cloud);
  
  /**
   * @brief 从PointXYZ向量提取原始点云中的索引
   * @param cloud 输入的PointXYZ向量
   * @return 索引向量
   */
  Eigen::VectorXi toIndices(const vector<PointXYZ> &cloud);

  /**
   * @brief 将点加入到点集中
   * @param cloud 目标点集
   * @param add 要添加的点集
   */
  void addCloud(vector<PointXYZ> &cloud, vector<PointXYZ> &add);

  /**
   * @brief 清空同心圆环模型中的所有点数据，重置为初始状态
   * @param czm 同心圆环模型
   */
  void flush_patches(std::vector<Zone> &czm);

  /**
   * @brief 将点云点转换为同心圆环模型中的对应位置
   * 根据每个点的极坐标(角度、距离)，将其分配到合适的补丁中
   * @param src 输入点云矩阵 (N, 3)
   * @param czm 同心圆环模型的引用
   */
  void pc2czm(const Eigen::MatrixXf &src, std::vector<Zone> &czm);

  /**
   * @brief 反射噪声移除 - 去除LiDAR反射产生的噪声点
   * 根据垂直射线角度和反射强度判断。射线角度太陡且反射强度弱的点被认为是噪声
   * @param cloud_in 输入点云，会被修改
   */
  void reflected_noise_removal(Eigen::MatrixXf &cloud_in);

  /**
   * @brief 时间域地面回退 - 利用前一帧的地面信息纠正当前帧
   * 如果当前环被判定为非地面，但前一帧该环是地面，在一定条件下可以将其回退为地面
   * @param ring_flatness 当前环的平整度值列表
   * @param candidates 候选的回退点集
   * @param concentric_idx 同心圆环区域索引
   */
  void temporal_ground_revert(std::vector<double> ring_flatness,
                              std::vector<patchwork::RevertCandidate> candidates,
                              int concentric_idx);

  /**
   * @brief 计算点到平面的有向距离
   * 距离 = |ax + by + cz + d| / sqrt(a^2 + b^2 + c^2)
   * @param p 点的坐标
   * @param normal 平面法向量 (a, b, c)
   * @param d 平面参数
   * @return 有向距离值
   */
  double calc_point_to_plane_d(PointXYZ p, Eigen::VectorXf normal, double d);
  
  /**
   * @brief 计算向量的平均值和标准差
   * @param vec 输入向量
   * @param mean 输出的平均值
   * @param stdev 输出的标准差
   */
  void calc_mean_stdev(std::vector<double> vec, double &mean, double &stdev);

  /**
   * @brief 更新高度阈值 - 根据历史数据自适应调整
   * elevation_thr会根据过去分割结果的统计信息更新
   */
  void update_elevation_thr();
  
  /**
   * @brief 更新平整度阈值 - 根据历史数据自适应调整
   * flatness_thr会根据过去分割结果的统计信息更新
   */
  void update_flatness_thr();

  /**
   * @brief 将二维笛卡尔坐标(x, y)转换为极坐标角度θ
   * @param x X坐标
   * @param y Y坐标
   * @return 角度值(弧度)，范围 [-π, π]
   */
  double xy2theta(const double &x, const double &y);

  /**
   * @brief 将二维笛卡尔坐标(x, y)转换为极坐标距离r
   * @param x X坐标
   * @param y Y坐标
   * @return 距离值(米)
   */
  double xy2radius(const double &x, const double &y);

  /**
   * @brief 使用PCA方法估计点集的最佳拟合平面
   * 计算点集的协方差矩阵，最小奇异值对应的奇异向量为平面法向量
   * @param ground 地面点集
   */
  void estimate_plane(const vector<PointXYZ> &ground);

  /**
   * @brief 对指定区域进行分段地面提取
   * 逐个处理该区域的补丁，使用PCA和自适应阈值进行地面判定
   * @param zone_idx 同心圆环区域索引
   * @param src 该区域的输入点集
   * @param dst 输出的地面点集
   * @param non_ground_dst 输出的非地面点集
   */
  void extract_piecewiseground(const int zone_idx,
                               const vector<PointXYZ> &src,
                               vector<PointXYZ> &dst,
                               vector<PointXYZ> &non_ground_dst);

  /**
   * @brief 初始种子选择（使用默认阈值）
   * 选择每个环中最低的N个点作为地面的初始估计
   * @param zone_idx 同心圆环区域索引
   * @param p_sorted 按高度排序的点集（已排序）
   * @param init_seeds 输出的初始种子点集
   */
  void extract_initial_seeds(const int zone_idx,
                             const vector<PointXYZ> &p_sorted,
                             vector<PointXYZ> &init_seeds);

  /**
   * @brief 初始种子选择（使用自定义阈值）
   * @param zone_idx 同心圆环区域索引
   * @param p_sorted 按高度排序的点集（已排序）
   * @param init_seeds 输出的初始种子点集
   * @param th_seed 自定义的高度阈值
   */
  void extract_initial_seeds(const int zone_idx,
                             const vector<PointXYZ> &p_sorted,
                             vector<PointXYZ> &init_seeds,
                             double th_seed);
};

};  // namespace patchwork

#endif
