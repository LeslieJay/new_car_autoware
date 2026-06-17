// Copyright 2026 BYD. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef REVERSE_PARKING_PLANNER__REEDS_SHEPP_HPP_
#define REVERSE_PARKING_PLANNER__REEDS_SHEPP_HPP_

#include <vector>
#include <cmath>
#include <limits>

namespace reverse_parking_planner
{

/**
 * @brief Reeds-Shepp路径段类型
 */
enum class SegmentType
{
  NOP = 0,      // 无操作
  LEFT = 1,     // 左转
  STRAIGHT = 2, // 直行
  RIGHT = 3     // 右转
};

/**
 * @brief 路径点结构体
 */
struct PathPoint
{
  double x;
  double y;
  double yaw;
  double curvature{0.0};  // 曲率 [1/m]
  bool is_reverse;  // 是否为倒车段
  
  PathPoint(double x_ = 0, double y_ = 0, double yaw_ = 0, bool reverse = false, double curv = 0.0)
    : x(x_), y(y_), yaw(yaw_), curvature(curv), is_reverse(reverse) {}
};

/**
 * @brief Reeds-Shepp路径
 */
class ReedsSheppPath
{
public:
  ReedsSheppPath();
  
  void setSegments(const SegmentType* types, double t, double u, double v, double w = 0, double x = 0);
  
  double length() const { return total_length_; }
  bool valid() const { return total_length_ < std::numeric_limits<double>::max() / 2.0; }
  
  SegmentType types_[5];
  double lengths_[5];
  double total_length_;
};

/**
 * @brief Reeds-Shepp状态空间规划器
 * 
 * 实现Reeds-Shepp曲线，用于计算考虑前进和倒车的最短路径
 */
class ReedsSheppPlanner
{
public:
  /**
   * @brief 构造函数
   * @param turning_radius 最小转弯半径 [m]
   */
  explicit ReedsSheppPlanner(double turning_radius);
  
  /**
   * @brief 设置转弯半径
   */
  void setTurningRadius(double radius) { turning_radius_ = radius; }
  
  /**
   * @brief 计算两个状态之间的Reeds-Shepp距离
   */
  double distance(double x0, double y0, double yaw0, 
                  double x1, double y1, double yaw1) const;
  
  /**
   * @brief 计算最优Reeds-Shepp路径
   */
  ReedsSheppPath planPath(double x0, double y0, double yaw0,
                          double x1, double y1, double yaw1) const;
  
  /**
   * @brief 采样路径点
   * @param path Reeds-Shepp路径
   * @param x0, y0, yaw0 起始状态
   * @param resolution 采样分辨率
   * @return 路径点序列
   */
  std::vector<PathPoint> samplePath(const ReedsSheppPath& path,
                                     double x0, double y0, double yaw0,
                                     double resolution) const;

private:
  double turning_radius_;
  
  // 18种Reeds-Shepp路径类型
  static const SegmentType path_types_[18][5];
  
  // 路径计算辅助函数
  ReedsSheppPath computeOptimalPath(double x, double y, double phi) const;
  
  // 各类路径公式 (来自Reeds-Shepp论文)
  bool LpSpLp(double x, double y, double phi, double& t, double& u, double& v) const;
  bool LpSpRp(double x, double y, double phi, double& t, double& u, double& v) const;
  bool LpRmL(double x, double y, double phi, double& t, double& u, double& v) const;
  bool LpRupLumRm(double x, double y, double phi, double& t, double& u, double& v) const;
  bool LpRumLumRp(double x, double y, double phi, double& t, double& u, double& v) const;
  bool LpRmSmLm(double x, double y, double phi, double& t, double& u, double& v) const;
  bool LpRmSmRm(double x, double y, double phi, double& t, double& u, double& v) const;
  bool LpRmSLmRp(double x, double y, double phi, double& t, double& u, double& v) const;
  
  // 工具函数
  static double mod2pi(double x);
  static void polar(double x, double y, double& r, double& theta);
  static void tauOmega(double u, double v, double xi, double eta, double phi, 
                       double& tau, double& omega);
  
  // 路径插值
  PathPoint interpolate(double x0, double y0, double yaw0,
                        const ReedsSheppPath& path, double s) const;
};

}  // namespace reverse_parking_planner

#endif  // REVERSE_PARKING_PLANNER__REEDS_SHEPP_HPP_
