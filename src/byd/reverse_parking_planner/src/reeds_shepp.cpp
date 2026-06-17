// Copyright 2026 BYD. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "reverse_parking_planner/reeds_shepp.hpp"

#include <algorithm>
#include <cmath>

namespace reverse_parking_planner
{

namespace
{
const double PI = M_PI;
const double TWO_PI = 2.0 * PI;
const double ZERO = 10 * std::numeric_limits<double>::epsilon();
}  // namespace

// 18种Reeds-Shepp路径类型定义
const SegmentType ReedsSheppPlanner::path_types_[18][5] = {
  {SegmentType::LEFT, SegmentType::RIGHT, SegmentType::LEFT, SegmentType::NOP, SegmentType::NOP},        // 0
  {SegmentType::RIGHT, SegmentType::LEFT, SegmentType::RIGHT, SegmentType::NOP, SegmentType::NOP},       // 1
  {SegmentType::LEFT, SegmentType::RIGHT, SegmentType::LEFT, SegmentType::RIGHT, SegmentType::NOP},      // 2
  {SegmentType::RIGHT, SegmentType::LEFT, SegmentType::RIGHT, SegmentType::LEFT, SegmentType::NOP},      // 3
  {SegmentType::LEFT, SegmentType::RIGHT, SegmentType::STRAIGHT, SegmentType::LEFT, SegmentType::NOP},   // 4
  {SegmentType::RIGHT, SegmentType::LEFT, SegmentType::STRAIGHT, SegmentType::RIGHT, SegmentType::NOP},  // 5
  {SegmentType::LEFT, SegmentType::STRAIGHT, SegmentType::RIGHT, SegmentType::LEFT, SegmentType::NOP},   // 6
  {SegmentType::RIGHT, SegmentType::STRAIGHT, SegmentType::LEFT, SegmentType::RIGHT, SegmentType::NOP},  // 7
  {SegmentType::LEFT, SegmentType::RIGHT, SegmentType::STRAIGHT, SegmentType::RIGHT, SegmentType::NOP},  // 8
  {SegmentType::RIGHT, SegmentType::LEFT, SegmentType::STRAIGHT, SegmentType::LEFT, SegmentType::NOP},   // 9
  {SegmentType::RIGHT, SegmentType::STRAIGHT, SegmentType::RIGHT, SegmentType::LEFT, SegmentType::NOP},  // 10
  {SegmentType::LEFT, SegmentType::STRAIGHT, SegmentType::LEFT, SegmentType::RIGHT, SegmentType::NOP},   // 11
  {SegmentType::LEFT, SegmentType::STRAIGHT, SegmentType::RIGHT, SegmentType::NOP, SegmentType::NOP},    // 12
  {SegmentType::RIGHT, SegmentType::STRAIGHT, SegmentType::LEFT, SegmentType::NOP, SegmentType::NOP},    // 13
  {SegmentType::LEFT, SegmentType::STRAIGHT, SegmentType::LEFT, SegmentType::NOP, SegmentType::NOP},     // 14
  {SegmentType::RIGHT, SegmentType::STRAIGHT, SegmentType::RIGHT, SegmentType::NOP, SegmentType::NOP},   // 15
  {SegmentType::LEFT, SegmentType::RIGHT, SegmentType::STRAIGHT, SegmentType::LEFT, SegmentType::RIGHT}, // 16
  {SegmentType::RIGHT, SegmentType::LEFT, SegmentType::STRAIGHT, SegmentType::RIGHT, SegmentType::LEFT}  // 17
};

ReedsSheppPath::ReedsSheppPath()
  : total_length_(std::numeric_limits<double>::max())
{
  for (int i = 0; i < 5; ++i) {
    types_[i] = SegmentType::NOP;
    lengths_[i] = 0.0;
  }
}

void ReedsSheppPath::setSegments(const SegmentType* types, double t, double u, double v, double w, double x)
{
  for (int i = 0; i < 5; ++i) {
    types_[i] = types[i];
  }
  lengths_[0] = t;
  lengths_[1] = u;
  lengths_[2] = v;
  lengths_[3] = w;
  lengths_[4] = x;
  total_length_ = std::abs(t) + std::abs(u) + std::abs(v) + std::abs(w) + std::abs(x);
}

ReedsSheppPlanner::ReedsSheppPlanner(double turning_radius)
  : turning_radius_(turning_radius)
{
}

double ReedsSheppPlanner::mod2pi(double x)
{
  double v = std::fmod(x, TWO_PI);
  if (v < -PI) {
    v += TWO_PI;
  } else if (v > PI) {
    v -= TWO_PI;
  }
  return v;
}

void ReedsSheppPlanner::polar(double x, double y, double& r, double& theta)
{
  r = std::sqrt(x * x + y * y);
  theta = std::atan2(y, x);
}

void ReedsSheppPlanner::tauOmega(double u, double v, double xi, double eta, double phi,
                                  double& tau, double& omega)
{
  double delta = mod2pi(u - v);
  double A = std::sin(u) - std::sin(delta);
  double B = std::cos(u) - std::cos(delta) - 1.0;
  double t1 = std::atan2(eta * A - xi * B, xi * A + eta * B);
  double t2 = 2.0 * (std::cos(delta) - std::cos(v) - std::cos(u)) + 3.0;
  tau = (t2 < 0) ? mod2pi(t1 + PI) : mod2pi(t1);
  omega = mod2pi(tau - u + v - phi);
}

// Formula 8.1: L+S+L+
bool ReedsSheppPlanner::LpSpLp(double x, double y, double phi, double& t, double& u, double& v) const
{
  polar(x - std::sin(phi), y - 1.0 + std::cos(phi), u, t);
  if (t >= -ZERO) {
    v = mod2pi(phi - t);
    if (v >= -ZERO) {
      return true;
    }
  }
  return false;
}

// Formula 8.2: L+S+R+
bool ReedsSheppPlanner::LpSpRp(double x, double y, double phi, double& t, double& u, double& v) const
{
  double t1, u1;
  polar(x + std::sin(phi), y - 1.0 - std::cos(phi), u1, t1);
  u1 = u1 * u1;
  if (u1 >= 4.0) {
    double theta;
    u = std::sqrt(u1 - 4.0);
    theta = std::atan2(2.0, u);
    t = mod2pi(t1 + theta);
    v = mod2pi(t - phi);
    if (t >= -ZERO && v >= -ZERO) {
      return true;
    }
  }
  return false;
}

// Formula 8.3/8.4: L+R-L-
bool ReedsSheppPlanner::LpRmL(double x, double y, double phi, double& t, double& u, double& v) const
{
  double xi = x - std::sin(phi);
  double eta = y - 1.0 + std::cos(phi);
  double r, theta;
  polar(xi, eta, r, theta);
  if (r <= 4.0) {
    u = -2.0 * std::asin(0.25 * r);
    t = mod2pi(theta + 0.5 * u + PI);
    v = mod2pi(phi - t + u);
    if (t >= -ZERO && u <= ZERO) {
      return true;
    }
  }
  return false;
}

// Formula 8.7: L+R+L-R-
bool ReedsSheppPlanner::LpRupLumRm(double x, double y, double phi, double& t, double& u, double& v) const
{
  double xi = x + std::sin(phi);
  double eta = y - 1.0 - std::cos(phi);
  double rho = 0.25 * (2.0 + std::sqrt(xi * xi + eta * eta));
  if (rho <= 1.0) {
    u = std::acos(rho);
    tauOmega(u, -u, xi, eta, phi, t, v);
    if (t >= -ZERO && v <= ZERO) {
      return true;
    }
  }
  return false;
}

// Formula 8.8: L+R-L-R+
bool ReedsSheppPlanner::LpRumLumRp(double x, double y, double phi, double& t, double& u, double& v) const
{
  double xi = x + std::sin(phi);
  double eta = y - 1.0 - std::cos(phi);
  double rho = (20.0 - xi * xi - eta * eta) / 16.0;
  if (rho >= 0.0 && rho <= 1.0) {
    u = -std::acos(rho);
    if (u >= -0.5 * PI) {
      tauOmega(u, u, xi, eta, phi, t, v);
      if (t >= -ZERO && v >= -ZERO) {
        return true;
      }
    }
  }
  return false;
}

// Formula 8.9: L+R-S-L-
bool ReedsSheppPlanner::LpRmSmLm(double x, double y, double phi, double& t, double& u, double& v) const
{
  double xi = x - std::sin(phi);
  double eta = y - 1.0 + std::cos(phi);
  double r, theta;
  polar(xi, eta, r, theta);
  if (r >= 2.0) {
    double rr = std::sqrt(r * r - 4.0);
    u = 2.0 - rr;
    t = mod2pi(theta + std::atan2(rr, -2.0));
    v = mod2pi(phi - 0.5 * PI - t);
    if (t >= -ZERO && u <= ZERO && v <= ZERO) {
      return true;
    }
  }
  return false;
}

// Formula 8.10: L+R-S-R-
bool ReedsSheppPlanner::LpRmSmRm(double x, double y, double phi, double& t, double& u, double& v) const
{
  double xi = x + std::sin(phi);
  double eta = y - 1.0 - std::cos(phi);
  double r, theta;
  polar(-eta, xi, r, theta);
  if (r >= 2.0) {
    t = theta;
    u = 2.0 - r;
    v = mod2pi(t + 0.5 * PI - phi);
    if (t >= -ZERO && u <= ZERO && v <= ZERO) {
      return true;
    }
  }
  return false;
}

// Formula 8.11: L+R-S-L-R+
bool ReedsSheppPlanner::LpRmSLmRp(double x, double y, double phi, double& t, double& u, double& v) const
{
  double xi = x + std::sin(phi);
  double eta = y - 1.0 - std::cos(phi);
  double r, theta;
  polar(xi, eta, r, theta);
  if (r >= 2.0) {
    u = 4.0 - std::sqrt(r * r - 4.0);
    if (u <= ZERO) {
      t = mod2pi(std::atan2((4.0 - u) * xi - 2.0 * eta, -2.0 * xi + (u - 4.0) * eta));
      v = mod2pi(t - phi);
      if (t >= -ZERO && v >= -ZERO) {
        return true;
      }
    }
  }
  return false;
}

ReedsSheppPath ReedsSheppPlanner::computeOptimalPath(double x, double y, double phi) const
{
  ReedsSheppPath best_path;
  double t, u, v;
  
  // 尝试所有可能的路径类型，选择最短的
  auto tryPath = [&](int path_idx, double t_val, double u_val, double v_val, 
                     double w_val = 0.0, double x_val = 0.0) {
    ReedsSheppPath path;
    path.setSegments(path_types_[path_idx], t_val, u_val, v_val, w_val, x_val);
    if (path.length() < best_path.length()) {
      best_path = path;
    }
  };
  
  // CSC paths (公式 8.1, 8.2)
  if (LpSpLp(x, y, phi, t, u, v)) tryPath(14, t, u, v);
  if (LpSpLp(-x, y, -phi, t, u, v)) tryPath(14, -t, -u, -v);
  if (LpSpLp(x, -y, -phi, t, u, v)) tryPath(15, t, u, v);
  if (LpSpLp(-x, -y, phi, t, u, v)) tryPath(15, -t, -u, -v);
  
  if (LpSpRp(x, y, phi, t, u, v)) tryPath(12, t, u, v);
  if (LpSpRp(-x, y, -phi, t, u, v)) tryPath(12, -t, -u, -v);
  if (LpSpRp(x, -y, -phi, t, u, v)) tryPath(13, t, u, v);
  if (LpSpRp(-x, -y, phi, t, u, v)) tryPath(13, -t, -u, -v);
  
  // CCC paths (公式 8.3, 8.4)
  if (LpRmL(x, y, phi, t, u, v)) tryPath(0, t, u, v);
  if (LpRmL(-x, y, -phi, t, u, v)) tryPath(0, -t, -u, -v);
  if (LpRmL(x, -y, -phi, t, u, v)) tryPath(1, t, u, v);
  if (LpRmL(-x, -y, phi, t, u, v)) tryPath(1, -t, -u, -v);
  
  // CCCC paths (公式 8.7, 8.8)
  if (LpRupLumRm(x, y, phi, t, u, v)) tryPath(2, t, u, -u, v);
  if (LpRupLumRm(-x, y, -phi, t, u, v)) tryPath(2, -t, -u, u, -v);
  if (LpRupLumRm(x, -y, -phi, t, u, v)) tryPath(3, t, u, -u, v);
  if (LpRupLumRm(-x, -y, phi, t, u, v)) tryPath(3, -t, -u, u, -v);
  
  if (LpRumLumRp(x, y, phi, t, u, v)) tryPath(2, t, u, u, v);
  if (LpRumLumRp(-x, y, -phi, t, u, v)) tryPath(2, -t, -u, -u, -v);
  if (LpRumLumRp(x, -y, -phi, t, u, v)) tryPath(3, t, u, u, v);
  if (LpRumLumRp(-x, -y, phi, t, u, v)) tryPath(3, -t, -u, -u, -v);
  
  // CCSC paths (公式 8.9, 8.10)
  if (LpRmSmLm(x, y, phi, t, u, v)) tryPath(4, t, -0.5 * PI, u, v);
  if (LpRmSmLm(-x, y, -phi, t, u, v)) tryPath(4, -t, 0.5 * PI, -u, -v);
  if (LpRmSmLm(x, -y, -phi, t, u, v)) tryPath(5, t, -0.5 * PI, u, v);
  if (LpRmSmLm(-x, -y, phi, t, u, v)) tryPath(5, -t, 0.5 * PI, -u, -v);
  
  if (LpRmSmRm(x, y, phi, t, u, v)) tryPath(8, t, -0.5 * PI, u, v);
  if (LpRmSmRm(-x, y, -phi, t, u, v)) tryPath(8, -t, 0.5 * PI, -u, -v);
  if (LpRmSmRm(x, -y, -phi, t, u, v)) tryPath(9, t, -0.5 * PI, u, v);
  if (LpRmSmRm(-x, -y, phi, t, u, v)) tryPath(9, -t, 0.5 * PI, -u, -v);
  
  // CCSCC paths (公式 8.11)
  if (LpRmSLmRp(x, y, phi, t, u, v)) tryPath(16, t, -0.5 * PI, u, -0.5 * PI, v);
  if (LpRmSLmRp(-x, y, -phi, t, u, v)) tryPath(16, -t, 0.5 * PI, -u, 0.5 * PI, -v);
  if (LpRmSLmRp(x, -y, -phi, t, u, v)) tryPath(17, t, -0.5 * PI, u, -0.5 * PI, v);
  if (LpRmSLmRp(-x, -y, phi, t, u, v)) tryPath(17, -t, 0.5 * PI, -u, 0.5 * PI, -v);
  
  return best_path;
}

double ReedsSheppPlanner::distance(double x0, double y0, double yaw0,
                                    double x1, double y1, double yaw1) const
{
  // 转换到以起点为原点的局部坐标系
  double dx = x1 - x0;
  double dy = y1 - y0;
  double c = std::cos(yaw0);
  double s = std::sin(yaw0);
  
  double x = (c * dx + s * dy) / turning_radius_;
  double y = (-s * dx + c * dy) / turning_radius_;
  double phi = mod2pi(yaw1 - yaw0);
  
  ReedsSheppPath path = computeOptimalPath(x, y, phi);
  return path.length() * turning_radius_;
}

ReedsSheppPath ReedsSheppPlanner::planPath(double x0, double y0, double yaw0,
                                            double x1, double y1, double yaw1) const
{
  // 转换到以起点为原点的局部坐标系（归一化）
  double dx = x1 - x0;
  double dy = y1 - y0;
  double c = std::cos(yaw0);
  double s = std::sin(yaw0);
  
  double x = (c * dx + s * dy) / turning_radius_;
  double y = (-s * dx + c * dy) / turning_radius_;
  double phi = mod2pi(yaw1 - yaw0);
  
  return computeOptimalPath(x, y, phi);
}

PathPoint ReedsSheppPlanner::interpolate(double x0, double y0, double yaw0,
                                          const ReedsSheppPath& path, double s) const
{
  double x = x0;
  double y = y0;
  double yaw = yaw0;
  bool is_reverse = false;
  double curvature = 0.0;
  
  double seg = s / turning_radius_;  // 归一化
  
  for (int i = 0; i < 5 && path.types_[i] != SegmentType::NOP; ++i) {
    double len = path.lengths_[i];
    double abs_len = std::abs(len);
    
    // 计算当前段的几何曲率
    double seg_curvature = 0.0;
    if (path.types_[i] == SegmentType::LEFT) {
      seg_curvature = 1.0 / turning_radius_;
    } else if (path.types_[i] == SegmentType::RIGHT) {
      seg_curvature = -1.0 / turning_radius_;
    }
    
    if (seg <= abs_len) {
      // 在当前段内
      double d = (len >= 0) ? seg : -seg;
      is_reverse = (len < 0);
      curvature = seg_curvature;
      
      switch (path.types_[i]) {
        case SegmentType::LEFT:
          x += turning_radius_ * (std::sin(yaw + d) - std::sin(yaw));
          y += turning_radius_ * (-std::cos(yaw + d) + std::cos(yaw));
          yaw = mod2pi(yaw + d);
          break;
        case SegmentType::RIGHT:
          x += turning_radius_ * (-std::sin(yaw - d) + std::sin(yaw));
          y += turning_radius_ * (std::cos(yaw - d) - std::cos(yaw));
          yaw = mod2pi(yaw - d);
          break;
        case SegmentType::STRAIGHT:
          x += turning_radius_ * d * std::cos(yaw);
          y += turning_radius_ * d * std::sin(yaw);
          break;
        default:
          break;
      }
      return PathPoint(x, y, yaw, is_reverse, curvature);
    }
    
    // 完成当前段
    double d = len;
    is_reverse = (len < 0);
    curvature = seg_curvature;
    
    switch (path.types_[i]) {
      case SegmentType::LEFT:
        x += turning_radius_ * (std::sin(yaw + d) - std::sin(yaw));
        y += turning_radius_ * (-std::cos(yaw + d) + std::cos(yaw));
        yaw = mod2pi(yaw + d);
        break;
      case SegmentType::RIGHT:
        x += turning_radius_ * (-std::sin(yaw - d) + std::sin(yaw));
        y += turning_radius_ * (std::cos(yaw - d) - std::cos(yaw));
        yaw = mod2pi(yaw - d);
        break;
      case SegmentType::STRAIGHT:
        x += turning_radius_ * d * std::cos(yaw);
        y += turning_radius_ * d * std::sin(yaw);
        break;
      default:
        break;
    }
    
    seg -= abs_len;
  }
  
  return PathPoint(x, y, yaw, is_reverse, curvature);
}

std::vector<PathPoint> ReedsSheppPlanner::samplePath(const ReedsSheppPath& path,
                                                      double x0, double y0, double yaw0,
                                                      double resolution) const
{
  std::vector<PathPoint> points;
  
  if (!path.valid()) {
    return points;
  }
  
  double total_length = path.length() * turning_radius_;
  int num_points = static_cast<int>(std::ceil(total_length / resolution)) + 1;
  
  for (int i = 0; i <= num_points; ++i) {
    double s = std::min(static_cast<double>(i) * resolution, total_length);
    points.push_back(interpolate(x0, y0, yaw0, path, s));
  }
  
  return points;
}

}  // namespace reverse_parking_planner
