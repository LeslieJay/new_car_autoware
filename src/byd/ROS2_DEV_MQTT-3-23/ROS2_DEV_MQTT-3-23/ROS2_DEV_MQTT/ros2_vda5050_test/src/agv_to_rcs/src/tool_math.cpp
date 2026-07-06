/************************************** File Info ****************************************
* @file:       tool_math.cpp                                                                  
* @author:     刘鸿彬                                                              
* @date:       2024-12-19 
* @version:    V0.0                                                                              
* @brief:      一些使用的数学计算公式函数
******************************************************************************************/

# include "tool_math.h"


// 计算两点之间的欧几里得距离
double Math_Tool::distance(Point p1, Point p2) 
{
    return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
}

// 计算B样条基函数
double Math_Tool::basisFunction(int i, int p, double u,  std::vector<double>& knots) {
    if (p == 0) {
        return (knots[i] <= u && u < knots[i + 1]) ? 1.0 : 0.0;
    }

    double denom1 = knots[i + p] - knots[i];
    double term1 = 0.0;
    if (denom1 > 0) {
        term1 = (u - knots[i]) / denom1 * basisFunction(i, p - 1, u, knots);
    }

    double denom2 = knots[i + p + 1] - knots[i + 1];
    double term2 = 0.0;
    if (denom2 > 0) {
        term2 = (knots[i + p + 1] - u) / denom2 * basisFunction(i + 1, p - 1, u, knots);
    }

    return term1 + term2;
}

// 当前点到一群点的最小距离
double Math_Tool::minDistance(CurrentPose currentpose, std::vector<Point> points)
{
    double min_distance = 99999999.9;
    double distance;
    if(points.empty())
        return 0.0;
    else
    {
        for(size_t i=0; i<points.size(); i++)
        {
            distance = std::sqrt((points[i].x-currentpose.current_x) * (points[i].x-currentpose.current_x) + (points[i].y-currentpose.current_y) * (points[i].y-currentpose.current_y));
            if(distance < min_distance)
                min_distance = distance;
        }
    }

    return min_distance;
}

// 计算NURBS曲线上的点
Point Math_Tool::evaluateNURBS(double u,  Trajectory& trajectory) {
    int p = trajectory.degree;
    auto& knots = trajectory.knots;
    auto& control_points = trajectory.control_points;

    double w = 0.0;
    Point point;
    point.x = 0;
    point.y = 0;
    point.theta = 0;

    if(-0.0001 < u && u < 0.0001) // 如果是第一个取样点，则返回起点
    {
        point.x = trajectory.control_points[0].x;
        point.y = trajectory.control_points[0].y;
        return point;
    }
    if(-0.0001 < u-1 && u-1 < 0.0001) // 如果是最后一个取样点，则返回终点
    {
        point.x = trajectory.control_points[trajectory.control_points.size()-1].x;
        point.y = trajectory.control_points[trajectory.control_points.size()-1].y;
        return point;
    }

    for (size_t i = 0; i < control_points.size(); ++i) {
        double N = basisFunction(i, p, u, knots);
        double weighted_N = N * control_points[i].weight;

        point.x += weighted_N * control_points[i].x;
        point.y += weighted_N * control_points[i].y;
        w += weighted_N;
    }

    if (w != 0.0) {
        point.x /= w;
        point.y /= w;
    }

    return point;
}

// 计算贝塞尔曲线上的点
Point Math_Tool::evaluateBezier(double t,  Trajectory& trajectory) 
{
    auto& control_points = trajectory.control_points;
    int n = control_points.size() - 1;

    Point point;
    point.x = 0;
    point.y = 0;
    point.theta = 0;

    for (int i = 0; i <= n; ++i) {
        double binomial_coeff = std::tgamma(n + 1) / (std::tgamma(i + 1) * std::tgamma(n - i + 1));
        double bernstein = binomial_coeff * std::pow(t, i) * std::pow(1 - t, n - i);

        point.x += bernstein * control_points[i].x;
        point.y += bernstein * control_points[i].y;
    }

    return point;
}

// 计算贝塞尔曲线在参数t处的切线方向角（解析导数）
// 公式：B'(t) = n * Σ_{i=0}^{n-1} C(n-1,i) * t^i * (1-t)^{n-1-i} * (P_{i+1} - P_i)
double Math_Tool::evaluateBezierTangentAngle(double t, Trajectory& trajectory)
{
    auto& control_points = trajectory.control_points;
    int n = control_points.size() - 1;  // 曲线阶数

    double dx = 0.0, dy = 0.0;
    int m = n - 1;  // 导数曲线阶数

    for (int i = 0; i <= m; ++i) {
        double binomial = std::tgamma(m + 1) / (std::tgamma(i + 1) * std::tgamma(m - i + 1));
        double bernstein = binomial * std::pow(t, i) * std::pow(1.0 - t, m - i);
        dx += bernstein * (control_points[i + 1].x - control_points[i].x);
        dy += bernstein * (control_points[i + 1].y - control_points[i].y);
    }
    // atan2 只关心方向，n 系数不影响结果，但保留语义
    return std::atan2(dy, dx);
}

// 计算NURBS曲线在参数u处的切线方向角（中心差分数值微分）
double Math_Tool::evaluateNURBSTangentAngle(double u, Trajectory& trajectory)
{
    const double epsilon = 1e-3;
    double u1 = u - epsilon;
    double u2 = u + epsilon;

    // 边界处退化为单侧差分
    if (u1 < 0.0) u1 = 0.0;
    if (u2 > 1.0) u2 = 1.0;

    Point p1 = evaluateNURBS(u1, trajectory);
    Point p2 = evaluateNURBS(u2, trajectory);

    return std::atan2(p2.y - p1.y, p2.x - p1.x);
}

// 计算直线上某一点的坐标
Point Math_Tool::evaluateStraightLine(double u,  Point& start_point,  Point& end_point) 
{
    Point point;
    point.x = 0;
    point.y = 0;
    point.theta = 0;

    // 确保 u 在 [0, 1] 范围内
    if (u < 0.0) u = 0.0;
    if (u > 1.0) u = 1.0;

    // 使用线性插值公式计算点的坐标
    point.x = (1 - u) * start_point.x + u * end_point.x;
    point.y = (1 - u) * start_point.y + u * end_point.y;

    // 计算直线的方向角
    double dx = end_point.x - start_point.x;
    double dy = end_point.y - start_point.y;
    point.theta = std::atan2(dy, dx); // 方向角（弧度）

    return point;
}

// 根据ordermessage中的起始点、终止点、轨迹信息，生成采样点
std::vector<Point> Math_Tool::generateTrajectory(Point start_point, Point end_point, Trajectory trajectory, double orientation)
{
    // 如果degree为-1，返回空集合
    if (trajectory.degree == -1)
    {
        return std::vector<Point>();
    }
    
    // 参数设置
    double step_distance = agv_config.micro_distance; // 相邻点之间距离

    // 计算总距离
    double total_distance = distance(start_point, end_point);
    int num_steps = static_cast<int>(std::round(total_distance / step_distance));

    // 如果总距离小于步长，则只输出起点和终点
    if (num_steps < 1) num_steps = 1;

    // 生成采样点
    std::vector<Point> sampled_points;

    Point last_point = start_point;
    for (int i = 1; i <= num_steps; ++i) { // 注意：i从1开始，表示曲线不取样起始点，因为小车已经在起始点上了
        double t = static_cast<double>(i) / num_steps; // 参数 t 或 u 的值
        Point point;

        // 分类逻辑重新设计：
        // 1. knot不为空：B样条曲线（NURBS）
        // 2. knot为空：
        //    - 控制点个数 = 2：直线
        //    - 控制点个数 >= 3：贝塞尔曲线
        //    - 其他情况：报错
        
        if (!trajectory.knots.empty())
        {
            // B样条曲线（NURBS）
            point = evaluateNURBS(t, trajectory);
        }
        else if (trajectory.control_points.empty())
        {
            // 没有控制点，使用起点终点生成直线
            point = evaluateStraightLine(t, start_point, end_point);
        }
        else if (trajectory.control_points.size() == 2)
        {
            // 2个控制点：直线
            point = evaluateStraightLine(t, start_point, end_point);
        }
        else if (trajectory.control_points.size() >= 3)
        {
            // 3个或更多控制点：贝塞尔曲线
            point = evaluateBezier(t, trajectory);
        }
        else
        {
            // 非法情况（控制点个数为1）
            std::cout << "ERROR! Illegal trajectory! Control points size: " << trajectory.control_points.size() << std::endl;
            break;
        }

        double PI = agv_config.PI;
        double tangent_angle;
        if (!trajectory.knots.empty()) {
            // NURBS：中心差分数值微分求真实切线角
            tangent_angle = evaluateNURBSTangentAngle(t, trajectory);
        } else if (trajectory.control_points.size() >= 3) {
            // 贝塞尔曲线：解析导数求真实切线角
            tangent_angle = evaluateBezierTangentAngle(t, trajectory);
        } else {
            // 直线：切线方向恒定，割线即切线
            tangent_angle = std::atan2(point.y - last_point.y, point.x - last_point.x);
        }
        point.theta = tangent_angle + orientation;
        point.theta = std::fmod(point.theta, 2 * PI);
        while(point.theta > PI)
            point.theta -= 2 * PI;
        while(point.theta <= -1 * PI)
            point.theta += 2 * PI;

        last_point = point;
        sampled_points.push_back(point);
    }

    return sampled_points;
}

// 检测三点构成的角是否为锐角（尖点检测）
bool Math_Tool::isSharpPoint(Point pA, Point pB, Point pC)
{
    // 计算向量 BA = (xA - xB, yA - yB) 和向量 BC = (xC - xB, yC - yB)
    double ba_x = pA.x - pB.x;
    double ba_y = pA.y - pB.y;
    double bc_x = pC.x - pB.x;
    double bc_y = pC.y - pB.y;
    
    // 计算向量长度
    double len_ba = std::sqrt(ba_x * ba_x + ba_y * ba_y);
    double len_bc = std::sqrt(bc_x * bc_x + bc_y * bc_y);
    
    // 如果向量长度过小，无法计算角度，返回false
    if(len_ba <= 1e-6 || len_bc <= 1e-6)
    {
        return false;
    }
    
    // 计算点积
    double dot_product = ba_x * bc_x + ba_y * bc_y;
    
    // 计算余弦值
    double cos_angle = dot_product / (len_ba * len_bc);
    
    // 限制余弦值在[-1, 1]范围内，防止数值误差
    cos_angle = std::max(-1.0, std::min(1.0, cos_angle));
    
    // 计算角度（弧度）
    double angle = std::acos(cos_angle);
    
    // 如果角度小于90度（π/2），则为锐角，是尖点
    return angle < M_PI / 2.0;
}

