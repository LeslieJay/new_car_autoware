/************************************** File Info ****************************************
* @file:       tool_math.h                                                                   
* @author:     刘鸿彬                                                              
* @date:       2024-12-19 
* @version:    V0.0                                                                              
* @brief:      一些使用的数学计算公式函数
******************************************************************************************/
#ifndef TOOL_MATH_H

#define TOOL_MATH_H

# include "agv_config.h"

class Math_Tool
{
public:
    Math_Tool() = default;
    
    // 计算两点之间的欧几里得距离
    double distance(Point p1, Point p2);

    // 计算B样条基函数
    double basisFunction(int i, int p, double u,  std::vector<double>& knots);

    // 当前点到一群点的最小距离
    double minDistance(CurrentPose currentpose, std::vector<Point> points);

    // 计算NURBS曲线上的点
    Point evaluateNURBS(double u,  Trajectory& trajectory);

    // 计算贝塞尔曲线上的点
    Point evaluateBezier(double t,  Trajectory& trajectory);

    // 计算贝塞尔曲线在参数t处的切线方向角（解析导数）
    double evaluateBezierTangentAngle(double t, Trajectory& trajectory);

    // 计算NURBS曲线在参数u处的切线方向角（中心差分数值微分）
    double evaluateNURBSTangentAngle(double u, Trajectory& trajectory);

    // 计算直线上某一点的坐标
    Point evaluateStraightLine(double u,  Point& start_point,  Point& end_point);

    // 根据ordermessage中的起始点、终止点、轨迹信息，生成采样点
    std::vector<Point> generateTrajectory(Point start_point, Point end_point, Trajectory trajectory, double orientation);

    // 检测三点构成的角是否为锐角（尖点检测）
    // 参数：pA - 上一个点，pB - 当前点，pC - 下一个点
    // 返回：如果角ABC是锐角（小于90度），返回true，否则返回false
    bool isSharpPoint(Point pA, Point pB, Point pC);


private:

};


#endif // TOOL_MATH_H