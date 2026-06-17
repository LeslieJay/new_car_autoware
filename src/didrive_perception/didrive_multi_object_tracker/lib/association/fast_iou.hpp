#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <random>

using Point = std::pair<double, double>;
using Polygon = std::vector<Point>;

// ================== 工具函数 ==================

// 计算两点之间的距离平方
double sq_dist(const Point &a, const Point &b)
{
    double dx = a.first - b.first;
    double dy = a.second - b.second;
    return dx * dx + dy * dy;
}

// Douglas–Peucker 简化算法（递归）
void simplify_ramer_douglas_peucker(const Polygon &pointList, double epsilon, Polygon &out, int first, int last)
{
    if (last - first <= 1)
    {
        out.push_back(pointList[first]);
        return;
    }

    double maxDist = 0.0;
    int index = first;

    for (int i = first + 1; i < last; ++i)
    {
        double dist = 0.0;
        // 线段 pointList[first] -> pointList[last] 到 pointList[i] 的距离平方
        double x1 = pointList[first].first, y1 = pointList[first].second;
        double x2 = pointList[last].first, y2 = pointList[last].second;
        double x0 = pointList[i].first, y0 = pointList[i].second;

        double dx = x2 - x1;
        double dy = y2 - y1;
        double d = dx * dx + dy * dy;
        double cross = (y2 - y1) * (x0 - x1) - (x2 - x1) * (y0 - y1);
        dist = cross * cross / d;

        if (dist > maxDist)
        {
            maxDist = dist;
            index = i;
        }
    }

    if (maxDist > epsilon * epsilon)
    {
        simplify_ramer_douglas_peucker(pointList, epsilon, out, first, index);
        simplify_ramer_douglas_peucker(pointList, epsilon, out, index, last);
    }
    else
    {
        out.push_back(pointList[first]);
    }
}

Polygon simplify_polygon(const Polygon &poly, double epsilon)
{
    Polygon result;
    if (poly.empty())
        return result;
    simplify_ramer_douglas_peucker(poly, epsilon, result, 0, poly.size() - 1);
    result.push_back(poly.back());
    return result;
}

// ================== AABB 近似 IOU ==================

struct BoundingBox
{
    double xmin, ymin, xmax, ymax;

    double area() const
    {
        return (xmax - xmin) * (ymax - ymin);
    }
};

BoundingBox get_aabb(const Polygon &poly)
{
    if (poly.empty())
        return {0, 0, 0, 0};

    BoundingBox box;
    box.xmin = box.xmax = poly[0].first;
    box.ymin = box.ymax = poly[0].second;

    for (size_t i = 1; i < poly.size(); ++i)
    {
        box.xmin = std::min(box.xmin, poly[i].first);
        box.xmax = std::max(box.xmax, poly[i].first);
        box.ymin = std::min(box.ymin, poly[i].second);
        box.ymax = std::max(box.ymax, poly[i].second);
    }

    return box;
}

bool boxes_overlap(const BoundingBox &a, const BoundingBox &b)
{
    return !(a.xmax < b.xmin || b.xmax < a.xmin ||
             a.ymax < b.ymin || b.ymax < a.ymin);
}

double compute_iou_approximate(const Polygon &a, const Polygon &b, double simplification_eps = 0.05)
{
    // 打印 poly1 和 poly2 的顶点
    std::cout << "poly1:\n";
    for (const auto &pt : a)
    {
        std::cout << "(" << pt.first << ", " << pt.second << ") ";
    }
    std::cout << "\n";

    std::cout << "poly2:\n";
    for (const auto &pt : b)
    {
        std::cout << "(" << pt.first << ", " << pt.second << ") ";
    }
    std::cout << "\n";
    // Step 1: 简化多边形
    auto a_simple = simplify_polygon(a, simplification_eps);
    auto b_simple = simplify_polygon(b, simplification_eps);

    std::cout << "After simplification:\n";
    std::cout << "a_simple size: " << a_simple.size() << "\n";
    std::cout << "b_simple size: " << b_simple.size() << "\n";
    // Step 2: 获取包围盒
    auto box_a = get_aabb(a_simple);
    auto box_b = get_aabb(b_simple);

    if (!boxes_overlap(box_a, box_b))
    {
        return 0.0;
    }

    // Step 3: 将多边形投影到平面上，使用面积比近似 IOU
    // 假设形状接近矩形，使用 AABB 面积作为近似
    double intersection_area = std::max(0.0, std::min(box_a.xmax, box_b.xmax) - std::max(box_a.xmin, box_b.xmin)) *
                               std::max(0.0, std::min(box_a.ymax, box_b.ymax) - std::max(box_a.ymin, box_b.ymin));

    double union_area = box_a.area() + box_b.area() - intersection_area;

    return intersection_area / (union_area + 1e-6);
}

// ================== 测试用例 ==================

Polygon generate_random_polygon(int num_vertices, double center_x, double center_y, double radius)
{
    static std::mt19937 gen(42);
    static std::uniform_real_distribution<> angle_dist(0, 2 * M_PI);
    static std::uniform_real_distribution<> radius_noise(-radius * 0.2, radius * 0.2);

    Polygon poly;
    std::vector<double> angles;

    for (int i = 0; i < num_vertices; ++i)
    {
        angles.push_back(angle_dist(gen));
    }
    std::sort(angles.begin(), angles.end());

    for (double angle : angles)
    {
        double r = radius + radius_noise(gen);
        double x = center_x + r * cos(angle);
        double y = center_y + r * sin(angle);
        poly.emplace_back(x, y);
    }

    return poly;
}

int main()
{
    // 生成两个随机多边形
    auto poly1 = generate_random_polygon(8, 10, 10, 5);
    auto poly2 = generate_random_polygon(8, 10.5, 10.5, 5);

    // 测量耗时
    auto start = std::chrono::high_resolution_clock::now();

    double iou_approx = compute_iou_approximate(poly1, poly2, 0.05); // 允许 5cm 误差

    auto end = std::chrono::high_resolution_clock::now();
    auto duration_us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

    std::cout << "Approximate IOU: " << iou_approx << std::endl;
    std::cout << "Time taken: " << duration_us << " μs" << std::endl;

    return 0;
}