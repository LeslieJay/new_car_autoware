#include "didrive/multi_object_tracker/utils/utils.hpp"
namespace utils {

cv::RotatedRect GetMinAreaRect(const autoware_utils_geometry::Polygon2d& polygon) {
    std::vector<cv::Point2f> pts;
    pts.reserve(polygon.outer().size());
    for (const auto& p : polygon.outer()) {
        pts.emplace_back(cv::Point2f(boost::geometry::get<0>(p),
                   boost::geometry::get<1>(p)));
    }
    return cv::minAreaRect(pts); // OpenCV 自带最小包围矩形算法
}

double ComputeRotatedRectIoU(const cv::RotatedRect& r1, const cv::RotatedRect& r2) {
  // 计算交集区域
  std::vector<cv::Point2f> intersectingRegion;
 
  cv::rotatedRectangleIntersection(r1, r2, intersectingRegion);

  float inter_area = 0;
  float area_1 = r1.size.area();
  float area_2 = r2.size.area();
  if (intersectingRegion.empty()) {
    inter_area = 0;
  } else {
    // 计算交集区域的面积
    // std::vector<cv::Point2f> hull;
    // cv::convexHull(intersectingRegion, hull);
    inter_area = cv::contourArea(intersectingRegion);
  }
  return inter_area / (area_1 + area_2 - inter_area + 1e-6);
}

}