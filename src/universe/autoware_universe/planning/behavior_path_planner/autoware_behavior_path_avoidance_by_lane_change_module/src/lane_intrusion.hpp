#ifndef AVOIDANCE_BY_LANE_CHANGE_LANE_INTRUSION_HPP_
#define AVOIDANCE_BY_LANE_CHANGE_LANE_INTRUSION_HPP_

#include <autoware_utils/geometry/geometry.hpp>

#include <boost/geometry/algorithms/area.hpp>
#include <boost/geometry/algorithms/intersection.hpp>
#include <boost/geometry/algorithms/is_valid.hpp>

#include <vector>

namespace autoware::behavior_path_planner
{

inline std::vector<autoware_utils::Polygon2d> positiveAreaOverlap(
  const autoware_utils::Polygon2d & object, const autoware_utils::Polygon2d & lane)
{
  std::vector<autoware_utils::Polygon2d> result;
  if (!boost::geometry::is_valid(object) || !boost::geometry::is_valid(lane) ||
      boost::geometry::area(object) <= 1e-6 || boost::geometry::area(lane) <= 1e-6) {
    return result;
  }
  std::vector<autoware_utils::Polygon2d> intersections;
  boost::geometry::intersection(object, lane, intersections);
  for (const auto & polygon : intersections) {
    if (boost::geometry::area(polygon) > 1e-6) result.push_back(polygon);
  }
  return result;
}

}  // namespace autoware::behavior_path_planner

#endif  // AVOIDANCE_BY_LANE_CHANGE_LANE_INTRUSION_HPP_
