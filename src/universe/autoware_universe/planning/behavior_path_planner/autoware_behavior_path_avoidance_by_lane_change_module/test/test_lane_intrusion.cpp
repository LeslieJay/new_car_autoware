#include "lane_intrusion.hpp"

#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/io/wkt/read.hpp>

#include <cassert>
#include <string>

using autoware_utils::Polygon2d;
using autoware::behavior_path_planner::positiveAreaOverlap;

Polygon2d polygon(const std::string & wkt)
{
  Polygon2d value;
  boost::geometry::read_wkt(wkt, value);
  boost::geometry::correct(value);
  return value;
}

int main()
{
  const auto lane = polygon("POLYGON((0 0,10 0,10 4,0 4,0 0))");
  const auto concave = polygon(
    "POLYGON((9 3,11 3,11 5,10.5 5,10.5 3.5,9.5 3.5,9.5 5,9 5,9 3))");
  assert(!positiveAreaOverlap(concave, lane).empty());

  const auto rotated = polygon("POLYGON((9 1,11 2,10 4,8 3,9 1))");
  assert(!positiveAreaOverlap(rotated, lane).empty());

  const auto tangent = polygon("POLYGON((10 1,12 1,12 3,10 3,10 1))");
  assert(positiveAreaOverlap(tangent, lane).empty());

  const auto outside = polygon("POLYGON((10.05 1,12 1,12 3,10.05 3,10.05 1))");
  assert(positiveAreaOverlap(outside, lane).empty());

  const auto tiny = polygon("POLYGON((9.9999999 1,12 1,12 3,9.9999999 3,9.9999999 1))");
  assert(positiveAreaOverlap(tiny, lane).empty());
  return 0;
}
