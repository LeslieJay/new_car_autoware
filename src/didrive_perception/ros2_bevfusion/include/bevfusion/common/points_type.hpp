#ifndef POINTS_TYPE_H
#define POINTS_TYPE_H

#include <Eigen/Core>
#include <Eigen/Dense>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

struct PointXYZIR
{
    PCL_ADD_POINT4D
    // PCL_ADD_INTENSITY
    float intensity;
    uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIR,
(float,x,x)
(float,y,y)
(float,z,z)
(float, intensity, intensity)
(uint16_t,ring,ring)
)

typedef PointXYZIR PointType;

#endif
