#pragma once

#include <pcl/point_types.h>

#include <cstdint>

namespace rla::core {

struct PointXYZIRT {
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY
    uint16_t ring;  //!< Number of ring of the point. A ring could reference a
                    //!< hight level or number of scan.
    uint32_t t;  //!< Time with respect to pointcloud start (micro s). Max range
                 //!< is 71m.
} EIGEN_ALIGN16;

}  // namespace rla::core

POINT_CLOUD_REGISTER_POINT_STRUCT(
    rla::core::PointXYZIRT,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity,
                                            intensity)(uint16_t, ring,
                                                       ring)(uint32_t, t, t))
