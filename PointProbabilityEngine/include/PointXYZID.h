#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// Define a custom PCL point type with XYZ coordinates and an ID field
struct PointXYZID
{
    PCL_ADD_POINT4D; // This adds the XYZ coordinates + padding
    int id;          // Custom ID field

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // Make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // Enforce SSE padding for correct memory alignment

// Register the point type with PCL
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZID,
                                  (float, x, x)(float, y, y)(float, z, z)(int, id, id))
