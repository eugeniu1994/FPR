#define PCL_NO_PRECOMPILE

#include <Eigen/Eigen>
#include <Eigen/Core>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/PointCloud2.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosbag/message_instance.h>

// opencv
// #include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <iostream>
#include <math.h>
#include <csignal>
#include <unistd.h>
#include <ros/ros.h>
#include <chrono>

#include <tbb/parallel_for.h>
#include <tsl/robin_map.h>

struct EIGEN_ALIGN16 PointType
{
    PCL_ADD_POINT4D;
    float intensity;
    double timestamp;
    uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(PointType,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(double, timestamp, timestamp)(uint16_t, ring, ring))

typedef pcl::PointCloud<PointType> PointCloudXYZI;

using Voxel = Eigen::Vector3i;
struct VoxelHash
{
    size_t operator()(const Voxel &voxel) const
    {
        const uint32_t *vec = reinterpret_cast<const uint32_t *>(voxel.data());
        return ((1 << 20) - 1) & (vec[0] * 73856093 ^ vec[1] * 19349663 ^ vec[2] * 83492791);
    }
};


struct VoxelAccum {
    Eigen::Vector3d weighted_sum = Eigen::Vector3d::Zero();
    double weight_sum = 0.0001; //to avoid zero division
    float max_intensity = -1.0f;
    PointType max_point; 
};


const int kernel_row = 1, kernel_col = 5;


// config of the hesai
const int N_SCAN = 32;         // number of vertical channels
const float ang_res_x = 0.18f; // horizontal angular resolution in degrees
const int Horizon_SCAN = 2000;  // 360° / ang_res_x° resolution

//config of the hdl-64
// const int N_SCAN = 64;
// const float ang_res_x = 0.2; // horizontal angular resolution in degrees
// const int Horizon_SCAN = 1800; 



//const float ang_res_y = 1.;
// const float ang_bottom = 15.0; // 360° x 31° Field of View put mid 31/2
// ang_bottom is the vertical angle corresponding to the bottom-most laser ring (lowest vertical FOV limit)
// ang_bottom = last ang_res_y  = vertical_fov / (N_SCAN - 1)​
