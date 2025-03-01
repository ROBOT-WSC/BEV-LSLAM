#pragma once

#include <ros/ros.h>

#include "imagecloud_msg/cloudparam.h"

#include <std_msgs/Header.h>
#include "std_msgs/Float64MultiArray.h"
#include <sensor_msgs/Imu.h>
#include "sensor_msgs/Image.h"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include<image_transport/image_transport.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

#include <cv_bridge/cv_bridge.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h> 
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ceres/ceres.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

 
#include <vector>
#include <string>
#include <map>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>

#include <boost/format.hpp>

using namespace std;
using namespace Eigen;
using namespace pcl;

#define R_EARTH 6371393.0 // m

int USE_IMU = 1, USE_GNSS = 0, USE_GROUND = 0, LoopClosureEnable = 1;
float init_x = 0, init_y = 0, init_z = 0, init_yaw = 0;

typedef pcl::PointXYZI PointType;

typedef pcl::PointXYZINormal PointType2;

double NormalizationAngle(double angle)
{
    if (angle > M_PI)
        angle -= 2 * M_PI;
    else if (angle < -M_PI)
        angle += 2 * M_PI;

    return angle;
}

double NormalizationRollPitchAngle(double angle)
{
    if (angle > M_PI / 2)
        angle -= M_PI;
    else if (angle < -M_PI / 2)
        angle += M_PI;

    return angle;
}

class Utility
{
  public:
    static Eigen::Vector3d R2ypr(const Eigen::Matrix3d &R)
    {
        Eigen::Vector3d n = R.col(0);
        Eigen::Vector3d o = R.col(1);
        Eigen::Vector3d a = R.col(2);

        Eigen::Vector3d ypr(3);
        double y = atan2(n(1), n(0));
        double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
        double r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
        ypr(0) = y;
        ypr(1) = p;
        ypr(2) = r;

        return ypr / M_PI * 180.0;
    }

    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 3> ypr2R(const Eigen::MatrixBase<Derived> &ypr)
    {
        typedef typename Derived::Scalar Scalar_t;

        Scalar_t y = ypr(0) / 180.0 * M_PI;
        Scalar_t p = ypr(1) / 180.0 * M_PI;
        Scalar_t r = ypr(2) / 180.0 * M_PI;

        Eigen::Matrix<Scalar_t, 3, 3> Rz;
        Rz << cos(y), -sin(y), 0,
            sin(y), cos(y), 0,
            0, 0, 1;

        Eigen::Matrix<Scalar_t, 3, 3> Ry;
        Ry << cos(p), 0., sin(p),
            0., 1., 0.,
            -sin(p), 0., cos(p);

        Eigen::Matrix<Scalar_t, 3, 3> Rx;
        Rx << 1., 0., 0.,
            0., cos(r), -sin(r),
            0., sin(r), cos(r);

        return Rz * Ry * Rx;
    }
    //预积分add
    template <typename Derived>
    static Eigen::Quaternion<typename Derived::Scalar> positify(const Eigen::QuaternionBase<Derived> &q)
    {
        //printf("a: %f %f %f %f", q.w(), q.x(), q.y(), q.z());
        //Eigen::Quaternion<typename Derived::Scalar> p(-q.w(), -q.x(), -q.y(), -q.z());
        //printf("b: %f %f %f %f", p.w(), p.x(), p.y(), p.z());
        //return q.template w() >= (typename Derived::Scalar)(0.0) ? q : Eigen::Quaternion<typename Derived::Scalar>(-q.w(), -q.x(), -q.y(), -q.z());
        return q;
    }
    
    template <typename Derived>
    static Eigen::Quaternion<typename Derived::Scalar> deltaQ(const Eigen::MatrixBase<Derived> &theta)
    {
        typedef typename Derived::Scalar Scalar_t;

        Eigen::Quaternion<Scalar_t> dq;
        Eigen::Matrix<Scalar_t, 3, 1> half_theta = theta;
        half_theta /= static_cast<Scalar_t>(2.0);
        dq.w() = static_cast<Scalar_t>(1.0);
        dq.x() = half_theta.x();
        dq.y() = half_theta.y();
        dq.z() = half_theta.z();
        return dq;
    }

    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 4, 4> Qleft(const Eigen::QuaternionBase<Derived> &q)
    {
        Eigen::Quaternion<typename Derived::Scalar> qq = positify(q);
        Eigen::Matrix<typename Derived::Scalar, 4, 4> ans;
        ans(0, 0) = qq.w(), ans.template block<1, 3>(0, 1) = -qq.vec().transpose();
        ans.template block<3, 1>(1, 0) = qq.vec(), ans.template block<3, 3>(1, 1) = qq.w() * Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() + skewSymmetric(qq.vec());
        return ans;
    }

    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 4, 4> Qright(const Eigen::QuaternionBase<Derived> &p)
    {
        Eigen::Quaternion<typename Derived::Scalar> pp = positify(p);
        Eigen::Matrix<typename Derived::Scalar, 4, 4> ans;
        ans(0, 0) = pp.w(), ans.template block<1, 3>(0, 1) = -pp.vec().transpose();
        ans.template block<3, 1>(1, 0) = pp.vec(), ans.template block<3, 3>(1, 1) = pp.w() * Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() - skewSymmetric(pp.vec());
        return ans;
    }

    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 3> skewSymmetric(const Eigen::MatrixBase<Derived> &q)
    {
        Eigen::Matrix<typename Derived::Scalar, 3, 3> ans;
        ans << typename Derived::Scalar(0), -q(2), q(1),
            q(2), typename Derived::Scalar(0), -q(0),
            -q(1), q(0), typename Derived::Scalar(0);
        return ans;
    }

};

class Mid_Filter
{
  public:
    double data_buf[500], filte_buf[500];
    int filter_count = 0, filter_size = 201;

    Mid_Filter(int size)
    {
        filter_size = size;
    }
    
    double MFilter(double data)
    {
        double tem = 0;

        data_buf[filter_count] = data;
        filter_count++;
        if (filter_count >= filter_size) filter_count = 0;

        for (int i = 0; i < filter_size; i++) filte_buf[i] = data_buf[i];
    
        for (int i = 0; i < filter_size - 1; i++)
        {
            for (int j = 0; j < filter_size - i - 1; j++)
            {
                if (filte_buf[j] > filte_buf[j + 1])
                {
                    tem = filte_buf[j]; 
                    filte_buf[j] = filte_buf[j + 1]; 
                    filte_buf[j + 1] = tem;
                }
            }
        }
        tem = filte_buf[(filter_size - 1) / 2];
        return tem;
    }

};

typedef struct
{
    int count = 0;
    
    double t = 0;
    double k = 0.002;
    double ax = 0, ay = 0, az = 0;
    double gx = 0, gy = 0, gz = 0;
    // Eigen::Vector3d ba = Eigen::Vector3d(0.23054, -0.22046, -0.14313);
    // Eigen::Vector3d bg = Eigen::Vector3d(0.00127, -0.00061, -0.00267);//小觅
    Eigen::Vector3d ba = Eigen::Vector3d(-0.004485, -0.021716, 0.025998);
    Eigen::Vector3d bg = Eigen::Vector3d(0.003796, 0.001246, 0.002479);//xsens

    Eigen::Quaterniond Q_init = Eigen::Quaterniond::Identity();
    Eigen::Matrix3d R_init = Eigen::Matrix3d::Identity();
    Eigen::Quaterniond q_eigen = Eigen::Quaterniond::Identity();
    Eigen::Matrix3d Rwi = Eigen::Matrix3d::Identity();
    Eigen::Vector3d Pwi = Eigen::Vector3d::Zero();
    Eigen::Vector3d Vwi = Eigen::Vector3d::Zero();
    Eigen::Quaterniond Qwi = Eigen::Quaterniond::Identity();

    double roll = 0, pitch = 0, yaw = 0;
    double roll_acc = 0, pitch_acc = 0, yaw_acc = 0;
    double roll_init = -0.013, pitch_init = 0.0096, yaw_init = 0;
} imu_s;

struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                   (double, time, time))

typedef PointXYZIRPYT  PointTypePose;


class ParamServer
{
public:

    ros::NodeHandle nh;

    std::string robot_id;

    // 话题
    string pointCloudTopic; // points_raw 原始点云数据
    string imuTopic;        // imu_raw 对应park数据集，imu_correct对应outdoor数据集，都是原始imu数据，不同的坐标系表示


    // 激光传感器参数
    int N_SCANS;             // 扫描线数，例如16、64
    int Horizon_SCAN;       // 扫描一周计数，例如每隔0.2°扫描一次，一周360°可以扫描1800次
    double MINIMUM_RANGE;    // 最小范围
    double MAXMUM_RANGE;    // 最大范围
    double lineResolution = 0.2;
    double planeResolution = 0.4;
    int intensity_enable = 0;
    // BEV参数
    double image_resolution = 0.1;
    int min_distance = 50;
    int image_length = 60;
    int image_height = 30;

    int skipFrameNum = 1;
    int USE_IMU = 1, USE_GNSS = 0, USE_GROUND = 0;
    int odom_pub = 0;
    float init_x = 0, init_y = 0, init_z = 0, init_yaw = 0;

    int slipwide = 10;
    int BA_freq = 5;
    int local_BA_type = 0;
    int local_BA_enable = 1;

    //Mapping 参数
    int LoopClosureEnable = 1; 
    int enable_DoWB = 1;
    float keyframeAddingDistance = 0.5; // 0.5m
    float keyframeAddingAngle = 0.3; // 57.3*0.2 degree 
    float surroundingKeyframeSearchRadius = 15; // m
    float globalMapDensity = 0.4;
    float down_simple_vgicp = 1.2;
    float MIN_LOOP_BOW_TH = 0.015;
    std::string saveDirectory = "/home/robot220/code_ws/catkin_ws_robot_navigation/database";
    std::string voc_path = "/home/wsc/bev_lo/src/orb_lio/config/orbvoc.dbow3";
    int map_update = 1, RMODE = 1;
    int Common_view_num = 20;
    int select_DoWB_num = 10;


    ParamServer()
    {
        nh.param<std::string>("orb_lio/pointCloudTopic", pointCloudTopic, "points_raw");
        nh.param<std::string>("orb_lio/imuTopic", imuTopic, "imu_correct");
    
        nh.param<int>("orb_lio/N_SCAN", N_SCANS, 16);
        nh.param<int>("orb_lio/Horizon_SCAN", Horizon_SCAN, 1800);
        nh.param<double>("orb_lio/MINIMUM_RANGE", MINIMUM_RANGE, 0.5);
        nh.param<double>("orb_lio/MAXMUM_RANGE", MAXMUM_RANGE, 90);
        nh.param<double>("orb_lio/image_resolution", image_resolution, 0.1);
        nh.param<int>("orb_lio/min_distance", min_distance, 50);
        nh.param<int>("orb_lio/image_length", image_length, 60);
        nh.param<int>("orb_lio/image_height", image_height, 30);
        nh.param<int>("orb_lio/intensity", intensity_enable, 16);

        nh.param<int>("orb_lio/mapping_skip_frame", skipFrameNum, 1);
        nh.param<int>("orb_lio/USE_IMU", USE_IMU, 0);
        //odom 参数
        nh.param<int>("orb_lio/odom_pub", odom_pub, 0);

        nh.param<int>("orb_lio/BA_freq", BA_freq, 5);
        nh.param<int>("orb_lio/windows_wideth", slipwide, 10);

        nh.param<int>("orb_lio/local_BA_type", local_BA_type, 0);
        nh.param<int>("orb_lio/local_BA_enable", local_BA_enable, 1);

        //Mapping 参数
        nh.param<double>("orb_mapping/lineResolution", lineResolution, 0.1);
        nh.param<double>("orb_mapping/planeResolution", planeResolution, 0.1);
        nh.param<int>("orb_mapping/LoopClosureEnable", LoopClosureEnable, 1);
        nh.param<float>("orb_mapping/keyframeAddingDistance", keyframeAddingDistance, 0.5);
		nh.param<float>("orb_mapping/keyframeAddingAngle", keyframeAddingAngle, 0.2);
		nh.param<float>("orb_mapping/surroundingKeyframeSearchRadius", surroundingKeyframeSearchRadius, 15);
		nh.param<float>("orb_mapping/globalMapDensity", globalMapDensity, 0.4);
        nh.param<string>("orb_mapping/saveDirectory", saveDirectory, "/home/robot220/code_ws/catkin_ws_robot_navigation/database");
		nh.param<int>("orb_mapping/RMODE", RMODE, 1); // default: mapping mode
		nh.param<int>("orb_mapping/map_update", map_update, 1); // default: update map
        nh.param<float>("orb_mapping/init_x", init_x, 0);
        nh.param<float>("orb_mapping/init_y", init_y, 0);
        nh.param<float>("orb_mapping/init_z", init_z, 0);
        nh.param<float>("orb_mapping/init_yaw", init_yaw, 0);
        nh.param<float>("orb_mapping/down_simple_vgicp", down_simple_vgicp, 1.2);
        nh.param<string>("orb_mapping/voc_path", voc_path, "/home/wsc/bev_lo/src/orb_lio/config/orbvoc.dbow3");
        nh.param<int>("orb_mapping/enable_DoWB", enable_DoWB, 1); // default: update map
        nh.param<float>("orb_mapping/MIN_LOOP_BOW_TH", MIN_LOOP_BOW_TH, 0.015);
        nh.param<int>("orb_mapping/select_DoWB_num", select_DoWB_num, 10); // default: update map
        nh.param<int>("orb_mapping/Common_view_num", Common_view_num, 20); // default: update map

        usleep(100);
    }
};

typedef struct
{
    cv::Mat BEV_height,BEV_intensity;//高度图和强度图
    std::vector<cv::KeyPoint> keypoints_height, keypoints_intensity;
    cv::Mat description_height, description_intensity; 
    pcl::PointCloud<PointType>::Ptr laserCloudheight;
    pcl::PointCloud<PointType>::Ptr laserCloudintensity; 
    int height_point_num = 0;
    int intensity_point_num = 0;
    Eigen::Quaterniond q_w_curr=Eigen::Quaterniond::Identity();
    Eigen::Vector3d t_w_curr=Eigen::Vector3d::Zero();

} BEV_s;

typedef struct
{
    cv::Mat BEV_image;//高度图和强度图
    cv::Mat BEV_RGB;
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat description; 
    std::vector<cv::KeyPoint> keypoints_dowb;
    cv::Mat description_dowb; 
} BEV_mapping;
