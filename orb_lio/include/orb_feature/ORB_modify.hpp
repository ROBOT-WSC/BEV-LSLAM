#pragma once

#include <iostream>
#include <string>

#include <ros/ros.h>

#include "ORBextractor.hpp"

#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;
using namespace ORB_SLAM2;

class ORB_modify
{
public:
    ORB_modify(const ros::NodeHandle &nh);

    void ORB_feature(cv::Mat &im);
    // void UndistortKeyPoints();

public:
    //ORB
    ORBextractor* mpORBextractor;

    cv::Mat mImGray;

    int N;

    // Vector of keypoints (original for visualization) and undistorted (actually used by the system).
    std::vector<cv::KeyPoint> mvKeys;

    // ORB descriptor, each row associated to a keypoint.
    cv::Mat mDescriptors;
};
