#include "orb_lio/utility.h"
#include "orb_lio/tic_toc.h"
#include "orb_feature/gms_matcher.hpp"
#include "orb_feature/ORB_modify.hpp"
#include "lidarFactor.hpp"

Eigen::Quaterniond q_w_curr(1, 0, 0, 0);
Eigen::Vector3d t_w_curr(0, 0, 0);
double para_q[4] = {0, 0, 0, 1};
double para_t[3] = {0, 0, 0};
Eigen::Map<Eigen::Quaterniond> q_last_curr(para_q);
Eigen::Map<Eigen::Vector3d> t_last_curr(para_t);

Eigen::Vector3d t_last_curr_histoary(0, 0, 0);

Eigen::Quaterniond q_last_curr_histoary(1, 0, 0, 0);


class ScanRegistration: public ParamServer
{
    private:
    std::mutex mBuf;

    image_transport::Publisher pubheightBEV;
    image_transport::Publisher pubintensityBEV;
    image_transport::Publisher pubheightBEV_match;
    image_transport::Publisher pubintensityBEV_match;
    
    ros::Publisher pubLaserCloudheight;
    ros::Publisher pubLaserCloudintensity;

    ros::Publisher pubLaserCloudheight_point;
    ros::Publisher pubLaserCloudintensity_point;

    ros::Publisher pubLaserOdometry;
    ros::Publisher pubLaserPath;

    ros::Subscriber subLaserCloud;
    public:

    const int groundScanInd = 7;
    const double scanPeriod = 0.1;
    const int systemDelay = 10;
    bool systemInited = false;
    int systemInitCount = 0;
    double laderH = 0.56;
    float Ground_scan_range[16] = {2.66, 3.04, 3.56, 4.30, 5.44, 7.41, 11.63, 27.12}; // h: 0.56m ; -15 至 -1 度； (pitch: 0.5deg)
    int detect_range = 4;
    cv::Mat BEV_detect;


    //laserodometry
    queue<sensor_msgs::ImageConstPtr> heightBEVBuf;
    queue<sensor_msgs::ImageConstPtr> intensityBEVBuf;
    queue<sensor_msgs::PointCloud2ConstPtr> heightPointsBuf;
    queue<sensor_msgs::PointCloud2ConstPtr> intensityPointsBuf;
    queue<imagecloud_msg::cloudparam> fullPointsBuf;

    Eigen::Vector3d t_rl;
    Eigen::Matrix3d R_rl;
    Eigen::Matrix4d T_rl;

    double timeheightBEV = 0;
    double timeintensityBEV = 0;
    double timeLaserCloudheight = 0;
    double timeLaserCloudintensity = 0;
    double timeLaserCloudfull = 0;
    double prevTime = 0, curTime = 0;

    BEV_s BEV_now;
    BEV_s BEV_last;

    int laserCloudheightNum = 0;
    int laserCloudintensityNum = 0;
    int submapflag = 0;

    int iter_num = 0;

    int skipFrameNum = 1;
    const float SCAN_PERIOD = 0.1;
    const float DISTANCE_SQ_THRESHOLD = 9.0;
    const float NEARBY_SCAN = 2.5;
    double deg2rad = M_PI / 180.0, rad2deg = 180.0 / M_PI;
    int BEV_rows = image_height/image_resolution;//行数
    int BEV_cols = image_length/image_resolution;//列数
    boost::shared_ptr<ORB_modify> orb;
    // cv::Ptr<cv::ORB> orb = cv::ORB::create(1000, 1.2f, 8, 15, 0, 3, cv::ORB::HARRIS_SCORE, 15, 20);

    deque<BEV_s> surroundingBEV;
    deque<pcl::PointCloud<PointType>> surrounding_heightpoint;
    deque<pcl::PointCloud<PointType>> surrounding_intensitypoint;
    deque<std::vector<cv::DMatch>> keyframe_matches;
    deque<std::vector<cv::DMatch>> keyframe_matches2;

    int surround_keyframes = 0;
    int keyframes_opt = 1;
    float keyframeAddingDistance = 0.3; // 0.5m
    float keyframeAddingAngle = 0.2; // 57.3*0.2 degree 

    std::vector<pcl::PointCloud<PointType>> last_cloud_select;


    nav_msgs::Path laserPath;

    ScanRegistration()
    {  
        image_transport::ImageTransport in(nh);
        printf("--scanRegistration: scan line number %d \n", N_SCANS);
        printf("--scanRegistration: minimum_range: %.3f, maxmum_range: %.3f \n", MINIMUM_RANGE, MAXMUM_RANGE);
        printf("--scanRegistration: line resolution: %f, plane resolution: %f \n", lineResolution, planeResolution);
        printf("--LaserOdometry: Mapping %d Hz \n", 10 / skipFrameNum);
        printf("--LaserOdometry: USE_IMU %d \n", USE_IMU);
        printf("--LaserOdometry: init_x %f \n", init_x);
        printf("--LaserOdometry: init_y %f \n", init_y);
        printf("--LaserOdometry: init_z %f \n", init_z);
        printf("--LaserOdometry: init_yaw %f \n", init_yaw);

        orb.reset(new ORB_modify(nh));

        t_rl = Eigen::Vector3d(0.68, 0, 0.34);
        R_rl = Utility::ypr2R(Eigen::Vector3d(0.0, -0.0, 0.0));
        T_rl = Eigen::Matrix4d::Identity();
		T_rl.block<3, 3>(0, 0) = R_rl;
        T_rl.block<3, 1>(0, 3) = t_rl;

        BEV_now.laserCloudheight.reset(new pcl::PointCloud<PointType>());
        BEV_now.laserCloudintensity.reset(new pcl::PointCloud<PointType>());
        BEV_last.laserCloudheight.reset(new pcl::PointCloud<PointType>());
        BEV_last.laserCloudintensity.reset(new pcl::PointCloud<PointType>());

        BEV_detect = cv::Mat::zeros(2*detect_range-1, 2*detect_range-1, CV_8UC1);
        for(int r = 1-detect_range; r < detect_range; r++)
        {
            for(int c = 1-detect_range; c < detect_range; c++)
            {
                int temp_x = r + detect_range-1;
                int temp_y = c + detect_range-1;
                if(abs(r) == detect_range-1 || abs(c) == detect_range-1)
                {
                    if(abs(r-c)<2)
                    {
                        continue;
                    }
                    BEV_detect.at<uchar>(temp_x, temp_y) = 255;
                }
                else if(abs(r-c) == 0 && abs(r) == detect_range - 1)
                {
                    BEV_detect.at<uchar>(temp_x, temp_y) = 255;
                }
            }
        }

        if (N_SCANS != 16 && N_SCANS != 32 && N_SCANS != 64)
        {
            printf("only support velodyne with 16, 32 or 64 scan line!");
        }
        //bev图像的行和列数
        BEV_rows = image_height/image_resolution;
        BEV_cols = image_length/image_resolution;

        BEV_now.BEV_height = cv::Mat::zeros(BEV_rows, BEV_cols, CV_8UC1);
        BEV_now.BEV_intensity = cv::Mat::zeros(BEV_rows, BEV_cols, CV_8UC1);
        BEV_last.BEV_height = cv::Mat::zeros(BEV_rows, BEV_cols, CV_8UC1);
        BEV_last.BEV_intensity = cv::Mat::zeros(BEV_rows, BEV_cols, CV_8UC1);

        pubheightBEV = in.advertise("/laserodometry/heightBEV",10);
        pubintensityBEV = in.advertise("/laserodometry/intensityBEV",10);

        pubheightBEV_match = in.advertise("/laserodometry/heightBEV_match",10);
        pubintensityBEV_match = in.advertise("/laserodometry/intensityBEV_match",10);

        pubLaserCloudheight = nh.advertise<sensor_msgs::PointCloud2>("/laserodometry/laser_cloud_height", 10);
        pubLaserCloudintensity = nh.advertise<sensor_msgs::PointCloud2>("/laserodometry/laser_cloud_intensity", 10);

        pubLaserCloudheight_point = nh.advertise<sensor_msgs::PointCloud2>("/laserodometry/pubLaserCloudheight_point", 10);
        pubLaserCloudintensity_point = nh.advertise<sensor_msgs::PointCloud2>("/laserodometry/pubLaserCloudintensity_point", 10);

        pubLaserOdometry = nh.advertise<nav_msgs::Odometry>("/laserodometry/laser_odom_to_init", 10);
        pubLaserPath = nh.advertise<nav_msgs::Path>("/laserodometry/laser_odom_path", 10);

        subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(pointCloudTopic, 100, &ScanRegistration::laserCloudHandler, this);//  /points_raw /kitti/velo/pointcloud

}

void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    mBuf.lock();
    if (!systemInited)
    {
        systemInitCount++;
        if (systemInitCount > systemDelay)
        {
            systemInited = true;
        }
        else
        {
            mBuf.unlock();
            return;
        }
    }
    // 读取雷达点云
    TicToc t_wo;
    pcl::PointCloud<PointType> laserCloudIn;
    pcl::fromROSMsg(*laserCloudMsg, laserCloudIn);
    // 去除无效点
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn, indices);
    removeClosedPointCloud(laserCloudIn, laserCloudIn, MINIMUM_RANGE, MAXMUM_RANGE);
    timeheightBEV = laserCloudMsg->header.stamp.toSec();
    timeintensityBEV = laserCloudMsg->header.stamp.toSec();
    timeLaserCloudheight = laserCloudMsg->header.stamp.toSec();
    timeLaserCloudintensity = laserCloudMsg->header.stamp.toSec();
    timeLaserCloudfull = laserCloudMsg->header.stamp.toSec();
    int cloudSize = laserCloudIn.points.size();
    //读取最值
    float min_z = 10000;
    float max_z = -10000;
    int min_intensity = 10000;
    int max_intensity = -10000;
    for(int i = 0; i < cloudSize; i++)
    {
        laserCloudIn.points[i].intensity = laserCloudIn.points[i].intensity * 255;
        PointType point_temp1 = laserCloudIn.points[i];
         if(point_temp1.x<(-image_height/2+0.5)||point_temp1.x>(image_height/2-0.5)||point_temp1.y<(-image_length/2+0.5)||point_temp1.y>(image_length/2-0.5))//越界判断
        {
            continue;
        }
        if(point_temp1.z > max_z)
        {
            max_z = point_temp1.z;
        }
        if(point_temp1.z < min_z)
        {
            min_z = point_temp1.z;
        }
        if(point_temp1.intensity > max_intensity)
        {
            max_intensity = point_temp1.intensity;
        }
        if(point_temp1.intensity < min_intensity)
        {
            min_intensity = point_temp1.intensity;
        }
    }
    // 初始化俩张图像
    cv::Mat BEV_hight,BEV_intensity;//高度图和强度图
    BEV_hight = cv::Mat::zeros(BEV_rows, BEV_cols, CV_8UC1);
    BEV_intensity = cv::Mat::zeros(BEV_rows, BEV_cols, CV_8UC1);
    // 初始化特征点
    pcl::PointCloud<PointType>::Ptr height_cloud(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr intensity_cloud(new pcl::PointCloud<PointType>);
    height_cloud->points.resize(BEV_rows*BEV_cols);
    intensity_cloud->points.resize(BEV_rows*BEV_cols);

    int BEV_1 = (image_height/2-0)/image_resolution;
    int BEV_2 = (image_length/2+0)/image_resolution;

    std::vector<pcl::PointCloud<PointType>> cloud_select;
    cloud_select.resize(BEV_rows*BEV_cols);
    //填充图像
    for(int i = 0; i < cloudSize; i++)
    {
        PointType point_temp = laserCloudIn.points[i];
        if(point_temp.x<(-image_height/2+0.5)||point_temp.x>(image_height/2-0.5)||point_temp.y<(-image_length/2+0.5)||point_temp.y>(image_length/2-0.5))//越界判断
        {
            continue;
        }
        int BEV_X = (image_height/2-point_temp.x)/image_resolution;
        int BEV_Y = (image_length/2-point_temp.y)/image_resolution;

        int norm_z = (255-min_distance)*((point_temp.z - min_z)/(max_z - min_z))+min_distance;
        int norm_intensity = (255-min_distance)*((point_temp.intensity - min_intensity)/(max_intensity - min_intensity))+min_distance;
        //判断高度BEV
        if(norm_z > BEV_hight.at<uchar>(BEV_X, BEV_Y))
        {
            BEV_hight.at<uchar>(BEV_X, BEV_Y) = norm_z;
            height_cloud->points[BEV_X*BEV_cols + BEV_Y] = point_temp;
        }
        //判断强度BEV
        if(norm_intensity > BEV_intensity.at<uchar>(BEV_X, BEV_Y))
        {
            BEV_intensity.at<uchar>(BEV_X, BEV_Y) = norm_intensity;
            intensity_cloud->points[BEV_X*BEV_cols + BEV_Y] = point_temp;
        }
        
        cloud_select[BEV_X*BEV_cols + BEV_Y].push_back(point_temp);
    }
    cv::Mat BEV_hight_raw = BEV_hight.clone();
    cv::Mat BEV_intensity_raw = BEV_intensity.clone();
    pixel_detect(height_cloud, &BEV_hight);
    cv::Mat BEV_hight_improve = BEV_hight.clone();
    pixel_detect_intensity(intensity_cloud, &BEV_intensity);
    cv::Mat BEV_intensity_improve = BEV_intensity.clone();
    // pixel_detect(intensity_cloud, &BEV_intensity);
    //publish height BEV
    cv::equalizeHist(BEV_hight, BEV_hight);
    cv::GaussianBlur(BEV_hight, BEV_hight, cv::Size(5, 5), 0, 0, cv::BORDER_DEFAULT);
    cv::equalizeHist(BEV_intensity, BEV_intensity);
    cv::GaussianBlur(BEV_intensity, BEV_intensity, cv::Size(3, 3), 0, 0, cv::BORDER_DEFAULT);
    // ROS_ERROR("point image ---:speed time:%f",t_wo.toc());

    BEV_now.laserCloudheight->clear();
    BEV_now.laserCloudheight = height_cloud;
    laserCloudheightNum = BEV_now.laserCloudheight->size();

    BEV_now.laserCloudintensity->clear();
    BEV_now.laserCloudintensity = intensity_cloud;
    BEV_now.BEV_height = BEV_hight;
    BEV_now.BEV_intensity = BEV_intensity;

    orb->ORB_feature(BEV_now.BEV_height);
    BEV_now.keypoints_height = orb->mvKeys;
    BEV_now.description_height = orb->mDescriptors;
    orb->ORB_feature(BEV_now.BEV_intensity);
    BEV_now.keypoints_intensity = orb->mvKeys;
    BEV_now.description_intensity = orb->mDescriptors;    
    
    BEV_now.height_point_num = BEV_now.keypoints_height.size();
    BEV_now.intensity_point_num = BEV_now.keypoints_intensity.size();
    int ba_flag = 1;
 // optimization
    if (BEV_last.laserCloudheight->points.size() != 0 && BEV_last.laserCloudintensity->points.size() != 0)
    {
        if(submapflag == 0)
        {
            surroundingBEV.push_back(BEV_last);
            surrounding_heightpoint.push_back(*(BEV_last.laserCloudheight));
            surrounding_intensitypoint.push_back(*(BEV_last.laserCloudintensity));
            surround_keyframes++;
        }
        submapflag++;

        vector<cv::DMatch> matches_height,matches_intensity;
        cv::BFMatcher matcher(cv::NORM_HAMMING);

        matcher.match(BEV_now.description_height,BEV_last.description_height,matches_height);
        std::vector<bool> height_vbInliers;
        gms_matcher gms_height(BEV_now.keypoints_height, BEV_now.BEV_height.size(), BEV_last.keypoints_height, BEV_last.BEV_height.size(), matches_height);
        int height_lnliers = gms_height.GetInlierMask(height_vbInliers, false, true);

        matcher.match(BEV_now.description_intensity,BEV_last.description_intensity,matches_intensity);
        std::vector<bool> intensity_vbInliers;
        gms_matcher gms_intensity(BEV_now.keypoints_intensity, BEV_now.BEV_intensity.size(), BEV_last.keypoints_intensity, BEV_last.BEV_intensity.size(), matches_intensity);
        int intensity_lnliers = gms_intensity.GetInlierMask(intensity_vbInliers, false, true);
        
        double max_dist = -100000;
        double min_dist = 100000;
        double max_dist2 = -100000;
        double min_dist2 = 100000;
        for(int i = 0; i < matches_height.size(); i++)
        {
            double dist = abs(matches_height[i].distance);
            
            if(dist < min_dist) min_dist = dist ;
            if(dist > max_dist) max_dist = dist ;
        }
        for(int i = 0; i < matches_intensity.size(); i++)
        {
            double dist = abs(matches_intensity[i].distance);
            
            if(dist < min_dist2) min_dist2 = dist ;
            if(dist > max_dist2) max_dist2 = dist ;
        }
        std::vector<cv::DMatch> good_matches_height;
        for(int i = 0; i < height_vbInliers.size(); i++)
        {
            if(height_vbInliers[i] == true && matches_height[i].distance < (2*min_dist>40? 2*min_dist : 40))
            {
                good_matches_height.push_back(matches_height[i]);
            }
        }
        std::vector<cv::DMatch> good_matches_intensity;
        for(int i = 0; i < intensity_vbInliers.size(); i++)
        {
            if(intensity_vbInliers[i] == true && matches_intensity[i].distance < (2*min_dist2>80? 2*min_dist2 : 80))
            {
                good_matches_intensity.push_back(matches_intensity[i]);
            }
        }
        ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
        ceres::LocalParameterization *q_parameterization = new ceres::EigenQuaternionParameterization();
        ceres::Problem::Options problem_options;
        ceres::Problem problem(problem_options);
        problem.AddParameterBlock(para_q, 4, q_parameterization);
        problem.AddParameterBlock(para_t, 3);
        if(odom_pub == 1)
        {
            cv::Mat img_matches_height;
            cv::Mat img_matches_intensity;
            cv::drawMatches(BEV_now.BEV_height, BEV_now.keypoints_height, BEV_last.BEV_height, BEV_last.keypoints_height, good_matches_height, img_matches_height, cv::Scalar(0,255,0), cv::Scalar(0,255,0), vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
            cv::drawMatches(BEV_now.BEV_intensity, BEV_now.keypoints_intensity, BEV_last.BEV_intensity, BEV_last.keypoints_intensity, good_matches_intensity, img_matches_intensity, cv::Scalar(0,255,0), cv::Scalar(0,255,0), vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);


            cv::Mat outimg1, outimg2;
            cv::drawKeypoints(BEV_hight_improve,BEV_now.keypoints_height,outimg1,cv::Scalar(0,255,0),cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
            cv::drawKeypoints(BEV_intensity_improve,BEV_now.keypoints_intensity,outimg2,cv::Scalar(0,255,0),cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
            sensor_msgs::ImagePtr hight_msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", img_matches_height).toImageMsg();
            hight_msg->header.stamp = ros::Time().fromSec(timeLaserCloudheight);
            hight_msg->header.frame_id = "camera_init";
            pubheightBEV_match.publish(hight_msg);
            boost::format fmt_image("%s%d%s");
            int iter_num2 = iter_num;
            // cv::imwrite((fmt_image % "/home/wsc/bev_lo/src/orb_lio/img/height_bev/BEV_height_" %iter_num2%".png").str(), BEV_now.BEV_height);
            // cv::imwrite((fmt_image % "/home/wsc/bev_lo/src/orb_lio/img/intensity_bev/BEV_intensity_" %iter_num2%".png").str(), BEV_now.BEV_intensity);
            // cv::imwrite((fmt_image % "/home/wsc/bev_lo/src/orb_lio/img/height1/BEV_height_" %iter_num2%".png").str(), outimg1);
            // cv::imwrite((fmt_image % "/home/wsc/bev_lo/src/orb_lio/img/height2/BEV_height_" %iter_num2%".png").str(), img_matches_height);
            // cv::imwrite((fmt_image % "/home/wsc/bev_lo/src/orb_lio/img/intensity1/BEV_intensity_" %iter_num2%".png").str(), outimg2);
            // cv::imwrite((fmt_image % "/home/wsc/bev_lo/src/orb_lio/img/intensity2/BEV_intensity_" %iter_num2%".png").str(), img_matches_intensity);
            // cv::imwrite((fmt_image % "/home/wsc/bev_lo/src/orb_lio/img/height_raw/BEV_height_" %iter_num2%".png").str(), BEV_hight_raw);
            // cv::imwrite((fmt_image % "/home/wsc/bev_lo/src/orb_lio/img/height_improve/BEV_height_" %iter_num2%".png").str(), BEV_hight_improve);
            // cv::imwrite((fmt_image % "/home/wsc/bev_lo/src/orb_lio/img/intensity_raw/BEV_intensity_" %iter_num2%".png").str(), BEV_intensity_raw);
            // cv::imwrite((fmt_image % "/home/wsc/bev_lo/src/orb_lio/img/intensity_improve/BEV_intensity_" %iter_num2%".png").str(), BEV_intensity_improve);
            
            // cv::Mat img_matches_intensity;
            // cv::drawMatches(BEV_now.BEV_intensity, BEV_now.keypoints_intensity, BEV_last.BEV_intensity, BEV_last.keypoints_intensity, good_matches_intensity, img_matches_intensity, cv::Scalar::all(-1), cv::Scalar::all(-1), vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
            sensor_msgs::ImagePtr intensity_msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", img_matches_intensity).toImageMsg();
            intensity_msg->header.stamp = ros::Time().fromSec(timeLaserCloudheight);;
            intensity_msg->header.frame_id = "camera_init";
            pubintensityBEV_match.publish(intensity_msg);
        }


        pcl::PointCloud<PointType>::Ptr height_cloud_points(new pcl::PointCloud<PointType>);
        pcl::PointCloud<PointType>::Ptr intensity_cloud_points(new pcl::PointCloud<PointType>);

        for(int i = 0; i < good_matches_height.size(); i++)
        {
            PointType point_now;
            PointType point_last;
            int now_index = good_matches_height[i].queryIdx;
            int last_index = good_matches_height[i].trainIdx;
            int point_index_now = BEV_now.keypoints_height[now_index].pt.y*BEV_cols + BEV_now.keypoints_height[now_index].pt.x;
            int point_index_last = BEV_last.keypoints_height[last_index].pt.y*BEV_cols + BEV_last.keypoints_height[last_index].pt.x;
            point_now = BEV_now.laserCloudheight->points[point_index_now];
            point_last = BEV_last.laserCloudheight->points[point_index_last];
            if(point_now.x == 0 && point_now.y == 0 && point_now.z == 0)
            {
                continue;
            }
            if(point_last.x == 0 && point_last.y == 0 && point_last.z == 0)
            {
                continue;
            }
            pcl::PointCloud<PointType> temp_cloud;
            height_cloud_points->push_back(point_now);
            Eigen::Vector3d curr_point(point_now.x, point_now.y, point_now.z);
            Eigen::Vector3d last_point(point_last.x, point_last.y, point_last.z);
            temp_cloud = last_cloud_select[point_index_last];
            if(temp_cloud.size()>=5)
            {
                double min_dis = 9999; 
                for(int i = 0; i < temp_cloud.size() - 1; i++)
                {
                    for(int j = 0; j < temp_cloud.size() - 1 - i; j++)
                    {
                        double dis_1 = sqrt((temp_cloud.points[j].x - point_last.x)*(temp_cloud.points[j].x - point_last.x) + (temp_cloud.points[j].y - point_last.y)*(temp_cloud.points[j].y - point_last.y) + (temp_cloud.points[j].z - point_last.z)*(temp_cloud.points[j].z - point_last.z));
                        double dis_2 = sqrt((temp_cloud.points[j+1].x - point_last.x)*(temp_cloud.points[j+1].x - point_last.x) + (temp_cloud.points[j+1].y - point_last.y)*(temp_cloud.points[j+1].y - point_last.y) + (temp_cloud.points[j+1].z - point_last.z)*(temp_cloud.points[j+1].z - point_last.z));
                        PointType cloud_temp;
                        if(dis_1 > dis_2)
                        {
                            cloud_temp = temp_cloud.points[j+1];
                            temp_cloud.points[j+1] = temp_cloud.points[j];
                            temp_cloud.points[j] = cloud_temp; 
                        }
                    }
                }
                Eigen::Vector3d center(0, 0, 0);
                std::vector<Eigen::Vector3d> nearCorners;
                for (int j = 0; j < 5; j++)
                {
                    Eigen::Vector3d tmp(temp_cloud[j].x, temp_cloud[j].y, temp_cloud[j].z);
                    center = center + tmp;
                    nearCorners.push_back(tmp);
                }
                center = center / 5.0;
                Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();
                for (int j = 0; j < 5; j++)
                {
                    Eigen::Matrix<double, 3, 1> tmpZeroMean = nearCorners[j] - center;
                    covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
                }
                Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);
                Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);
                if(saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1])
                {
                    Eigen::Vector3d point_on_line = center;
                    Eigen::Vector3d point_a, point_b;
                    point_a = 0.1 * unit_direction + point_on_line;
                    point_b = -0.1 * unit_direction + point_on_line;
                    ceres::CostFunction *cost_function = LidarEdgeFactor::Create(curr_point, point_a, point_b, 100);
                    problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);                            
                }
                else
                {
                    Eigen::Vector3d factor(100, 100, 0.01);
                    ceres::CostFunction *cost_function = LidarICPFactor::Create(curr_point, last_point, factor);
                    problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);
                }
            }
            else
            {
                    Eigen::Vector3d factor(100, 100, 0.01);
                    ceres::CostFunction *cost_function = LidarICPFactor::Create(curr_point, last_point, factor);
                    problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);
            }

            // Eigen::Vector2d last_point_index(BEV_last.keypoints_height[last_index].pt.y, BEV_last.keypoints_height[last_index].pt.x);
            // Eigen::Vector3d image_params(image_height,image_length,image_resolution);
            // ceres::CostFunction *cost_function_project = Reprojection_Factor::Create(curr_point,last_point_index, image_params);
            // problem.AddResidualBlock(cost_function_project, loss_function, para_q, para_t);
        }
        for(int i = 0; i < good_matches_intensity.size(); i++)
        {
            PointType point_now;
            PointType point_last;
            int now_index = good_matches_intensity[i].queryIdx;
            int last_index = good_matches_intensity[i].trainIdx;
            int point_index_now = BEV_now.keypoints_intensity[now_index].pt.y*BEV_cols + BEV_now.keypoints_intensity[now_index].pt.x;
            int point_index_last = BEV_last.keypoints_intensity[last_index].pt.y*BEV_cols + BEV_last.keypoints_intensity[last_index].pt.x;
            point_now = BEV_now.laserCloudintensity->points[point_index_now];
            point_last = BEV_last.laserCloudintensity->points[point_index_last];
            if(point_now.x == 0 && point_now.y == 0 && point_now.z == 0)
            {
                continue;
            }
            if(point_last.x == 0 && point_last.y == 0 && point_last.z == 0)
            {
                continue;
            }
            pcl::PointCloud<PointType> temp_cloud;

            intensity_cloud_points->push_back(point_now);
            Eigen::Vector3d curr_point(point_now.x, point_now.y, point_now.z);
            Eigen::Vector3d last_point(point_last.x, point_last.y, point_last.z);
            temp_cloud = last_cloud_select[point_index_last];
            if(temp_cloud.size()>=5)
            {
                double min_dis = 9999; 
                for(int i = 0; i < temp_cloud.size() - 1; i++)
                {
                    for(int j = 0; j < temp_cloud.size() - 1 - i; j++)
                    {
                        double dis_1 = sqrt((temp_cloud.points[j].x - point_last.x)*(temp_cloud.points[j].x - point_last.x) + (temp_cloud.points[j].y - point_last.y)*(temp_cloud.points[j].y - point_last.y) + (temp_cloud.points[j].z - point_last.z)*(temp_cloud.points[j].z - point_last.z));
                        double dis_2 = sqrt((temp_cloud.points[j+1].x - point_last.x)*(temp_cloud.points[j+1].x - point_last.x) + (temp_cloud.points[j+1].y - point_last.y)*(temp_cloud.points[j+1].y - point_last.y) + (temp_cloud.points[j+1].z - point_last.z)*(temp_cloud.points[j+1].z - point_last.z));
                        PointType cloud_temp;
                        if(dis_1 > dis_2)
                        {
                            cloud_temp = temp_cloud.points[j+1];
                            temp_cloud.points[j+1] = temp_cloud.points[j];
                            temp_cloud.points[j] = cloud_temp; 
                        }
                    }
                }
                Eigen::Vector3d center(0, 0, 0);
                std::vector<Eigen::Vector3d> nearCorners;
                for (int j = 0; j < 5; j++)
                {
                    Eigen::Vector3d tmp(temp_cloud[j].x, temp_cloud[j].y, temp_cloud[j].z);
                    center = center + tmp;
                    nearCorners.push_back(tmp);
                }
                center = center / 5.0;
                Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();
                for (int j = 0; j < 5; j++)
                {
                    Eigen::Matrix<double, 3, 1> tmpZeroMean = nearCorners[j] - center;
                    covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
                }
                Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);
                Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);
                if(saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1])
                {
                    Eigen::Vector3d point_on_line = center;
                    Eigen::Vector3d point_a, point_b;
                    point_a = 0.1 * unit_direction + point_on_line;
                    point_b = -0.1 * unit_direction + point_on_line;
                    // ceres::CostFunction *cost_function = LidarEdgeFactor::Create(curr_point, point_a, point_b, 100);
                    // problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);                            
                }
                else
                {
                    Eigen::Vector3d factor(0.1, 0.1, 100);
                    ceres::CostFunction *cost_function = LidarICPFactor::Create(curr_point, last_point, factor);
                    problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);
                }
            }
            else
            {
                    Eigen::Vector3d factor(0.1, 0.1, 100);
                    ceres::CostFunction *cost_function = LidarICPFactor::Create(curr_point, last_point, factor);
                    problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);
            }

            // Eigen::Vector2d last_point_index(BEV_last.keypoints_intensity[last_index].pt.y, BEV_last.keypoints_intensity[last_index].pt.x);
            // Eigen::Vector3d image_params(image_height,image_length,image_resolution);
            // ceres::CostFunction *cost_function_project = Reprojection_Factor::Create(curr_point,last_point_index, image_params);
            // problem.AddResidualBlock(cost_function_project, loss_function, para_q, para_t);
        }
        if(good_matches_height.size() == 0)
        {
            ROS_ERROR("--laserOdometry: height orb not matching!!!!!!!!");
        }
        if(good_matches_intensity.size() == 0)
        {
            ROS_ERROR("--laserOdometry: intensity orb not matching!!!!!!!!");
        }
        if(max_dist == 0)
        {
            ROS_ERROR("--laserOdometry: height orb max=0 !!!!!!!!");
        }
        if(max_dist2 == 0)
        {
            ROS_ERROR("--laserOdometry: intensity orb max= !!!!!!!!");
        }
        TicToc t_solver;
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.max_num_iterations = 10;
        options.minimizer_progress_to_stdout = false;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        if((t_last_curr-t_last_curr_histoary).norm()>2)
        {
            t_last_curr = Eigen::Vector3d(0, 0, 0);
            q_last_curr = Eigen::Quaterniond(1, 0, 0, 0);
            ba_flag = 0;
            ROS_ERROR("--laserOdometry: erroooooooooooooooooooooo !!!!!!!!");
        }
        t_w_curr = t_w_curr + q_w_curr * t_last_curr; // 通过优化得到两帧之间位姿增量，累加得到当前位姿；
        q_w_curr = q_w_curr * q_last_curr;
        q_w_curr.normalize();

        sensor_msgs::PointCloud2 laserCloudheight_point_Msg;
        pcl::toROSMsg(*height_cloud_points, laserCloudheight_point_Msg);
        laserCloudheight_point_Msg.header.stamp = ros::Time().fromSec(timeLaserCloudintensity);
        laserCloudheight_point_Msg.header.frame_id = "aft_mapped"; // /camera
        pubLaserCloudheight_point.publish(laserCloudheight_point_Msg);

        sensor_msgs::PointCloud2 laserCloudintensity_point_Msg;
        pcl::toROSMsg(*intensity_cloud_points, laserCloudintensity_point_Msg);
        laserCloudintensity_point_Msg.header.stamp = ros::Time().fromSec(timeLaserCloudintensity);
        laserCloudintensity_point_Msg.header.frame_id = "aft_mapped"; // /camera
        pubLaserCloudintensity_point.publish(laserCloudintensity_point_Msg);

        // std::cout<<"max:"<<max_dist<<std::endl;
        // cv::Mat outimg1, outimg2;
        // cv::drawKeypoints(BEV_now.BEV_height,BEV_now.keypoints_height,outimg1,cv::Scalar::all(-1),cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        // cv::drawKeypoints(BEV_last.BEV_height,BEV_last.keypoints_height,outimg2,cv::Scalar::all(-1),cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        // cv::imwrite("/home/wsc/bev_lo/src/orb_lio/img/BEV_height.png", outimg1);
        // cv::imwrite("/home/wsc/bev_lo/src/orb_lio/img/BEV_height_last.png", outimg2);

    }
    
    if(surround_keyframes%BA_freq == 0 && !surroundingBEV.empty() && keyframes_opt == 0 && local_BA_enable == 1)
    {
        BEV_now.q_w_curr = q_w_curr;
        BEV_now.t_w_curr = t_w_curr;
        local_Bundle_Adjustment(BEV_now, surroundingBEV, surrounding_heightpoint, keyframe_matches, surrounding_intensitypoint, keyframe_matches2, local_BA_type);
        // ROS_WARN("--laserOdometry local BA finish");
    }
    if((t_w_curr-BEV_now.t_w_curr).norm()>3)
    {
        t_w_curr = BEV_now.t_w_curr;
        q_w_curr = BEV_now.q_w_curr;
        ba_flag = 0;
        ROS_ERROR("--laserOdometry: erroooooooooooooooooooooo !!!!!!!!");
    }
    BEV_now.q_w_curr = q_w_curr;
    BEV_now.t_w_curr = t_w_curr;

    if(!surroundingBEV.empty())
    {
        Eigen::Vector3d ypr_b = Utility::R2ypr((surroundingBEV.back().q_w_curr).toRotationMatrix());
        Eigen::Vector3d ypr_c = Utility::R2ypr(q_w_curr.toRotationMatrix());
        float dx = (surroundingBEV.back().t_w_curr).x() - t_w_curr.x();
        float dy = (surroundingBEV.back().t_w_curr).y() - t_w_curr.y();
        float dz = (surroundingBEV.back().t_w_curr).z() - t_w_curr.z();
        float dyaw = ypr_b.x() - ypr_c.x();
        float dpitch = ypr_b.y() - ypr_c.y();
        float droll = ypr_b.z() - ypr_c.z(); 
        if (dyaw > M_PI) dyaw = dyaw - M_PI * 2; 
        if (dyaw < -M_PI) dyaw = dyaw + M_PI * 2; 
        if (abs(droll) > keyframeAddingAngle || 
            abs(dpitch) > keyframeAddingAngle || 
            abs(dyaw) > keyframeAddingAngle || 
            sqrt(dx * dx + dy * dy + dz * dz) > keyframeAddingDistance)
        {
            if(local_BA_enable == 1)
            {
                if(local_BA_type == 0 || local_BA_type == 2)
                {
                    std::vector<cv::DMatch> keymatches;
                    extract_goodmatches(BEV_now, surroundingBEV.back(), &keymatches, 0);
                    keyframe_matches.push_back(keymatches);
                }
                if(local_BA_type == 1 || local_BA_type == 2)
                {
                    std::vector<cv::DMatch> keymatches2;
                    extract_goodmatches(BEV_now, surroundingBEV.back(), &keymatches2, 1);
                    keyframe_matches2.push_back(keymatches2);
                }
            }
            surroundingBEV.push_back(BEV_now);
            surrounding_heightpoint.push_back(*(BEV_now.laserCloudheight));
            surrounding_intensitypoint.push_back(*(BEV_now.laserCloudintensity));
            keyframes_opt = 0;
            surround_keyframes++;
        }
    }
    if(surroundingBEV.size() > slipwide)
    {
        surroundingBEV.pop_front();
        surrounding_heightpoint.pop_front();
        surrounding_intensitypoint.pop_front();
        if(local_BA_enable == 1)
        {
            if(local_BA_type == 0 || local_BA_type == 2)
            keyframe_matches.pop_front();
            if(local_BA_type == 1 || local_BA_type == 2)
            keyframe_matches2.pop_front();
        }
    }
                
// publish laser odometry    
///////////////////////////////////////////////////////////////////////
    nav_msgs::Odometry laserOdometry;
    laserOdometry.header.frame_id = "camera_init";
    laserOdometry.child_frame_id = "laser_odom";
    laserOdometry.header.stamp = ros::Time().fromSec(timeLaserCloudheight);
    laserOdometry.pose.pose.orientation.x = q_w_curr.x(); // q_w_curr
    laserOdometry.pose.pose.orientation.y = q_w_curr.y();
    laserOdometry.pose.pose.orientation.z = q_w_curr.z();
    laserOdometry.pose.pose.orientation.w = q_w_curr.w();
    laserOdometry.pose.pose.position.x = t_w_curr.x(); // t_w_curr
    laserOdometry.pose.pose.position.y = t_w_curr.y();
    laserOdometry.pose.pose.position.z = t_w_curr.z();
    pubLaserOdometry.publish(laserOdometry);

    iter_num++;
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin(tf::Vector3(t_w_curr.x(),t_w_curr.y(),t_w_curr.z())); // t_w_curr
    q.setW(q_w_curr.w()); // q_w_curr
    q.setX(q_w_curr.x());
    q.setY(q_w_curr.y());
    q.setZ(q_w_curr.z());
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, laserOdometry.header.stamp, "camera_init", "laser_odom"));

    Eigen::Matrix4d T_lk = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d T_rk = Eigen::Matrix4d::Identity();
    T_lk.block<3, 1>(0, 3) = t_w_curr;
    T_lk.block<3, 3>(0, 0) = Eigen::Matrix3d(q_w_curr);
    T_rk = T_rl * T_lk * T_rl.inverse();
    Eigen::Vector3d t_robot;
    Eigen::Quaterniond q_robot;
    t_robot = T_rk.block<3, 1>(0, 3);
    q_robot = T_rk.block<3, 3>(0, 0);

    geometry_msgs::PoseStamped RobotPose;
    RobotPose.header.stamp = laserOdometry.header.stamp;
    RobotPose.header.frame_id = "camera_init";
    RobotPose.pose.orientation.x = q_robot.x();
    RobotPose.pose.orientation.y = q_robot.y();
    RobotPose.pose.orientation.z = q_robot.z();
    RobotPose.pose.orientation.w = q_robot.w();
    RobotPose.pose.position.x = t_robot.x();
    RobotPose.pose.position.y = t_robot.y();
    RobotPose.pose.position.z = t_robot.z();
    laserPath.header.stamp = laserOdometry.header.stamp;
    laserPath.header.frame_id = "camera_init";
    laserPath.poses.push_back(RobotPose);
    pubLaserPath.publish(laserPath); 
    //publish 点云
    pcl::PointCloud<PointType>::Ptr height_cloud2(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr intensity_cloud2(new pcl::PointCloud<PointType>);
    std::vector<bool> select_flag;
    select_flag.resize(BEV_rows*BEV_cols);
    for(int i = 0; i < BEV_now.keypoints_height.size(); i++)
    {
        int point_index_now = BEV_now.keypoints_height[i].pt.y*BEV_cols + BEV_now.keypoints_height[i].pt.x;
        if(BEV_now.laserCloudheight->points[point_index_now].x == 0 && BEV_now.laserCloudheight->points[point_index_now].y == 0 && BEV_now.laserCloudheight->points[point_index_now].z == 0)
        {
                continue;
        }
        pcl::PointCloud<PointType> temp_cloud;
        temp_cloud = cloud_select[point_index_now];
        if(temp_cloud.empty())
            continue;
        *height_cloud2 += temp_cloud;
        select_flag[point_index_now] = 1;
    }
    for(int i = 0; i < BEV_now.keypoints_intensity.size(); i++)
    {
        int point_index_now = BEV_now.keypoints_intensity[i].pt.y*BEV_cols + BEV_now.keypoints_intensity[i].pt.x;
        if(select_flag[point_index_now] == 1)
        {
            continue;
        }
        if(BEV_now.laserCloudintensity->points[point_index_now].x == 0 && BEV_now.laserCloudintensity->points[point_index_now].y == 0 && BEV_now.laserCloudintensity->points[point_index_now].z == 0)
        {
                continue;
        }
        pcl::PointCloud<PointType> temp_cloud;
        temp_cloud = cloud_select[point_index_now];
        if(temp_cloud.empty())
            continue;
        *intensity_cloud2 += temp_cloud;
    }   
    sensor_msgs::PointCloud2 laserCloudheight_Msg;
    pcl::toROSMsg(*height_cloud2, laserCloudheight_Msg);
    laserCloudheight_Msg.header.stamp = ros::Time().fromSec(timeLaserCloudheight);
    laserCloudheight_Msg.header.frame_id = "aft_mapped"; // /camera
    pubLaserCloudheight.publish(laserCloudheight_Msg);

    sensor_msgs::PointCloud2 laserCloudintensity_Msg;
    pcl::toROSMsg(*intensity_cloud2, laserCloudintensity_Msg);
    laserCloudintensity_Msg.header.stamp = ros::Time().fromSec(timeLaserCloudintensity);
    laserCloudintensity_Msg.header.frame_id = "aft_mapped"; // /camera
    pubLaserCloudintensity.publish(laserCloudintensity_Msg);

    sensor_msgs::ImagePtr hight_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", BEV_now.BEV_height).toImageMsg();
    hight_msg->header.stamp = ros::Time().fromSec(timeLaserCloudheight);;
    hight_msg->header.frame_id = "camera_init";
    pubheightBEV.publish(hight_msg);

    sensor_msgs::ImagePtr intensity_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", BEV_now.BEV_intensity).toImageMsg();
    intensity_msg->header.stamp = ros::Time().fromSec(timeLaserCloudheight);;
    intensity_msg->header.frame_id = "camera_init";
    pubintensityBEV.publish(intensity_msg);



    // last赋值
    BEV_last.laserCloudintensity->clear();
    BEV_last.laserCloudintensity = BEV_now.laserCloudintensity->makeShared();
    BEV_last.laserCloudheight->clear();
    BEV_last.laserCloudheight = BEV_now.laserCloudheight->makeShared();
    BEV_last.BEV_height = BEV_now.BEV_height;
    BEV_last.BEV_intensity = BEV_now.BEV_intensity;
    BEV_last.description_height = BEV_now.description_height;
    BEV_last.description_intensity = BEV_now.description_intensity;
    BEV_last.keypoints_height = BEV_now.keypoints_height;
    BEV_last.keypoints_intensity = BEV_now.keypoints_intensity;
    BEV_last.height_point_num = BEV_now.height_point_num;
    BEV_last.intensity_point_num = BEV_now.intensity_point_num;
    BEV_last.q_w_curr = BEV_now.q_w_curr;
    BEV_last.t_w_curr = BEV_now.t_w_curr;
    last_cloud_select = cloud_select;

    t_last_curr_histoary = t_last_curr;
    q_last_curr_histoary = q_last_curr;
    
    mBuf.unlock();
}
void removeClosedPointCloud(const pcl::PointCloud<PointType> &cloud_in, pcl::PointCloud<PointType> &cloud_out, float th1,  float th2)
    {
        if (&cloud_in != &cloud_out)
        {
            cloud_out.header = cloud_in.header;
            cloud_out.points.resize(cloud_in.points.size());
        }

        size_t j = 0;

        for (size_t i = 0; i < cloud_in.points.size(); ++i)
        {
            float dis = cloud_in.points[i].x * cloud_in.points[i].x + cloud_in.points[i].y * cloud_in.points[i].y + cloud_in.points[i].z * cloud_in.points[i].z;
            if(dis < th1 * th1)
                continue;
            if(dis > th2 * th2)
                continue;
            if(cloud_in.points[i].x < 0 && abs(cloud_in.points[i].y) < 0.5)
                continue;
            cloud_out.points[j] = cloud_in.points[i];
            j++;
        }

        if (j != cloud_in.points.size())
        {
            cloud_out.points.resize(j);
        }

        cloud_out.height = 1;
        cloud_out.width = static_cast<uint32_t>(j);
        cloud_out.is_dense = true;
    }
void pixel_detect(const pcl::PointCloud<PointType>::Ptr cloud_in, cv::Mat *BEV_in)
{
    for(int bev_x = 0; bev_x < BEV_rows; bev_x++)
    {
        for(int bev_y = 0; bev_y < BEV_cols; bev_y++)
        {
            int indexA = bev_x*BEV_cols + bev_y;
            if((cloud_in->points[indexA].x == 0 && cloud_in->points[indexA].y == 0 && cloud_in->points[indexA].z == 0) || cloud_in->points[indexA].z < -1)
            {
                BEV_in->at<uchar>(bev_x, bev_y) = 0;
                continue;
            }
            Eigen::Vector3d pointA(cloud_in->points[indexA].x, cloud_in->points[indexA].y, cloud_in->points[indexA].z);
            double max_cos = -999;
            for(int r = 1-detect_range; r < detect_range; r++)
            {
                for(int c = 0; c < detect_range; c++)
                {
                    int temp_x = r + detect_range-1;
                    int temp_y = c + detect_range-1;
                    if(BEV_detect.at<uchar>(temp_x,temp_y) == 255)
                    {
                        if(bev_x - abs(r) < 0 || bev_x + abs(r) >= BEV_rows)
                        {
                            continue;
                        }
                        if(bev_y + c >= BEV_cols || (bev_y - c) < 0)
                        {
                            continue;
                        }
                        int indexB = (bev_x + r)*BEV_cols + (bev_y + c);
                        int indexC = (bev_x - r)*BEV_cols + (bev_y - c);
                        Eigen::Vector3d pointB(cloud_in->points[indexB].x, cloud_in->points[indexB].y, cloud_in->points[indexB].z);
                        Eigen::Vector3d pointC(cloud_in->points[indexC].x, cloud_in->points[indexC].y, cloud_in->points[indexC].z);
                        if(pointB.x() == 0 && pointB.y() == 0 && pointB.z() == 0)
                        {
                            continue;
                        }
                        if(pointC.x() == 0 && pointC.y() == 0 && pointC.z() == 0)
                        {
                            continue;
                        }                        
                        double point_cos = ((pointB - pointA).dot(pointC - pointA))/((pointB - pointA).norm()*(pointC - pointA).norm());
                        point_cos += 1;
                        if(point_cos > max_cos)
                        {
                            max_cos = point_cos;
                        }
                    }
                
                }
            }
            if(max_cos != -999)
            {
                BEV_in->at<uchar>(bev_x, bev_y) = (max_cos/2)*BEV_in->at<uchar>(bev_x, bev_y);
            }
        }
    }
}

void pixel_detect2(const pcl::PointCloud<PointType>::Ptr cloud_in, cv::Mat *BEV_in)
{
    for(int bev_x = 0; bev_x < BEV_rows; bev_x++)
    {
        for(int bev_y = 0; bev_y < BEV_cols; bev_y++)
        {
            int indexA = bev_x*BEV_cols + bev_y;
            Eigen::Vector3d pt(cloud_in->points[indexA].x, cloud_in->points[indexA].y, cloud_in->points[indexA].z);
            double range = pt.norm();
            double geo = 0;
            int n_u_f = bev_x-1;
            int n_u_b = bev_x+1;
            int n_v_f = bev_y-1;
            int n_v_b = bev_y+1;
            if (n_u_f<0)
			    n_u_f=0;
            if (n_v_f<0)
                n_v_f=0;
            if (n_u_b>=BEV_rows)
                n_u_b=BEV_rows;
            if (n_v_b>=BEV_cols)
                n_v_b=BEV_cols;
            if (n_u_f==0&&n_v_f==0)
            {
                BEV_in->at<uchar>(bev_x, bev_y) = 0;
                continue;
            }
            if (n_u_b==BEV_rows&&n_v_b==BEV_cols)
            {
                BEV_in->at<uchar>(bev_x, bev_y) = 0;
                continue;
            }
            int indexf = n_u_f*BEV_cols + n_v_f;
            int indexb = n_u_b*BEV_cols + n_v_b;
            Eigen::Vector3d pt_neibor_f(cloud_in->points[indexf].x, cloud_in->points[indexf].y, cloud_in->points[indexf].z);
            Eigen::Vector3d pt_neibor_b(cloud_in->points[indexb].x, cloud_in->points[indexb].y, cloud_in->points[indexb].z);
            double dis_A = (pt - pt_neibor_f).norm();
            double dis_B = (pt - pt_neibor_b).norm();
            double dis_C = (pt_neibor_f - pt_neibor_b).norm();
		    //float dis_C= std::sqrt(pt_neibor_f.x*pt_neibor_f.x + pt_neibor_f.y*pt_neibor_f.y + pt_neibor_f.z*pt_neibor_f.z);
            //float dis_dif=range-dis_C;
            if (range <= 0.1)
            {
                geo = 0;                
            }
            else if(dis_C <= 0.1 && dis_B <= 0.1)
            {
                geo = 0;
            }
            else
            {
                if ((pt.z() + 3) <= 0)
				    geo = 0;
                if ((pt.z() + 3) >= 15) //kitti05/08 dataset (pt.z+4)>=10, 0.025, 0.98   loopnum05=358 
                    geo = 255;  //kitti05 dataset (pt.z+3)>=10, qingxi kandao qiche  loopnum=361
                else
                {
                    // geo= (pt.z() + 3)/15*255;
                    // geo= (atan((pt.z()-pt_neibor_b.z())/5)+M_PI/2)/M_PI*255;
                    geo=255-2*acos( (dis_A*dis_A+range*range-dis_C*dis_C)/(2*dis_A*range) )*255/M_PI;
                }
                    
            }
            BEV_in->at<uchar>(bev_x, bev_y) = geo;
            
        }
    }
}

void pixel_detect_intensity(const pcl::PointCloud<PointType>::Ptr cloud_in, cv::Mat *BEV_in)
{
    double max_h = -1000; 
    double min_h = 1000;
    for(int bev_x = 0; bev_x < BEV_rows; bev_x++)
    {
        for(int bev_y = 0; bev_y < BEV_cols; bev_y++)
        {
            int indexA = bev_x*BEV_cols + bev_y;
            if(cloud_in->points[indexA].x == 0 && cloud_in->points[indexA].y == 0 && cloud_in->points[indexA].z == 0)
            {
                BEV_in->at<uchar>(bev_x, bev_y) = 0;
                continue;
            }
            Eigen::Vector3d pointA(cloud_in->points[indexA].x, cloud_in->points[indexA].y, cloud_in->points[indexA].z);
            double diffI = 0;
            int N = 0;
            for(int r = 1-detect_range; r < detect_range; r++)
            {
                for(int c = 1-detect_range; c < detect_range; c++)
                {
                    int temp_x = r + detect_range-1;
                    int temp_y = c + detect_range-1;
                    if(BEV_detect.at<uchar>(temp_x,temp_y) == 255)
                    {
                        if(bev_x + r < 0 || bev_x + r >= BEV_rows)
                        {
                            continue;
                        }
                        if(bev_y + c >= BEV_cols || bev_y + c < 0)
                        {
                            continue;
                        }
                        int indexB = (bev_x + r)*BEV_cols + (bev_y + c);
                        Eigen::Vector3d pointB(cloud_in->points[indexB].x, cloud_in->points[indexB].y, cloud_in->points[indexB].z);

                        if(pointB.x() == 0 && pointB.y() == 0 && pointB.z() == 0)
                        {
                            continue;
                        }
                        diffI += cloud_in->points[indexB].intensity - cloud_in->points[indexA].intensity;
                        N++;
                    }
                
                }
            }
            if(N != 0)
            {
                BEV_in->at<uchar>(bev_x, bev_y) = abs(diffI)/N;
                if(abs(diffI)/N < min_h)
                {
                    min_h = abs(diffI)/N;
                }
                if(abs(diffI)/N > max_h)
                {
                    max_h = abs(diffI)/N;
                }
            }
            else
            {
                BEV_in->at<uchar>(bev_x, bev_y) = 0;
            }
        }
    }

    for(int bev_x = 0; bev_x < BEV_rows; bev_x++)
    {
        for(int bev_y = 0; bev_y < BEV_cols; bev_y++)
        {
            if(BEV_in->at<uchar>(bev_x, bev_y) != 0)
            {
                BEV_in->at<uchar>(bev_x, bev_y) = 255*((BEV_in->at<uchar>(bev_x, bev_y) - min_h)/(max_h - min_h));
            }
        }
    }

}

void adjustDistortion()
{
    // 点云去畸变，并转换到这一帧的时间戳处(即lidar帧最后一个点的坐标系下)
    Eigen::Quaterniond q_last_inverse = q_last_curr.inverse();
    // 这里存在一个假设：在去除点云畸变时认为机器人在lidar扫描一帧的过程中是匀速运动的
    for (int i = 0; i < laserCloudheightNum; i++)
    {
        double s = 1 - (BEV_now.laserCloudheight->points[i].intensity - int(BEV_now.laserCloudheight->points[i].intensity)) / SCAN_PERIOD;
        Eigen::Quaterniond q_point_last = Eigen::Quaterniond::Identity().slerp(s, q_last_inverse);
        Eigen::Vector3d t_point_last = s * t_last_curr;
        Eigen::Vector3d point(BEV_now.laserCloudheight->points[i].x, BEV_now.laserCloudheight->points[i].y, BEV_now.laserCloudheight->points[i].z);
        Eigen::Vector3d point_end = q_point_last * (point - t_point_last);
        BEV_now.laserCloudheight->points[i].x = point_end.x();
        BEV_now.laserCloudheight->points[i].y = point_end.y();
        BEV_now.laserCloudheight->points[i].z = point_end.z();
    }

    for (int i = 0; i < laserCloudintensityNum; i++)
    {
        double s = 1 - (BEV_now.laserCloudintensity->points[i].intensity - int(BEV_now.laserCloudintensity->points[i].intensity)) / SCAN_PERIOD;
        Eigen::Quaterniond q_point_last = Eigen::Quaterniond::Identity().slerp(s, q_last_inverse);
        Eigen::Vector3d t_point_last = s * t_last_curr;
        Eigen::Vector3d point(BEV_now.laserCloudintensity->points[i].x, BEV_now.laserCloudintensity->points[i].y, BEV_now.laserCloudintensity->points[i].z);
        Eigen::Vector3d point_end = q_point_last * (point - t_point_last);
        BEV_now.laserCloudintensity->points[i].x = point_end.x();
        BEV_now.laserCloudintensity->points[i].y = point_end.y();
        BEV_now.laserCloudintensity->points[i].z = point_end.z();
    }
}

void local_Bundle_Adjustment(BEV_s BEV_cur, deque<BEV_s> BEV_history, deque<pcl::PointCloud<PointType>> point_history, deque<std::vector<cv::DMatch>> matches_history,  deque<pcl::PointCloud<PointType>> point_history2, deque<std::vector<cv::DMatch>> matches_history2, int BA_type)
{
    double para_q_w[4] = {0, 0, 0, 1};
    double para_t_w[3] = {0, 0, 0};
    Eigen::Map<Eigen::Quaterniond> w_q_curr(para_q_w);
    w_q_curr = BEV_now.q_w_curr;
    Eigen::Map<Eigen::Vector3d> w_t_curr(para_t_w);
    w_t_curr = BEV_now.t_w_curr;

    ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
    ceres::LocalParameterization *q_parameterization = new ceres::EigenQuaternionParameterization();
    ceres::Problem::Options problem_options;
    ceres::Problem problem(problem_options);
    problem.AddParameterBlock(para_q_w, 4, q_parameterization);
    problem.AddParameterBlock(para_t_w, 3);

    std::vector<cv::DMatch> cur2last_matches;
    if(BA_type == 0 || BA_type == 2)
    {
        extract_goodmatches(BEV_cur, BEV_history.back(), &cur2last_matches, 0);
    
        for (int key = 0; key < cur2last_matches.size(); key++)
        {
                PointType point_now;
                PointType point_last;
                int now_index = cur2last_matches[key].queryIdx;
                int last_index = cur2last_matches[key].trainIdx;
                
                int point_index_now = BEV_cur.keypoints_height[now_index].pt.y*BEV_cols + BEV_cur.keypoints_height[now_index].pt.x;
                int point_index_last = BEV_history.back().keypoints_height[last_index].pt.y*BEV_cols + BEV_history.back().keypoints_height[last_index].pt.x;
                point_now = BEV_cur.laserCloudheight->points[point_index_now];
                // ROS_WARN("--laserOdometry point_historysize:%d, key:%d, pointsize:%d, point_index_last:%d", point_history.size(), key, point_history[key].points.size(), point_index_last);
                point_last = point_history.back().points[point_index_last];
                if(point_now.x == 0 && point_now.y == 0 && point_now.z == 0)
                {
                    continue;
                }
                if(point_last.x == 0 && point_last.y == 0 && point_last.z == 0)
                {
                    continue;
                }
                
                
                Eigen::Vector3d curr_point(point_now.x, point_now.y, point_now.z);
                Eigen::Vector3d last_point(point_last.x, point_last.y, point_last.z);
                Eigen::Vector3d last_point_world = BEV_history.back().q_w_curr * last_point + BEV_history.back().t_w_curr;
                if(BA_type == 0)
                {
                    Eigen::Vector3d factor(1, 1, 1);
                    ceres::CostFunction *cost_function = local_BA_Factor::Create(curr_point,last_point_world, factor);
                    problem.AddResidualBlock(cost_function, loss_function, para_q_w, para_t_w);
                }
                if(BA_type == 2)
                {
                    Eigen::Vector3d factor(100, 100, 0.01);
                    ceres::CostFunction *cost_function = local_BA_Factor::Create(curr_point,last_point_world, factor);
                    problem.AddResidualBlock(cost_function, loss_function, para_q_w, para_t_w);
                }

                int break_flag = 1;
                for(int i = BEV_history.size()-2; i>=0; i--)
                {
                    if(i<0)
                    {
                        break;
                    }
                    for(int j = 0; j < matches_history[i].size(); j++)
                    {
                        if(matches_history[i][j].queryIdx == last_index)
                        {
                            last_index = matches_history[i][j].trainIdx;
                            point_index_last = BEV_history[i].keypoints_height[last_index].pt.y*BEV_cols + BEV_history[i].keypoints_height[last_index].pt.x;
                            point_last = point_history[i].points[point_index_last];
                            if(point_last.x == 0 && point_last.y == 0 && point_last.z == 0)
                            {
                                break;
                            }
                            last_point = Eigen::Vector3d(point_last.x, point_last.y, point_last.z);
                            last_point_world = BEV_history[i].q_w_curr * last_point + BEV_history[i].t_w_curr;

                            if(BA_type == 0)
                            {
                                Eigen::Vector3d factor(1, 1, 1);
                                ceres::CostFunction *cost_function = local_BA_Factor::Create(curr_point,last_point_world, factor);
                                problem.AddResidualBlock(cost_function, loss_function, para_q_w, para_t_w);
                            }
                            if(BA_type == 2)
                            {
                                Eigen::Vector3d factor(100, 100, 0.01);
                                ceres::CostFunction *cost_function = local_BA_Factor::Create(curr_point,last_point_world, factor);
                                problem.AddResidualBlock(cost_function, loss_function, para_q_w, para_t_w);
                            }
                            break_flag = 0;
                            break;
                        }
                    }
                    if(break_flag == 1)
                    {
                        break;
                    }
                    if(break_flag == 0)
                    {
                        break_flag = 1;
                    }
                }        
        }
    }
    std::vector<cv::DMatch> cur2last_matches2;
    if(BA_type == 1 || BA_type == 2)
    {
        extract_goodmatches(BEV_cur, BEV_history.back(), &cur2last_matches2, 1);

        for (int key = 0; key < cur2last_matches2.size(); key++)
        {
                PointType point_now;
                PointType point_last;
                int now_index = cur2last_matches2[key].queryIdx;
                int last_index = cur2last_matches2[key].trainIdx;
                
                int point_index_now = BEV_cur.keypoints_intensity[now_index].pt.y*BEV_cols + BEV_cur.keypoints_intensity[now_index].pt.x;
                int point_index_last = BEV_history.back().keypoints_intensity[last_index].pt.y*BEV_cols + BEV_history.back().keypoints_intensity[last_index].pt.x;
                point_now = BEV_cur.laserCloudintensity->points[point_index_now];
                // ROS_WARN("--laserOdometry point_historysize:%d, key:%d, pointsize:%d, point_index_last:%d", point_history.size(), key, point_history[key].points.size(), point_index_last);
                point_last = point_history2.back().points[point_index_last];
                if(point_now.x == 0 && point_now.y == 0 && point_now.z == 0)
                {
                    continue;
                }
                if(point_last.x == 0 && point_last.y == 0 && point_last.z == 0)
                {
                    continue;
                }
                
                
                Eigen::Vector3d curr_point(point_now.x, point_now.y, point_now.z);
                Eigen::Vector3d last_point(point_last.x, point_last.y, point_last.z);
                Eigen::Vector3d last_point_world = BEV_history.back().q_w_curr * last_point + BEV_history.back().t_w_curr;

                if(BA_type == 1)
                {
                    Eigen::Vector3d factor(1, 1, 1);
                    ceres::CostFunction *cost_function = local_BA_Factor::Create(curr_point,last_point_world, factor);
                    problem.AddResidualBlock(cost_function, loss_function, para_q_w, para_t_w);
                }
                if(BA_type == 2)
                {
                    Eigen::Vector3d factor(0.01, 0.01, 100);
                    ceres::CostFunction *cost_function = local_BA_Factor::Create(curr_point,last_point_world, factor);
                    problem.AddResidualBlock(cost_function, loss_function, para_q_w, para_t_w);
                }
                int break_flag = 1;
                for(int i = BEV_history.size()-2; i>=0; i--)
                {
                    if(i<0)
                    {
                        break;
                    }
                    for(int j = 0; j < matches_history2[i].size(); j++)
                    {
                        if(matches_history2[i][j].queryIdx == last_index)
                        {
                            last_index = matches_history2[i][j].trainIdx;
                            point_index_last = BEV_history[i].keypoints_intensity[last_index].pt.y*BEV_cols + BEV_history[i].keypoints_intensity[last_index].pt.x;
                            point_last = point_history2[i].points[point_index_last];
                            if(point_last.x == 0 && point_last.y == 0 && point_last.z == 0)
                            {
                                break;
                            }
                            last_point = Eigen::Vector3d(point_last.x, point_last.y, point_last.z);
                            last_point_world = BEV_history[i].q_w_curr * last_point + BEV_history[i].t_w_curr;
                            if(BA_type == 1)
                            {
                                Eigen::Vector3d factor(1, 1, 1);
                                ceres::CostFunction *cost_function = local_BA_Factor::Create(curr_point,last_point_world, factor);
                                problem.AddResidualBlock(cost_function, loss_function, para_q_w, para_t_w);
                            }
                            if(BA_type == 2)
                            {
                                Eigen::Vector3d factor(0.01, 0.01, 100);
                                ceres::CostFunction *cost_function = local_BA_Factor::Create(curr_point,last_point_world, factor);
                                problem.AddResidualBlock(cost_function, loss_function, para_q_w, para_t_w);
                            }
                            break_flag = 0;
                            break;
                        }
                    }
                    if(break_flag == 1)
                    {
                        break;
                    }
                    if(break_flag == 0)
                    {
                        break_flag = 1;
                    }
                }        
        }
    }
    
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = 10;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    q_w_curr = w_q_curr;
    t_w_curr = w_t_curr;
}

void TransformToStart(PointType const *const pi, PointType *const po)
{
    // 使用当前优化得到的q、t将当前lidar点云向上一帧点云坐标系下转化
    Eigen::Vector3d point(pi->x, pi->y, pi->z);
    Eigen::Vector3d un_point = q_last_curr * point + t_last_curr;

    po->x = un_point.x();
    po->y = un_point.y();
    po->z = un_point.z();
    po->intensity = pi->intensity;
}

void extract_goodmatches(const BEV_s BEV_left, const BEV_s BEV_right, std::vector<cv::DMatch> *good_matches, int extract_type)
{
    cv::BFMatcher matcher(cv::NORM_HAMMING);
    if(extract_type == 0)
    {  
        vector<cv::DMatch> matches_height;
        matcher.match(BEV_left.description_height,BEV_right.description_height,matches_height);
        std::vector<bool> height_vbInliers;
        gms_matcher gms_height(BEV_left.keypoints_height, BEV_left.BEV_height.size(), BEV_right.keypoints_height, BEV_right.BEV_height.size(), matches_height);
        int height_lnliers = gms_height.GetInlierMask(height_vbInliers, false, true);
        double max_dist = -100000;
        double min_dist = 100000;
        for(int i = 0; i < matches_height.size(); i++)
        {
            double dist = abs(matches_height[i].distance);
            
            if(dist < min_dist) min_dist = dist ;
            if(dist > max_dist) max_dist = dist ;
        }

        for(int i = 0; i < height_vbInliers.size(); i++)
        {
            if(height_vbInliers[i] == true && matches_height[i].distance < (2*min_dist>50? 2*min_dist : 50))
            {
                good_matches->push_back(matches_height[i]);
            }
        }
        if(max_dist == 0)
        {
            ROS_ERROR("--laserOdometry: height orb max=0 !!!!!!!!");
        }
    }
    if(extract_type == 1)
    {
        vector<cv::DMatch> matches_intensity;
        matcher.match(BEV_left.description_intensity,BEV_right.description_intensity,matches_intensity);
        std::vector<bool> intensity_vbInliers;
        gms_matcher gms_intensity(BEV_left.keypoints_intensity, BEV_left.BEV_intensity.size(), BEV_right.keypoints_intensity, BEV_right.BEV_intensity.size(), matches_intensity);
        int intensity_lnliers = gms_intensity.GetInlierMask(intensity_vbInliers, false, true);
        double max_dist2 = -100000;
        double min_dist2 = 100000;
        for(int i = 0; i < matches_intensity.size(); i++)
        {
            double dist = abs(matches_intensity[i].distance);
            
            if(dist < min_dist2) min_dist2 = dist ;
            if(dist > max_dist2) max_dist2 = dist ;
        }
        for(int i = 0; i < intensity_vbInliers.size(); i++)
        {
            if(intensity_vbInliers[i] == true && matches_intensity[i].distance < (2*min_dist2>50? 2*min_dist2 : 50))
            {
                good_matches->push_back(matches_intensity[i]);
            }
        }
            if(max_dist2 == 0)
        {
            ROS_ERROR("--laserOdometry: intensity orb max= !!!!!!!!");
        }
    }
}

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "scanRegistration");

    ScanRegistration SR;

    ros::spin();
    return 0;
}