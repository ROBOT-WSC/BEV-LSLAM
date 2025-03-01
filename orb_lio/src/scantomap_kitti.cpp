#include "lidarFactor.hpp"
#include "orb_lio/utility.h"
#include "orb_lio/tic_toc.h"
#include <fast_gicp/gicp/fast_vgicp.hpp>
#include "orb_feature/gms_matcher.hpp"
#include "orb_feature/ORB_modify.hpp"
#include "DBoW3/DBoW3.h"

Eigen::Quaterniond q_wodom_curr(1, 0, 0, 0);
Eigen::Vector3d t_wodom_curr(0, 0, 0);
Eigen::Quaterniond q_wmap_wodom(1, 0, 0, 0);
Eigen::Vector3d t_wmap_wodom(0, 0, 0);

// double para_q[4] = {0, 0, 0, 1};
// double para_t[3] = {0, 0, 0};
Eigen::Quaterniond q_w_curr(1, 0, 0, 0);
Eigen::Vector3d t_w_curr(0, 0, 0);
// double para_q_last[4] = {0, 0, 0, 1};
// double para_t_last[3] = {0, 0, 0};
Eigen::Quaterniond q_w_last(1, 0, 0, 0);
Eigen::Vector3d t_w_last(0, 0, 0);

class LaserMapping: public ParamServer
{
    public:
	image_transport::Subscriber subheightBEV;
    image_transport::Subscriber subintensityBEV;

	image_transport::Publisher pubheightBEV;

    ros::Subscriber subLaserCloudHeight;
	ros::Subscriber subLaserCloudIntensity;
	ros::Subscriber subLaserOdometry;
	ros::Subscriber subtime;

    ros::Publisher pubLaserCloudSurround;
	ros::Publisher pubLaserCloudMap;
	ros::Publisher pubScanCloudMap;

    ros::Publisher pubOdomAftMapped;
	ros::Publisher pubOdomAftMappedHighFrec;
	ros::Publisher pubLaserAfterMappedPath;
	ros::Publisher pubLoamPath;
	ros::Publisher pubaLoamRobotPath;
	ros::Publisher pubGlobalPoseGraphPath;

	ros::Publisher pubMarker;
	ros::Publisher pubLoopConstraintEdge;
	ros::Publisher pubGlobalPoseGraphPoint;
	ros::Publisher pubGlobalMapKeyPose;
	ros::Publisher pubGlobalMapKeyPoseDS;
	ros::Publisher pubRobotPose;
	ros::Publisher pubtempcloud;

    std::mutex mBuf;

	queue<sensor_msgs::ImageConstPtr> heightBEVBuf;
    queue<sensor_msgs::ImageConstPtr> intensityBEVBuf;
    queue<sensor_msgs::PointCloud2ConstPtr> laserHeightBuf;
	queue<sensor_msgs::PointCloud2ConstPtr> laserIntensityBuf;
    queue<nav_msgs::Odometry::ConstPtr> odometryBuf;

    Eigen::Matrix3d R_rl, R_il;
	Eigen::Vector3d t_rl;
	Eigen::Matrix4d T_rl;

	double timeLaserCloudHeight = 0;
	double timeLaserCloudIntensity = 0;
	double timeLaserOdometry = 0;
	double timeimage = 0;
	double prevTime = 0, curTime = 0;
	double start_time = 0;
	int start_flag = 1;

	BEV_mapping BEV_now;
    BEV_mapping BEV_last;

    pcl::PointCloud<PointType>::Ptr laserCloudHeight;
    pcl::PointCloud<PointType>::Ptr laserCloudIntensity;

    pcl::PointCloud<PointType>::Ptr laserCloudHeightDS, laserCloudHeightLastDS;
    pcl::PointCloud<PointType>::Ptr laserCloudIntensityDS, laserCloudIntensityLastDS;
	pcl::PointCloud<PointType>::Ptr surroundingMapDS;

    pcl::PointCloud<PointType>::Ptr laserCloudHeightFromMap;
	pcl::PointCloud<PointType>::Ptr laserCloudHeightFromMapDS;
    pcl::PointCloud<PointType>::Ptr laserCloudIntensityFromMap;
    pcl::PointCloud<PointType>::Ptr laserCloudIntensityFromMapDS;

	int laserCloudHeightDSNum = 0, laserCloudHeightLastDSNum = 0;
    int laserCloudIntensityDSNum = 0, laserCloudIntensityLastDSNum = 0;
	int laserCloudHeightFromMapDSNum = 0;
    int laserCloudIntensityFromMapDSNum = 0;

    std::mutex mKeyframe;

    pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D, copy_cloudKeyPoses3D;
    pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D, copy_cloudKeyPoses6D;
    std::map<int, pcl::PointCloud<PointType>::Ptr> heightCloudKeyFrames;
    std::map<int, pcl::PointCloud<PointType>::Ptr> intensityCloudKeyFrames;
	std::map<int, BEV_mapping> DoWB_key_frames;
	deque<pcl::PointCloud<PointType>::Ptr> surroundingHeightCloud;
	deque<pcl::PointCloud<PointType>::Ptr> surroundingIntensityCloud;
	vector<int> surroundingExistingKeyPoseID;

    deque<Eigen::Quaterniond> histoary_pose;
	deque<Eigen::Vector3d> histoary_trans;

    int initfirst = 0;//2
	int initskipframe = 0;//1 20

	int loop_numm = 0;

    int keyFrameNum = 0;
    float surroundingKeyframeDensity = 0.3; // m
	int mapping_count = 0, laserOdom_count = 0;
	double cost_mapping = 0;

    Eigen::Vector3d t_robot;
	Eigen::Quaterniond q_robot;

	pcl::PointCloud<PointType>::Ptr latestKeyFrameCloud;
    pcl::PointCloud<PointType>::Ptr nearHistoryKeyFrameCloud;
	std::map<int, PointTypePose> KeyPose6D;
	std::map<int, float> travel_distance; // m
	std::map<int, float> travel_angle; // rad
	std::map<int, PointTypePose> correctedKeyPose6DByLoop;

    int loopClosureCount = 0;
	bool bnewKeyFrame = false;
	bool bLoopIsClosed = false;
	bool bKeyFramePoseGraphUpdated = false;
	float poseGraphSearchRadius = 15;
	float historyKeyframeSearchRadius = 10; // meters, key frame that is within n meters from current pose will be considerd for loop closure
	float historyKeyframeFitnessScore = 0.6; // icp threshold, the smaller the better alignment kiyyi00 2
    int historyKeyframeSearchNum = 50; // number of hostory key frames will be fused into a submap for loop closure
	float DistanceByLoop = 0;
	float DRIFT_FACTOR = 0.02; // SLAM的漂移度，随着运行距离越远，定位误差以该漂移系数逐渐增大；
	float loopKeyframeDisDiff = 20; // meters, distance between current and loop keyframe must more than n meters;

    bool bGNSSIsAdded = false;

    Eigen::Affine3f T_Drift;
	Eigen::Vector3f t_drift;
	Eigen::Quaternionf q_drift;

	cv::Mat MASK;

    typedef struct
	{
		int key_curr;
		int key_loop;
		PointTypePose keyPose6DCurr;
		PointTypePose keyPose6DLoop;
		Eigen::Vector3f t_loop_curr;
		Eigen::Quaternionf q_loop_curr;
		float noise;
	} loopInfo;

    std::map<int, loopInfo> loopInfoContainer;
	std::vector<std::pair<int, int> > currLoopKeyContainer;

    pcl::KdTreeFLANN<PointType> kdtreeSurroundingKeyPoses;
    pcl::KdTreeFLANN<PointType> kdtreeHistoryKeyPoses;
	pcl::VoxelGrid<PointType> downSizeFilterHeight;
    pcl::VoxelGrid<PointType> downSizeFilterIntensity;
    pcl::VoxelGrid<PointType> downSizeFilterICP;


	float globalMapVisualizationPoseDensity = 0.5;

	int loaded_map_size = 0;
	double deg2rad = M_PI / 180.0, rad2deg = 180.0 / M_PI;

	int BEV_rows = image_height/image_resolution;//行数
    int BEV_cols = image_length/image_resolution;//列数
	boost::shared_ptr<ORB_modify> orb;
	boost::shared_ptr<ORB_modify> orb_dowb;

    nav_msgs::Path laserAfterMappedPath;
	nav_msgs::Path aloam_robot_Path;
	nav_msgs::Path globalPoseGraphPath;

    std::ofstream f_save_pose;
	std::ofstream f_save_pose_evo;
	std::ofstream f_save_loop_id;
	std::ifstream f_read_pose;

	DBoW3::Database wordbag_loop;//词袋回环
    DBoW3::Vocabulary* voc;
	

	std::thread LaserMappingThread;
	std::thread PoseGraphThread;

    LaserMapping()
    {
        printf("--LaserMapping: line resolution: %.2f, plane resolution: %.2f \n", lineResolution, planeResolution);
        printf("--LaserMapping: LoopClosureEnable: %d \n", LoopClosureEnable);   
        printf("--LaserMapping: keyframeAddingDistance: %.1f \n", keyframeAddingDistance);
        printf("--LaserMapping: keyframeAddingAngle: %.1f \n", keyframeAddingAngle);
        printf("--LaserMapping: surroundingKeyframeSearchRadius: %.1f \n", surroundingKeyframeSearchRadius);
        printf("--LaserMapping: globalMapDensity: %.1f \n", globalMapDensity);
        printf("--LaserMapping: saveDirectory: %s \n", saveDirectory.c_str());
        printf("--LaserMapping: init_x %f \n", init_x);
        printf("--LaserMapping: init_y %f \n", init_y);
        printf("--LaserMapping: init_z %f \n", init_z);
        printf("--LaserMapping: init_yaw %f \n", init_yaw);
		orb.reset(new ORB_modify(nh));
		nh.setParam("orb_lio/nFeatures", 2500);
		orb_dowb.reset(new ORB_modify(nh));
		image_transport::ImageTransport in(nh); 
		voc = new DBoW3::Vocabulary(voc_path);
    	wordbag_loop.setVocabulary(*voc, false, 0);

        subLaserCloudHeight = nh.subscribe<sensor_msgs::PointCloud2>("/laserodometry/laser_cloud_height", 1000, &LaserMapping::laserCloudHeightHandler, this);
		subLaserCloudIntensity = nh.subscribe<sensor_msgs::PointCloud2>("/laserodometry/laser_cloud_intensity", 1000, &LaserMapping::laserCloudIntensityHandler, this);
		subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("/laserodometry/laser_odom_to_init", 1000, &LaserMapping::laserOdometryHandler, this);

		subheightBEV = in.subscribe("/laserodometry/heightBEV", 1000, &LaserMapping::heightBEVCallback, this);
		subtime = nh.subscribe<sensor_msgs::PointCloud2>(pointCloudTopic, 1000, &LaserMapping::timeHandler, this);
        // subintensityBEV = in.subscribe("/laserodometry/intensityBEV", 10, &LaserMapping::intensityBEVCallback, this);

        pubLaserCloudSurround = nh.advertise<sensor_msgs::PointCloud2>("/lasermapping/laser_cloud_surround", 10);
		pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>("/lasermapping/laser_cloud_map", 10);
		pubScanCloudMap = nh.advertise<sensor_msgs::PointCloud2>("/lasermapping/laser_cloud_map_scan", 10);

        pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>("/lasermapping/aft_mapped_to_init", 10);
		pubOdomAftMappedHighFrec = nh.advertise<nav_msgs::Odometry>("/lasermapping/odom", 10);
		pubLaserAfterMappedPath = nh.advertise<nav_msgs::Path>("/lasermapping/aft_mapped_path", 10);
		pubLoamPath = nh.advertise<nav_msgs::Path>("/lasermapping/loam_path", 10);
		pubaLoamRobotPath = nh.advertise<nav_msgs::Path>("/lasermapping/aloam_robot_path", 10);
		pubGlobalPoseGraphPath = nh.advertise<nav_msgs::Path>("/lasermapping/global_pose_graph_path", 10);

		pubheightBEV = in.advertise("/lasermapping/match_BEV",10);



		pubMarker = nh.advertise<visualization_msgs::Marker>("/lasermapping/robot_marker", 10);
		pubLoopConstraintEdge = nh.advertise<visualization_msgs::MarkerArray>("/lasermapping/loop_closure_constraints", 10);
		pubGlobalPoseGraphPoint = nh.advertise<sensor_msgs::PointCloud2>("/lasermapping/global_pose_graph_point", 10);
		pubGlobalMapKeyPose = nh.advertise<sensor_msgs::PointCloud2>("/lasermapping/global_map_keyPose", 10);
		pubGlobalMapKeyPoseDS = nh.advertise<sensor_msgs::PointCloud2>("/lasermapping/global_map_keyPoseDS", 10);
		pubRobotPose = nh.advertise<geometry_msgs::PoseStamped>("/lasermapping/robot_pose", 10);
		pubtempcloud = nh.advertise<sensor_msgs::PointCloud2>("/lasermapping/temp_cloud", 10);

		BEV_rows = image_height/image_resolution;
        BEV_cols = image_length/image_resolution;

		MASK = cv::Mat(BEV_rows, BEV_cols, CV_8UC1, cv::Scalar(255));
		for (int i = 0; i < BEV_rows; ++i)
			for (int j = 0; j < BEV_cols; ++j)
				if (j < 256 || j > BEV_cols - 256)
					MASK.at<uchar>(i,j) = 0;

        BEV_now.BEV_image = cv::Mat::zeros(BEV_rows, BEV_cols, CV_8UC1);
        BEV_last.BEV_image = cv::Mat::zeros(BEV_rows, BEV_cols, CV_8UC1);

        R_il = Utility::ypr2R(Eigen::Vector3d(-1.29, -0.15, 0.65)); // 将IMU和激光雷达与重力对齐；
		// Eigen::Quaterniond qq_li(0.999479, -0.000357, 0.0194128, 0.0257984); // 将IMU和激光雷达与重力齐；
		// R_il = qq_li.inverse();
		t_rl = Eigen::Vector3d(0.68, 0, 0.34);
		R_rl = Utility::ypr2R(Eigen::Vector3d(0.0, 0.0, -0.0));
		T_rl = Eigen::Matrix4d::Identity();
		T_rl.block<3, 3>(0, 0) = R_rl;
		T_rl.block<3, 1>(0, 3) = t_rl;

        allocateMemory();

        downSizeFilterHeight.setLeafSize(lineResolution, lineResolution,lineResolution);
		downSizeFilterIntensity.setLeafSize(lineResolution, lineResolution, lineResolution);
		downSizeFilterICP.setLeafSize(planeResolution, planeResolution, planeResolution);

        boost::format fmt_pose("%s/%s");
		f_read_pose.open((fmt_pose % saveDirectory % "/pose.txt").str(), std::fstream::in);
        f_save_pose.open((fmt_pose % saveDirectory % "/pose.txt").str(), std::fstream::out);
		f_save_pose_evo.open((fmt_pose % saveDirectory % "/pose_evo.txt").str(), std::fstream::out);
		f_save_loop_id.open((fmt_pose % saveDirectory % "/loop_id.txt").str(), std::fstream::out);

        LaserMappingThread = std::thread(&LaserMapping::LaserMapping_thread, this);
		PoseGraphThread = std::thread(&LaserMapping::poseGraphOptimizationThread, this);


    }
    ~LaserMapping()
    {
		saveKeyPoseToFileAsTUM();
		f_save_pose.close();
		f_save_pose_evo.close();
		f_save_loop_id.close();
        printf("--LaserMapping exit !!! \n");
    }

	void timeHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudHeightMsg)
	{
		if(start_flag == 1)
		{
			start_time = laserCloudHeightMsg->header.stamp.toSec();
			start_flag = 0;
		}
	}

	void saveKeyPoseToFileAsTUM()
	{
		// 将机器人位姿按照TUM数据集格式写入文件；

		for (auto it = currLoopKeyContainer.begin(); it != currLoopKeyContainer.end(); ++it)
        {
            int loopNewKeyID = it->first;
            int loopOldKeyID = it->second;
			PointTypePose loopNewPose6D, loopOldPose6D;

			auto itNew = KeyPose6D.find(loopNewKeyID);
			if (itNew != KeyPose6D.end())
			{
				loopNewPose6D = KeyPose6D.at(loopNewKeyID);
			}
			else
			{
				continue;
			}

			auto itOld = KeyPose6D.find(loopOldKeyID);
			if (itOld != KeyPose6D.end())
			{
				loopOldPose6D = KeyPose6D.at(loopOldKeyID);
			}
			else
			{
				continue;
			}

			f_save_loop_id << std::fixed << loopOldKeyID << " " << loopOldPose6D.x << " " << loopOldPose6D.y << " " << loopOldPose6D.z << std::endl;
			f_save_loop_id << std::fixed << loopNewKeyID << " " << loopNewPose6D.x << " " << loopNewPose6D.y << " " << loopNewPose6D.z << std::endl;
		}
		
		int keySize = cloudKeyPoses6D->size();
		for (int i = 0; i < keySize; ++i)
		{
			PointTypePose KeyPose6D = cloudKeyPoses6D->points[i];
			if (map_update == 1)
			{
				f_save_pose << std::fixed << KeyPose6D.intensity << " " << KeyPose6D.x << " " << KeyPose6D.y << " " << KeyPose6D.z << " " 
						    << KeyPose6D.yaw << " " << KeyPose6D.pitch << " " << KeyPose6D.roll << " " << KeyPose6D.time << " " 
							<< travel_distance[i] << " " << travel_angle[i] << std::endl;
				// tf::Quaternion q = tf::createQuaternionFromRPY(KeyPose6D.roll, KeyPose6D.pitch, KeyPose6D.yaw);
				// f_save_pose_evo << std::fixed << KeyPose6D.time << " " << KeyPose6D.x << " " << KeyPose6D.y << " " << KeyPose6D.z << " " 
				// 		    << q.x() << " " << q.y()<< " " << q.z() << " " << q.w() << std::endl;
			}
		}
		for (int i = 0; i<globalPoseGraphPath.poses.size();i++)
		{
			if (map_update == 1)
			{
				f_save_pose_evo << std::fixed << std::setprecision(6) << globalPoseGraphPath.poses[i].header.stamp.toSec() - start_time<< " " <<  std::setprecision(9) << globalPoseGraphPath.poses[i].pose.position.x << " " << globalPoseGraphPath.poses[i].pose.position.y << " " << globalPoseGraphPath.poses[i].pose.position.z << " " 
						    << globalPoseGraphPath.poses[i].pose.orientation.x << " " << globalPoseGraphPath.poses[i].pose.orientation.y << " " << globalPoseGraphPath.poses[i].pose.orientation.z << " " << globalPoseGraphPath.poses[i].pose.orientation.w << std::endl;
			}
		}
	}

    void allocateMemory()
    {
		laserCloudHeight.reset(new pcl::PointCloud<PointType>());
        laserCloudIntensity.reset(new pcl::PointCloud<PointType>());

        laserCloudHeightDS.reset(new pcl::PointCloud<PointType>());
        laserCloudIntensityDS.reset(new pcl::PointCloud<PointType>());
		laserCloudHeightLastDS.reset(new pcl::PointCloud<PointType>());
        laserCloudIntensityLastDS.reset(new pcl::PointCloud<PointType>());
		
		surroundingMapDS.reset(new pcl::PointCloud<PointType>());
		laserCloudHeightFromMap.reset(new pcl::PointCloud<PointType>());
		laserCloudHeightFromMapDS.reset(new pcl::PointCloud<PointType>());
        laserCloudIntensityFromMap.reset(new pcl::PointCloud<PointType>());
        laserCloudIntensityFromMapDS.reset(new pcl::PointCloud<PointType>());

        cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
        cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());
        copy_cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
        copy_cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());
		
		latestKeyFrameCloud.reset(new pcl::PointCloud<PointType>());
        nearHistoryKeyFrameCloud.reset(new pcl::PointCloud<PointType>());

	}
    // 回调函数
	void heightBEVCallback(const sensor_msgs::ImageConstPtr& heightmsg)
    {
        mBuf.lock();
        heightBEVBuf.push(heightmsg);
        mBuf.unlock();
    }

    void intensityBEVCallback(const sensor_msgs::ImageConstPtr& intensitymsg)
    {
        mBuf.lock();
        intensityBEVBuf.push(intensitymsg);
        mBuf.unlock();
    }

    void laserCloudHeightHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudHeightMsg)
	{
		mBuf.lock();
		laserHeightBuf.push(laserCloudHeightMsg);
		mBuf.unlock();
	}

	void laserCloudIntensityHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudIntensityMsg)
	{
		mBuf.lock();
		laserIntensityBuf.push(laserCloudIntensityMsg);
		mBuf.unlock();
	}

    void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr &laserOdometry)
	{
		mBuf.lock();
		odometryBuf.push(laserOdometry);
		mBuf.unlock();

		laserOdom_count++;

		// high frequence publish
		Eigen::Quaterniond q_wodom_curr;
		Eigen::Vector3d t_wodom_curr;
		q_wodom_curr.x() = laserOdometry->pose.pose.orientation.x;
		q_wodom_curr.y() = laserOdometry->pose.pose.orientation.y;
		q_wodom_curr.z() = laserOdometry->pose.pose.orientation.z;
		q_wodom_curr.w() = laserOdometry->pose.pose.orientation.w;
		t_wodom_curr.x() = laserOdometry->pose.pose.position.x;
		t_wodom_curr.y() = laserOdometry->pose.pose.position.y;
		t_wodom_curr.z() = laserOdometry->pose.pose.position.z;

		Eigen::Quaterniond q_w_new = q_wmap_wodom * q_wodom_curr;
		Eigen::Vector3d t_w_new = q_wmap_wodom * t_wodom_curr + t_wmap_wodom;

		nav_msgs::Odometry odomAftMapped;
		odomAftMapped.header.frame_id = "camera_init";
		odomAftMapped.child_frame_id = "aft_mapped";
		odomAftMapped.header.stamp = laserOdometry->header.stamp;
		odomAftMapped.pose.pose.orientation.x = q_w_new.x();
		odomAftMapped.pose.pose.orientation.y = q_w_new.y();
		odomAftMapped.pose.pose.orientation.z = q_w_new.z();
		odomAftMapped.pose.pose.orientation.w = q_w_new.w();
		odomAftMapped.pose.pose.position.x = t_w_new.x();
		odomAftMapped.pose.pose.position.y = t_w_new.y();
		odomAftMapped.pose.pose.position.z = t_w_new.z();
		pubOdomAftMappedHighFrec.publish(odomAftMapped);

		// if (USE_IMU)
		// {
		// 	printf("--IMU_ypr_%d: %.3f  %.3f  %.3f\n", IMU.count, IMU.yaw*57.3, IMU.pitch*57.3, IMU.roll*57.3);
		// }

		// printf("--Mapping: height:%d, intensity:%d; map-height:%d, intensity:%d; KF:%d; cost:%.1f \n",
		// laserCloudHeightDSNum, laserCloudIntensityDSNum, laserCloudHeightFromMapDSNum, laserCloudIntensityFromMapDSNum, 
		// keyFrameNum, cost_mapping);

		// lidar to robot frame
		Eigen::Vector3d ypr;
		Eigen::Matrix3d r_robot;
		Eigen::Matrix4d T_lk = Eigen::Matrix4d::Identity();
		Eigen::Matrix4d T_rk = Eigen::Matrix4d::Identity();
		T_lk.block<3, 1>(0, 3) = t_w_new;
		T_lk.block<3, 3>(0, 0) = Eigen::Matrix3d(q_w_new);
		T_rk = T_rl * T_lk * T_rl.inverse();
		t_robot = T_rk.block<3, 1>(0, 3);
		r_robot = T_rk.block<3, 3>(0, 0);
		q_robot = Eigen::Quaterniond(r_robot);

		// T_lk.block<3, 1>(0, 3) = t_wodom_curr;
		// T_lk.block<3, 3>(0, 0) = Eigen::Matrix3d(q_wodom_curr);
		// T_rk = T_rl * T_lk * T_rl.inverse();
		// Eigen::Vector3d t_odom_robot = T_rk.block<3, 1>(0, 3);
		// Eigen::Quaterniond q_odom_robot = Eigen::Quaterniond(T_rk.block<3, 3>(0, 0));

		// // 2021.06.19
		// ypr = Utility::R2ypr(Eigen::Matrix3d(q_w_new));
		// printf("--lidar_pose_%d : x:%.3f y:%.3f z:%.3f , y:%.2f p:%.2f r:%.2f, time:%.1f \n",
		// 	mapping_count, t_w_new(0),t_w_new(1),t_w_new(2),  ypr(0), ypr(1), ypr(2), cost_mapping);

		// ypr = Utility::R2ypr(r_robot);
		// printf("--robot_pose_%d : x:%.3f y:%.3f z:%.3f , y:%.2f p:%.2f r:%.2f \n\n",
		// 	mapping_count, t_robot(0),t_robot(1),t_robot(2),  ypr(0), ypr(1), ypr(2));
		// // 2021.06.19

		// ypr = Utility::R2ypr(Eigen::Matrix3d(q_wodom_curr));
		// printf("--odom_lidar_%d : x:%.3f y:%.3f z:%.3f , y:%.2f p:%.2f r:%.2f \n",
		// 	laserOdom_count, t_wodom_curr(0),t_wodom_curr(1),t_wodom_curr(2),  ypr(0), ypr(1), ypr(2));

		// ypr = Utility::R2ypr(Eigen::Matrix3d(q_odom_robot));
		// printf("--odom_robot_%d : x:%.3f y:%.3f z:%.3f , y:%.2f p:%.2f r:%.2f \n",
		// 	laserOdom_count, t_odom_robot(0),t_odom_robot(1),t_odom_robot(2),  ypr(0), ypr(1), ypr(2));

		static tf::TransformBroadcaster br_mapped;
		tf::Transform transform;
		tf::Quaternion q;
		transform.setOrigin(tf::Vector3(t_w_new(0),t_w_new(1),t_w_new(2)));
		q.setW(q_w_new.w());
		q.setX(q_w_new.x());
		q.setY(q_w_new.y());
		q.setZ(q_w_new.z());
		transform.setRotation(q);
		br_mapped.sendTransform(tf::StampedTransform(transform, laserOdometry->header.stamp, "camera_init", "aft_mapped"));

		if (1)
		{
			static tf::TransformBroadcaster br_laser_to_world;
			tf::Transform transform_laser_to_world;
			tf::Quaternion q_laser_to_world;
			transform_laser_to_world.setOrigin(tf::Vector3(0,0,0));
			q_laser_to_world.setW(1.0);
			q_laser_to_world.setX(0);
			q_laser_to_world.setY(0);
			q_laser_to_world.setZ(0);
			transform_laser_to_world.setRotation(q_laser_to_world);
			br_laser_to_world.sendTransform(tf::StampedTransform(transform_laser_to_world, laserOdometry->header.stamp, "world", "camera_init"));

			static tf::TransformBroadcaster br_world_to_odom;
			tf::Transform transform_world_to_odom;
			tf::Quaternion q_world_to_odom;
			transform_world_to_odom.setOrigin(tf::Vector3(0,0,0));
			q_world_to_odom.setW(1.0);
			q_world_to_odom.setX(0);
			q_world_to_odom.setY(0);
			q_world_to_odom.setZ(0);
			transform_world_to_odom.setRotation(q_world_to_odom);
			br_world_to_odom.sendTransform(tf::StampedTransform(transform_world_to_odom, laserOdometry->header.stamp, "odom", "world"));

			static tf::TransformBroadcaster br_odom_to_map;
			tf::Transform transform_odom_to_map;
			tf::Quaternion q_odom_to_map;
			transform_odom_to_map.setOrigin(tf::Vector3(0,0,0));
			q_odom_to_map.setW(1.0);
			q_odom_to_map.setX(0);
			q_odom_to_map.setY(0);
			q_odom_to_map.setZ(0);
			transform_odom_to_map.setRotation(q_odom_to_map);
			br_odom_to_map.sendTransform(tf::StampedTransform(transform_odom_to_map, laserOdometry->header.stamp, "map", "odom"));


			static tf::TransformBroadcaster br_mapping_to_robot;
			tf::Transform transform_mapping_to_robot;
			tf::Quaternion q_mapping_to_robot;
			transform_mapping_to_robot.setOrigin(tf::Vector3(0,0,0));
			q_mapping_to_robot.setW(1.0);
			q_mapping_to_robot.setX(0);
			q_mapping_to_robot.setY(0);
			q_mapping_to_robot.setZ(0);
			transform_mapping_to_robot.setRotation(q_mapping_to_robot);
			br_mapping_to_robot.sendTransform(tf::StampedTransform(transform_mapping_to_robot, laserOdometry->header.stamp, "aft_mapped", "robot"));

			static tf::TransformBroadcaster br_base_to_robot;
			tf::Transform transform_base_to_robot;
			tf::Quaternion q_base_to_robot;
			transform_base_to_robot.setOrigin(tf::Vector3(0,0,0));
			q_base_to_robot.setW(1.0);
			q_base_to_robot.setX(0);
			q_base_to_robot.setY(0);
			q_base_to_robot.setZ(0);
			transform_base_to_robot.setRotation(q_base_to_robot);
			br_base_to_robot.sendTransform(tf::StampedTransform(transform_base_to_robot, laserOdometry->header.stamp, "robot", "base_link"));

			static tf::TransformBroadcaster br_velodyne_to_robot;
			tf::Transform transform_velodyne_to_robot;
			tf::Quaternion q_velodyne_to_robot;
			transform_velodyne_to_robot.setOrigin(tf::Vector3(0,0,0));
			q_velodyne_to_robot.setW(1.0);
			q_velodyne_to_robot.setX(0);
			q_velodyne_to_robot.setY(0);
			q_velodyne_to_robot.setZ(0);
			transform_velodyne_to_robot.setRotation(q_velodyne_to_robot);
			br_velodyne_to_robot.sendTransform(tf::StampedTransform(transform_velodyne_to_robot, laserOdometry->header.stamp, "base_link", "velodyne"));

			static tf::TransformBroadcaster br_laser_to_velodyne;
			tf::Transform transform_laser_to_velodyne;
			tf::Quaternion q_laser_to_velodyne;
			transform_laser_to_velodyne.setOrigin(tf::Vector3(0,0,0));
			q_laser_to_velodyne.setW(1.0);
			q_laser_to_velodyne.setX(0);
			q_laser_to_velodyne.setY(0);
			q_laser_to_velodyne.setZ(0);
			transform_laser_to_velodyne.setRotation(q_laser_to_velodyne);
			br_laser_to_velodyne.sendTransform(tf::StampedTransform(transform_laser_to_velodyne, laserOdometry->header.stamp, "velodyne", "base_laser"));

			static tf::TransformBroadcaster br_base_to_footprint;
			tf::Transform transform_base_to_footprint;
			tf::Quaternion q_base_to_footprint;
			transform_base_to_footprint.setOrigin(tf::Vector3(0,0,0));
			q_base_to_footprint.setW(1.0);
			q_base_to_footprint.setX(0);
			q_base_to_footprint.setY(0);
			q_base_to_footprint.setZ(0);
			transform_base_to_footprint.setRotation(q_base_to_footprint);
			br_base_to_footprint.sendTransform(tf::StampedTransform(transform_base_to_footprint, laserOdometry->header.stamp, "base_link", "base_footprint"));
		}

		geometry_msgs::PoseStamped laserAfterMappedPose;
		laserAfterMappedPose.header.stamp = laserOdometry->header.stamp;
		laserAfterMappedPose.header.frame_id = "camera_init";
		laserAfterMappedPose.pose.orientation.x = q_w_new.x();
		laserAfterMappedPose.pose.orientation.y = q_w_new.y();
		laserAfterMappedPose.pose.orientation.z = q_w_new.z();
		laserAfterMappedPose.pose.orientation.w = q_w_new.w();
		laserAfterMappedPose.pose.position.x = t_w_new.x();
		laserAfterMappedPose.pose.position.y = t_w_new.y();
		laserAfterMappedPose.pose.position.z = t_w_new.z();
		laserAfterMappedPath.header.stamp = laserOdometry->header.stamp;
		laserAfterMappedPath.header.frame_id = "camera_init";
		laserAfterMappedPath.poses.push_back(laserAfterMappedPose);
		pubLaserAfterMappedPath.publish(laserAfterMappedPath);
		
		geometry_msgs::PoseStamped RobotPose;
		RobotPose.header.stamp = laserOdometry->header.stamp;
		RobotPose.header.frame_id = "world";
		RobotPose.pose.orientation.x = q_robot.x();
		RobotPose.pose.orientation.y = q_robot.y();
		RobotPose.pose.orientation.z = q_robot.z();
		RobotPose.pose.orientation.w = q_robot.w();
		RobotPose.pose.position.x = t_robot.x();
		RobotPose.pose.position.y = t_robot.y();
		RobotPose.pose.position.z = t_robot.z();
		aloam_robot_Path.header.stamp = laserOdometry->header.stamp;
		aloam_robot_Path.header.frame_id = "world";
		aloam_robot_Path.poses.push_back(RobotPose);
		pubaLoamRobotPath.publish(aloam_robot_Path);

		pubRobotPose.publish(RobotPose);

		PubRobotMarker(t_w_new, q_w_new);
		// PubRobotMarker(t_w_new, q_w_new);
	}

    void PubRobotMarker(Eigen::Vector3d p, Eigen::Quaterniond q)
    {
        // uint8 ARROW=0
        // uint8 CUBE=1
        // uint8 SPHERE=2
        // uint8 CYLINDER=3
        // uint8 LINE_STRIP=4
        // uint8 LINE_LIST=5
        // uint8 CUBE_LIST=6
        // uint8 SPHERE_LIST=7
        // uint8 POINTS=8
        // uint8 TEXT_VIEW_FACING=9
        // uint8 MESH_RESOURCE=10
        // uint8 TRIANGLE_LIST=11

        visualization_msgs::Marker tempMarker;
        tempMarker.id = 0;

        tempMarker.header.frame_id = "world";
        tempMarker.header.stamp = ros::Time::now();
        tempMarker.id = 0;
        tempMarker.ns = "GR-loam";
        tempMarker.type = visualization_msgs::Marker::CUBE;
        tempMarker.action = visualization_msgs::Marker::ADD;

        tempMarker.pose.position.x = p(0);
        tempMarker.pose.position.y = p(1);
        tempMarker.pose.position.z = p(2);
        tempMarker.pose.orientation.x = q.x();
        tempMarker.pose.orientation.y = q.y();
        tempMarker.pose.orientation.z = q.z();
        tempMarker.pose.orientation.w = q.w();

        tempMarker.scale.x = 1.2;
        tempMarker.scale.y = 0.7;
        tempMarker.scale.z = 0.5;
 
        tempMarker.color.r = 1.0f;
        tempMarker.color.g = 0.0f;
        tempMarker.color.b = 0.0f;
        tempMarker.color.a = 1.0;
 
        tempMarker.lifetime = ros::Duration();

        pubMarker.publish(tempMarker);
    }

    void LaserMapping_thread()
	{
		printf("--LaserMapping_thread begain !!! \n");

		while(1)
		{
			while (!laserHeightBuf.empty() && !laserIntensityBuf.empty() && !heightBEVBuf.empty() && !odometryBuf.empty())
			{
				mBuf.lock();

				while (!odometryBuf.empty() && odometryBuf.front()->header.stamp.toSec() < laserHeightBuf.front()->header.stamp.toSec())
						odometryBuf.pop();
				if (odometryBuf.empty())
				{
					mBuf.unlock();
					break;
				}

				while (!laserIntensityBuf.empty() && laserIntensityBuf.front()->header.stamp.toSec() < laserHeightBuf.front()->header.stamp.toSec())
						laserIntensityBuf.pop();
				if (laserIntensityBuf.empty())
				{
					mBuf.unlock();
					break;
				}
				
				while (!heightBEVBuf.empty() && heightBEVBuf.front()->header.stamp.toSec() < laserHeightBuf.front()->header.stamp.toSec())
						heightBEVBuf.pop();
				if (heightBEVBuf.empty())
				{
					mBuf.unlock();
					break;
				}

				timeLaserCloudHeight = laserHeightBuf.front()->header.stamp.toSec();
				timeLaserCloudIntensity = laserIntensityBuf.front()->header.stamp.toSec();
				timeLaserOdometry = odometryBuf.front()->header.stamp.toSec();
				timeimage = heightBEVBuf.front()->header.stamp.toSec();
				if (timeLaserCloudHeight != timeLaserOdometry || timeLaserCloudIntensity != timeLaserOdometry || timeimage != timeLaserOdometry)
				{
					printf("time height %f intensity %f odom %f \n", timeLaserCloudHeight, timeLaserCloudIntensity, timeLaserOdometry);
					printf("--LaserMapping_thread: unsync messeage!");
					mBuf.unlock();
					break;
				}

				TicToc t_whole;

				curTime = timeLaserOdometry;	
				if (laserHeightBuf.size() < initfirst) // 缓存一帧激光数据，使得imu数据完全充满上一帧；
				{
					mBuf.unlock();
					std::chrono::milliseconds time(10);
					std::this_thread::sleep_for(time);
					break;	
				}
				static int first_flag = 0;
				if (first_flag < initfirst)
				{
					first_flag++;
					laserHeightBuf.pop(); // 丢掉第一帧；
					prevTime = curTime;

					mBuf.unlock();
					std::chrono::milliseconds time(10);
					std::this_thread::sleep_for(time);
					break;
				}

				laserCloudHeight->clear();
				pcl::fromROSMsg(*laserHeightBuf.front(), *laserCloudHeight);
				laserHeightBuf.pop();

				laserCloudIntensity->clear();
				pcl::fromROSMsg(*laserIntensityBuf.front(), *laserCloudIntensity);
				laserIntensityBuf.pop();

				q_wodom_curr.x() = odometryBuf.front()->pose.pose.orientation.x;
				q_wodom_curr.y() = odometryBuf.front()->pose.pose.orientation.y;
				q_wodom_curr.z() = odometryBuf.front()->pose.pose.orientation.z;
				q_wodom_curr.w() = odometryBuf.front()->pose.pose.orientation.w;
				t_wodom_curr.x() = odometryBuf.front()->pose.pose.position.x;
				t_wodom_curr.y() = odometryBuf.front()->pose.pose.position.y;
				t_wodom_curr.z() = odometryBuf.front()->pose.pose.position.z;
				odometryBuf.pop();

				BEV_now.BEV_image =  cv_bridge::toCvShare(heightBEVBuf.front(), "mono8")->image;
				cv::cvtColor(BEV_now.BEV_image, BEV_now.BEV_RGB, cv::COLOR_GRAY2RGB);
				heightBEVBuf.pop();

				while (laserHeightBuf.size() > 2)
				{
					laserHeightBuf.pop();
					printf("--laserMapping: drop lidar frame in mapping for real time !!! \n\n\n");
				}

				mBuf.unlock();

				TicToc t_prepareMap;
				orb->ORB_feature(BEV_now.BEV_image);
                BEV_now.keypoints = orb->mvKeys;
                BEV_now.description = orb->mDescriptors;
				orb_dowb->ORB_feature(BEV_now.BEV_image);
                BEV_now.keypoints_dowb = orb_dowb->mvKeys;
                BEV_now.description_dowb = orb_dowb->mDescriptors;				

				extractSurroundingKeyFramesAndMap();//构建局部关键帧和地图

				downsampleCurrentLaserCloud();

				printf("--laserMapping: height: %d,  intensity: %d \n", laserCloudHeightDSNum, laserCloudIntensityDSNum);

				transformAssociateToMap(); // set initial guess from laserOdometry;

				if(laserCloudHeightDSNum > 100 && laserCloudIntensityDSNum>100 && laserCloudHeightFromMapDSNum > 100 && laserCloudIntensityFromMapDSNum > 100)
				{
					TicToc t_opt;
					//pre 
					Eigen::Matrix4f T2;
					T2.setIdentity();
					T2.block<3,3>(0,0) = (q_w_curr.toRotationMatrix()).cast<float>();
					T2.topRightCorner(3, 1) = t_w_curr.cast<float>();
					//define vgicp
					fast_gicp::FastVGICP<pcl::PointXYZI, pcl::PointXYZI> vgicp;
					pcl::PointCloud<pcl::PointXYZI>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZI>);
					vgicp.setResolution(down_simple_vgicp);
					vgicp.setMaximumIterations(25);
					// vgicp.setMaxCorrespondenceDistance(1);
					vgicp.setTransformationEpsilon(1e-6);//收敛条件 两次迭代的转换矩阵的最大容差
					vgicp.setEuclideanFitnessEpsilon(1e-6);//收敛条件 均方误差和小于阈值
					vgicp.setRANSACIterations(0);// 设置RANSAC运行次数
					vgicp.setNumThreads(14);		
					pcl::PointCloud<PointType>::Ptr vgicp_input_target(new pcl::PointCloud<PointType>);
					pcl::PointCloud<PointType>::Ptr vgicp_input_source(new pcl::PointCloud<PointType>);
					*vgicp_input_target = *laserCloudHeightFromMapDS + *laserCloudIntensityFromMapDS;
					*vgicp_input_source = *laserCloudHeightDS + *laserCloudIntensityDS;

					// //source downfilter
                    // pcl::PointCloud<PointType> vgicp_input_sourceDS;
                    // pcl::VoxelGrid<PointType> downSizeFilterFull;
                    // pcl::PointCloud<PointType>::Ptr FullPointsLessFlat(new pcl::PointCloud<PointType>);
                    // downSizeFilterFull.setLeafSize(0.2, 0.2, 0.2);
                    // downSizeFilterFull.setInputCloud(vgicp_input_source);
                    // downSizeFilterFull.filter(vgicp_input_sourceDS);
                    // *FullPointsLessFlat+=vgicp_input_sourceDS;
                    // //target downfilter
                    // pcl::PointCloud<PointType> vgicp_input_targetDS;
                    // pcl::VoxelGrid<PointType> downSizeFilterFulllast;
                    // pcl::PointCloud<PointType>::Ptr FullPointsLessFlatlast(new pcl::PointCloud<PointType>);
                    // downSizeFilterFulllast.setLeafSize(0.3, 0.3, 0.3);
                    // downSizeFilterFulllast.setInputCloud(vgicp_input_target);
                    // downSizeFilterFulllast.filter(vgicp_input_targetDS);
                    // *FullPointsLessFlatlast+=vgicp_input_targetDS;
					// for(int i = 0; i < vgicp_input_source->size(); i++)
					// {
					// 	TransformToWorld(&(vgicp_input_source->points[i]), &(vgicp_input_source->points[i]));
					// }
					// sensor_msgs::PointCloud2 tempcloudMsg;
					// pcl::toROSMsg(*vgicp_input_target, tempcloudMsg);
					// tempcloudMsg.header.stamp = ros::Time().fromSec(timeLaserOdometry);
					// tempcloudMsg.header.frame_id = "camera_init";
					// pubtempcloud.publish(tempcloudMsg);

					vgicp.setInputTarget(vgicp_input_target);
					vgicp.setInputSource(vgicp_input_source);	
					vgicp.align(*aligned, T2);	
					Eigen::Affine3f T_match;
					Eigen::Vector3f t_match;
					Eigen::Quaternionf q_match;
					T_match = vgicp.getFinalTransformation();
					t_match = T_match.translation();
					q_match = T_match.rotation();
					
					t_w_curr =  t_match.cast<double>();
					q_w_curr = Eigen::Quaterniond(q_match).cast<double>();


				}
				else
				{
					if (laserCloudHeightDSNum < 100 || laserCloudIntensityDSNum < 100)
					{
						
						ROS_WARN("--laserMapping: laserCloud height and intensity num are not enough!!!");
					}
					if (laserCloudHeightFromMapDSNum < 100 || laserCloudIntensityFromMapDSNum < 100)
					{
						ROS_WARN("--laserMapping: map height and intensity num are not enough!!!");
					}
				}
				// q_w_last.normalize();
				q_w_curr.normalize();
				static int frameCount = 0;
				if (frameCount < initskipframe)
				{
					t_w_curr = Eigen::Vector3d::Zero();
					q_w_curr = Eigen::Quaterniond::Identity();

				}
				frameCount++;
				transformUpdate(); // 求解最新的地图坐标系下的位姿后，更新地图与里程计坐标之间的变换；
				Eigen::Vector3d ypr_last = Utility::R2ypr(Eigen::Matrix3d(q_w_last)) * deg2rad;
				saveKeyframeAndOdomFactor(t_w_last, ypr_last, prevTime);
				q_w_last = q_w_curr;
				t_w_last = t_w_curr;
				BEV_last = BEV_now;
				prevTime = curTime;

				if (frameCount % 2 == 0)
				{
					surroundingMapDS->clear();
					*surroundingMapDS += *laserCloudHeightFromMapDS;
					*surroundingMapDS += *laserCloudIntensityFromMapDS;

					sensor_msgs::PointCloud2 laserCloudSurroundMsg;
					pcl::toROSMsg(*surroundingMapDS, laserCloudSurroundMsg);
					laserCloudSurroundMsg.header.stamp = ros::Time().fromSec(timeLaserOdometry);
					laserCloudSurroundMsg.header.frame_id = "camera_init";
					pubLaserCloudSurround.publish(laserCloudSurroundMsg);
				}
				mapping_count++;

				nav_msgs::Odometry odomAftMapped;
				odomAftMapped.header.frame_id = "camera_init";
				odomAftMapped.child_frame_id = "aft_mapped";
				odomAftMapped.header.stamp = ros::Time().fromSec(timeLaserOdometry);
				odomAftMapped.pose.pose.orientation.x = q_w_curr.x();
				odomAftMapped.pose.pose.orientation.y = q_w_curr.y();
				odomAftMapped.pose.pose.orientation.z = q_w_curr.z();
				odomAftMapped.pose.pose.orientation.w = q_w_curr.w();
				odomAftMapped.pose.pose.position.x = t_w_curr.x();
				odomAftMapped.pose.pose.position.y = t_w_curr.y();
				odomAftMapped.pose.pose.position.z = t_w_curr.z();
				pubOdomAftMapped.publish(odomAftMapped);

				cost_mapping = t_whole.toc();
				printf("--lasermapping: whole mapping time: %.1f ms \n", cost_mapping);
			}
			std::chrono::milliseconds dura(2);
			std::this_thread::sleep_for(dura);
		}

    }

	void extractSurroundingKeyFramesAndMap()
    {
		if (cloudKeyPoses3D->points.empty() == true)
            return; 
		// 全局优化后更新最新的位姿，而且需要对优化期间新添加的关键帧进行矫正，维护局部一致性后才能进行地图构建优化；
		if (bKeyFramePoseGraphUpdated == true) 
		{
			// 对全局优化后的所有关键帧位姿进行更新；

			correctKeyFramePoseGraph();

			// 还要维护局部一致性;
			q_w_last = q_drift.cast<double>() * q_w_last;
			t_w_last = q_drift.cast<double>() * t_w_last + t_drift.cast<double>();
			q_w_last.normalize();
			
			q_wmap_wodom = q_drift.cast<double>() * q_wmap_wodom;
			t_wmap_wodom = q_drift.cast<double>() * t_wmap_wodom + t_drift.cast<double>();
			q_wmap_wodom.normalize();

			bKeyFramePoseGraphUpdated = false;
		}
        PointType currPos;
		currPos.x = t_w_curr.x();
		currPos.y = t_w_curr.y();
		currPos.z = t_w_curr.z();
		currPos.intensity = 0;

		pcl::PointCloud<PointType>::Ptr surroundingKeyPoses(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr surroundingKeyPosesDS(new pcl::PointCloud<PointType>());

		std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;

		

		// extract all the nearby key poses and downsample them
		kdtreeSurroundingKeyPoses.setInputCloud(cloudKeyPoses3D); // create kd-tree

		kdtreeSurroundingKeyPoses.radiusSearch(currPos, (double)surroundingKeyframeSearchRadius, pointSearchInd, pointSearchSqDis);

		for (int i = 0; i < (int)pointSearchInd.size(); ++i)
        {
			surroundingKeyPoses->push_back(cloudKeyPoses3D->points[pointSearchInd[i]]);
        }

		// in each voxel, all the points present will be approximated with the closest point to the center of the voxel.
		//pcl::PointCloud<int> keypointIndices;
		pcl::UniformSampling<PointType> UniformSamplingSurroundingKeyPoses;
		UniformSamplingSurroundingKeyPoses.setInputCloud(surroundingKeyPoses);
		UniformSamplingSurroundingKeyPoses.setRadiusSearch(surroundingKeyframeDensity);
		UniformSamplingSurroundingKeyPoses.filter(*surroundingKeyPosesDS); 

        //pcl::copyPointCloud(*surroundingKeyPoses, keypointIndices.points, *surroundingKeyPosesDS);
		int numsurroundingKeyPosesDS = surroundingKeyPosesDS->points.size();
		for (int i = 0; i < (int)surroundingExistingKeyPoseID.size(); ++i)
		{
			bool existingflag = false;
			for (int j = 0; j < numsurroundingKeyPosesDS; ++j)
			{
				if (surroundingExistingKeyPoseID[i] == (int)surroundingKeyPosesDS->points[j].intensity)
				{
					existingflag = true;
					break;
				}
			}
			if (existingflag == false)
			{
				surroundingExistingKeyPoseID.erase(surroundingExistingKeyPoseID.begin() + i);
				surroundingHeightCloud.erase(surroundingHeightCloud.begin() + i);
				surroundingIntensityCloud.erase(surroundingIntensityCloud.begin() + i);
				i--;
			}
		}
		for (int i = 0; i < numsurroundingKeyPosesDS; ++i)
		{
			bool existingflag = false;
			for (int j = 0; j < (int)surroundingExistingKeyPoseID.size(); ++j)
			{
				if ((int)surroundingKeyPosesDS->points[i].intensity == surroundingExistingKeyPoseID[j])
				{
					existingflag = true;
                    break;
				}
			}
			if (existingflag == false)
			{
				int thisKeyInd = (int)surroundingKeyPosesDS->points[i].intensity;
                PointTypePose thisTransformation = cloudKeyPoses6D->points[thisKeyInd];
                auto itCor = heightCloudKeyFrames.find(thisKeyInd);
                auto itSur = intensityCloudKeyFrames.find(thisKeyInd);
                if (itCor != heightCloudKeyFrames.end() && itSur != intensityCloudKeyFrames.end())
                {
                    surroundingExistingKeyPoseID.push_back(thisKeyInd);
                    surroundingHeightCloud.push_back(transformPointCloud(heightCloudKeyFrames.at(thisKeyInd), &thisTransformation));
                    surroundingIntensityCloud.push_back(transformPointCloud(intensityCloudKeyFrames.at(thisKeyInd), &thisTransformation));
                }
			}
		}

		// fuse the map
        laserCloudHeightFromMap->clear();
        laserCloudIntensityFromMap->clear(); 
		laserCloudHeightFromMapDS->clear();
        laserCloudIntensityFromMapDS->clear();  
        for (int i = 0; i < (int)surroundingKeyPosesDS->size(); ++i)
        {
			*laserCloudHeightFromMap += *surroundingHeightCloud[i];
			*laserCloudIntensityFromMap += *surroundingIntensityCloud[i];
        }

	    // Downsample the surrounding height key frames (or map)
        downSizeFilterHeight.setInputCloud(laserCloudHeightFromMap);
        downSizeFilterHeight.filter(*laserCloudHeightFromMapDS);
		// *laserCloudHeightFromMapDS = *laserCloudHeightFromMap;
        laserCloudHeightFromMapDSNum = laserCloudHeightFromMapDS->size();
        // Downsample the surrounding intensity key frames (or map)
        downSizeFilterIntensity.setInputCloud(laserCloudIntensityFromMap);
        downSizeFilterIntensity.filter(*laserCloudIntensityFromMapDS);
		// *laserCloudIntensityFromMapDS = *laserCloudIntensityFromMap;
        laserCloudIntensityFromMapDSNum = laserCloudIntensityFromMapDS->size();
    }

	void correctKeyFramePoseGraph()
	{
		// 如果发生了全局因子优化，需要更新整个位姿图；
		if (bLoopIsClosed == true || bGNSSIsAdded == true)
        {
            // clear map cache, because poses have changed;
            surroundingExistingKeyPoseID.clear();
            surroundingHeightCloud.clear();
            surroundingIntensityCloud.clear();
			// clear global path and need update;
        	globalPoseGraphPath.poses.clear();

            int numPoses = cloudKeyPoses6D->size();
			mKeyframe.lock();
            for (int i = 0; i < numPoses; ++i)
            {
				int keyID = cloudKeyPoses6D->points[i].intensity; // 取出每个关键帧的ID；
				if(correctedKeyPose6DByLoop.find(keyID) != correctedKeyPose6DByLoop.end()) // 如果存在优化容器中；
				{
					PointTypePose correctedKeyPose6D = correctedKeyPose6DByLoop.at(keyID); // 取出优化后的帧；
					cloudKeyPoses3D->points[i].x = correctedKeyPose6D.x;
					cloudKeyPoses3D->points[i].y = correctedKeyPose6D.y;
					cloudKeyPoses3D->points[i].z = correctedKeyPose6D.z;
					cloudKeyPoses6D->points[i] = correctedKeyPose6D;
					updateGlobalPath(cloudKeyPoses6D->points[i]);
				}
            }

			// 因为有俩线程对优化期间新添加的关键帧进行矫正，维护局部一致性后才能进行地图构建优化；
			PointTypePose currKeyPose6D = correctedKeyPose6DByLoop.rbegin()->second; // 最后一个元素为位姿图优化中最新的元素；
			int key_curr = currKeyPose6D.intensity;
			for (int i = 0; i < numPoses; ++i)
			{
				PointTypePose tmpPose6D = cloudKeyPoses6D->points[i];
				int keyID = tmpPose6D.intensity; // 取出每个关键帧的ID；

				// 对于已经校正的关键帧跳过
				if (keyID <= key_curr)
					continue;

				Eigen::Affine3f TBefore = pcl::getTransformation(tmpPose6D.x, tmpPose6D.y, tmpPose6D.z, tmpPose6D.roll, tmpPose6D.pitch, tmpPose6D.yaw);
				Eigen::Affine3f TAfter = T_Drift * TBefore;
				pcl::getTranslationAndEulerAngles(TAfter, tmpPose6D.x, tmpPose6D.y, tmpPose6D.z, tmpPose6D.roll, tmpPose6D.pitch, tmpPose6D.yaw);
				
				cloudKeyPoses3D->points[i].x = tmpPose6D.x;
				cloudKeyPoses3D->points[i].y = tmpPose6D.y;
				cloudKeyPoses3D->points[i].z = tmpPose6D.z;
				cloudKeyPoses6D->points[i] = tmpPose6D;
				updateGlobalPath(cloudKeyPoses6D->points[i]);
			}
			mKeyframe.unlock();

			// 对KeyPose6D容器也更新；
			KeyPose6D.clear();
			int keySize = cloudKeyPoses6D->size();
			for (int i = 0; i < keySize; ++i)
			{
				PointTypePose thisPose6D = cloudKeyPoses6D->points[i];
				int keyID = thisPose6D.intensity;
				KeyPose6D[keyID] = thisPose6D;
			}

			// 位姿图更新后，重新绘制闭环约束可视化；
			visualizeLoopConstraintEdge();

            bLoopIsClosed = false;
            bGNSSIsAdded = false;
        }
	}

	void downsampleCurrentLaserCloud(void)
    {
		// save last cloud
		laserCloudHeightLastDS->clear();
		laserCloudIntensityLastDS->clear();

		pcl::copyPointCloud(*laserCloudHeightDS, *laserCloudHeightLastDS);
        pcl::copyPointCloud(*laserCloudIntensityDS, *laserCloudIntensityLastDS);

		laserCloudHeightLastDSNum = laserCloudHeightDSNum;
		laserCloudIntensityLastDSNum = laserCloudIntensityDSNum;

        // Downsample cloud from current scan
        laserCloudHeightDS->clear();
        downSizeFilterHeight.setInputCloud(laserCloudHeight);
        downSizeFilterHeight.filter(*laserCloudHeightDS);
		// *laserCloudHeightDS = *laserCloudHeight;
        laserCloudHeightDSNum = laserCloudHeightDS->size();

        laserCloudIntensityDS->clear();
        downSizeFilterIntensity.setInputCloud(laserCloudIntensity);
        downSizeFilterIntensity.filter(*laserCloudIntensityDS);
		// *laserCloudIntensityDS = *laserCloudIntensity;
        laserCloudIntensityDSNum = laserCloudIntensityDS->size();
    }

	void transformAssociateToMap()
	{
		q_w_curr = q_wmap_wodom * q_wodom_curr;
		t_w_curr = q_wmap_wodom * t_wodom_curr + t_wmap_wodom;
		q_w_curr.normalize();
	}

	void transformUpdate()
	{
		q_wmap_wodom = q_w_curr * q_wodom_curr.inverse();
		t_wmap_wodom = t_w_curr - q_wmap_wodom * t_wodom_curr;
	}

	bool saveKeyframeAndOdomFactor(Eigen::Vector3d pos, Eigen::Vector3d rot, double time)
	{
		if (laserCloudHeightLastDSNum == 0 || laserCloudIntensityLastDSNum == 0) return false;
		if (map_update == 0) return false;
		if (cloudKeyPoses3D->size() > 10)//太近不认为是关键帧
		{
			PointTypePose lastPose6D = cloudKeyPoses6D->back();
			float dx = pos.x() - lastPose6D.x;
			float dy = pos.y() - lastPose6D.y;
			float dz = pos.z() - lastPose6D.z;
			float dyaw = rot.x() - lastPose6D.yaw;
			float dpitch = rot.y() - lastPose6D.pitch;
			float droll = rot.z() - lastPose6D.roll;
			if (dyaw > M_PI) dyaw = dyaw - M_PI * 2; 
			if (dyaw < -M_PI) dyaw = dyaw + M_PI * 2; 

			if (abs(droll) < keyframeAddingAngle && 
                abs(dpitch) < keyframeAddingAngle && 
                abs(dyaw) < keyframeAddingAngle && 
                sqrt(dx * dx + dy * dy + dz * dz) < keyframeAddingDistance)
			{
				return false;
			}
		}

		// 拼接地图时流出重定位的时间
		static int initial_count = 0;
		if (RMODE == 0 && map_update == 1)
		{
			if (initial_count < 70)
			{
				initial_count++;
				return false;
			}
		}

		bnewKeyFrame = true;
		
		// temp key poses
        PointType thisPose3D;
        PointTypePose thisPose6D, lastPose6D;

        thisPose3D.x = pos.x();
        thisPose3D.y = pos.y();
        thisPose3D.z = pos.z();
        thisPose3D.intensity = keyFrameNum; // this can be used as index
        
        thisPose6D.x = thisPose3D.x;
        thisPose6D.y = thisPose3D.y;
        thisPose6D.z = thisPose3D.z;
		thisPose6D.intensity = thisPose3D.intensity ; // this can be used as index
        thisPose6D.yaw = rot.x();
        thisPose6D.pitch = rot.y();
        thisPose6D.roll = rot.z();
        thisPose6D.time = time;

		// calculate travel distance;
		if (cloudKeyPoses6D->empty())
        {
            travel_distance[keyFrameNum] = 0;
			travel_angle[keyFrameNum] = 0;
        }
        else
        {
			lastPose6D = cloudKeyPoses6D->back();

            float dis_delta = sqrt((thisPose6D.x - lastPose6D.x) * (thisPose6D.x - lastPose6D.x) + 
								   (thisPose6D.y - lastPose6D.y) * (thisPose6D.y - lastPose6D.y) +
								   (thisPose6D.z - lastPose6D.z) * (thisPose6D.z - lastPose6D.z));
            float dis_temp = travel_distance.rbegin()->second + dis_delta;//累计路程
            travel_distance[keyFrameNum] = dis_temp;

			float rad_delta = thisPose6D.yaw - lastPose6D.yaw;
			if (rad_delta > M_PI) rad_delta = rad_delta - M_PI * 2; 
			if (rad_delta < -M_PI) rad_delta = rad_delta + M_PI * 2; 
            float rad_temp = travel_angle.rbegin()->second + abs(rad_delta);//累计旋转
            travel_angle[keyFrameNum] = rad_temp;
        }
		
		pcl::PointCloud<PointType>::Ptr thisHeightKeyFrame(new pcl::PointCloud<PointType>());
		pcl::PointCloud<PointType>::Ptr thisIntensityKeyFrame(new pcl::PointCloud<PointType>());
		pcl::PointCloud<PointType>::Ptr thisScanFrame(new pcl::PointCloud<PointType>());
		pcl::copyPointCloud(*laserCloudHeightLastDS, *thisHeightKeyFrame);
		pcl::copyPointCloud(*laserCloudIntensityLastDS, *thisIntensityKeyFrame);
		bool usingRawCloud = true;
		mKeyframe.lock();
		cloudKeyPoses3D->push_back(thisPose3D);
		//printf("num:%d",cloudKeyPoses3D->size());
		cloudKeyPoses6D->push_back(thisPose6D);
		heightCloudKeyFrames[keyFrameNum] = thisHeightKeyFrame;
		intensityCloudKeyFrames[keyFrameNum] = thisIntensityKeyFrame;
		KeyPose6D[keyFrameNum] = thisPose6D;
		
		//词袋
		if(enable_DoWB)
		{
			DoWB_key_frames[keyFrameNum] = BEV_now;	
			wordbag_loop.add(BEV_now.description_dowb);
			
		}
        //sc处理
		// if( usingRawCloud ) { // v2 uses downsampled raw point cloud, more fruitful height information than using feature points (v1)
        //     scManager.makeAndSaveScancontextAndKeys(*thisScanFrame);
        // }
        // else { // v1 uses thisIntensityKeyFrame, it also works. (empirically checked at Mulran dataset sequences)
        //     scManager.makeAndSaveScancontextAndKeys(*thisIntensityKeyFrame); 
        // }
		mKeyframe.unlock();
		// 存储pcd文件
		// boost::format fmt_pcd("%s/%s/%d.%s");
        // pcl::io::savePCDFileASCII((fmt_pcd % saveDirectory % "heightcloud" % keyFrameNum % "pcd").str(), *thisHeightKeyFrame);
        // pcl::io::savePCDFileASCII((fmt_pcd % saveDirectory % "intensitycloud" % keyFrameNum % "pcd").str(), *thisIntensityKeyFrame);

		keyFrameNum++;

		updateGlobalPath(thisPose6D);

		return true;
	}

	void updateGlobalPath(const PointTypePose& pose_in)
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time().fromSec(pose_in.time);
        pose_stamped.header.frame_id = "camera_init";
        pose_stamped.pose.position.x = pose_in.x;
        pose_stamped.pose.position.y = pose_in.y;
        pose_stamped.pose.position.z = pose_in.z;
        tf::Quaternion q = tf::createQuaternionFromRPY(pose_in.roll, pose_in.pitch, pose_in.yaw);
        pose_stamped.pose.orientation.x = q.x();
        pose_stamped.pose.orientation.y = q.y();
        pose_stamped.pose.orientation.z = q.z();
        pose_stamped.pose.orientation.w = q.w();
        globalPoseGraphPath.poses.push_back(pose_stamped);
    }

	// 位姿图优化线程
    void poseGraphOptimizationThread()
    {
		ros::Rate rate(1);
		while (ros::ok())
		{
			rate.sleep();
			// 全局优化后位姿图已经更新，需要建图线程确认并维护局部一致性后才能进行下次全局优化；
			if(bKeyFramePoseGraphUpdated == true) continue;

			copyKeyPosesData();
			
			if(enable_DoWB)
			{
				detectAndCalculateLoopFactor_WB();
			}
			else
			{
				detectAndCalculateLoopFactor();
			}
			PoseGraphOptimize4DoF();

			publishGlobalPoseGraph();

			publishGlobalMap();

		}
    }
	
	void copyKeyPosesData()
	{
		if (cloudKeyPoses3D->empty()) return;

		copy_cloudKeyPoses3D->clear();
        copy_cloudKeyPoses6D->clear();

		mKeyframe.lock();
        *copy_cloudKeyPoses3D = *cloudKeyPoses3D;
        *copy_cloudKeyPoses6D = *cloudKeyPoses6D;
        mKeyframe.unlock();

		correctedKeyPose6DByLoop.clear();
		int keyPoseNum = copy_cloudKeyPoses6D->size();
		for (int i = 0; i < keyPoseNum; ++i)
		{
			PointTypePose thisPose6D = copy_cloudKeyPoses6D->points[i];
			int keyI = thisPose6D.intensity;
			correctedKeyPose6DByLoop[keyI] = thisPose6D;
		}
	}

	void detectAndCalculateLoopFactor()
    {
		static int loopContinueCount = 0;
		static bool lowDriftFlag = false; 
		static float lastLoopDistance = -1000;

		if (LoopClosureEnable != 1) return;

		if (map_update != 1) return; // 定位模式下不进行回环检测

        if (cloudKeyPoses3D->points.empty()) return;

		// control loop in proper frequency;
		// 控制规则：
		// 1.根据距离上次闭环后的运动距离来决定连续闭环的状态；
		// 2.超过连续3次闭环，进入低漂移状态；距离上次闭环后的运动距离大于设定阈值，进入高漂移状态；
		// 3.在低漂移状态下，每隔10米接受一次闭环；高漂移状态每个周期都接受闭环；

		// 系统进入低漂移状态时，每隔10米进行一次闭环优化；
		PointTypePose latestKeyPose6D, closestHistoryKeyPose6D;
		latestKeyPose6D = copy_cloudKeyPoses6D->back();
        int latestKeyFrameID = latestKeyPose6D.intensity;

		if (bnewKeyFrame == false)
            return;
		bnewKeyFrame = false;

		// begin loop detection;
		TicToc t_loop;
        int latestFrameID = -1, closestHistoryFrameID = -1;
        if (detectLoopClosure(&latestFrameID, &closestHistoryFrameID, &closestHistoryKeyPose6D) == false)
		{
			return;
		}
		visualizeLoopConstraintEdge();
		if (lowDriftFlag == true)
		{
			if (abs(travel_distance.at(latestKeyFrameID) - lastLoopDistance) < 5)
				return;
		}
		if (abs(travel_distance.at(latestKeyFrameID) - lastLoopDistance) > 20)
		{
			lowDriftFlag = false;
		}
        // ICP Settings
        static pcl::IterativeClosestPoint<PointType, PointType> icp;
        icp.setMaxCorrespondenceDistance(poseGraphSearchRadius*2);//匹配最大距离
        icp.setMaximumIterations(100);//最大迭代次数
        icp.setTransformationEpsilon(1e-6);//收敛条件 两次迭代的转换矩阵的最大容差
        icp.setEuclideanFitnessEpsilon(1e-6);//收敛条件 均方误差和小于阈值
        icp.setRANSACIterations(0);// 设置RANSAC运行次数
		// icp.setNumThreads(14);

        // Downsample map cloud
        pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
        downSizeFilterICP.setInputCloud(nearHistoryKeyFrameCloud);
        downSizeFilterICP.filter(*cloud_temp);
        *nearHistoryKeyFrameCloud = *cloud_temp;

        // Align clouds
        icp.setInputSource(latestKeyFrameCloud);
        icp.setInputTarget(nearHistoryKeyFrameCloud);
        pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
        icp.align(*unused_result);

		float alignDistance = icp.getFitnessScore();
        if (icp.hasConverged() == false || alignDistance > historyKeyframeFitnessScore)//参1为成功flag 参2为配准后的点云的最近点之间距离的均值
		{
			printf("\n--PoseGraph: false loop between : %d and %d , dis: %.2f and %.2f, score: %.3f, time: %.1f, return !!!!!!!!!!!!!!! \n\n", 
			latestFrameID, closestHistoryFrameID, travel_distance.at(latestFrameID), travel_distance.at(closestHistoryFrameID), alignDistance, t_loop.toc());
			return;
		}
		else
		{
			loopClosureCount++;
			printf("\n--PoseGraph: detected loop between : %d and %d , dis: %.2f and %.2f, score: %.3f, loop_count: %d, time: %.1f !!!!!!!!!!!!!!! \n\n", 
			latestFrameID, closestHistoryFrameID, travel_distance.at(latestFrameID), travel_distance.at(closestHistoryFrameID), alignDistance, loopClosureCount, t_loop.toc());
		}

		// ICP的估计结果为：当前位姿下的点云，乘以多少变换，就能与地图点云对齐，也就是得到了漂移量;
		// 计算漂移量；
		T_Drift = icp.getFinalTransformation();
		t_drift = T_Drift.translation();
		q_drift = T_Drift.rotation();
		q_drift.normalize();

		// transform from world to wrong pose;
        Eigen::Affine3f T_w_latest = pclPointToAffine3f(latestKeyPose6D); // 最新帧的原始位姿；
        // transform from world to corrected pose；
        Eigen::Affine3f T_w_correct = T_Drift * T_w_latest; // 通过漂移量校正后的最新帧位姿；
		// transform from world to loop pose;
        Eigen::Affine3f T_w_loop = pclPointToAffine3f(closestHistoryKeyPose6D); // 闭环帧的位姿；
		// transform from correct to loop pose;
        Eigen::Affine3f T_loop_correct = T_w_loop.inverse() * T_w_correct; 
	
		loopInfo loopInfoTmp;
		loopInfoTmp.key_curr = latestFrameID;
		loopInfoTmp.key_loop = closestHistoryFrameID;
		loopInfoTmp.keyPose6DCurr = latestKeyPose6D;
		loopInfoTmp.keyPose6DLoop = closestHistoryKeyPose6D;
		loopInfoTmp.t_loop_curr = T_loop_correct.translation();
		loopInfoTmp.q_loop_curr = T_loop_correct.rotation();
		loopInfoTmp.noise = alignDistance;
		loopInfoContainer[latestFrameID] = loopInfoTmp;

		if (loopInfoContainer.size() > 100)
		{
			loopInfoContainer.erase(loopInfoContainer.begin());
		}

        // save all loop constriant for visualize;
        currLoopKeyContainer.push_back(std::make_pair(latestFrameID, closestHistoryFrameID));

		visualizeLoopConstraintEdge();

		// 对系统的漂移状态进行判断，低漂移状态时限制闭环优化的频率；
		if (abs(travel_distance.at(latestFrameID) - lastLoopDistance) < 10) // 判断数据库当前帧与上个闭环帧之间的运动距离；
			loopContinueCount++;
		else
			loopContinueCount = 0; // reset
		
		if (loopContinueCount > 4)
			lowDriftFlag = true;
		
		// 闭环帧之间的距离，代表着这两帧之间的漂移被消除；
		DistanceByLoop = travel_distance.at(latestFrameID) - travel_distance.at(closestHistoryFrameID);
		if (DistanceByLoop < 0)
		{
			DistanceByLoop = 0;
			ROS_WARN("--Loopfactor: wrong DistanceByLoop !!!");
		}
		lastLoopDistance = travel_distance.at(latestFrameID);

        bLoopIsClosed = true;
    }

	bool detectLoopClosure(int *latestID, int *closestID, PointTypePose *closestKeyPose6D)
    {
		int latestFrameIndex = copy_cloudKeyPoses3D->size()-1;
        int latestFrameKeyID = copy_cloudKeyPoses3D->points[latestFrameIndex].intensity;
        int closestHistoryFrameID = -1, closestHistoryFrameIndex = -1;

		// 闭环优化后，SLAM在闭环区域内的漂移消除，因此减去这段区域的运动产生的漂移；
        poseGraphSearchRadius = historyKeyframeSearchRadius + (travel_distance.rbegin()->second - DistanceByLoop) * DRIFT_FACTOR;

        latestKeyFrameCloud->clear();
        nearHistoryKeyFrameCloud->clear();

        // find the closest history key frame
        std::vector<int> pointSearchIndLoop;
        std::vector<float> pointSearchSqDisLoop;
        kdtreeHistoryKeyPoses.setInputCloud(copy_cloudKeyPoses3D);
        kdtreeHistoryKeyPoses.radiusSearch(copy_cloudKeyPoses3D->back(), poseGraphSearchRadius, pointSearchIndLoop, pointSearchSqDisLoop, 0);

        for (int i = 0; i < (int)pointSearchIndLoop.size(); ++i)
        {
            int index = pointSearchIndLoop[i];
			int keyID = copy_cloudKeyPoses3D->points[index].intensity;
			// 闭环帧之间的运行距离应该超过一定阈值，防止刚刚添加的相近的帧产生闭环；
            if (abs(travel_distance.at(keyID) - travel_distance.rbegin()->second) > (loopKeyframeDisDiff + poseGraphSearchRadius))
            {
				if (keyID < 10) continue;
                closestHistoryFrameID = keyID;
				closestHistoryFrameIndex = index;
				*closestKeyPose6D = copy_cloudKeyPoses6D->points[index];
                break;
            }
        }

        if (closestHistoryFrameID == -1 || closestHistoryFrameIndex == -1)
            return false;

        if (latestFrameKeyID == closestHistoryFrameID)
            return false;

        // save latest key frames
		PointTypePose latestKeyPose6D = copy_cloudKeyPoses6D->points[latestFrameIndex];

		auto itCor = heightCloudKeyFrames.find(latestFrameKeyID);
		if (itCor != heightCloudKeyFrames.end())
		{
			*latestKeyFrameCloud += *transformPointCloud(heightCloudKeyFrames.at(latestFrameKeyID), &latestKeyPose6D);
		}
		auto itSur = intensityCloudKeyFrames.find(latestFrameKeyID);
		if (itSur != intensityCloudKeyFrames.end())
		{
			*latestKeyFrameCloud += *transformPointCloud(intensityCloudKeyFrames.at(latestFrameKeyID), &latestKeyPose6D);
		}
        
        // save history near key frames
        for (int i = -historyKeyframeSearchNum; i <= historyKeyframeSearchNum; ++i)
        {
			int thisIndex = closestHistoryFrameIndex + i;
			if (thisIndex < 0 || thisIndex >= latestFrameIndex) // 索引内存在一定范围内；
                continue;

			int thisKeyID = copy_cloudKeyPoses6D->points[thisIndex].intensity;
            if (thisKeyID < 0 || thisKeyID >= latestFrameKeyID) // id在一定范围内；
                continue;

			PointTypePose historyKeyPose6D = copy_cloudKeyPoses6D->points[thisIndex];
			auto itCor = heightCloudKeyFrames.find(thisKeyID);
			if (itCor != heightCloudKeyFrames.end())
			{
				*nearHistoryKeyFrameCloud += *transformPointCloud(heightCloudKeyFrames.at(thisKeyID), &historyKeyPose6D);
			}
			auto itSur = intensityCloudKeyFrames.find(thisKeyID);
			if (itSur != intensityCloudKeyFrames.end())
			{
				*nearHistoryKeyFrameCloud += *transformPointCloud(intensityCloudKeyFrames.at(thisKeyID), &historyKeyPose6D);
			}
        }

        if (nearHistoryKeyFrameCloud->points.empty())
            return false;

        *latestID = latestFrameKeyID;
        *closestID = closestHistoryFrameID;

        return true;
    }

	void detectAndCalculateLoopFactor_WB()
	{
		static int loopContinueCount = 0;
		static bool lowDriftFlag = false; 
		static float lastLoopDistance = -1000;

		if (LoopClosureEnable != 1) return;

		if (map_update != 1) return; // 定位模式下不进行回环检测

        if (cloudKeyPoses3D->points.empty()) return;

		// control loop in proper frequency;
		// 控制规则：
		// 1.根据距离上次闭环后的运动距离来决定连续闭环的状态；
		// 2.超过连续3次闭环，进入低漂移状态；距离上次闭环后的运动距离大于设定阈值，进入高漂移状态；
		// 3.在低漂移状态下，每隔10米接受一次闭环；高漂移状态每个周期都接受闭环；

		// 系统进入低漂移状态时，每隔10米进行一次闭环优化；
		PointTypePose latestKeyPose6D, closestHistoryKeyPose6D;
		latestKeyPose6D = copy_cloudKeyPoses6D->back();
        int latestKeyFrameID = latestKeyPose6D.intensity;

		if (bnewKeyFrame == false)
            return;
		bnewKeyFrame = false;

		// begin loop detection;
		TicToc t_loop;
        int latestFrameID = -1, closestHistoryFrameID = -1;
        if (detectLoopClosure_WB(&latestFrameID, &closestHistoryFrameID, &closestHistoryKeyPose6D) == false)
		{
			if (detectLoopClosure(&latestFrameID, &closestHistoryFrameID, &closestHistoryKeyPose6D) == false)
			{
				return;
			}
		}
		visualizeLoopConstraintEdge();
		if (lowDriftFlag == true)
		{
			if (abs(travel_distance.at(latestKeyFrameID) - lastLoopDistance) < 5)
				return;
		}
		if (abs(travel_distance.at(latestKeyFrameID) - lastLoopDistance) > 20)
		{
			lowDriftFlag = false;
		}
        // ICP Settings
		// static fast_gicp::FastGICP<pcl::PointXYZI, pcl::PointXYZI> icp;
        static pcl::IterativeClosestPoint<PointType, PointType> icp;
		// icp.setResolution(down_simple_vgicp);
        icp.setMaxCorrespondenceDistance(poseGraphSearchRadius*2);//匹配最大距离
        icp.setMaximumIterations(100);//最大迭代次数
        icp.setTransformationEpsilon(1e-6);//收敛条件 两次迭代的转换矩阵的最大容差
        icp.setEuclideanFitnessEpsilon(1e-6);//收敛条件 均方误差和小于阈值
		// icp.setNumThreads(14);	
        icp.setRANSACIterations(0);// 设置RANSAC运行次数
		// icp.setNumThreads(14);

        // Downsample map cloud
        pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
        downSizeFilterICP.setInputCloud(nearHistoryKeyFrameCloud);
        downSizeFilterICP.filter(*cloud_temp);
        *nearHistoryKeyFrameCloud = *cloud_temp;

        // Align clouds
        icp.setInputSource(latestKeyFrameCloud);
        icp.setInputTarget(nearHistoryKeyFrameCloud);
        pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
        icp.align(*unused_result);

		float alignDistance = icp.getFitnessScore();
        if (icp.hasConverged() == false || alignDistance > historyKeyframeFitnessScore)//参1为成功flag 参2为配准后的点云的最近点之间距离的均值
		{
			printf("\n--PoseGraph: false loop between : %d and %d , dis: %.2f and %.2f, score: %.3f, time: %.1f, return !!!!!!!!!!!!!!! \n\n", 
			latestFrameID, closestHistoryFrameID, travel_distance.at(latestFrameID), travel_distance.at(closestHistoryFrameID), alignDistance, t_loop.toc());
			return;
		}
		else
		{
			loopClosureCount++;
			printf("\n--PoseGraph: detected loop between : %d and %d , dis: %.2f and %.2f, score: %.3f, loop_count: %d, time: %.1f !!!!!!!!!!!!!!! \n\n", 
			latestFrameID, closestHistoryFrameID, travel_distance.at(latestFrameID), travel_distance.at(closestHistoryFrameID), alignDistance, loopClosureCount, t_loop.toc());
		}

		// ICP的估计结果为：当前位姿下的点云，乘以多少变换，就能与地图点云对齐，也就是得到了漂移量;
		// 计算漂移量；
		T_Drift = icp.getFinalTransformation();
		t_drift = T_Drift.translation();
		q_drift = T_Drift.rotation();
		q_drift.normalize();

		// transform from world to wrong pose;
        Eigen::Affine3f T_w_latest = pclPointToAffine3f(latestKeyPose6D); // 最新帧的原始位姿；
        // transform from world to corrected pose；
        Eigen::Affine3f T_w_correct = T_Drift * T_w_latest; // 通过漂移量校正后的最新帧位姿；
		// transform from world to loop pose;
        Eigen::Affine3f T_w_loop = pclPointToAffine3f(closestHistoryKeyPose6D); // 闭环帧的位姿；
		// transform from correct to loop pose;
        Eigen::Affine3f T_loop_correct = T_w_loop.inverse() * T_w_correct; 
	
		loopInfo loopInfoTmp;
		loopInfoTmp.key_curr = latestFrameID;
		loopInfoTmp.key_loop = closestHistoryFrameID;
		loopInfoTmp.keyPose6DCurr = latestKeyPose6D;
		loopInfoTmp.keyPose6DLoop = closestHistoryKeyPose6D;
		loopInfoTmp.t_loop_curr = T_loop_correct.translation();
		loopInfoTmp.q_loop_curr = T_loop_correct.rotation();
		loopInfoTmp.noise = alignDistance;
		loopInfoContainer[latestFrameID] = loopInfoTmp;

		if (loopInfoContainer.size() > 100)
		{
			loopInfoContainer.erase(loopInfoContainer.begin());
		}
        // save all loop constriant for visualize;
        currLoopKeyContainer.push_back(std::make_pair(latestFrameID, closestHistoryFrameID));

		visualizeLoopConstraintEdge();

		// 对系统的漂移状态进行判断，低漂移状态时限制闭环优化的频率；
		if (abs(travel_distance.at(latestFrameID) - lastLoopDistance) < 10) // 判断数据库当前帧与上个闭环帧之间的运动距离；
			loopContinueCount++;
		else
			loopContinueCount = 0; // reset
		
		if (loopContinueCount > 4)
			lowDriftFlag = true;
		
		// 闭环帧之间的距离，代表着这两帧之间的漂移被消除；
		DistanceByLoop = travel_distance.at(latestFrameID) - travel_distance.at(closestHistoryFrameID);
		if (DistanceByLoop < 0)
		{
			DistanceByLoop = 0;
			ROS_WARN("--Loopfactor: wrong DistanceByLoop !!!");
		}
		lastLoopDistance = travel_distance.at(latestFrameID);

        bLoopIsClosed = true;
	}

	bool detectLoopClosure_WB(int *latestID, int *closestID, PointTypePose *closestKeyPose6D)
    {
		int latestFrameIndex = copy_cloudKeyPoses3D->size()-1;
        int latestFrameKeyID = copy_cloudKeyPoses3D->points[latestFrameIndex].intensity;
        int closestHistoryFrameID = -1, closestHistoryFrameIndex = -1;
		cv::BFMatcher matcher(cv::NORM_HAMMING);

		// 闭环优化后，SLAM在闭环区域内的漂移消除，因此减去这段区域的运动产生的漂移；
        poseGraphSearchRadius = historyKeyframeSearchRadius + (travel_distance.rbegin()->second - DistanceByLoop) * DRIFT_FACTOR;

        latestKeyFrameCloud->clear();
        nearHistoryKeyFrameCloud->clear();

		// find the closest history key frame
        // std::vector<int> pointSearchIndLoop;
        // std::vector<float> pointSearchSqDisLoop;
        // kdtreeHistoryKeyPoses.setInputCloud(copy_cloudKeyPoses3D);
        // kdtreeHistoryKeyPoses.radiusSearch(copy_cloudKeyPoses3D->back(), poseGraphSearchRadius, pointSearchIndLoop, pointSearchSqDisLoop, 0);

        // for (int i = 0; i < (int)pointSearchIndLoop.size(); ++i)
        // {
        //     int index = pointSearchIndLoop[i];
		// 	int keyID = copy_cloudKeyPoses3D->points[index].intensity;
		// 	// 闭环帧之间的运行距离应该超过一定阈值，防止刚刚添加的相近的帧产生闭环；
        //     if (abs(travel_distance.at(keyID) - travel_distance.rbegin()->second) > (loopKeyframeDisDiff + poseGraphSearchRadius))
        //     {
		// 		if (keyID < 10) continue;
				
		// 		vector<cv::DMatch> matches;
		// 		matcher.match(DoWB_key_frames.at(latestFrameIndex).description,DoWB_key_frames.at(index).description,matches);
		// 		std::vector<bool> vbInliers;
		// 		gms_matcher gms_height(DoWB_key_frames.at(latestFrameIndex).keypoints, DoWB_key_frames.at(latestFrameIndex).BEV_image.size(), DoWB_key_frames.at(index).keypoints, DoWB_key_frames.at(index).BEV_image.size(), matches);
		// 		int linliers_pose = gms_height.GetInlierMask(vbInliers, false, false);
		// 		if(linliers_pose > 150)
		// 		{
		// 			closestHistoryFrameID = keyID;
		// 			closestHistoryFrameIndex = index;
		// 			*closestKeyPose6D = copy_cloudKeyPoses6D->points[index];
		// 			break;
		// 		}
        //     }
        // }
		//----------------------------wordbag初始化---------------------------------
		// ROS_ERROR("11111111111111111");
		// if(closestHistoryFrameID == -1 || closestHistoryFrameIndex == -1 || latestFrameKeyID == closestHistoryFrameID)
		// {
			TicToc loop_time;
			DBoW3::QueryResults ret;
			std::vector<DBoW3::QueryResults> ret_queue;
			std::vector<int> index_sort;
			wordbag_loop.query(DoWB_key_frames.at(latestFrameIndex).description_dowb,ret,select_DoWB_num, latestFrameIndex - 150);
			if(ret.size()<=1)
			{
				return false;
			}
			bool find_loop = false;
			if (ret.size() >= 1 && ret[0].Score > MIN_LOOP_BOW_TH)
			{
				for (unsigned int i = 0; i < ret.size(); i++)
				{
					if (ret[i].Score > MIN_LOOP_BOW_TH)
					{          
						find_loop = true;
					}
					index_sort.push_back(ret[i].Id);
				}
			}
			else
			{
				return false;
			}
			int min_index = -1;
			// std::queue<int> id_deque;
			float min_source;
			if(find_loop)
			{
				
				// for (unsigned int i = 1; i < ret.size(); i++)
				// {
				// 	if (min_index == -1 || ((int)ret[i].Id < min_index && ret[i].Score > MIN_LOOP_BOW_TH))
				// 	{
				// 		min_index = ret[i].Id;
				// 		min_source = ret[i].Score;
				// 		id_deque.push(min_index);
				// 		if(id_deque.size()>select_DoWB_num/2)
				// 		{
				// 			id_deque.pop();
				// 		}
				// 	}	

				// }
				for (int i = 0; i < index_sort.size() - 1; i++)
				{
					for(int j = 0; j < index_sort.size() - 1 - i; j++)
					{
						int temp_sort;
						if(index_sort[j] > index_sort[j + 1])
						{
							temp_sort = index_sort[j + 1];
							index_sort[j + 1] = index_sort[j];
							index_sort[j] = temp_sort;
						}
					}
				}
				min_index = index_sort[0];
			}
			else
			{
				return false;
			}
			if(min_index == -1)
			{
				return false;
			}
			
			// if(min_source <= MIN_LOOP_BOW_TH)
			// {
			// 	return false;
			// }
			
			
			int linliers;
			int max_linliers = -1;

			// vector<cv::DMatch> matches, good_matches;
			// matcher.match(DoWB_key_frames.at(latestFrameIndex).description,DoWB_key_frames.at(min_index).description,matches);
			// std::vector<bool> vbInliers;
			// gms_matcher gms_height(DoWB_key_frames.at(latestFrameIndex).keypoints, DoWB_key_frames.at(latestFrameIndex).BEV_image.size(), DoWB_key_frames.at(min_index).keypoints, DoWB_key_frames.at(min_index).BEV_image.size(), matches);
			// int height_linliers = gms_height.GetInlierMask(vbInliers, false, false);

			// for(int i = 0; i < vbInliers.size(); i++)
			// {
			// 	if(vbInliers[i] == true)
			// 	{
			// 		good_matches.push_back(matches[i]);
			// 	}
			// }

			// std::sort(good_matches.begin(), good_matches.end());
			// if ((int)good_matches.size() < 15)
			// {
			// 	return false;
			// }
			// vector<cv::Point2f> matched_2d_cur, matched_2d_old;
			// for (size_t i=0; i < good_matches.size(); i++)
			// {
			// 	int cur_index = good_matches[i].queryIdx;
			// 	matched_2d_cur.push_back(DoWB_key_frames.at(latestFrameIndex).keypoints[cur_index].pt);
				
			// 	int old_index = good_matches[i].trainIdx;
			// 	matched_2d_old.push_back(DoWB_key_frames.at(id_deque.front()).keypoints[old_index].pt);
			// }
			// vector<uchar>inlier;
			// P2PRANSAC(matched_2d_old, matched_2d_cur, inlier);	
			// int match_num = 0;
			// for (size_t i = 0; i < inlier.size(); i++)
			// {
			// 	if (inlier[i])
			// 	{
			// 		match_num+=1;
			// 	}
			// }
			// linliers = match_num;	
			// max_linliers = linliers;
			// if(!distributionValidation(matched_2d_cur, matched_2d_old))
			// {
			// 	return false;
			// }

			// while (!id_deque.empty())
			// {
			// 	vector<cv::DMatch> matches, good_matches;
			// 	matcher.match(DoWB_key_frames.at(latestFrameIndex).description,DoWB_key_frames.at(id_deque.front()).description,good_matches);
			// 	std::sort(good_matches.begin(), good_matches.end());

			// 	if ((int)good_matches.size() > 200)
			// 	{
			// 		vector<cv::Point2f> matched_2d_cur, matched_2d_old;
			// 		for (size_t i=0; i < good_matches.size(); i++)
			// 		{
			// 			int cur_index = good_matches[i].queryIdx;
			// 			matched_2d_cur.push_back(DoWB_key_frames.at(latestFrameIndex).keypoints[cur_index].pt);
						
			// 			int old_index = good_matches[i].trainIdx;
			// 			matched_2d_old.push_back(DoWB_key_frames.at(id_deque.front()).keypoints[old_index].pt);
			// 		}
			// 		vector<uchar>inlier;
			// 		P2PRANSAC(matched_2d_old, matched_2d_cur, inlier);	
			// 		int match_num = 0;
			// 		for (size_t i = 0; i < inlier.size(); i++)
			// 		{
			// 			if (inlier[i])
			// 			{
			// 				match_num+=1;
			// 			}
			// 		}
			// 		linliers = match_num;	
			// 		if(linliers > max_linliers)
			// 		{
			// 			max_linliers = linliers;
			// 			min_index = id_deque.front();
			// 		}	
			// 		id_deque.pop();
			// 	}
			// 	else
			// 	{
			// 		id_deque.pop();
			// 	}

			// }
			int index_num = 0;
			vector<cv::DMatch> good_matches_height;
			while (index_num < index_sort.size())
			{
				vector<cv::DMatch> matches, good_matches;
				matcher.match(DoWB_key_frames.at(latestFrameIndex).description,DoWB_key_frames.at(index_sort[index_num]).description,matches);
				std::vector<bool> vbInliers;
				gms_matcher gms_height(DoWB_key_frames.at(latestFrameIndex).keypoints, DoWB_key_frames.at(latestFrameIndex).BEV_image.size(), DoWB_key_frames.at(index_sort[index_num]).keypoints, DoWB_key_frames.at(index_sort[index_num]).BEV_image.size(), matches);
				linliers = gms_height.GetInlierMask(vbInliers, false, true);
				for(int i = 0; i < vbInliers.size(); i++)
				{
					if(vbInliers[i] == true)
					{
						good_matches.push_back(matches[i]);
					}
				}
				vector<cv::Point2f> matched_2d_cur, matched_2d_old;
				for (size_t i=0; i < good_matches.size(); i++)
				{
					int cur_index = good_matches[i].queryIdx;
					matched_2d_cur.push_back(DoWB_key_frames.at(latestFrameIndex).keypoints[cur_index].pt);
					
					int old_index = good_matches[i].trainIdx;
					matched_2d_old.push_back(DoWB_key_frames.at(index_sort[index_num]).keypoints[old_index].pt);
				}

				if(linliers > max_linliers && distributionValidation(matched_2d_cur, matched_2d_old))
				{
					max_linliers = linliers;
					min_index = index_sort[index_num];
					good_matches_height = good_matches;
				}
				index_num++;
			}
			
			if(max_linliers < 50)
			{
				return false;
			}
			printf("--laserMapping: word bag num: %d ,point_num: %d , spendtime:%f!!! \n\n\n",min_index, max_linliers, loop_time.toc());
			// printf("--laserMapping: word bag num: %d ,point_num: %d , spendtime:%f!!! \n\n\n",min_index, max_linliers, loop_time.toc());
			// currLoopKeyContainer.push_back(std::make_pair(latestFrameKeyID, min_index));
			// visualizeLoopConstraintEdge();

			int min_keyID = copy_cloudKeyPoses3D->points[min_index].intensity;
			
			
			if (abs(travel_distance.at(min_keyID) - travel_distance.rbegin()->second) > (loopKeyframeDisDiff + poseGraphSearchRadius))
			{
				closestHistoryFrameID = min_keyID;
				closestHistoryFrameIndex = min_index;
				*closestKeyPose6D = copy_cloudKeyPoses6D->points[closestHistoryFrameID];
			}
			else
			{
				closestHistoryFrameID = -1;
			}
		// }
		// ROS_ERROR("333333333333333333");
        if (closestHistoryFrameID == -1 || closestHistoryFrameIndex == -1)
            return false;

        if (latestFrameKeyID == closestHistoryFrameID)
            return false;
		
		cv::Mat img_matches_height;

		cv::drawMatches(DoWB_key_frames.at(latestFrameIndex).BEV_RGB, DoWB_key_frames.at(latestFrameIndex).keypoints, DoWB_key_frames.at(min_index).BEV_RGB, DoWB_key_frames.at(min_index).keypoints, good_matches_height, img_matches_height, cv::Scalar(255,0,0), cv::Scalar(255,0,0), vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
		sensor_msgs::ImagePtr hight_msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", img_matches_height).toImageMsg();
		hight_msg->header.stamp = ros::Time::now();
		hight_msg->header.frame_id = "camera_init";
		pubheightBEV.publish(hight_msg);
		// boost::format fmt_image("%s%d%s");

		// cv::imwrite((fmt_image % "home/wsc/bev_lo/src/orb_lio/img/loopclose/BEV_height_" %loop_numm%".png").str(), img_matches_height);
		// cv::imwrite((fmt_image % "home/wsc/bev_lo/src/orb_lio/img/loopclose/BEV_height_last_" %loop_numm%".png").str(), DoWB_key_frames.at(min_index).BEV_RGB);
		printf("--laserMapping: word bag num: %d ,point_num: %d , spendtime:%f!!! \n\n\n",min_index, max_linliers, loop_time.toc());
		loop_numm++;
		
		// visualizeLoopConstraintEdge();
        // save latest key frames
		  // save latest key frames
		PointTypePose latestKeyPose6D = copy_cloudKeyPoses6D->points[latestFrameIndex];

		auto itCor = heightCloudKeyFrames.find(latestFrameKeyID);
		if (itCor != heightCloudKeyFrames.end())
		{
			*latestKeyFrameCloud += *transformPointCloud(heightCloudKeyFrames.at(latestFrameKeyID), &latestKeyPose6D);
		}
		auto itSur = intensityCloudKeyFrames.find(latestFrameKeyID);
		if (itSur != intensityCloudKeyFrames.end())
		{
			*latestKeyFrameCloud += *transformPointCloud(intensityCloudKeyFrames.at(latestFrameKeyID), &latestKeyPose6D);
		}
        
        // save history near key frames
        for (int i = -historyKeyframeSearchNum; i <= historyKeyframeSearchNum; ++i)
        {
			int thisIndex = closestHistoryFrameIndex + i;
			if (thisIndex < 0 || thisIndex >= latestFrameIndex) // 索引内存在一定范围内；
                continue;

			int thisKeyID = copy_cloudKeyPoses6D->points[thisIndex].intensity;
            if (thisKeyID < 0 || thisKeyID >= latestFrameKeyID) // id在一定范围内；
                continue;

			PointTypePose historyKeyPose6D = copy_cloudKeyPoses6D->points[thisIndex];
			auto itCor = heightCloudKeyFrames.find(thisKeyID);
			if (itCor != heightCloudKeyFrames.end())
			{
				*nearHistoryKeyFrameCloud += *transformPointCloud(heightCloudKeyFrames.at(thisKeyID), &historyKeyPose6D);
			}
			auto itSur = intensityCloudKeyFrames.find(thisKeyID);
			if (itSur != intensityCloudKeyFrames.end())
			{
				*nearHistoryKeyFrameCloud += *transformPointCloud(intensityCloudKeyFrames.at(thisKeyID), &historyKeyPose6D);
			}
        }

        if (nearHistoryKeyFrameCloud->points.empty())
            return false;

		currLoopKeyContainer.push_back(std::make_pair(latestFrameKeyID, closestHistoryFrameID));
        *latestID = latestFrameKeyID;
        *closestID = closestHistoryFrameID;

        return true;
    }

	bool detectLoopClosure_WB2(int *latestID, int *closestID, PointTypePose *closestKeyPose6D)
	{
		int latestFrameIndex = copy_cloudKeyPoses3D->size()-1;
        int latestFrameKeyID = copy_cloudKeyPoses3D->points[latestFrameIndex].intensity;
        int closestHistoryFrameID = -1, closestHistoryFrameIndex = -1;
		cv::BFMatcher matcher(cv::NORM_HAMMING);
		TicToc loop_time;
		DBoW3::QueryResults ret;
		std::vector<DBoW3::QueryResults> ret_queue;
		std::vector<int> index_sort;
		wordbag_loop.query(DoWB_key_frames.at(latestFrameIndex).description_dowb,ret,select_DoWB_num, latestFrameIndex - 50);
		if(ret.size()<=1)
		{
			return false;
		}
		bool find_loop = false;
		if (ret.size() >= 1 && ret[0].Score > MIN_LOOP_BOW_TH)
		{
			for (unsigned int i = 0; i < ret.size(); i++)
			{
				if (ret[i].Score > MIN_LOOP_BOW_TH)
				{          
					find_loop = true;
				}
				index_sort.push_back(ret[i].Id);
			}
		}
		else
		{
			return false;
		}
		int min_index = -1;
		// std::queue<int> id_deque;
		float min_source;
		if(find_loop)
		{
			for (int i = 0; i < index_sort.size() - 1; i++)
			{
				for(int j = 0; j < index_sort.size() - 1 - i; j++)
				{
					int temp_sort;
					if(index_sort[j] > index_sort[j + 1])
					{
						temp_sort = index_sort[j + 1];
						index_sort[j + 1] = index_sort[j];
						index_sort[j] = temp_sort;
					}
				}
			}
			min_index = index_sort[0];
		}
		else
		{
			return false;
		}
		if(min_index == -1)
		{
			return false;
		}
	
		int linliers;
		int max_linliers = -1;

		int index_num = 0;
		while (index_num < index_sort.size())
		{
			vector<cv::DMatch> matches, good_matches;
			matcher.match(DoWB_key_frames.at(latestFrameIndex).description,DoWB_key_frames.at(index_sort[index_num]).description,matches);
			std::vector<bool> vbInliers;
			gms_matcher gms_height(DoWB_key_frames.at(latestFrameIndex).keypoints, DoWB_key_frames.at(latestFrameIndex).BEV_image.size(), DoWB_key_frames.at(index_sort[index_num]).keypoints, DoWB_key_frames.at(index_sort[index_num]).BEV_image.size(), matches);
			linliers = gms_height.GetInlierMask(vbInliers, false, false);
			for(int i = 0; i < vbInliers.size(); i++)
			{
				if(vbInliers[i] == true)
				{
					good_matches.push_back(matches[i]);
				}
			}
			vector<cv::Point2f> matched_2d_cur, matched_2d_old;
			for (size_t i=0; i < good_matches.size(); i++)
			{
				int cur_index = good_matches[i].queryIdx;
				matched_2d_cur.push_back(DoWB_key_frames.at(latestFrameIndex).keypoints[cur_index].pt);
				
				int old_index = good_matches[i].trainIdx;
				matched_2d_old.push_back(DoWB_key_frames.at(index_sort[index_num]).keypoints[old_index].pt);
			}

			if(linliers > max_linliers && distributionValidation(matched_2d_cur, matched_2d_old))
			{
				max_linliers = linliers;
				min_index = index_sort[index_num];
				if(max_linliers > 150)
				{
					break;

				}
			}
			index_num++;
		}
		
		if(max_linliers < 150)
		{
			return false;
		}
		printf("--laserMapping: word bag num: %d ,point_num: %d , spendtime:%f!!! \n\n\n",min_index, max_linliers, loop_time.toc());
		// printf("--laserMapping: word bag num: %d ,point_num: %d , spendtime:%f!!! \n\n\n",min_index, max_linliers, loop_time.toc());
		// currLoopKeyContainer.push_back(std::make_pair(latestFrameKeyID, min_index));
		// visualizeLoopConstraintEdge();
		int min_keyID = copy_cloudKeyPoses3D->points[min_index].intensity;
		
		
		if (abs(travel_distance.at(min_keyID) - travel_distance.rbegin()->second) > (loopKeyframeDisDiff + poseGraphSearchRadius))
		{
			closestHistoryFrameID = min_keyID;
			closestHistoryFrameIndex = min_index;
			*closestKeyPose6D = copy_cloudKeyPoses6D->points[closestHistoryFrameID];
		}
		else
		{
			closestHistoryFrameID = -1;
		}
		// }
		// ROS_ERROR("333333333333333333");
        if (closestHistoryFrameID == -1 || closestHistoryFrameIndex == -1)
            return false;

        if (latestFrameKeyID == closestHistoryFrameID)
            return false;
		
		// currLoopKeyContainer.push_back(std::make_pair(latestFrameKeyID, min_index));
		// visualizeLoopConstraintEdge();
        // save latest key frames
		  // save latest key frames
		PointTypePose latestKeyPose6D = copy_cloudKeyPoses6D->points[latestFrameIndex];

		auto itCor = heightCloudKeyFrames.find(latestFrameKeyID);
		if (itCor != heightCloudKeyFrames.end())
		{
			*latestKeyFrameCloud += *transformPointCloud(heightCloudKeyFrames.at(latestFrameKeyID), &latestKeyPose6D);
		}
		auto itSur = intensityCloudKeyFrames.find(latestFrameKeyID);
		if (itSur != intensityCloudKeyFrames.end())
		{
			*latestKeyFrameCloud += *transformPointCloud(intensityCloudKeyFrames.at(latestFrameKeyID), &latestKeyPose6D);
		}
        
        // save history near key frames
        for (int i = -historyKeyframeSearchNum; i <= historyKeyframeSearchNum; ++i)
        {
			int thisIndex = closestHistoryFrameIndex + i;
			if (thisIndex < 0 || thisIndex >= latestFrameIndex) // 索引内存在一定范围内；
                continue;

			int thisKeyID = copy_cloudKeyPoses6D->points[thisIndex].intensity;
            if (thisKeyID < 0 || thisKeyID >= latestFrameKeyID) // id在一定范围内；
                continue;

			PointTypePose historyKeyPose6D = copy_cloudKeyPoses6D->points[thisIndex];
			auto itCor = heightCloudKeyFrames.find(thisKeyID);
			if (itCor != heightCloudKeyFrames.end())
			{
				*nearHistoryKeyFrameCloud += *transformPointCloud(heightCloudKeyFrames.at(thisKeyID), &historyKeyPose6D);
			}
			auto itSur = intensityCloudKeyFrames.find(thisKeyID);
			if (itSur != intensityCloudKeyFrames.end())
			{
				*nearHistoryKeyFrameCloud += *transformPointCloud(intensityCloudKeyFrames.at(thisKeyID), &historyKeyPose6D);
			}
        }

        if (nearHistoryKeyFrameCloud->points.empty())
            return false;

        *latestID = latestFrameKeyID;
        *closestID = closestHistoryFrameID;

        return true;
    }
	
	void P2PRANSAC(const std::vector<cv::Point2f> &matched_2d_old, const vector<cv::Point2f> &matched_2d_cur, std::vector<uchar> &inliers)
	{
		Mat H = findHomography(matched_2d_cur, matched_2d_old, inliers, RANSAC);
	}
	
	bool distributionValidation(const vector<cv::Point2f>& new_point_2d_uv, const vector<cv::Point2f>& old_point_2d_uv)
	{
		if (new_point_2d_uv.empty())
			return false;

		pcl::PointCloud<pcl::PointXYZ>::Ptr new_cloud(new pcl::PointCloud<pcl::PointXYZ> ());
		for (size_t i = 0; i < new_point_2d_uv.size(); ++i)
		{
			pcl::PointXYZ p;
			p.x = new_point_2d_uv[i].x;
			p.y = new_point_2d_uv[i].y;
			p.z = 0;
			new_cloud->push_back(p);
		}

		pcl::PointCloud<pcl::PointXYZ>::Ptr old_cloud(new pcl::PointCloud<pcl::PointXYZ> ());
		for (size_t i = 0; i < old_point_2d_uv.size(); ++i)
		{
			pcl::PointXYZ p;
			p.x = old_point_2d_uv[i].x;
			p.y = old_point_2d_uv[i].y;
			p.z = 0;
			old_cloud->push_back(p);
		}

		Eigen::Vector4f new_xyz_centroid;
		Eigen::Matrix3f new_covariance_matrix;
		pcl::compute3DCentroid(*new_cloud, new_xyz_centroid);
		pcl::computeCovarianceMatrix(*new_cloud, new_xyz_centroid, new_covariance_matrix); 

		Eigen::Vector4f old_xyz_centroid;
		Eigen::Matrix3f old_covariance_matrix;
		pcl::compute3DCentroid(*old_cloud, old_xyz_centroid);
		pcl::computeCovarianceMatrix(*old_cloud, old_xyz_centroid, old_covariance_matrix);

		float new_cov_x = sqrt(new_covariance_matrix(0,0));
		float new_cov_y = sqrt(new_covariance_matrix(1,1));
		float old_cov_x = sqrt(old_covariance_matrix(0,0));
		float old_cov_y = sqrt(old_covariance_matrix(1,1));
		float cov_x_diff = abs(new_cov_x - old_cov_x);
		float cov_y_diff = abs(new_cov_y - old_cov_y);
		if (cov_x_diff > 3 * std::min(new_cov_x, old_cov_x) || cov_y_diff > 0.75 * std::min(new_cov_y, old_cov_y))
		{
			return false;
		}

		return true;
	}

	void visualizeLoopConstraintEdge()
    {
        visualization_msgs::MarkerArray markerArray;
        // loop nodes
        visualization_msgs::Marker markerNode;
        markerNode.header.frame_id = "world";
        markerNode.header.stamp = ros::Time().fromSec(timeLaserOdometry);
        markerNode.action = visualization_msgs::Marker::ADD;
        markerNode.type = visualization_msgs::Marker::SPHERE_LIST;
        markerNode.ns = "loop_nodes";
        markerNode.id = 0;
        markerNode.pose.orientation.w = 1;
        markerNode.scale.x = 0.4; markerNode.scale.y = 0.4; markerNode.scale.z = 0.4; 
        markerNode.color.r = 0; markerNode.color.g = 0.8; markerNode.color.b = 1;
        markerNode.color.a = 1;

        // loop edges
        visualization_msgs::Marker markerEdge;
        markerEdge.header.frame_id = "world";
        markerEdge.header.stamp = ros::Time().fromSec(timeLaserOdometry);
        markerEdge.action = visualization_msgs::Marker::ADD;
        markerEdge.type = visualization_msgs::Marker::LINE_LIST;
        markerEdge.ns = "loop_edges";
        markerEdge.id = 1;
        markerEdge.pose.orientation.w = 1;
        markerEdge.scale.x = 0.2; markerEdge.scale.y = 0.2; markerEdge.scale.z = 0.2;
        markerEdge.color.r = 0.9; markerEdge.color.g = 0.9; markerEdge.color.b = 0;
        markerEdge.color.a = 1;

        for (auto it = currLoopKeyContainer.begin(); it != currLoopKeyContainer.end(); ++it)
        {
            int loopNewKeyID = it->first;
            int loopOldKeyID = it->second;
			PointTypePose loopNewPose6D, loopOldPose6D;

			auto itNew = KeyPose6D.find(loopNewKeyID);
			if (itNew != KeyPose6D.end())
			{
				loopNewPose6D = KeyPose6D.at(loopNewKeyID);
			}
			else
			{
				continue;
			}

			auto itOld = KeyPose6D.find(loopOldKeyID);
			if (itOld != KeyPose6D.end())
			{
				loopOldPose6D = KeyPose6D.at(loopOldKeyID);
			}
			else
			{
				continue;
			}

            geometry_msgs::Point p;
            p.x = loopNewPose6D.x;
            p.y = loopNewPose6D.y;
            p.z = loopNewPose6D.z;
            markerNode.points.push_back(p);
            markerEdge.points.push_back(p);
            p.x = loopOldPose6D.x;
            p.y = loopOldPose6D.y;
            p.z = loopOldPose6D.z;
            markerNode.points.push_back(p);
            markerEdge.points.push_back(p);
        }

        markerArray.markers.push_back(markerNode);
        markerArray.markers.push_back(markerEdge);

        pubLoopConstraintEdge.publish(markerArray);
    }

	void PoseGraphOptimize4DoF()
	{
		if (cloudKeyPoses3D->empty()) return;

		// 如果没有闭环或者全局约束发生的话，则不进行位姿图优化；
		if (bLoopIsClosed == false && bGNSSIsAdded == false) return;

		// 将最新帧的key作为最大长度；
		int max_length = copy_cloudKeyPoses6D->back().intensity + 1; 

		double t_array[max_length][3];
		double euler_array[max_length][3];
		Eigen::Quaterniond q_array[max_length];

		double t_loop[3]={0};

		ceres::Problem problem;
		ceres::LocalParameterization* angle_local_parameterization = AngleLocalParameterization::Create();

		int odomFactorNum = 0, loopFactorNum = 0, oldestLoopKey = 100000;
		int keyPoseNum = copy_cloudKeyPoses6D->size();

		// 找到key值最小的闭环帧；
		for (int i = 0; i < keyPoseNum; ++i)
		{
			PointTypePose KeyPoses6D = copy_cloudKeyPoses6D->points[i]; // 取出点；
			int keyI = KeyPoses6D.intensity; // 取出key；

			if (loopInfoContainer.find(keyI) != loopInfoContainer.end()) // 当前帧存在闭环约束；
			{
				// 提取闭环信息；
				loopInfo loopInfoTmp = loopInfoContainer.at(keyI);
				int key_loop = loopInfoTmp.key_loop;

				// 判断闭环帧是否也存在于位姿图中，确保闭环约束正确性；
				if (correctedKeyPose6DByLoop.find(key_loop) != correctedKeyPose6DByLoop.end())
				{
					if (key_loop < oldestLoopKey)
						oldestLoopKey = key_loop;
				}			
			}
		}

		// 添加变量节点和约束因子；
		for (int i = 0; i < keyPoseNum; ++i)
		{
			PointTypePose KeyPoses6D = copy_cloudKeyPoses6D->points[i]; // 取出点；
			int keyI = KeyPoses6D.intensity; // 取出key；

			t_array[keyI][0] = KeyPoses6D.x;
			t_array[keyI][1] = KeyPoses6D.y;
			t_array[keyI][2] = KeyPoses6D.z;
			euler_array[keyI][0] = KeyPoses6D.yaw * rad2deg;
			euler_array[keyI][1] = KeyPoses6D.pitch * rad2deg;
			euler_array[keyI][2] = KeyPoses6D.roll * rad2deg;
			q_array[keyI] = Utility::ypr2R(Eigen::Vector3d(KeyPoses6D.yaw, KeyPoses6D.pitch, KeyPoses6D.roll) * rad2deg);
			
			// 添加所有变量节点；
			problem.AddParameterBlock(euler_array[keyI], 1, angle_local_parameterization); //1个角度
			problem.AddParameterBlock(t_array[keyI], 3); //3个平移

			if(i == 0) continue;

			// 添加里程计约束因子；
			PointTypePose KeyPoses6DFrom = copy_cloudKeyPoses6D->points[i-1]; // 取出上一帧；
			int keyFrom = KeyPoses6DFrom.intensity; // 取出key；
			Eigen::Vector3d t_w_ij(t_array[keyI][0] - t_array[keyFrom][0], t_array[keyI][1] - t_array[keyFrom][1], t_array[keyI][2] - t_array[keyFrom][2]);
			Eigen::Vector3d t_i_ij = q_array[keyFrom].inverse() * t_w_ij; // 当前帧相对于上一帧的位姿；局部增量；以上帧位姿为参考坐标系；
			double relative_yaw = euler_array[keyI][0] - euler_array[keyFrom][0];
			ceres::CostFunction* cost_function = FourDOFError::Create(t_i_ij.x(), t_i_ij.y(), t_i_ij.z(), relative_yaw, KeyPoses6DFrom.pitch * rad2deg, KeyPoses6DFrom.roll * rad2deg );
			problem.AddResidualBlock(cost_function, NULL, euler_array[keyFrom], t_array[keyFrom], euler_array[keyI], t_array[keyI]);

			// 添加闭环约束因子；
			if (loopInfoContainer.find(keyI) != loopInfoContainer.end()) // 当前帧存在闭环约束；
			{
				// 提取闭环信息；
				loopInfo loopInfoTmp = loopInfoContainer.at(keyI);
				int key_loop = loopInfoTmp.key_loop;
				PointTypePose keyPose6DLoop = loopInfoTmp.keyPose6DLoop;
				Eigen::Vector3d t_loop_curr = loopInfoTmp.t_loop_curr.cast<double>();
				Eigen::Quaterniond q_loop_curr = loopInfoTmp.q_loop_curr.cast<double>();

				// 提取闭环信息后，判断闭环帧是否也存在于位姿图中，确保闭环约束正确性；
				if (correctedKeyPose6DByLoop.find(key_loop) != correctedKeyPose6DByLoop.end()) 
				{
					// 添加闭环约束；
					Eigen::Vector3d euler_loop_curr = Utility::R2ypr(q_loop_curr.toRotationMatrix());
					double relative_yaw = euler_loop_curr.x();
					ceres::CostFunction* cost_function = FourDOFError::Create(t_loop_curr.x(), t_loop_curr.y(), t_loop_curr.z(), relative_yaw, keyPose6DLoop.pitch * rad2deg, keyPose6DLoop.roll * rad2deg);
					problem.AddResidualBlock(cost_function, NULL, euler_array[key_loop], t_array[key_loop], euler_array[keyI], t_array[keyI]);

					loopFactorNum++;
					printf("--PoseGraph: add Loop Factor between %d and %d, count: %d \n", keyI, key_loop, loopFactorNum );
				}
			}
			
			odomFactorNum++;
		}

		printf("--PoseGraph: add odom Factors num: %d \n", odomFactorNum );

		// 将最小闭环帧位设为固定；
		if (oldestLoopKey < 99999)
		{   
			//将闭环回环帧的位姿设置为固定；
			problem.SetParameterBlockConstant(euler_array[oldestLoopKey]);
			problem.SetParameterBlockConstant(t_array[oldestLoopKey]);
			printf("--PoseGraph: Set loop keyframe Constant: keyID: %d, pos: x: %.3f, y: %.3f, z: %.3f \n", oldestLoopKey, t_array[oldestLoopKey][0], t_array[oldestLoopKey][1], t_array[oldestLoopKey][2]);
			t_loop[0] = t_array[oldestLoopKey][0];
			t_loop[1] = t_array[oldestLoopKey][1];
			t_loop[2] = t_array[oldestLoopKey][2];
		}
		else
		{
			ROS_WARN("\n\n\n--PoseGraph: not find oldestLoopKey, pose grapgh not be optimized !!!");
			return;
		}

		if (RMODE == 0 && map_update == 1)
		{
			for (int i = 0; i < loaded_map_size; i++)
			{
				// 将闭环回环帧的位姿设置为固定；
				problem.SetParameterBlockConstant(euler_array[i]);
				problem.SetParameterBlockConstant(t_array[i]);
			}
		}
		
		// 开始求解；
		TicToc t_solve;
		ceres::Solver::Options options;
		options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;  
		options.max_num_iterations = 10;
		ceres::Solver::Summary summary;
		ceres::Solve(options, &problem, &summary);
		printf("--PoseGraph: ceres::Solve time: %.1f \n", t_solve.toc());

		// 将优化校正后的位姿保存起来，供建图节点用；
		correctedKeyPose6DByLoop.clear();
		for (int i = 0; i < keyPoseNum; ++i)
		{
			PointTypePose KeyPoses6DCorrected = copy_cloudKeyPoses6D->points[i]; // 取出点；
			int keyI = KeyPoses6DCorrected.intensity; // 取出key；

			if (keyI == oldestLoopKey)
			{
				// 确认闭环帧的位姿固定不变；
				// 测试中发现ceres一个bug：第一帧位姿为0，当设定第一帧位姿固定，优化后第一帧发生了变化，导致整个位姿图发生了移动；
				if(abs(t_loop[0] - t_array[keyI][0]) > 0.01 || abs(t_loop[1] - t_array[keyI][1]) > 0.01 || abs(t_loop[2] - t_array[keyI][2]) > 0.01)
				{
					ROS_WARN("\n\n\n--PoseGraph: loop pose changed !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
					ROS_WARN("--PoseGraph: loop pose before: x: %.3f, y: %.3f, z: %.3f ", t_loop[0], t_loop[1], t_loop[2]);
					ROS_WARN("--PoseGraph: loop pose after : x: %.3f, y: %.3f, z: %.3f ", t_array[keyI][0], t_array[keyI][0], t_array[keyI][0] );
					return;
				}
			}

			KeyPoses6DCorrected.x = t_array[keyI][0];
			KeyPoses6DCorrected.y = t_array[keyI][1];
			KeyPoses6DCorrected.z = t_array[keyI][2];
			KeyPoses6DCorrected.yaw = euler_array[keyI][0] * deg2rad;
			KeyPoses6DCorrected.pitch = euler_array[keyI][1] * deg2rad;
			KeyPoses6DCorrected.roll = euler_array[keyI][2] * deg2rad;

			copy_cloudKeyPoses3D->points[i].x = KeyPoses6DCorrected.x;
			copy_cloudKeyPoses3D->points[i].y = KeyPoses6DCorrected.y;
			copy_cloudKeyPoses3D->points[i].z = KeyPoses6DCorrected.z;
			copy_cloudKeyPoses6D->points[i] = KeyPoses6DCorrected;

			correctedKeyPose6DByLoop[keyI] = KeyPoses6DCorrected; // 将校正后的位姿保存起来，供建图节点用；
		}

		bKeyFramePoseGraphUpdated = true;
	}

	void publishGlobalPoseGraph()
	{
		globalPoseGraphPath.header.stamp = ros::Time().fromSec( timeLaserOdometry );
		globalPoseGraphPath.header.frame_id = "camera_init";
		pubGlobalPoseGraphPath.publish(globalPoseGraphPath);

		sensor_msgs::PointCloud2 ROSCloudMsg;
		pcl::toROSMsg(*copy_cloudKeyPoses3D, ROSCloudMsg);
		ROSCloudMsg.header.stamp = ros::Time().fromSec(timeLaserOdometry);
		ROSCloudMsg.header.frame_id = "camera_init";
		pubGlobalPoseGraphPoint.publish(ROSCloudMsg);
	}

	void publishGlobalMap(void)
    {
        if (cloudKeyPoses3D->points.empty() == true) return;

		TicToc t_pubGlobalMap;
        pcl::PointCloud<PointType>::Ptr globalMapKeyPoses(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapKeyPosesDS(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMap(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapDS(new pcl::PointCloud<PointType>());
		pcl::PointCloud<PointType>::Ptr globalMapScan(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapScanDS(new pcl::PointCloud<PointType>());

		int keyNum = copy_cloudKeyPoses3D->size();
        for (int i = 0; i < keyNum; ++i)
		{
			globalMapKeyPoses->push_back(copy_cloudKeyPoses3D->points[i]);
		}

		//pcl::PointCloud<int> keypointIndices;
		pcl::UniformSampling<PointType> UniformSamplingGlobalMapKeyPoses;
		UniformSamplingGlobalMapKeyPoses.setInputCloud(globalMapKeyPoses);
		UniformSamplingGlobalMapKeyPoses.setRadiusSearch(globalMapVisualizationPoseDensity);
		// UniformSamplingGlobalMapKeyPoses.compute(keypointIndices);
		
        UniformSamplingGlobalMapKeyPoses.filter(*globalMapKeyPosesDS);
		//pcl::copyPointCloud(*globalMapKeyPoses, keypointIndices.points, *globalMapKeyPosesDS);
        // extract visualized and downsampled key frames
        for (int i = 0; i < (int)globalMapKeyPosesDS->size(); ++i)
		{
			PointTypePose keyPose6D = copy_cloudKeyPoses6D->points[globalMapKeyPosesDS->points[i].intensity];
            int thisKeyInd = (int)keyPose6D.intensity;

			auto itCor = heightCloudKeyFrames.find(thisKeyInd);
			if (itCor != heightCloudKeyFrames.end())
			{
				*globalMap += *transformPointCloud(heightCloudKeyFrames.at(thisKeyInd), &keyPose6D);
			}
			auto itSur = intensityCloudKeyFrames.find(thisKeyInd);
			if (itSur != intensityCloudKeyFrames.end())
			{
				*globalMap += *transformPointCloud(intensityCloudKeyFrames.at(thisKeyInd), &keyPose6D);
			}
			// auto itScan = scanCloudKeyFrames.find(thisKeyInd);
			// if (itScan != scanCloudKeyFrames.end())
			// {
			// 	*globalMapScan += *transformPointCloud(scanCloudKeyFrames.at(thisKeyInd), &keyPose6D);
			// }
        }

        // downsample visualized points
        pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyFrames; // for global map visualization
        downSizeFilterGlobalMapKeyFrames.setLeafSize(globalMapDensity, globalMapDensity, globalMapDensity); // for global map visualization
        downSizeFilterGlobalMapKeyFrames.setInputCloud(globalMap);
        downSizeFilterGlobalMapKeyFrames.filter(*globalMapDS);
		pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyFramescan; // for global map visualization
		downSizeFilterGlobalMapKeyFramescan.setInputCloud(globalMapScan);
        downSizeFilterGlobalMapKeyFramescan.filter(*globalMapScanDS);

		sensor_msgs::PointCloud2 laserCloudMsg;
		pcl::toROSMsg(*globalMapDS, laserCloudMsg);
		laserCloudMsg.header.stamp = ros::Time().fromSec(timeLaserOdometry);
		laserCloudMsg.header.frame_id = "camera_init";
		pubLaserCloudMap.publish(laserCloudMsg);

		// sensor_msgs::PointCloud2 laserCloudMsgScan;
		// pcl::toROSMsg(*globalMapScanDS, laserCloudMsgScan);
		// laserCloudMsgScan.header.stamp = ros::Time().fromSec(timeLaserOdometry);
		// laserCloudMsgScan.header.frame_id = "camera_init";
		// pubScanCloudMap.publish(laserCloudMsgScan);

		sensor_msgs::PointCloud2 KeyPosesMsg;
		pcl::toROSMsg(*globalMapKeyPoses, KeyPosesMsg);
		KeyPosesMsg.header.stamp = ros::Time().fromSec(timeLaserOdometry);
		KeyPosesMsg.header.frame_id = "camera_init";
		pubGlobalMapKeyPose.publish(KeyPosesMsg);

		sensor_msgs::PointCloud2 KeyPosesDSMsg;
		pcl::toROSMsg(*globalMapKeyPosesDS, KeyPosesDSMsg);
		KeyPosesDSMsg.header.stamp = ros::Time().fromSec(timeLaserOdometry);
		KeyPosesDSMsg.header.frame_id = "camera_init";
		pubGlobalMapKeyPoseDS.publish(KeyPosesDSMsg);

		int keyDSNum = globalMapKeyPosesDS->size();
		//printf("--GlobalMap: construct and pub GlobalMap time: %.1f, keyNum: %d, keyDSNum: %d \n", t_pubGlobalMap.toc(), keyNum, keyDSNum);
    }

	pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose* transformIn)
    {
        pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

        PointType *pointFrom;
        int cloudSize = cloudIn->size();
        cloudOut->resize(cloudSize);
		Eigen::Quaterniond q_temp;
		Eigen::Vector3d t_temp(transformIn->x, transformIn->y, transformIn->z);
		q_temp = Utility::ypr2R(Eigen::Vector3d(transformIn->yaw, transformIn->pitch, transformIn->roll) * rad2deg);
		for (int i = 0; i < cloudSize; ++i)
		{
			pointFrom = &cloudIn->points[i];
			Eigen::Vector3d point_curr(pointFrom->x, pointFrom->y, pointFrom->z);
			Eigen::Vector3d point_w = q_temp * point_curr + t_temp;
			cloudOut->points[i].x = point_w.x();
			cloudOut->points[i].y = point_w.y();
			cloudOut->points[i].z = point_w.z();
			cloudOut->points[i].intensity = pointFrom->intensity;
		}
        return cloudOut;
    }

	Eigen::Affine3f pclPointToAffine3f(PointTypePose thisPoint)
    { 
        return pcl::getTransformation(thisPoint.x, thisPoint.y, thisPoint.z, thisPoint.roll, thisPoint.pitch, thisPoint.yaw);
    }

	void TransformToWorld(PointType const *const pi, PointType *const po)
    {
        // 使用当前优化得到的q、t将当前lidar点云向上一帧点云坐标系下转化
        Eigen::Vector3d point(pi->x, pi->y, pi->z);
        Eigen::Vector3d un_point = q_w_curr * point + t_w_curr;

        po->x = un_point.x();
        po->y = un_point.y();
        po->z = un_point.z();
        po->intensity = pi->intensity;
    }
};


int main(int argc, char **argv)
{
	ros::init(argc, argv, "laserMapping");

	LaserMapping LaserMapper;

	ros::spin();

	return 0;
}

