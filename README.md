# <div align = "center">RGC-SLAM: </div>

## <div align = "center">Robust Ground Constrained SLAM for Mobile Robot With Sparse-Channel LiDAR</div>


> Shaocong Wang, Fengkui Cao, Ting Wang, Shiliang Shao, and Lianqing Liu
>
> [IEEE Transactions on Intelligent Vehicles](https://ieeexplore.ieee.org/abstract/document/10654559)

## News



* **`13 Sept 2024`:**  Code updata
* **`28 Aug 2024`:** Accepted by [IEEE TIV](https://ieeexplore.ieee.org/xpl/RecentIssue.jsp?punumber=7274857)! 

## Getting Started


### Instructions
RGC-SLAM requires an input point cloud of type `sensor_msgs::PointCloud2` with an optional IMU input of type `sensor_msgs::Imu`.

### Dependencies

- Ubuntu 18.04 or 20.04
- ROS Melodic or Noetic (`roscpp`, `std_msgs`, `sensor_msgs`, `geometry_msgs`, `pcl_ros`)
- C++ 14
- OpenMP
- Point Cloud Library
- Eigen >=3.3.4
- Ceres >=1.14

### Compiling

Create a catkin workspace, clone the `ground_msg` and  `rgc_slam`  repository into the `src` folder, and compile via the [`catkin_tools`](https://catkin-tools.readthedocs.io/en/latest/) package (or [`catkin_make`](http://wiki.ros.org/catkin/commands/catkin_make) if preferred):

```sh
mkdir ws && cd ws && mkdir src && catkin init && cd src
git clone https://github.com/ROBOT-WSC/RGC-SLAM.git
catkin_make
```

### Execution

For your convenience, we provide example test data [here](https://drive.google.com/drive/folders/1bt9vWPVgTF8I8JXSUO-Dpi3n2vomG6t9) (4 sequences, `sequence1_to_4.zip`). To run, first launch RGC-SLAM (with default point cloud and IMU topics) via:

```sh
roslaunch rgc_slam run.launch
```

In a separate terminal session, play back the downloaded bag:

```
rosbag play mynteye_stereo_velodyne_wheel_angle_GPS_2020-09-18-15-03-4#-playgroud.bag --clock
```

## Citation

If you find RGC-SLAM is useful in your research or applications, please consider giving us a star 🌟 and citing it by the following BibTeX entry.

```bibtex
@ARTICLE{wang2024rgcslam,
  author={Wang, Shaocong and Cao, Fengkui and Wang, Ting and Shao, Shiliang and Liu, Lianqing},
  journal={IEEE Transactions on Intelligent Vehicles}, 
  title={Robust Ground Constrained SLAM for Mobile Robot With Sparse-Channel LiDAR}, 
  year={2024},
  volume={},
  number={},
  pages={1-12},
  keywords={Laser radar;Simultaneous localization and mapping;Feature extraction;Robots;Degradation;Point cloud compression;Odometry;SLAM;Mobile robot;Ground constraint;Degraded environment;Sparse-channel LiDAR},
  doi={10.1109/TIV.2024.3451137}}
```
## Acknowledgements

We thank the authors of the [FastGICP](https://github.com/SMRT-AIST/fast_gicp) and [A-LOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM) open-source packages.
