# <div align = "center">BEV-LSLAM: </div>

## <div align = "center">A Novel and Compact BEV LiDAR SLAM for Outdoor Environment</div>


> Shaocong Wang, Fengkui Cao, Xieyuanli Chen, Ting Wang and Lianqing Liu
>
> [IEEE Robotics and Automation Letters](https://ieeexplore.ieee.org/document/10845798)

## News



* **`1 March 2025`:**  Code updata
* **`7 January 2025`:** Accepted by [IEEE RAL](https://ieeexplore.ieee.org/document/10845798)! 

## Getting Started


### Instructions
BEV-LSLAM requires an input point cloud of type `sensor_msgs::PointCloud2`

### Dependencies

- Ubuntu 18.04 or 20.04
- ROS Melodic or Noetic (`roscpp`, `std_msgs`, `sensor_msgs`, `geometry_msgs`, `pcl_ros`)
- cv_bridge
- Opencv
- C++ 14
- OpenMP
- Point Cloud Library
- Eigen >=3.3.4
- Ceres >=1.14

### Compiling

Create a catkin workspace, clone the `imagecloud_msg` and  `orb_lio`  repository into the `src` folder, and compile via the [`catkin_tools`](https://catkin-tools.readthedocs.io/en/latest/) package (or [`catkin_make`](http://wiki.ros.org/catkin/commands/catkin_make) if preferred):

```sh
mkdir ws && cd ws && mkdir src && catkin init && cd src
git clone https://github.com/ROBOT-WSC/BEV-LSLAM.git
catkin_make
```

### Execution

For your convenience, KITTI, Urbanloco and Groundrobot can be test on BEV-LSLAM.(Different datasets have different intensity range， please check the 222 line in scantoscan.cpp before your experiment.) For Groundrobot, we provide example test data [here](https://drive.google.com/drive/folders/1bt9vWPVgTF8I8JXSUO-Dpi3n2vomG6t9). To run, first launch BEV-LSLAM via:

```sh
roslaunch orb_lio orb_lo.launch
```

In a separate terminal session, play back the downloaded bag:

```
rosbag play bag's name --clock
```

## Citation

If you find BEV-LSLAM is useful in your research or applications, please consider giving us a star 🌟 and citing it by the following BibTeX entry.

```bibtex
@ARTICLE{10845798,
  author={Cao, Fengkui and Wang, Shaocong and Chen, Xieyuanli and Wang, Ting and Liu, Lianqing},
  journal={IEEE Robotics and Automation Letters}, 
  title={BEV-LSLAM: A Novel and Compact BEV LiDAR SLAM for Outdoor Environment}, 
  year={2025},
  volume={10},
  number={3},
  pages={2462-2469},
  keywords={Laser radar;Feature extraction;Simultaneous localization and mapping;Point cloud compression;Visualization;Tracking loops;Robots;Optimization;Pose estimation;Pipelines;SLAM;localization;mapping},
  doi={10.1109/LRA.2025.3531727}}
```
## Acknowledgements

We thank the authors of the [FastGICP](https://github.com/SMRT-AIST/fast_gicp), [orb-slam](https://github.com/raulmur/ORB_SLAM2) and [A-LOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM) open-source packages.
