# orbslam2_cg

modified version from [raulmur/ORB_SLAM2](https://github.com/raulmur/ORB_SLAM2) (commit f2e6f51  on Oct 11, 2017)  

[ORB-SLAM](http://webdiis.unizar.es/~raulmur/orbslam/) is a versatile and accurate SLAM solution for Monocular, Stereo and RGB-D cameras.

* [ethz-asl/orb_slam_2_ros](https://github.com/ethz-asl/orb_slam_2_ros)
* [ORB-Slam2: Implementation on my Ubuntu 16.04 with ROS Kinect](https://medium.com/@j.zijlmans/orb-slam-2052515bd84c)

-----

[TOC]

# Dependencies

* OpenCV 3
* Eigen3
* Pangolin
  ```bash
  git clone https://github.com/stevenlovegrove/Pangolin.git
  cd Pangolin & mkdir build & cd build
  cmake .. & cmake --build .
  ```
* DBoW2 and G2O
  - Dowload from [ORB_SLAM2/Thirdparty/](https://github.com/raulmur/ORB_SLAM2/tree/master/Thirdparty) and move them to `orbslam2_cg/Thirdparty`
* ROS (optional)

# Build

```bash
cd orbslam2_cg/scripts
./build.sh
```

# Run

```bash
cd orbslam2_cg/scripts
./run_<mono_tum>.sh  # modify it before run
```
