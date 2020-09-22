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
  ```sh
  sudo apt install libeigen3-dev
  ```
* Pangolin
  ```bash
  git clone https://github.com/stevenlovegrove/Pangolin.git
  cd Pangolin & mkdir build & cd build
  cmake .. & cmake --build .
  ```
* G2O
  ```sh
  git clone https://github.com/RainerKuemmerle/g2o
  ```
* DBoW2 
  ```sh
  git clone https://github.com/cggos/DBoW2
  ```
* ROS (optional)
  
* Vocabulary ORBvoc.txt
  ```sh
  cd orbslam2/Vocabulary
  wget https://raw.githubusercontent.com/raulmur/ORB_SLAM2/master/Vocabulary/ORBvoc.txt.tar.gz
  tar -xf ORBvoc.txt.tar.gz
  ```

# Build

* with ROS
  ```sh
  cd orbslam2_cg/platforms/ros_wrapper
  catkin_make -j1
  ```

* without ROS
  ```sh
  cd orbslam2_cg/platforms/app
  mkdir build & cd build
  cmake .. & make -j1
  ```

# Calibration Params

* Stereo Config: **ROS Stereo Calibration** and get data from the result  
  ```sh
  rosrun camera_calibration cameracalibrator.py \
      --approximate=0.05 \
      --size 11x7 \
      --square 0.036 \
      left:=/mynteye/left/image_raw \
      right:=/mynteye/right/image_raw        
  ```

# Run

* without ROS
  ```bash
  cd orbslam2_cg/platforms/app/build
  ../scripts/run_<mono_tum>.sh  # modify it before run
  ```

* with ROS
  ```sh
  roslaunch orbslam2_ros run_<mono>.launch

  roslaunch orbslam2_ros run_stereo_euroc.launch [rviz:=true]

  roslaunch orbslam2_ros run_stereo_mynteye_s1030.launch
  ```

  <p align="center">
    <img src="images/stereo_mynteye_s1030.jpg"/>
  </p>
