/**
 * @file test_orb_detector_mask.cpp
 * @author Gavin Gao (cggos@outlook.com)
 * @brief
 *      build: cmake -DWITH_ORB_C=OFF ..
 *      run: ./test_orb_detector_mask
 * @version 0.1
 * @date 2022-04-14
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>

#include <fstream>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>

#include "read_bag/ORBextractor3.h"

using namespace ORB_SLAM3;

class ReadBag {
 public:
  ReadBag() {
    int nFeatures = 1500;
    float fScaleFactor = 1.2;
    int nLevels = 8;
    int fIniThFAST = 15;
    int fMinThFAST = 7;
    mpORBextractor = new ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);
  }

  ~ReadBag() {}

  void cb(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(msg);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    cv::Mat img, img_gray;
    img = cv_ptr->image.clone();

    if (img.channels() == 3)
      cv::cvtColor(img, img_gray, CV_BGR2GRAY);
    else
      img_gray = img.clone();

    std::vector<cv::KeyPoint> mvKeys;
    cv::Mat mDescriptors;
    int x0 = 0;
    int x1 = 0;
    std::vector<int> vLapping = {x0, x1};
    (*mpORBextractor)(img_gray, cv::Mat(), mvKeys, mDescriptors, vLapping);

    std::cout << "mvKeys size: " << mvKeys.size() << std::endl;

    if (img.channels() == 1) cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);

    const float r = 5;
    for (int i = 0; i < mvKeys.size(); i++) {
      const cv::Point2f& pt = mvKeys[i].pt;
      cv::Point2f pt1, pt2;
      pt1.x = pt.x - r;
      pt1.y = pt.y - r;
      pt2.x = pt.x + r;
      pt2.y = pt.y + r;
      cv::rectangle(img, pt1, pt2, cv::Scalar(0, 255, 0));
      cv::circle(img, pt, 2, cv::Scalar(0, 255, 0), -1);
    }

    cv::imshow("orb_detector with mask", img);
    cv::waitKey(50);
  }

 private:
  ORBextractor* mpORBextractor;
};

int main(int argc, char* argv[]) {
  std::cout << "test orb_detector with mask" << std::endl;

  ros::init(argc, argv, "orb_detector_mask");

  ros::NodeHandle nh("~");

  ReadBag rb;

  ros::Subscriber sub = nh.subscribe("/T265/fisheye1/image_raw", 1000, &ReadBag::cb, &rb);

  // ros::spin();
  while (ros::ok()) {
    ros::spinOnce();
  }

  ros::shutdown();

  return 0;
}
