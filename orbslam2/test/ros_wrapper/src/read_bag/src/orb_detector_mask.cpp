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
    // mask
    mask_ = cv::imread("/home/cg/Pictures/mask_fisheye_848_800_r390.png", 0);  // in test/data dir
  }

  ~ReadBag() {}

  void draw(const std::vector<cv::KeyPoint>& kpts, cv::Mat& img) {
    const float r = 5;
    for (int i = 0; i < kpts.size(); i++) {
      const cv::Point2f& pt = kpts[i].pt;
      cv::Point2f pt1, pt2;
      pt1.x = pt.x - r;
      pt1.y = pt.y - r;
      pt2.x = pt.x + r;
      pt2.y = pt.y + r;
      cv::rectangle(img, pt1, pt2, cv::Scalar(0, 255, 0));
      cv::circle(img, pt, 2, cv::Scalar(0, 255, 0), -1);
    }
    std::stringstream ss;
    ss << "keypoints: " << kpts.size();
    cv::putText(img, ss.str(), cv::Point(20, 30), 0, 1, cv::Scalar(0, 0, 255), 3);
  }

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

    std::vector<cv::KeyPoint> vkpt0_tmp, vkpt0, vkpt1;
    cv::Mat vdesc0, vdesc1;
    int x0 = 0;
    int x1 = 0;
    std::vector<int> vLapping = {x0, x1};
    (*mpORBextractor)(img_gray, mask_, vkpt0, vdesc0, vLapping);
    (*mpORBextractor)(img_gray, cv::Mat(), vkpt1, vdesc1, vLapping);

    // remove points along the circle edge
    int th = 5;
    int r = 390 - th;
    int r2 = r * r;
    cv::Point2f pt_center(424, 400);
    vkpt0_tmp.reserve(vkpt0.size());
    for (const auto& kp : vkpt0) {
      cv::Point2f dp = kp.pt - pt_center;
      float dist = dp.dot(dp);
      if (dist > r2) continue;
      vkpt0_tmp.push_back(kp);
    }
    vkpt0 = vkpt0_tmp;

    std::cout << "====================" << std::endl;
    std::cout << vkpt0.size() << " -- " << vkpt1.size() << std::endl;

    // show
    {
      cv::Mat img0, img1;
      cv::cvtColor(img, img0, cv::COLOR_GRAY2BGR);
      cv::cvtColor(img, img1, cv::COLOR_GRAY2BGR);

      draw(vkpt0, img0);
      draw(vkpt1, img1);

      cv::Mat img_show;
      cv::hconcat(img0, img1, img_show);

      cv::imshow("orb_detector with mask", img_show);
      cv::waitKey(100);
    }
  }

 private:
  ORBextractor* mpORBextractor;
  cv::Mat mask_;
};

int main(int argc, char* argv[]) {
  std::cout << "test orb_detector with mask" << std::endl;

  ros::init(argc, argv, "orb_detector_mask");

  ros::NodeHandle nh("~");

  ReadBag rb;

  ros::Subscriber sub = nh.subscribe("/T265/fisheye1/image_raw", 500, &ReadBag::cb, &rb);

  // ros::spin();
  while (ros::ok()) {
    ros::spinOnce();
  }

  ros::shutdown();

  return 0;
}
