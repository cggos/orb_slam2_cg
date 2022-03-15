/**
 * @file test_orb_detector.cpp
 * @author cggos (cggos@outlook.com)
 * @brief
 *      build: cmake -DWITH_ORB_C=ON ..
 *      run: ./test_orb_detector
 * @version 0.1
 * @date 2022-03-15
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <fstream>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>

#include "ORBextractor.h"

using namespace ORB_SLAM2;

int main() {
  std::cout << "test orb_detector" << std::endl;

  cv::Mat img, img_gray;
  img = cv::imread("../data/lena.bmp");
  if (img.empty()) {
    std::cerr << "img empty!!!" << std::endl;
    return -1;
  }
  cv::cvtColor(img, img_gray, CV_BGR2GRAY);

  ORBextractor* mpORBextractor;
  {
    int nFeatures = 1000;
    float fScaleFactor = 1.2;
    int nLevels = 8;
    int fIniThFAST = 20;
    int fMinThFAST = 7;
    mpORBextractor = new ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);
  }

  // main
  std::vector<cv::KeyPoint> mvKeys;
  cv::Mat mDescriptors;
  (*mpORBextractor)(img_gray, cv::Mat(), mvKeys, mDescriptors);

  // show and save
  std::cout << "mvKeys: " << mvKeys.size() << std::endl;

  std::vector<std::pair<float, cv::Point2f*>> v_res_pt;
  v_res_pt.reserve(mvKeys.size());

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

    v_res_pt.push_back(std::make_pair(mvKeys[i].response, &mvKeys[i].pt));
  }

  // key_pts.raw: origin ORB Detector with lena.bmp
  // key_pts.new: new alg.
  std::string str_file;
#ifdef WITH_ORB_C
  str_file = "../data/key_pts.new";
#else
  str_file = "../data/key_pts.raw";
#endif
  std::ofstream of;
  of.open(str_file.c_str());
  std::sort(v_res_pt.begin(), v_res_pt.end());
  for (int i = 0; i < mvKeys.size(); i++) of << *(v_res_pt[i].second) << std::endl;
  if (of.is_open()) of.close();

  cv::imshow("orb_detector", img);
  cv::waitKey(0);

  return 0;
}
