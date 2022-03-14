#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

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

  // show
  const float r = 5;
  for (int i = 0; i < mvKeys.size(); i++) {
    cv::Point2f pt1, pt2;
    pt1.x = mvKeys[i].pt.x - r;
    pt1.y = mvKeys[i].pt.y - r;
    pt2.x = mvKeys[i].pt.x + r;
    pt2.y = mvKeys[i].pt.y + r;
    cv::rectangle(img, pt1, pt2, cv::Scalar(0, 255, 0));
    cv::circle(img, mvKeys[i].pt, 2, cv::Scalar(0, 255, 0), -1);
  }
  cv::imshow("orb_detector", img);
  cv::waitKey(0);

  return 0;
}
