/**
 * @file orb_extractor_c.h
 * @author cggos (cggos@outlook.com)
 * @brief
 * @version 0.1
 * @date 2022-03-15
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef ORBEXTRACTOR_C_H
#define ORBEXTRACTOR_C_H

#include <list>
#include <opencv2/core/core.hpp>
#include <vector>

namespace cg {

typedef short int Int;

struct Point2I {
  Int x;
  Int y;

  Point2I() : x(0), y(0) {}

  Point2I(Int x, Int y) : x(x), y(y) {}

  Point2I &operator=(const Point2I &rhs) {
    if (this == &rhs) return *this;
    x = rhs.x;
    y = rhs.y;
    return *this;
  }
};

struct KeyPointCG {
  Point2I pt;
  float size;
  float angle;
  float response;
  int octave;

  KeyPointCG &operator=(const KeyPointCG &rhs) {
    if (this == &rhs) return *this;
    pt = rhs.pt;
    size = rhs.size;
    angle = rhs.angle;
    response = rhs.response;
    octave = rhs.octave;
    return *this;
  }
};

struct ExtractorNodeCG {
  bool bNoMore = false;

  Point2I UL, UR, BL, BR;

  Int sz_keys = 0;
  KeyPointCG *ptr_keys = nullptr;

  void push_keypts(const KeyPointCG &kp) {
    assert(ptr_keys == nullptr);
    ptr_keys[sz_keys] = kp;
    sz_keys++;
  }

  // TODO
  // std::vector<cv::KeyPoint> vKeys;
  std::list<ExtractorNodeCG>::iterator lit;

  ExtractorNodeCG() {}

  void DivideNode(ExtractorNodeCG &n1, ExtractorNodeCG &n2, ExtractorNodeCG &n3, ExtractorNodeCG &n4);
};

KeyPointCG kp_cv2cg(const cv::KeyPoint &kp);
cv::KeyPoint kp_cg2cv(const KeyPointCG &kpcg);

std::vector<cv::KeyPoint> distribute_quadtree_c(const std::vector<cv::KeyPoint> &vToDistributeKeys,
                                                const int &minX,
                                                const int &maxX,
                                                const int &minY,
                                                const int &maxY,
                                                const int &N,
                                                const int &level,
                                                const int &nfeatures);

}  // namespace cg

#endif