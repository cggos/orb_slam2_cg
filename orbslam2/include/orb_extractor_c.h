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

// #include <list>
#include <opencv2/core/core.hpp>
// #include <vector>

#include "link_list.h"

// clang-format off
#define SAFE_DELETE(p)       { if(p) { delete (p);     (p)=NULL; } }
#define SAFE_DELETE_ARRAY(p) { if(p) { delete[] (p);   (p)=NULL; } }
#define SAFE_RELEASE(p)      { if(p) { (p)->Release(); (p)=NULL; } }
// clang-format on
#define SAFE_DELETE_2POINTERS(p, sz) \
  for (int i = 0; i < sz; i++)       \
    if ((p)[i]) {                    \
      delete (p)[i];                 \
      (p)[i] = nullptr;              \
    }                                \
  if (p) {                           \
    delete (p);                      \
    (p) = nullptr;                   \
  }

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
  float x = 0.f;
  float y = 0.f;
  float size = 0.f;
  float angle = 0.f;
  float response = 0.f;
  int octave = 0;

  KeyPointCG() {}

  KeyPointCG &operator=(const KeyPointCG &rhs) {
    if (this == &rhs) return *this;
    x = rhs.x;
    y = rhs.y;
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
  List<ExtractorNodeCG>::iterator lit;

  ExtractorNodeCG() {}

  void DivideNode(ExtractorNodeCG &n1, ExtractorNodeCG &n2, ExtractorNodeCG &n3, ExtractorNodeCG &n4);
};

KeyPointCG kp_cv2cg(const cv::KeyPoint &kp);
cv::KeyPoint kp_cg2cv(const KeyPointCG &kpcg);

cg::KeyPointCG **distribute_quadtree_c(cg::KeyPointCG *arr_to_dis_keys[],
                                       const int &sz_to_dis_keys,
                                       const int &minX,
                                       const int &maxX,
                                       const int &minY,
                                       const int &maxY,
                                       const int &N,
                                       const int &level,
                                       int &ret_sz);

}  // namespace cg

#endif  // ORBEXTRACTOR_C_H