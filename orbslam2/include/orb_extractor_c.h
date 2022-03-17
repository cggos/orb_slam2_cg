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
#define SAFE_DELETE(p)       { if(p) { delete (p);     (p)=nullptr; } }
#define SAFE_DELETE_ARRAY(p) { if(p) { delete[] (p);   (p)=nullptr; } }
#define SAFE_RELEASE(p)      { if(p) { (p)->Release(); (p)=nullptr; } }
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

  // KeyPointCG() {}

  // KeyPointCG &operator=(const KeyPointCG &rhs) {
  //   if (this == &rhs) return *this;
  //   x = rhs.x;
  //   y = rhs.y;
  //   size = rhs.size;
  //   angle = rhs.angle;
  //   response = rhs.response;
  //   octave = rhs.octave;
  //   return *this;
  // }
};

class ExtractorNodeCG {
 public:
  ExtractorNodeCG() {}

  ExtractorNodeCG(Int sz_keys_max) : sz_keys_max(sz_keys_max) { ptr_keys = new KeyPointCG[sz_keys_max]; }

  ExtractorNodeCG(const ExtractorNodeCG &rhs) {
    bNoMore = rhs.bNoMore;
    UL = rhs.UL;
    UR = rhs.UR;
    BL = rhs.BL;
    BR = rhs.BR;
    sz_keys_max = rhs.sz_keys_max;
    idx_keys = rhs.idx_keys;
    if (rhs.ptr_keys != nullptr && sz_keys_max != 0) {
      ptr_keys = new KeyPointCG[sz_keys_max];
      if (idx_keys != 0)
        for (int i = 0; i < idx_keys; i++) ptr_keys[i] = rhs.ptr_keys[i];
    }
  }

  ~ExtractorNodeCG() { SAFE_DELETE(ptr_keys) }

  void push_keypts(const KeyPointCG &kp) {
    assert(ptr_keys == nullptr);
    ptr_keys[idx_keys] = kp;
    idx_keys++;
  }

  void DivideNode(ExtractorNodeCG &n1, ExtractorNodeCG &n2, ExtractorNodeCG &n3, ExtractorNodeCG &n4);

 public:
  bool bNoMore = false;

  Point2I UL, UR, BL, BR;

  Int idx_keys = 0;
  Int sz_keys_max = 0;
  KeyPointCG *ptr_keys = nullptr;

  List<ExtractorNodeCG>::iterator lit;
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