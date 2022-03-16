/**
 * @file orb_extractor_c.cc
 * @author cggos (cggos@outlook.com)
 * @brief
 * @version 0.1
 * @date 2022-03-15
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "orb_extractor_c.h"

#include <iostream>

using namespace std;

namespace cg {

void ExtractorNodeCG::DivideNode(ExtractorNodeCG &n1, ExtractorNodeCG &n2, ExtractorNodeCG &n3, ExtractorNodeCG &n4) {
  if (sz_keys == 0 || ptr_keys == nullptr) return;

  const int halfX = ceil(static_cast<float>(UR.x - UL.x) / 2);
  const int halfY = ceil(static_cast<float>(BR.y - UL.y) / 2);

  // Define boundaries of childs
  n1.UL = UL;
  n1.UR = Point2I(UL.x + halfX, UL.y);
  n1.BL = Point2I(UL.x, UL.y + halfY);
  n1.BR = Point2I(UL.x + halfX, UL.y + halfY);

  n2.UL = n1.UR;
  n2.UR = UR;
  n2.BL = n1.BR;
  n2.BR = Point2I(UR.x, UL.y + halfY);

  n3.UL = n1.BL;
  n3.UR = n1.BR;
  n3.BL = BL;
  n3.BR = Point2I(n1.BR.x, BL.y);

  n4.UL = n3.UR;
  n4.UR = n2.BR;
  n4.BL = n3.BR;
  n4.BR = BR;

  n1.ptr_keys = new KeyPointCG[sz_keys];
  n2.ptr_keys = new KeyPointCG[sz_keys];
  n3.ptr_keys = new KeyPointCG[sz_keys];
  n4.ptr_keys = new KeyPointCG[sz_keys];

  // Associate points to childs
  for (size_t i = 0; i < sz_keys; i++) {
    const KeyPointCG &kp = ptr_keys[i];
    if (kp.x < n1.UR.x) {
      if (kp.y < n1.BR.y)
        n1.push_keypts(kp);
      else
        n3.push_keypts(kp);
    } else if (kp.y < n1.BR.y)
      n2.push_keypts(kp);
    else
      n4.push_keypts(kp);
  }

  if (n1.sz_keys == 1) n1.bNoMore = true;
  if (n2.sz_keys == 1) n2.bNoMore = true;
  if (n3.sz_keys == 1) n3.bNoMore = true;
  if (n4.sz_keys == 1) n4.bNoMore = true;
}

KeyPointCG kp_cv2cg(const cv::KeyPoint &kp) {
  KeyPointCG kpcg;
  kpcg.x = kp.pt.x;
  kpcg.y = kp.pt.y;
  kpcg.angle = kp.angle;
  kpcg.octave = kp.octave;
  kpcg.response = kp.response;
  kpcg.size = kp.size;
  return kpcg;
}

cv::KeyPoint kp_cg2cv(const KeyPointCG &kpcg) {
  cv::KeyPoint kp;
  kp.pt.x = kpcg.x;
  kp.pt.y = kpcg.y;
  kp.angle = kpcg.angle;
  kp.octave = kpcg.octave;
  kp.response = kpcg.response;
  kp.size = kpcg.size;
  return kp;
}

vector<cv::KeyPoint> distribute_quadtree_c(cg::KeyPointCG *arr_to_dis_keys[],
                                           const int &sz_to_dis_keys,
                                           const int &minX,
                                           const int &maxX,
                                           const int &minY,
                                           const int &maxY,
                                           const int &N,
                                           const int &level,
                                           const int &nfeatures) {
  std::cout << "level: " << level << ", " << __FUNCTION__ << std::endl;

  assert(arr_to_dis_keys == nullptr || sz_to_dis_keys == 0);

  // Compute how many initial nodes
  const int nIni = round(static_cast<float>(maxX - minX) / (maxY - minY));

  const float hX = static_cast<float>(maxX - minX) / nIni;

  list<ExtractorNodeCG> lNodes;

  ExtractorNodeCG *vpIniNodes[nIni];

  for (int i = 0; i < nIni; i++) {
    ExtractorNodeCG ni;
    ni.UL = Point2I(hX * static_cast<float>(i), 0);
    ni.UR = Point2I(hX * static_cast<float>(i + 1), 0);
    ni.BL = Point2I(ni.UL.x, maxY - minY);
    ni.BR = Point2I(ni.UR.x, maxY - minY);
    ni.ptr_keys = new KeyPointCG[sz_to_dis_keys];

    lNodes.push_back(ni);
    vpIniNodes[i] = &lNodes.back();
  }

  // Associate points to childs
  for (size_t i = 0; i < sz_to_dis_keys; i++) {
    const KeyPointCG &kp = *(arr_to_dis_keys[i]);
    vpIniNodes[(int)std::floor(kp.x / hX)]->push_keypts(kp);
  }

  list<ExtractorNodeCG>::iterator lit = lNodes.begin();

  while (lit != lNodes.end()) {
    if (lit->sz_keys == 1) {
      lit->bNoMore = true;
      lit++;
    } else if (lit->sz_keys == 0)
      lit = lNodes.erase(lit);
    else
      lit++;
  }

  bool bFinish = false;

  int iteration = 0;

  vector<pair<int, ExtractorNodeCG *> > vSizeAndPointerToNode;
  vSizeAndPointerToNode.reserve(lNodes.size() * 4);

  while (!bFinish) {
    iteration++;

    int prevSize = lNodes.size();

    lit = lNodes.begin();

    int nToExpand = 0;

    vSizeAndPointerToNode.clear();

    while (lit != lNodes.end()) {
      if (lit->bNoMore) {
        // If node only contains one point do not subdivide and continue
        lit++;
        continue;
      } else {
        // If more than one point, subdivide
        ExtractorNodeCG n1, n2, n3, n4;
        lit->DivideNode(n1, n2, n3, n4);

        // Add childs if they contain points
        if (n1.sz_keys > 0) {
          lNodes.push_front(n1);
          if (n1.sz_keys > 1) {
            nToExpand++;
            vSizeAndPointerToNode.push_back(make_pair(n1.sz_keys, &lNodes.front()));
            lNodes.front().lit = lNodes.begin();
          }
        }
        if (n2.sz_keys > 0) {
          lNodes.push_front(n2);
          if (n2.sz_keys > 1) {
            nToExpand++;
            vSizeAndPointerToNode.push_back(make_pair(n2.sz_keys, &lNodes.front()));
            lNodes.front().lit = lNodes.begin();
          }
        }
        if (n3.sz_keys > 0) {
          lNodes.push_front(n3);
          if (n3.sz_keys > 1) {
            nToExpand++;
            vSizeAndPointerToNode.push_back(make_pair(n3.sz_keys, &lNodes.front()));
            lNodes.front().lit = lNodes.begin();
          }
        }
        if (n4.sz_keys > 0) {
          lNodes.push_front(n4);
          if (n4.sz_keys > 1) {
            nToExpand++;
            vSizeAndPointerToNode.push_back(make_pair(n4.sz_keys, &lNodes.front()));
            lNodes.front().lit = lNodes.begin();
          }
        }

        lit = lNodes.erase(lit);
        continue;
      }
    }

    // Finish if there are more nodes than required features
    // or all nodes contain just one point
    if ((int)lNodes.size() >= N || (int)lNodes.size() == prevSize) {
      bFinish = true;
    } else if (((int)lNodes.size() + nToExpand * 3) > N) {
      while (!bFinish) {
        prevSize = lNodes.size();

        vector<pair<int, ExtractorNodeCG *> > vPrevSizeAndPointerToNode = vSizeAndPointerToNode;
        vSizeAndPointerToNode.clear();

        sort(vPrevSizeAndPointerToNode.begin(), vPrevSizeAndPointerToNode.end());
        for (int j = vPrevSizeAndPointerToNode.size() - 1; j >= 0; j--) {
          ExtractorNodeCG n1, n2, n3, n4;
          vPrevSizeAndPointerToNode[j].second->DivideNode(n1, n2, n3, n4);

          // Add childs if they contain points
          if (n1.sz_keys > 0) {
            lNodes.push_front(n1);
            if (n1.sz_keys > 1) {
              vSizeAndPointerToNode.push_back(make_pair(n1.sz_keys, &lNodes.front()));
              lNodes.front().lit = lNodes.begin();
            }
          }
          if (n2.sz_keys > 0) {
            lNodes.push_front(n2);
            if (n2.sz_keys > 1) {
              vSizeAndPointerToNode.push_back(make_pair(n2.sz_keys, &lNodes.front()));
              lNodes.front().lit = lNodes.begin();
            }
          }
          if (n3.sz_keys > 0) {
            lNodes.push_front(n3);
            if (n3.sz_keys > 1) {
              vSizeAndPointerToNode.push_back(make_pair(n3.sz_keys, &lNodes.front()));
              lNodes.front().lit = lNodes.begin();
            }
          }
          if (n4.sz_keys > 0) {
            lNodes.push_front(n4);
            if (n4.sz_keys > 1) {
              vSizeAndPointerToNode.push_back(make_pair(n4.sz_keys, &lNodes.front()));
              lNodes.front().lit = lNodes.begin();
            }
          }

          lNodes.erase(vPrevSizeAndPointerToNode[j].second->lit);

          if ((int)lNodes.size() >= N) break;
        }

        if ((int)lNodes.size() >= N || (int)lNodes.size() == prevSize) bFinish = true;
      }
    }
  }

  // Retain the best point in each node
  vector<cv::KeyPoint> vResultKeys;
  vResultKeys.reserve(nfeatures);
  for (list<ExtractorNodeCG>::iterator lit = lNodes.begin(); lit != lNodes.end(); lit++) {
    KeyPointCG kp = lit->ptr_keys[0];
    float maxResponse = kp.response;
    for (size_t k = 1; k < lit->sz_keys; k++) {
      if (lit->ptr_keys[k].response > maxResponse) {
        kp = lit->ptr_keys[k];
        maxResponse = kp.response;
      }
    }

    vResultKeys.push_back(kp_cg2cv(kp));
  }

  return vResultKeys;
}

}  // namespace cg