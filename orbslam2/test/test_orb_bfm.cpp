/**
 * @file test_orb_bfm.cpp
 * @author Gavin Gao (cggos@outlook.com)
 * @brief
 * @version 0.1
 * @date 2022-03-29
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <fstream>
#include <iostream>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>
#include <tuple>

#include "ORBextractor.h"

using namespace ORB_SLAM2;
using namespace std;

/**
 * @brief Hanming Distance for Descriptor Matching
 *
 * @ref Bit set count operation from: http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel
 * @param a
 * @param b
 * @return int
 */
int descriptor_distance(const cv::Mat &a, const cv::Mat &b) {
  const int *pa = a.ptr<int32_t>();
  const int *pb = b.ptr<int32_t>();

  int dist = 0;

  for (int i = 0; i < 8; i++, pa++, pb++) {
    unsigned int v = *pa ^ *pb;
    v = v - ((v >> 1) & 0x55555555);
    v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
    dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
  }

  return dist;
}

void compute_three_maxima(vector<int> *histo, const int L, int &ind1, int &ind2, int &ind3) {
  int max1 = 0;
  int max2 = 0;
  int max3 = 0;

  for (int i = 0; i < L; i++) {
    const int s = histo[i].size();
    if (s > max1) {
      max3 = max2;
      max2 = max1;
      max1 = s;
      ind3 = ind2;
      ind2 = ind1;
      ind1 = i;
    } else if (s > max2) {
      max3 = max2;
      max2 = s;
      ind3 = ind2;
      ind2 = i;
    } else if (s > max3) {
      max3 = s;
      ind3 = i;
    }
  }

  if (max2 < 0.1f * (float)max1) {
    ind2 = -1;
    ind3 = -1;
  } else if (max3 < 0.1f * (float)max1) {
    ind3 = -1;
  }
}

/**
 * @brief Brute Force Matching for ORB Descriptor
 * @details for @param mfNNratio and @param mbCheckOrientation
 *          1) LocalMapping: (0.6,false)
 *          2) TrackReferenceKeyFrame: (0.7, true)
 *          3) LoopClosing::ComputeSim3: (0.75,true)
 *
 * @param Descriptors1
 * @param Descriptors2
 * @param vKeysUn1
 * @param vKeysUn2
 * @param pair_scores
 * @param mfNNratio
 * @param mbCheckOrientation
 * @return int
 */
int orb_match_bfm(const cv::Mat &Descriptors1,
                  const cv::Mat &Descriptors2,
                  const vector<cv::KeyPoint> &vKeysUn1,
                  const vector<cv::KeyPoint> &vKeysUn2,
                  vector<std::tuple<int, int, float>> &pair_scores,
                  float mfNNratio = 0.6f,
                  bool mbCheckOrientation = false) {
  const int TH_HIGH = 100;
  const int TH_LOW = 50;
  const int HISTO_LENGTH = 30;

  const int N1 = Descriptors1.rows;
  const int N2 = Descriptors2.rows;

  // const vector<MapPoint *> vpMapPoints1 = pKF1->GetMapPointMatches();
  // const vector<MapPoint *> vpMapPoints2 = pKF2->GetMapPointMatches();

  // vpMatches12 = vector<MapPoint *>(vpMapPoints1.size(), static_cast<MapPoint *>(NULL));
  // vector<bool> vbMatched2(vpMapPoints2.size(), false);

  vector<int> vMatches12(N1, -1);
  vector<float> scores(N1, 0.f);

  vector<int> rotHist[HISTO_LENGTH];
  for (int i = 0; i < HISTO_LENGTH; i++) rotHist[i].reserve(500);

  const float factor = 1.0f / HISTO_LENGTH;

  int nmatches = 0;

  for (int idx1 = 0; idx1 < N1; idx1++) {
    // MapPoint *pMP1 = vpMapPoints1[idx1];
    // if (!pMP1) continue;
    // if (pMP1->isBad()) continue;

    const cv::Mat &d1 = Descriptors1.row(idx1);

    // int bestDist1 = 256;
    // int bestDist2 = 256;

    int bestDist = TH_LOW;
    int bestIdx2 = -1;

    for (int idx2 = 0; idx2 < N2; idx2++) {
      // MapPoint *pMP2 = vpMapPoints2[idx2];
      // if (vbMatched2[idx2] || !pMP2) continue;
      // if (pMP2->isBad()) continue;

      const cv::Mat &d2 = Descriptors2.row(idx2);

      int dist = descriptor_distance(d1, d2);

      if (dist > TH_LOW || dist > bestDist) continue;

      // if (dist < bestDist1) {
      //   bestDist2 = bestDist1;
      //   bestDist1 = dist;
      //   bestIdx2 = idx2;
      // } else if (dist < bestDist2) {
      //   bestDist2 = dist;
      // }

      if (dist < bestDist) {
        bestDist = dist;
        bestIdx2 = idx2;
      }
    }

    // if (bestDist1 < TH_LOW) {
    //   if (static_cast<float>(bestDist1) < mfNNratio * static_cast<float>(bestDist2)) {
    if (bestIdx2 >= 0) {
      // vpMatches12[idx1] = vpMapPoints2[bestIdx2];
      // vbMatched2[bestIdx2] = true;
      vMatches12[idx1] = bestIdx2;
      scores[idx1] = bestDist;

      if (mbCheckOrientation) {
        float rot = vKeysUn1[idx1].angle - vKeysUn2[bestIdx2].angle;
        if (rot < 0.0) rot += 360.0f;
        int bin = round(rot * factor);
        if (bin == HISTO_LENGTH) bin = 0;
        assert(bin >= 0 && bin < HISTO_LENGTH);
        rotHist[bin].push_back(idx1);
      }

      nmatches++;
    }
    //   }
    // }
  }

  if (mbCheckOrientation) {
    int ind1 = -1;
    int ind2 = -1;
    int ind3 = -1;

    compute_three_maxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3);

    for (int i = 0; i < HISTO_LENGTH; i++) {
      if (i != ind1 && i != ind2 && i != ind3) {
        for (size_t j = 0, jend = rotHist[i].size(); j < jend; j++) {
          // CurrentFrame.mvpMapPoints[rotHist[i][j]] = NULL;
          vMatches12[rotHist[i][j]] = -1;
          nmatches--;
        }
      }
    }
  }

  // vMatchedPairs.clear();
  // vMatchedPairs.reserve(nmatches);

  pair_scores.clear();
  pair_scores.reserve(nmatches);

  for (size_t i = 0, iend = vMatches12.size(); i < iend; i++) {
    if (vMatches12[i] < 0) continue;
    // vMatchedPairs.push_back(make_pair(i, vMatches12[i]));
    pair_scores.push_back(std::make_tuple(i, vMatches12[i], scores[i]));
  }

  return nmatches;
}

void good_matches(const vector<cv::DMatch> &matches_cv, vector<cv::DMatch> &matches_cv_good) {
  double min_dist = 10000, max_dist = 0;
  for (int i = 0; i < matches_cv.size(); i++) {
    double dist = matches_cv[i].distance;
    if (dist < min_dist) min_dist = dist;
    if (dist > max_dist) max_dist = dist;
  }
  // min_dist = min_element(matches_cv.begin(), matches_cv.end(), [](const cv::DMatch &m1, const cv::DMatch &m2) {
  //              return m1.distance < m2.distance;
  //            })->distance;
  // max_dist = max_element(matches_cv.begin(), matches_cv.end(), [](const cv::DMatch &m1, const cv::DMatch &m2) {
  //              return m1.distance < m2.distance;
  //            })->distance;
  for (int i = 0; i < matches_cv.size(); i++)
    if (matches_cv[i].distance <= max(2 * min_dist, 30.0)) matches_cv_good.push_back(matches_cv[i]);
  printf("-- Good Matches size : %d \n", matches_cv_good.size());
  printf("-- Max dist : %f \n", max_dist);
  printf("-- Min dist : %f \n", min_dist);
  std::cout << std::endl;
}

int main() {
  cv::Mat img0, img1, img0_gray, img1_gray;
  img0 = cv::imread("../data/tum/1305031102.343233.png");
  img1 = cv::imread("../data/tum/1305031104.943262.png");
  if (img0.empty() || img1.empty()) {
    std::cerr << "img empty!!!" << std::endl;
    return -1;
  }
  cv::cvtColor(img0, img0_gray, CV_BGR2GRAY);
  cv::cvtColor(img1, img1_gray, CV_BGR2GRAY);

  ORBextractor *mpORBextractor;
  {
    int nFeatures = 1000;
    float fScaleFactor = 1.2;
    int nLevels = 8;
    int fIniThFAST = 20;
    int fMinThFAST = 7;
    mpORBextractor = new ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);
  }

  // detect
  std::vector<cv::KeyPoint> keys0, keys1;
  cv::Mat descriptors0, descriptors1;
  (*mpORBextractor)(img0_gray, cv::Mat(), keys0, descriptors0);
  (*mpORBextractor)(img1_gray, cv::Mat(), keys1, descriptors1);

  // free
  if (mpORBextractor != nullptr) {
    delete mpORBextractor;
    mpORBextractor = nullptr;
  }

  // orb_match_bfm
  std::cout << "test orb_match_bfm" << std::endl;
  std::vector<cv::DMatch> matches_bfm, matches_bfm_good;
  vector<std::tuple<int, int, float>> pair_scores;
  {
    orb_match_bfm(descriptors0, descriptors1, keys0, keys1, pair_scores, 0.6, false);

    for (int i = 0; i < pair_scores.size(); i++) {
      cv::DMatch match;
      const auto &pair_score = pair_scores[i];
      match.queryIdx = std::get<0>(pair_score);
      match.trainIdx = std::get<1>(pair_score);
      match.distance = std::get<2>(pair_score);
      matches_bfm.push_back(match);
    }
    good_matches(matches_bfm, matches_bfm_good);
  }

  // test cv::DescriptorMatcher
  std::cout << "test cv::DescriptorMatcher" << std::endl;
  vector<cv::DMatch> matches_cv, matches_cv_good;
  {
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
    matcher->match(descriptors0, descriptors1, matches_cv);

    good_matches(matches_cv, matches_cv_good);
  }

  // draw
  {
    cv::Mat img_match;
    cv::drawMatches(img0, keys0, img1, keys1, matches_bfm_good, img_match, cv::Scalar(0, 255, 0), cv::Scalar(0, 0, 255));
    cv::imshow("所有匹配点对", img_match);
  }
  cv::waitKey(0);

  return 0;
}