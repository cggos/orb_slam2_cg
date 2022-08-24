#ifndef G2OTYPES_H
#define G2OTYPES_H

#include <math.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>

#include "CameraModels/KannalaBrandt8.h"
#include "g2o/core/base_unary_edge.h"
// #include "g2o/core/base_binary_edge.h"
// #include "g2o/core/base_multi_edge.h"
// #include "g2o/core/base_vertex.h"
// #include "g2o/types/types_sba.h"

namespace ORB_SLAM2 {

class EdgeFisheyeStereoOnlyPose : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, g2o::VertexSE3Expmap> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeFisheyeStereoOnlyPose(const Eigen::Vector3f &Xw_) : Xw(Xw_.cast<double>()) {}

  bool read(std::istream &is) { return false; }
  bool write(std::ostream &os) const { return false; }

  void computeError() {
    const g2o::VertexSE3Expmap *v1 = static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
    Eigen::Vector3d pt_c1 = Tc1c0_ * v1->estimate().map(Xw);

    Eigen::Vector2d obs(_measurement);
    _error = obs - mpCameraFE->project(pt_c1);
  }

  virtual void linearizeOplus();

  bool isDepthPositive() {
    const g2o::VertexSE3Expmap *v1 = static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
    return (v1->estimate().map(Xw))(2) > 0.0;
  }

  Eigen::Matrix<double, 6, 6> GetHessian() {
    linearizeOplus();
    return _jacobianOplusXi.transpose() * information() * _jacobianOplusXi;
  }

 public:
  const Eigen::Vector3d Xw;

  // TF from RGB Cam to Fisheye Cam
  g2o::SE3Quat Tc1c0_;
  GeometricCamera *mpCameraFE;
};

class EdgeFisheyeStereo : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexSBAPointXYZ, g2o::VertexSE3Expmap> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeFisheyeStereo() {}

  bool read(std::istream &is) {}

  bool write(std::ostream &os) const {}

  void computeError() {
    const g2o::VertexSE3Expmap *v1 = static_cast<const g2o::VertexSE3Expmap *>(_vertices[1]);
    const g2o::VertexSBAPointXYZ *v2 = static_cast<const g2o::VertexSBAPointXYZ *>(_vertices[0]);
    Eigen::Vector2d obs(_measurement);
    Eigen::Vector3d pt_c1 = Tc1c0_ * v1->estimate().map(v2->estimate());
    _error = obs - mpCameraFE->project(pt_c1);
  }

  bool isDepthPositive() {
    const g2o::VertexSE3Expmap *v1 = static_cast<const g2o::VertexSE3Expmap *>(_vertices[1]);
    const g2o::VertexSBAPointXYZ *v2 = static_cast<const g2o::VertexSBAPointXYZ *>(_vertices[0]);
    return (v1->estimate().map(v2->estimate()))(2) > 0.0;
  }

  virtual void linearizeOplus();

  // TF from RGB Cam to Fisheye Cam
  g2o::SE3Quat Tc1c0_;
  GeometricCamera *mpCameraFE;
};

}  // namespace ORB_SLAM2

#endif
