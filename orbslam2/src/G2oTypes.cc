#include "G2oTypes.h"

namespace ORB_SLAM2 {

void EdgeFisheyeStereoOnlyPose::linearizeOplus() {
  const g2o::VertexSE3Expmap* VPose = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);

  const Eigen::Vector3d Xc = VPose->estimate().map(Xw);

  Eigen::Matrix<double, 2, 3> proj_jac = mpCameraFE->projectJac(Xc);

  Eigen::Matrix<double, 3, 3> Rc1c0 = Tc1c0_.rotation().toRotationMatrix();

  Eigen::Matrix<double, 3, 6> SE3deriv;
  double x = Xc(0);
  double y = Xc(1);
  double z = Xc(2);
  // clang-format off
  SE3deriv << 0.0, z, -y, 1.0, 0.0, 0.0, 
              -z, 0.0, x, 0.0, 1.0, 0.0, 
              y, -x, 0.0, 0.0, 0.0, 1.0;
  // clang-format on

  _jacobianOplusXi = proj_jac * Rc1c0 * SE3deriv;  // symbol different becasue of update mode
}

void EdgeFisheyeStereo::linearizeOplus() {
  const g2o::VertexSE3Expmap* VPose = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
  const g2o::VertexSBAPointXYZ* VPoint = static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);

  const Eigen::Vector3d Xc = VPose->estimate().map(VPoint->estimate());

  Eigen::Matrix<double, 2, 3> proj_jac = mpCameraFE->projectJac(Xc);

  Eigen::Matrix<double, 3, 3> Rc1c0 = Tc1c0_.rotation().toRotationMatrix();

  Eigen::Matrix<double, 3, 6> SE3deriv;
  double x = Xc(0);
  double y = Xc(1);
  double z = Xc(2);
  // clang-format off
  SE3deriv << 0.0, z, -y, 1.0, 0.0, 0.0, 
              -z, 0.0, x, 0.0, 1.0, 0.0, 
              y, -x, 0.0, 0.0, 0.0, 1.0;
  // clang-format on

  _jacobianOplusXi = proj_jac * Rc1c0 * VPose->estimate().rotation().toRotationMatrix();
  _jacobianOplusXj = proj_jac * Rc1c0 * SE3deriv;  // symbol different becasue of update mode
}

};  // namespace ORB_SLAM2
