#include "sophus/se3.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <iostream>

using namespace std;
using namespace Eigen;

/// Basic Sophus implementation

int main(int argc, char **argv) {

  // Rotation of 90 degrees about thez axis
  Matrix3d R = AngleAxisd(M_PI / 2, Vector3d(0, 0, 1)).toRotationMatrix();
  // Quaternion object
  Quaterniond q(R);
  Sophus::SO3d SO3_R(R);  // Sophus::SO3d Lie algebra object from rotation matrix
  Sophus::SO3d SO3_q(q);  // Same lie algebra object from quaternion
  // Print the SO3 matrix from both the lie algebra object
  cout << "SO(3) from matrix:\n" << SO3_R.matrix() << endl;
  cout << "SO(3) from quaternion:\n" << SO3_q.matrix() << endl;
  cout << "they are equal" << endl;

  // Log is the 3D vector Lie Algebra representation of the rotation matrix
  Vector3d so3 = SO3_R.log();
  cout << "so3 = " << so3.transpose() << endl;
  // hat representation which is the skew symmetric matrix
  cout << "so3 hat=\n" << Sophus::SO3d::hat(so3) << endl;
  // vee gets the vector from the skew symmetric matrix
  cout << "so3 hat vee= " << Sophus::SO3d::vee(Sophus::SO3d::hat(so3)).transpose() << endl;

  // Apply a left perturbation
  Vector3d update_so3(1e-4, 0, 0);  // A small perturbation
  // Get the lie group (Rotation matrix) from the rotation vector and apply the rotation
  Sophus::SO3d SO3_updated = Sophus::SO3d::exp(update_so3) * SO3_R;
  cout << "SO3 updated = \n" << SO3_updated.matrix() << endl;

  cout << "*******************************" << endl;
  // SE3 sophus
  Vector3d t(1, 0, 0);        // Translation by 1m
  Sophus::SE3d SE3_Rt(R, t);  // Lie Algebra from R,t SE(3)
  Sophus::SE3d SE3_qt(q, t);  // Lie Algebra from q,t SE(3)
  cout << "SE3 from R,t= \n" << SE3_Rt.matrix() << endl;
  cout << "SE3 from q,t= \n" << SE3_qt.matrix() << endl;
  // Lie Algebra for SE3 is a 6 Dof Vector
  typedef Eigen::Matrix<double, 6, 1> Vector6d;
  Vector6d se3 = SE3_Rt.log();
  cout << "se3 = " << se3.transpose() << endl;
  // Get the hat and recover the vector format of Lie Algebra from the hat
  cout << "se3 hat = \n" << Sophus::SE3d::hat(se3) << endl;
  cout << "se3 hat vee = " << Sophus::SE3d::vee(Sophus::SE3d::hat(se3)).transpose() << endl;

  // Left perturb SE3
  Vector6d update_se3;  // Small perturbation in roh
  update_se3.setZero();
  update_se3(0, 0) = 1e-4;
  Sophus::SE3d SE3_updated = Sophus::SE3d::exp(update_se3) * SE3_Rt;
  cout << "SE3 updated = " << endl << SE3_updated.matrix() << endl;

  return 0;
}
