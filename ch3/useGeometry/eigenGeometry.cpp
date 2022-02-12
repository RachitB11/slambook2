#include <cmath>
#include <iostream>

using namespace std;

#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace Eigen;

// This demonstrates the eigen geometry modules

int main(int argc, char **argv) {

  // The Eigen/Geometry module provides a variety of rotation and translation
  // representations
  // 3D rotation matrix directly using Matrix3d or Matrix3f
  Matrix3d rotation_matrix = Matrix3d::Identity();
  // The rotation vector uses AngleAxis, the underlying layer is not directly Matrix,
  // but the operation can be treated as a matrix (because the operator is
  // overloaded)
  AngleAxisd rotation_vector(M_PI / 4, Vector3d(0, 0, 1));  // Rotate about Z at 45 degrees
  cout.precision(3);
  cout << "rotation matrix =\n"
       << rotation_vector.matrix() << endl;  // Convert to matrix with matrix()
  rotation_matrix = rotation_vector.toRotationMatrix();
  // coordinate transformation with AngleAxis
  Vector3d v(1, 0, 0);
  Vector3d v_rotated = rotation_vector * v;
  cout << "(1,0,0) after rotation (by angle axis) = " << v_rotated.transpose() << endl;
  // Same with the matrix
  v_rotated = rotation_matrix * v;
  cout << "(1,0,0) after rotation (by matrix) = " << v_rotated.transpose() << endl;

  // Convert to euler angles. Here these are Z->Y->X changing axis rotations
  // or X->Y->Z fixed axis rotations
  Vector3d euler_angles = rotation_matrix.eulerAngles(2, 1, 0);  // ZYX，yaw-pitch-roll
  cout << "yaw pitch roll = " << euler_angles.transpose() << endl;

  // Euclidean transformation Eigen::Isometry
  Isometry3d T = Isometry3d::Identity();  // Although called 3d, it is essentially a 4∗4 matrix
  T.rotate(rotation_vector);              // Rotate using the rotation vector
  T.pretranslate(Vector3d(1, 3, 4));      // Then translate
  cout << "Transform matrix = \n" << T.matrix() << endl;

  // Use the transformation matrix for coordinate transformation
  Vector3d v_transformed = T * v;  // This is equivalent to R*v+t
  cout << "v tranformed = " << v_transformed.transpose() << endl;

  // For affine and projective transformations, use Eigen::Affine3d and Eigen::Projective3d.

  // Quaternion
  // You can assign AngleAxis directly to quaternions, and vice versa
  Quaterniond q = Quaterniond(rotation_vector);
  cout << "quaternion from rotation vector = " << q.coeffs().transpose()
       << endl;  // Note that the order of coeffs is (x, y, z, w), w is the real part, the
                 // first three are the imaginary part
  q = Quaterniond(rotation_matrix);
  cout << "quaternion from rotation matrix = " << q.coeffs().transpose() << endl;
  // Rotate a vector with a quaternion and use overloaded multiplication
  v_rotated = q * v;  // Very important to note that under the hood its equivalent to qvq^{-1}
  cout << "(1,0,0) after rotation = " << v_rotated.transpose() << endl;
  // But remember that if you're representing the vector as an imaginary quat you need to
  // do it the conventional way. The operator is NOT automatically overloaded here.
  // Its just for quat * vector
  cout << "should be equal to "
       << (q * Quaterniond(0, 1, 0, 0) * q.inverse()).coeffs().transpose() << endl;

  return 0;
}
