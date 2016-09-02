#include "state_variables.h"
#include "axis_angle.h"
#include <cfloat>
#include <cmath>

using std::cout;
using std::endl;

const SVector6f::vector3f& SVector6f::r() {
  float *p = vec_.data();
  return Eigen::Map<Eigen::Matrix<float, 3, 1> >(p, 3);
}

const SVector6f::vector3f& SVector6f::t() {
  float *p = vec_.data();
  return Eigen::Map<Eigen::Matrix<float, 3, 1> >(p+3, 3);
}

std::ostream& operator<<(std::ostream& out, const SVector6f& state)
{
  out << "r[ "
  << state.vec_(0) << ", "
  << state.vec_(1) << ", "
  << state.vec_(2) << " ]^T, t["
  << state.vec_(3) << ", "
  << state.vec_(4) << ", "
  << state.vec_(5) << " ]^T.";
  return out;
}

//------------------------------------------------------------
// 从矩阵表示转换到向量表示
void SVector6f::fromMatrix4f(Eigen::Matrix4f &mat)
{
  vec_(3) = mat(0,3);
  vec_(4) = mat(1,3);
  vec_(5) = mat(2,3);
  
  Eigen::Matrix3f rotmat = mat.block<3,3>(0,0);
  Eigen::Vector3f rotat_vec;
  rotationMatrixToEulerAngle_XYZ(rotmat, rotat_vec);
  
  vec_(0) = rotat_vec(0);
  vec_(1) = rotat_vec(1);
  vec_(2) = rotat_vec(2);
}

//------------------------------------------------------------
// 从向量表示转换到矩阵表示
void SVector6f::toMatrix4f(Eigen::Matrix4f &mat)
{
  Eigen::Vector3f rotat_vec = this->r();
  Eigen::Matrix3f r;
  eulerAnglesToRotationMatrix_XYZ(rotat_vec, r);
  
  // 将平移向量和旋转矩阵转换成变换矩阵
  mat = Eigen::Matrix4f::Identity();
  mat(0,0) = r(0,0); mat(0,1) = r(0,1); mat(0,2) = r(0,2);
  mat(1,0) = r(1,0); mat(1,1) = r(1,1); mat(1,2) = r(1,2);
  mat(2,0) = r(2,0); mat(2,1) = r(2,1); mat(2,2) = r(2,2);
  // 平移向量
  mat(0,3) = vec_(3);
  mat(1,3) = vec_(4);
  mat(2,3) = vec_(5);
}


//------------------------------------------------------------
//
//------------------------------------------------------------


SVector12f::SVector12f(SVector6f trans) {
  vec_ = Vector12f::Zero();
  // rotation
  vec_(0) = trans.vec_(0);
  vec_(1) = trans.vec_(1);
  vec_(2) = trans.vec_(2);
  // translation
  vec_(6) = trans.vec_(3);
  vec_(7) = trans.vec_(4);
  vec_(8) = trans.vec_(5);
}

void SVector12f::fromMatrix4f(Eigen::Matrix4f& mat)
{
  SVector6f trans = SVector6f(mat);
  
  // rotation
  vec_(0) = trans.vec_(0);
  vec_(1) = trans.vec_(1);
  vec_(2) = trans.vec_(2);
  // translation
  vec_(6) = trans.vec_(3);
  vec_(7) = trans.vec_(4);
  vec_(8) = trans.vec_(5);
}

// 获取状态向量，以六自由度状态类（sixDoFState）形式保存
SVector6f SVector12f::getState()
{
  SVector6f res;
  
  // Eigen::Matrix::block,
  // Block of size (p,q), starting at (i,j)
  // matrix.block(i,j,p,q);
  // matrix.block<p,q>(i,j);
  res.vec_.block<3,1>(0,0) = vec_.block<3,1>(0,0);
  res.vec_.block<3,1>(3,0) = vec_.block<3,1>(6,0);  
  
  cout << "getState: " << res << endl;
  return res;
}

SVector6f SVector12f::getRatio()
{
  SVector6f res;
  res.vec_.block<3,1>(0,0) = vec_.block<3,1>(3,0);
  res.vec_.block<3,1>(3,0) = vec_.block<3,1>(9,0);
  
  cout << "getRatio: " << res << endl;
  return res;
}

std::ostream& operator<<(std::ostream& out, const SVector12f& state) {
  out << state.vec_.block<6,1>(0,0) << "\n" << state.vec_.block<6,1>(6,0);
  return out;
}