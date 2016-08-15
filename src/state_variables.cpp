#include "state_variables.h"
#include "axis_angle.h"
#include <cfloat>
#include <cmath>

using std::cout;
using std::endl;

std::ostream& operator<<(std::ostream& out, const sixDoFState& state)
{
    out << "[ "
        << state.vec_(0) << ", "
        << state.vec_(1) << ", "
        << state.vec_(2) << " ]^T, ["
        << state.vec_(3) << ", "
        << state.vec_(4) << ", "
        << state.vec_(5) << " ]^T.";
    return out;
}

//------------------------------------------------------------
// 从矩阵表示转换到向量表示
void sixDoFState::fromMatrix4f(Eigen::Matrix4f &mat)
{
    vec_(0) = mat(0,3);
    vec_(1) = mat(1,3);
    vec_(2) = mat(2,3);

    Eigen::Matrix3f rotmat = mat.block<3,3>(0,0);
    Eigen::Vector3f rotat_vec;
    rotationMatrixToEulerAngle_XYZ(rotmat, rotat_vec);

    vec_(3) = rotat_vec(0);
    vec_(4) = rotat_vec(1);
    vec_(5) = rotat_vec(2);
}

//------------------------------------------------------------
// 从向量表示转换到矩阵表示
void sixDoFState::toMatrix4f(Eigen::Matrix4f &mat)
{
    Eigen::Vector3f rotat_vec = vec_.block<3,1>(3,0);
    Eigen::Matrix3f r;
    eulerAnglesToRotationMatrix_XYZ(rotat_vec, r);

    // 将平移向量和旋转矩阵转换成变换矩阵
    mat = Eigen::Matrix4f::Identity();
    mat(0,0) = r(0,0); mat(0,1) = r(0,1); mat(0,2) = r(0,2);
    mat(1,0) = r(1,0); mat(1,1) = r(1,1); mat(1,2) = r(1,2);
    mat(2,0) = r(2,0); mat(2,1) = r(2,1); mat(2,2) = r(2,2);
    // 平移向量
    mat(0,3) = vec_(0);
    mat(1,3) = vec_(1);
    mat(2,3) = vec_(2);
}


//------------------------------------------------------------
//
//------------------------------------------------------------
