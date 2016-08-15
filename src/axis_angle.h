#ifndef AXIS_ANGLE_H_
#define AXIS_ANGLE_H_

#include <Eigen/Dense>
#include <Eigen/Geometry>

//------------------------------------------------------------
// use Eigen library
// 定义模板函数，float 和 double
//------------------------------------------------------------


//------------------------------------------------------------
template<typename scalarT>
void axisAngleToQuaternion(const Eigen::Matrix<scalarT,3,1> &axis,
                                 scalarT angle,
                                 Eigen::Quaternion<scalarT> &quaternion);

//------------------------------------------------------------
template<typename scalarT>
void quaternionToAxisAngle(const Eigen::Quaternion<scalarT> &quaternion,
                                 Eigen::Matrix<scalarT,3,1> &axis,
                                 scalarT &angle);

//------------------------------------------------------------
template<typename scalarT>
void axisAngleToRotationMatrix(const Eigen::Matrix<scalarT,3,1> &axis,
                                     scalarT angle,
                                     Eigen::Matrix<scalarT,3,3> &rotmat);

//------------------------------------------------------------
template<typename scalarT>
void rotationMatrixToAxisAngle(const Eigen::Matrix<scalarT,3,3> &rotmat,
                                     Eigen::Matrix<scalarT,3,1> &axis,
                                     scalarT &angle);

//------------------------------------------------------------
template<typename scalarT>
void rotationMatrixToQuaternion(const Eigen::Matrix<scalarT,3,3> &rotmat,
                                      Eigen::Quaternion<scalarT> &quaternion);

//------------------------------------------------------------
template<typename scalarT>
void quaternionToRotationMatrix(const Eigen::Quaternion<scalarT> &quaternion,
                                      Eigen::Matrix<scalarT,3,3> &rotmat);

//------------------------------------------------------------
template<typename scalarT>
void eulerAngleToQuaternion_XYZ(const Eigen::Matrix<scalarT,3,1> &eulerAngles,
                                      Eigen::Quaternion<scalarT> &quaternion);

//------------------------------------------------------------
template<typename scalarT>
void quaternionToEulerAngle_XYZ(const Eigen::Quaternion<scalarT> &quaternion,
                                      Eigen::Matrix<scalarT,3,1> &eulerAngles);

//------------------------------------------------------------
template<typename scalarT>
void rotationMatrixToEulerAngle_XYZ(const Eigen::Matrix<scalarT,3,3> &rotmat,
                                          Eigen::Matrix<scalarT,3,1> &eulerAngles);

//------------------------------------------------------------
template<typename scalarT>
void eulerAnglesToRotationMatrix_XYZ(const Eigen::Matrix<scalarT,3,1> &eulerAngles,
                                           Eigen::Matrix<scalarT,3,3> &rotmat);

//------------------------------------------------------------
// double array representation
//------------------------------------------------------------


//------------------------------------------------------------
void axisAngleToRotationMatrix(const double *axis,
                               double angle,
                               double *rotmat,
                               bool normalized_p = true);

//------------------------------------------------------------
void rotationMatrixToAxisAngle(const double *mat,
                               double *axis,
                               double &angle);

//------------------------------------------------------------
void rotationMatrixToAxisAngle_simple(const double *rotmat,
                                      double *axis,
                                      double &angle);

//------------------------------------------------------------
void axisAngleToQuaternion(const double *axis,
                           double angle,
                           double *quaternion);

//------------------------------------------------------------
void quaternionToAxisAngle(const double *quaternion,
                           double *axis,
                           double &angle);

//------------------------------------------------------------
void rotationMatrixToQuaternion(const double *rotmat, double *quaterntion);

//------------------------------------------------------------
void quaternionToRotationMatrix(const double *quaternion, double *rotmat);

//------------------------------------------------------------
void eulerAngleToQuaternion_XYZ(const double *eulerAngles, double *quaternion);

//------------------------------------------------------------
void quaternionToEulerAngle_XYZ(const double *quaternion, double *eulerAngles);

//------------------------------------------------------------
void rotationMatrixToEulerAngle_XYZ(const double *rotmat, double *eulerAngles);

//------------------------------------------------------------
void eulerAnglesToRotationMatrix_XYZ(const double *eulerAngles, double *rotmat);


#endif // AXIS_ANGLE_H_
