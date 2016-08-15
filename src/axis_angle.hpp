#include "axis_angle.h"

#include <cmath>
#include <cfloat>
#include <Eigen/Dense>
#include <Eigen/Geometry>

//------------------------------------------------------------
// two assistant functions
// return the sign a float or a double
//------------------------------------------------------------
/// get the sign of a double
double getSign(double x)
{
    if(fabs(x)>DBL_EPSILON)
    {
        if(x>0.0)
            return 1.0;
        else
            return -1.0;
    }
    else
        return 1.0;
}

/// get the sign of a double
float getSign(float x)
{
    if(fabs(x)>FLT_EPSILON)
    {
        if(x>0.0f)
            return 1.0f;
        else
            return -1.0f;
    }
    else
        return 1.0f;
}



//------------------------------------------------------------
// use Eigen library
//------------------------------------------------------------


//------------------------------------------------------------
template<typename scalarT>
void axisAngleToQuaternion(const Eigen::Matrix<scalarT,3,1> &axis,
                           scalarT angle,
                           Eigen::Quaternion<scalarT> &quaternion)
{
    scalarT c = cos(angle/2);
    scalarT s = sin(angle/2);
    quaternion.w() = c;
    quaternion.x() = axis(0)*s;
    quaternion.y() = axis(1)*s;
    quaternion.z() = axis(2)*s;
}

//------------------------------------------------------------
template<typename scalarT>
void quaternionToAxisAngle(const Eigen::Quaternion<scalarT> &quaternion,
                           Eigen::Matrix<scalarT,3,1> &axis,
                           scalarT &angle)
{
    // assume angle is in [0, pi]
    scalarT q0 = quaternion.w();
    scalarT half_angle = 0.0;

    if(q0<0)
    {
        // means that angle is beyond [0, pi], >pi
        half_angle = acosf(q0);
        scalarT s = sin(half_angle);

        angle = half_angle*2.0-M_PI;
        axis(0) = -quaternion.x()/s;
        axis(1) = -quaternion.y()/s;
        axis(2) = -quaternion.z()/s;
    }
    else if(fabs(q0)<FLT_EPSILON)
    {
        // q0 = 0, angle = pi
        angle = M_PI;
        axis(0) = quaternion.x();
        axis(1) = quaternion.y();
        axis(2) = quaternion.z();
    }
    else
    {
        // q0 > 0
        half_angle = acos(q0);
        scalarT s = sinf(half_angle);

        angle = half_angle*2.0f;
        axis(0) = quaternion.x()/s;
        axis(1) = quaternion.y()/s;
        axis(2) = quaternion.z()/s;
    }
}

//------------------------------------------------------------
template<typename scalarT>
void axisAngleToRotationMatrix(const Eigen::Matrix<scalarT,3,1> &axis,
                               scalarT angle,
                               Eigen::Matrix<scalarT,3,3> &rotmat)
{
    scalarT ux = axis(0);
    scalarT uy = axis(1);
    scalarT uz = axis(2);

    // use c and s to represent cos angle and sin angle
    scalarT c = cos(angle);
    scalarT s = sin(angle);

    // compute the rotation matrix element
    rotmat(0,0) = ux*ux*(1-c)+c;    rotmat(0,1) = ux*uy*(1-c)-uz*s; rotmat(0,2) = ux*uz*(1-c)+uy*s;
    rotmat(1,0) = ux*uy*(1-c)+uz*s; rotmat(1,1) = uy*uy*(1-c)+c;    rotmat(1,2) = uy*uz*(1-c)-ux*s;
    rotmat(2,0) = ux*uz*(1-c)-uy*s; rotmat(2,1) = uy*uz*(1-c)+ux*s; rotmat(2,2) = uz*uz*(1-c)+c;
}

//------------------------------------------------------------
template<typename scalarT>
void rotationMatrixToAxisAngle(const Eigen::Matrix<scalarT,3,3> &rotmat,
                               Eigen::Matrix<scalarT,3,1> &axis,
                               scalarT &angle)
{
    // compute the matrix trace to get rotation angle.
    // here rotation angle is in the interval [0, pi],
    // so attention should paid to the rotation angle value outside this
    // interval.
    scalarT trace = rotmat(0,0)+rotmat(1,1)+rotmat(2,2);
    scalarT cos_theta = 0.5f*(trace-1);
    if(fabs((double)cos_theta+1)<2*DBL_EPSILON)
        angle = M_PI;
    else
        angle = acos(cos_theta);

    // compute [R-I], to get the eigen vector corresponding to the eigen velue 1
    Eigen::Matrix<scalarT,3,3> rot_i = rotmat - Eigen::Matrix<scalarT,3,3>::Identity();
    // do svd, the ideal eigen vector is the right eigen vector corresponding to
    // the smallest singular value 0
    Eigen::JacobiSVD<Eigen::Matrix<scalarT,3,3> > svd(rot_i, Eigen::ComputeFullV);
    Eigen::Matrix<scalarT,3,1> rot_axis = svd.matrixV().col(2);

    // to handle the direction of the rotation axis. because the calculated
    // angle is always within [0, pi], for other true angle value, the axis
    // direction should flipped so that the [angle] can be fit to [0, pi].
    scalarT ux = fabs(rot_axis(0));
    scalarT uy = fabs(rot_axis(1));
    scalarT uz = fabs(rot_axis(2));

    // [sin angle] is always >= 0, so the sign of uz_sin is up to uz, so is the
    // uy_sin anf ux_sin.
    scalarT uz_sin = rotmat(1,0)-rotmat(0,1);
    scalarT uy_sin = rotmat(0,2)-rotmat(2,0);
    scalarT ux_sin = rotmat(2,1)-rotmat(1,2);

    axis(0) = ux*getSign(ux_sin);
    axis(1) = uy*getSign(uy_sin);
    axis(2) = uz*getSign(uz_sin);
}

//------------------------------------------------------------
template<typename scalarT>
void rotationMatrixToQuaternion(const Eigen::Matrix<scalarT,3,3> &rotmat,
                                Eigen::Quaternion<scalarT> &quaternion)
{
    scalarT trace_r = rotmat(0,0)+rotmat(1,1)+rotmat(2,2);
    scalarT q0 = sqrt(trace_r+1)/2.0;

    // if q0 == 0, use the angle axis decomposition
    // else use equation
    if(fabs((double)q0)<DBL_EPSILON)
    {
        // first to calsulate the rotation axis and rotation angle
        Eigen::Matrix<scalarT,3,1> axis(0.0, 0.0, 0.0);
        scalarT angle = 0.0;
        rotationMatrixToAxisAngle<scalarT>(rotmat, axis, angle);

        // then construct the quaternion with the axis and angle
        scalarT c = cos(angle/2);
        scalarT s = sin(angle/2);
        quaternion.w() = c;
        quaternion.x() = axis(0)*s;
        quaternion.y() = axis(1)*s;
        quaternion.z() = axis(2)*s;
    }
    else
    {
        quaternion.w() = q0;
        quaternion.x() = (rotmat(2,1)-rotmat(1,2))/(4.0f*q0);
        quaternion.y() = (rotmat(0,2)-rotmat(2,0))/(4.0f*q0);
        quaternion.z() = (rotmat(1,0)-rotmat(0,1))/(4.0f*q0);
    }
}

//------------------------------------------------------------
template<typename scalarT>
void quaternionToRotationMatrix(const Eigen::Quaternion<scalarT> &quaternion,
                                Eigen::Matrix<scalarT,3,3> &rotmat)
{
    // 利用Eigen库自己的函数
    rotmat = quaternion.matrix();
}

//------------------------------------------------------------
// 欧拉角向四元数转换，转换公式中欧拉角旋转顺序为：X轴旋转--y轴旋转--z轴旋转
template<typename scalarT>
void eulerAngleToQuaternion_XYZ(const Eigen::Matrix<scalarT,3,1> &eulerAngles,
                                Eigen::Quaternion<scalarT> &quaternion)
{
    // euler angle
    scalarT phi = eulerAngles(2); // z axis rotation angle
    scalarT theta = eulerAngles(1); // y axis rotation angle
    scalarT psi = eulerAngles(0); // x axis rotation angle

    scalarT c_psi_h = cos(psi/2.0);
    scalarT s_psi_h = sin(psi/2.0);
    scalarT c_theta_h = cos(theta/2.0);
    scalarT s_theta_h = sin(theta/2.0);
    scalarT c_phi_h = cos(phi/2.0);
    scalarT s_phi_h = sin(phi/2.0);

    quaternion.w() = c_psi_h*c_theta_h*c_phi_h + s_psi_h*s_theta_h*s_phi_h;
    quaternion.x() = s_psi_h*c_theta_h*c_phi_h - c_psi_h*s_theta_h*s_phi_h;
    quaternion.y() = c_psi_h*s_theta_h*c_phi_h + s_psi_h*c_theta_h*s_phi_h;
    quaternion.z() = c_psi_h*c_theta_h*s_phi_h - s_psi_h*s_theta_h*c_phi_h;
}

//------------------------------------------------------------
// 四元数向欧拉角转换， 转换公式中欧拉角旋转顺序为：X轴旋转--y轴旋转--z轴旋转
template<typename scalarT>
void quaternionToEulerAngle_XYZ(const Eigen::Quaternion<scalarT> &quaternion,
                                Eigen::Matrix<scalarT,3,1> &eulerAngles)
{
    scalarT q0 = quaternion.w();
    scalarT q1 = quaternion.x();
    scalarT q2 = quaternion.y();
    scalarT q3 = quaternion.z();
    scalarT theta = asin(2.0*(q0*q2-q3*q1));
    scalarT phi = 0.0;
    scalarT psi = 0.0;

    // 根据 theta 值分三种情况讨论
    if(fabs((double)theta-M_PI/2.0f)<DBL_EPSILON)
    {
        // 情况1：theta = pi/2
        phi = 0.0;
        psi = phi - 2.0*atan2(q1, q0);
    }
    else if(fabs((double)theta+M_PI/2.0f)<DBL_EPSILON)
    {
        // 情况2：theta = -pi/2
        phi = 0.0;
        psi = -phi + 2.0*atan2(q1, q0);
    }
    else
    {
        // 情况3：|theta| ！= pi/2
        phi = atan2(2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2));
        psi = atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3));
    }

    eulerAngles(0) = psi; // x axis
    eulerAngles(1) = theta; // y axis
    eulerAngles(2) = phi; // z axis
}

//------------------------------------------------------------
// transformation from rotation matrix to the correspondent Euler angles,
// the rotation order of xyz axis is x-axis-rot --> y-axis-rot --> z-axis-rot.
template<typename scalarT>
void rotationMatrixToEulerAngle_XYZ(const Eigen::Matrix<scalarT,3,3> &rotmat,
                                    Eigen::Matrix<scalarT,3,1> &eulerAngles)
{
    scalarT r11 = rotmat(0,0); scalarT r12 = rotmat(0,1); scalarT r13 = rotmat(0,2);
    scalarT r21 = rotmat(1,0); scalarT r22 = rotmat(1,1); scalarT r23 = rotmat(1,2);
    scalarT r31 = rotmat(2,0); scalarT r32 = rotmat(2,1); scalarT r33 = rotmat(2,2);

    scalarT psi, theta, phi;
    if(fabs((double)r31+1)<DBL_EPSILON)
    {
        // theta = pi/2
        theta = M_PI/2.0;
        phi = 0.0;
        psi = phi + atan2(r12, r13);
    }
    else if(fabs((double)r31-1)<DBL_EPSILON)
    {
        // theta = -pi/2
        theta = -M_PI/2.0;
        phi = 0.0;
        psi = phi + atan2(-r12, -r13);
    }
    else
    {
        theta = -asin(r31);
        psi = atan2(r32/cos(theta), r33/cos(theta));
        phi = atan2(r21/cos(theta), r11/cos(theta));
    }

    eulerAngles(0) = psi; // x axis
    eulerAngles(1) = theta; // y axis
    eulerAngles(2) = phi; // z axis
}

//------------------------------------------------------------
// 欧拉角 到 旋转矩阵 的变换， 欧拉角旋转顺序为：X轴旋转--y轴旋转--z轴旋转
template<typename scalarT>
void eulerAnglesToRotationMatrix_XYZ(const Eigen::Matrix<scalarT,3,1> &eulerAngles,
                                     Eigen::Matrix<scalarT,3,3> &rotmat)
{
    // euler angle
    scalarT phi = eulerAngles(2); // z axis
    scalarT theta = eulerAngles(1); // y axis
    scalarT psi = eulerAngles(0); // x axis

    scalarT c_psi = cos(psi);
    scalarT s_psi = sin(psi);
    scalarT c_theta = cos(theta);
    scalarT s_theta = sin(theta);
    scalarT c_phi = cos(phi);
    scalarT s_phi = sin(phi);

    // 根据旋转公式
    rotmat(0,0) = c_theta*c_phi; rotmat(0,1) = s_psi*s_theta*c_phi-c_psi*s_phi; rotmat(0,2) = c_psi*s_theta*c_phi+s_psi*s_phi;
    rotmat(1,0) = c_theta*s_phi; rotmat(1,1) = s_psi*s_theta*s_phi+c_psi*c_phi; rotmat(2,1) = c_psi*s_theta*s_phi-s_psi*c_phi;
    rotmat(2,0) = -s_theta;      rotmat(2,1) = s_psi*c_theta;                   rotmat(2,2) = c_psi*c_theta;
}

