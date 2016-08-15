#include "axis_angle.h"
#include "axis_angle.hpp"

//------------------------------------------------------------
// use Eigen library
// 实例化模板函数，用于 float 和 double
//------------------------------------------------------------

//------------------------------------------------------------
template void axisAngleToQuaternion(
const Eigen::Matrix<float,3,1> &axis,
float angle,
Eigen::Quaternion<float> &quaternion);

template void axisAngleToQuaternion(
const Eigen::Matrix<double,3,1> &axis,
double angle,
Eigen::Quaternion<double> &quaternion);

//------------------------------------------------------------
template void quaternionToAxisAngle (
const Eigen::Quaternion<float> &quaternion,
Eigen::Matrix<float,3,1> &axis,
float &angle);

template void quaternionToAxisAngle (
const Eigen::Quaternion<double> &quaternion,
Eigen::Matrix<double,3,1> &axis,
double &angle);

//------------------------------------------------------------
template void axisAngleToRotationMatrix(
const Eigen::Matrix<float,3,1> &axis,
float angle,
Eigen::Matrix<float,3,3> &rotmat);

template void axisAngleToRotationMatrix(
const Eigen::Matrix<double,3,1> &axis,
double angle,
Eigen::Matrix<double,3,3> &rotmat);

//------------------------------------------------------------
template void rotationMatrixToAxisAngle(
const Eigen::Matrix<float,3,3> &rotmat,
Eigen::Matrix<float,3,1> &axis,
float &angle);

template void rotationMatrixToAxisAngle(
const Eigen::Matrix<double,3,3> &rotmat,
Eigen::Matrix<double,3,1> &axis,
double &angle);

//------------------------------------------------------------
template void rotationMatrixToQuaternion(
const Eigen::Matrix<float,3,3> &rotmat,
Eigen::Quaternion<float> &quaternion);

template void rotationMatrixToQuaternion(
const Eigen::Matrix<double,3,3> &rotmat,
Eigen::Quaternion<double> &quaternion);

//------------------------------------------------------------
template void quaternionToRotationMatrix(
const Eigen::Quaternion<float> &quaternion,
Eigen::Matrix<float,3,3> &rotmat);

template void quaternionToRotationMatrix(
const Eigen::Quaternion<double> &quaternion,
Eigen::Matrix<double,3,3> &rotmat);

//------------------------------------------------------------
template void eulerAngleToQuaternion_XYZ(
const Eigen::Matrix<float,3,1> &eulerAngles,
Eigen::Quaternion<float> &quaternion);

template void eulerAngleToQuaternion_XYZ(
const Eigen::Matrix<double,3,1> &eulerAngles,
Eigen::Quaternion<double> &quaternion);

//------------------------------------------------------------
template void quaternionToEulerAngle_XYZ(
const Eigen::Quaternion<float> &quaternion,
Eigen::Matrix<float,3,1> &eulerAngles);

template void quaternionToEulerAngle_XYZ(
const Eigen::Quaternion<double> &quaternion,
Eigen::Matrix<double,3,1> &eulerAngles);

//------------------------------------------------------------
template void rotationMatrixToEulerAngle_XYZ(
const Eigen::Matrix<float,3,3> &rotmat,
Eigen::Matrix<float,3,1> &eulerAngles);

template void rotationMatrixToEulerAngle_XYZ(
const Eigen::Matrix<double,3,3> &rotmat,
Eigen::Matrix<double,3,1> &eulerAngles);

//------------------------------------------------------------
template void eulerAnglesToRotationMatrix_XYZ(
const Eigen::Matrix<float,3,1> &eulerAngles,
Eigen::Matrix<float,3,3> &rotmat);

template void eulerAnglesToRotationMatrix_XYZ(
const Eigen::Matrix<double,3,1> &eulerAngles,
Eigen::Matrix<double,3,3> &rotmat);




//------------------------------------------------------------
// double array representation
//------------------------------------------------------------


//------------------------------------------------------------
///
/// \brief axisAngleRotationMatrix:
/// compose a rotation matrix from given rotation axis and angle,
/// make sure axis is of length 3, rotmat is of length 9.
/// \param axis: input, length 3
/// \param angle: input
/// \param rotmat: output, rotation matrix, length 9
/// \param normalized_p: whether the axis is normalized
///
void axisAngleToRotationMatrix(const double *axis, double angle, double *rotmat, bool normalized_p)
{
    double ux = axis[0];
    double uy = axis[1];
    double uz = axis[2];

    // check if the axis is an unit vector, if not, convert to unit vector
    if(!normalized_p)
    {
        double axis_length = std::sqrt(ux*ux+uy*uy+uz*uz);
        ux = ux/axis_length;
        uy = uy/axis_length;
        uz = uz/axis_length;
    }

    // use c and s to represent cos angle and sin angle
    double c = cos(angle);
    double s = sin(angle);

    // compute the rotation matrix element
    rotmat[0] = ux*ux*(1-c)+c;    rotmat[1] = ux*uy*(1-c)-uz*s; rotmat[2] = ux*uz*(1-c)+uy*s;
    rotmat[3] = ux*uy*(1-c)+uz*s; rotmat[4] = uy*uy*(1-c)+c;    rotmat[5] = uy*uz*(1-c)-ux*s;
    rotmat[6] = ux*uz*(1-c)-uy*s; rotmat[7] = uy*uz*(1-c)+ux*s; rotmat[8] = uz*uz*(1-c)+c;
}


//------------------------------------------------------------
///
/// \brief rotationMatrixToAxisAngle:
/// decomposite the rotation matrix to extract the rotating axis and angle.
/// \param rotmat: input, length 9
/// \param axis: output, rotaion axis, length 3;
/// \param angle: output, rotation angle, [0, pi]
///
void rotationMatrixToAxisAngle(const double *mat, double *axis, double &angle)
{
    // copy input mat to rotmat, because i do not know how to use [Eigen::Map]
    // with [const double *]
    double rotmat[9] = {0.0};
    for(size_t i=0; i<9; ++i)
        rotmat[i] = mat[i];

    // compute the matrix trace to get rotation angle.
    // here rotation angle is in the interval [0, pi],
    // so attention should paid to the rotation angle value outside this
    // interval.
    double trace = rotmat[0]+rotmat[4]+rotmat[8];
    double cos_theta = 0.5*(trace-1);
    if(fabs(cos_theta+1)<2*DBL_EPSILON)
        angle = M_PI;
    else
        angle = acos(cos_theta);

    // convert [double *] to [Eigen::Matrix3d],
    // take care that [Eigen::Matrix] is default to use the column major, so here
    // use the transpose.
    Eigen::Matrix3d rot = Eigen::Map<Eigen::MatrixXd>(rotmat, 3, 3).transpose();

    // compute [R-I], to get the eigen vector corresponding to the eigen velue 1
    Eigen::Matrix3d rot_i = rot - Eigen::Matrix3d::Identity();
    // do svd, the ideal eigen vector is the right eigen vector corresponding to
    // the smallest singular value 0
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(rot_i, Eigen::ComputeFullV);
    Eigen::Vector3d rot_axis = svd.matrixV().col(2);

    // to handle the direction of the rotation axis. because the calculated
    // angle is always within [0, pi], for other true angle value, the axis
    // direction should flipped so that the [angle] can be fit to [0, pi].
    double ux = fabs(rot_axis(0));
    double uy = fabs(rot_axis(1));
    double uz = fabs(rot_axis(2));

    // [sin angle] is always >= 0, so the sign of uz_sin is up to uz, so is the
    // uy_sin anf ux_sin.
    double uz_sin = rot(1,0)-rot(0,1);
    double uy_sin = rot(0,2)-rot(2,0);
    double ux_sin = rot(2,1)-rot(1,2);

    axis[0] = ux*getSign(ux_sin);
    axis[1] = uy*getSign(uy_sin);
    axis[2] = uz*getSign(uz_sin);
}



//------------------------------------------------------------
///
/// \brief rotationMatrixToAxisAngle_simple:
/// decomposite the rotation matrix to extract the rotating axis and angle.
/// simpler approach
/// \param rotmat: input, length 9
/// \param axis: output, rotaion axis, length 3;
/// \param angle: output, rotation angle, [0, pi]
///
void rotationMatrixToAxisAngle_simple(const double *rotmat, double *axis, double &angle)
{
    // compute the matrix trace to get rotation angle.
    // here rotation angle is in the interval [0, pi],
    // so attention should paid to the rotation angle value outside this
    // interval.
    double trace = rotmat[0]+rotmat[4]+rotmat[8];
    double cos_theta = 0.5*(trace-1.0);

    // three conditions for rotation angle
    if(fabs(cos_theta-1)<DBL_EPSILON)
    {
        // if angle = 0, the there is no rotation at all,
        // we can assign any value to the axis. here simply choose the x axis
        axis[0] = 1.0;
        axis[1] = 0.0;
        axis[2] = 0.0;

        angle =0.0;
    }
    else if((fabs(cos_theta+1)<2*DBL_EPSILON))
    {
        // if the rotation angle is equal to PI
        double ux_sq = (rotmat[0]+1)/2.0;
        double uy_sq = (rotmat[4]+1)/2.0;
        double uz_sq = (rotmat[8]+1)/2.0;
        double ux = sqrt(ux_sq);
        double uy = sqrt(uy_sq);
        double uz = sqrt(uz_sq);
        axis[0] = ux;
        axis[1] = uy*getSign(rotmat[1]+rotmat[3]);
        axis[2] = uz*getSign(rotmat[2]+rotmat[6]);

        angle = M_PI;
    }
    else
    {
        angle = acos(cos_theta);
        // other condition
        // notice that sin_theta is always positive.
        double sin_theta = sqrt(1-cos_theta*cos_theta);
        axis[0] = (rotmat[7]-rotmat[5])/2.0/sin_theta;
        axis[1] = (rotmat[2]-rotmat[6])/2.0/sin_theta;
        axis[2] = (rotmat[3]-rotmat[1])/2.0/sin_theta;
    }
}


//------------------------------------------------------------
///
/// \brief rotationMatrixToQuaternion
/// extract Quaternion from rotation matrix
/// \param rotmat: input, rotation matrix
/// \param quaterntion, output
///
void rotationMatrixToQuaternion(const double *rotmat, double *quaternion)
{
    double trace_r = rotmat[0]+rotmat[4]+rotmat[8];
    double q0 = sqrt(trace_r+1)/2.0;

    // if q0 == 0, use the angle axis decomposition
    // else use equation
    if(fabs(q0)<DBL_EPSILON)
    {
        // first to calsulate the rotation axis and rotation angle
        double axis[3] = {0.0};
        double angle = 0.0;
        rotationMatrixToAxisAngle(rotmat, axis, angle);

        // then construct the quaternion with the axis and angle
        double c = cos(angle/2);
        double s = sin(angle/2);
        quaternion[0] = c;
        quaternion[1] = axis[0]*s;
        quaternion[2] = axis[1]*s;
        quaternion[3] = axis[2]*s;
    }
    else
    {
        quaternion[0] = q0;
        quaternion[1] = (rotmat[7]-rotmat[5])/(4.0*q0);
        quaternion[2] = (rotmat[2]-rotmat[6])/(4.0*q0);
        quaternion[3] = (rotmat[3]-rotmat[1])/(4.0*q0);
    }
}

//------------------------------------------------------------
///
/// \brief quaternionToRotationMatrix
/// compute rotation matrix directly from quaternion
/// \param quaternion
/// \param rotmat
///
void quaternionToRotationMatrix(const double *quaternion, double *rotmat)
{
    double q0 = quaternion[0];
    double q1 = quaternion[1];
    double q2 = quaternion[2];
    double q3 = quaternion[3];

    rotmat[0] = 1-2*q2*q2-2*q3*q3; rotmat[1] = 2*(q1*q2-q3*q0);   rotmat[2] = 2*(q1*q3+q2*q0);
    rotmat[3] = 2*(q1*q2+q3*q0);   rotmat[4] = 1-2*q1*q1-2*q3*q3; rotmat[5] = 2*(q2*q3-q1*q0);
    rotmat[6] = 2*(q1*q3-q2*q0);   rotmat[7] = 2*(q2*q3+q1*q0);   rotmat[8] = 1-2*q1*q1-2*q2*q2;
}

//------------------------------------------------------------
///
/// \brief axisAngleToQuaternion
/// \param axis
/// \param angle
/// \param quaternion
///
void axisAngleToQuaternion(const double *axis,
                           double angle,
                           double *quaternion)
{
    double c = cos(angle/2);
    double s = sin(angle/2);
    quaternion[0] = c;
    quaternion[1] = axis[0]*s;
    quaternion[2] = axis[1]*s;
    quaternion[3] = axis[2]*s;
}

//------------------------------------------------------------
///
/// \brief quaternionToAxisAngle
/// \param quaternion
/// \param axis
/// \param angle
///
void quaternionToAxisAngle(const double *quaternion,
                           double *axis,
                           double &angle)
{
    // assume angle is in [0, pi]
    double q0 = quaternion[0];
    double half_angle = 0.0;

    if(q0<0)
    {
        // means that angle is beyond [0, pi], >pi
        half_angle = acos(q0);
        double s = sin(half_angle);
        angle = half_angle*2.0-M_PI;
        axis[0] = -quaternion[1]/s;
        axis[1] = -quaternion[2]/s;
        axis[2] = -quaternion[3]/s;
    }
    else if(fabs(q0)<DBL_EPSILON)
    {
        // q0 = 0, angle = pi
        angle = M_PI;
        axis[0] = quaternion[1];
        axis[1] = quaternion[2];
        axis[2] = quaternion[3];
    }
    else
    {
        // q0 > 0
        half_angle = acos(q0);
        double s = sin(half_angle);
        angle = half_angle*2.0;
        axis[0] = quaternion[1]/s;
        axis[1] = quaternion[2]/s;
        axis[2] = quaternion[3]/s;
    }
}

//------------------------------------------------------------
///
/// \brief eulerAngleToQuaternion
/// convert euler angles to quaternion, rotatioin order: x->y->z
/// \param eulerAngles: input, euler angles, [angle_x, angle_y, angle_z]
/// \param quaternion: output
///
void eulerAngleToQuaternion_XYZ(const double *eulerAngles, double *quaternion)
{
    // euler angle
    double phi = eulerAngles[2]; // z axis
    double theta = eulerAngles[1]; // y axis
    double psi = eulerAngles[0]; // x axis

    double c_psi_h = cos(psi/2.0);
    double s_psi_h = sin(psi/2.0);
    double c_theta_h = cos(theta/2.0);
    double s_theta_h = sin(theta/2.0);
    double c_phi_h = cos(phi/2.0);
    double s_phi_h = sin(phi/2.0);

    quaternion[0] = c_psi_h*c_theta_h*c_phi_h + s_psi_h*s_theta_h*s_phi_h;
    quaternion[1] = s_psi_h*c_theta_h*c_phi_h - c_psi_h*s_theta_h*s_phi_h;
    quaternion[2] = c_psi_h*s_theta_h*c_phi_h + s_psi_h*c_theta_h*s_phi_h;
    quaternion[3] = c_psi_h*c_theta_h*s_phi_h - s_psi_h*s_theta_h*c_phi_h;
}


//------------------------------------------------------------
///
/// \brief quaternionToEulerAngle
/// convert quaternion to euler angles
/// \param quaternion: input
/// \param eulerAngles: output, [angle_x, angle_y, angle_z]
///
void quaternionToEulerAngle_XYZ(const double *quaternion, double *eulerAngles)
{
    double q0 = quaternion[0];
    double q1 = quaternion[1];
    double q2 = quaternion[2];
    double q3 = quaternion[3];
    double theta = asin(2.0*(q0*q2-q3*q1));
    double phi = 0.0;
    double psi = 0.0;
    if(fabs(theta-M_PI/2.0)<DBL_EPSILON)
    {
        // theta = pi/2
        phi = 0.0;
        psi = phi - 2.0*atan2(q1, q0);
    }
    else if(fabs(theta+M_PI/2.0)<DBL_EPSILON)
    {
        // theta = -pi/2
        phi = 0.0;
        psi = -phi + 2.0*atan2(q1, q0);
    }
    else
    {
        phi = atan2(2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2));
        psi = atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3));
    }

    eulerAngles[0] = psi; // x axis
    eulerAngles[1] = theta; // y axis
    eulerAngles[2] = phi; // z axis
}


//------------------------------------------------------------
///
void rotationMatrixToEulerAngle_XYZ(const double *rotmat, double *eulerAngles)
{
    double r11 = rotmat[0]; double r12 = rotmat[1]; double r13 = rotmat[2];
    double r21 = rotmat[3]; double r22 = rotmat[4]; double r23 = rotmat[5];
    double r31 = rotmat[6]; double r32 = rotmat[7]; double r33 = rotmat[8];

    double psi, theta, phi;
    if(fabs(r31+1)<DBL_EPSILON)
    {
        // theta = pi/2
        theta = M_PI/2.0;
        phi = 0.0;
        psi = phi + atan2(r12, r13);
    }
    else if(fabs(r31-1)<DBL_EPSILON)
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

    eulerAngles[0] = psi; // x axis
    eulerAngles[1] = theta; // y axis
    eulerAngles[2] = phi; // z axis
}


//------------------------------------------------------------
///
void eulerAnglesToRotationMatrix_XYZ(const double *eulerAngles, double *rotmat)
{
    // euler angle
    double phi = eulerAngles[2]; // z axis
    double theta = eulerAngles[1]; // y axis
    double psi = eulerAngles[0]; // x axis

    double c_psi = cos(psi);
    double s_psi = sin(psi);
    double c_theta = cos(theta);
    double s_theta = sin(theta);
    double c_phi = cos(phi);
    double s_phi = sin(phi);

    rotmat[0] = c_theta*c_phi; rotmat[1] = s_psi*s_theta*c_phi-c_psi*s_phi; rotmat[2] = c_psi*s_theta*c_phi+s_psi*s_phi;
    rotmat[3] = c_theta*s_phi; rotmat[4] = s_psi*s_theta*s_phi+c_psi*c_phi; rotmat[5] = c_psi*s_theta*s_phi-s_psi*c_phi;
    rotmat[6] = -s_theta;      rotmat[7] = s_psi*c_theta;                   rotmat[8] = c_psi*c_theta;
}
