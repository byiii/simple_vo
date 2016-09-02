#include "kalman_filter.h"
#include <ctime>

#include "common_definitions.h"
using std::cout;
using std::endl;

kalmanFilter::kalmanFilter()
{
  states_cov_est_ = SS_MatT::Identity()*0.03;
  Q_ = SS_MatT::Identity()*0.005;
  R_ = MM_MatT::Identity()*0.02;
  
  F_ = SS_MatT::Identity();
  H_ = MS_MatT::Zero();
  H_.block<3,3>(0,0) = Eigen::Matrix3f::Identity();
  H_.block<3,3>(3,6) = Eigen::Matrix3f::Identity();
  
  time_stamp_ = 0.0;
  delta_time_ = 1.0;
  update_F_Matrix();
}

void kalmanFilter::predict()
{
  states_predict_.vec_ = F_*states_est_.vec_;
  measurement_predict_.vec_ = H_*states_predict_.vec_;
  states_cov_predict_ = F_*states_cov_est_*F_.transpose() + Q_;
  MM_MatT S = H_*states_cov_predict_*H_.transpose()+R_;
  kalman_gain_ = states_cov_est_*H_.transpose()*S.inverse();
}

void kalmanFilter::update()
{
  states_est_.vec_ = states_predict_.vec_ + kalman_gain_*(measurement_.vec_ - measurement_predict_.vec_);
  SS_MatT tmp = kalman_gain_*(H_*states_cov_predict_*H_.transpose()+R_)*kalman_gain_.transpose();
  states_cov_est_ = states_cov_predict_ - tmp;
}

void kalmanFilter::update_F_Matrix()
{
  // update F_
  F_(0,3) = delta_time_; F_(1,4) = delta_time_; F_(2,5) = delta_time_;
  F_(6,9) = delta_time_; F_(7,10) = delta_time_; F_(8,11) = delta_time_;
}
void kalmanFilter::updateMeasurement(MeasureVecT& meas, float timestamp)
{
  delta_time_ = timestamp - time_stamp_;
  time_stamp_ = timestamp;
  measurement_ = meas;
  update_F_Matrix();
  
  predict();
  update();
  
  //   cout << "states: \n" << states_est_ << endl;
  //   cout << "states pre: \n" << states_predict_ << endl;
  //   cout << "states: \n" << states_cov_est_ << endl;
  //   cout << "F_: \n" << F_ << endl;
}

void kalmanFilter::keyFrameReset() {
  states_est_(0) = 0.0;
  states_est_(1) = 0.0;
  states_est_(2) = 0.0;
  states_est_(6) = 0.0;
  states_est_(7) = 0.0;
  states_est_(8) = 0.0;
}