#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <Eigen/Dense>

#include "common_definitions.h"
#include "state_variables.h"

class kalmanFilter
{
public:
  typedef SVector12f StateVecT;
  typedef SVector6f MeasureVecT;
  typedef Eigen::Matrix<float, 12, 12> SS_MatT;
  typedef Eigen::Matrix<float, 6, 6> MM_MatT;
  typedef Eigen::Matrix<float, 6, 12> MS_MatT;
  typedef Eigen::Matrix<float, 12, 6> SM_MatT;
protected:
  StateVecT states_est_; // estimated state
  MeasureVecT measurement_; // input measurement
  
  SS_MatT states_cov_est_; // estimated state covariance matrix
  SS_MatT Q_; // model noise
  MM_MatT R_; // measurement noise covariance;
  
  StateVecT states_predict_; // predicted state
  MeasureVecT measurement_predict_; // predicted measurement
  SS_MatT states_cov_predict_; // predicted state covariance matrix
  
  SM_MatT kalman_gain_;
  
  SS_MatT F_; // model function
  MS_MatT H_; // measurement function
  
  float time_stamp_;
  float delta_time_; //;
public:
  kalmanFilter();
  ~kalmanFilter() { }
  
  inline float getTimestamp() {
    return time_stamp_;
  }
  inline StateVecT getStates() {
    return states_est_;
  }
  // 设置 模型函数 F
  inline void setModelFunction(SS_MatT& f) {
    F_ = f;
  }
  // 设置量测函数
  inline void setMeasurementFunction(MS_MatT& h) {
    H_ = h;
  }
  inline SVector6f getPoseVec() {
    return states_est_.getState();
  }
  inline SVector6f getRatioVec() {
    return states_est_.getRatio();
  }
  void keyFrameReset();
  void updateMeasurement(MeasureVecT& meas, float timestamp);
  
protected:
  void predict();
  void update();
  void update_F_Matrix();
};

#endif // KALMAN_FILTER_H
