#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <Eigen/Dense>

#include "common_definitions.h"
#include "state_variables.h"

class kalmanFilter
{
public:
    typedef Eigen::Matrix<float, 12, 1> StateVecT;
    typedef Eigen::Matrix<float, 6, 1> MeasureVecT;
    typedef Eigen::Matrix<float, 12, 12> SS_MatT;
    typedef Eigen::Matrix<float, 6, 6> MM_MatT;
    typedef Eigen::Matrix<float, 6, 12> MS_MatT;
    typedef Eigen::Matrix<float, 12, 6> SM_MatT;
protected:
    StateVecT states_est_; // estimated state
    MeasureVecT measurement_; // input measurement

    SS_MatT states_cov_est_; // estimated state covariance matrix
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
    ~kalmanFilter()
    {

    }

    float getTimestamp()
    {
        return time_stamp_;
    }

    StateVecT getStates()
    {
        return states_est_;
    }

    void updateMeasurement(MeasureVecT& meas, float timestamp)
    {
        delta_time_ = timestamp - time_stamp_;
        time_stamp_ = timestamp;
        measurement_ = meas;
        update_F_Matrix();

        predict();
        update();
    }
    // 设置 模型函数 F
    void setModelFunction(SS_MatT& f)
    {
        F_ = f;
    }
    // 设置量测函数
    void setMeasurementFunction(MS_MatT& h)
    {
        H_ = h;
    }

protected:
    void predict();
    void update();
    void  update_F_Matrix()
    {
        // update F_
        F_(0,3) = delta_time_; F_(1,4) = delta_time_; F_(2,5) = delta_time_;
        F_(6,9) = delta_time_; F_(7,10) = delta_time_; F_(8,11) = delta_time_;
    }
};

#endif // KALMAN_FILTER_H
