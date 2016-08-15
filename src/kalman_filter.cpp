#include "kalman_filter.h"
#include <ctime>

kalmanFilter::kalmanFilter()
{
    states_est_ = StateVecT::Zero();
    states_predict_ = states_est_;
    measurement_ = MeasureVecT::Zero();
    states_cov_est_ = SS_MatT::Identity()*0.01;
    R_ = MM_MatT::Identity()*0.01;
    F_ = SS_MatT::Identity();
    H_ = MS_MatT::Zero();

    time_stamp_ = 0.0;
    delta_time_ = 1.0;
    update_F_Matrix();
}

void kalmanFilter::predict()
{
    states_predict_ = F_*states_est_;
    measurement_predict_ = H_*states_predict_;
    states_cov_predict_ = F_*states_cov_est_*F_.transpose();
    MM_MatT S = H_*states_cov_predict_*H_.transpose()+R_;
    kalman_gain_ = states_cov_est_*H_.transpose()*S.inverse();
}

void kalmanFilter::update()
{
    states_est_ = states_predict_ + kalman_gain_*(measurement_ - measurement_predict_);
    SS_MatT tmp = kalman_gain_*(H_*states_cov_predict_*H_.transpose()+R_)*kalman_gain_.transpose();
    states_cov_est_ = states_cov_predict_ - tmp;
}
