#include "main_func.h"

#include <iostream>

#include "measurement_base.h"
#include <vector>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/gicp.h>

#include <iomanip>

int main_kalman(int argc, char ** argv)
{
    //------------------------------------------------------------
    // namespace
    using std::cout;
    cout << std::fixed;
    using std::endl;

    //------------------------------------------------------------
    // 数据来源， source
    cameraIntrinsicParameters camera;
    fileSource source("../data");
    source.depth_dir_ = "../data/depth_png";
    source.depth_marker_ = "";
    source.depth_extension_ = ".png";
    source.setCamera(camera);
    source.setStartIndex(150);
    source.setEndIndex(250);
    float downsample_leaf_size = 0.015;
    source.setDownSampleLeafSize(downsample_leaf_size);

    //------------------------------------------------------------
    // visualization
    //pcl::visualization::CloudViewer viewer("viewer");

    //------------------------------------------------------------
    // state
    Eigen::Matrix4f current_F_TO_LKF = Eigen::Matrix4f::Identity(); // current frame to latest key frame
    //Eigen::Matrix4f last_F_TO_LKF = Eigen::Matrix4f::Identity(); // last frame to latest key frame
    Eigen::Matrix4f latest_KF_TO_RKF = Eigen::Matrix4f::Identity(); // latest key frame to reference key frame
    Eigen::Matrix4f current_F_TO_RKF = Eigen::Matrix4f::Identity(); // current frame to reference key frame
    SVector6f measure_CF_TO_LKF;
    //sixDoFState measure_LKF_TO_RKF;
    SVector6f measure_CF_TO_RKF;

    //------------------------------------------------------------
    // icp
    icp icp_runner;
    icp_runner.initializeICPSolver();

    //------------------------------------------------------------
    // loop closure
    loopClosure loop_closure_check;
    loop_closure_check.initialize();

    //------------------------------------------------------------
    // keyframes
    std::vector<keyFrame, Eigen::aligned_allocator<keyFrame> > KEYFRAMES_VECTOR;

    //------------------------------------------------------------
    // kalman filter
    kalmanFilter KALMAN_FILTER;
    kalmanFilter::MS_MatT H = kalmanFilter::MS_MatT::Zero();
    H.block<3,3>(0,0) = Eigen::Matrix3f::Identity();
    H.block<3,3>(3,6) = Eigen::Matrix3f::Identity();
    KALMAN_FILTER.setMeasurementFunction(H);

    //------------------------------------------------------------
    // first frame
    frame current_frame;
    keyFrame current_keyframe;
    if(source.generateNewFrame(current_frame)==0)
    {
        keyFrame a_keyframe(current_frame);
        KEYFRAMES_VECTOR.push_back(a_keyframe);
        current_keyframe = a_keyframe;

        loop_closure_check.initialize();
        icp_runner.updateLatestKeyFrame(current_keyframe);

        KALMAN_FILTER.updateMeasurement(measure_CF_TO_RKF.vec_, current_frame.getTimeStamp());

        cout << "first frame, finished. " << endl;
    }
    else
        return EXIT_FAILURE;

    //------------------------------------------------------------
    // data processing loop
    float old_timestamp = current_frame.getTimeStamp();
    float current_timestamp = 0;
    PointCloudT_Ptr transformed_tmp_cloud(new PointCloudT);
    loopClosure::KeyFrameCheckResult keyframe_check;
    Eigen::Matrix4f guess = Eigen::Matrix4f::Identity();
    float delta_time;

    while (source.generateNewFrame(current_frame)==0)
    {
        // generate transformation guess
        current_timestamp = current_frame.getTimeStamp();
        delta_time = current_timestamp - old_timestamp;
        old_timestamp = current_timestamp;
        cout << "delta_time: " << delta_time << endl;

        kalmanFilter::StateVecT current_state =
                KALMAN_FILTER.getStates();

        SVector6f cf_to_lf_guess_trans; // current frame to last frame guess
        cf_to_lf_guess_trans.t() = current_state.block<3,1>(3,0)*delta_time;
        cf_to_lf_guess_trans.r() = current_state.block<3,1>(9,0)*delta_time;
        cf_to_lf_guess_trans.toMatrix4f(guess);
        guess = current_F_TO_LKF;// * guess;

        SVector6f cf_to_LKF_guess; // current frame to latest key frame guess
        cf_to_LKF_guess.fromMatrix4f(guess);
        cout << "guess: " << cf_to_LKF_guess << endl;

        // icp registration
        bool has_converged_p = icp_runner.registerNewFrame(current_frame,
                                                           *transformed_tmp_cloud,
                                                           guess);
        if(has_converged_p==false)
        {
            cout << "icp did not converge, skip this frame. " << endl;
            continue;
        }

        // loop closure detect
        current_F_TO_LKF = icp_runner.getIncrementalTransformation();
        measure_CF_TO_LKF.fromMatrix4f(current_F_TO_LKF);
        cout << "incre: " << measure_CF_TO_LKF << endl;

        keyframe_check = loop_closure_check.detectNewKeyFrame(current_keyframe,
                                                              current_frame,
                                                              current_F_TO_LKF);
        if(keyframe_check == loopClosure::TOO_CLOSE)
        {
            current_F_TO_RKF = latest_KF_TO_RKF * current_F_TO_LKF;
            measure_CF_TO_RKF.fromMatrix4f(current_F_TO_RKF);
            KALMAN_FILTER.updateMeasurement(measure_CF_TO_RKF.vec_, current_timestamp);

            // refine current keyframe
            current_keyframe.refine(transformed_tmp_cloud);
            icp_runner.updateLatestKeyFrame(current_keyframe);

            cout << "current_F_TO_RKF: " << endl
                 << current_F_TO_RKF << endl;
            cout << "too close" << endl;
        }
        else if(keyframe_check == loopClosure::TOO_FAR_AWAY)
        {
            cout << "frame is too far away from the latest key frame, "
                    "there's probablely not a good registration. So skip this frame."
                 << endl;
            continue;
        }
        else
        {
            // a keyframe
            current_F_TO_RKF = latest_KF_TO_RKF * current_F_TO_LKF;
            measure_CF_TO_RKF.fromMatrix4f(current_F_TO_RKF);
            current_F_TO_LKF = Eigen::Matrix4f::Identity();

            // update kalman filter measurement
            latest_KF_TO_RKF = current_F_TO_RKF;
            KALMAN_FILTER.updateMeasurement(measure_CF_TO_RKF.vec_, current_timestamp);

            // generate a new keyframe, add it to keyframe vector
            keyFrame a_keyframe(current_frame);
            KEYFRAMES_VECTOR.push_back(a_keyframe);
            current_keyframe = a_keyframe;

            // update icp target cloud
            icp_runner.updateLatestKeyFrame(current_keyframe);

            cout << "current_F_TO_RKF: " << endl
                 << current_F_TO_RKF << endl;
            cout << "a new keyframe!" << endl;
        }
    }

    return 0;
}


