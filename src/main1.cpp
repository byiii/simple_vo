#include "main_func.h"

#include <iostream>

#include "measurement_base.h"
#include <vector>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/gicp.h>

int main_simple(int argc, char ** argv)
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
    Eigen::Matrix4f measure_CF_TO_LKF = Eigen::Matrix4f::Identity();
    //Eigen::Matrix4f partial_full_transformation = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f measure_LKF_TO_RKF = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f measure_CF_TO_RKF = Eigen::Matrix4f::Identity();
    sixDoFState incremental_state;
    sixDoFState old_state;
    sixDoFState latest_state;
    sixDoFState state_ratio;

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

        cout << "first frame, finished. " << endl;
    }
    else
        return EXIT_FAILURE;

    //------------------------------------------------------------
    // data processing loop
    frame old_frame = current_frame;
    PointCloudT_Ptr transformed_tmp_cloud(new PointCloudT);
    loopClosure::KeyFrameCheckResult keyframe_check;
    Eigen::Matrix4f guess = Eigen::Matrix4f::Identity();
    float delta_time;
    while (source.generateNewFrame(current_frame)==0)
    {
        // generate transformation guess
        delta_time = current_frame.getTimeStamp() - old_frame.getTimeStamp();
        cout << "delta_time: " << delta_time << endl;

        sixDoFState guess_trans;
        guess_trans.t() = state_ratio.t() * delta_time;
        guess_trans.r() = state_ratio.r() * delta_time;
        guess_trans.toMatrix4f(guess);
        guess = measure_CF_TO_LKF * guess;
        guess_trans.fromMatrix4f(guess);
        cout << "guess: " << guess_trans << endl;

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
        measure_CF_TO_LKF = icp_runner.getIncrementalTransformation();
        incremental_state.fromMatrix4f(measure_CF_TO_LKF);
        cout << "incre: " << incremental_state << endl;

        keyframe_check = loop_closure_check.detectNewKeyFrame(current_keyframe,
                                                              current_frame,
                                                              measure_CF_TO_LKF);
        if(keyframe_check == loopClosure::TOO_CLOSE)
        {
            measure_CF_TO_RKF = measure_LKF_TO_RKF * measure_CF_TO_LKF;

            latest_state.fromMatrix4f(measure_CF_TO_LKF);
            state_ratio.t() = (latest_state.t() - old_state.t())/delta_time;
            state_ratio.r() = (latest_state.r() - old_state.r())/delta_time;

            // refine current keyframe
            current_keyframe.refine(transformed_tmp_cloud);
            // downsample current keyframe's point cloud;
            current_keyframe.downsampleCloud(downsample_leaf_size);
            icp_runner.updateLatestKeyFrame(current_keyframe);

            cout << "measure_CF_TO_RKF: " << endl
                 << measure_CF_TO_RKF << endl;
            cout << "too close" << endl;
        }
        else if(keyframe_check == loopClosure::TOO_FAR_AWAY)
        {
            cout << "frame is too far away from the latest key frame, "
                    "there's probablely not a good registration. So, skip this frame."
                 << endl;
            continue;
        }
        else
        {
            // a keyframe
            measure_CF_TO_RKF = measure_LKF_TO_RKF * measure_CF_TO_LKF;
            measure_LKF_TO_RKF = measure_CF_TO_RKF;
            latest_state.fromMatrix4f(measure_CF_TO_LKF);
            state_ratio.t() = (latest_state.t() - old_state.t())/delta_time;
            state_ratio.r() = (latest_state.r() - old_state.r())/delta_time;

            latest_state = sixDoFState();
            measure_CF_TO_LKF = Eigen::Matrix4f::Identity();

            // generate a new keyframe, add it to keyframe vector
            keyFrame a_keyframe(current_frame);
            KEYFRAMES_VECTOR.push_back(a_keyframe);
            current_keyframe = a_keyframe;
            // update icp target cloud
            icp_runner.updateLatestKeyFrame(current_keyframe);

            cout << "measure_CF_TO_RKF: " << endl
                 << measure_CF_TO_RKF << endl;
            cout << "a new keyframe!" << endl;
        }

        // update state variables
        old_state = latest_state;
        old_frame = current_frame;
    }

    return 0;
}
