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
  fileSource source;
  source.initialize();
  source.setCamera(camera);
  
  //------------------------------------------------------------
  // visualization
  //pcl::visualization::CloudViewer viewer("viewer");
  
  //------------------------------------------------------------
  // state
  // CF: current frame
  // LKF: latest keyframe
  // RKF: reference keyframe
  Eigen::Matrix4f trans_CF_LKF = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f trans_LKF_RKF = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f trans_CF_RKF = Eigen::Matrix4f::Identity();
  SVector6f measure_CF_LKF;
  
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
    
    KALMAN_FILTER.updateMeasurement(measure_CF_LKF, current_frame.getTimeStamp());
    
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
    
    SVector6f guess_trans = KALMAN_FILTER.getRatioVec();
    cout << "guess: " << guess_trans << endl;
    guess_trans.vec_ = guess_trans.vec_*delta_time;
    cout << "guess: " << guess_trans << endl;
    
    guess_trans.toMatrix4f(guess);
    guess = trans_CF_LKF * guess;
    
    guess_trans.fromMatrix4f(guess);
    cout << "guess: " << guess_trans << endl;
    
    // icp registration
    bool has_converged_p = icp_runner.registerNewFrame(current_frame,
                                                       *transformed_tmp_cloud,
                                                       guess);
    if(has_converged_p==false) {
      cout << "icp did not converge, skip this frame. " << endl;
      continue;
    }
    
    // loop closure detect
    trans_CF_LKF = icp_runner.getIncrementalTransformation();
    measure_CF_LKF.fromMatrix4f(trans_CF_LKF);
    cout << "incre: " << measure_CF_LKF << endl;
    
    keyframe_check = loop_closure_check.detectNewKeyFrame(current_keyframe,
                                                          current_frame,
                                                          trans_CF_LKF);
    
    if(keyframe_check == loopClosure::CLOSE) {
      trans_CF_RKF = trans_LKF_RKF * trans_CF_LKF;
      measure_CF_LKF.fromMatrix4f(trans_CF_LKF);
      KALMAN_FILTER.updateMeasurement(measure_CF_LKF, current_timestamp);
      
      // refine current keyframe
      current_keyframe.refine(transformed_tmp_cloud);
      icp_runner.updateLatestKeyFrame(current_keyframe);
      
      cout << "trans_CF_RKF: " << endl
      << trans_CF_RKF << endl;
      cout << "too close" << endl;
    } else if(keyframe_check==loopClosure::KEYFRAME) {
      // a keyframe
      trans_CF_RKF = trans_LKF_RKF * trans_CF_LKF;
      
      // generate a new keyframe, add it to keyframe vector
      keyFrame a_keyframe(current_frame);
      a_keyframe.setTransToLKF(trans_CF_LKF);
      a_keyframe.setTransToRKF(trans_CF_RKF);
      KEYFRAMES_VECTOR.push_back(a_keyframe);
      current_keyframe = a_keyframe;
      
      trans_CF_LKF = Eigen::Matrix4f::Identity();
      measure_CF_LKF.fromMatrix4f(trans_CF_LKF);
      
      // update kalman filter measurement
      trans_LKF_RKF = trans_CF_RKF;
      KALMAN_FILTER.keyFrameReset();
      //KALMAN_FILTER.updateMeasurement(measure_CF_LKF, current_timestamp);
      
      // update icp target cloud
      icp_runner.updateLatestKeyFrame(current_keyframe);
      
      cout << "trans_CF_RKF: " << endl
      << trans_CF_RKF << endl;
      cout << "a new keyframe!" << endl;
    } else {
      cout << "frame is too far away from the latest key frame, "
      "there's probablely not a good registration. So skip this frame."
      << endl;
      continue;
    }
  }
  
  return 0;
}


