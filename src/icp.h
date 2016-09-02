#ifndef ICP_H
#define ICP_H

#include "common_definitions.h"
#include "frame.h"
#include "key_frame.h"

#include <string>

#include <pcl/registration/incremental_registration.h>

// 核心算法，用迭代最近点配准两个点云，得到4x4相对旋转矩阵
class icp
{
public:
  typedef pcl::Registration< PointT, PointT>::Ptr RegistrationPtr;
  
  // ICP算法 必要参数
  struct parameters
  {
    enum {ICP_SIMPLE=0,
      ICP_WITH_NORAML, // not implemented here, so no use
      ICP_NON_LINEAR,
      ICP_GENERALIZED};
      // ICP 算法的类型
      int icp_type;
      // source 和 target 匹配点之间允许的最大距离
      float max_correspondence_distance;
      // 最大迭代次数
      int max_iteration;
      // 前一个变换矩阵和当前变换矩阵的差异小于阈值时，就认为已经收敛了
      float transformation_epsilon;
      // 还有一条收敛条件是均方误差和小于阈值， 停止迭代
      float euclidean_fitness_epsilon;
      
      parameters()
      {
        icp_type = ICP_NON_LINEAR;
        max_correspondence_distance = 0.05;
        max_iteration = 100;
        transformation_epsilon = 1e-8;
        euclidean_fitness_epsilon = 0.01;
      }
  };
  
  icp();
  ~icp()
  {}
  
  // 更新最新关键帧
  void updateLatestKeyFrame(keyFrame& aKeyFrame);
  // 重置
  void resetData();
  // 初始化
  void initializeICPSolver();
  // 将最近帧配准到最新关键帧当中
  inline bool registerNewFrame(frame& newFrame,
                               PointCloudT& transformed_cloud,
                               Eigen::Matrix4f& guess) 
  {
    return registerNewFrameToLatestKeyFrame(newFrame, transformed_cloud, guess);
  }
  inline bool registerNewFrame(frame& newFrame,
                               PointCloudT& transformed_cloud) 
  {
    Eigen::Matrix4f guess = Eigen::Matrix4f::Identity();
    return registerNewFrameToLatestKeyFrame(newFrame, transformed_cloud, guess);
  }
  // 给出增量变换矩阵
  inline Eigen::Matrix4f getIncrementalTransformation() {
    return incremental_transformation_;
  }
  
protected:
  RegistrationPtr icp_solver_;
  keyFrame* latest_keyframe_;
  Eigen::Matrix4f incremental_transformation_;
  parameters params_;
  
  // 将最近帧配准到最新关键帧当中
  bool registerNewFrameToLatestKeyFrame(frame& newFrame,
                                        PointCloudT& transformed_cloud,
                                        Eigen::Matrix4f& guess);
};

#endif // ICP_H
