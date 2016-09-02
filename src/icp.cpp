#include "icp.h"

#include "common_definitions.h"

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/gicp.h>

using namespace std;
using namespace pcl;
icp::icp()
{
  this->icp_solver_ = NULL;
  this->latest_keyframe_ = NULL;
}

// 初始化
void icp::initializeICPSolver()
{
  CONFIGURE.get<int>("icp_type", 
                     params_.icp_type);
  CONFIGURE.get<int>("icp_max_iteration", 
                     params_.max_iteration);
  CONFIGURE.get<float>("icp_transformation_epsilon", 
                       params_.transformation_epsilon);
  CONFIGURE.get<float>("icp_euclidean_fitness_epsilon", 
                       params_.euclidean_fitness_epsilon);
  CONFIGURE.get<float>("icp_max_correspondence_distance", 
                       params_.max_correspondence_distance);
  
  switch(params_.icp_type)
  {
    case parameters::ICP_NON_LINEAR:
      icp_solver_.reset(new IterativeClosestPointNonLinear<PointT, PointT>);
    case parameters::ICP_GENERALIZED:
      icp_solver_.reset(new GeneralizedIterativeClosestPoint<PointT, PointT>);
    default:
      icp_solver_.reset(new IterativeClosestPoint<PointT, PointT>);
  }
  
  icp_solver_->setEuclideanFitnessEpsilon(params_.euclidean_fitness_epsilon);
  icp_solver_->setMaxCorrespondenceDistance(params_.max_correspondence_distance);
  icp_solver_->setMaximumIterations(params_.max_iteration);
  icp_solver_->setTransformationEpsilon(params_.transformation_epsilon);
}

// 更新最新关键帧
void icp::updateLatestKeyFrame(keyFrame& aKeyFrame) {
  if(icp_solver_==NULL)
    initializeICPSolver();
  this->latest_keyframe_ = &aKeyFrame;
  icp_solver_->setInputTarget(latest_keyframe_->getPointCloud()->makeShared());
}

void icp::resetData() {
  latest_keyframe_ = NULL;
  incremental_transformation_ = Eigen::Matrix4f::Identity();
}
// 将最近帧配准到最新关键帧当中
bool icp::registerNewFrameToLatestKeyFrame(frame& newFrame,
                                           PointCloudT& transformed_cloud,
                                           Eigen::Matrix4f& guess)
{
  icp_solver_->setInputSource(newFrame.getPointCloud());
  //
  icp_solver_->align(transformed_cloud, guess);
  //icp_solver_->align(transformed_cloud);
  incremental_transformation_ = icp_solver_->getFinalTransformation();
  
  return icp_solver_->hasConverged();
}
