#include "frame.h"

#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/kdtree/kdtree.h>

#include "common_definitions.h" // 设置参数

using namespace pcl;
using namespace std;


frame::frame()
{
  point_cloud_.reset(new PointCloudT);
}
// 构造函数，从点云数据和时间标记生成
frame::frame(const PointCloudT cloud, float frame_time_stamp, int id)
{
  generate_time_ = frame_time_stamp;
  point_cloud_ = cloud.makeShared();
  id_ = id;
}

frame& frame::operator=(const frame &another)
{
  this->generate_time_ = another.generate_time_;
  this->point_cloud_.reset(new PointCloudT);
  this->point_cloud_ = another.point_cloud_->makeShared();
  this->id_ = another.id_;
  return *this;
}

// 点云下采样
void frame::downsampleCloud()
{
  float leaf_size = 0.01;
  CONFIGURE.get<float>("frame_downsample_leaf_size", leaf_size);
  
  VoxelGrid<PointT> voxel_down;
  voxel_down.setLeafSize(leaf_size, leaf_size, leaf_size);
  voxel_down.setInputCloud(this->point_cloud_);
  voxel_down.filter(*point_cloud_);
  
  cout << "frame::downsampleCloud, done" << endl;
}

// 采集关键点
void frame::getUniformKeyPoints(float search_radius)
{
  keypoints_.reset(new PointCloudT);
  UniformSampling<PointT> uniform_sample;
  uniform_sample.setInputCloud(this->point_cloud_);
  uniform_sample.setRadiusSearch((double)search_radius);
  uniform_sample.filter(*keypoints_);
  
  cout << "frame::getUniformKeyPoints, done" << endl;
}

// 计算点云中的法向量
void frame::estimateNormalAtKeyPoints()
{
  int normal_num_of_thread = 1;
  CONFIGURE.get<int>("normal_n_threads", normal_num_of_thread);
  float search_radius;
  CONFIGURE.get<float>("normal_search_radius", search_radius);
  
  NormalEstimationOMP<PointT, NormalT> normal_est;
  normal_est.setNumberOfThreads(normal_num_of_thread);
  normal_est.setInputCloud(this->keypoints_);
  normal_est.setSearchSurface(this->point_cloud_);
  normal_est.setRadiusSearch(search_radius);
  normal_est.compute(*this->normal_Cloud_);
  
  cout << "frame::estimateNormalAtKeyPoints, done" << endl;
  return;
}

void frame::estimateNormalAtKeyPoints(float search_radius)
{
  int normal_num_of_thread = 1;
  CONFIGURE.get<int>("normal_n_threads", normal_num_of_thread);
  
  NormalEstimationOMP<PointT, NormalT> normal_est;
  search::KdTree<PointT>::Ptr kdtree(new search::KdTree<PointT>());
  normal_est.setNumberOfThreads(normal_num_of_thread);
  normal_est.setInputCloud(this->keypoints_);
  normal_est.setSearchSurface(this->point_cloud_);
  normal_est.setSearchMethod(kdtree);
  normal_est.setRadiusSearch(search_radius);
  normal_est.compute(*this->normal_Cloud_);
  
  cout << "frame::estimateNormalAtKeyPoints, done" << endl;
  return;
}
