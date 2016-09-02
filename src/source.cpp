#include "source.h"

#include <iostream>
#include <stdio.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using std::cout;
using std::endl;

rgbdSource::rgbdSource()
{
}


fileSource::fileSource(float fr, float ct)
{
  setFrameRate(fr);
  current_time_ = ct;
  frame_count_ = 0;
}


void fileSource::setFrameRate(float f)
{
  if(f!=0)
  {
    frame_rate_ = f;
    frame_time_interval_ = 1/f;
  }
  else
  {
    frame_rate_ = 1;
    frame_time_interval_ = 1/frame_rate_;
  }
}


void fileSource::initialize()
{
  const char *tmp = "";
  CONFIGURE.get<const char*>("source_dir", tmp);
  param_.source_dir = tmp;
  CONFIGURE.get<const char*>("source_depth_dir", tmp);
  param_.depth_dir = tmp;
  CONFIGURE.get<const char*>("source_depth_marker", tmp);
  param_.depth_marker = tmp;
  CONFIGURE.get<const char*>("source_depth_extension", tmp);
  param_.depth_extension = tmp;
  CONFIGURE.get<unsigned int>("source_start_idx", param_.start_idx);
  CONFIGURE.get<unsigned int>("source_end_idx", param_.end_idx);
  
  cout << "\n----------------------------------------\n"
  << "[source params] \n"
  << "source dir: " << param_.source_dir << endl
  << "depth_dir: " << param_.depth_dir << endl
  << "depth_marker: " << param_.depth_marker << endl
  << "depth_extension: " << param_.depth_extension << endl
  << "start_idx: " << param_.start_idx << endl
  << "end_idx: " << param_.end_idx << endl << endl;
  
  frame_idx_ = param_.start_idx;
  float leaf_size = 0.01;
  CONFIGURE.get<float>("source_downsample_leaf_size", leaf_size);
  voxel_grid_down_.setLeafSize(leaf_size, leaf_size, leaf_size);
}

//------------------------------------------------------------
int fileSource::generateNewFrame(frame &aframe)
{
  if(frame_idx_ > param_.end_idx)
  {
    cout << "\n----------------------------------------\n"
    "last frame reached."
    << endl;
    return EXIT_FAILURE;
  }
  
  char depthFile[100] = {0};
  sprintf(depthFile, "%s/%d%s",
          param_.depth_dir.c_str(), 
          frame_idx_, 
          param_.depth_extension.c_str());
  
  current_time_ += frame_time_interval_;
  frame_count_ += 1;
  cout << "\n----------------------------------------\n";
  cout << "current time: " << current_time_ << "s, "
  << "frame number #" << frame_count_ << endl
  << "reading files: " << depthFile << endl;
  
  cv::Mat depth = cv::imread(depthFile, -1);
  if(!depth.data)
  {
    cout << "Error during reading files " << depthFile
    << "." << endl;
    return EXIT_FAILURE;
  }
  
  PointCloudT_Ptr cloud = generatePointCloud(depth);
  aframe = frame(*cloud, current_time_, frame_idx_);
  
  ++frame_idx_;
  depth.release();
  cloud->points.clear();
  cloud->clear();
  
  return EXIT_SUCCESS;
}


PointCloudT_Ptr fileSource::generatePointCloud(const cv::Mat& depth)
{ 
  // 点云变量
  // 使用智能指针，创建一个空点云。这种指针用完会自动释放。
  PointCloudT::Ptr cloud ( new PointCloudT );
  // 遍历深度图
  for (int m = 0; m < depth.rows; m++)
    for (int n=0; n < depth.cols; n++)
    {
      // 获取深度图中(m,n)处的值
      ushort d = depth.ptr<ushort>(m)[n];
      // d 可能没有值，若如此，跳过此点
      if (d == 0)
        continue;
      // d 存在值，则向点云增加一个点
      PointT p;
      
      // 计算这个点的空间坐标
      p.z = double(d) / camera_.scale;
      p.x = (n - camera_.cx) * p.z / camera_.fx;
      p.y = (m - camera_.cy) * p.z / camera_.fy;
      
      // 把p加入到点云中
      cloud->points.push_back( p );
    }
    // 设置点云
    cloud->height = depth.rows;
    cloud->width = depth.cols;
    cloud->is_dense = false;
    
    cout << "original cloud size: " << cloud->points.size() << endl;
    voxel_grid_down_.setInputCloud(cloud);
    voxel_grid_down_.filter(*cloud);
    cout << "downsampled cloud size: " << cloud->points.size() << endl;
    
    return cloud;
}
