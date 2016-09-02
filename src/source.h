#ifndef RGBDSOURCE_H
#define RGBDSOURCE_H

#include "common_definitions.h"
#include "frame.h"

#include <iostream>
#include <string>

#include <pcl/filters/voxel_grid.h>

class rgbdSource
{
public:
  rgbdSource();
};

// 从文件中获取输入数据的类
class fileSource
{
  //------------------------------------------------------------
  // public member variables
public:
  struct params {
    // 文件地址，文件名，文件扩展名
    std::string depth_dir;
    std::string depth_marker;
    std::string depth_extension;
    std::string source_dir;
    unsigned start_idx;
    unsigned end_idx;
    params() {
      source_dir = "../data";
      depth_dir = "../data/depth_png";
      depth_marker = "";
      depth_extension = ".png";
      start_idx = 0;
      end_idx = 0;
    }
  };
  
  //------------------------------------------------------------
  // protected member variables
protected:
  float frame_rate_;
  float frame_time_interval_;
  float current_time_;
  unsigned frame_count_;
  unsigned frame_idx_;
  
  params param_;
  cameraIntrinsicParameters camera_;
  pcl::VoxelGrid<PointT> voxel_grid_down_;
  
  //------------------------------------------------------------
  // public member functions
public:
  
  // constructor
  fileSource(float fr=1.0, float ct=0.0);
  ~fileSource() { }
  
  inline float getCurrentTime() {
    return current_time_;
  }
  inline void setSourceDir(const char* str) {
    param_.source_dir = str;
  }
  inline void setCamera(cameraIntrinsicParameters &c) {
    this->camera_ = c;
  }
  inline void setCurrentTime(float t) {
    current_time_ = t;
  }
  
  void initialize();
  void setFrameRate(float f);
  int generateNewFrame(frame &aframe);
  
  //------------------------------------------------------------
  // protected member functions
protected:
  PointCloudT_Ptr generatePointCloud(const cv::Mat& depth);
};

class openniDeviceSource
{
};

#endif // RGBDSOURCE_H
