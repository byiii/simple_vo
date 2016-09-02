#ifndef FRAME_H
#define FRAME_H

#include <opencv2/core/core.hpp>
#include "common_definitions.h"

////////////////////////////////////////////////////////////
/// \brief The frame class
///
class frame
{
  //------------------------------------------------------------
  // member variables
protected:
  // 生成这一帧的时间，time stamp
  int id_;
  float generate_time_;
  // 这一帧对应的传感器点云数据
  PointCloudT_Ptr point_cloud_;
  PointCloudT_Ptr keypoints_;
  PointCloudT_Ptr downsampled_;
  NormalCloudT_Ptr normal_Cloud_;
  
  //------------------------------------------------------------
  // member functions
public:
  // default constructor
  frame();
  // 构造函数，从点云数据和时间标记生成
  frame(const PointCloudT cloud, float frame_time_stamp, int id);
  // destructor
  virtual ~frame() {
    this->release();
  }
  
  // 运算符重载： =， 复制构造
  virtual frame& operator=(const frame &another_frame);
  
  // release resource
  virtual void release() {
    point_cloud_->points.clear();
    point_cloud_->clear();
  }
  
  // 向外提供点云数据，直接给指针，外部可改变这帧的数据内容
  inline PointCloudT_Ptr getPointCloud() const {
    return this->point_cloud_;
  }
  
  // 向外提供点云数据，提供复制的版本
  inline PointCloudT_Ptr getPointCloudCopy() const {
    return this->point_cloud_->makeShared();
  }
  
  // 返回时间标记，时间戳
  inline float getTimeStamp() const {
    return this->generate_time_;
  }
  inline int id() const {
    return id_;
  }
  
  // 点云下采样
  void downsampleCloud();
  // 采集关键点
  void getUniformKeyPoints(float search_radius);
  // 计算点云中的法向量
  void estimateNormalAtKeyPoints();
  void estimateNormalAtKeyPoints(float search_radius);
};

#endif // FRAME_H
