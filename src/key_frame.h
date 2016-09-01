#ifndef KEY_FRAME_H
#define KEY_FRAME_H

#include "common_definitions.h"
#include "frame.h"
#include "state_variables.h"

//  关键帧 类
class keyFrame : public frame
{
protected:
  Eigen::Matrix4f toLKF_;
  Eigen::Matrix4f toRKF_;
  keyFrame *ptrLKF_;
  
public:
  keyFrame();
  keyFrame(frame& aFrame);
  ~keyFrame();
  
  inline void setTransToLKF(Eigen::Matrix4f &mat) { toLKF_ = mat; }
  inline void setTransToLKF(SVector6f &vec) { vec.toMatrix4f(toLKF_); }
  inline void setTransToRKF(Eigen::Matrix4f &mat) { toRKF_ = mat; }
  inline void setTransToRKF(SVector6f &vec) { vec.toMatrix4f(toRKF_); }
  inline Eigen::Matrix4f getTransToLKF() { return toLKF_; }
  inline Eigen::Matrix4f getTransToRKF() { return toRKF_; }
  
  inline void setLKFPtr(keyFrame *p) { ptrLKF_ = p; }
  inline keyFrame* getLKFPtr() { return ptrLKF_; }
  
  keyFrame& operator=(const keyFrame &another);
  // 融合新的输入帧点云数据，优化当前关键帧
  void refine(PointCloudT_Ptr &transformed_cloud);
};

#endif // KEY_FRAME_H
