#include "key_frame.h"

keyFrame::keyFrame()
{
}

keyFrame::keyFrame(frame& aFrame)
{
  this->generate_time_ = aFrame.getTimeStamp();
  this->point_cloud_ = aFrame.getPointCloud();
  ptrLKF_ = NULL;
}

keyFrame::~keyFrame()
{
  this->point_cloud_->points.clear();
  this->point_cloud_->clear();
  ptrLKF_ = NULL;
}

keyFrame& keyFrame::operator=(const keyFrame &another)
{
  this->generate_time_ = another.generate_time_;
  this->point_cloud_.reset(new PointCloudT);
  this->point_cloud_ = another.point_cloud_->makeShared();
  this->toLKF_ = another.toLKF_;
  this->toRKF_ = another.toRKF_;
  this->ptrLKF_ = another.ptrLKF_;
  this->id_ = another.id_;
  return *this;
}

void keyFrame::refine(PointCloudT_Ptr &transformed_cloud)
{
  *(this->point_cloud_) = *(this->point_cloud_) + *transformed_cloud;
  this->downsampleCloud();
}