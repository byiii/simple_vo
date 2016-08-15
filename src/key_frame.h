#ifndef KEY_FRAME_H
#define KEY_FRAME_H

#include "common_definitions.h"
#include "frame.h"
#include "state_variables.h"

//  关键帧 类
class keyFrame : public frame
{
protected:
    fullState state_;
public:
    keyFrame();

    keyFrame(frame& aFrame)
    {
        this->generate_time_ = aFrame.getTimeStamp();
        this->point_cloud_ = aFrame.getPointCloud();
    }

    ~keyFrame()
    {
        this->point_cloud_->points.clear();
        this->point_cloud_->clear();
    }

    keyFrame& operator=(const keyFrame &another_frame)
    {
        this->state_ = another_frame.state_;
        this->generate_time_ = another_frame.generate_time_;

        point_cloud_.reset(new PointCloudT);
        this->point_cloud_ = another_frame.point_cloud_->makeShared();

        return *this;
    }

    // 融合新的输入帧点云数据，优化当前关键帧
    void refine(PointCloudT_Ptr &transformed_cloud)
    {
        *(this->point_cloud_) = *(this->point_cloud_) + *transformed_cloud;
    }
    void setEstimatedState(fullState& state)
    {
        state_ = state;
    }
};

#endif // KEY_FRAME_H
