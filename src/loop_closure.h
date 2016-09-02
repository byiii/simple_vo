#ifndef LOOP_CLOSURE_H
#define LOOP_CLOSURE_H

#include "common_definitions.h"
#include "frame.h"
#include "key_frame.h"

class loopClosure
{
public:
    // 回环检测结果
    enum KeyFrameCheckResult{CLOSE=0, KEYFRAME, FAR_AWAY};

    // 构造函数
    loopClosure();
    // 析构函数
    ~loopClosure()
    {}

    // 检测一个输入帧是否是关键帧
    KeyFrameCheckResult detectNewKeyFrame(keyFrame& currentKF,
                                          frame& aFrame,
                                          Eigen::Matrix4f& transformation);

    //
    void initialize();
    // 检测临近回环
    void detectNearbyKeyFrames();
    // 随机抽查检测回环
    void detectRandomKeyFrames();

protected:
    float keyframe_threshold;
    float too_far_away_threshold;

    // 量化一个六自由度变换，作为是否信任新输入帧信息的判定
    float evaluateTransformation(Eigen::Matrix4f& transformation);
};

#endif // LOOP_CLOSURE_H
