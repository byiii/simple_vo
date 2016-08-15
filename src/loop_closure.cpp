#include "loop_closure.h"
#include "state_variables.h"

#include <cmath>

// 构造函数
loopClosure::loopClosure()
{
    keyframe_threshold = 0.05;
    too_far_away_threshold = 0.10;
}

// 返回两个数中较小的值，min用起来有问题
inline float smallerOne(float a, float b)
{
    if(a<b)
        return a;
    else
        return b;
}


// 检测一个输入帧是否是关键帧
loopClosure::KeyFrameCheckResult loopClosure::detectNewKeyFrame(keyFrame& currentKF,
                                                   frame& aFrame,
                                                   Eigen::Matrix4f& transformation)
{
    float score = evaluateTransformation(transformation);
    // 如果score太小，认为太接近
    if(score<keyframe_threshold)
        return TOO_CLOSE;
    // 距离合适，判定为关键帧
    else if(score<too_far_away_threshold)
        return KEYFRAME;
    // 否则太远了，可能不准确
    else
        return TOO_FAR_AWAY;
}

// 量化一个六自由度变换，作为是否信任新输入帧信息的判定
// 这里简单采用衡量变换大概位移的长度。
float loopClosure::evaluateTransformation(Eigen::Matrix4f& transformation)
{
    sixDoFState trans(transformation);
    Eigen::Vector3f translation_tmp = trans.vec_.block<3,1>(0,0);
    Eigen::Vector3f rotation_tmp = trans.vec_.block<3,1>(3,0);
    return fabs(smallerOne(translation_tmp.norm(),
                    2*M_PI-rotation_tmp.norm()))
            + fabs(translation_tmp.norm());
}
