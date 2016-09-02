#include "loop_closure.h"
#include "state_variables.h"

using std::cout;
using std::endl;

#include <cmath>

// 返回两个数中较小的值，min用起来有问题
inline float mMIN(float a, float b)
{
    if(a<b)
        return a;
    else
        return b;
}

// 构造函数
loopClosure::loopClosure()
{
    keyframe_threshold = 0.08;
    too_far_away_threshold = 0.13;
}

    void loopClosure::initialize() {
        CONFIGURE.get<float>("lc_keyframe_threshold", keyframe_threshold);
        CONFIGURE.get<float>("lc_too_far_away_threshold", too_far_away_threshold);
    }
    
// 检测一个输入帧是否是关键帧
loopClosure::KeyFrameCheckResult loopClosure::detectNewKeyFrame(keyFrame& currentKF,
                                                   frame& aFrame,
                                                   Eigen::Matrix4f& transformation)
{
    float score = evaluateTransformation(transformation);
    // 如果score太小，认为太接近
    if(score<keyframe_threshold)
        return CLOSE;
    // 距离合适，判定为关键帧
    else if(score<too_far_away_threshold)
        return KEYFRAME;
    // 否则太远了，可能不准确
    else
        return FAR_AWAY;
}

// 量化一个六自由度变换，作为是否信任新输入帧信息的判定
// 这里简单采用衡量变换大概位移的长度。
float loopClosure::evaluateTransformation(Eigen::Matrix4f& transformation)
{
    SVector6f trans(transformation);
    Eigen::Vector3f translation_tmp = trans.t();
    Eigen::Vector3f rotation_tmp = trans.r();
    
    return fabs(mMIN(translation_tmp.norm(),
                    2*M_PI-rotation_tmp.norm()))
            + fabs(translation_tmp.norm());
}
