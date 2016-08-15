#ifndef VISUALIZER_H
#define VISUALIZER_H

#include <string>
#include "common_definitions.h"

// 简单的可视化类，不需要太复杂
class visualizerSimple
{
protected:
    std::string name_;
public:
    visualizerSimple(const char* str)
    {
        name_ = str;
    }

    void showPointCloud(PointCloudT_Ptr pc);
};

#endif // VISUALIZER_H
