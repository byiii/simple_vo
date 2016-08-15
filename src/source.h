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
    // 文件地址，文件名，文件扩展名
    std::string depth_dir_;
    std::string depth_marker_;
    std::string depth_extension_;
    std::string source_dir_;
    //------------------------------------------------------------
    // protected member variables
protected:
    float frame_rate_;
    float frame_time_interval_;
    float current_time_;
    unsigned frame_count_;
    unsigned frame_start_;
    unsigned frame_end_;

    cameraIntrinsicParameters camera_;
    pcl::VoxelGrid<PointT> voxel_grid_down_;

    //------------------------------------------------------------
    // public member functions
public:

    // constructor
    fileSource(const char* str, float fr=1.0, float ct=0.0)
    {
        source_dir_ = std::string(str);
        std::cout << "file source dir: " << source_dir_ << std::endl;

        if(fr==0.0)
            frame_rate_ = 1.0;
        else
            frame_rate_ = fr;
        current_time_ = ct;
        frame_time_interval_ = 1/frame_rate_;
        frame_count_ = 0;
        frame_start_ = 0;
        frame_end_ = 0;
    }
    ~fileSource()
    {

    }

    float getCurrentTime()
    {
        return current_time_;
    }

    unsigned getCurrentFrameIndex()
    {
        return (frame_start_+frame_count_);
    }

    void setSourceDir(const char* str)
    {
        source_dir_ = str;
    }

    void setCamera(cameraIntrinsicParameters &c)
    {
        this->camera_ = c;
    }

    void setFrameRate(float f)
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

    void setCurrentTime(float t)
    {
        current_time_ = t;
    }

    void setStartIndex(unsigned n)
    {
        frame_start_ = n;
    }

    void setEndIndex(unsigned n)
    {
        frame_end_ = n;
    }

    int generateNewFrame(frame &aframe);

    void setDownSampleLeafSize(float leaf_size=0.005)
    {
        voxel_grid_down_.setLeafSize(leaf_size,
                                     leaf_size,
                                     leaf_size);
    }

    //------------------------------------------------------------
    // protected member functions
protected:
    PointCloudT_Ptr generatePointCloud(const cv::Mat& depth);
};

class openniDeviceSource
{
};

#endif // RGBDSOURCE_H
