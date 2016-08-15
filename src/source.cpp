#include "source.h"

#include <iostream>
#include <stdio.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

rgbdSource::rgbdSource()
{
}

//------------------------------------------------------------
int fileSource::generateNewFrame(frame &aframe)
{
    if(frame_start_+frame_count_ > frame_end_)
    {
        std::cout << "\n----------------------------------------\n"
                     "last frame reached."
                  << std::endl;
        return EXIT_FAILURE;
    }

    char depthFile[100] = {0};
    sprintf(depthFile, "%s/%d%s",
            depth_dir_.c_str(), frame_start_+frame_count_, depth_extension_.c_str());

    current_time_ += frame_time_interval_;
    frame_count_ += 1;
    std::cout << "\n----------------------------------------\n";
    std::cout << "current time: " << current_time_ << "s, "
              << "frame number #" << frame_count_ << std::endl
              << "reading files: " << depthFile << std::endl;

    cv::Mat depth = cv::imread(depthFile, -1);
    if(!depth.data)
    {
        std::cout << "Error during reading files " << depthFile
                  << "." << std::endl;
        return EXIT_FAILURE;
    }

    PointCloudT_Ptr cloud = generatePointCloud(depth);
    aframe = frame(*cloud, current_time_);

    depth.release();
    cloud->points.clear();
    cloud->clear();

    return EXIT_SUCCESS;
}


PointCloudT_Ptr fileSource::generatePointCloud(const cv::Mat& depth)
{

    using std::cout;
    using std::endl;

    // 点云变量
    // 使用智能指针，创建一个空点云。这种指针用完会自动释放。
    PointCloudT::Ptr cloud ( new PointCloudT );
    // 遍历深度图
    for (int m = 0; m < depth.rows; m++)
        for (int n=0; n < depth.cols; n++)
        {
            // 获取深度图中(m,n)处的值
            ushort d = depth.ptr<ushort>(m)[n];
            // d 可能没有值，若如此，跳过此点
            if (d == 0)
                continue;
            // d 存在值，则向点云增加一个点
            PointT p;

            // 计算这个点的空间坐标
            p.z = double(d) / camera_.scale;
            p.x = (n - camera_.cx) * p.z / camera_.fx;
            p.y = (m - camera_.cy) * p.z / camera_.fy;

            // 把p加入到点云中
            cloud->points.push_back( p );
        }
    // 设置点云
    cloud->height = depth.rows;
    cloud->width = depth.cols;
    cloud->is_dense = false;

    std::cout << "original cloud size: " << cloud->points.size() << std::endl;
    voxel_grid_down_.setInputCloud(cloud);
    voxel_grid_down_.filter(*cloud);
    std::cout << "downsampled cloud size: " << cloud->points.size() << std::endl;

    return cloud;
}
