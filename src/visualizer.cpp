#include "visualizer.h"

#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread.hpp>

// 显示点云
void visualizerSimple::showPointCloud(PointCloudT_Ptr pc)
{
    pcl::visualization::PCLVisualizer viewer(name_.c_str());
    viewer.setBackgroundColor(0.3, 0.4, 0.4);
    viewer.addPointCloud(pc);

    while(!viewer.wasStopped())
    {
        viewer.spinOnce(100);
    }

    viewer.close();
}
