#include "main_func.h"

#include <iostream>

#include "measurement_base.h"
#include <vector>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/gicp.h>

int main_test_source(int argc, char ** argv)
{
    using std::cout;
    using std::endl;

    cout << "Hey, dude.\n" << endll;

    cameraIntrinsicParameters camera;

    fileSource source("../data");
    source.depth_dir_ = "../data/depth_png";
    source.depth_marker_ = "";
    source.depth_extension_ = ".png";
    source.setCamera(camera);
    source.setStartIndex(150);
    source.setEndIndex(250);

    pcl::visualization::CloudViewer viewer("viewer");

    frame currFrame;
    while (source.generateNewFrame(currFrame)==0)
    {
        viewer.showCloud(currFrame.getPointCloud());
    }

    return 0;
}
