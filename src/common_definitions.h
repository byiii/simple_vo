#ifndef COMMON_DEFINITIONS_H
#define COMMON_DEFINITIONS_H

// 输出，方便换行留空白
#define endll std::endl<<std::endl;

#include <opencv2/core.hpp>
#include <Eigen/Geometry>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>

#include <string>
#include <cfloat>
#include <cmath>

#include "parameter_config.h"

// 定义点云类型
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef PointCloudT::Ptr PointCloudT_Ptr;
typedef pcl::Normal NormalT;
typedef pcl::PointCloud<NormalT> NormalCloudT;
typedef NormalCloudT::Ptr NormalCloudT_Ptr;

//// g2o的头文件
//#include <g2o/core/sparse_optimizer.h>
//#include <g2o/core/block_solver.h>
//#include <g2o/core/factory.h>
//#include <g2o/core/optimization_algorithm_factory.h>
//#include <g2o/core/optimization_algorithm_gauss_newton.h>
//#include <g2o/solvers/csparse/linear_solver_csparse.h>

//// g2o 类型定义
//typedef g2o::BlockSolver_6_3 SlamBlockSolver;
//typedef g2o::LinearSolverCSparse< SlamBlockSolver::PoseMatrixType > SlamLinearSolver;

// 相机内参数，用的地方比较少
class cameraIntrinsicParameters
{
public:
    double cx;
    double cy;
    double fx;
    double fy;
    double scale;

    cameraIntrinsicParameters()
        : cx(325.5),
          cy(253.5),
          fx(518.0),
          fy(519.0),
          scale(1.0e3)
    {
    }

    // 重载运算符： ！=
    bool operator !=(const cameraIntrinsicParameters& another)
    {
        return !(operator ==(another));
    }

    // 运算符重载： ==，可能用不到
    bool operator ==(const cameraIntrinsicParameters& another)
    {
        bool result = false;
        result = (cx==another.cx) && (cy==another.cy) && (fx==another.fx)
                && (fy==another.fy) && (scale==another.scale);
        return result;
    }
};


// 用文件设置算法参数
// const char* CONFIG_FILE = "../config/parameters.cfg";
static parameterConfig CONFIGURE("../config/parameters.cfg");

inline void printVector(Eigen::VectorXf vec, bool row_p = true)
{
    if(!row_p)
        std::cout << vec << std::endl;
    else
    {
        for(size_t i=0; i<vec.rows()-1; ++i)
            std::cout << vec(i) << ", ";
        std::cout << vec(vec.rows());
    }
}

#endif // COMMON_DEFINITIONS_H
