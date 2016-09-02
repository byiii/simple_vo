#include "common_definitions.h"

bool cameraIntrinsicParameters::operator ==(const cameraIntrinsicParameters& another)
{
  bool result = false;
  result = (cx==another.cx) && (cy==another.cy) && (fx==another.fx)
  && (fy==another.fy) && (scale==another.scale);
  return result;
}

void printVector(Eigen::VectorXf vec, bool row_p)
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