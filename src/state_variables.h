#ifndef STATE_VARIABLES_H
#define STATE_VARIABLES_H

#include <ostream>
#include "common_definitions.h"

//------------------------------------------------------------
// 六自由度状态，以位移向量和三轴的欧拉旋转角表示
// 可以表示位置和姿态，也可以表示运动的位移和旋转角
struct SVector6f
{
  typedef Eigen::Matrix<float, 6, 1> vector6f;
  typedef Eigen::Vector3f vector3f;
  //    // 欧拉旋转角，三个分量分别代表
  //    // [0] -- x-axis rotation angle
  //    // [1] -- y-axis rotation angle
  //    // [2] -- z-axis rotation angle
  //    // 质心位移向量
  //    // [3] -- x-axis translation
  //    // [4] -- y-axis translation
  //    // [5] -- z-axis translation
  vector6f vec_;
  
  // 默认构造函数
  SVector6f() {
    vec_ = vector6f::Zero();
  }
  
  // 构造函数，从变换矩阵生成
  SVector6f(Eigen::Matrix4f &mat) {
    vec_ = vector6f::Zero();
    fromMatrix4f(mat);
  }
  
  inline SVector6f& operator=(const SVector6f& another) {
    this->vec_ = another.vec_;
    return *this;
  }
  
  friend std::ostream& operator<<(std::ostream& out, const SVector6f& state);
  // 将向量表示转换成4x4矩阵表示
  void fromMatrix4f(Eigen::Matrix4f &mat);
  // 将4x4矩阵转换成六自由度向量表示
  void toMatrix4f(Eigen::Matrix4f &mat);
  // 获取平移向量
  vector3f t();
  // 获取旋转角向量
  vector3f r();
};

//------------------------------------------------------------
// 表示运动状态的类
struct SVector12f
{
  typedef Eigen::Matrix<float, 12, 1> Vector12f;
  // 时间戳
  // 状态向量，12维：
  // [0,1,2]: 欧拉角
  // [3,4,5]: 欧拉角，角速度
  // [6,7,8]: 位置
  // [9,10,11]: 平移速度 
  Vector12f vec_;
  
public:
  SVector12f() {
    vec_ = Vector12f::Zero();
  }
  SVector12f(SVector6f trans);
  
  ~SVector12f() {
    vec_.resize(0);
  }
  
  inline SVector12f& operator=(const SVector12f& another) {
    this->vec_ = another.vec_;
    return *this;
  }
  
  // 从旋转矩阵中得到状态向量
  void fromMatrix4f(Eigen::Matrix4f& mat);
  
  // 获取状态向量，以六自由度状态类（sixDoFState）形式保存
  SVector6f getState();
  // 获取速度向量，以六自由度状态（sixDoFState）类形式保存
  SVector6f getRatio();
};

#endif // STATE_VARIABLES_H
