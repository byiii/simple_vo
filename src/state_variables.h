#ifndef STATE_VARIABLES_H
#define STATE_VARIABLES_H

#include <ostream>
#include "common_definitions.h"

//------------------------------------------------------------
// 六自由度状态，以位移向量和三轴的欧拉旋转角表示
// 可以表示位置和姿态，也可以表示运动的位移和旋转角
struct sixDoFState
{
    typedef Eigen::Matrix<float, 6, 1> SixDoFVec;

    float timestamp_;
    SixDoFVec vec_;
    //    // 欧拉旋转角，三个分量分别代表
    //    // [0] -- x-axis rotation angle
    //    // [1] -- y-axis rotation angle
    //    // [2] -- z-axis rotation angle
    //    Eigen::Matrix<float, 3, 1> rotat_vec_;
    //    // 质心位移向量
    //    // [0] -- x-axis translation
    //    // [1] -- y-axis translation
    //    // [2] -- z-axis translation
    //    Eigen::Matrix<float, 3, 1> trans_vec_;

    // 默认构造函数
    sixDoFState()
    {
        timestamp_ = 0;
        vec_ = SixDoFVec::Zero();
    }

    // 构造函数，从变换矩阵生成
    sixDoFState(Eigen::Matrix4f &mat)
    {
        fromMatrix4f(mat);
    }

    friend std::ostream& operator<<(std::ostream& out, const sixDoFState& state);
    // 将向量表示转换成4x4矩阵表示
    void fromMatrix4f(Eigen::Matrix4f &mat);
    // 将4x4矩阵转换成六自由度向量表示
    void toMatrix4f(Eigen::Matrix4f &mat);
    // 获取平移向量
    Eigen::Map<Eigen::Matrix<float, 3, 1> > t()
    {
        float *p = vec_.data();
        return Eigen::Map<Eigen::Matrix<float, 3, 1> >(p, 3);
    }
    // 获取旋转角向量
    Eigen::Map<Eigen::Matrix<float, 3, 1> > r()
    {
        float *p = vec_.data();
        return Eigen::Map<Eigen::Matrix<float, 3, 1> >(p+3, 3);
    }
};

//------------------------------------------------------------
// 表示运动状态的类
struct fullState
{
    typedef Eigen::Matrix<float, 12, 1> FullStateVec;
    // 时间戳
    float timestamp_;
    // 状态向量，12维：
    // [0,1,2]: xyz方向的位移
    // [3,4,5]: xyz方向的位移速度
    // [6,7,8]: 绕xyz轴旋转的欧拉旋转角
    // [9,10,11]: 绕xyz轴的欧拉旋转角变化速度
    FullStateVec vec_;

public:
    fullState()
    {
        for(size_t i=0; i<12; ++i)
            vec_(i) = 0.0f;
        timestamp_ = 0.0;
    }
    fullState(sixDoFState trans)
    {
        for(size_t i=0; i<12; ++i)
            vec_(i) = 0.0f;
        // translation
        vec_(0) = trans.vec_(0);
        vec_(1) = trans.vec_(1);
        vec_(2) = trans.vec_(2);
        // rotation
        vec_(6) = trans.vec_(3);
        vec_(7) = trans.vec_(4);
        vec_(8) = trans.vec_(5);

        timestamp_ = trans.timestamp_;
    }

    ~fullState()
    {
        vec_.resize(0);
    }

    fullState& operator=(const fullState& another)
    {
        this->timestamp_ = another.timestamp_;
        this->vec_ = another.vec_;

        return *this;
    }

    // 从旋转矩阵中得到状态向量
    void fromTransformationMatrix(Eigen::Matrix4f& mat)
    {
        sixDoFState trans = sixDoFState(mat);

        // translation
        vec_(0) = trans.vec_(0);
        vec_(1) = trans.vec_(1);
        vec_(2) = trans.vec_(2);
        // rotation
        vec_(6) = trans.vec_(3);
        vec_(7) = trans.vec_(4);
        vec_(8) = trans.vec_(5);
    }

    // 获取状态向量，以六自由度状态类（sixDoFState）形式保存
    sixDoFState getSixDoFState()
    {
        sixDoFState res;

        // Eigen::Matrix::block,
        // Block of size (p,q), starting at (i,j)
        // matrix.block(i,j,p,q);
        // matrix.block<p,q>(i,j);
        res.vec_.block<3,1>(0,0) = vec_.block<3,1>(0,0);
        res.vec_.block<3,1>(3,0) = vec_.block<3,1>(6,0);
        res.timestamp_ = timestamp_;

        return res;
    }
    // 获取速度向量，以六自由度状态（sixDoFState）类形式保存
    sixDoFState getSixDoFStateRatio()
    {
        sixDoFState res;
        res.vec_.block<3,1>(0,0) = vec_.block<3,1>(3,0);
        res.vec_.block<3,1>(3,0) = vec_.block<3,1>(9,0);
        res.timestamp_ = timestamp_;
        return res;
    }
};

#endif // STATE_VARIABLES_H
