//
// Created by 高翔 on 2017/12/19.
// 本程序演示如何从Essential矩阵计算R,t
//

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>



//#include <sophus/so3.hpp>

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "sophus/so3.h"
#include "sophus/se3.h"
using namespace Eigen;
using namespace std;




int main(int argc, char **argv)
{

    // 给定Essential矩阵 3*3
    Matrix3d E;
    E << -0.0203618550523477, -0.4007110038118445, -0.03324074249824097,
            0.3939270778216369, -0.03506401846698079, 0.5857110303721015,
            -0.006788487241438284, -0.5815434272915686, -0.01438258684486258;

    // 待计算的R,t
    Matrix3d R;
    Vector3d t;
    // SVD and fix sigular values
    // START YOUR CODE HERE

    std::cout<<"E :\n"<<E<<std::endl;
    //A = U *E*VT
    JacobiSVD<Eigen::Matrix3d>svd(E, Eigen::ComputeFullU | Eigen::ComputeFullV );

    Matrix3d U = svd.matrixU();
    Matrix3d V = svd.matrixV();
    Matrix3d S = U.inverse() * E * V.transpose().inverse(); // S = U^-1 * E * VT * -1
    // std::cout<<"U :\n"<<U<<std::endl;
    // std::cout<<"EE :\n"<<E<<std::endl;
    // std::cout<<"V :\n"<<V<<std::endl;
    //std::cout<<"U * EE * V2T :\n"<<U * S * V.transpose()<<std::endl;
    // END YOUR CODE HERE

    // set t1, t2, R1, R2 
    // START YOUR CODE HERE
    Matrix3d t_wedge1;
    Matrix3d t_wedge2;
    Matrix3d R1;
    Matrix3d R2;
    //定义旋转矩阵
    Eigen::AngleAxisd rotation_vector1 ( M_PI/2, Eigen::Vector3d ( 0,0,1 ) );
    Eigen::Matrix3d RZ1(rotation_vector1);

    Eigen::AngleAxisd rotation_vector2 ( -M_PI/2, Eigen::Vector3d ( 0,0,1 ) );
    Eigen::Matrix3d RZ2(rotation_vector1);

    t_wedge1 = U*S*RZ1*U.transpose();
    t_wedge2 = U*S*RZ2*U.transpose();
    R1 = U*S*RZ1.transpose()*V.transpose();
    R2 = U*S*RZ2.transpose()*V.transpose();
    // END YOUR CODE HERE
    cout << "R1 = " << R1 << endl;
    cout << "R2 = " << R2 << endl;
    //vee  反对称矩阵到向量
    cout << "t1 = " << Sophus::SO3::vee(t_wedge1) << endl;
    cout << "t2 = " << Sophus::SO3::vee(t_wedge2) << endl;

    // check t^R=E up to scale
    Matrix3d tR = t_wedge1 * R1;
    cout << "t^R = " << tR << endl;

    return 0;
}