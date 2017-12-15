#include <iostream>
#include <cmath>

// Eigen 部分
#include <Eigen/Core>
// 稠密矩阵的代数运算（逆，特征值等）
#include <Eigen/Dense>
//Eigen 几何模块
#include <Eigen/Geometry>

using namespace std;


int main(int argc, char **argv) {
    Eigen::Quaterniond q1(0.35, 0.2, 0.3, 0.1);
    Eigen::Quaterniond q2(-0.5, 0.4, -0.1, 0.2);
    Eigen::Vector3d t1(0.3, 0.1, 0.1);
    Eigen::Vector3d t2(-0.1, 0.5, 0.3);
    Eigen::Vector3d p1(0.5, 0, 0.2);
    
    Eigen::Quaterniond q1_one = q1.normalized();
    Eigen::Quaterniond q2_one = q2.normalized();
    
    
    //way1
    
    Eigen::Vector3d v = q1_one.inverse() * (p1 - t1);
    Eigen::Vector3d v2 = q2_one * v + t2;
    cout << "way1 v2 = " << endl << v2 << endl;

    
    //way2    
    Eigen::Matrix3d R1 = Eigen::Matrix3d(q1_one);
    Eigen::Matrix3d R2 = Eigen::Matrix3d(q2_one);
    Eigen::Vector3d v_2 = R1.inverse() * (p1 - t1);
    Eigen::Vector3d v_2_2 = R2 * v_2 + t2;
    cout << "way2 v2= " << endl << v_2_2 << endl;
     
    
    return 0;
}