//
// Created by liyubo on 12/15/17.
//
#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
using namespace std;
void hw() 
{
	//q1 = [0:35; 0:2; 0:3; 0:1] 第一项为实部
    Eigen::Quaterniond q1(0.35, 0.2, 0.3, 0.1);
	Eigen::Vector3d t1(0.3, 0.1, 0.1);
	cout<< "q1.coeffs \n"<<q1.coeffs()<<endl; //利用coffs输出，实部在最后一位
    Eigen::Quaterniond q2(-0.5, 0.4, -0.1, 0.2);
    Eigen::Vector3d t2(-0.1, 0.5, 0.3);
	
    Eigen::Vector3d p1(0.5, 0, 0.2);
    
	//归一化
    q1 = q1.normalized();
	cout<<"归一化 q1.coeffs() :\n"<<q1.coeffs()<<endl;
	
	cout<<"q1.inverse :\n"<<q1.inverse().coeffs()<<endl;
    q2 = q2.normalized();
    
    //方法一
    // v *q1 + t1 = p1
    Eigen::Vector3d w = q1.inverse() * (p1 - t1);
	// v *q2 + t2 = p2
    Eigen::Vector3d p2 = q2 * w + t2;
    cout << "v2 = \n" << p2 << endl;

    
    //way2    
    Eigen::Matrix3d R1 = Eigen::Matrix3d(q1); //转换为3*3旋转矩阵
    Eigen::Matrix3d R2 = Eigen::Matrix3d(q2);
    Eigen::Vector3d v_2 = R1.inverse()*(p1 - t1);
    Eigen::Vector3d v_2_2 = R2 * v_2 + t2;
    cout << "way2 v2= " << endl << v_2_2 << endl;
     
    
    return ;
}
