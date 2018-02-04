#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include <fstream>
#include <iostream>
#include <sophus/se3.h>

using namespace Eigen;
using namespace std;

typedef vector<Vector3d,Eigen::aligned_allocator<Vector3d> > VecVector3d;
typedef vector<Vector2d,Eigen::aligned_allocator<Vector2d> > VecVector2d;

typedef Matrix<double ,6,1> Vector6d;
string p3d_file="../p3d.txt";
string p2d_file="../p2d.txt";

int main(int argc ,char** argv)
{
    VecVector2d p2d;
    VecVector3d p3d;
    Matrix3d k; //内参
    double fx=520.9,fy=521.0,cx=325.1,cy=249.7;
    k<<fx,0,cx,0,fy,cy,0,0,1;

    //load points into p3d and p2d
    //START YOUR CODE HERE
    ifstream file1(p3d_file);
    if(!file1.is_open())
    {
        perror("p3d open failed");
    }
    ifstream file2(p2d_file);
    if(!file2.is_open())
    {
        perror("p2d open failed");
    }
    while (!file1.eof())
    {
        double p3[3] = {0};
        for (auto &p:p3)
        {
            file1 >> p;
        }
        p3d.push_back(Vector3d(p3[0], p3[1], p3[2]));
    }
    while (!file2.eof())
    {
        double p2[2]={0};
        for (auto& p:p2)
        {
            file2>>p;
        }
        Vector2d v(p2[0],p2[1]);
        p2d.push_back(v);
        //cout<<"p2d "<<v<<" p3d "<<Vector3d(p3[0],p3[1],p3[2])<<endl;
    }
    //END YOUR CODE HERE
    assert( p3d.size() == p2d.size() ); //如果二者行数相等，则而已继续执行，否则程序停止运行

    int iterations=100;
    double cost=0,lastcost=0;
    int nPoints=p3d.size();
    cout<<"points: "<<nPoints<<endl;


    Matrix3d I = Matrix3d::Identity(); //声明单位矩阵
    Vector3d t ;
    t.setZero();
    //cout<< "I:\n"<<I<<endl;
    //cout<< "t:\n"<<t<<endl;
    Sophus::SE3 T_esti(I,t);//变换矩阵
    //Sophus::SE3 T_esti;
    cout<<"T_esti:\n"<<T_esti.matrix()<<endl;



   for (int iter = 0; iter <iterations ; ++iter) //迭代100次
    {
        Matrix<double,6,6> H = Matrix<double,6,6>::Zero();
        Vector6d b = Vector6d::Zero();

        cost=0;
        //compute cost
        for (int i = 0; i <p3d.size() ; ++i) //遍历数组
        {
            //compute cost for p3d[i] and p2d[i]
            //START YOUR CODE HERE
            Vector2d ui=p2d[i]; //2*1 p2p点
            //Vector3d pi=p3d[i];
            //（7.34）4*4 的变换矩阵 × 4*1的3d点

            Vector4d pii = T_esti.matrix()*Vector4d(p3d[i][0],p3d[i][1],p3d[i][2],1);

           // cout<< "pii:\n"<< pii <<endl;
           // cout<< "k:\n" << k << endl;
            // 3*1的s*u =3*3 的内参 × 3*1的(SE3*P)
            Vector3d pi=k*Vector3d(pii[0],pii[1],pii[2]);//s*u

            cout<< "pi:\n"<< pi<<endl;

            //求误差
            Vector2d e( ui[0]-pi[0]/pi[2] , ui[1]-pi[1]/pi[2] ); //除以pi[2]是归一化过程，误差为2*1
            cost += e(0,0)*e(0,0)+e(1,0)*e(1,0);//e.transpose()*e;
            //END YOUR CODE  HERE

            //compute jacobian
            Matrix<double,2,6> J; //定义2*6 雅克比

            //START YOUR CODE HERE
            //7.45
            J(0,0)=-fx/pii[2];
            J(0,1)=0;
            J(0,2)=fx*pii[0]/(pii[2]*pii[2]);
            J(0,3)=fx*pii[0]*pii[1]/(pii[2]*pii[2]);
            J(0,4)=-fx-fx*pii[0]*pii[0]/(pii[2]*pii[2]);
            J(0,5)=fx*pii[1]/pii[2];

            J(1,0)=0;
            J(1,1)=-fy/pii[2];
            J(1,2)=fy*pii[1]/(pii[2]*pii[2]);
            J(1,3)=fy+fy*pii[2]*pii[2]/(pii[0]*pii[0]);
            J(1,4)=-fy*pii[0]*pii[1]/(pii[2]*pii[2]);
            J(1,5)=-fy*pii[0]/pii[2];
            //END YOUR CODE HERE

            H+=J.transpose()*J;
            b+=-J.transpose()*e;
        }

        //solve dx
        Vector6d dx;

        //START YOUR CODE HERE
        dx=H.ldlt().solve(b);
        //END YOUR CODE HERE

        cout<<"iteration "<<iter<<" cost="<<cout.precision(12)<<cost<<endl;
        if(isnan(dx[0]))
        {
            cout<<"result is nan!"<<endl;
            break;
        }

        if(iter>0&&cost>=lastcost)
        {
            //cost increase,update is not good
            cout<<"cost: "<<cout.precision(12)<<cost<<", last cost: "<<cout.precision(12)<<lastcost<<endl;
            break;
        }

        //update your estimation
        //START YOUR CODE HERE
        T_esti=Sophus::SE3::exp(dx) * T_esti;

        //END YOUR CODE HERE
        lastcost=cost;
        cout<<"iteration "<<iter<<" cost="<<cout.precision(12)<<cost<<endl;
    }

    cout<<"estimated pose: \n"<<T_esti.matrix()<<endl;

    return 0;
}
