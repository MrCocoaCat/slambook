###2 验证向量叉乘的李代数性质


###3 推导 SE(3) 的指数映射



###4 伴随 (2 分,约 1 小时)


###5 轨迹的描绘

``` c++
#include <sophus/se3.h>
#include <string>
#include <iostream>
#include <fstream>
#include <pangolin/pangolin.h>
string trajectory_file = "../trajectory.txt";
void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>>);

int main(int argc, char **argv) 
{

    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses;

    /// implement pose reading code
    // start your code here (5~10 lines)
    ifstream fin(trajectory_file);
    double t,tx,ty,tz,qx,qy,qz,qw;
    while(fin>>t>>tx>>ty>>tz>>qx>>qy>>qz>>qw)
    {

        Eigen::Vector3d tt(tx,ty,tz);
        Eigen::Quaterniond q(qw,qx,qy,qz);//四元数
        Eigen::Matrix3d R(q); //旋转矩阵
        Sophus::SE3 point(R,tt);
        poses.push_back(point);
    }

    // end your code here
    // draw trajectory in pangolin
    DrawTrajectory(poses);
    return 0;
}

/*******************************************************************************************/
void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses)
{
    if (poses.empty())
    {
        cerr << "Trajectory is empty!" << endl;
        return;
    }

    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));


    while (pangolin::ShouldQuit() == false)
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glLineWidth(2);
        for (size_t i = 0; i < poses.size() - 1; i++)
        {
            glColor3f(1 - (float) i / poses.size(), 0.0f, (float) i / poses.size());
            glBegin(GL_LINES);
            auto p1 = poses[i], p2 = poses[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }

}

```


###6 *轨迹的误差

```
int main(int argc, char **argv)
{

    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses;

    /// implement pose reading code
    // start your codsume here (5~10 lines)
    ifstream fin1(ground_truth);
    if (!fin1.is_open())
    {
        perror("Error opening file");
        exit (1);
    }

    ifstream fin2(estimate);
    if (!fin2.is_open())
    {

        perror("Error opening file");
        exit (1);
    }
    double arr1[8];
    double arr2[8];
    double t1,tx1,ty1,tz1,qx1,qy1,qz1,qw1;
    double t2,tx2,ty2,tz2,qx2,qy2,qz2,qw2;
    static double sum = 0,e = 0;
    static int i = 0;
    while(fin1>>t1>>tx1>>ty1>>tz1>>qx1>>qy1>>qz1>>qw1)
    {

        fin2>>t2>>tx2>>ty2>>tz2>>qx2>>qy2>>qz2>>qw2;
        Eigen::Quaterniond q1(qw1,qx1,qy1,qz1);//四元数
        Eigen::Matrix3d R1(q1); //旋转矩阵
        Eigen::Vector3d t1(tx1,ty1,tz1);
        Sophus::SE3 gi(q1,t1);

        Eigen::Quaterniond q2(qw2,qx2,qy2,qz2);//四元数
        Eigen::Matrix3d R2(q2); //旋转矩阵
        Eigen::Vector3d t2(tx2,ty2,tz2);
        Sophus::SE3 ei(q2,t2);

        //Eigen::Matrix<double,6,1>se = ( gi.inverse() * ei).log();
        cout<< "gi :"<<endl;
        cout<<gi <<endl;
        auto se = ( gi.inverse() * ei).log();
        e = se.norm(); //二范数
        sum = sum + e*e;
        i++;
    }
    double RMSE = sqrt(sum/i);
    cout<<RMSE<<endl;
    // end your code here
    // draw trajectory in pangolin
    //DrawTrajectory(poses);
    return 0;
}
```