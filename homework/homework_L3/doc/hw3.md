## 第三次作业
### 2验证向量叉乘的李代数性质
### 3推导SE(3)的指数映射
$e^{\xi^\wedge}=I+\begin{bmatrix}
   \phi^\wedge & \rho \\\\
   0 & 0
  \end{bmatrix}+\frac{1}{2!}\begin{bmatrix}
   \phi^\wedge & \rho \\\\
   0 & 0
  \end{bmatrix}+\frac{1}{3!}\begin{bmatrix}
   (\hat{\phi}\ \ )^2 & \hat{\phi}\ \ \rho \\\\
   0 & 0
  \end{bmatrix}+\dots=\begin{bmatrix}
   \sum_{n=0}^{\infty}\frac{1}{n!}(\hat{\phi}\ \ )^n & \sum_{n=0}^{\infty}\frac{1}{(n+1)!}(\hat{\phi}\ \ )^n\rho \\\\
   0^T & 1
  \end{bmatrix}$

  已知$$R=e^{\hat{\phi}}=\sum_{n=0}^{\infty}\frac{1}{n!}(\hat{\phi}\ \ )^n=cos\theta\cdot I+(1-cos\theta)aa^T+sin\theta\cdot\hat{a}\tag{1}$$

  令$J=\sum_{n=0}^{\infty}\frac{1}{(n+1)!}(\hat{\phi}\ \ )^n$,
$J=I+\frac{1}{2!}\hat{\phi}\ +\frac{1}{3!}(\hat{\phi}\ )^2+\frac{1}{4!}(\hat{\phi}\ )^3+\frac{1}{5!}(\hat{\phi}\ )^4 + \dots$

$=aa^T-\hat{a}\ \hat{a}+\frac{1}{2!}\theta\hat{a}\ +\frac{1}{3!}\theta^2\hat{a}\ \hat{a}-\frac{1}{4!}\theta^3\hat{a}\ -\frac{1}{5!}\theta^4\hat{a}\ \hat{a} + \dots$

$=aa^T-\hat{a}\ \hat{a}\ (1-\frac{1}{3!}\theta^2+\frac{1}{5!}\theta^4+\dots)+\hat{a}\ (\frac{1}{2!}\theta-\frac{1}{4!}\theta^3+\dots)$

$=aa^T-\frac{1}{\theta}\hat{a}\ \hat{a}\ (\theta-\frac{1}{3!}\theta^3+\frac{1}{5!}\theta^5+\dots)+\frac{1}{\theta}\hat{a}\ (\frac{1}{2!}\theta^2-\frac{1}{4!}\theta^4+\dots)$

$=aa^T-\frac{sin\theta}{\theta}\hat{a}\ \hat{a}\ +\frac{1-cos\theta}{\theta}\hat{a}\ $

$=aa^T-\frac{sin\theta}{\theta}(aa^T-I)+\frac{1-cos\theta}{\theta}\hat{a}\ $

$=\frac{sin\theta}{\theta}I\ +(1-\frac{sin\theta}{\theta})aa^T+\frac{1-cos\theta}{\theta}\hat{a}\ $

then  $e^{\hat{\xi}}=\begin{bmatrix}
   \sum_{n=0}^{\infty}\frac{1}{n!}(\hat{\phi}\ \ )^n & \sum_{n=0}^{\infty}\frac{1}{(n+1)!}(\hat{\phi}\ \ )^n\rho \\\\
   0^T & 1
  \end{bmatrix}
  =\begin{bmatrix}
   R & J\rho \\\\
   0^T & 0
  \end{bmatrix}$,
where $R=cos\theta\cdot I+(1-cos\theta)aa^T+sin\theta\cdot\hat{a}\ \ , J=\frac{sin\theta}{\theta}I\ +(1-\frac{sin\theta}{\theta})aa^T+\frac{1-cos\theta}{\theta}\hat{a}\ $
### 4伴随(2 分，约1 小时)
### 5轨迹的描绘

``` cpp
int main(int argc, char **argv)
 {

    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses;

    /// implement pose reading code
    // start your code here (5~10 lines)
    ifstream fin(trajectory_file);
    double t,tx,ty,tz,qx,qy,qz,qw;
    while(fin>>t>>tx>>ty>>tz>>qx>>qy>>qz>>qw)
    {
        //cout<<t<<" "<<tx<<" "<<ty<<" "<<tz<<" "<<qx<<" "<<qy<<" "<<qz<<" "<<qw<<endl;
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
```

### 6轨迹的误差

``` cpp
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