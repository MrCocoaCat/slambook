
//
// Created by swayfreeda on 17-10-20.
//
#include "bal_problem.h"
#include "ceres/problem.h"
#include "ceres/solver.h"
#include "snavely_reprojection_error.h"


#include "functions.h"
#include "robust_matcher.h"

#include <opencv2/core.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <fstream>

int main(int argc, char* argv[])
{
// ./test_imgs/DSC_9811.JPG ./test_imgs/DSC_9813.JPG focal length are 3696.000
    if(argc<3){
        std::cerr<<"usage: bfm_matcher matching img1, img2"<<std::endl;
        return -1;
    }

    // 加载第一幅图像
    cv::Mat img1=cv::imread(argv[1]);
    if(!img1.data){
        std::cerr<<"fail to load img1"<<std::endl;
        return -1;
    }
    std::cout<<"img1 infos: "<< img1.rows<<" x "<<img1.cols<<std::endl;


    // 加载第二幅图像
    cv::Mat img2=cv::imread(argv[2]);
    if(!img2.data){
        std::cerr<<"fail to load img2"<<std::endl;
        return -1;
    }
    std::cout<<"img2 infos: "<< img2.rows<<" x "<<img2.cols<<std::endl;


    // 为了更好的显示图像，对图像进行降采样处理
    cv::resize(img1, img1, cv::Size(img1.cols*0.2, img1.rows*0.2), 0.5, 0.5);
    cv::resize(img2, img2, cv::Size(img2.cols*0.2, img2.rows*0.2), 0.5, 0.5);
    std::cout<<"img1 infos after resize: "<< img1.rows<<" x "<<img1.cols<<std::endl;
    std::cout<<"img2 infos after resize: "<< img2.rows<<" x "<<img2.cols<<std::endl;


    /************************ 1.0检测特征点并进行匹配***********************/
    // 检测特征点并进行匹配
    clock_t  time_begin = clock();
    std::vector<cv::KeyPoint> sift_keypts1, sift_keypts2;
    std::vector<cv::DMatch> sift_matches;

    features_and_matching(img1, img2, sift_keypts1, sift_keypts2, sift_matches );
    std::cout<<sift_matches.size()<<" matches are obtained"<<std::endl;
    clock_t  time_end = clock();
    std::cout<<"sift matching cost "<<double(time_end-time_begin)/(double)CLOCKS_PER_SEC<<" s"<<std::endl;


    /************************ 2.0 姿态估计***********************************/
    // 利用RANSAC对特征匹配进行外点去除
    if(!refine_matches_via_fundemental(sift_keypts1, sift_keypts2, sift_matches)){
        std::cout<<" fail to refine mathces via homography!"<<std::endl;
        return 0;
    }
    std::cout<<sift_matches.size()<<" are remained after RANSAC based refinement"<<std::endl;


    // 相机焦距
    float focal_len = 3696.000;
    // 光心点位置
    cv::Point2f principal_point(img1.cols/2.0, img1.rows/2.0);
    // 构造内参数矩阵
    cv::Mat K=(cv::Mat_<double>(3,3)<< focal_len,    0,       principal_point.x,
            0,      focal_len,  principal_point.y,
            0,         0,              1.0);
    std::cout<<"K is: "<< K<<std::endl;

    // 估计第二幅图像相对于第一幅图像的旋转矩阵R和平移矩阵t
    cv::Mat R2, t2;
    pose_estimation(sift_keypts1, sift_keypts2, sift_matches, K, R2, t2);
    std::cout<<"R is: "<<std::endl<<R2<<std::endl;
    std::cout<<"t is: "<<std::endl<<t2<<std::endl;


    /************************ 3.0 三角化获取三维点*******************************/
    std::vector<cv::Point3d> pts3d;
    cv::Mat R1 = cv::Mat::eye(3,3,CV_64FC1);
    cv::Mat t1 = cv::Mat::zeros(3,1, CV_64FC1);
    triangulation(sift_keypts1,sift_keypts2, sift_matches, K, R1, t1, K, R2, t2, pts3d);


    /************************ 4.0 捆绑调整***********************************/
    // 收集相机和三维点的参数，用于bundle adjustment

    // 相机个数 C
    int n_cameras=2;
    // 三维点云个数 N
    int n_points = pts3d.size();

    // 在本实验中，每个三维点在两个相机中都可见，因此residual blocks的个数是CN

    // camera_index的长度是CNx1 每个元素对应residual block中的相机索引
    std::vector<int> camera_index;
    // point_index的长度是CNx1, 每个元素对应的是residual block中的三维点的索引
    std::vector<int> point_index;
    // 观察点的个数，总共有CN个
    std::vector<cv::Point2f> observations;
    std::vector<double> params;

    for(int i=0;i< sift_matches.size(); i++)
	{

        int query_id= sift_matches[i].queryIdx;
        cv::Point2f p1 =  sift_keypts1[query_id].pt;
        camera_index.push_back(0);
        point_index.push_back(i);
        observations.push_back(p1-principal_point); // 优化过程中以图像中心为坐标原点， principal point不需要传递到ceres优化过程


        int train_id= sift_matches[i].trainIdx;
        cv::Point2f p2 = sift_keypts2[train_id].pt;
        camera_index.push_back(1);
        point_index.push_back(i);
        observations.push_back(p2-principal_point);
    }
    std::cout<<"observation num shoule be "<<n_cameras*n_points<<std::endl;
    std::cout<<"params num  "<< observations.size()<<std::endl;

    // 相机参数，每个相机共有R,t, f, k1, k2 共9个参数， 总共有9* n_camera个参数
    cv::Mat r1, r2;
    cv::Rodrigues(R1, r1);// 角轴法 将3x3的旋转矩阵转换成3x1的向量
    cv::Rodrigues(R2, r2);

    // 径向畸变系数
    double k1=0, k2=0;

    // 设置第一个相机的参数 f, k1, k2, R, t
    for(int i=0; i<3; i++) params.push_back(r1.at<double>(i));
    for(int i=0; i<3; i++) params.push_back(t1.at<double>(i));
    params.push_back(focal_len);
    params.push_back(k1);
    params.push_back(k2);

    // 设置第二个相机的参数
    for(int i=0; i<3; i++) params.push_back(r2.at<double>(i));
    for(int i=0; i<3; i++) params.push_back(t2.at<double>(i));
    params.push_back(focal_len);
    params.push_back(k1);
    params.push_back(k2);


    // 设置三维点的参数，每个点有3个坐标，总共有3*n_points个参数
    for(int i=0; i<pts3d.size(); i++)
	{
        params.push_back(pts3d[i].x);
        params.push_back(pts3d[i].y);
        params.push_back(pts3d[i].z);
    }
    std::cout<<"params num shoule be "<< 9*n_cameras+3*n_points<<std::endl;
    std::cout<<"params num  "<< params.size()<<std::endl;


    // 构建bundle adjustment 问题
    ceres::examples::BALProblem bal_problem(n_cameras
                                           ,n_points
                                           ,observations.size()
                                           ,point_index
                                           ,camera_index
                                           ,observations
                                           ,params);


    // 获取观测到的图像点
    const double* observations_ptr = bal_problem.observations();
    // 对于每一个观察点，构造一个残差方程 |predicted_x - observed_x|^2 + |predicted_y - observed_y|^2
    ceres::Problem problem;
    for (int i = 0; i < bal_problem.num_observations(); ++i) 
	{
        // 每一个残差模块输入一个相机和一个三维点，输出一个二维的残差，实际上残差反映了冲投影误差
        ceres::CostFunction* cost_function =
                ceres::examples::SnavelyReprojectionError::Create(observations_ptr[2 * i + 0], observations_ptr[2 * i + 1]);
        problem.AddResidualBlock(cost_function,
                                 NULL /* squared loss */,
                                 bal_problem.mutable_camera_for_observation(i),
                                 bal_problem.mutable_point_for_observation(i));
    }

    //优化过程
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 500;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";


    // 获取优化结果，恢复相机参数
    cv::Mat r1_BA(3,1, CV_64FC1);
    cv::Mat r2_BA(3,1, CV_64FC1);
    cv::Mat R1_BA;
    cv::Mat R2_BA;
    cv::Mat t1_BA(3, 1, CV_64FC1);
    cv::Mat t2_BA(3, 1, CV_64FC1);
    double f1_BA, f2_BA, k11_BA, k12_BA, k21_BA, k22_BA;;
    const double * cam_params1 = bal_problem.mutable_camera_for_observation(0);
    recover_camera_parames(cam_params1, R1_BA, t1_BA,f1_BA, k11_BA, k12_BA);
    const double * cam_params2 = bal_problem.mutable_camera_for_observation(1);
    recover_camera_parames(cam_params2, R2_BA, t2_BA,f2_BA, k21_BA, k22_BA);

    // 打印输出结果
    std::cout<<std::endl<<"Camera1 before BA :"<<std::endl;
    std::cout<<"R:"<<std::endl<<R1<<std::endl;
    std::cout<<"t:"<<std::endl<<t1<<std::endl;
    std::cout<<"f:"<<focal_len<<std::endl;
    std::cout<<"k1:"<<k1<<std::endl;
    std::cout<<"k2:"<<k2<<std::endl;

    std::cout<<std::endl<<"Camera1 after BA :"<<std::endl;
    std::cout<<"R:"<<std::endl<<R1_BA<<std::endl;
    std::cout<<"t:"<<std::endl<<t1_BA<<std::endl;
    std::cout<<"f:"<<f1_BA<<std::endl;
    std::cout<<"k1:"<<k11_BA<<std::endl;
    std::cout<<"k2:"<<k12_BA<<std::endl;

    std::cout<<std::endl<<"Camera2 before BA :"<<std::endl;
    std::cout<<"R:"<<std::endl<<R2<<std::endl;
    std::cout<<"t:"<<std::endl<<t2<<std::endl;
    std::cout<<"f:"<<focal_len<<std::endl;
    std::cout<<"k1:"<<k1<<std::endl;
    std::cout<<"k2:"<<k2<<std::endl;

    std::cout<<std::endl<<"Camera2 after BA :"<<std::endl;
    std::cout<<"R:"<<std::endl<<R2_BA<<std::endl;
    std::cout<<"t:"<<std::endl<<t2_BA<<std::endl;
    std::cout<<"f:"<<f2_BA<<std::endl;
    std::cout<<"k1:"<<k21_BA<<std::endl;
    std::cout<<"k2:"<<k22_BA<<std::endl;



#if 0
   /********************************5. 显示bundle_adjustment 的结果 ************************/
    std::vector<cv::Point2f> pts1(sift_matches.size());
    std::vector<cv::Point2f> pts2(sift_matches.size());
    for(int i=0;i< sift_matches.size(); i++)
	{
        int query_id= sift_matches[i].queryIdx;
        int train_id= sift_matches[i].trainIdx;

        pts1[i]=  sift_keypts1[query_id].pt;
        pts2[i] = sift_keypts2[train_id].pt;
    }
   cv::Mat img1_before_BA = img1;
   cv::Mat img2_before_BA = img2;

    //投影矩阵
   cv::Mat P1 = get_projection_matrix(K, R1, t1);
   cv::Mat P2 = get_projection_matrix(K, R2, t2);
   for(int i=0; i< pts3d.size(); i++)
   {

       float r = rand()&255;
       float g = rand()&255;
       float b = rand()&255;

       cv::Mat pt4d = (cv::Mat_<double>(4,1)<<pts3d[i].x, pts3d[i].y, pts3d[i].z, 1.0);

       // 在第一幅图像上画出观测到的点
       cv::circle(img1_before_BA, pts1[i], 5, cv::Scalar(r, g, b), 1);
       cv::Mat x1 = P1 * pt4d;
       cv::Point2f p1(x1.at<double>(0)/x1.at<double>(2),x1.at<double>(1)/x1.at<double>(2) );
       // 在第一幅图像上画出预测的点
       cv::circle(img1_before_BA, p1, 2, cv::Scalar(0, 0, 255), 1);
       cv::line(img1_before_BA, pts1[i], p1, cv::Scalar(0, 255, 0), 1);

       // 在第二幅图像上画出检测倒的特征点
       cv::circle(img2_before_BA, pts2[i], 5, cv::Scalar(r, g, b), 1);
       cv::Mat x2 = P2 * pt4d;
       cv::Point2f p2(x2.at<double>(0)/x2.at<double>(2),x2.at<double>(1)/x2.at<double>(2) );
       // 在第一幅图像上画出预测的点
       cv::circle(img2_before_BA, p2, 2, cv::Scalar(0, 0, 255), 1);
       cv::line(img2_before_BA, pts2[i], p2, cv::Scalar(0, 255, 0), 1);
   }
    cv::imwrite("./img1_before_BA.jpg", img1_before_BA);
    cv::imwrite("./img2_before_BA.jpg", img2_before_BA);
#endif

    return 0;
}
