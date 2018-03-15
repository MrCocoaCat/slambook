//
// Created by swayfreeda on 17-10-20.
//

//
// Created by swayfreeda on 17-10-24.
//
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


    // 检测特征点并进行匹配
    clock_t  time_begin = clock();
    std::vector<cv::KeyPoint> sift_keypts1, sift_keypts2;
    std::vector<cv::DMatch> sift_matches;

    features_and_matching(img1, img2, sift_keypts1, sift_keypts2, sift_matches );
    std::cout<<sift_matches.size()<<" matches are obtained"<<std::endl;
    clock_t  time_end = clock();
    std::cout<<"sift matching cost "<<double(time_end-time_begin)/(double)CLOCKS_PER_SEC<<" s"<<std::endl;


    // 利用RANSAC对特征匹配进行外点去除
    if(!refine_matches_via_fundemental(sift_keypts1, sift_keypts2, sift_matches)){
        std::cout<<" fail to refine mathces via homography!"<<std::endl;
        return 0;
    }
    std::cout<<sift_matches.size()<<" are remained after RANSAC based refinement"<<std::endl;


    // 相机焦距
    float focal_len = 3696.000;
    // 光心点位置
    cv::Point2d principal_point(img1.cols/2.0, img1.rows/2.0);
    // 构造内参数矩阵
    cv::Mat K=(cv::Mat_<double>(3,3)<< focal_len,    0,       principal_point.x,
            0,      focal_len,  principal_point.y,
            0,         0,              1.0);
    std::cout<<"K is: "<< K<<std::endl;
    // 估计第二幅图像相对于第一幅图像的旋转矩阵R和平移矩阵t
    cv::Mat R, t;
    pose_estimation(sift_keypts1, sift_keypts2, sift_matches, K, R, t);
    std::cout<<"R is: "<<std::endl<<R<<std::endl;
    std::cout<<"t is: "<<std::endl<<t<<std::endl;



    /*****************开始三角化匹配点*******************/
    // 第一幅图像的投影矩阵
    cv::Mat P1 = get_projection_matrix(K, cv::Mat::eye(3,3,CV_64FC1), cv::Mat::zeros(3,1, CV_64FC1));

    // 第二幅图像的投影矩阵
    cv::Mat P2 = get_projection_matrix(K, R, t);

    // 获取点的两幅图像上特征点的坐标
    std::vector<cv::Point2f> pts1(sift_matches.size());
    std::vector<cv::Point2f> pts2(sift_matches.size());
    for(int i=0;i< sift_matches.size(); i++){
        int query_id= sift_matches[i].queryIdx;
        int train_id= sift_matches[i].trainIdx;

        pts1[i]=  sift_keypts1[query_id].pt;
        pts2[i] = sift_keypts2[train_id].pt;
    }

    cv::Mat pts4d;
    cv::triangulatePoints(P1
                         ,P2
                         ,pts1
                         ,pts2
                         ,pts4d // 三角化的三维点以齐次坐标的方式存储 每一列存储一个三维点 4xN
                         );

    std::vector<cv::Point3d> pts3d;
    for(int i=0; i<pts4d.cols; i++){
        cv::Mat pth = pts4d.col(i);
        pth/=pth.at<float>(3);
        pts3d.push_back(cv::Point3d(pth.at<float>(0), pth.at<float>(1), pth.at<float>(2)));
    }

    std::cout<<"triangulated point: "<<std::endl;
    for(int i=0; i< pts3d.size(); i++){
        std::cout<< pts3d[i]<<std::endl;
    }

    /*****************************验证三角化*****************************/
    // 保存已知的内参数矩阵，旋转矩阵，平移向量，三维空间点和第二幅图像上的重新投影点，用于测试triangulation
    std::ofstream out_put("3d2dcoresspondence");
    assert(out_put.is_open());

    //write image name
    out_put<<argv[2]<<std::endl;

    // write K
    for(int i=0; i< K.rows; i++){
        for(int j=0; j< K.cols; j++){
            out_put<<K.at<double>(i,j)<<" ";
        }
    }
    out_put<<std::endl;

    // write R
    for(int i=0; i< R.rows; i++){
        for(int j=0; j< R.cols; j++){
            out_put<<R.at<double>(i,j)<<" ";
        }
    }
    out_put<<std::endl;

    // write t
    for(int i=0; i< t.rows; i++){
        for(int j=0; j< t.cols; j++){
            out_put<<t.at<double>(i,j)<<" ";
        }
    }
    out_put<<std::endl;

    for(int i=0; i< pts4d.cols; i++){
        cv::Mat pth = pts4d.col(i);
        pth/=pth.at<float>(3);
        pth.convertTo(pth, CV_64FC1);
        cv::Mat x1 = P1* pth;
        x1/= x1.at<double>(2);

        //std::cout<<pth.at<double>(0)<<" "<<pth.at<double>(1)<<" "<< pth.at<double>(2)<<" ";

        float reproj_error1 = std::sqrt((x1.at<double>(0)- pts1[i].x)*(x1.at<double>(0)- pts1[i].x)
                                      + (x1.at<double>(1)- pts1[i].y)*(x1.at<double>(1)- pts1[i].y));
        std::cout<<"reprojection error1: "<<reproj_error1<<" ";

        out_put<< pth.at<double>(0)<< " "<<pth.at<double>(1)<<" "<< pth.at<double>(2)<<" ";


//        cv::Mat pt3d = (cv::Mat_<double>(3,1)<<pth.at<double>(0), pth.at<double>(1), pth.at<double>(2));
//       // cv::Mat rotated = R*pt3d;
//       // std::cout<<"R: "<<R<<std::endl;
//       // std::cout<<"rotated: "<<rotated<<std::endl;
//        cv::Mat rotated_trans = rotated + t;
//        std::cout<<"translated: "<< rotated_trans<<std::endl;
//        std::cout<<"image coordinate: "<< focal_len*rotated_trans/rotated_trans.at<double>(2)<<std::endl;


        cv::Mat x2 = P2* pth;
        x2/= x2.at<double>(2);
        //std::cout<<"x2: "<<x2.at<double>(0)<<" "<<x2.at<double>(1)<<" "<< pts2[i].x<<" "<<pts2[i].y<<std::endl;
        float reproj_error2 = std::sqrt( (x2.at<double>(0)- pts2[i].x)*(x2.at<double>(0)- pts2[i].x)
                                        +(x2.at<double>(1)- pts2[i].y)*(x2.at<double>(1)- pts2[i].y));
        std::cout<<"reprojection error2: "<<reproj_error2<<std::endl;

        out_put<< x2.at<double>(0)<< " "<<x2.at<double>(1)<<std::endl;

    }


    return 0;
}

