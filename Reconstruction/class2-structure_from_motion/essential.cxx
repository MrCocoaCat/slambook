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

//  只对这两幅图像有用，./test_imgs/DSC_9811.JPG ./test_imgs/DSC_9813.JPG 因为相机焦距focal length are 3696.000是预先知道的
// 如果要测试其它图像的话，需要预先知道相机的焦距

int main(int argc, char* argv[])
{

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
    // 画出匹配结果图
    cv::Mat img_sift_matches;
    cv::drawMatches(img1
            ,sift_keypts1
            ,img2
            ,sift_keypts2
            ,sift_matches
            ,img_sift_matches
            ,cv::Scalar::all(-1) // 连接线的颜色
    );
    cv::imwrite("./sift_matching.jpg", img_sift_matches);
    clock_t  time_end = clock();
    std::cout<<"sift matching cost "<<double(time_end-time_begin)/(double)CLOCKS_PER_SEC<<" s"<<std::endl;


    // 利用RANSAC对特征匹配进行外点去除
    if(!refine_matches_via_fundemental(sift_keypts1, sift_keypts2, sift_matches)){
        std::cout<<" fail to refine mathces via homography!"<<std::endl;
        return 0;
    }
    std::cout<<sift_matches.size()<<"  matches inliers after refinement"<<std::endl;
    // 画出优化后的匹配结果图
    cv::Mat img_sift_matches_after_refinement;
    cv::drawMatches(img1
            ,sift_keypts1
            ,img2
            ,sift_keypts2
            ,sift_matches
            ,img_sift_matches_after_refinement
            ,cv::Scalar::all(-1) // 连接线的颜色
    );
    cv::imwrite("./sift_matching_after_refinement.jpg", img_sift_matches_after_refinement);



    // 计算本质矩阵
    // 获取匹配点的特征位置
    const int minimun_matches_allowed=8;
    if(sift_matches.size()< minimun_matches_allowed){
        std::cerr<<"more than 8 mathces are needed!"<<std::endl;
        return 0;
    }

    // 获取点的两幅图像上特征点的坐标
    std::vector<cv::Point2f> pts1(sift_matches.size());
    std::vector<cv::Point2f> pts2(sift_matches.size());
    for(int i=0;i< sift_matches.size(); i++){
        int query_id= sift_matches[i].queryIdx;
        int train_id= sift_matches[i].trainIdx;

        pts1[i]= sift_keypts1[query_id].pt;
        pts2[i] =sift_keypts2[train_id].pt;
    }

    // 相机焦距
    float focal_len = 3696.000;

    // 光心点位置
    cv::Point2d principal_point(img1.cols/2.0, img1.rows/2.0);

    // 构造内参数矩阵
    cv::Mat K=(cv::Mat_<double>(3,3)<< focal_len,    0,       principal_point.x,
            0,      focal_len,  principal_point.y,
            0,         0,              1.0);


    // 计算本质矩阵
    cv::Mat essential_matrix = cv::findEssentialMat(pts1, pts2, focal_len, principal_point, cv::LMEDS);
    std::cout<<"essential matrix: "<<std::endl<<essential_matrix/essential_matrix.at<double>(2,2)<<std::endl;

    cv::Mat R, t;
    cv::recoverPose(essential_matrix, pts1, pts2, R, t, focal_len, principal_point);
    std::cout<<"R is: "<<std::endl<<R<<std::endl;
    std::cout<<"t is: "<<std::endl<<t<<std::endl;


    // 验证本征矩阵的特征值
    cv::SVD svd;
    cv::Mat S, U, V;
    svd.compute(essential_matrix, S, U, V);
    std::cout<<"eigenvalues of essential matrix: "<<std::endl<<S<<std::endl;


    // 验证[t]xR 注意上面计算的essential_matrix是有尺度空间上的变化的，也就是说本质矩阵乘以任何常数都会满足对极约束
    // 因此为了验证[t]xR 和essential_matrix我们进行了归一化，essential_matrix/essential_matrix.at<double>(2,2)
    // E_est/E_est.at<double>(2,2)。
    cv::Mat t_x = (cv::Mat_<double>(3,3)<< 0,                 -t.at<double>(2),  t.at<double>(1),
                                           t.at<double>(2),   0,                -t.at<double>(0),
                                           -t.at<double>(1),  t.at<double>(0) , 0);
    cv::Mat E_est = t_x* R;
    std::cout<<"[t]xR is "<< std::endl<<E_est/E_est.at<double>(2,2)<<std::endl;

    // 验证对极约束

    for(int i=0; i< pts1.size(); i++)
    {
       cv::Point2d x1 = pixel2cam(pts1[i], K);
       cv::Point2d x2 = pixel2cam(pts2[i], K);
       cv::Mat x1m = (cv::Mat_<double>(3,1)<< x1.x, x1.y, 1.0);
       cv::Mat x2m = (cv::Mat_<double>(3,1)<< x2.x, x2.y,1.0);

       cv::Mat d =  x2m.t() * E_est * x1m;
       std::cout<<" epipolar constraint value shoule be 0: "<<d.at<double>(0)<<std::endl;
    }



    return 0;
}