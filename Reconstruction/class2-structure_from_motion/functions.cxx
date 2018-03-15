//
// Created by swayfreeda on 17-10-24.
//
#include "functions.h"
#include <iostream>
#include <opencv2/xfeatures2d/nonfree.hpp>

void features_and_matching(const cv::Mat &img1
                          ,const cv::Mat& img2
                          ,std::vector<cv::KeyPoint> &sift_keypts1
                          ,std::vector<cv::KeyPoint> &sift_keypts2
                          ,std::vector<cv::DMatch> &matches) {

    //设置需要检测的特征个数
    int n_features = 10000;

    // 进行sift特征的检测和匹配
    cv::Mat sift_descrs1, sift_descrs2;

    // 检测sift特征
    cv::Ptr<cv::xfeatures2d::SIFT> sift_detector = cv::xfeatures2d::SIFT::create(n_features, 3, 0.04, 10, 1.6);
    sift_detector->detect(img1, sift_keypts1);
    sift_detector->detect(img2, sift_keypts2);
    std::cout << sift_keypts1.size() << " key points are detected from img1" << std::endl;
    std::cout << sift_keypts2.size() << " key points are detected from img2" << std::endl;

    // 计算sift描述子
    sift_detector->compute(img1, sift_keypts1, sift_descrs1);
    sift_detector->compute(img2, sift_keypts2, sift_descrs2);


    // sift 特征匹配
    clock_t time_begin = clock();
    cv::Ptr<RobustMatcher> robust_matcher = RobustMatcher::create();
    robust_matcher->match(sift_descrs1, sift_descrs2, matches);
}

bool refine_matches_via_homography(const std::vector<cv::KeyPoint> & keypts1
        ,const std::vector<cv::KeyPoint> &keypts2
        , std::vector<cv::DMatch> &matches)
{

    const int minimun_matches_allowed=8;
    if(matches.size()< minimun_matches_allowed){
        std::cerr<<" more than 8 matches are needed for homography"<<std::endl;
        return false;
    }

    // 获取点的两幅图像上特征点的坐标
    std::vector<cv::Point2f> pts1(matches.size());
    std::vector<cv::Point2f> pts2(matches.size());
    for(int i=0;i< matches.size(); i++){
        int query_id= matches[i].queryIdx;
        int train_id= matches[i].trainIdx;

        pts1[i]= keypts1[query_id].pt;
        pts2[i] = keypts2[train_id].pt;
    }

    std::vector<uchar> inlier_mask(matches.size());

    cv::Mat homography = cv::findHomography(pts1
            ,pts2
            ,cv::RANSAC // 计算方法
            ,1  // 在另一幅图像上计算的坐标误差(像素)
            ,inlier_mask // 指示内点
           );

    std::vector<cv::DMatch> inlier_matches;
    for(int i=0; i<inlier_mask.size(); i++){
        if(inlier_mask[i])
            inlier_matches.push_back(matches[i]);
    }
    //std::cout<<"homography: "<<homography<<std::endl;

    matches.swap(inlier_matches);
    return true;
}

bool refine_matches_via_fundemental(const std::vector<cv::KeyPoint> & keypts1
        ,const std::vector<cv::KeyPoint> & keypts2
        ,std::vector<cv::DMatch> &matches)
{
    const int minimun_matches_allowed=8;
    if(matches.size()< minimun_matches_allowed){
        std::cerr<<" more than 8 matches are needed for fundemental matrix"<<std::endl;
        return false;
    }

    // 获取点的两幅图像上特征点的坐标
    std::vector<cv::Point2f> pts1(matches.size());
    std::vector<cv::Point2f> pts2(matches.size());
    for(int i=0;i< matches.size(); i++){
        int query_id= matches[i].queryIdx;
        int train_id= matches[i].trainIdx;

        pts1[i]= keypts1[query_id].pt;
        pts2[i] = keypts2[train_id].pt;
    }

    // 存储内点匹配对
    std::vector<uchar> inlier_mask(matches.size());

    // RANSAC 计算Fundamental Mat
    cv::Mat fundemental = cv::findFundamentalMat(pts1, pts2 ,inlier_mask, cv::FM_RANSAC, 1, 0.99);

    // 去掉外点匹配对
    std::vector<cv::DMatch> inlier_matches;
    for(int i=0; i<inlier_mask.size(); i++){
        if(inlier_mask[i])
            inlier_matches.push_back(matches[i]);
    }
    matches.swap(inlier_matches);
    return true;
}

cv::Point2d pixel2cam(const cv::Point2d & p, const cv::Mat& K){

    cv::Mat ph = (cv::Mat_<double>(3,1)<< p.x,p.y, 1.0);
    cv::Mat xc = K.inv()* ph;
    return cv::Point2d(xc.at<double>(0), xc.at<double>(1));

}

void pose_estimation( const std::vector<cv::KeyPoint> keypts1
        ,const std::vector<cv::KeyPoint> keypts2
        ,const std::vector<cv::DMatch> &matches
        ,const cv::Mat&K
        ,cv::Mat&R
        ,cv::Mat&t)
{
    // 计算本质矩阵
    // 获取匹配点的特征位置
    const int minimun_matches_allowed=8;
    if(matches.size()< minimun_matches_allowed){
        std::cerr<<"more than 8 mathces are needed!"<<std::endl;
        return;
    }

    // 获取点的两幅图像上特征点的坐标
    std::vector<cv::Point2f> pts1(matches.size());
    std::vector<cv::Point2f> pts2(matches.size());
    for(int i=0;i< matches.size(); i++){
        int query_id= matches[i].queryIdx;
        int train_id= matches[i].trainIdx;

        pts1[i]= keypts1[query_id].pt;
        pts2[i] =keypts2[train_id].pt;
    }

    // 计算本质矩阵
    cv::Mat essential_matrix = cv::findEssentialMat(pts1
                                                  , pts2
                                                  , K.at<double>(0,0)
                                                  , cv::Point2d(K.at<double>(0,2), K.at<double>(1,2))
                                                  , cv::LMEDS);
    std::cout<<"essential matrix: "<<std::endl<<essential_matrix/essential_matrix.at<double>(2,2)<<std::endl;

    cv::recoverPose(essential_matrix
                   ,pts1
                   ,pts2
                   ,R
                   ,t
                   ,K.at<double>(0,0)
                   ,cv::Point2d(K.at<double>(0,2), K.at<double>(1,2))
                   );
    //std::cout<<"R is: "<<std::endl<<R<<std::endl;
    //std::cout<<"t is: "<<std::endl<<t<<std::endl;

}

void triangulation(const std::vector<cv::KeyPoint> keypts1
        ,const std::vector<cv::KeyPoint> keypts2
        ,const std::vector<cv::DMatch> &matches
        ,const cv::Mat & K1
        ,const cv::Mat & R1
        ,const cv::Mat & t1
        ,const cv::Mat & K2
        ,const cv::Mat & R2
        ,const cv::Mat & t2
        ,std::vector<cv::Point3d> & pts3d){


    /*****************开始三角化匹配点*******************/
    // 第一幅图像的投影矩阵
    cv::Mat left_part1 = K1*R1;
    cv::Mat right_part1 = K1*t1;
    cv::Mat P1(3,4, CV_64FC1);
    P1.setTo(0);
    for(int i=0; i<3; i++){
        for(int j=0; j<3; j++){
            P1.at<double>(i,j)=left_part1.at<double>(i,j);
        }
    }
    for(int i=0; i<3; i++)P1.at<double>(i, 3)= right_part1.at<double>(i);

    // 第二幅图像的投影矩阵
    cv::Mat left_part2  =  K2*R2;
    cv::Mat right_part2 =  K2*t2;
    cv::Mat P2(3,4, CV_64FC1);
    for(int i=0; i<3; i++){
        for(int j=0; j<3; j++){
            P2.at<double>(i,j)= left_part2.at<double>(i,j);
        }
    }
    for(int i=0; i<3; i++)P2.at<double>(i, 3)= right_part2.at<double>(i);


    // 获取点的两幅图像上特征点的坐标
    std::vector<cv::Point2f> pts1(matches.size());
    std::vector<cv::Point2f> pts2(matches.size());
    for(int i=0;i< matches.size(); i++){
        int query_id= matches[i].queryIdx;
        int train_id= matches[i].trainIdx;

        pts1[i]=  keypts1[query_id].pt;
        pts2[i] = keypts2[train_id].pt;
    }

    cv::Mat pts4d;
    cv::triangulatePoints(P1
            ,P2
            ,pts1
            ,pts2
            ,pts4d // 三角化的三维点以齐次坐标的方式存储 每一列存储一个三维点 4xN
    );

    pts3d.clear();
    for(int i=0; i<pts4d.cols; i++){
        cv::Mat pth = pts4d.col(i);
        pth/=pth.at<float>(3);
        pts3d.push_back(cv::Point3d(pth.at<float>(0), pth.at<float>(1), pth.at<float>(2)));
    }
}


cv::Mat get_projection_matrix(const cv::Mat& K, const cv::Mat & R, const cv::Mat & t){
    // 第二幅图像的投影矩阵
    cv::Mat left_part = K*R;
    cv::Mat right_part = K*t;
    cv::Mat P(3,4, CV_64FC1);
    for(int i=0; i<3; i++){
        for(int j=0; j<3; j++){
            P.at<double>(i,j)= left_part.at<double>(i,j);
        }
    }
    for(int i=0; i<3; i++)P.at<double>(i, 3)= right_part.at<double>(i);

    return P;
}

void recover_camera_parames(const double* params, cv::Mat & R, cv::Mat & t, double& f, double & k1, double &k2){

    R.create(3, 3, CV_64FC1);
    t.create(3, 1, CV_64FC1);

    cv::Mat r(3,1, CV_64FC1);

    assert(params);

    // 恢复旋转矩阵
    for(int i=0; i<3; i++) r.at<double>(i) = params[i];
    cv::Rodrigues(r, R); // 角轴法 将3x1的向量换成3x3的旋转矩阵转

    // 恢复平移向量
    for(int i=0; i<3; i++) t.at<double>(i) = params[i+3];

    // 焦距
    f = params[6];

    //径向畸变系数
    k1= params[7];
    k2= params[7];
}