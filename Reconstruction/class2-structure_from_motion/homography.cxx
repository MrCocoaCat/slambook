//
// Created by swayfreeda on 17-10-10.
//
#include "functions.h"
#include "robust_matcher.h"

#include <opencv2/core.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>
#include <opencv2/imgcodecs.hpp>

int main(int argc, char* argv[])
{

    if(argc<3)
	{
        std::cerr<<"usage: bfm_matcher matching img1, img2"<<std::endl;
        return -1;
    }

    // 加载第一幅图像
    cv::Mat img1=cv::imread(argv[1]);
    if(!img1.data)
	{
        std::cerr<<"fail to load img1"<<std::endl;
        return -1;
    }
    std::cout<<"img1 infos: "<< img1.rows<<" x "<<img1.cols<<std::endl;


    // 加载第二幅图像
    cv::Mat img2=cv::imread(argv[2]);
    if(!img2.data)
	{
        std::cerr<<"fail to load img2"<<std::endl;
        return -1;
    }
    std::cout<<"img2 infos: "<< img2.rows<<" x "<<img2.cols<<std::endl;


    // 为了更好的显示图像，对图像进行降采样处理
    cv::resize(img1, img1, cv::Size(img1.cols*0.2, img1.rows*0.2), 0.5, 0.5);
    cv::resize(img2, img2, cv::Size(img2.cols*0.2, img2.rows*0.2), 0.5, 0.5);
    std::cout<<"img1 infos after resize: "<< img1.rows<<" x "<<img1.cols<<std::endl;
    std::cout<<"img2 infos after resize: "<< img2.rows<<" x "<<img2.cols<<std::endl;


    //设置需要检测的特征个数
    int n_features=8000;

    // 进行sift特征的检测和匹配
    std::vector<cv::KeyPoint> sift_keypts1, sift_keypts2;
    cv::Mat sift_descrs1, sift_descrs2;


    // 检测sift特征
    cv::Ptr<cv::xfeatures2d::SIFT> sift_detector=cv::xfeatures2d::SIFT::create(n_features
            ,3
            ,0.04
            ,10
            ,1.6);
    sift_detector->detect(img1, sift_keypts1);
    sift_detector->detect(img2, sift_keypts2);
    std::cout<<sift_keypts1.size()<<" key points are detected from img1"<<std::endl;
    std::cout<<sift_keypts2.size()<<" key points are detected from img2"<<std::endl;

    // 计算sift描述子
    sift_detector->compute(img1, sift_keypts1, sift_descrs1);
    sift_detector->compute(img2, sift_keypts2, sift_descrs2);

    // sift 特征匹配
    clock_t  time_begin = clock();
    std::vector<cv::DMatch> sift_matches;
    cv::Ptr<RobustMatcher> robust_matcher = RobustMatcher::create();
    robust_matcher->match( sift_descrs1, sift_descrs2, sift_matches);

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
    if(!refine_matches_via_homography(sift_keypts1, sift_keypts2, sift_matches))
	{
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

    // 计算单应矩阵
    // 获取匹配点的特征位置
	
    const int minimun_matches_allowed=8;
    if(sift_matches.size()< minimun_matches_allowed)
	{
        std::cerr<<"more than 8 mathces are needed!"<<std::endl;
        return 0;
    }

    // 获取点的两幅图像上特征点的坐标
    std::vector<cv::Point2f> pts1(sift_matches.size());
    std::vector<cv::Point2f> pts2(sift_matches.size());
    for(int i=0;i< sift_matches.size(); i++)
	{
        int query_id= sift_matches[i].queryIdx;
        int train_id= sift_matches[i].trainIdx;

        pts1[i]= sift_keypts1[query_id].pt;
        pts2[i] =sift_keypts2[train_id].pt;
    }

    cv::Mat H = cv::findHomography(pts1
            ,pts2
            ,0);

    std::cout<<"Homography is: "<<H<<std::endl;


    // 将特征点画到图像上
    for(int i=0; i< pts1.size(); i++)
	{
        float r = rand()&255;
        float g = rand()&255;
        float b = rand()&255;
        cv::circle(img1, pts1[i],3, cv::Scalar(r,g, b),2);
        cv::circle(img2, pts2[i],3, cv::Scalar(r,g, b),2);
    }

    // 图像WRAP
    cv::Mat result;
    cv::warpPerspective(img1 // input image
                        ,result // 输出图像
                        ,H
                        ,cv::Size(2*img1.cols, img1.rows)//输出图像的尺寸
                        );
    // 将图像1覆盖倒result图像的左半部分
    cv::Mat half(result,cv::Rect(0,0,img2.cols,img2.rows));
    // 将图像2复制倒result的右半部分
    img2.copyTo(half);
    cv::imwrite("./homograpy_retult.jpg", result);


    for(int i=0; i< pts1.size(); i++)
	{
        cv::Mat pl(3, 1, CV_64FC1);
        cv::Mat pr(1, 3, CV_64FC1);

        // 第一个视角中的点
        pl.at<double>(0) = pts1[i].x;
        pl.at<double>(1) = pts1[i].y;
        pl.at<double>(2) = 1.0;


        cv::Mat pt_result = H*pl;
        double x = pt_result.at<double>(0)/=pt_result.at<double>(2);
        double y = pt_result.at<double>(1)/=pt_result.at<double>(2);

        std::cout<< "coordinates should be the same: ("<< pts2[i].x<<", "<<pts2[i].y<<" )--->"
                                               <<"( "<<x<<", "<<y<<") "<<std::endl;
    }


    return 0;
}