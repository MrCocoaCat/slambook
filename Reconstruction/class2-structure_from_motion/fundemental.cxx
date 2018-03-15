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
    robust_matcher->match(sift_descrs1, sift_descrs2, sift_matches);


    // 划除匹配结果图
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


    // 通过RANSAC对匹配点进行滤波
    if(!refine_matches_via_fundemental(sift_keypts1, sift_keypts2, sift_matches))
	{
        std::cout<<"fail to refine matches via fundemental"<<std::endl;
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

    // 计算本征矩阵
    // 获取匹配点的特征位置
    {

        const int minimun_matches_allowed=8;
        if(sift_matches.size()< minimun_matches_allowed)
		{
            std::cerr<<"more than 8 mathces are needed!"<<std::endl;
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

        cv::Mat F = cv::findFundamentalMat(pts1
                ,pts2
                ,CV_FM_8POINT);

        std::cout<<"Fundemental Mat is: "<<std::endl<<F<<std::endl;
        // draw the left points corresponding epipolar  lines in right image
        //在第二幅图像中画出第一幅图像上的特征点对应的极线
        std::vector<cv::Vec3f> lines1;
        cv::computeCorrespondEpilines(cv::Mat(pts1) // 图像特征点
                ,1
                ,F // 本征矩阵
                ,lines1 // 极线
        );

        cv::Mat img_with_epiloar_lines = img2;
        for (std::vector<cv::Vec3f>::const_iterator it= lines1.begin(); it!=lines1.end(); ++it) {
            cv::line(img_with_epiloar_lines
                    ,cv::Point(0,-(*it)[2]/(*it)[1])
                    ,cv::Point(img2.cols,-((*it)[2]+ (*it)[0]*img2.cols)/(*it)[1]),
                     cv::Scalar(255,0,0));
        }
        cv::imwrite("./img_with_epiloar_lines.jpg", img_with_epiloar_lines );


        // 基础矩阵的秩为2，行列式值为0
        std::cout<<"determinant of Fundemental matrix should be 0 : "<< cv::determinant(F)<<std::endl;

        // 基础矩阵的奇异值为diag[sigma1, sigma2, 0]
        // 验证本征矩阵的特征值
        cv::SVD svd;
        cv::Mat S, U, V;
        svd.compute(F, S, U, V);
        std::cout<<"eigenvalues of fundemental matrix: "<<std::endl<<S<<std::endl;

        for(int i=0; i< pts1.size(); i++){
            cv::Mat pl(3, 1, CV_64FC1);
            cv::Mat pr(1, 3, CV_64FC1);

            // 第一个视角中的点
            pl.at<double>(0) = pts1[i].x;
            pl.at<double>(1) = pts1[i].y;
            pl.at<double>(2) = 1.0;

            // 第二个视角中的点
            pr.at<double>(0) = pts2[i].x;
            pr.at<double>(1) = pts2[i].y;
            pr.at<double>(2) = 1.0;


            cv::Mat tmp = pr*F;
            cv::Mat dot = tmp*pl;
            std::cout<< "value should be 0: "<< dot.at<double>(0)<<std::endl;
        }

    }

    return 0;
}
