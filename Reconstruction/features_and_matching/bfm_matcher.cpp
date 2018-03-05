//
// Created by swayfreeda on 17-10-24.
//
#include "functions.h"

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
    /*it is false, this is will be default BFMatcher behaviour when it finds the k
    nearest neighbors for each query descriptor. If crossCheck==true , then the knnMatch()
    method with k=1 will only return pairs (i,j) such that for i-th query descriptor the j-th
    descriptor in the matcher’s collection is the nearest and vice versa, i.e. the BFMatcher will
    only return consistent pairs. Such technique usually produces best results with minimal
    number of outliers when there are enough matches. This is alternative to the ratio test, used
    by D. Lowe in SIFT paper.*/
    cv::Ptr<cv::BFMatcher> sift_matcher = cv::BFMatcher::create(cv::NORM_L2
                                                                ,true /* cross check */
                                                                );
    
    sift_matcher->match(sift_descrs1, sift_descrs2, sift_matches);
    clock_t  time_end = clock();
    std::cout<<sift_matches.size()<<"  matches are found "<<std::endl;
    std::cout<<"bfm sift matching cost "<<double(time_end-time_begin)/(double)CLOCKS_PER_SEC<<" s"<<std::endl;


    // 选择前50个匹配结果进行显示
    std::vector<cv::DMatch>sift_matches_for_display;
    sift_matches_for_display.insert(sift_matches_for_display.end(), sift_matches.begin(), sift_matches.end());
    if(sift_matches_for_display.size()> 50){
         std::nth_element(sift_matches_for_display.begin(), sift_matches_for_display.begin()+50, sift_matches_for_display.end());
         sift_matches_for_display.erase(sift_matches_for_display.begin()+51, sift_matches_for_display.end());
    }

    // 选择前百分之十的特征点进行homograpy检测
    int nth_element=std::max(int (0.1*n_features),500);
    std::vector<cv::DMatch>good_sift_matches;
    good_sift_matches.insert(good_sift_matches.end(), sift_matches.begin(), sift_matches.end());
    if(good_sift_matches.size()> nth_element){
        std::nth_element(good_sift_matches.begin(), good_sift_matches.begin()+nth_element, good_sift_matches.end());
        good_sift_matches.erase(good_sift_matches.begin()+nth_element, good_sift_matches.end());
    }

    // 划除匹配结果图
    cv::Mat img_sift_matches_before_homography;
    cv::drawMatches(img1
                    ,sift_keypts1
                    ,img2
                    ,sift_keypts2
                    ,sift_matches_for_display
                    ,img_sift_matches_before_homography
                    ,cv::Scalar::all(-1) // 连接线的颜色
                    );
    cv::imwrite("./sift_matching_before_homography.jpg", img_sift_matches_before_homography);


    // 利用homography 对相机对匹配点进行筛选
    std::cout<<"num of sift features before/after homography: "<< good_sift_matches.size()<<"--->";
    refine_matches_via_homography(sift_keypts1, sift_keypts2, good_sift_matches);
    std::cout<<good_sift_matches.size()<<std::endl;

    // 划除匹配结果图
    cv::Mat img_sift_matches_after_homography;
    cv::drawMatches(img1
            ,sift_keypts1
            ,img2
            ,sift_keypts2
            ,good_sift_matches
            ,img_sift_matches_after_homography
            ,cv::Scalar::all(-1) // 连接线的颜色
    );
    cv::imwrite("./sift_matching_after_homography.jpg", img_sift_matches_after_homography);



//********************************ORB features and detection **********************************//
    //设置需要检测的特征个数

    // 进行sift特征的检测和匹配
    std::vector<cv::KeyPoint> orb_keypts1, orb_keypts2;
    cv::Mat orb_descrs1, orb_descrs2;


    // 检测sift特征
    cv::Ptr<cv::ORB> orb_detector=cv::ORB::create(n_features
                                                  ,1.2f
                                                  ,8
                                                  ,31
                                                  ,0
                                                  ,2
                                                  ,cv::ORB::HARRIS_SCORE
                                                  ,31
                                                  ,20
                                                 );
    orb_detector->detect(img1, orb_keypts1);
    orb_detector->detect(img2, orb_keypts2);
    std::cout<<orb_keypts1.size()<<" orb key points are detected from img1"<<std::endl;
    std::cout<<orb_keypts2.size()<<" orb key points are detected from img2"<<std::endl;

    // 计算orb描述子
    orb_detector->compute(img1, orb_keypts1, orb_descrs1);
    orb_detector->compute(img2, orb_keypts2, orb_descrs2);


    // orb 特征匹配
    time_begin = clock();
    std::vector<cv::DMatch> orb_matches;
    cv::Ptr<cv::BFMatcher> orb_matcher = cv::BFMatcher::create(cv::NORM_HAMMING
                                                               ,true /* cross check */
                                                              );
    orb_matcher->match(orb_descrs1, orb_descrs2, orb_matches);
    time_end = clock();
    std::cout<<orb_matches.size()<<"  matches are found "<<std::endl;
    std::cout<<"orb matching cost "<<double(time_end-time_begin)/(double)CLOCKS_PER_SEC<<" s"<<std::endl;


    std::vector<cv::DMatch>orb_matches_for_display;
    orb_matches_for_display.insert(orb_matches_for_display.end(), orb_matches.begin(), orb_matches.end());
    if(orb_matches_for_display.size()> 50){
        std::nth_element(orb_matches_for_display.begin(), orb_matches_for_display.begin()+50, orb_matches_for_display.end());
        orb_matches_for_display.erase(orb_matches_for_display.begin()+51, orb_matches_for_display.end());
    }

    std::vector<cv::DMatch>good_orb_matches;
    good_orb_matches.insert(good_orb_matches.end(), orb_matches.begin(), orb_matches.end());
    if(good_orb_matches.size()> nth_element){
        std::nth_element(good_orb_matches.begin(), good_orb_matches.begin()+nth_element, good_orb_matches.end());
        good_orb_matches.erase(good_orb_matches.begin()+nth_element, good_orb_matches.end());
    }

    // 画出匹配结果图
    cv::Mat img_orb_matches__before_homography;
    cv::drawMatches(img1
            ,orb_keypts1
            ,img2
            ,orb_keypts2
            ,orb_matches_for_display
            ,img_orb_matches__before_homography
            ,cv::Scalar::all(-1) // 连接线的颜色
    );
    cv::imwrite("./orb_matching_before_homography.jpg", img_orb_matches__before_homography);


    // 利用homography 对相机对匹配点进行筛选
    std::cout<<"num of orb features before/after homography: "<< good_orb_matches.size()<<"--->";
    refine_matches_via_homography(orb_keypts1, orb_keypts2, good_orb_matches);
    std::cout<<good_orb_matches.size()<<std::endl;

    // 划除匹配结果图
    cv::Mat img_orb_matches_after_homography;
    cv::drawMatches(img1
            ,orb_keypts1
            ,img2
            ,orb_keypts2
            ,good_orb_matches
            ,img_orb_matches_after_homography
            ,cv::Scalar::all(-1) // 连接线的颜色
    );
    cv::imwrite("./orb_matching_after_homography.jpg", img_orb_matches_after_homography);



    return 0;
}
