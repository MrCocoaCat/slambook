//
// Created by swayfreeda on 17-10-24.
//
// FlannBaseMatcher 不支持Hamming距离，因此不能用于二进制描述子的距离计算， 如ORB

#include "functions.h"
#include <opencv2/xfeatures2d.hpp>
#include <iostream>


int main(int argc, char* argv[]) {

    if (argc < 3) {
        std::cerr << "usage: bfm_matcher matching img1, img2" << std::endl;
        return -1;
    }

    // 加载第一幅图像
    cv::Mat img1 = cv::imread(argv[1]);
    if (!img1.data) {
        std::cerr << "fail to load img1 from "<<argv[1] << std::endl;
        return -1;
    }
    std::cout << "img1 infos: " << img1.rows << " x " << img1.cols << std::endl;


    // 加载第二幅图像
    cv::Mat img2 = cv::imread(argv[2]);
    if (!img2.data) {
        std::cerr << "fail to load img2 from "<<argv[2] << std::endl;
        return -1;
    }
    std::cout << "img2 infos: " << img2.rows << " x " << img2.cols << std::endl;


    // 为了更好的显示图像，对图像进行降采样处理
    cv::resize(img1, img1, cv::Size(img1.cols * 0.2, img1.rows * 0.2), 0.5, 0.5);
    cv::resize(img2, img2, cv::Size(img2.cols * 0.2, img2.rows * 0.2), 0.5, 0.5);
    std::cout << "img1 infos after resize: " << img1.rows << " x " << img1.cols << std::endl;
    std::cout << "img2 infos after resize: " << img2.rows << " x " << img2.cols << std::endl;


    //设置需要检测的特征个数
    int n_features = 8000;

    // 进行sift特征的检测和匹配
    std::vector<cv::KeyPoint> sift_keypts1, sift_keypts2;
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


    std::vector<std::vector<cv::DMatch> > sift_matches;
    cv::Ptr<cv::FlannBasedMatcher> flann_matcher=cv::FlannBasedMatcher::create();
    flann_matcher->add(sift_descrs2);
    flann_matcher->train();
   // flann_matcher->match(sift_descrs1, sift_matches);
    clock_t time_begin = clock();
    flann_matcher->knnMatch( sift_descrs1
                            ,sift_matches
                            ,2 /*最好的和次好的匹配 */
                            );
    clock_t  time_end = clock();
    std::cout<<"knn matching time is "<<double(time_end-time_begin)/(double)CLOCKS_PER_SEC<<" s"<<std::endl;

    // lowe ratio test
    float lowe_ratio = 0.8;
    std::vector<cv::DMatch> good_sift_matches;
    for(int i=0; i<sift_matches.size(); i++){
        const cv::DMatch best_match = sift_matches[i][0];
        const cv::DMatch second_best_match = sift_matches[i][1];
        float dist_ratio_square = best_match.distance/second_best_match.distance;
        if(dist_ratio_square > lowe_ratio*lowe_ratio) continue;
        good_sift_matches.push_back(best_match);
    }
    std::cout<<"num of sift matches: "<< sift_matches.size()<<"--->"<< good_sift_matches.size()<<std::endl;

    // 选择前50个匹配结果进行显示
    std::vector<cv::DMatch>sift_matches_for_display;
    sift_matches_for_display.insert(sift_matches_for_display.end(), good_sift_matches.begin(), good_sift_matches.end());
    if(sift_matches_for_display.size()> 50){
        std::nth_element(sift_matches_for_display.begin(), sift_matches_for_display.begin()+50, sift_matches_for_display.end());
        sift_matches_for_display.erase(sift_matches_for_display.begin()+51, sift_matches_for_display.end());
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


    // 选择前百分之十的特征点进行homograpy检测
    int nth_element=std::max(int (0.1*n_features),500);
    if(good_sift_matches.size()> nth_element){
        std::nth_element(good_sift_matches.begin(), good_sift_matches.begin()+nth_element, good_sift_matches.end());
        good_sift_matches.erase(good_sift_matches.begin()+nth_element, good_sift_matches.end());
    }

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

    return 1;

}
