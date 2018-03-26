
#include "functions.h"

bool refine_matches_via_homography(const std::vector<cv::KeyPoint> & keypts1
        ,const std::vector<cv::KeyPoint> &keypts2
        , std::vector<cv::DMatch> &matches)
{

    const int minimun_matches_allowed=8;
    if(matches.size()< minimun_matches_allowed) return false;

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
            ,cv::RANSAC
            ,3
            ,inlier_mask);

    std::vector<cv::DMatch> inlier_matches;
    for(int i=0; i<inlier_mask.size(); i++){
        if(inlier_mask[i])
            inlier_matches.push_back(matches[i]);
    }
    //std::cout<<"homography: "<<homography<<std::endl;

    matches.swap(inlier_matches);
    return true;
}

