//
// Created by swayfreeda on 17-10-31.
//

#include <iostream>
#include <opencv/cv.hpp>
#include "robust_matcher.h"


cv::Ptr<RobustMatcher> RobustMatcher::create(float ratio)
{
    return cv::Ptr<RobustMatcher>(new RobustMatcher(ratio));
}

void RobustMatcher::oneway_match(const cv::Mat & descrs1, const cv::Mat & descrs2
        , std::vector<cv::DMatch>& matches)
{
    std::vector<std::vector<cv::DMatch> > knn_matches;
    cv::Ptr<cv::FlannBasedMatcher> flann_matcher=cv::FlannBasedMatcher::create();
    flann_matcher->add(descrs2);
    flann_matcher->train();
    // flann_matcher->match(sift_descrs1, sift_matches);
    clock_t time_begin = clock();
    flann_matcher->knnMatch(descrs1
            ,knn_matches
            ,2 /*最好的和次好的匹配 */
    );
    //clock_t  time_end = clock();
    //std::cout<<"knn matching time is "<<double(time_end-time_begin)/(double)CLOCKS_PER_SEC<<" s"<<std::endl;

    matches= lowe_ratio_test(knn_matches);
}

std::vector<cv::DMatch>  RobustMatcher::lowe_ratio_test(std::vector<std::vector<cv::DMatch> > & two_best_matches)
{

    int n_failed_matches = 0;

    std::vector<cv::DMatch> best_matches;

    for (std::vector<std::vector<cv::DMatch>>::iterator miter= two_best_matches.begin();
         miter!= two_best_matches.end(); ++miter)
    {
        // 检测是否是两个最近邻
        assert(miter->size()==2);

        // 检测lowe_ratio
        if ((*miter)[0].distance/ (*miter)[1].distance > lowe_ratio_*lowe_ratio_){

            n_failed_matches++;
            continue;
        }

        best_matches.push_back((*miter)[0]);
    }
    std::cout<< n_failed_matches<<" of "<<two_best_matches.size()<<" failed in lowe ratio test!"<<std::endl;
    return best_matches;
}

void RobustMatcher::remove_inconsistent_matches(std::vector<cv::DMatch>& sym_matches)
{

    //int n_inconsistent_matches = 0;

    sym_matches.clear();


    // 对所有results1_2_中的匹配对
    for (std::vector<cv::DMatch> ::const_iterator matchIterator1 = results1_2_.cbegin();
         matchIterator1!= results1_2_.end(); matchIterator1++)
    {

     // 对所有results2_1_中的匹配对
        for (std::vector<cv::DMatch>::
             const_iterator matchIterator2= results2_1_.cbegin();
             matchIterator2!= results2_1_.end(); matchIterator2++)
        {

            // 检测匹配对是否一致
            if (matchIterator1->queryIdx == matchIterator2->trainIdx &&
                matchIterator2->queryIdx == matchIterator1->trainIdx) {

                sym_matches.push_back(
                        cv::DMatch(matchIterator1->queryIdx,
                                   matchIterator1->trainIdx,
                                   matchIterator1->distance));
                break;
            }
        }
    }
    //std::cout<<n_inconsistent_matches<<" matchtes are detected!"<<std::endl;

}

/*
void RobustMatcher::refine_via_fundemental(const std::vector<cv::KeyPoint> & keypts1
        ,const std::vector<cv::KeyPoint> & keypts2
        ,std::vector<cv::DMatch> &matches){

    const int minimun_matches_allowed=8;
    assert(matches.size()>= minimun_matches_allowed);

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
    cv::Mat fundemental = cv::findFundamentalMat(pts1,pts2 ,inlier_mask, cv::FM_RANSAC, 2, 0.99);

    // 去掉外点匹配对
    std::vector<cv::DMatch> inlier_matches;
    for(int i=0; i<inlier_mask.size(); i++){
        if(inlier_mask[i])
            inlier_matches.push_back(matches[i]);
    }
    matches.swap(inlier_matches);
}
*/
 void RobustMatcher::match(const cv::Mat & descrs1
                         ,const cv::Mat& descrs2
                         ,std::vector<cv::DMatch>& matches)
   {
    oneway_match(descrs1,descrs2, results1_2_);
    oneway_match(descrs2,descrs1, results2_1_);
    remove_inconsistent_matches(matches);
    std::cout<<matches.size()<<"  consistent matches are detected!"<<std::endl;
    //refine_via_fundemental(keypts1, keypts2, matches);
    std::cout<<matches.size()<<"  inlier matches are detected!"<<std::endl;
}


