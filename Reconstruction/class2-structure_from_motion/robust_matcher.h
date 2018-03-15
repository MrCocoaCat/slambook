//
// Created by swayfreeda on 17-10-31.
//

#ifndef FEATURES_AND_MATCHING_ROBUST_MATCHER_H
#define FEATURES_AND_MATCHING_ROBUST_MATCHER_H

#include <opencv2/features2d.hpp>
#include <opencv2/core/cvstd.hpp>

class RobustMatcher{
public:

    RobustMatcher(float ratio=0.8):lowe_ratio_(ratio){}
    static cv::Ptr<RobustMatcher> create(float ratio=0.8);

    // 主程序，输入descriptors1和descriptors2之后进行knn_mathcer匹配，
    // 对匹配结果进行lowe_ratio_test和remove_inconsitent_matches操作，获取精度较高的匹配点
    void match(const cv::Mat & descrs1
              ,const cv::Mat& descrs2
              ,std::vector<cv::DMatch>& matches);
protected:

    // 对descriptors1中的每个特征点，从descriptors中找到与其匹配的特征点，即最近邻点。oneway_match完成的是单向匹配
    void oneway_match(const cv::Mat & descrs1, const cv::Mat & descrs2, std::vector<cv::DMatch>& matches);

    // lowe_ratio_test保证最近邻匹配点和次近邻匹配点的距离比小于给定值，以去除错误的匹配
    std::vector<cv::DMatch> lowe_ratio_test(std::vector<std::vector<cv::DMatch> > & two_best_matches);

    // 对results1_2_ 和 results2_1_进行处理，保证最终的匹配结果张，descriptors1中的特征点A匹配到descriptors2中的特征B
    // 同时descriptors2中的特征B与descriptors1中的特征A匹配
    void remove_inconsistent_matches(std::vector<cv::DMatch>& sym_matches);

    // 通过RANSAC求解本征矩阵进一步去除匹配外点
//    void refine_via_fundemental(const std::vector<cv::KeyPoint> & keypts1
//            ,const std::vector<cv::KeyPoint> & keypts2
//            ,std::vector<cv::DMatch> &matches);


private:

    // 最近邻居匹配点和次近邻匹配点之间的距离比要求小于lowe_ratio_,默认值是0.6
    float lowe_ratio_;

    // 将desciptors1匹配到descriptors2的结果, results1_2_.size()  == descriptors1.size()
    std::vector<cv::DMatch> results1_2_;

    // 将descriptors2匹配到descriptors1的结果， results2_1_.size() == descriptors2.size()
    std::vector<cv::DMatch> results2_1_;

};


#endif //FEATURES_AND_MATCHING_ROBUST_MATCHER_H
