//
// Created by swayfreeda on 17-10-24.
//

#ifndef FEATURES_AND_MATCHING_FUNCTIONS_H
#define FEATURES_AND_MATCHING_FUNCTIONS_H

#include "robust_matcher.h"
#include <opencv/cv.hpp>


// 输入两幅图像，检测特征点并进行匹配
void features_and_matching(const cv::Mat &img1
                          ,const cv::Mat& img2
                          ,std::vector<cv::KeyPoint> &sift_keypts1
                          ,std::vector<cv::KeyPoint> &sift_keypts2
                          ,std::vector<cv::DMatch> &matches);

bool refine_matches_via_homography(const std::vector<cv::KeyPoint> & keypts1
                                  ,const std::vector<cv::KeyPoint> & keypts2
                                  ,std::vector<cv::DMatch> &matches);

bool refine_matches_via_fundemental(const std::vector<cv::KeyPoint> & keypts1
        ,const std::vector<cv::KeyPoint> & keypts2
        ,std::vector<cv::DMatch> &matches);

// @param p -- 图像上一点的像素坐标
// @param K -- 内参数矩阵
// @return  -- 归一化像平面上的坐标
cv::Point2d pixel2cam(const cv::Point2d & p, const cv::Mat& K);


/* 输入两幅图像的特征点和匹配关系，计算第二幅图像相对于第一幅图像的旋转矩阵和偏移矩阵
 * @param keypts1--输入第一幅图像上的特征点
 * @param keypts2--输入第二幅图像上的特征点
 * @param matches--输入特征点匹配
 * @param K--输入相机内参矩阵
 * @param R-- 输出估计的旋转矩阵
 * @param t-- 输出估计的平移向量
 */
void pose_estimation( const std::vector<cv::KeyPoint> keypts1
                     ,const std::vector<cv::KeyPoint> keypts2
                     ,const std::vector<cv::DMatch> &matches
                     ,const cv::Mat&K
                     ,cv::Mat&R
                     ,cv::Mat&t);



// 输入两幅图像的匹配点和相机参数，通过三角化得到匹配点的三维空间点
void triangulation(const std::vector<cv::KeyPoint> keypts1
        ,const std::vector<cv::KeyPoint> keypts2
        ,const std::vector<cv::DMatch> &matches
        ,const cv::Mat & K1
        ,const cv::Mat & R1
        ,const cv::Mat & t1
        ,const cv::Mat & K2
        ,const cv::Mat & R2
        ,const cv::Mat & t2
        ,std::vector<cv::Point3d> & pts3d);


// 构造相机投影矩阵
cv::Mat get_projection_matrix(const cv::Mat& K, const cv::Mat & R, const cv::Mat & t);


// 从捆绑调整优化结果获取相机的参数
// @params-- ptr存储相机优化参数， 维度是9，其中: 0-2 表示角轴法参数, 3-5 表示平移向量， 6-焦距， 7-8-径向畸变系数
void recover_camera_parames(const double* params, cv::Mat & R, cv::Mat & t, double& f, double & k1, double &k2);
#endif //FEATURES_AND_MATCHING_FUNCTIONS_H

