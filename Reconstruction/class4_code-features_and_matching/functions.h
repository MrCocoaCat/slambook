//
// Created by swayfreeda on 17-10-24.
//

#ifndef FEATURES_AND_MATCHING_FUNCTIONS_H
#define FEATURES_AND_MATCHING_FUNCTIONS_H
#include <opencv/cv.hpp>

bool refine_matches_via_homography(const std::vector<cv::KeyPoint> & keypts1
                                  ,const std::vector<cv::KeyPoint> & keypts2
                                  ,std::vector<cv::DMatch> &matches);

#endif //FEATURES_AND_MATCHING_FUNCTIONS_H
