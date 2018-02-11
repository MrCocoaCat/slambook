/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  <copyright holder> <email>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef VISUALODOMETRY_H
#define VISUALODOMETRY_H

#include "myslam/common_include.h"
#include "myslam/map.h"

#include <opencv2/features2d/features2d.hpp>

namespace myslam 
{
class VisualOdometry
{
public:
    typedef shared_ptr<VisualOdometry> Ptr;
    enum VOState {
        INITIALIZING=-1, //初始化
        OK=0, //正常
        LOST  //丢失
    };
    //VOState 是枚举 VO状态
    VOState     state_;     // current VO status
    //map 类的指针，放关键帧
    Map::Ptr    map_;       // map with all frames and map points

    //参考帧
    Frame::Ptr  ref_;       // reference frame
    //当前帧
    Frame::Ptr  curr_;      // current frame 
    //orb特征
    cv::Ptr<cv::ORB> orb_;  // orb detector and computer
    //参考帧的3d点
    vector<cv::Point3f>     pts_3d_ref_;        // 3d points in reference frame
    //当前帧的关键点
    vector<cv::KeyPoint>    keypoints_curr_;    // keypoints in current frame
    //当前帧的描述
    Mat                     descriptors_curr_;  // descriptor in current frame
    //参考帧的描述
    Mat                     descriptors_ref_;   // descriptor in reference frame
    //挑选之后的匹配点
    vector<cv::DMatch>      feature_matches_;
    //当前帧的状态估计
    SE3 T_c_r_estimated_;  // the estimated pose of current frame
    //
    int num_inliers_;        // number of inlier features in icp
    int num_lost_;           // number of lost times
    
    // parameters
    //特征数量
    int num_of_features_;   // number of features
    //
    double scale_factor_;   // scale in image pyramid
    //金字塔层数
    int level_pyramid_;     // number of pyramid levels
    //选取比例
    float match_ratio_;      // ratio for selecting  good matches
    int max_num_lost_;      // max number of continuous lost times
    //最小的链接数量
    int min_inliers_;       // minimum inliers
    
    double key_frame_min_rot;   // minimal rotation of two key-frames
    double key_frame_min_trans; // minimal translation of two key-frames
    
public: // functions

    VisualOdometry();
    ~VisualOdometry();
    
    bool addFrame( Frame::Ptr frame );      // add a new frame 
    
protected:  
    // inner operation 
    void extractKeyPoints(); //计算当前帧角点
    void computeDescriptors();  //计算当前帧的描述子
    void featureMatching();
    void poseEstimationPnP(); 
    void setRef3DPoints();
    
    void addKeyFrame();
    bool checkEstimatedPose(); 
    bool checkKeyFrame();
    
};
}

#endif // VISUALODOMETRY_H