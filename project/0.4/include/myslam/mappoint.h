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

#ifndef MAPPOINT_H
#define MAPPOINT_H

#include "myslam/common_include.h"

namespace myslam
{
    
class Frame;

    //地图特征点
class MapPoint
{
public:
    typedef shared_ptr<MapPoint> Ptr;
    unsigned long      id_;        // 通过factory_id_ 进行赋值ID

    //静态成员变量，属于该类，而不是成员，用于记录ID号
    static unsigned long factory_id_;    // factory id
    bool        good_;      // wheter a good point 
    Vector3d    pos_;       // 世界坐标 Position in world
    Vector3d    norm_;      // Normal of viewing direction 
    Mat         descriptor_; // Descriptor for matching 
    
    list<Frame*>    observed_frames_;   // key-frames that can observe this point 
    
    int         matched_times_;     // being an inliner in pose estimation
    int         visible_times_;     // being visible in current frame 
    
    MapPoint();

    //带参数构造
    MapPoint( 
        unsigned long id, 
        const Vector3d& position, 
        const Vector3d& norm, 
        Frame* frame=nullptr, 
        const Mat& descriptor=Mat() 
    );
    //内联函数，在编译时进行展开，
    //const 修饰this 指针，不可以对该类的成员函数进行修改
    inline cv::Point3f getPositionCV() const
    {
        return cv::Point3f( pos_(0,0), pos_(1,0), pos_(2,0) );
    }

    //通过该函数进行构造
    static MapPoint::Ptr createMapPoint();
    //重载函数，相同函数名不同参数列表
    static MapPoint::Ptr createMapPoint( 
        const Vector3d& pos_world, 
        const Vector3d& norm_,
        const Mat& descriptor,
        Frame* frame );
};
}

#endif // MAPPOINT_H
