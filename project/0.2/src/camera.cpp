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

#include "myslam/camera.h"
#include "myslam/config.h"

namespace myslam
{

Camera::Camera()
{
    //读取配置
    fx_ = Config::get<float>("camera.fx");
    fy_ = Config::get<float>("camera.fy");
    cx_ = Config::get<float>("camera.cx");
    cy_ = Config::get<float>("camera.cy");
    depth_scale_ = Config::get<float>("camera.depth_scale");
}
//世界坐标到相机坐标的转换
Vector3d Camera::world2camera ( const Vector3d& p_w, const SE3& T_c_w )
{
    //p_c在相机坐标系下该店的位置
    //p_c = T_c_w * p_w （3.40）
    return T_c_w*p_w;
}

Vector3d Camera::camera2world ( const Vector3d& p_c, const SE3& T_c_w )
{
    //P_w在世界坐标系下该点的坐标
    //p_w = T_w_c * p_c （3.41）
    return T_c_w.inverse() *p_c;

}

Vector2d Camera::camera2pixel ( const Vector3d& p_c )
{
    //(5.5)相机坐标到像素坐标
    return Vector2d (
        fx_ * p_c ( 0,0 ) / p_c ( 2,0 ) + cx_,
        fy_ * p_c ( 1,0 ) / p_c ( 2,0 ) + cy_
    );
}

Vector3d Camera::pixel2camera ( const Vector2d& p_p, double depth )
{
    //(5.17)
    return Vector3d (
        ( p_p ( 0,0 )-cx_ ) *depth/fx_,
        ( p_p ( 1,0 )-cy_ ) *depth/fy_,
        depth
    );
}
//世界坐标系到像素坐标系
Vector2d Camera::world2pixel ( const Vector3d& p_w, const SE3& T_c_w )
{
    return camera2pixel ( world2camera ( p_w, T_c_w ) );
}


Vector3d Camera::pixel2world ( const Vector2d& p_p, const SE3& T_c_w, double depth )
{
    return camera2world ( pixel2camera ( p_p, depth ), T_c_w );
}


}
