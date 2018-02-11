#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/opencv.hpp>


#include <Eigen/Core>
#include <Eigen/Geometry>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <chrono>

using namespace std;
using namespace cv;

void find_feature_matches (
    const Mat& img_1, const Mat& img_2,
    std::vector<KeyPoint>& keypoints_1,
    std::vector<KeyPoint>& keypoints_2,
    std::vector< DMatch >& matches );


Point2d pixel2cam ( const Point2d& p, const Mat& K )
{
    return Point2d
            (
                    ( p.x - K.at<double> ( 0,2 ) ) / K.at<double> ( 0,0 ),
                    ( p.y - K.at<double> ( 1,2 ) ) / K.at<double> ( 1,1 )
            );
}



int main ( int argc, char** argv )
{
    if ( argc != 5 )
    {
        cout<<" usage: pose_estimation_3d2d img1 img2 depth1 depth2"<<endl;
        return 1;
    }
    //-- 读取图像
    Mat img_1 = imread ( argv[1], CV_LOAD_IMAGE_COLOR );
    Mat img_2 = imread ( argv[2], CV_LOAD_IMAGE_COLOR );

    vector<KeyPoint> keypoints_1, keypoints_2;
    vector<DMatch> matches;
    find_feature_matches ( img_1, img_2, keypoints_1, keypoints_2, matches );

    cout<<"一共找到了"<<matches.size() <<"组匹配点"<<endl;


    // 建立3D点
    Mat d1 = imread ( argv[3], CV_LOAD_IMAGE_UNCHANGED );       // 深度图为16位无符号数，单通道图像
    Mat K = ( Mat_<double> ( 3,3 ) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1 );
    vector<Point3f> pts_3d;
    vector<Point2f> pts_2d;
    for ( DMatch m:matches )
    {
        //计算3D坐标
        ushort d = d1.ptr<unsigned short> (int ( keypoints_1[m.queryIdx].pt.y )) [ int ( keypoints_1[m.queryIdx].pt.x ) ];

        if ( d == 0 )   // bad depth
            continue;
        float dd = d/5000.0;
        //调用计数坐标函数
        Point2d p1 = pixel2cam ( keypoints_1[m.queryIdx].pt, K );

        pts_3d.push_back ( Point3f ( p1.x*dd, p1.y*dd, dd ) );
        pts_2d.push_back ( keypoints_2[m.trainIdx].pt );
    }

    cout<<"3d-2d pairs: "<<pts_3d.size() <<endl;

    Mat r, t;

    solvePnP ( pts_3d, pts_2d, K, Mat(), r, t, false ); // 调用OpenCV 的 PnP 求解，可选择EPNP，DLS等方法
    //solvePnPRansac ( pts_3d, pts_2d, K, Mat(), r, t, false );
    Mat R;
    //旋转向量到旋转矩阵
    cv::Rodrigues ( r, R ); // r为旋转向量形式，用Rodrigues公式转换为矩阵

    cout<<"R="<<endl<<R<<endl;
    cout<<"t="<<endl<<t<<endl;


    vector<cv::Point2f> next_keypoints;
    vector<cv::Point3f> prev_keypoints_3d;
    vector<cv::Point2f> prev_keypoints;
   // list< cv::Point2f > keypoints;      // 因为要删除跟踪失败的点，使用list
    vector<cv::KeyPoint> kps;
    cv::Ptr<cv::FastFeatureDetector> detector = cv::FastFeatureDetector::create();
    detector->detect( img_1, kps );
    for ( auto kp:kps )
    {
        prev_keypoints.push_back( kp.pt ); //将坐标放入keypoints链表中
    }


//    for ( auto kp:keypoints )
//    {
//        prev_keypoints.push_back(kp); //prev_keypoints 赋值为keypoints
//    }
    vector<unsigned char> status;
    vector<float> error;

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();

    cv::calcOpticalFlowPyrLK( img_1, img_2, prev_keypoints, next_keypoints, status, error );

    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>( t2-t1 );
    cout<<"LK Flow use time："<<time_used.count()<<" seconds."<<endl;

    cout<<"prev:"<<prev_keypoints.size() <<"  next:"<<next_keypoints.size()<<endl;

    vector<cv::KeyPoint> n_Keypoints;
    KeyPoint::convert(next_keypoints, n_Keypoints, 1, 1, 0, -1);


    Ptr<DescriptorMatcher> matcher  = DescriptorMatcher::create ( "BruteForce-Hamming" );
    vector<DMatch> match;
    // BFMatcher matcher ( NORM_HAMMING );
    matcher->match ( kps, n_Keypoints, match );




    // 把跟丢的点删掉，keypoint 放当前帧

//    int i=0;
//    for ( auto iter=keypoints.begin(); iter!=keypoints.end(); i++)
//    {
//        if ( status[i] == 0 ) //表示状态
//        {
//            iter = keypoints.erase(iter);
//            continue;
//        }
//        *iter = next_keypoints[i];
//        iter++;
//    }
//    for ( Point2f m:prev_keypoints )
//    {
//        //计算3D坐标
//        ushort d = d1.ptr<unsigned short> (int (m.y)) [ int (m.x) ];
//        if ( d == 0 )   // bad depth
//            continue;
//        float dd = d/5000.0;
//        //调用计数坐标函数
//        prev_keypoints_3d.push_back ( Point3f ( m.x*dd, m.y*dd, dd ) );
//    }
//
//
//    Mat r2, t22;
//
//    solvePnP ( prev_keypoints_3d, next_keypoints, K, Mat(), r2, t22, false ); // 调用OpenCV 的 PnP 求解，可选择EPNP，DLS等方法
//    //solvePnPRansac ( pts_3d, pts_2d, K, Mat(), r, t, false );
//    Mat R2;
//    //旋转向量到旋转矩阵
//    cv::Rodrigues ( r2, R2 ); // r为旋转向量形式，用Rodrigues公式转换为矩阵
//
//    cout<<"R="<<endl<<R2<<endl;
//    cout<<"t="<<endl<<t22<<endl;
    return 0;

}

void find_feature_matches ( const Mat& img_1, const Mat& img_2,
                            std::vector<KeyPoint>& keypoints_1,
                            std::vector<KeyPoint>& keypoints_2,
                            std::vector< DMatch >& matches )
{
    //-- 初始化
    Mat descriptors_1, descriptors_2;
    // used in OpenCV3
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    // use this if you are in OpenCV2
    // Ptr<FeatureDetector> detector = FeatureDetector::create ( "ORB" );
    // Ptr<DescriptorExtractor> descriptor = DescriptorExtractor::create ( "ORB" );
    Ptr<DescriptorMatcher> matcher  = DescriptorMatcher::create ( "BruteForce-Hamming" );
    //-- 第一步:检测 Oriented FAST 角点位置
    detector->detect ( img_1,keypoints_1 );
    detector->detect ( img_2,keypoints_2 );

    //-- 第二步:根据角点位置计算 BRIEF 描述子
    descriptor->compute ( img_1, keypoints_1, descriptors_1 );
    descriptor->compute ( img_2, keypoints_2, descriptors_2 );

    //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
    vector<DMatch> match;
    // BFMatcher matcher ( NORM_HAMMING );
    matcher->match ( descriptors_1, descriptors_2, match );

    //-- 第四步:匹配点对筛选
    double min_dist=10000, max_dist=0;

    //找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        double dist = match[i].distance;
        if ( dist < min_dist ) min_dist = dist;
        if ( dist > max_dist ) max_dist = dist;
    }

    printf ( "-- Max dist : %f \n", max_dist );
    printf ( "-- Min dist : %f \n", min_dist );

    //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        if ( match[i].distance <= max ( 2*min_dist, 30.0 ) )
        {
            matches.push_back ( match[i] );
        }
    }
}


