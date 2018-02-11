#include <iostream>
#include <fstream>
#include <list>
#include <vector>
#include <chrono>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/tracking.hpp>
using namespace std;
int main( int argc, char** argv )
{


    string path_to_dataset ="../../data/";

    string associate_file = path_to_dataset + "/associate.txt";
    
    ifstream fin( associate_file ); //读文件
    if ( !fin ) 
    {
        cerr<<"I cann't find associate.txt!"<<endl;
        return 1;
    }
    
    string rgb_file, depth_file, time_rgb, time_depth;
    list< cv::Point2f > keypoints;      // 因为要删除跟踪失败的点，使用list
    cv::Mat color, depth, last_color;  //定义Mat 格式文件
    
    //for ( int index=0; index<100; index++ )
  //  {
        fin>>time_rgb>>rgb_file>>time_depth>>depth_file; //将文件内容读入字符串
        last_color = cv::imread( path_to_dataset+"/"+rgb_file ); //读rgd图像
        depth = cv::imread( path_to_dataset+"/"+depth_file, -1 );//读depth图像

        fin>>time_rgb>>rgb_file>>time_depth>>depth_file; //将文件内容读入字符串
        color = cv::imread( path_to_dataset+"/"+rgb_file ); //读rgd图像
        depth = cv::imread( path_to_dataset+"/"+depth_file, -1 );//读depth图像
       // if (index ==0 )
       // {
            // 对第一帧提取FAST特征点
            vector<cv::KeyPoint> kps;
            cv::Ptr<cv::FastFeatureDetector> detector = cv::FastFeatureDetector::create();
            detector->detect( color, kps );
            for ( auto kp:kps )
            {
                keypoints.push_back( kp.pt ); //将坐标放入keypoints链表中
            }
            //last_color = color;
            //continue;
   //     }
        if ( color.data==nullptr || depth.data==nullptr )
        {
           // continue;
        }
        // 对其他帧用LK跟踪特征点
        vector<cv::Point2f> next_keypoints; 
        vector<cv::Point2f> prev_keypoints;
        for ( auto kp:keypoints )
        {
            prev_keypoints.push_back(kp); //prev_keypoints 赋值为keypoints
        }
        vector<unsigned char> status;
        vector<float> error; 
        chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
        //调用calcOpticalFlowPyrLK函数,光流法函数,带金字塔
        //last_color第一个图像,color第二个图像,prev_keypoints第一个关键点,输出来的放在next_keypoints下一个关键点,status估计结果
        cv::calcOpticalFlowPyrLK( last_color, color, prev_keypoints, next_keypoints, status, error );

        chrono::steady_clock::time_point t2 = chrono::steady_clock::now();

        chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>( t2-t1 );

        cout<<"LK Flow use time："<<time_used.count()<<" seconds."<<endl;
        // 把跟丢的点删掉，keypoint 放当前帧
        int i=0;
        for ( auto iter=keypoints.begin(); iter!=keypoints.end(); i++)
        {
            if ( status[i] == 0 ) //表示状态
            {
                iter = keypoints.erase(iter);
                continue;
            }
            *iter = next_keypoints[i];
            iter++;


        }
        cout<<" prev:"<<prev_keypoints.size()<<" next:"<<next_keypoints.size()<<endl;
        for ( int i =0;i<prev_keypoints.size() ; i++)
        {

            cout<<"prev_keypoints"<<prev_keypoints[i]<<"  prev_keypoints"<<next_keypoints[i]<<endl;

        }

        cout<<"tracked keypoints: "<<keypoints.size()<<endl;
        if (keypoints.size() == 0)
        {
            cout<<"all keypoints are lost."<<endl;
            //break;
        }
        // 画出 keypoints
        cv::Mat img_show = color.clone();
        for ( auto kp:keypoints )
        {
            //用circle
            //cv::circle(img_show, kp, 10, cv::Scalar(0, 240, 0), 1);
            cv::circle(img_show, kp, 2, cv::Scalar(0, 240, 0), 2);
        }
        cv::imshow("corners", img_show);
        cv::waitKey(200);
        last_color = color;
   // }
    return 0;
}