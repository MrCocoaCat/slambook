// Farneback dense optical flow calculate and show in Munsell system of colors  
// Author : Zouxy  
// Date   : 2013-3-15  
// HomePage : http://blog.csdn.net/zouxy09  
//http://blog.csdn.net/zouxy09/article/details/8683859
// Email  : zouxy09@qq.com  
  
// API calcOpticalFlowFarneback() comes from OpenCV, and this  
// 2D dense optical flow algorithm from the following paper:  
// Gunnar Farneback. "Two-Frame Motion Estimation Based on Polynomial Expansion".  
// And the OpenCV source code locate in ..\opencv2.4.3\modules\video\src\optflowgf.cpp  
  
#include <iostream>  
#include "opencv2/opencv.hpp"  
  
using namespace cv;  
using namespace std;  
  
#define UNKNOWN_FLOW_THRESH 1e9  
  
// Color encoding of flow vectors from:  
// http://members.shaw.ca/quadibloc/other/colint.htm  
// This code is modified from:  
// http://vision.middlebury.edu/flow/data/  
void makecolorwheel(vector<Scalar> &colorwheel)  
{  
    int RY = 15;  
    int YG = 6;  
    int GC = 4;  
    int CB = 11;  
    int BM = 13;  
    int MR = 6;  
  
    int i;  
  
    for (i = 0; i < RY; i++) colorwheel.push_back(Scalar(255,       255*i/RY,     0));  
    for (i = 0; i < YG; i++) colorwheel.push_back(Scalar(255-255*i/YG, 255,       0));  
    for (i = 0; i < GC; i++) colorwheel.push_back(Scalar(0,         255,      255*i/GC));  
    for (i = 0; i < CB; i++) colorwheel.push_back(Scalar(0,         255-255*i/CB, 255));  
    for (i = 0; i < BM; i++) colorwheel.push_back(Scalar(255*i/BM,      0,        255));  
    for (i = 0; i < MR; i++) colorwheel.push_back(Scalar(255,       0,        255-255*i/MR));  
}  
  
void motionToColor(Mat flow, Mat &color)  
{  
    if (color.empty())  
        color.create(flow.rows, flow.cols, CV_8UC3);  
  
    static vector<Scalar> colorwheel; //Scalar r,g,b  
    if (colorwheel.empty())  
        makecolorwheel(colorwheel);  
  
    // determine motion range:  
    float maxrad = -1;  
  
    // Find max flow to normalize fx and fy  
    for (int i= 0; i < flow.rows; ++i)   
    {  
        for (int j = 0; j < flow.cols; ++j)   
        {  
            Vec2f flow_at_point = flow.at<Vec2f>(i, j);  
            float fx = flow_at_point[0];  
            float fy = flow_at_point[1];  
            if ((fabs(fx) >  UNKNOWN_FLOW_THRESH) || (fabs(fy) >  UNKNOWN_FLOW_THRESH))  
                continue;  
            float rad = sqrt(fx * fx + fy * fy);  
            maxrad = maxrad > rad ? maxrad : rad;  
        }  
    }  
  
    for (int i= 0; i < flow.rows; ++i)   
    {  
        for (int j = 0; j < flow.cols; ++j)   
        {  
            uchar *data = color.data + color.step[0] * i + color.step[1] * j;  
            Vec2f flow_at_point = flow.at<Vec2f>(i, j);  
  
            float fx = flow_at_point[0] / maxrad;  
            float fy = flow_at_point[1] / maxrad;  
            if ((fabs(fx) >  UNKNOWN_FLOW_THRESH) || (fabs(fy) >  UNKNOWN_FLOW_THRESH))  
            {  
                data[0] = data[1] = data[2] = 0;  
                continue;  
            }  
            float rad = sqrt(fx * fx + fy * fy);  
  
            float angle = atan2(-fy, -fx) / CV_PI;  
            float fk = (angle + 1.0) / 2.0 * (colorwheel.size()-1);  
            int k0 = (int)fk;  
            int k1 = (k0 + 1) % colorwheel.size();  
            float f = fk - k0;  
            //f = 0; // uncomment to see original color wheel  
  
            for (int b = 0; b < 3; b++)   
            {  
                float col0 = colorwheel[k0][b] / 255.0;  
                float col1 = colorwheel[k1][b] / 255.0;  
                float col = (1 - f) * col0 + f * col1;  
                if (rad <= 1)  
                    col = 1 - rad * (1 - col); // increase saturation with radius  
                else  
                    col *= .75; // out of range  
                data[2 - b] = (int)(255.0 * col);  
            }  
        }  
    }  
}  
  
int main(int, char**)  
{  
    VideoCapture cap;  
    cap.open(0);  
    //cap.open("test_02.wmv");  
  
    if( !cap.isOpened() )
    {
        return -1;
    }

    Mat prevgray, gray, flow, cflow, frame;  
    namedWindow("flow", 1);  
  
    Mat motion2color;  
  
    for(;;)  
    {  
        double t = (double)cvGetTickCount();  //获取时间
  
        cap >> frame;  
        cvtColor(frame, gray, CV_BGR2GRAY);  //颜色转换
        imshow("original", frame);
        imshow("gray", gray);

        if( prevgray.data )  
        {  
            calcOpticalFlowFarneback(prevgray, gray, flow, 0.5, 3, 15, 3, 5, 1.2, 0);//
            //利用用Gunnar Farneback的算法计算全局性的稠密光流算法
            // _prev0：输入前一帧图像
            // _next0：输入后一帧图像
            // _flow0：输出的光流
            // pyr_scale：金字塔上下两层之间的尺度关系
            // levels：金字塔层数
            // winsize：均值窗口大小，越大越能denoise并且能够检测快速移动目标，但会引起模糊运动区域
            // iterations：迭代次数
            // poly_n：像素领域大小，一般为5，7等
            // poly_sigma：高斯标注差，一般为1-1.5
            // flags：计算方法。主要包括OPTFLOW_USE_INITIAL_FLOW和OPTFLOW_FARNEBACK_GAUSSIAN
            motionToColor(flow, motion2color);//调用显示函数，不同颜色表示不同的运动方向，深浅就表示运动的快慢了。

            imshow("flow", motion2color);  
        }  
        if(waitKey(100)>=0)//等待按键
        {
            break;
        }

        std::swap(prevgray, gray);  
  
        t = (double)cvGetTickCount() - t;  //获取时间
        cout << "cost time: " << t / ((double)cvGetTickFrequency()*1000.) << endl;  
    }  
    return 0;  
}





//// OpenCV中此函数源码
//void cv::calcOpticalFlowFarneback( InputArray _prev0, InputArray _next0,
//                                   InputOutputArray _flow0, double pyr_scale, int levels, int winsize,
//                                   int iterations, int poly_n, double poly_sigma, int flags )
//{
//#ifdef HAVE_OPENCL
//    bool use_opencl = ocl::useOpenCL() && _flow0.isUMat();
//    if( use_opencl && ocl_calcOpticalFlowFarneback(_prev0, _next0, _flow0, pyr_scale, levels, winsize, iterations, poly_n, poly_sigma, flags))
//    {
//        CV_IMPL_ADD(CV_IMPL_OCL);
//        return;
//    }
//#endif
//    // 将_prev0和_next0转换为Mat类型
//    Mat prev0 = _prev0.getMat(), next0 = _next0.getMat();
//    const int min_size = 32;
//    // 创建img指针数组，img[0]指向prev0，img[1]指向next0
//    const Mat* img[2] = { &prev0, &next0 };
//
//    int i, k;
//    double scale;
//    Mat prevFlow, flow, fimg;
//    // 检查prev0和next0是否大小相同、单通道图像，金字塔尺度关系pyr_scale小于1
//    CV_Assert( prev0.size() == next0.size() && prev0.channels() == next0.channels() &&
//               prev0.channels() == 1 && pyr_scale < 1 );
//    // 创建和prev0大小相同，32位浮点双通道图像
//    _flow0.create( prev0.size(), CV_32FC2 );
//    // 将_flow0转换成Mat类型
//    Mat flow0 = _flow0.getMat();
//    // 循环确定金字塔层数
//    for( k = 0, scale = 1; k < levels; k++ )
//    {
//        // scale用于存放第k层图像与原图的尺寸广西
//        scale *= pyr_scale;
//        // 判断第k层图像行数与min_size关系，确定金字塔层数结束
//        if( prev0.cols*scale < min_size || prev0.rows*scale < min_size )
//            break;
//    }
//    // 将计算出的金字塔层数k赋给levels
//    levels = k;
//    // 遍历金字塔层数
//    for( k = levels; k >= 0; k-- )
//    {
//        // 计算原图与k-1层图像的尺寸关系
//        for( i = 0, scale = 1; i < k; i++ )
//            scale *= pyr_scale;
//        // 定义高斯滤波系数
//        double sigma = (1./scale-1)*0.5;
//        int smooth_sz = cvRound(sigma*5)|1;
//        // 得到高斯滤波器模板大小
//        smooth_sz = std::max(smooth_sz, 3);
//        // 计算第k层图像矩阵的列数
//        int width = cvRound(prev0.cols*scale);
//        // 计算第k层图像矩阵的行数
//        int height = cvRound(prev0.rows*scale);
//
//        if( k > 0 )
//            // 创建第k层图像尺寸大小的32位双通道图像，用于存储第k层图像光流flow
//            flow.create( height, width, CV_32FC2 );
//        else
//            // 否则为原图像
//            flow = flow0;
//        // 如果preFlow未指向任何矩阵数据
//        if( prevFlow.empty() )
//        {
//            // 如果flags为OPTFLOW_USE_INITIAL_FLOW
//            if( flags & OPTFLOW_USE_INITIAL_FLOW )
//            {
//                // 改变flow0图像大小为flow，用像素关系重采样插值
//                // 插值使用錓NTER_AREA，主要是为了避免波纹出现
//                resize( flow0, flow, Size(width, height), 0, 0, INTER_AREA );
//                // 将flow缩小scale
//                flow *= scale;
//            }
//                // flags为OPTFLOW_FARNEBACK_GAUSSIAN
//            else
//                // 创建一个Mat给flow
//                flow = Mat::zeros( height, width, CV_32FC2 );
//        }
//        else
//        {
//            // 改变prevFlow图像大小为flow，利用INTER_LINEAR方式进行双线性插值
//            resize( prevFlow, flow, Size(width, height), 0, 0, INTER_LINEAR );
//            // 将flow增加(1./pyr_sacle)倍
//            flow *= 1./pyr_scale;
//        }
//
//        Mat R[2], I, M;
//        for( i = 0; i < 2; i++ )
//        {
//            // 将img[i]转换为CV_32F格式
//            img[i]->convertTo(fimg, CV_32F);
//            // 对输出图像fimg进行高斯滤波后用fimg输出
//            GaussianBlur(fimg, fimg, Size(smooth_sz, smooth_sz), sigma, sigma);
//            // 改变fimg图像大小I,使用双线性插值INTER_LINEAR
//            resize( fimg, I, Size(width, height), INTER_LINEAR );
//            // 计算邻域图像R[i]
//            FarnebackPolyExp( I, R[i], poly_n, poly_sigma );
//        }
//        // 依据R[0]、R[1]、flow等信息更新矩阵M
//        FarnebackUpdateMatrices( R[0], R[1], flow, M, 0, flow.rows );
//
//        for( i = 0; i < iterations; i++ )
//        {
//            // flags为OPTFLOW_FARNEBACK_GAUSSIAN
//            if( flags & OPTFLOW_FARNEBACK_GAUSSIAN )
//                // 利用R[0]、R[1]、M等，利用高斯平滑原理，更新光流flow
//                FarnebackUpdateFlow_GaussianBlur( R[0], R[1], flow, M, winsize, i < iterations - 1 );
//                // flags为OPTFLOW_USE_INITIAL_FLOW
//            else
//                // 利用R[0]、R[1]、M等初始化光流flow
//                FarnebackUpdateFlow_Blur( R[0], R[1], flow, M, winsize, i < iterations - 1 );
//        }
//        // 将flow赋给prevFlow
//        prevFlow = flow;
//    }
//}