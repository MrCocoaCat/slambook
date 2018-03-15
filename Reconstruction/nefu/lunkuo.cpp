#include<iostream>  
#include<opencv2/opencv.hpp>  
#include<vector>  
  
using namespace cv;  
using namespace std;  
  
int main()  
{  
    Mat srcImage = imread("group.jpg");  
    imshow("【原图】", srcImage);  
  
    //首先对图像进行空间的转换  
    Mat grayImage;  
    cvtColor(srcImage, grayImage, CV_BGR2GRAY);  
    //对灰度图进行滤波  
    GaussianBlur(grayImage, grayImage, Size(3, 3), 0, 0);  
    imshow("【滤波后的图像】", grayImage);  
  
    //为了得到二值图像，对灰度图进行边缘检测  
    Mat cannyImage;  
    Canny(grayImage, cannyImage, 128, 255, 3);  
    //在得到的二值图像中寻找轮廓  
    vector<vector<Point>> contours;  
    vector<Vec4i> hierarchy;  
    findContours(cannyImage, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));  
  
    //绘制轮廓  
    for (int i = 0; i < (int)contours.size(); i++)  
    {  
        drawContours(cannyImage, contours, i, Scalar(255), 1, 8);  
    }  
    imshow("【处理后的图像】", cannyImage);  
  
    //计算轮廓的面积  
    for (int i = 0; i < (int)contours.size(); i++)  
    {  
        double g_dConArea = contourArea(contours[i], true);  
        cout << "【用轮廓面积计算函数计算出来的第" << i << "个轮廓的面积为：】" << g_dConArea << endl;  
    }  
  
    waitKey(0);  
  
    return 0;  
}  