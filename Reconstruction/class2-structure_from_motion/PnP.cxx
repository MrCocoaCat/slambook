//
// Created by swayfreeda on 17-11-2.
//
#include "functions.h"
#include "robust_matcher.h"

#include <opencv2/core.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <fstream>

void get_all_infos(const char* file_name
                  ,std::string& img_file_name
                  ,cv::Mat& K
                  ,cv::Mat& R
                  ,cv::Mat& t
                  ,std::vector<cv::Point3d>&pts3d
                  ,std::vector<cv::Point2d>&pts2d){

    std::ifstream in_file(file_name);
    assert(in_file.is_open());
    std::string line;
    int n_lines = 0;
    while(getline(in_file, line))
	{
        std::stringstream stream(line);

        if(n_lines==0)
		{
            stream>>img_file_name;
            n_lines++;
            continue;
        }
        if(n_lines==1)
		{
            // get K
            for(int i=0; i<3; i++){
                for(int j=0; j<3; j++){
                    stream>> K.at<double>(i,j);
                }
            }
            n_lines++;
            continue;
        }
        if(n_lines==2)
		{
            // get R
            for(int i=0; i<3; i++){
                for(int j=0; j<3; j++){
                    stream>> R.at<double>(i,j);
                }
            }
            n_lines++;
            continue;
        }
        if(n_lines==3)
		{
            // get t
            for(int i=0; i<3; i++){
                stream>> t.at<double>(i);
            }
            n_lines++;
            continue;
        }
        if(n_lines>3)
		{
            cv::Point3d pt3d;
            stream>>pt3d.x>>pt3d.y>>pt3d.z;
            cv::Point2d pt2d;
            stream>>pt2d.x>>pt2d.y;
            pts3d.push_back(pt3d);
            pts2d.push_back(pt2d);
            n_lines++;
            continue;
        }
    }

}


int main(int argc, char *argv[])
{
    if(argc<2){
        std::cerr<<"usage: pnp 3d2dcoresspondence"<<std::endl;
        return 0;
    }

    std::string file_name;
    cv::Mat K(3,3, CV_64FC1);
    cv::Mat R_true(3,3, CV_64FC1);
    cv::Mat t_true(3,1, CV_64FC1);

    std::vector<cv::Point3d> pts3d;
    std::vector<cv::Point2d> pts2d;
    get_all_infos(argv[1],file_name, K, R_true, t_true, pts3d, pts2d);

    std::cout<<" img file name: "<<std::endl<<file_name<<std::endl;
    std::cout<<" K is "<<std::endl<<K<<std::endl;
    std::cout<<" R is "<<std::endl<<R_true<<std::endl;
    std::cout<<" t is "<<std::endl<<t_true<<std::endl;
    for(int i=0; i< pts3d.size(); i++){
        std::cout<<pts3d[i].x<<" "<<pts3d[i].y<<" "<<pts3d[i].z<<" "<<pts2d[i].x<<" "<<pts2d[i].y<<std::endl;
    }

    // 加载第一幅图像
    cv::Mat img=cv::imread(file_name.c_str());
    if(!img.data){
        std::cerr<<"fail to load img"<<std::endl;
        return -1;
    }
    std::cout<<"img infos: "<< img.rows<<" x "<<img.cols<<std::endl;


    float ratiox = K.at<double>(0,2)/ img.cols;
    float ratioy = K.at<double>(1,2)/ img.rows;
    // 为了更好的显示图像，对图像进行降采样处理
    cv::resize(img, img, cv::Size(img.cols*ratiox, img.rows*ratioy), 0.5, 0.5);
    std::cout<<"img2 infos after resize: "<< img.rows<<" x "<<img.cols<<std::endl;


    /****************************pose estimation via EPnP(effient pnp)**************/
    cv::Mat r, t_estimated;
    cv::solvePnP(pts3d, pts2d, K, cv::Mat(), r, t_estimated, false, cv::SOLVEPNP_EPNP);
    cv::Mat R_estimated;
    cv::Rodrigues(r, R_estimated);
    std::cout<<" R ground truth: "<<std::endl<<R_true<<std::endl;
    std::cout<<" R estimated: "<<std::endl<< R_estimated<<std::endl;
    std::cout<<" t ground truth: "<<std::endl<<t_true<<std::endl;
    std::cout<<" t estimated: "<<std::endl<< t_estimated<<std::endl;

    return 0;
}

