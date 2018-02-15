#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

int main(int argc, char* argv[]) {

  cv::Mat img_src=cv::imread(argv[1]);

  if(argc<2){
      std::cerr<<"usage: features image_file_name"<<std::endl;
      return -1;
  }

  // 加载图像
  if(!img_src.data){
     std::cerr<<"usage: error to load data from "<<argv[2]<<std::endl;
      return -1;
  }
  std::cout<<"image size: "<<img_src.rows<<" x "<< img_src.cols<<std::endl;

  // 为了更好的显示，对图像进行下采样处理
//  cv::resize(img_src, img_src, cv::Size(img_src.cols*0.2, img_src.rows*0.2), 0.5, 0.5);

  // 设置需要检测的特征点的个数
  int n_features=4000;

    /*********************************Harris Corners*****************************/
  clock_t  time_begin=clock();
  // harris 角点检测
  cv::Ptr<cv::GFTTDetector> gftt_detector=cv::GFTTDetector::create(n_features /* maximum of number of keypoints 最大找到的特征点数量*/
                                                                    ,0.01 /* quality level */
                                                                    ,1   /* minimum distances between key points */
                                                                    ,3    /* block size 取窗口大小*/
                                                                    ,true /* use harris detector */
                                                                    ,0.04 /* the value of k in C=det(H) -k trace(H)^2 系数*/
                                                                    );

  // harris 关键点
  std::vector<cv::KeyPoint> harris_keypts;
  // 开始检测关键点
  gftt_detector->detect(img_src, harris_keypts);
  clock_t time_end=clock();
  std::cout<<harris_keypts.size()<<" harris corners are detected!"<<std::endl;
  std::cout<<"harris corners detection cost "<<double(time_end-time_begin)/(double)CLOCKS_PER_SEC<<" s"<<std::endl;

  // draw detected results
  cv::Mat img_harris_result;
  cv::drawKeypoints(img_src, harris_keypts, img_harris_result, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
  cv::imwrite("./harris_detected_result.jpg", img_harris_result);



  /***************************************SIFT************************************/
  time_begin=clock();
  // sift detector
  cv::Ptr<cv::xfeatures2d::SIFT> sift_detector = cv::xfeatures2d::SIFT::create(n_features //特征点的个数
                                                                               ,3         // octave的个数
                                                                               ,0.04      // 高斯差分值的阈值，大于该值认为是特征
                                                                               ,10        // 边缘响应阈值
                                                                               ,1.6       // 初始的图像尺度
                                                                               );
  //sift keypoints
  std::vector<cv::KeyPoint> sift_keypts;
  sift_detector->detect(img_src, sift_keypts);
  time_end=clock();
  std::cout<<sift_keypts.size()<<" sift keypoints are detected!"<<std::endl;
  std::cout<<"sift keypoints detection cost "<<double(time_end-time_begin)/(double)CLOCKS_PER_SEC<<" s"<<std::endl;

  // draw detected results
  cv::Mat img_sift_result;
  cv::drawKeypoints(img_src, sift_keypts, img_sift_result, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
  cv::imwrite("./sift_detected_result.jpg", img_sift_result);


  /****************************************FAST**********************************/
  time_begin=clock();
  cv::Ptr<cv::FastFeatureDetector> fast_detector= cv::FastFeatureDetector::create(50   // 用来比较圆上点和中心点像素阈值
                                                                                  ,true // 是否使用非计大值约束
                                                                                  ,cv::FastFeatureDetector::TYPE_9_16);
  //fast keypoints
  std::vector<cv::KeyPoint> fast_keypts;
  fast_detector->detect(img_src, fast_keypts);
  time_end=clock();
  std::cout<<fast_keypts.size()<<" fast keypoints are detected!"<<std::endl;
  std::cout<<"fast keypoints detection cost "<<double(time_end-time_begin)/(double)CLOCKS_PER_SEC<<" s"<<std::endl;

  // draw detected results
  cv::Mat img_fast_result;
  cv::drawKeypoints(img_src, fast_keypts, img_fast_result, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
  cv::imwrite("./fast_detected_result.jpg", img_fast_result);

  /****************************************ORB**********************************/
  time_begin=clock();
  cv::Ptr<cv::ORB> orb_detector = cv::ORB::create(n_features   /*输出特征点的个数 */
                                                  ,1.2f  /*尺度因子-用来构建图像金字塔*/
                                                  ,8     /*图像金字塔的层数*/
                                                  ,31
                                                  ,0
                                                  ,2
                                                  ,cv::ORB::HARRIS_SCORE
                                                  ,31
                                                  ,20
                                                  );

  std::vector<cv::KeyPoint> orb_keypts;
  orb_detector->detect(img_src, orb_keypts);
  time_end=clock();
  std::cout<<orb_keypts.size()<<" orb key pts are detected!"<<std::endl;
  std::cout<<"orb keypoints detection cost "<<double(time_end-time_begin)/(double)CLOCKS_PER_SEC<<" s"<<std::endl;

  cv::Mat img_orb_result;
  cv::drawKeypoints(img_src, orb_keypts, img_orb_result, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
  cv::imwrite("./orb_detected_result.jpg", img_orb_result);

  return 0;
}

