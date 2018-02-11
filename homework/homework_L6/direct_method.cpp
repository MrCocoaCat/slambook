#include <opencv2/opencv.hpp>
#include <sophus/se3.h>
#include <Eigen/Core>
#include <vector>
#include <string>
#include <boost/format.hpp>
#include <pangolin/pangolin.h>

using namespace std;
using namespace Eigen;
using namespace cv;

typedef vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> VecVector2d;

// Camera intrinsics
// 内参
double fx = 718.856, fy = 718.856, cx = 607.1928, cy = 185.2157;
// 基线
double baseline = 0.573;
// paths
string left_file = "../left.png";
string disparity_file = "../disparity.png";
boost::format fmt_others("../%06d.png");    // other files

// useful typedefs
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 2, 6> Matrix26d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

// TODO implement this function
/**
 * pose estimation using direct method
 * @param img1
 * @param img2
 * @param px_ref
 * @param depth_ref
 * @param T21
 */
void DirectPoseEstimationMultiLayer(
        const cv::Mat &img1,
        const cv::Mat &img2,
        const VecVector2d &px_ref,
        const vector<double> depth_ref,
        Sophus::SE3 &T21
);

// TODO implement this function
/**
 * pose estimation using direct method
 * @param img1
 * @param img2
 * @param px_ref
 * @param depth_ref
 * @param T21
 */
void DirectPoseEstimationSingleLayer(
        const cv::Mat &img1,
        const cv::Mat &img2,
        const VecVector2d &px_ref,
        const vector<double> depth_ref,
        Sophus::SE3 &T21
);

// bilinear interpolation
inline float GetPixelValue(const cv::Mat &img, float x, float y) {
    uchar *data = &img.data[int(y) * img.step + int(x)];
    float xx = x - floor(x);
    float yy = y - floor(y);
    return float(
            (1 - xx) * (1 - yy) * data[0] +
            xx * (1 - yy) * data[1] +
            (1 - xx) * yy * data[img.step] +
            xx * yy * data[img.step + 1]
    );
}

int main(int argc, char **argv) {

    cv::Mat left_img = cv::imread(left_file, 0);
    cv::Mat disparity_img = cv::imread(disparity_file, 0);
   /* imshow("left_file",left_img);
    imshow("disparity",disparity_img);
    waitKey(0);*/

    // let's randomly pick pixels in the first image and generate some 3d points in the first image's frame
    cv::RNG rng;
    int nPoints = 1000;
    int boarder = 40;
    VecVector2d pixels_ref;
    vector<double> depth_ref;

    // generate pixels in ref and load depth data   这里面生成的数据是没有原始图像上面生成的随机点
    for (int i = 0; i < nPoints; i++) {
        int x = rng.uniform(boarder, left_img.cols - boarder);  // don't pick pixels close to boarder
        int y = rng.uniform(boarder, left_img.rows - boarder);  // don't pick pixels close to boarder
        int disparity = disparity_img.at<uchar>(y, x);
        double depth = fx * baseline / disparity; // you know this is disparity to depth
        depth_ref.push_back(depth);//参考帧的深度
        pixels_ref.push_back(Eigen::Vector2d(x, y));
    }
    //测试
//    cout<<"pixels_ref[0]: "<<pixels_ref[0].transpose()<<endl;

    // estimates 01~05.png's pose using this information
    Sophus::SE3 T_cur_ref;

    for (int i = 1; i < 6; i++) {  // 1~10//就前2张
        cv::Mat img = cv::imread((fmt_others % i).str(), 0);
     /*   imshow("img",img);
        waitKey(0); */
//         DirectPoseEstimationSingleLayer(left_img, img, pixels_ref, depth_ref, T_cur_ref);    // first you need to test single layer
        DirectPoseEstimationMultiLayer(left_img, img, pixels_ref, depth_ref, T_cur_ref);
    }
}

void DirectPoseEstimationSingleLayer(
        const cv::Mat &img1,
        const cv::Mat &img2,
        const VecVector2d &px_ref,
        const vector<double> depth_ref,
        Sophus::SE3 &T21
) {

    // parameters
    int half_patch_size = 4;
    int iterations = 100;

    double cost = 0, lastCost = 0;
    int nGood = 0;  // good projections
    VecVector2d goodProjection;

    for (int iter = 0; iter < iterations; iter++) {
        nGood = 0;
        goodProjection.clear();

        // Define Hessian and bias
        Matrix6d H = Matrix6d::Zero();  // 6x6 Hessian
        Vector6d b = Vector6d::Zero();  // 6x1 bias
        cost = 0 ;  //这个位置我加了cost= 0；
        for (size_t i = 0; i < px_ref.size(); i++) {

            // compute the projection in the second image
            // TODO START YOUR CODE HERE
            float u =0, v = 0;//uv就是I2中的投影坐标

            double P_pixel_x = px_ref[i][0];//P在I1的像素坐标
            double P_pixel_y = px_ref[i][1];

            double Z = depth_ref[i];
            double X = (P_pixel_x - cx)/fx*Z;//1（ref）的相机1坐标系下的空间点P
            double Y = (P_pixel_y - cy)/fy*Z;

            Vector3d p_rotated(X,Y,Z);      //坐标转换
            p_rotated = T21 * p_rotated;
            double p_rotated_x = p_rotated[0];
            double p_rotated_y = p_rotated[1];
            double p_rotated_z = p_rotated[2];
            u = fx * p_rotated_x / p_rotated_z + cx;
            v = fy * p_rotated_y / p_rotated_z + cy;
//            cout<<"u: "<<"  "<<u<<"v: "<<v<<endl;
            if(u<=0 || u>=img2.cols || v<=0 || v>=img2.rows)
                continue;
                nGood++;
                goodProjection.push_back(Eigen::Vector2d(u, v));

            // and compute error and jacobian
            for (int x = -half_patch_size; x < half_patch_size; x++)
                for (int y = -half_patch_size; y < half_patch_size; y++) {
                    double error =0;///////////////////////////////////
/*                    cout<<"///////////////////"<<endl;
                    cout<<"P_pixel_x+x: "<<P_pixel_x+x<<endl;
                    cout<<"P_pixel_x+y: "<<P_pixel_y+y<<endl;
                    cout<<"u+x: "<<u+x<<endl;
                    cout<<"u+y: "<<v+y<<endl;*///////
                    error = GetPixelValue(img1,P_pixel_x+x,P_pixel_y+y) - GetPixelValue(img2,u+x,v+y);
                    Matrix26d J_pixel_xi;   // pixel to \xi in Lie algebra
                    J_pixel_xi(0,0) = fx/p_rotated_z;
                    J_pixel_xi(0,1) = 0;
                    J_pixel_xi(0,2) = -fx * p_rotated_x/(p_rotated_z*p_rotated_z);
                    J_pixel_xi(0,3) = -fx * p_rotated_x*p_rotated_y/(p_rotated_z*p_rotated_z);
                    J_pixel_xi(0,4) = fx + fx * p_rotated_x * p_rotated_x/(p_rotated_z*p_rotated_z);
                    J_pixel_xi(0,5) = -fx*p_rotated_y/p_rotated_z;

                    J_pixel_xi(1,0) = 0;
                    J_pixel_xi(1,1) = fy/p_rotated_z;
                    J_pixel_xi(1,2) = -fy*p_rotated_y/(p_rotated_z*p_rotated_z);
                    J_pixel_xi(1,3) = -fy-fy*p_rotated_y*p_rotated_y/(p_rotated_z*p_rotated_z);
                    J_pixel_xi(1,4) = fy*p_rotated_x*p_rotated_y/(p_rotated_z*p_rotated_z);
                    J_pixel_xi(1,5) = fy*p_rotated_x/p_rotated_z;
/*                    J_pixel_xi(0,0) = fx/Z;
                    J_pixel_xi(0,1) = 0;
                    J_pixel_xi(0,2) = -fx * X/Z*Z;
                    J_pixel_xi(0,3) = -fx * X * Y/Z*Z;
                    J_pixel_xi(0,4) = fx + fx*X*X/Z*Z;
                    J_pixel_xi(0,5) = -fx*Y/Z;

                    J_pixel_xi(1,0) = 0;
                    J_pixel_xi(1,1) = fy/Z;
                    J_pixel_xi(1,2) = -fy*Y/Z*Z;
                    J_pixel_xi(1,3) = -fy-fy*Y*Y/Z*Z;
                    J_pixel_xi(1,4) = fy*X*Y/Z*Z;
                    J_pixel_xi(1,5) = fy*X/Z;*/
                    Eigen::Vector2d J_img_pixel;    // image gradients
                    J_img_pixel[0] = ( GetPixelValue(img2,u+x+1,v+y) - GetPixelValue(img2,u+x-1,v+y) )/2;
                    J_img_pixel[1] = ( GetPixelValue(img2,u+x,v+y+1) - GetPixelValue(img2,u+x,v+y-1) )/2;


                    // total jacobian
                    Vector6d J=-(J_img_pixel.transpose() * J_pixel_xi).transpose();//这是J的转置

                    H += J * J.transpose();//注意是 += 不是=
                    b += -error * J;
                    cost += error * error;

                }
            // TODO END YOUR CODE HERE
        }

        // solve update and put it into estimation
        // TODO START YOUR CODE HERE

        Vector6d update;
        update = H.ldlt().solve(b);
        T21 = Sophus::SE3::exp(update) * T21;
        //TODO END YOUR CODE HERE
//        cout<<"before /nGood:" <<cost<<endl;
        cost /= nGood;

        if (std::isnan(update[0])) {
            // sometimes occurred when we have a black or white patch and H is irreversible
            cout << "update is nan" << endl;
            break;
        }
        if (iter > 0 && cost > lastCost) {
            cout << "cost increased: " << cost << ", " << lastCost << endl;  ////////高翔的代码部分，为了快点解决问题注释掉的
            break;
        }
        lastCost = cost;
        cout << "cost = " << cost << ", good = " << nGood << endl;////////高翔的代码部分，为了快点解决问题注释掉的
    }
    cout << "good projection: " << nGood << endl;////////高翔的代码部分，为了快点解决问题注释掉的
    cout << "T21 = \n" << T21.matrix() << endl;

    // in order to help you debug, we plot the projected pixels here
    cv::Mat img1_show, img2_show;
    cv::cvtColor(img1, img1_show, CV_GRAY2BGR);
    cv::cvtColor(img2, img2_show, CV_GRAY2BGR);
    for (auto &px: px_ref) {
        cv::rectangle(img1_show, cv::Point2f(px[0] - 2, px[1] - 2), cv::Point2f(px[0] + 2, px[1] + 2),
                      cv::Scalar(0, 250, 0));
    }
    for (auto &px: goodProjection) {
        cv::rectangle(img1_show, cv::Point2f(px[0] - 2, px[1] - 2), cv::Point2f(px[0] + 2, px[1] + 2),
                      cv::Scalar(250, 0, 0));
    }
//    cv::imshow("reference", img1_show);////////高翔的代码部分，为了快点解决问题注释掉的
//    cv::imshow("current", img2_show);////////高翔的代码部分，为了快点解决问题注释掉的
//    cv::waitKey();////////高翔的代码部分，为了快点解决问题注释掉的
/*    Mat img2_multi;
    cv::cvtColor(img2, img2_multi, CV_GRAY2BGR);
    for (int i = 0; i < px_ref.size(); i++) {

            cv::circle(img2_multi, Point2f(goodProjection[i][0],goodProjection[i][1]), 2, cv::Scalar(0, 250, 0), 2);
            cv::line(img2_multi, Point2f(px_ref[i][0],px_ref[i][1]), Point2f(goodProjection[i][0],goodProjection[i][1]), cv::Scalar(0, 250, 0));

    }
    imshow("img2_multi",img2_multi);
    waitKey(0);*/
}

void DirectPoseEstimationMultiLayer(
        const cv::Mat &img1,
        const cv::Mat &img2,
        const VecVector2d &px_ref,
        const vector<double> depth_ref,
        Sophus::SE3 &T21
) {

    // parameters
    int pyramids = 4;
    double pyramid_scale = 0.5;
    double scales[] = {1.0, 0.5, 0.25, 0.125};

    // create pyramids
    vector<cv::Mat> pyr1, pyr2; // image pyramids
    // TODO START YOUR CODE HERE
        for(int level = 0;level<pyramids;level++)
        {
            Mat dst1,dst2;
            resize(img1,dst1,Size(0,0),scales[level],scales[level]);
            resize(img2,dst2,Size(0,0),scales[level],scales[level]);
            pyr1.push_back(dst1);
            pyr2.push_back(dst2);
//            imshow("dst1",dst1);
//            waitKey(0);
        }
    // TODO END YOUR CODE HERE

    double fxG = fx, fyG = fy, cxG = cx, cyG = cy;  // backup the old values
    for (int level = pyramids - 1; level >= 0; level--) {
        VecVector2d px_ref_pyr; // set the keypoints in this pyramid level
        for (auto &px: px_ref) {
            px_ref_pyr.push_back(scales[level] * px);
        }

        // TODO START YOUR CODE HERE
        // scale fx, fy, cx, cy in different pyramid levels
        fx = fxG*scales[level];
        fy = fyG*scales[level];
        cx = cxG*scales[level];
        cy = cyG*scales[level];
        // TODO END YOUR CODE HERE
        DirectPoseEstimationSingleLayer(pyr1[level], pyr2[level], px_ref_pyr, depth_ref, T21);
    }

}
