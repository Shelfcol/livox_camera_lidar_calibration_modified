#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <stdio.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <string>


#include "common.h"

using namespace std;

cv::Mat gray_img, src_img;
cv::RNG  random_number_generator;
string photo_path, output_name, intrinsic_path,photo_folder;
int end_mouse{0}; // 结束鼠标点击
vector<cv::Point2f> corners; // 保存鼠标点击的角点

void writeData(const string filename, const float x, const float y, uint mode) {
    ofstream outfile;
    static bool write_first = true;
    if(write_first){
        write_first = false;
        outfile = ofstream(filename.c_str(), ios_base::trunc); // 读取并清空文件夹
    } else
    {
        outfile = ofstream(filename.c_str(), ios_base::app); // 追加模式
    }
    if (!outfile) {
        cout << "Can not open the file: " << filename << endl;
        exit(0);
    }
    switch(mode) {
        case(0):
            outfile << "photo" << endl;
            outfile << "1" << endl;
            break;
        case(1):
            outfile << "2" << endl;
            break;
        case(2):
            outfile << "3" << endl;
            break;
        case(3):
            outfile << "4" << endl;
            break;
        default:
            cout << "[writeData] - Code error, unknown mode" << endl;
            exit(0);
    }
    outfile << float2str(x) << "        " << float2str(y) << endl;
}

void getParameters() {
    cout << "Get the parameters from the launch file" << endl;

    if (!ros::param::get("input_photo_path", photo_path)) {
        cout << "Can not get the value of input_photo_path" << endl;
        exit(1);
    }
    if (!ros::param::get("ouput_path", output_name)) {
        cout << "Can not get the value of ouput_path" << endl;
        exit(1);
    }
    if (!ros::param::get("intrinsic_path", intrinsic_path)) {
        cout << "Can not get the value of intrinsic_path" << endl;
        exit(1);
    }
    if (!ros::param::get("photo_folder", photo_folder)) {
        cout << "Can not get the value of photo_folder" << endl;
        exit(1);
    }
    
}


void OnMouse(int EVENT, int x, int y, int flags, void* userdata)
{
    cv::Mat hh = *(cv::Mat*)userdata;
    cv::Point p(x,y);
    switch(EVENT)
    {
        case cv::EVENT_LBUTTONDOWN:
        {
            // printf("b=%d\t", hh.at<cv::Vec3b>(p)[0]);
            // printf("g=%d\t", hh.at<cv::Vec3b>(p)[1]);
            // printf("r=%d\n", hh.at<cv::Vec3b>(p)[2]);
            printf("pixel (%d,%d)\n",x,y);
            corners.push_back(cv::Point2f(x,y));
            cv::circle(hh, p, 2, cv::Scalar(255),3);
            break;
        }
        case cv::EVENT_RBUTTONDOWN: // 结束坐标获取
        {
            end_mouse = 1;
            printf("end mouse\n");
            break;
        }
    }
}



// 遍历所有相同格式的文件file_path png
// vector<string> GetFiles(string file_path,const string& latter ="bmp")
// {
//     vector<string> pic_vec;
//     // file_path += "/*"; // 遍历所有文件
//     file_path += "/*."+latter;
//     long handle;
//     struct _finddata_t fileinfo;
//     handle = _findfirst(file_path.c_str(),&fileinfo);
//     if(handle == -1){return pic_vec;}
//     do{
//         pic_vec.emplace_back(fileinfo.name);
//     }while(!_findnext(handle,&fileinfo));

//     _findclose(handle);
//     return pic_vec;
// }

int main(int argc, char **argv) {
    ros::init(argc, argv, "cornerPhoto");
    getParameters();

    vector<float> intrinsic; // 内参
    getIntrinsic(intrinsic_path, intrinsic);
    vector<float> distortion; // 畸变系数 k1,k2,p1,p2,k3
    getDistortion(intrinsic_path, distortion);

    // set intrinsic parameters of the camera
    cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    cameraMatrix.at<double>(0, 0) = intrinsic[0];
    cameraMatrix.at<double>(0, 2) = intrinsic[2];
    cameraMatrix.at<double>(1, 1) = intrinsic[4];
    cameraMatrix.at<double>(1, 2) = intrinsic[5];

    // set radial distortion and tangential distortion
    cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64F);
    distCoeffs.at<double>(0, 0) = distortion[0];
    distCoeffs.at<double>(1, 0) = distortion[1];
    distCoeffs.at<double>(2, 0) = distortion[2];
    distCoeffs.at<double>(3, 0) = distortion[3];
    distCoeffs.at<double>(4, 0) = distortion[4];

    vector<string> filenames;
    GetFileNames(photo_folder,filenames);
    int idx = 0;
    for( ; idx<filenames.size();) {
        auto file = filenames[idx];
        end_mouse = 0;
        corners.clear();

        printf("reading file :%s\n",file.c_str());
        src_img = cv::imread(file);
        if(src_img.empty()) {  // use the file name to search the photo
            cout << "No Picture found by filename: " << photo_path << endl;
            return 0;
        }

        cv::Mat view, rview, map1, map2;
        cv::Size imageSize = src_img.size();
        cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(),cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0), imageSize, CV_16SC2, map1, map2);
        cv::remap(src_img, src_img, map1, map2, cv::INTER_LINEAR);  // correct the distortion


        // cv::namedWindow("source", CV_WINDOW_KEEPRATIO);
        cv::namedWindow("source");
        setMouseCallback("source",OnMouse,&src_img);

        while (!end_mouse)
        {
            cv::imshow("source", src_img);
            cv::waitKey(40);
        }
        cv::destroyWindow("source");

        /***********************************************************************************/
        if (corners.size()!=4) { // 如果觉得选择点选择得不好，就多点几个点，代码就会重新操作这张图片
            cout << "corners.size()!=4" << endl;
            continue;
        }
        cv::Size winSize = cv::Size(8, 8);
        cv::Size zerozone = cv::Size(-1, -1);
        cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 40, 0.001);
        
        // cv::namedWindow("output", CV_WINDOW_KEEPRATIO);
        cv::namedWindow("output");
        cv::cvtColor(src_img, gray_img, cv::COLOR_BGR2GRAY);
        cv::cornerSubPix(gray_img, corners, winSize, zerozone, criteria); // 根据角点获取亚像素

        cv::Mat result_img = src_img.clone();
        for(uint t = 0; t < corners.size(); ++t) {
            cv::circle(result_img, corners[t], 3, cv::Scalar(random_number_generator.uniform(0, 255), random_number_generator.uniform(0, 255), random_number_generator.uniform(0, 255)), 1, 8, 0);
            printf("(%.3f %.3f)", corners[t].x, corners[t].y);
            writeData(output_name, corners[t].x, corners[t].y, t);
        }
        cv::namedWindow("output");
        imshow("output", result_img);
        cv::waitKey(1000);
        cv::destroyWindow("output");
        ++idx;
    }
    return 0;
}







