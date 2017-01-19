//
// Created by steve on 17-1-17.
//

#include <opencv2/opencv.hpp>

#include "StereoVision.h"

#define Width 640
#define Height 480
#define MaxContourArea 450        // 面积大于该值的轮廓不是装甲的灯条
#define MinContourArea 20        // 面积小于该值的轮廓不是装甲的灯条
#define RED 0                    // 0代表红色
#define BLUE 1                    // 1代表蓝色
int main() {


    cv::VideoCapture capL("./v1/outputLeft.avi");
    cv::VideoCapture capR("./v1/outputRight.avi");
    std::string win_name("detector_test");
    std::string dis_name("distance_win");
    std::string dark_name("dark_win");
    std::string color_name("color_search");


    cv::namedWindow(win_name);
    cv::namedWindow(dis_name);
    cv::namedWindow(dark_name);
    cv::namedWindow(color_name);


    int threold_value(200);
    cv::createTrackbar("threold", win_name, &threold_value, 255, 0);

    int threold_dark(200);
    cv::createTrackbar("dark threold", dark_name, &threold_dark, 255, 0);
    /**
     *  camera matrix.
     */

    cv::Mat cameraMatrixL = (cv::Mat_<double>(3, 3) << 5.0484366501418896e+02, 0., 3.1825013014114188e+02,
            0., 5.0377604847756851e+02, 2.4937812035308363e+02, 0., 0., 1.);
    cv::Mat distCoeffL = (cv::Mat_<double>(1, 5) << 1.3939280830750547e-01, -5.8046874025808826e-01,
            6.3162331897060652e-03, -5.2155045805109441e-03, 1.1149066042238609e+00);

    cv::Mat cameraMatrixR = (cv::Mat_<double>(3, 3) << 5.0149028606911679e+02, 0., 3.7221361800666864e+02,
            0., 5.0061684086229377e+02, 2.4279999667360542e+02, 0., 0., 1.);
    cv::Mat distCoeffR = (cv::Mat_<double>(1, 5) << 9.5627057175548646e-02, -2.6355268585825314e-01,
            -3.4759260457199919e-03, -1.6600935943118656e-03, 1.9364700494030165e-01);

    cv::Mat T = (cv::Mat_<double>(3, 1) << -3.0166846668326889e+02, 5.9076636778872977e+00, -9.9926839177569837e-01);
    cv::Mat R = (cv::Mat_<double>(3, 3) << 9.9998251145554928e-01, 5.4010035812116402e-03, -2.4095525244369636e-03,
            -5.2847340658908329e-03, 9.9893213779210044e-01, 4.5898318835843048e-02,
            2.6548764387621594e-03, -4.5884782296744427e-02, 9.9894321079062109e-01);


    StereoVision stereoVision(cameraMatrixL, distCoeffL,
                              cameraMatrixR, distCoeffR,
                              T, R, Width, Height);


    cv::Mat imgL, imgR;
    cv::Mat grayL, grayR;
    cv::Mat distimg;
    cv::Mat darkimg;
//    cv::Mat hsvL,hsvR;

    cv::Mat hMat, sMat, vMat, lMat;
    hMat.create(cv::Size(640, 480), CV_8UC1);
    sMat.create(cv::Size(640, 480), CV_8UC1);
    vMat.create(cv::Size(640, 480), CV_8UC1);
    lMat.create(cv::Size(640, 480), CV_8UC1);
    cv::Mat hsvL[3] = {hMat, sMat, vMat};

    cv::Mat hslL[3] = {hMat, sMat, lMat};


    std::cout << cameraMatrixL << std::endl;
    std::cout << cameraMatrixR << std::endl;


    cv::Mat element;
    element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2, 2));

    while (capL.isOpened()) {


        capR >> imgR;
        capL >> imgL;

//        cv::remap(imgL, imgL, stereoVision.mapLx, stereoVision.mapLy, INTER_LINEAR);
//        cv::remap(imgR, imgR, stereoVision.mapRx, stereoVision.mapRy, INTER_LINEAR);
//        cv::cvtColor(imgL, grayL, cv::COLOR_BGR2GRAY);
//        cv::cvtColor(imgR, grayR, cv::COLOR_BGR2GRAY);
//
//
//        cv::cvtColor(imgL, imgL, cv::COLOR_BGR2HLS);
//        cv::split(imgL, hsvL);
//        cv::split(imgR, hslL);
//
//
//
//        cv::cvtColor(imgL, darkimg, cv::COLOR_BGR2GRAY);
//
//        darkimg = 255 - darkimg;
//        cv::threshold(darkimg, darkimg, double(threold_dark), 255, cv::THRESH_BINARY);
//
//        cv::threshold(grayL, grayL, double(threold_value), 255, cv::THRESH_BINARY);
//        cv::threshold(grayR, grayR, double(threold_value), 255, cv::THRESH_BINARY);
//
//        stereoVision.stereoMatch(grayL, grayR);
//
//        stereoVision.getDisparityImage(distimg);
//
//
////        cv::threshold(vMat,vMat,)
//        lMat = 255 - lMat;
//        cv::threshold(lMat, lMat, double(threold_dark), 255, cv::THRESH_BINARY);
//
//        cv::morphologyEx(lMat, lMat, cv::MORPH_OPEN, element);




        /**
         * All process in left image.
         */




//        cv::imshow(win_name, grayL);
//        cv::imshow(dis_name, distimg);
//        cv::imshow(dark_name, lMat);
        cv::waitKey(10);


    }
}
