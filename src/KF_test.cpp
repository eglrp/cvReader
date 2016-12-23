//Create by steve in 16-12-21 at 下午11:02
//
// Created by steve on 16-12-21.
//

#include <iostream>

#include <Eigen/Dense>

#include <chrono>

#include "KalmanFilter.hpp"

#include "opencv2/opencv.hpp"

#include "opencv2/opencv_modules.hpp"

#include <random>

#include "myHead.hpp"
#include "opticalFlow.h"


int main() {
//    std::cout << CV_VERSION << std::endl;
//    std::default_random_engine e_;
//    std::normal_distribution<> normal_distribution(0, 3.0);
//
//    KalmanFilter<double, 4, 2> kf_test(3.0, 3.0, Eigen::Vector4d(5.0, 2.0, 1.0, 1.0));
//
//    int j(0);
//    for (int i(1); i < 1000; ++i) {
//         j +=( 10+normal_distribution(e_));
//
//        double ti = double(i) + normal_distribution(e_);
//        double tj = double(j) + normal_distribution(e_);
//        Eigen::VectorXd tmp = kf_test.OneStep(Eigen::Vector2d(ti,
//                                                              tj));
//
//
//        std::cout << "i,j is :" << i << "," << j <<
//                  " ti,tj: " <<
//                  ti << "," << tj << std::endl;
//        std::cout << tmp.transpose() << std::endl;
//
//    }

    std::cout <<"here1" << std::endl;
    cv::VideoCapture capture("/dev/video0");
    std::cout << "here2" << std::endl;
    if(!capture.isOpened())
    {
        std::cout << "not opened:"<< std::endl;
        cv::waitKey(1000);
        capture.open(0);
        cv::waitKey(1000);
    }else{
        std::cout << "opened " << std::endl;
    }

    OpticalFlow droneOF(50,0.01,10);

    cv::Mat src,result;

    while(capture.isOpened())
    {
        capture >> src;
        cv::waitKey(10);
//
//        droneOF.tracking(src);
//        result = droneOF.output;

        cv::imshow("result",src);
        cv::waitKey(10);




    }




}