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

#include <opencv2/aruco.hpp>

#include "ArCode.hpp"

#define DISTINCT_TIMES 1000

#define PREDICT_TIME_STEP 20


int main() {
//    std::cout << CV_VERSION << std::endl;
//    std::default_random_engine e_;
//    std::normal_distribution<> normal_distribution(0, 3.0);
//
    own::KalmanFilter<double, 4, 2> kf_test(13.0, 3.0, Eigen::Vector4d(5.0, 5.0, 10.0, 10.0));
//
//    int j(0);
//    for (int i(1); i < 1000; ++i) {
//         j +=( 10+normal_distribution(e_));
//
//        double ti = double(i) + normal_distribution(e_);
//        double tj = double(j) + normal_distribution(e_);
//        Eigen::VectorXd tmp = kf_test.OneStep(Eigen::Vector2d(ti,
//                                                              tj));



    ArCodePort ar_detect(cv::aruco::DICT_6X6_100);


    cv::VideoCapture capture("/dev/video0");
    if (!capture.isOpened()) {
        std::cout << "not opened:" << std::endl;
        cv::waitKey(1000);
        capture.open(0);
        cv::waitKey(1000);
    } else {
        std::cout << "opened " << std::endl;
    }


    cv::Mat src, result;


    int dis_detect_num(DISTINCT_TIMES + 1);

    while (capture.isOpened()) {
        capture >> src;

        cv::waitKey(10);
        result = src;

        vector<int> markerIds;
        vector<vector<Point2f> > markerCorners;

        std::cout <<"distinct times : " << dis_detect_num << std::endl;

        if (ar_detect.DetectMarkers(src, markerCorners, markerIds, &result) > 0) {

            std::cout << markerCorners.size() << std::endl;
            cv::Point2f current_point = markerCorners.at(0).at(0);
            std::cout << "current point : " << current_point << std::endl;
            if (dis_detect_num > DISTINCT_TIMES) {
                kf_test.InitialState(
                        Eigen::VectorXd(Eigen::Vector4d(double(current_point.x), double(current_point.y), 0.0, 0.0)));
            } else {

                Eigen::VectorXd state = kf_test.OneStep(
                        Eigen::Vector2d(double(current_point.x), double(current_point.y)),
                        double(dis_detect_num+1));


//                Eigen::VectorXd predict_state = kf_test.Predict(double(PREDICT_TIME_STEP));
                Eigen::VectorXd predict_state(state);
//                std::cout << predict_state.transpose() << std::endl;

                cv::circle(result, cv::Point2f(predict_state(0), predict_state(1)), 20, cv::Scalar(20, 210, 20), 14);
//                cv::circle(result,current_point,20,cv::Scalar(20,200,10),14);
                predict_state = kf_test.Predict(double(10));
                std::cout << predict_state.transpose() << std::endl;

                cv::circle(result, cv::Point2f(predict_state(0), predict_state(1)), 20, cv::Scalar(20, 10, 200), 14);

            }

            markerCorners.clear();
            markerCorners.clear();

            dis_detect_num = 0;

        } else {
            if(dis_detect_num < DISTINCT_TIMES)
            {
                Eigen::VectorXd predict_state = kf_test.Predict(double(dis_detect_num));
                std::cout << predict_state.transpose() << std::endl;

                cv::circle(result, cv::Point2f(predict_state(0), predict_state(1)), 20, cv::Scalar(20, 10, 200), 14);
            }

            ++dis_detect_num;

        }
        cv::namedWindow("result");


        cv::imshow("result", result);
        cv::waitKey(10);


    }


}