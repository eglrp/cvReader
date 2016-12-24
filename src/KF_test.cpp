//Create by steve in 16-12-21 at 下午11:02
//
// Created by steve on 16-12-21.
//

#include <iostream>

#include <Eigen/Dense>

#include <chrono>

#include "KalmanFilter.hpp"

//#include "opencv2/opencv.hpp"


#include <random>

//#include "myHead.hpp"
#include <opencv2/opencv.hpp>

#include "opticalFlow.h"

#include <opencv2/aruco.hpp>

#include "ArCode.hpp"

//#include "SerialSimple.hpp"
#include "serialsom.h"

#define DISTINCT_TIMES 1000

#define PREDICT_TIME_STEP 20

int SetAngle(uint16_t picth_angle, uint16_t yaw_angle) {
    int fd;
    bool true_false;
    Serialport Serialport1("/dev/ttyUSB0");
    fd = Serialport1.open_port("/dev/ttyUSB0");
    Serialport1.set_opt(9600, 8, 'N', 1);
    for (int i = 0; i < 3; i++)
        true_false = Serialport1.usart3_send(picth_angle, yaw_angle);
    close(fd);
    return true_false;
}

int main() {


    own::KalmanFilter<double, 4, 2> kf_test(13.0, 3.0, Eigen::Vector4d(5.0, 5.0, 10.0, 10.0));

    ArCodePort ar_detect(cv::aruco::DICT_6X6_100);

    cv::Mat src, result;

    //////////////--------------------Open camera------------------
    cv::VideoCapture capture("/dev/video1");
    if (!capture.isOpened()) {
        std::cout << "not opened:" << std::endl;
        cv::waitKey(1000);
        capture.open(0);
        cv::waitKey(1000);
    } else {
        std::cout << "opened " << std::endl;
    }

    int dis_detect_num(DISTINCT_TIMES + 1);

    capture >> src;


//    SerialControl serialControl("/dev/ttyUSB0");
//    serialControl.SetCameraCentre(src.rows/2,src.cols/2);


/////-------------tracker-----
    while (capture.isOpened()) {
        capture >> src;

        cv::waitKey(10);
        result = src;

        vector<int> markerIds;
        vector<vector<Point2f> > markerCorners;

        std::cout << "distinct times : " << dis_detect_num << std::endl;

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
                        double(dis_detect_num + 1));


//                Eigen::VectorXd predict_state = kf_test.Predict(double(PREDICT_TIME_STEP));
                Eigen::VectorXd predict_state(state);
//                std::cout << predict_state.transpose() << std::endl;

                cv::circle(result, cv::Point2f(predict_state(0), predict_state(1)), 20, cv::Scalar(20, 210, 20), 14);
//                cv::circle(result,current_point,20,cv::Scalar(20,200,10),14);
                predict_state = kf_test.Predict(double(10));
                std::cout << predict_state.transpose() << std::endl;

                SetAngle(uint16_t(predict_state(0) - result.cols),
                         uint16_t(predict_state(1) - result.rows));
                cv::circle(result, cv::Point2f(predict_state(0), predict_state(1)), 20, cv::Scalar(20, 10, 200), 14);
//                SetAngle(10,280);

            }

            markerCorners.clear();
            markerCorners.clear();

            dis_detect_num = 0;

        } else {
            if (dis_detect_num < DISTINCT_TIMES) {
                Eigen::VectorXd predict_state = kf_test.Predict(double(dis_detect_num));
                std::cout << predict_state.transpose() << std::endl;

//                int16_t
                SetAngle(uint16_t(predict_state(0) - result.cols),
                         uint16_t(predict_state(1) - result.rows));

                cv::circle(result, cv::Point2f(predict_state(0), predict_state(1)), 20, cv::Scalar(20, 10, 200), 14);
            }

            ++dis_detect_num;

        }
        cv::namedWindow("result");


        cv::imshow("result", result);
        cv::waitKey(10);

    }


}