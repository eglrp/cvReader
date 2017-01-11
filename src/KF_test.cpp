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

#include <thread>
#include <chrono>

#define DISTINCT_TIMES 18

#define PREDICT_TIME_STEP 20

int picth(150), yaw(800);

uint16_t dp(0),dy(0);
bool IsSetDelta(true);

double last_delta_p(0.0);
double last_delta_y(0.0);

//TODO: spline 1K,delta



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

void SetDelta()
{
    while (1) {
        if (IsSetDelta) {
            SetAngle(dp, dy);
            usleep(1000);
        }
    }

//    std::chrono::s

//    usleep(1000);
    return;
}

bool ImagePose2Angle(int p, int y) {

//    picth += (p/13.0);//up and down
//    yaw -= (y/13.0);//left and right

    /////picth
    double delta_p = (p/13.0);

    picth += delta_p*0.8 + delta_p-last_delta_p;


    last_delta_p = delta_p;




    ////yaw
    double delta_yaw = (y/13.0);

    yaw -= (delta_yaw * 0.8 + delta_yaw-last_delta_y);

    last_delta_y = delta_yaw;


    if(picth > 250)
    {
        picth = 250;
    }else if(picth < 50)
    {
        picth = 50;
    }
    if(yaw > 1400)
    {
        yaw = 1400;
    }
    if(yaw < 200)
    {
        yaw = 200;
    }

    uint16_t t_p,t_r;
    t_p = uint16_t(picth);
    t_r = uint16_t(yaw);

//    t_p = t_p / 10 * 10;
//    t_r = t_r /10 * 10;
    std::cout << "tp ty:"<< t_p << " " << t_r << std::endl;
    SetAngle(t_p,t_r);

    return true;
}

int main() {


    own::KalmanFilter<double, 4, 2> kf_test(5.0, 33.0, Eigen::Vector4d(5.0, 5.0, 10.0, 10.0));

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

    std::thread sendangle(SetDelta);
    sendangle.detach();


//    SerialControl serialControl("/dev/ttyUSB0");
//    serialControl.SetCameraCentre(src.rows/2,src.cols/2);

    if(capture.isOpened())
    {
        IsSetDelta= true;
    }

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

//                SetAngle(uint16_t(predict_state(0) - result.cols),
//                         uint16_t(predict_state(1) - result.rows));
//                ImagePose2Angle((predict_state(1) - result.cols/2),
//                                (predict_state(0) - result.rows/2));
                dp = predict_state(0) - result.cols / 2;
                dy = predict_state(1) - result.rows / 2;
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
//                SetAngle(uint16_t(predict_state(0) - result.cols),
//                         uint16_t(predict_state(1) - result.rows));
//                ImagePose2Angle((predict_state(1) - result.cols/2),
//                                (predict_state(0) - result.rows/2));

                dp = predict_state(0) - result.cols / 2;
                dy = predict_state(1) - result.rows / 2;
                cv::circle(result, cv::Point2f(predict_state(0), predict_state(1)),
                           20, cv::Scalar(20, 10, 200), 14);
            }

            ++dis_detect_num;

        }
        cv::namedWindow("result");


        cv::imshow("result", result);
        cv::waitKey(1);

    }


}