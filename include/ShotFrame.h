#pragma once
//
// Created by steve on 17-1-9.
//
/**
 *                             _ooOoo_
 *                            o8888888o
 *                            88" . "88
 *                            (| -_- |)
 *                            O\  =  /O
 *                         ____/`---'\____
 *                       .'  \\|     |//  `.
 *                      /  \\|||  :  |||//  \
 *                     /  _||||| -:- |||||-  \
 *                     |   | \\\  -  /// |   |
 *                     | \_|  ''\---/''  |   |
 *                     \  .-\__  `-`  ___/-. /
 *                   ___`. .'  /--.--\  `. . __
 *                ."" '<  `.___\_<|>_/___.'  >'"".
 *               | | :  `- \`.;`\ _ /`;.`/ - ` : | |
 *               \  \ `-.   \_ __\ /__ _/   .-` /  /
 *          ======`-.____`-.___\_____/___.-`____.-'======
 *                             `=---='
 *          ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
 *                     佛祖保佑        永无BUG
 *            佛曰:
 *                   写字楼里写字间，写字间里程序员；
 *                   程序人员写程序，又拿程序换酒钱。
 *                   酒醒只在网上坐，酒醉还来网下眠；
 *                   酒醉酒醒日复日，网上网下年复年。
 *                   但愿老死电脑间，不愿鞠躬老板前；
 *                   奔驰宝马贵者趣，公交自行程序员。
 *                   别人笑我忒疯癫，我笑自己命太贱；
 *                   不见满街漂亮妹，哪个归得程序员？
*/
#ifndef CVREADER_SHOTFRAME_H
#define CVREADER_SHOTFRAME_H


#include <iostream>
#include <chrono>
#include <thread>
#include <mutex>
#include <random>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include "ArCode.hpp"
#include "serialsom.h" // TODO: rewrite this.

//#include "ShotFrame.cpp"

#include "KalmanFilter.hpp"

/**
 * Simple class for test and debug sub-function.
 *
 */
class ShotFrame {
public:
    /**
     * Use to initial the frame.
     * @param video_id video device id,(example:"/dev/video1");
     * @param control_id serial device id ,the first serial device may be /dev/ttyUSB0.
     */
    ShotFrame(std::string video_id, std::string control_id);

    /**
     * Run, include three independent thread(VisionProcess,SerialBind & MachineControl).
     */
    void Run();




protected:

    ////////////////FUNCTION

    /**
     * A independence thread use to process image from camera and find the
     * pose of target in image.
     */
    void VisionProcess();

    /**
     * Bind to a serial device,accept and pre-process data.
     */
    void SerialBind();

    /**
     * Use data from VisionProcess and SerialBind control the motor.
     */
    void MachineControl();



    //////////////////DATA
    /////Very Important Flag
    bool IsRun = true;//All thread will exit if this flag equal to false.

//    /////thread
//    std::thread


    /////Exchange
//    std::atomic<int> delta_x_, delta_y_;//atomic data x,y offset from central point.
//    std::atomic<double> pitch_, yaw_; // Current pose of the orientation.
    int delta_x_, delta_y_;
    double pitch_, yaw_;

    std::mutex py_mutex_; //
    std::mutex serial_mutex_;

    /////SerialBind() and Machine Control()
    Serialport serial_handle_; // handle for serial com.

    /////VisionProcess
    cv::VideoCapture shot_cap_;
    own::KalmanFilter<double, 4, 2> kf_;

    cv::Mat in_mat_, out_mat_;

    long double last_time_ = 0.0;

    std::string win_name_ = "cv_debug";


    //Some important parameters.
    long double refresh_time_;

    double predict_time_step_;//





private:
    //Tool Function
    long double now() {
        auto tt = std::chrono::system_clock::now();
        auto t_nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(tt.time_since_epoch());

        double time_now(double(t_nanosec.count()) * 1e-9);
        return time_now;
    }


};

ShotFrame::ShotFrame(std::string video_id, std::string control_id) :
        serial_handle_(const_cast<char *>(control_id.c_str())),
        kf_(5.0, 33.0, Eigen::Vector4d(5.0, 5.0, 10.0, 10.0)) {

//    kf_ = own::KalmanFilter<double,4,2>;
    delta_x_ = 0;
    delta_y_ = 0;

    pitch_ = 0;
    yaw_ = 0;



    /**
     * Open device port.
     */
    //TODO: Neet a completed method to finish this open port process.
//    serial_handle_ = open(control_id.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
//    if (isatty(serial_handle_) == 0) {
//        std::cout << "Standard inpute is not a termianl device!" << std::endl;
//
//    }
    serial_handle_.open_port(const_cast<char *>(control_id.c_str()));//TODO: Just for test.

    shot_cap_.open(video_id);
    if (!shot_cap_.isOpened()) {
        cv::waitKey(100);
        shot_cap_.open(0);

    }

}

void ShotFrame::Run() {
    /**
    * Start thread.
    */

    std::cout << "Run." << std::endl;

    std::thread vp(&ShotFrame::VisionProcess, this);
    vp.detach();

    std::thread sb(&ShotFrame::SerialBind, this);
    sb.detach();

    std::thread mc(&ShotFrame::MachineControl, this);
    mc.detach();

    std::cout << "Run ok!" << std::endl;
}

void ShotFrame::VisionProcess() {

    cv::namedWindow(win_name_, cv::WINDOW_AUTOSIZE);
//    createTrackbar("t", , &t, 256, 0);
    int refresh_tmp_int(10);
    int time_length(20);

    cv::createTrackbar("refresh time", win_name_, &refresh_tmp_int, 1000, 0);
    cv::createTrackbar("time length", win_name_, &time_length, 1000, 0);

    //TODO: add predict time length.


    ArCodePort detect(cv::aruco::DICT_6X6_100);

    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners;
    while (shot_cap_.isOpened() && IsRun) {

        refresh_time_ = double(refresh_tmp_int) / 500.0;
        predict_time_step_ = double(time_length) / 500.0;

        shot_cap_ >> in_mat_;

        markerIds.clear();
        markerCorners.clear();


        if (detect.DetectMarkers(in_mat_, markerCorners, markerIds, &out_mat_) > 0) {

            cv::Point2f current_point = markerCorners.at(0).at(0);
            long double the_time(now());
            if (the_time - last_time_ > refresh_time_) {
                kf_.InitialState(
                        Eigen::VectorXd(Eigen::Vector4d(double(current_point.x), double(current_point.y), 0.0, 0.0))
                );
                out_mat_ = in_mat_;

            } else {
                Eigen::VectorXd state = kf_.OneStep(
                        Eigen::Vector2d(double(current_point.x), double(current_point.y)),
                        double(the_time - last_time_));
                cv::circle(out_mat_, cv::Point2f(state(0), state(1)), 20, cv::Scalar(20, 210, 20), 14);
            }


            last_time_ = the_time;


        } else {
            out_mat_ = in_mat_;
        }
//        std::cout << "now:"<<now() << std::endl;

        cv::imshow(win_name_, out_mat_);
        cv::waitKey(10);


    }

}


void ShotFrame::MachineControl() {
    sleep(2);//Sleep 2 seconds wait for VisionProcess run;

    while (IsRun) {
        Eigen::VectorXd pre_state = kf_.Predict(predict_time_step_);
//        serial_handle_.sendAngle()
        serial_handle_.usart3_send(pre_state(0) - in_mat_.cols / 2, pre_state(1) - in_mat_.rows / 2);
        delta_x_ = pre_state(0);
        delta_y_ = pre_state(1);
        usleep(1000);
    }
}

void ShotFrame::SerialBind() {
    while (1) {
        sleep(10);
    }
}

#endif //CVREADER_SHOTFRAME_H
