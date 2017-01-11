//
// Created by steve on 17-1-9.
//

#include "ShotFrame.h"


ShotFrame::ShotFrame(std::string video_id, std::string control_id) {
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
    serial_handle_.open_port(control_id.c_str());//TODO: Just for test.

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

    std::thread vp(this->VisionProcess);
    vp.detach();

    std::thread sb(this->SerialBind);
    sb.detach();

    std::thread mc(this->MachineControl);
    mc.detach();
}

void ShotFrame::VisionProcess() {

    cv::namedWindow(win_name_, cv::WINDOW_AUTOSIZE);
    createTrackbar("t", g_szTitle, &t, 256, 0);
    cv::createTrackbar("refresh time", win_name_, &refresh_time_, 1000, 0);


    ArCodePort detect(cv::aruco::DICT_6X6_100);

    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners;
    while (shot_cap_.isOpened()) {
        shot_cap_ >> in_mat_;

        markerIds.clear();
        markerCorners.clear();

        if (detect.DetectMarkers(in_mat, markerCorners, markerIds, &out_mat_) > 0) {

            cv::Point2f current_point = markerCorners.at(0).at(0);
            if (now() - last_time_ > refresh_time_) {
                kf_.InitialState(
                        Eigen::VectorXd(Eigen::Vector4d(double(current_point.x), double(current_point.y), 0.0, 0.0))
                );

            } else {
                Eigen::VectorXd state = kf_.OneStep(
                        Eigen::Vector2d(double(current_point.x), double(current_point.y)),
                        double(dis_detect_num + 1));
            }
        }


    }

}