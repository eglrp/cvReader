//
// Created by steve on 17-1-9.
//

#include "ShotFrame.h"


ShotFrame::ShotFrame(std::string video_id, std::string control_id) {
    delta_x_ = 0;
    delta_y_ = 0;

    pitch_ = 0;
    yaw_ = 0;


    //TODO: Neet a completed method to finish this open port process.
    serial_handle_ = open(control_id.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (isatty(serial_handle_) == 0) {
        std::cout << "Standard inpute is not a termianl device!" << std::endl;

    }



}