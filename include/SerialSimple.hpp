#pragma once
//Create by steve in 16-12-21 at 下午11:02
//
// Created by steve on 16-12-21.
//

#ifndef CVREADER_SERIALSIMPLE_HPP
#define CVREADER_SERIALSIMPLE_HPP

#include <iostream>
#include "serialsom.h"

#include <thread>
#include <atomic>

#include <Eigen/Dense>

class SerialControl{
public:

    SerialControl(std::string port_name)
    {
        port_name_ = port_name;

//        Serialport Serialport1(port_name_);
        fd_ = serial_port1_.open_port(port_name_.c_str());
        serial_port1_.set_opt(9600,9,'N',1);
        current_ang_ = Eigen::Vector2d(0.0,0.0);

        pictch_ = 0;
        yaw_ = 0;
        thread1_=std::thread(keepAngle);
        thread1_.detach();
    }

    ~SerialControl()
    {
        close(fd_);
    }

    bool SetCameraCentre(int x,int y)
    {
        center_point_(x,y);
        return true;
    }

    Eigen::Vector2d input_target(Eigen::Vector2d point)
    {
        //target to central

        Eigen::Vector2d tmp(0.0,0.0);

        tmp(0) = point(0);
        tmp(1) = point(1);

        pictch_ = 20;
        yaw_ =20;




    }


    int SetAngle(uint8_t picth_angle, uint8_t yaw_angle)
    {
//        int fd;
        bool true_false;
//        S
//        Serialport1.set_opt(9600, 8, 'N', 1);
        for(int i = 0; i<3; i++)
            true_false = serial_port1_.usart3_send(picth_angle, yaw_angle);
//        close(fd);
        return true_false;
    }

protected:
    bool keepAngle()
    {
        // Range check;


        //
        SetAngle(pictch_,yaw_);

        std::this_thread::__sleep_for(0,2000000);//2ms.
    }

    std::atomic_int_fast8_t pictch_,yaw_;

private:
    std::string port_name_;


    Eigen::Vector2d current_ang_;


    int fd_;

    Serialport serial_port1_;

    std::thread thread1_;

    Eigen::Vector2d center_point_;





};


#endif //
