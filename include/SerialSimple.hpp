#pragma once
//Create by steve in 16-12-21 at 下午11:02
//
// Created by steve on 16-12-21.
//

#ifndef CVREADER_SERIALSIMPLE_HPP
#define CVREADER_SERIALSIMPLE_HPP

#include <iostream>
#include "serialsom.h"

class SerialControl{
public:

    SerialControl(std::string port_name)
    {
        port_name_ = port_name;
    }


    int SetAngle(uint8_t picth_angle, uint8_t yaw_angle)
    {
        int fd;
        bool true_false;
        Serialport Serialport1(port_name_);
        fd = Serialport1.open_port(port_name_);
        Serialport1.set_opt(9600, 8, 'N', 1);
        for(int i = 0; i<3; i++)
            true_false = Serialport1.usart3_send(picth_angle, yaw_angle);
        close(fd);
        return true_false;
    }

private:
    std::string port_name_;
};


#endif //CVREADER_SERIALSIMPLE_HPP
