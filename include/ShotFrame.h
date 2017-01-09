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
#include <opencv2/aruco>

#include "ArCode.hpp"
#include "serialsom.h" // TODO: rewrite this.

#include "ShotFrame.h"

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

    std::atomic<int> delta_x_, delta_y_;//atomic data x,y offset from central point.

    //////
    std::atomic<double> pitch_, yaw_; // Current pose of the orientation.

    std::mutex py_mutex_; //

    //////
    std::mutex serial_mutex_;

    /////
    int serial_handle_; // handle for serial com.

    /////




private:


};


#endif //CVREADER_SHOTFRAME_H
