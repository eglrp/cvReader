#pragma once
//Create by steve in 16-12-22 at 下午12:59
//
// Created by steve on 16-12-22.
//

#ifndef CVREADER_ARCODE_HPP
#define CVREADER_ARCODE_HPP


#include <opencv2/opencv.hpp>

#include <opencv2/aruco.hpp>


class ArCodePort{
public:
    ArCodePort(int aruco_type)
    {
        dictionary_ptr_ =cv::Ptr<cv::aruco::Dictionary>(cv::aruco::getPredefinedDictionary(aruco_type)) ;
    }

    int DetectMarkers(cv::Mat src_image,std::vector<std::vector<Point2f>> corners,std::vector<int> ids,cv::Mat *res)
    {
        cv::aruco::detectMarkers(src_image,dictionary_ptr_,corners,ids);
        if(ids.size() > 0)
        {
            cv::aruco::drawDetectedMarkers(*res,corners,ids);

        }
        return ids.size();
    }



private:
    cv::Ptr<cv::aruco::Dictionary> dictionary_ptr_;

    std::vector<std::vector<cv::Point2f>> corners_;
    std::vector<int> ids_;
};

#endif //CVREADER_ARCODE_HPP
