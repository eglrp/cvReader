//
// Created by steve on 17-1-17.
//

#include <opencv2/opencv.hpp>

int main() {


    cv::VideoCapture cap("./v1/outputLeft.avi");
    

    cv::Mat img;

    while (cap.isOpened()) {
        cap >> img;
        cv::imshow("Test", img);
        cv::waitKey(10);
    }
}
