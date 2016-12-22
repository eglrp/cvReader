//Create by steve in 16-12-21 at 下午11:02
//
// Created by steve on 16-12-21.
//

#include <iostream>

#include <Eigen/Dense>

#include <chrono>

#include "KalmanFilter.hpp"

#include "opencv2/opencv.hpp"

#include "opencv2/opencv_modules.hpp"

int main() {
    std::cout << CV_VERSION << std::endl;

    KalmanFilter<double, 4, 2> kf_test(2.0, 1.0, Eigen::Vector4d(1.0, 1.0, 1.0, 1.0));

    for (int i(1); i < 1000; ++i) {
        int j(i * 2);
        Eigen::VectorXd tmp = kf_test.OneStep(Eigen::Vector2d(i, j));

        std::cout << "i,j is :" << i << "," << j << std::endl;
        std::cout << tmp.transpose() << std::endl;

    }
    

}