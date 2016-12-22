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

#include <random>

int main() {
    std::cout << CV_VERSION << std::endl;
    std::default_random_engine e_;
    std::normal_distribution<> normal_distribution(0, 15.0);

    KalmanFilter<double, 4, 2> kf_test(2.0, 10.0, Eigen::Vector4d(2.0, 2.0, 1.0, 1.0));

    for (int i(1); i < 1000; ++i) {
        int j(i * i);

        double ti = double(i) + normal_distribution(e_);
        double tj = double(j) + normal_distribution(e_);
        Eigen::VectorXd tmp = kf_test.OneStep(Eigen::Vector2d(ti,
                                                              tj));


        std::cout << "i,j is :" << i << "," << j <<
                  " ti,tj: " <<
                  ti << "," << tj << std::endl;
        std::cout << tmp.transpose() << std::endl;

    }


}