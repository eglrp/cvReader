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

#include "myHead.hpp"
#include "opticalFlow.h"

#include <opencv2/aruco.hpp>
static bool readDetectorParameters(string filename, Ptr<aruco::DetectorParameters> &params) {
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["adaptiveThreshWinSizeMin"] >> params->adaptiveThreshWinSizeMin;
    fs["adaptiveThreshWinSizeMax"] >> params->adaptiveThreshWinSizeMax;
    fs["adaptiveThreshWinSizeStep"] >> params->adaptiveThreshWinSizeStep;
    fs["adaptiveThreshConstant"] >> params->adaptiveThreshConstant;
    fs["minMarkerPerimeterRate"] >> params->minMarkerPerimeterRate;
    fs["maxMarkerPerimeterRate"] >> params->maxMarkerPerimeterRate;
    fs["polygonalApproxAccuracyRate"] >> params->polygonalApproxAccuracyRate;
    fs["minCornerDistanceRate"] >> params->minCornerDistanceRate;
    fs["minDistanceToBorder"] >> params->minDistanceToBorder;
    fs["minMarkerDistanceRate"] >> params->minMarkerDistanceRate;
    fs["doCornerRefinement"] >> params->doCornerRefinement;
    fs["cornerRefinementWinSize"] >> params->cornerRefinementWinSize;
    fs["cornerRefinementMaxIterations"] >> params->cornerRefinementMaxIterations;
    fs["cornerRefinementMinAccuracy"] >> params->cornerRefinementMinAccuracy;
    fs["markerBorderBits"] >> params->markerBorderBits;
    fs["perspectiveRemovePixelPerCell"] >> params->perspectiveRemovePixelPerCell;
    fs["perspectiveRemoveIgnoredMarginPerCell"] >> params->perspectiveRemoveIgnoredMarginPerCell;
    fs["maxErroneousBitsInBorderRate"] >> params->maxErroneousBitsInBorderRate;
    fs["minOtsuStdDev"] >> params->minOtsuStdDev;
    fs["errorCorrectionRate"] >> params->errorCorrectionRate;
    return true;
}

int main() {
//    std::cout << CV_VERSION << std::endl;
//    std::default_random_engine e_;
//    std::normal_distribution<> normal_distribution(0, 3.0);
//
//    KalmanFilter<double, 4, 2> kf_test(3.0, 3.0, Eigen::Vector4d(5.0, 2.0, 1.0, 1.0));
//
//    int j(0);
//    for (int i(1); i < 1000; ++i) {
//         j +=( 10+normal_distribution(e_));
//
//        double ti = double(i) + normal_distribution(e_);
//        double tj = double(j) + normal_distribution(e_);
//        Eigen::VectorXd tmp = kf_test.OneStep(Eigen::Vector2d(ti,
//                                                              tj));
//
//
//        std::cout << "i,j is :" << i << "," << j <<
//                  " ti,tj: " <<
//                  ti << "," << tj << std::endl;
//        std::cout << tmp.transpose() << std::endl;
//
//    }
//    std::cout << cv::aruco::DICT_6X6_100 << std::endl;
    cv::Ptr<cv::aruco::Dictionary> dictionary(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL));
    int markerSide = 20;
    int imageSize = 200;
    cv::Ptr<cv::aruco::DetectorParameters> parameters(aruco::DetectorParameters::create());
    readDetectorParameters("detector_params.yml",parameters);
//    parameters->minMarkerPerimeterRate = min(4., (4. * markerSide) / float(imageSize) + 0.1);
    cv::VideoCapture capture("/dev/video0");
//    cv::VideoCapture capture("./44.avi");
    if (!capture.isOpened()) {
        std::cout << "not opened:" << std::endl;
        cv::waitKey(1000);
        capture.open(0);
        cv::waitKey(1000);
    } else {
        std::cout << "opened " << std::endl;
    }

//    OpticalFlow droneOF(50,0.01,10);

    cv::Mat src, result;


    while (capture.isOpened()) {
        capture >> src;

//        src = imread("/home/steve/opencv/opencv_contrib/modules/aruco/tutorials/aruco_detection/images/singlemarkersoriginal.png");
//        src = imread("./markers.jpg");
        cv::waitKey(10);
        result = src;
//
//        droneOF.tracking(src);
//        result = droneOF.output;
        vector<int> markerIds;
        vector<vector<Point2f> > markerCorners, rejectedCandidates;


//        cv::aruco::detectMarkers(src, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
        cv::aruco::detectMarkers(src, dictionary, markerCorners, markerIds);
        if (markerIds.size() > 0)
        {
            cv::aruco::drawDetectedMarkers(result, markerCorners, markerIds);
            std::cout << markerIds.size() << std::endl;
//            cv::waitKey();
        }
        std::cout << "-----" << std::endl;

        cv::namedWindow("result");


        cv::imshow("result", result);
        cv::waitKey(10);


    }


}