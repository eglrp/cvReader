//
// Created by steve on 17-1-21.
//

#include <opencv2/viz.hpp>
#include <iostream>

using namespace cv;
using namespace std;

void help() {
    cout
            << "--------------------------------------------------------------------------" << endl
            << "This program shows how to launch a 3D visualization window. You can stop event loop to continue executing. "
            << "You can access the same window via its name. You can run event loop for a given period of time. "
            << endl
            << "Usage:" << endl
            << "./launching_viz" << endl
            << endl;
}

int main() {
    help();
    viz::Viz3d myWindow("Viz Demo");
    myWindow.spin();

//    myWindow.showImage()

    while (!myWindow.wasStopped()) {
        myWindow.spinOnce(1, true);
    }
    cout << "Last event loop is over" << endl;
    return 0;
}