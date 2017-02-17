//
// Created by steve on 17-2-17.
//


#include <iostream>

#include <thread>
#include <memory>

#include <opencv2/videoio.hpp>
#include <atomic>
#include <opencv/cv.hpp>

class FastReadTest {
//    std::shared_ptr<cv::Mat> read_ptr_;
//    std::shared_ptr<cv::Mat> write_ptr_;
public:

    cv::Mat a,b;
//    std::shared_ptr<cv::Mat> a = ;

    int read_index = 0;
    int write_index ;

    FastReadTest() {
        write_index = 0;
        std::thread t(&FastReadTest::LoopRead,this);
        t.detach();
    }


    void LoopRead() {
        cv::VideoCapture cap(0);

        while (cap.isOpened()) {
            if(write_index == 0)
            {
                cap >> a;
            }else{
                cap >> b;
            }
//            cap >> a[write_index];
            write_index++;
            if (write_index > 1) {
                write_index = 0;
            }
        }
    }

    cv::Mat ReadFromBuff() {
        read_index = write_index + 1;
        if (read_index > 1) {
            read_index = 0;
        }
        if(read_index == 0)
        {
            return a;
        }else{
            return b;
        }
    }


};


int main() {
    FastReadTest frt;
    bool first_time(true);
    while (true) {
        cv::Mat test;
        if(first_time)
        {
            cv::waitKey(1000);
            first_time = false;
        }
        test = frt.ReadFromBuff();

        cv::imshow("test",test);
        cv::waitKey(1);
    }

}
