#include <opencv2/opencv.hpp>

#include "System.h"

#include <string>
#include <chrono> 
#include <iostream>

using namespace std;

string parameterFile = "./myslam.yaml";
string vocFile = "../../Vocabulary/ORBvoc.txt";

int main(int argc, char **argv) {

    ORB_SLAM2::System SLAM(vocFile, parameterFile, ORB_SLAM2::System::MONOCULAR, true);

    cv::VideoCapture cap(0);

    cap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);

    auto start = chrono::system_clock::now();

    while (1) {
        cv::Mat frame;
        cap >> frame;
        if ( frame.data == nullptr )
            break;

        auto now = chrono::system_clock::now();
        auto timestamp = chrono::duration_cast<chrono::milliseconds>(now - start);
        SLAM.TrackMonocular(frame, double(timestamp.count())/1000.0);
    }

    SLAM.Shutdown();

    return 0;
}
