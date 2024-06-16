#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include "unistd.h"
#include "QR_test.hpp"
using namespace std;
using namespace cv;

extern int result;


extern "C" {
void *read_QRCode(void* unused) {
    VideoCapture cap(0);
    QRCodeDetector detector;

    if(!cap.isOpened()){
        cout << "no camera" << endl;
        return NULL;
    }

    string info = "";
    Mat frame, gray;
    //cap.set(cv::CAP_PROP_FPS, 120);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 480);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 320);
	printf("test\n");
    while(1) {
        cap.read(frame);
        //cap >> frame;
        if(frame.empty()){
        cout << "no frame" << endl;
            //break;
        }

        cvtColor(frame, gray, COLOR_BGR2GRAY);
        vector<Point> points;

        info = detector.detectAndDecode(frame, points);
            //result = stoi(info);
            //intinfo = stoi(info);
            //cout << info << endl;
        //printf("now\n");
        if(!info.empty()) {
            //result = stoi(info);
			cout << "QR: " << info << endl;
			
			result = stoi(info);
			
			
                //printf("value : %d \n", infoint);
        }
        else {}
            //imshow("Camera Viewer", frame);
        //if(waitKey(1) == 'q') break;
    }


    cap.release();
    //destroyAllWindows();
    return NULL;
}
}

