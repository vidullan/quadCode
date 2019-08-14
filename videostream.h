#ifndef VIDEOSTREAM_H
#define VIDEOSTREAM_H

#include <opencv2/opencv.hpp>
#include <string>
#include <iostream>
#include <unistd.h>
#include <thread>
#include "common.h"

class VideoStream{
    int fps;
    cv::VideoCapture &cap;
    cv::VideoWriter &out;

    bool streamActive;
    std::thread streamThread;

    void stream();

public: 
    VideoStream(cv::VideoCapture &c, cv::VideoWriter &o, int send_fps);
    void startStream();
    void stopStream();

    ~VideoStream(){
        if(streamActive)
            stopStream();
    }
};

#endif