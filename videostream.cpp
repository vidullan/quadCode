#include "videostream.h"

VideoStream::VideoStream(cv::VideoCapture &c, cv::VideoWriter &o, int send_fps):cap(c), out(o){ 
    fps = send_fps;
}

void VideoStream::startStream(){
    if(streamActive == false){
            streamActive = true;
        
        //Spawn thread to stream camera frames
        streamThread=std::thread(&VideoStream::stream, this);

        #ifdef DISPLAY_MESSAGES
        cout<<"\n(VIDEOSTREAM) Spawning thread to start stream.";
        #endif
    }
    else{
        #ifdef DISPLAY_MESSAGES
        cout<<"\n(VIDEOSTREAM) Stream already running.";
        #endif
    }
}


void VideoStream::stream(){

    cv::Mat frame;

    while (streamActive){
        cap.read(frame);
        if(frame.empty()){
            //std::cout<<"\n(VIDEOSTREAM) ERROR: Frame not received from video source.";
            continue;
        }

        cv::imshow("i",frame);
                
        if(cv::waitKey(1) == 's')
            break;
        //out.write(frame);
    }

}

void VideoStream::stopStream(){
    if(streamActive){
        streamActive = false;
        #ifdef DISPLAY_MESSAGES
        cout<<"\n(VIDEOSTREAM) Stopping stream.";
        #endif
    }
    else{
        #ifdef DISPLAY_MESSAGES
        cout<<"\n(VIDEOSTREAM) Stream already stopped.";
        #endif
    }
}
