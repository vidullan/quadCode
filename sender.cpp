#include <iostream>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

void sender()
{
    // VideoCapture: Getting frames using 'v4l2src' plugin, format is 'BGR' because
    // the VideoWriter class expects a 3 channel image since we are sending colored images.
    // Both 'YUY2' and 'I420' are single channel images. 

//    VideoCapture cap("v4l2src device=\"/dev/video0\" ! video/x-raw,width=640,height=480,framerate=15/1 ! videoscale ! videoconvert ! appsink",cv::CAP_GSTREAMER);
   std::string options = "v4l2src device=\""+devName+"\" ! video/x-raw,width="+std::to_string(w)+",height="+std::to_string(h)+\
        ",framerate="+std::to_string(fps)+"/1 ! videoscale ! videoconvert ! appsink";
    cv::VideoCapture cap(options,cv::CAP_GSTREAMER);
    // VideoWriter: 'videoconvert' converts the 'BGR' images into 'YUY2' raw frames to be fed to
    // 'jpegenc' encoder since 'jpegenc' does not accept 'BGR' images. The 'videoconvert' is not
    // in the original pipeline, because in there we are reading frames in 'YUY2' format from 'v4l2src'
    VideoWriter out("appsrc ! videoconvert ! x264enc tune=zerolatency bitrate=1000 speed-preset=superfast ! rtph264pay ! udpsink host=192.168.10.13 port=5000",CAP_GSTREAMER,0,20,Size(640,480),true);
    
    if(!cap.isOpened() || !out.isOpened())
    {
        if (!out.isOpened())
            cout<<"out not opened"<<endl;
        cout<<"VideoCapture or VideoWriter not opened"<<endl;
        exit(-1);
    }

    Mat frame;
    
    while(true) {

        cap.read(frame);
        //imshow("Sender", frame);
       

        if(frame.empty())
            break;

        out.write(frame);

        
        if(waitKey(1) == 's')
            break;
    }
    destroyWindow("Sender");
}

int main()
{
    sender();

    return 0;
}