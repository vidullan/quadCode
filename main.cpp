#include <sys/time.h>
#include "pixhawk.h"
#include <iostream>
#include "struct.h"
#include <string>
#include <fstream>
#include <signal.h>
#include "connect.h"
#include "arduino.h"
#include "t265.h"
#include "common.h"


bool programExit=false;

void my_handler(int s){
    programExit=true;
}



int main(){

    struct sigaction signalHandler;
    signalHandler.sa_handler = my_handler;
    sigemptyset(&signalHandler.sa_mask);
    signalHandler.sa_flags=0;
    sigaction(SIGINT,&signalHandler,NULL);

	//std::ofstream out("out.txt");
	//std::streambuf *coutbuf=std::cout.rdbuf();
	//std::cout.rdbuf(out.rdbuf());

 //   cout<<"\nConnected!"<<endl;

	/*
    char ch;
	std::vector<std::string> serialPath;
	serialPath.push_back("/dev/ttySAC2");
    Arduino trinket(9600,serialPath);
    trinket.init();
    usleep(1e5);
    cout<<"\nOpening gripper. Press any key to continue...";
    trinket.sendChar('1');
    trinket.sendChar('1');
    cin>>ch;
    cout<<"\nClosing around the envelope";
    trinket.sendChar('5');
	*/



	CircularBuffer<mavlink_message_t> px4Telemetry(100); //if we want to send all msgs received
	CircularBuffer<mavlink_message_t> px4SendBuffer(4); //msgs to send to pixhawk
    CircularBuffer<mavlink_message_t> GSMavlinkMsgs(4); //mavlink msgs received from ground station
 	

    /*
     * Initialize object of class Connect.
     * Arguments are the IP address and the port number of the server.
	 * Object calls member function init of class Connect.
     */
    std::cout<<"\nConnecting to ground station...";
    Connect phoenix("192.168.10.13",9998, &GSMavlinkMsgs, &px4Telemetry);
	phoenix.init();

        //starting downward facing cam video stream
        /*
    std::string devName = "/dev/video0";
    int fps = 20;
    int w = 1280;
    int h = 720;
    std::string ip = "192.168.10.13";
    int port = 5000;
    int bitRate = 1500;

    std::string options = "v4l2src device=\""+devName+"\" ! video/x-raw,format=I420,width="+std::to_string(w)+",height="+std::to_string(h)+\
        ",framerate="+std::to_string(fps)+"/1 ! videoscale ! videoconvert ! appsink";
    cv::VideoCapture cap(options,cv::CAP_GSTREAMER);
    //std::cout<<options<<std::endl;
    //cv::VideoCapture cap("v4l2src device=\"/dev/video0\" ! video/x-raw,width=640,height=480,framerate=15/1 ! videoscale ! videoconvert ! appsink",cv::CAP_GSTREAMER);
    //cv::VideoCapture cap(0);

    // VideoWriter: 'videoconvert' converts the 'BGR' images into 'YUY2' raw frames to be fed to
    // 'jpegenc' encoder since 'jpegenc' does not accept 'BGR' images. The 'videoconvert' is not
    // in the original pipeline, because in there we are reading frames in 'YUY2' format from 'v4l2src'
    //options = "appsrc ! videoconvert ! x264enc tune=zerolatency bitrate="+std::to_string(bitRate)+" speed-preset=superfast ! "+\
    "rtph264pay ! udpsink host="+ip+" port="+std::to_string(port);
    //cv::VideoWriter out(options,cv::CAP_GSTREAMER,0,20,cv::Size(w,h),true);
   //std::cout<<options<<std::endl; 
     cv::VideoWriter out("appsrc ! videoconvert ! jpegenc tune=zerolatency bitrate=1000 speed-preset=superfast ! rtpjpegpay ! udpsink host=192.168.10.13 port=5000",cv::CAP_GSTREAMER,0,0,cv::Size(640,480),true);
    //cv::VideoWriter out;

    if(!cap.isOpened()){
        std::cout<<"\n(VIDEOSTREAM) ERROR: Video capture could not be opened.";
        programExit = true;
    } 
     
    if(!out.isOpened())
    {
        std::cout<<"\n(VIDEOSTREAM) ERROR: Video writer could not be opened.";
        programExit = true;
    }

    VideoStream downwardCam(cap, out, fps);
    downwardCam.startStream();

    //usleep(3e6);
    */

    Mavlink_Messages px4State; //pixhawk messages showing state of the quad

	T265Data visionData; 


	Pixhawk px4Mini(921600,"/dev/ttySAC2", &px4State, &px4Telemetry);
	px4Mini.init();



    /*
    Arduino trinket(0x30);
    trinket.init();
    usleep(1e5);
    
    for (int i=0;i<=50;i+=5){
        trinket.setServo(i);
        usleep(3e5);
    }
    for (int i=50;i>=0;i-=5){
        trinket.setServo(i);
        usleep(3e5);
    }
    return 1;
    */
    
    T265 t265(&visionData);
    t265.startReading();
    
    
    mavlink_vision_position_estimate_t vMsg;
    mavlink_message_t msg, setpoint;
    float x,y,z,roll,pitch,yaw;
    uint64_t t;

    mavlink_timesync_t timesync;

    uint64_t currTime = getTimeUsec();
    uint64_t pVMsgTime = getTimeUsec();
    uint64_t pSetpointTime = getTimeUsec();
    uint64_t pGSMsgSendTime = getTimeUsec();
    uint64_t pDispTime = getTimeUsec();
    uint64_t pTimesync = getTimeUsec();
    uint64_t pSendTime = getTimeUsec();


	while(!programExit){

        usleep(1e2);

        currTime = getTimeUsec();

        // Send vision position
        if(currTime - pVMsgTime >= 1.25e4){
            visionData.getData(x,y,z,roll,pitch,yaw,t);
            vMsg.x = x;
            vMsg.y = y;
            vMsg.z = z;
            vMsg.roll = roll;
            vMsg.pitch = pitch;
            vMsg.yaw = yaw;
            vMsg.usec = t;
            vMsg.covariance[0] = 0.005;
            vMsg.covariance[6] = 0.005;
            vMsg.covariance[11] = 0.005;
            vMsg.covariance[15] = 0.001;
            vMsg.covariance[18] = 0.001;
            vMsg.covariance[20] = 0.001;
            //std::cout<<vMsg.usec<<" "<<px4State.getTimeSkew()<<" "<<t<<" "<<(int64_t)currTime<<std::endl;
            
            mavlink_msg_vision_position_estimate_encode(1,1,&msg,&vMsg);
            px4Mini.sendMessage(msg);
            //cout<<"Size: "<<buffers.GSComm.size()<<endl;

            pVMsgTime = currTime;
        }

        // forwarded msgs received from ground station
        if(currTime - pGSMsgSendTime > 1e4){
            while(GSMavlinkMsgs.size()){
                GSMavlinkMsgs.pop(msg);	
                px4SendBuffer.push(msg);

                // store a copy of the last setpoint in case wifi drops
                if(msg.msgid == MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED)
				{
                    setpoint = msg;
                    pSetpointTime = currTime;

				}
                //px4Mini.sendMessage(msg);
                //std::cout<<"\nSent";
            }

            				//if(msg.msgid == 76)
				//{	
					//mavlink_command_long_t m;
					//mavlink_msg_command_long_decode(&msg,&m);
					//cout<<m.target_system<< " "<< m.target_component<<endl;
					//cout<<endl<<m.command<<" "<<m.param1<<" "<<m.param2<<" "<<m.param3<<" "<<m.param4<<" "<<m.param5<<" "<<m.param6<<" "<<m.param7;
					//mavlink_msg_command_long_pack(0,0,&msg,1,0,400,1,1.,0,0,0,0,0,0);
					//buffers.GSComm.push(msg);
				//}
            pGSMsgSendTime = currTime;
        }
        
        // maintain a constant stream of setpoints
        if(currTime - pSetpointTime > 2.5e5){
            mavlink_position_target_local_ned_t m;
            mavlink_msg_position_target_local_ned_decode(&setpoint,&m);
            m.time_boot_ms = getTimeUsec();
            //std::cout<<m.x<<" "<<m.y<<" "<<m.z<<" "<<m.time_boot_ms<<" "<<std::endl;
            //m.x = 0;
            //m.y = 0;
            //m.z = 1.2;
            m.coordinate_frame = MAV_FRAME_LOCAL_NED;
            m.type_mask = 0b111111111000;
            mavlink_msg_position_target_local_ned_encode(1,1,&setpoint, &m);
            px4SendBuffer.push(setpoint);

            pSetpointTime = currTime;            
        }

        // send messages to pixhawk
        if(currTime - pSendTime > 1e3){
            while(px4SendBuffer.size()){
                px4SendBuffer.pop(msg);	
                px4Mini.sendMessage(msg);
            }
            pSendTime = currTime;
        }

        // things that need to be displayed
        if (currTime-pDispTime > 1e6){
            px4State.printMessages();
            pDispTime = currTime;
            //std::cout<<px4State.getTimeSkew()<<" "<<getTimeUsec()<<" "<<std::endl;
            
            //visionData.getData(x,y,z,roll,pitch,yaw,t);
            //std::cout<<x<< " "<<y<<" "<<z<<" "<<roll*57.3<<" "<<pitch*57.3<<" "<<yaw*57.3<<std::endl;

            //std::cout<<"\nNumber of pixhawk telemetry messages: "<<px4Telemetry.size()<<"\n";
            
            //mavlink_local_position_ned_t pos;
            //uint64_t t;
            //px4State.localPos.getData(pos, t);
            //std::cout<<"\n\tPos:"<<pos.x<<","<<pos.y<<","<<pos.z<<std::endl;
		
  //cout<<buffers.imgData.dropoff.mX<<" "<<buffers.imgData.dropoff.mY<<endl;
		//cout<<buffers.fSonarDist<<endl;
		//std::cout<<"\n\n\tPos:"<<buffers.messages.localPos.x<<","<<buffers.messages.localPos.y<<","<<buffers.messages.localPos.z;
		//std::cout<<"\n\tVel: "<<buffers.messages.localPos.vx<<","<<buffers.messages.localPos.vy<<","<<buffers.messages.localPos.vz;
		//std::cout<<"\n"<<buffers.t265.x<<" "<<buffers.t265.y<<" "<<buffers.t265.z<<" "<<buffers.t265.roll<<" "<<buffers.t265.pitch<<" "<<buffers.t265.yaw;


        }


    }

//    autopilot.stop();
//    image.stopProcessing();
    px4Mini.deInit();
    t265.stopReading();
   
//    trinket.deInit();

	std::cout<<"\nExit gracefully";

	return 0;
}
