#include "t265.h"


void T265Data::setData(float xS,float yS,float zS,
                        float rollS,float pitchS,float yawS, uint64_t &t){
    mu.lock();
    x = xS; y = yS; z = zS;
    roll = rollS; pitch = pitchS; yaw = yawS;
    timestamp = t;
    mu.unlock();
}

void T265Data::getData(float &xD,float &yD,float &zD,
                float &rollD,float &pitchD,float &yawD, uint64_t &t){
    mu.lock();
    xD = x; yD = y; zD = z;
    rollD = roll; pitchD = pitch; yawD = yaw;
    t = timestamp;
    mu.unlock();
}


T265::T265(T265Data *d){
    data = d;
}

void T265::readPose(){
        
    float pos[3], q[4];
    float posNED[3], qNED[4], roll, pitch, yaw;
    uint64_t t;

    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_POSE,RS2_FORMAT_6DOF);
    pipe.start(cfg);
    
    //count frames per second
    uint64_t pCntTime = getTimeUsec();
    int cnt;
    
    while(readThreadActive){
        
        //calc framerate
	/*
        cnt++;
        if(getTimeUsec()-pCntTime >1e6){
            std::cout<<"\n(T265) FPS = "<<cnt*1e6/(getTimeUsec()-pCntTime);
            pCntTime=getTimeUsec();
            cnt=0;
        }
	*/
		
        auto frames = pipe.wait_for_frames();
        auto f = frames.first_or_default(RS2_STREAM_POSE);
        auto pose_data = f.as<rs2::pose_frame>().get_pose_data();
        
        t = getTimeUsec();
        //cout<<pose_data.translation.x<<" "<<pose_data.translation.y<<" "<<pose_data.translation.z<<endl;
        
        pos[0] = pose_data.translation.x;
        pos[1] = pose_data.translation.y;
        pos[2] = pose_data.translation.z;
        q[0] = pose_data.rotation.w;
        q[1] = pose_data.rotation.x;
        q[2] = pose_data.rotation.y;
        q[3] = pose_data.rotation.z;
        

        float dA[3] = {0*3.14159265/180, -45*3.1415926535/180, 0};
        //poseTransArb("forward",q,pos,dA,qNED,posNED);
        attTrans("forward", q, qNED);
        posTrans("forward", pos, posNED);
        quat2euler321(qNED, roll, pitch, yaw);

        //roll += 1.570796; //correction for upsidedown t265
	    pitch += 0.785398; //correction for angled t265
        
        
        data->setData(posNED[0],posNED[1],posNED[2],roll, pitch, yaw, t);
        
        //usleep(3e5);
    }	
}

void T265::startReading(){
    if(readThreadActive==false){
        readThreadActive=true;
        //Spawn read thread for messages
        readThread=std::thread(&T265::readPose,this);
        #ifdef DISPLAY_MESSAGES
        std::cout<<"\n(T265) Spawning thread to get vision pose.";
        #endif
    }
}

void T265::stopReading(){
    if(readThreadActive){
        readThreadActive=false;
        if(readThread.joinable()){
            readThread.join();
            #ifdef DISPLAY_MESSAGES
            std::cout<<"\n(T265) Joined thread that read pose.";
            #endif
        }
    }
}
