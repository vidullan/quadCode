//Defines the class that sends data to the pixhawk over USB

#ifndef T265_H
#define T265_H

#include <cmath>
#include <librealsense2/rs.hpp>
#include <iostream>
#include <unistd.h>
#include <mutex>

#include <thread>

#include "transforms.h"
#include "common.h"

class T265Data{
    uint64_t timestamp; //Unix tiime since epoch
    float x;
    float y;
    float z;
    float roll;
    float pitch;
    float yaw;

	std::mutex mu;
public:
	void setData(float xS,float yS,float zS,float rollS,float pitchS,float yawS, uint64_t &t);
	void getData(float &xD,float &yD,float &zD,float &rollD,float &pitchD,float &yawD, uint64_t &t);
};

class T265{

private:
	    
	//threading
	std::thread readThread;
	bool readThreadActive;
	std::mutex mutex;

	void readPose();

	T265Data *data;

public:

	T265(T265Data *d);

	void startReading();
	
	void stopReading();


	~T265(){
		stopReading();
		data = NULL;
	}

};


#endif
