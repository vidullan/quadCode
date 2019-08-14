//Defines the class that sends data to the pixhawk over USB

#ifndef PIXHAWK_H
#define PIXHAWK_H

#include <termios.h>
#include <string>
#include <sys/time.h>

#include <iostream>
#include <mutex>
#include <thread>


#include "serial.h"
#include "mavlink2/common/mavlink.h" //changed this :- Mollik
#include "common.h"
#include "struct.h"


struct mavlinkMsgsDefined{
	mavlink_heartbeat_t heartbeat;
	mavlink_sys_status_t sysStatus;
	mavlink_system_time_t sysTime;
	mavlink_timesync_t timesync;
	mavlink_battery_status_t batteryStatus;
	mavlink_radio_status_t radioStatus;
	mavlink_local_position_ned_t localPos;
	mavlink_position_target_local_ned_t localPosTarget;
	mavlink_highres_imu_t imuData;
	mavlink_attitude_t attitude;
	mavlink_attitude_target_t attitudeTarget;
	mavlink_optical_flow_rad_t opticalFlow;
};

class Mavlink_Messages{

	Mavlink_Messages(const Mavlink_Messages&);
  	Mavlink_Messages& operator=(const Mavlink_Messages&);


	int64_t timeSkew;
	std::mutex muTimeSkew;

public:

	int sysID;
	int compID;

	int64_t getTimeSkew(){
		int64_t t;
		muTimeSkew.lock();
		t = timeSkew;
		muTimeSkew.unlock();
		return t;
	}

	void setTimeSkew(int64_t skew){
		muTimeSkew.lock();
		timeSkew = skew;
		muTimeSkew.unlock();
	}

	// wrapper to provide mutex functionality 
	template <class T>
	class msgWrapper{
		T msg;
		uint64_t timestamp;
		std::mutex mu;
	public:
		void getData(T &dst, uint64_t &t){
			mu.lock();
			dst = msg;
			t = timestamp;
			mu.unlock();
		}
		void getData(uint64_t &t){
			mu.lock();
			t = timestamp;
			mu.unlock();
		}
		void getData(T &dst){
			mu.lock();
			dst = msg;
			mu.unlock();
		}

		void setData(const T &src, const uint64_t &t){
			mu.lock();
			msg = src;
			timestamp = t;
			mu.unlock();
		}

		msgWrapper(){
			timestamp = 0;
		}

	};
	
	//Messages we care about
	//Make sure they match mavlinkMessagesDefined class
	msgWrapper<mavlink_heartbeat_t> heartbeat;
	msgWrapper<mavlink_sys_status_t> sysStatus;
	msgWrapper<mavlink_system_time_t> sysTime;
	msgWrapper<mavlink_battery_status_t> batteryStatus;
	msgWrapper<mavlink_radio_status_t> radioStatus;
	msgWrapper<mavlink_local_position_ned_t> localPos;
	msgWrapper<mavlink_position_target_local_ned_t> localPosTarget;
	msgWrapper<mavlink_highres_imu_t> imuData;
	msgWrapper<mavlink_attitude_t> attitude;
	msgWrapper<mavlink_attitude_target_t> attitudeTarget;
	msgWrapper<mavlink_optical_flow_rad_t> opticalFlow;


	Mavlink_Messages(){
		sysID=-1;
		compID=-1;
	}


	void printMessages(){
		uint64_t t;
		mavlinkMsgsDefined msgs;

		std::cout<<"\n\n--------------------------------------------------";
		std::cout<<"\nReceived buffers.messages: ";
		
		heartbeat.getData(msgs.heartbeat, t);
		std::cout<<"\n\nGot heartbeat at "<<t;
		std::cout<<"\n\nautopilot: "<<msgs.heartbeat.autopilot;
		std::cout<<"\n\tb: "<<msgs.heartbeat.base_mode;
		std::cout<<"\tc: "<<msgs.heartbeat.custom_mode;
		std::cout<<"\tstatus: "<<msgs.heartbeat.system_status;
		
		sysTime.getData(msgs.sysTime, t);
		std::cout<<"\n\nGot system time at "<<t;
		std::cout<<"\nEpoch: "<<msgs.sysTime.time_unix_usec<<"\tBoot: "<<msgs.sysTime.time_boot_ms<<"\tSkew: "<<msgs.sysTime.time_unix_usec - t;
		
		sysStatus.getData(msgs.sysStatus, t);
		std::cout<<"\nGot system status at "<<t;
		//This doesn't exist? cout<<"\n\tMode: "<<buffers.messages.sysStatus.mode<<"\t"<<buffers.messages.sysStatus.nav_mode;
		//This doesn't exist? cout<<"\n\tStatus: "<<buffers.messages.sysStatus.status;
		std::cout<<"\n\tBat vol: "<<msgs.sysStatus.voltage_battery;
		
		batteryStatus.getData(msgs.batteryStatus, t);
		std::cout<<"\nGot battery status at "<<t;

		radioStatus.getData(msgs.radioStatus, t);
		std::cout<<"\nGot radio status at "<<t;


		localPos.getData(msgs.localPos, t);
		std::cout<<"\nGot local position at "<<t;
		std::cout<<"\n\tPos: "<<msgs.localPos.x<<","<<msgs.localPos.y<<","<<msgs.localPos.z;
		std::cout<<"\n\tVel: "<<msgs.localPos.vx<<","<<msgs.localPos.vy<<","<<msgs.localPos.vz;
		
		//std::cout<<"\nGot global position at "<<timeStamps.globalPos;

		localPosTarget.getData(msgs.localPosTarget, t);
		std::cout<<"\nGot local position target at "<<t;
		std::cout<<"\n\tPos Tar: "<<msgs.localPosTarget.x<<","<<msgs.localPosTarget.y<<","<<msgs.localPosTarget.z;
		std::cout<<"\n\tVel Tar: "<<msgs.localPosTarget.vx<<","<<msgs.localPosTarget.vy<<","<<msgs.localPosTarget.vz;
		
		//std::cout<<"\n\nGot global position target at "<<buffers.messages.timeStamps.globalPosTarget;

		imuData.getData(msgs.imuData, t);
		std::cout<<"\n\nGot imu data at "<<t;
		
		//opticalFlow.getData(msgs.opticalFlow, t);
		//std::cout<<"\n\nGot distance data from optical flow at "<<timeStamps.opticalFlow;
		//std::cout<<"\n\tDistance: "<<opticalFlow.distance;

		attitude.getData(msgs.attitude, t);	
		std::cout<<"\n\nGot attitude at "<<t;
		std::cout<<"\n\tRoll: "<<msgs.attitude.roll<<"\tPitch: "<<msgs.attitude.pitch<<"\tYaw: "<<msgs.attitude.yaw;
		std::cout<<"\n--------------------------------------------------"<<std::endl;
	}
		
};



class Pixhawk: private Serial{

private:
//USB connection
    Serial device;
    int baudRate;
	std::string devName;

//pixhawk parameters
	Mavlink_Messages *px4StateMsgs; //Where to store received msgs
	CircularBuffer<mavlink_message_t> *px4TelemBuffer; //to store orig telem msgs 

	
//threading
	std::thread recvThread;
	bool recvThreadActive;

	bool msgWaitingToSend;

	void getMessages();

	void startReceivingMsgs();
	void stopReceivingMsgs();
	void printMessages();

public:
	Pixhawk(int bR, std::string dN, Mavlink_Messages *m, 
	CircularBuffer<mavlink_message_t> *telem);

	//initialize the USB connection. Needs to be called after obj creations
	int init();

	int sendMessage(mavlink_message_t &msg);

	void deInit();

	~Pixhawk(){
		stopReceivingMsgs();
	}
};



#endif
