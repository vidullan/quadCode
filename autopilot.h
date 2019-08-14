#ifndef _AUTOPILOT_H_
#define _AUTOPILOT_H_

#include <termios.h>
#include <string>
#include <sys/time.h>
#include <unistd.h>
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>
#include <cmath>
#include <atomic>
#include <fstream>

#include "pixhawk.h"
#include "image.h"

#include "struct.h"
#include "mavlink2/common/mavlink.h" //changed this :- Mollik


typedef unsigned short int usi;
using namespace std;


#define MAVLINK_MSG_SET_POS_TAR_L_NED_POSITION 		0b0000110111111000
#define MAVLINK_MSG_SET_POS_TAR_L_NED_POSITION_X	0b0000110111111110
#define MAVLINK_MSG_SET_POS_TAR_L_NED_POSITION_Y    0b0000110111111101
#define MAVLINK_MSG_SET_POS_TAR_L_NED_POSITION_z 	0b0000110111111011

#define MAVLINK_MSG_SET_POS_TAR_L_NED_VELOCITY 		0b0000110111000111
#define MAVLINK_MSG_SET_POS_TAR_L_NED_VELOCITY_X	0b0000110111110111
#define MAVLINK_MSG_SET_POS_TAR_L_NED_VELOCITY_Y 	0b0000110111101111
#define MAVLINK_MSG_SET_POS_TAR_L_NED_VELOCITY_Z 	0b0000110111011111
#define MAVLINK_MSG_SET_POS_TAR_L_NED_YAW		    0b0000000111111111
#define MAVLINK_MSG_SET_POS_TAR_L_NED_ALL		    0b0000110111000000

#define MAVLINK_MSG_SET_POS_TAR_L_NED_TAKEOFF		0x1000

#define MAVLINK_MSG_SET_ATT_EULER_RATE			0b11111000
#define MAVLINK_MSG_SET_ATT_QUATERNION			0b01111111
#define MAVLINK_MSG_SET_ATT_THRUST			0b10111111

#define MODE_OFFBOARD		393216
#define MODE_STABILIZED		458752

extern Buffers buffers;


struct QuadParam{
	mavlink_local_position_ned_t iPos;
	mavlink_attitude_t iAtt;

	mavlink_local_position_ned_t cPos;
	uint64_t cPosTimestamp;
	mavlink_attitude_t cAtt;
    uint64_t cAttTimestamp;

	ImgData imgData;

	mavlink_local_position_ned_t hPos;
	mavlink_local_position_ned_t tPos;
	mavlink_local_position_ned_t dPos;

	float fSonarDist;
	uint64_t fSonarDistTimestamp;

	struct{
        uint64_t iPos;
        uint64_t iAtt;
        uint64_t fSonarDist;
        uint64_t hPos;
        uint64_t tPos;
        uint64_t dPos;
	}timestamp;


	QuadParam(){
        fSonarDist=-1;
        timestamp.iPos=-1;
        timestamp.hPos=-1;
        timestamp.tPos=-1;
        timestamp.dPos=-1;
        timestamp.iAtt=-1;
        timestamp.fSonarDist=-1;
	}

	QuadParam(const QuadParam &obj){
		if(this!=&obj){
			iPos=obj.iPos;
			tPos=obj.tPos;
			hPos=obj.hPos;
			dPos=obj.dPos;
			iAtt=obj.iAtt;
			cPos=obj.cPos;
			cAtt=obj.cAtt;
			imgData=obj.imgData;
			fSonarDist=obj.fSonarDist;

            timestamp.iPos=obj.timestamp.iPos;
            timestamp.iPos=obj.timestamp.hPos;
            timestamp.iPos=obj.timestamp.tPos;
            timestamp.iPos=obj.timestamp.dPos;
            timestamp.iAtt=obj.timestamp.iAtt;
            timestamp.fSonarDist=obj.timestamp.fSonarDist;
		}
	}

	QuadParam & operator= (const QuadParam &obj){
		if(this!=&obj){
			iPos=obj.iPos;
			tPos=obj.tPos;
			hPos=obj.hPos;
			dPos=obj.dPos;
			iAtt=obj.iAtt;
			cPos=obj.cPos;
			cAtt=obj.cAtt;
			imgData=obj.imgData;
			fSonarDist=obj.fSonarDist;

            timestamp.iPos=obj.timestamp.iPos;
            timestamp.iPos=obj.timestamp.hPos;
            timestamp.iPos=obj.timestamp.tPos;
            timestamp.iPos=obj.timestamp.dPos;
            timestamp.iAtt=obj.timestamp.iAtt;
            timestamp.fSonarDist=obj.timestamp.fSonarDist;
		}
		return *this;
	}
};

class Behaviour{
protected:
	string name;
	std::thread thread;
	std::atomic<bool> threadRunning;
	std::atomic<bool> taskCompleted;
	std::atomic<bool> stopFlag;

	QuadParam &param; //Quad parameters
	mavlink_set_position_target_local_ned_t &posTarget; //pos target to send to the pixhawk

	virtual void ruleBase()=0; //rules that govern the behaviour

public:
	Behaviour(QuadParam &q,mavlink_set_position_target_local_ned_t &posT)
                                                    :param(q),posTarget(posT){
		name="GenericBehaviour";
		threadRunning=false;
		taskCompleted=false;
		stopFlag=false;
	}

	virtual ~Behaviour(){};

	virtual void start()=0; //To start the behaviour
	virtual void stop()=0; //To stop the behaviour

	bool completed(){return taskCompleted;}
	string getName(){return name;}

	uint64_t getTimeUsec(){
		struct timespec t;
		clock_gettime(CLOCK_REALTIME,&t);
		return (t.tv_sec)*1e6+(t.tv_nsec)/1e3;
    }

protected:
    void bodyToNED(float bx, float by, float theta, float &lx, float &ly){
        lx=bx*cos(theta)-by*sin(theta);
        ly=bx*sin(theta)+by*cos(theta);
    }

    void NEDTobody(float lx, float ly, float theta, float &bx, float &by){
        bx=+lx*cos(theta)+ly*sin(theta);
        by=-lx*sin(theta)+ly*cos(theta);
    }
};


class Action{
	int sysID;
	int compID;

	Pixhawk &pixhawk;
	Arduino &arduino;
    Image &image;

	QuadParam param;

	vector<Behaviour*> behaviour;
    vector<mavlink_set_position_target_local_ned_t*> posVec;

	std::thread thread;
	bool threadRunning;

	bool obFlag;

    mavlink_set_position_target_local_ned_t cPosTarget;
	mavlink_message_t msg;

	enum {SLEEP,TAKEOFF,SEARCHFORTARGET,SEARCHFORPICKUP,SEARCHFORHOMEBASE,AVOIDBOUNDARY,AVOIDOBSTACLE,LAND}state,pState;

	void getParameters(){

		param.cPos=buffers.messages.localPos;
		param.cAtt=buffers.messages.attitude;

        param.timestamp.iPos=buffers.messages.timeStamps.localPos;
        param.timestamp.iAtt=buffers.messages.timeStamps.attitude;

		param.imgData=buffers.imgData;
		param.fSonarDist = buffers.fSonarDist;

	}

    uint64_t getTimeUsec(){
		struct timespec t;
		clock_gettime(CLOCK_REALTIME,&t);
		return (t.tv_sec)*1e6+(t.tv_nsec)/1e3;
    }

    void bodyToNED(float bx, float by, float theta, float &lx, float &ly){
        lx=bx*cos(theta)-by*sin(theta);
        ly=bx*sin(theta)+by*cos(theta);
    }

    void NEDTobody(float lx, float ly, float theta, float &bx, float &by){
        bx=+lx*cos(theta)+ly*sin(theta);
        by=-lx*sin(theta)+ly*cos(theta);
    }


public:
	void setBehaviour();

	Action(Pixhawk &px, Arduino &ard, Image &img):pixhawk(px),arduino(ard),image(img){
		threadRunning=false;
		sysID=0;compID=0;
		state=SLEEP;
		pState=SLEEP;

		param.iAtt.yaw=-7; //yaw cannot be greater than 2*pi

		cPosTarget.target_system=sysID;
		cPosTarget.target_component=compID;
		cPosTarget.vx=0;
		cPosTarget.vy=0;
		cPosTarget.vz=0;
		cPosTarget.z=-1;
		cPosTarget.type_mask=MAVLINK_MSG_SET_POS_TAR_L_NED_VELOCITY;

		param.fSonarDist=buffers.fSonarDist;

		obFlag=false;
	}

	void eventHandler();
	void start();
	void stop();
	int rebootPixhawk();
};

#endif
