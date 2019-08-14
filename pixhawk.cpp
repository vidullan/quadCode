#include "pixhawk.h"

//Initializes object with USB path and baud rate
Pixhawk::Pixhawk(int bR, std::string dN, Mavlink_Messages *m, 
	CircularBuffer<mavlink_message_t> *telem){

	px4StateMsgs = m;
	px4TelemBuffer = telem;

	baudRate=bR;
	devName=dN;

	recvThreadActive=false;

}


//Initializes the serial connection
int Pixhawk::init(){
	if(device.connect(baudRate,devName)==0){
        #ifdef DISPLAY_MESSAGES
        std::cout<<"\n(PIXHAWK) Connected to the device.";
        #endif
    }
	else{
		perror("\n(PIXHAWK) ERROR: Could not connect to the device.");
		return -1;
	}

	startReceivingMsgs();

	return 0;
}


void Pixhawk::deInit(){
	stopReceivingMsgs();
	device.disconnect();
	px4StateMsgs = NULL;
	px4TelemBuffer = NULL;
}


//Send mavlink msg to the pixhawk
int Pixhawk::sendMessage(mavlink_message_t &msg){
	uint16_t len;
	uint8_t buffer[MAVLINK_MAX_PACKET_LEN]; //mavlink msg to serialized buffer
	len=mavlink_msg_to_send_buffer(buffer,&msg);

	msgWaitingToSend=true;
	int n=device.sendBytes(buffer,len);
    msgWaitingToSend=false;

	if(n==len){
		#ifdef DISPLAY_PX_MESSAGES
		std::cout<<"\n(PIXHAWK) Message transmitted.";
		#endif
        return 1;
    }
    else{
        #ifdef DISPLAY_PX_MESSAGES
		perror("\n(PIXHAWK) ERROR: Message transmission failed.");
		#endif
    }

    return 0;
}


void Pixhawk::startReceivingMsgs(){
	
	if(recvThreadActive==false){
		recvThreadActive=true;
		//Spawn read thread for messages
		recvThread=std::thread(&Pixhawk::getMessages,this);
		
		#ifdef DISPLAY_MESSAGES
		std::cout<<"\n(PIXHAWK) Spawning thread to decode mavlink messages.";
		#endif

	}
}


void Pixhawk::stopReceivingMsgs(){
	if(recvThreadActive){
		recvThreadActive=false;
		if(recvThread.joinable()){
			recvThread.join();
			
			#ifdef DISPLAY_MESSAGES
			std::cout<<"\n(PIXHAWK) Joined thread that decoded mavlink messages.";
			#endif
		}
	}
}


void Pixhawk::getMessages(){
	uint8_t byte, msgReceived;
	mavlink_status_t status;
	//uint16_t len;
	int n;
	mavlink_message_t msg;

	mavlinkMsgsDefined msgs;

	//bool receivedAllMsgs=false;

	//Counting transfer rate
	int cnt=0;
	uint64_t pCntTime=getTimeUsec();


	while(recvThreadActive){

		//cout<<"\nreading from "<<file<<std::flush;

		//if(msgWaitingToSend)
        //    usleep(1); //if this is too small shit hits the fan

		//read a byte from serial port

		n=device.recvBytes(&byte,1);

		if(n>0){
			
            //counting transfer rate
	/*		
			cnt+=n;
			if(getTimeUsec()-pCntTime >1e6){
				std::cout<<"\nTransfer rate (b/s) = "<<cnt*1e6/(getTimeUsec()-pCntTime);
				pCntTime=getTimeUsec();
				cnt=0;
			}
          */  

			msgReceived=mavlink_parse_char(MAVLINK_COMM_0,byte,&msg,&status);
			if(msgReceived){

				//telemetry
				if(!px4TelemBuffer->push(msg)){
					#ifdef DISPLAY_MESSAGES
					perror("\n(PIXHAWK) ERROR: Telemetry buffer full.");
					#endif
				}

				//implement check for correct sys id at some point
				//or you know just ignore this as usual
				px4StateMsgs->sysID=msg.sysid;
				px4StateMsgs->compID=msg.compid;

				switch(msg.msgid){
					case MAVLINK_MSG_ID_HEARTBEAT:
						mavlink_msg_heartbeat_decode(&msg,&(msgs.heartbeat));
						px4StateMsgs->heartbeat.setData(msgs.heartbeat,getTimeUsec());
					break;

					case MAVLINK_MSG_ID_SYSTEM_TIME:
						mavlink_msg_system_time_decode(&msg,&(msgs.sysTime));
						px4StateMsgs->sysTime.setData(msgs.sysTime, getTimeUsec());
						px4StateMsgs->setTimeSkew(msgs.sysTime.time_unix_usec - (int64_t)getTimeUsec());
					break;

					case MAVLINK_MSG_ID_SYS_STATUS:
						mavlink_msg_sys_status_decode(&msg,&(msgs.sysStatus));
						px4StateMsgs->sysStatus.setData(msgs.sysStatus, getTimeUsec());
					break;
					case MAVLINK_MSG_ID_BATTERY_STATUS:
						mavlink_msg_battery_status_decode(&msg,&(msgs.batteryStatus));
						px4StateMsgs->batteryStatus.setData(msgs.batteryStatus, getTimeUsec());
					break;
					case MAVLINK_MSG_ID_RADIO_STATUS:
						mavlink_msg_radio_status_decode(&msg,&(msgs.radioStatus));
						px4StateMsgs->radioStatus.setData(msgs.radioStatus, getTimeUsec());
					break;
					case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
						mavlink_msg_local_position_ned_decode(&msg,&(msgs.localPos));
						px4StateMsgs->localPos.setData(msgs.localPos, getTimeUsec());
					break;

					case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED:
						mavlink_msg_position_target_local_ned_decode(&msg,&(msgs.localPosTarget));
						px4StateMsgs->localPosTarget.setData(msgs.localPosTarget, getTimeUsec());
					break;

					case MAVLINK_MSG_ID_HIGHRES_IMU:
						mavlink_msg_highres_imu_decode(&msg,&(msgs.imuData));
						px4StateMsgs->imuData.setData(msgs.imuData, getTimeUsec());

					break;
					case MAVLINK_MSG_ID_ATTITUDE:
						mavlink_msg_attitude_decode(&msg,&(msgs.attitude));
						px4StateMsgs->attitude.setData(msgs.attitude, getTimeUsec());
					break;

					case MAVLINK_MSG_ID_ATTITUDE_TARGET:
						mavlink_msg_attitude_target_decode(&msg,&(msgs.attitudeTarget));
						px4StateMsgs->attitudeTarget.setData(msgs.attitudeTarget, getTimeUsec());;
					break;

					case MAVLINK_MSG_ID_OPTICAL_FLOW_RAD:
						mavlink_msg_optical_flow_rad_decode(&msg,&(msgs.opticalFlow));
						px4StateMsgs->opticalFlow.setData(msgs.opticalFlow, getTimeUsec());
					break;

					case MAVLINK_MSG_ID_TIMESYNC:
						mavlink_msg_timesync_decode(&msg, &(msgs.timesync));
						//std::cout<<msgs.timesync.ts1<<" "<<msgs.timesync.tc1<<std::endl;
						//msgs.timesync.tc1 = getTimeUsec();
						//mavlink_message_t msg;
						//mavlink_msg_timesync_encode(1,1,&msg,&(msgs.timesync));
						//sendMessage(msg);
						//px4StateMsgs->setTimeSkew(msgs.timesync.ts1 - (int64_t)getTimeUsec());

					break;
				}
	//no mutex as getting an older msgs doesn't seem like an issue atm
	//might not be the fastest solution
				
				//px4StateMsgs->printMessages();
				
			}//msg received end
		}//n end
	}//while loop end
}




