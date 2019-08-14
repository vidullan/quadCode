#include "connect.h"
#include <linux/types.h>
#include <iostream>
#include "struct.h"

#define DISPLAY_MESSAGES

using namespace std;

//Constructor
Connect::Connect(std::string ip, int portNum,
	CircularBuffer<mavlink_message_t> *recvBuf,
	CircularBuffer<mavlink_message_t> *sendBuf){
    
	serverIP=ip;
	serverPortNo=portNum;
	
	sendThreadActive=false;
	receiveThreadActive=false;

	recvMavlink = recvBuf;
	sendMavlink = sendBuf;
}

//Destructor
Connect::~Connect(){
	stopReceiveThread();
	stopSendThread();

	recvMavlink = NULL;
	sendMavlink = NULL;
}


int Connect::init(){
	//set TCP socket
    //Initialize two socket file descriptors

	sockfd=socket(AF_INET, SOCK_STREAM,0);
	if(sockfd<0){
		perror("\n(NETWORK) ERROR: Cannot open socket");
		return -1;
	}
	
    //Send server IP and get host name
    //Function returns  pointer to hostent struct
    //Function fills variable values of struct server
	server=gethostbyname(serverIP.c_str());

    //If gethostbyname returns null then print error
	if(server==NULL){
		perror("\n(NETWORK) ERROR: No such host\n");
		return -2;
	}

	//Initialize connection for telem
    //Function intializes struct to NULL
 	bzero((char *) &serv_addr, sizeof(serv_addr));
    //Set value of struct variable sin_family to AF_INET
    //sin_family stands for socket in family
    //AF_INET is an address family and designates that the socket can comminicate with IPv4 addresses
    serv_addr.sin_family = AF_INET;
    //Copy h_addr in struct server to sin_addr.s_addr in struct serv_addr
    //Copy host address to socket address
    bcopy((char *)server->h_addr,
        (char *)&serv_addr.sin_addr.s_addr,
        server->h_length);
    //Convert host to network byte address and store in socker port
    serv_addr.sin_port = htons(serverPortNo);

    //Connect socket to address of the server using first socket file descriptor
	if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0){
        perror("\n(NETWORK) ERROR: Cannot connect to server");
		return -3;
	}

	//Set read timeout to .1 sec
	struct timeval tv;
	tv.tv_sec=0;
	tv.tv_usec=100000;
	setsockopt(sockfd,SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv,sizeof(struct timeval));


	startSendThread();
	startReceiveThread();

	return 1;
}


void Connect::startSendThread(){
	if(sendThreadActive==false){
		sendThreadActive=true;
		//Spawn thread for sending data
		sendThread=std::thread(&Connect::sendData,this);

		#ifdef DISPLAY_MESSAGES
		cout<<"\n(NETWORK) Spawning thread to send data.";
		#endif
	}
}


void Connect::sendData(){

	mavlink_message_t msg;
	uint16_t len;
	uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
	int n;
	//uint8_t req;

	uint64_t uTimeDelay = 1e4;
	uint64_t pT = getTimeUsec();
	uint64_t cT = getTimeUsec();

	while(sendThreadActive){

		int numMsgs = sendMavlink->size();

		if(numMsgs){
			for(int i=0; i<numMsgs; i++){
				sendMavlink->pop(msg);
			
				len=mavlink_msg_to_send_buffer(buffer,&msg);
				
				n=write(sockfd,buffer,len);
				if(n<0)
					perror("(NETWORK) ERROR: Could not send telemetry bytes");

			}
		}

		//Maintain close to this time delay as possible to reduce load
		
		cT = getTimeUsec();
		if ( (cT - pT) < uTimeDelay)
			usleep(uTimeDelay - (cT - pT));
		//cout<<getTimeUsec()<<" "<<cT - pT<<endl;
		pT = cT;
		
		//cout<<"Semt: "<<numMsgs<<endl;
		//usleep(uTimeDelay);
	}
}


void Connect::stopSendThread(){
	if(sendThreadActive){
		sendThreadActive=false;
		if(sendThread.joinable())
			sendThread.join();
	}
}


void Connect::startReceiveThread(){
	if(receiveThreadActive==false){
		receiveThreadActive=true;
		//Spawn thread for telemetry
		receiveThread=std::thread(&Connect::receiveData,this);

		#ifdef DISPLAY_MESSAGES
		cout<<"\n(NETWORK) Spawning thread to receive data";
		#endif
	}
}


void Connect::receiveData(){

	int n;
	//char req;

	uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

	uint64_t uTimeDelay = 1e2;
	uint64_t pT = getTimeUsec();
	uint64_t cT = getTimeUsec();

	uint8_t msgReceived;
	mavlink_status_t status;

	while(receiveThreadActive){

		n=read(sockfd,buffer,MAVLINK_MAX_PACKET_LEN);
		
		if(n>0){
			mavlink_message_t msg;
			for(int i=0; i < n ; i++)
				msgReceived=mavlink_parse_char(MAVLINK_COMM_0,buffer[i],&msg,&status);
			if (msgReceived)
				if(!recvMavlink->push(msg))
					cout<<"\n(NETWORK) ERROR: GS Mavlink msg receive buffer full.";
		}

		usleep(uTimeDelay);
	}
}

void Connect::stopReceiveThread(){
	if(receiveThreadActive){
		receiveThreadActive=false;
		if(receiveThread.joinable())
			receiveThread.join();
	}
}



