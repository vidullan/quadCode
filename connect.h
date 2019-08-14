/*Defines class Connect that uses sockets to implement TCP communication
with the command center*/

#ifndef CONNECT
#define CONNECT
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <stdint.h>
#include <thread>
#include <mutex>
#include "struct.h"
#include "common.h"


class Connect{
	
	std::string serverIP;
	int serverPortNo;

	int sockfd; //socket descriptor, port no, status variable
	struct sockaddr_in serv_addr;
	struct hostent *server;

	std::thread sendThread;
	bool sendThreadActive;

	std::thread receiveThread;
	bool receiveThreadActive;
	
	CircularBuffer<mavlink_message_t> *recvMavlink;
	CircularBuffer<mavlink_message_t> *sendMavlink;

	void startSendThread();
	void stopSendThread();
	void startReceiveThread();
	void stopReceiveThread();

	void receiveData();
	void sendData();
	void sendDataImg();


	
public:
    //initializes socket (ip address, port no)
    //needs to be called after declaring object
	Connect(std::string a, int b, 
		CircularBuffer<mavlink_message_t> *recvBuf,
		CircularBuffer<mavlink_message_t> *sendBuf);
	~Connect();
	int init();

};

#endif
