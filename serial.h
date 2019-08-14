#ifndef SERIAL_H
#define SERIAL_H

#include <termios.h>
#include <string>
#include <unistd.h>
#include <iostream>
#include <fcntl.h>
#include <sys/ioctl.h>

class Serial{

	int file; //file descriptor

public:

	int connect(int bR, std::string dN);

    int sendBytes(uint8_t *buffer, int len);
    int recvBytes(uint8_t *buffer, int len);

    Serial(){
        file=-1;
    }

    void disconnect(){
        if(file>0)
            close(file);
    }

};

#endif