#include <unistd.h>				//Needed for I2C port
#include <fcntl.h>				//Needed for I2C port
#include <sys/ioctl.h>			//Needed for I2C port
#include <linux/i2c-dev.h>		//Needed for I2C port
#include <stdio.h>
#include <time.h>
#include <stdint.h>

class Arduino {
    int devFile;
    int devAddr;

    unsigned char buffer[4];

 	uint64_t getTimeUsec(){
		struct timespec t;
		clock_gettime(CLOCK_REALTIME,&t);
		return (t.tv_sec)*1e6+(t.tv_nsec)/1e3;
	}

	void sendData();
	void receiveData();
	void printMessages();

public:

	Arduino(int a);

	//initialize the I2C connection. Needs to be called after obj creations
	int init();

	void setServo(int angle);
    float getSonarDist();
	void deInit();

	~Arduino(){
	}

};