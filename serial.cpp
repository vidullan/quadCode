#include "serial.h"

int Serial::connect(int bR, std::string dN){

    speed_t baud;

	switch(bR){
        case 9600:baud=B9600;
		break;
		case 57600:baud=B57600;
		break;
		case 115200:baud=B115200;
		break;
		case 921600:baud=B921600;
		break;
        default:
		    std::cout<<"\nUnsupported baud rate";
	}

	//opens port
	file=open(dN.c_str(),O_RDWR | O_NOCTTY | O_NONBLOCK);

	if(file<0){
		perror("(SERIAL) ERROR: Could not open serial port");
		return -1; //failed to open USB
	}

	#ifdef DISPLAY_MESSAGES
	std::cout<<"\n(SERIAL) Connected to "<<dN;
	#endif

	usleep(1e5);

	fcntl(file,F_SETFL,0); //???

	struct termios toptions; //set linux serial options

	//gets current attributes
	if(tcgetattr(file,&toptions)<0){
		perror("(SERIAL) ERROR: Could not get serial port term attributes");
		return -2;
	}

	#ifdef DISPLAY_MESSAGES
	std::cout<<"\n(SERIAL) Port term attributes read."<<std::flush;
	#endif

	//px4 options
	toptions.c_iflag &= ~(IGNBRK|BRKINT|ICRNL|INLCR|PARMRK|INPCK|ISTRIP|IXON);
	toptions.c_oflag &= ~(OCRNL|ONLCR|ONLRET|ONOCR|OFILL|OPOST);
	//set parity, stop bit, and data bits
	toptions.c_cflag &= ~(PARENB|CSIZE);
	toptions.c_cflag |= CS8;
	//toptions.c_cflag |= CREAD|CLOCAL;
	toptions.c_lflag &= ~(ICANON|ECHO|ECHONL|ISIG|IEXTEN);
	//return immediately
	toptions.c_cc[VMIN]=1;
	toptions.c_cc[VTIME]=10; //was 0

	#ifdef OLCUC
		toptions.c_oflag &= ~OLCUC;
	#endif

	#ifdef ONOEOT
		toptions.c_oflag &= ~ONOEOT;
	#endif

	//set baud rate
	cfsetspeed(&toptions, baud);

    	//set attributes
	if(tcsetattr(file,TCSANOW|TCSAFLUSH,&toptions)<0){
		perror("(SERIAL) ERROR: Could not set serial port term attributes");
		return -3;
	}

	#ifdef DISPLAY_MESSAGES
	std::cout<<"\n(SERIAL) Port term attributes set successfully."<<std::flush;
	#endif

	//NEED TO DO ERROR CHECKING HERE to check if they are set properly
	int status;
	ioctl(file,TIOCMGET,&status);
	status|=TIOCM_DTR;
	status|=TIOCM_RTS;
	ioctl(file,TIOCMSET,&status);

	return 0;
}

//Send bytes to buffer. Should send len number of bytes. Check with n.
int Serial::sendBytes(uint8_t *buffer, int len){
	int n;
	n=write(file,buffer,len);
	tcdrain(file);
	return n;
}

//Read bytes to buffer. Tries to read len number of bytes. Check with n.
int Serial::recvBytes(uint8_t *buffer, int len){
    int n;
    n=read(file,buffer,len);
    return n;
}