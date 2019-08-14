#include "arduino.h"

//Initializes object
Arduino::Arduino(int a){
    devAddr = a;
}


//Initializes the i2c connection
int Arduino::init(){
    //----- OPEN THE I2C BUS -----
    char *filename = (char*)"/dev/i2c-1";
    if ((devFile = open(filename, O_RDWR)) < 0)
    {
        //ERROR HANDLING: you can check errno to see what went wrong
        printf("Failed to open the i2c bus");
        return -1;
    }
    if (ioctl(devFile, I2C_SLAVE, devAddr) < 0)
    {
        printf("Failed to acquire bus access and/or talk to slave.\n");
        //ERROR HANDLING; you can check errno to see what went wrong
        return -1;
    }

    return 0;
}

void Arduino::deInit(){
}

void Arduino::setServo(int angle){
    if (angle >= 0 and angle <= 90){
        buffer[0] = 'G';
        buffer[1] = angle & 0xFF;
        buffer[2] = '\r';

        //write() returns the number of bytes actually written, 
        //if it doesn't match then an error occurred (e.g. no response from the device)    
        if (write(devFile, buffer, 3) != 3)		    {
            // ERROR HANDLING: i2c transaction failed 
            printf("Failed to write to the i2c bus.\n");
        }
    }
}

float Arduino::getSonarDist(){

    buffer[0] = 'S';
    buffer[2] = '\r';

    //write() returns the number of bytes actually written, 
    //if it doesn't match then an error occurred (e.g. no response from the device)    
    if (write(devFile, buffer, 3) != 3)		    {
        // ERROR HANDLING: i2c transaction failed 
        printf("Failed to write to the i2c bus.\n");
    }

    if (read(devFile, buffer, 4) != 4)		//read() returns the number of bytes actually read, if it doesn't match then an error occurred (e.g. no response from the device)
    {
        //ERROR HANDLING: i2c transaction failed
        printf("Failed to read from the i2c bus.\n");
    }
    else
    {
        //handle input data
    }

}
