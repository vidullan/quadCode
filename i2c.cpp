#include <unistd.h>				//Needed for I2C port
#include <fcntl.h>				//Needed for I2C port
#include <sys/ioctl.h>			//Needed for I2C port
#include <linux/i2c-dev.h>		//Needed for I2C port
#include <stdio.h>
#include <time.h>
#include <stdint.h>
#include <iostream>

int main(){

    int length;
    unsigned char buffer[4];
    int file_i2c;

    char *filename = (char*)"/dev/i2c-1";
    if ((file_i2c= open(filename, O_RDWR)) < 0)
    {
        //ERROR HANDLING: you can check errno to see what went wrong
        printf("Failed to open the i2c bus");
        return -1;
    }
    if (ioctl(file_i2c, I2C_SLAVE, 0x30) < 0)
    {
        printf("Failed to acquire bus access and/or talk to slave.\n");
        //ERROR HANDLING; you can check errno to see what went wrong
        return -1;
    }

    //----- WRITE BYTES -----
    buffer[0] = 0x01;
    length = 1;			//<<< Number of bytes to write
    if (write(file_i2c, buffer, length) != length)		//write() returns the number of bytes actually written, if it doesn't match then an error occurred (e.g. no response from the device)
    {
        /* ERROR HANDLING: i2c transaction failed*/ 
        printf("Failed to write to the i2c bus.\n");
    }
    

    //----- READ BYTES -----
    while(1){
    length = 4;			//<<< Number of bytes to read
    if (read(file_i2c, buffer, length) != length)		//read() returns the number of bytes actually read, if it doesn't match then an error occurred (e.g. no response from the device)
    {
        //ERROR HANDLING: i2c transaction failed
        printf("Failed to read from the i2c bus.\n");
    }
    else
    {
        if(buffer[0] == 'R'){
            uint16_t dist = buffer[2] << 8 & 0xFFFF | buffer[1] & 0xFFFF;
            std::cout<<dist<<std::endl;
        }
    }
    }
    return 0;
    
}