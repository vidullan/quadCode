CC=g++
CFLAGS= -std=c++11 -pthread 
CVFLAGS=`pkg-config --cflags --libs opencv`
all: main

main:common.o pixhawk.o connect.o t265.o videostream.o main.cpp 
	$(CC) $(CFLAGS)  $(LPATH) -lrt common.o struct.o transforms.o serial.o t265.o connect.o pixhawk.o videostream.o main.cpp -o a.out -lrealsense2 $(CVFLAGS)

arduino.o:arduino.cpp arduino.h 
	$(CC) $(CFLAGS) -c arduino.cpp

pixhawk.o:serial.o common.o pixhawk.cpp pixhawk.h struct.o
	$(CC) $(CFLAGS) -I ./mavlink2/common -c pixhawk.cpp

connect.o:common.o struct.o connect.cpp connect.h 
	$(CC) $(CFLAGS) -I ./mavlink2/common -c connect.cpp

t265.o: common.o t265.h t265.cpp transforms.o struct.o
	$(CC) $(CFLAGS) -lrealsense2 -c t265.cpp

videostream.o: common.o videostream.h videostream.cpp
	$(CC) $(CFLAGS)  -c videostream.cpp

serial.o:common.o serial.h serial.cpp
	$(CC) $(CFLAGS) -c serial.cpp

struct.o:struct.h struct.cpp
	$(CC) $(CFLAGS) -I ./mavlink2/common -c struct.cpp

transforms.o: common.o transforms.h transforms.cpp
	$(CC) $(CFLAGS) -c transforms.cpp

common.o: common.h common.cpp
	$(CC) $(CFLAGS) -c common.cpp

clean:
	rm connect.o pixhawk.o autopilot.o main.o a.out
