CC = g++
CFLAGS = -g -Wall

default: field

field: Field.o Cam.o Robot.o Pose.o
	$(CC) $(CFLAGS) -o field Field.o Cam.o Robot.o Pose.o -lopencv2

Field.o: Field.cpp Field.hpp Cam.hpp Robot.hpp
	$(CC) $(CFLAGS) -c Field.cpp

Cam.o: Cam.cpp Cam.hpp
	$(CC) $(CFLAGS) -c Cam.cpp

Robot.o: Robot.cpp Robot.hpp Pose.hpp
	$(CC) $(CFLAGS) -c Robot.cpp

Pose.o: Pose.cpp Pose.hpp
	$(CC) $(CFLAGS) -c Pose.cpp

clean:
	$(RM) count *.o *~
