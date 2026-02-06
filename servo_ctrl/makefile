CXX = g++
CXXFLAGS = -Wall -O3 -std=c++11

OBJS = servo_ctrl.o mailbox.o rpi_gpio.o

.PHONY: clean

servo_ctrl: $(OBJS)
	$(CXX) -o servo_ctrl $(OBJS) -lpthread

mailbox.o: ../mailbox.c
	$(CXX) $(CXXFLAGS) -c ../mailbox.c
	
rpi_gpio.o: ../rpi_gpio.cpp
	$(CXX) $(CXXFLAGS) -c ../rpi_gpio.cpp

servo_ctrl.o: servo_ctrl.cpp
	$(CXX) $(CXXFLAGS) -c servo_ctrl.cpp

clean:
	rm *.o
