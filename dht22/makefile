CXX = g++
CXXFLAGS = -Wall -O3 -std=c++11

OBJS = dht22.o mailbox.o rpi_gpio.o

.PHONY: clean

dht22: $(OBJS)
	$(CXX) -o dht22 $(OBJS) -lpthread

mailbox.o: ../mailbox.c
	$(CXX) $(CXXFLAGS) -c ../mailbox.c
	
rpi_gpio.o: ../rpi_gpio.cpp
	$(CXX) $(CXXFLAGS) -I/opt/vc/include -c ../rpi_gpio.cpp

dht22.o: dht22.cpp
	$(CXX) $(CXXFLAGS) -c dht22.cpp

clean:
	rm *.o
