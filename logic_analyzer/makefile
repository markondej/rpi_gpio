CXX = g++
CXXFLAGS = -Wall -O3 -std=c++11

OBJS = logic_analyzer.o mailbox.o rpi_gpio.o

.PHONY: clean

logic_analyzer: $(OBJS)
	$(CXX) -o logic_analyzer $(OBJS) -lpthread

mailbox.o: ../mailbox.c
	$(CXX) $(CXXFLAGS) -c ../mailbox.c
	
rpi_gpio.o: ../rpi_gpio.cpp
	$(CXX) $(CXXFLAGS) -c ../rpi_gpio.cpp

logic_analyzer.o: logic_analyzer.cpp
	$(CXX) $(CXXFLAGS) -c logic_analyzer.cpp

clean:
	rm *.o
