CXX = g++
CXXFLAGS = -Wall -O3 -std=c++11

OBJS = irq_demo.o mailbox.o rpi_gpio.o

.PHONY: clean

irq_demo: $(OBJS)
	$(CXX) -o irq_demo $(OBJS) -lpthread

mailbox.o: ../mailbox.c
	$(CXX) $(CXXFLAGS) -c ../mailbox.c
	
rpi_gpio.o: ../rpi_gpio.cpp
	$(CXX) $(CXXFLAGS) -c ../rpi_gpio.cpp

irq_demo.o: irq.cpp
	$(CXX) $(CXXFLAGS) -c irq.cpp -o irq_demo.o

clean:
	rm *.o
