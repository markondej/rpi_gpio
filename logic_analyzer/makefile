FLAGS = -Wall -O3 -std=c++11

all: logic_analyzer.o mailbox.o rpi_gpio.o
	g++ -L/opt/vc/lib -lpthread -lbcm_host -o logic_analyzer logic_analyzer.o mailbox.o rpi_gpio.o

mailbox.o: ../mailbox.c ../mailbox.h
	g++ $(FLAGS) -c ../mailbox.c
	
rpi_gpio.o: ../rpi_gpio.cpp ../rpi_gpio.hpp
	g++ $(FLAGS) -I/opt/vc/include -c ../rpi_gpio.cpp

logic_analyzer.o: logic_analyzer.cpp
	g++ $(FLAGS) -c logic_analyzer.cpp

clean:
	rm *.o
