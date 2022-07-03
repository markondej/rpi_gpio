FLAGS = -Wall -O3 -std=c++11

all: dht22.o mailbox.o rpi_gpio.o
	g++ -o dht22 dht22.o mailbox.o rpi_gpio.o -L/opt/vc/lib -lpthread -lbcm_host

mailbox.o: ../mailbox.c ../mailbox.h
	g++ $(FLAGS) -c ../mailbox.c
	
rpi_gpio.o: ../rpi_gpio.cpp ../rpi_gpio.hpp
	g++ $(FLAGS) -I/opt/vc/include -c ../rpi_gpio.cpp

dht22.o: dht22.cpp
	g++ $(FLAGS) -c dht22.cpp

clean:
	rm *.o
