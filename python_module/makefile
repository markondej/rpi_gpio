FLAGS = -Wall -O3 -std=c++11

all: gpiomodule.o mailbox.o rpi_gpio.o
	g++ -shared -fPIC -o gpio.so gpiomodule.o mailbox.o rpi_gpio.o -L/opt/vc/lib -lpthread -lbcm_host -lpython3.9

mailbox.o: ../mailbox.c ../mailbox.h
	g++ $(FLAGS) -c ../mailbox.c
	
rpi_gpio.o: ../rpi_gpio.cpp ../rpi_gpio.hpp
	g++ $(FLAGS) -I/opt/vc/include -c ../rpi_gpio.cpp

gpiomodule.o: gpiomodule.cpp
	g++ $(FLAGS) -I/usr/include/python3.9 -c gpiomodule.cpp

clean:
	rm *.o