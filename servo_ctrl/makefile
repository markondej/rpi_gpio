FLAGS = -Wall -O3 -std=c++11

all: servo_ctrl.o mailbox.o rpi_gpio.o
	g++ -L/opt/vc/lib -lpthread -lbcm_host -o servo_ctrl servo_ctrl.o mailbox.o rpi_gpio.o

mailbox.o: ../mailbox.c ../mailbox.h
	g++ $(FLAGS) -c ../mailbox.c
	
rpi_gpio.o: ../rpi_gpio.cpp ../rpi_gpio.hpp
	g++ $(FLAGS) -I/opt/vc/include -c ../rpi_gpio.cpp

servo_ctrl.o: servo_ctrl.cpp
	g++ $(FLAGS) -c servo_ctrl.cpp

clean:
	rm *.o
