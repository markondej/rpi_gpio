FLAGS = -Wall -O3 -std=c++11

all: irq_demo.o mailbox.o rpi_gpio.o
	g++ -L/opt/vc/lib -lpthread -lbcm_host -o irq_demo irq_demo.o mailbox.o rpi_gpio.o

mailbox.o: ../mailbox.c ../mailbox.h
	g++ $(FLAGS) -c ../mailbox.c
	
rpi_gpio.o: ../rpi_gpio.cpp ../rpi_gpio.hpp
	g++ $(FLAGS) -I/opt/vc/include -c ../rpi_gpio.cpp

irq_demo.o: irq.cpp
	g++ $(FLAGS) -c irq.cpp -o irq_demo.o

clean:
	rm *.o
