#include "../rpi_gpio.hpp"
#include <iostream>
#include <thread>
#include <chrono>
#include <csignal>

bool stop = false;

void sigIntHandler(int sigNum)
{
    stop = true;
}

int main(int argc, char** argv) {
    std::signal(SIGINT, sigIntHandler);
    std::signal(SIGTSTP, sigIntHandler);	
	
	GPIO::Controller &gpio = GPIO::Controller::GetInstance();
	while (!stop) {
		for (auto event : gpio.GetEvents()) {
			std::cout << "GPIO " << event.number << ", time: " << event.time << "us, state: " << (event.high ? "HIGH" : "LOW") << std::endl;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
	return 0;
}