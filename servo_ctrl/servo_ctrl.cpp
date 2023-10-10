#include "../rpi_gpio.hpp"
#include <iostream>
#include <csignal>
#include <unistd.h>

void sigIntHandler(int sigNum)
{
}

int main(int argc, char** argv) {
    unsigned pin = GPIO4;
    int opt;
    while ((opt = getopt(argc, argv, "p:")) != -1) {
        switch (opt) {
        case 'p':
            pin = std::atoi(optarg);
            break;
        }
    }

    std::signal(SIGINT, sigIntHandler);
    std::signal(SIGTERM, sigIntHandler);

    try {
        GPIO::Controller &gpio = GPIO::Controller::GetInstance();
        gpio.SetMode(pin, GPIO::Mode::Out);
        std::vector<GPIO::Event> schedule = {
            { 0, pin, true },
            { 1000, pin, false }
        };
        gpio.SetSchedule(schedule, 20000);
        
        std::cout << "Servo angle (0-180): " << std::endl;
        while (true) {
            std::string str;
            std::cin >> str;
            int angle = std::stoi(str);
            if (angle < 0) {
                angle = 0;
            }
            if (angle > 180) {
                angle = 180;
            }
            
            schedule[1].time = 1000 + angle * 1000 / 180;
            gpio.SetSchedule(schedule, 20000);
        }
    } catch (std::exception &error) {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
