#include "../rpi_gpio.hpp"
#include <iostream>
#include <thread>
#include <chrono>
#include <csignal>
#include <functional>
#include <atomic>
#include <unistd.h>

std::atomic_bool stop(false);

void sigIntHandler(int sigNum)
{
    stop = true;
}

void IRQThread(unsigned pin, bool high, const std::function<void()> &callback) {
    GPIO::Controller &gpio = GPIO::Controller::GetInstance();
    while (!stop) {
        for (auto event : gpio.GetEvents()) {
            if ((event.number == pin) && (event.high == high)) {
                callback();
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
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
    std::signal(SIGTSTP, sigIntHandler);

    std::thread irqThread(IRQThread, pin, true, [&]() {
        std::cout << "Interrupt" << std::endl;
    });
    while (!stop) {
        std::cout << "Working" << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    irqThread.join();
    return 0;
}