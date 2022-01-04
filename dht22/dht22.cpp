#include "../rpi_gpio.hpp"
#include <utility>
#include <iostream>
#include <thread>
#include <chrono>
#include <unistd.h>

#define DHT22_READ_PROCESS_PRIORITY 10

std::pair<double, double> ReadDHT22(unsigned pin) {
    enum class Communication {
        StartSignal,
        PullUpAfterStart,
        ResponseSignal,
        PullUpAfterResponse,
        DataStart,
        DataValue
    } status = Communication::StartSignal;

    auto check = [&](unsigned long long value, unsigned long long begin, unsigned long long end) -> bool {
        return (value >= begin) && (value <= end); 
    };

    GPIO::Controller &gpio = GPIO::Controller::GetInstance();
    gpio.SetResistor(pin, GPIO::Resistor::PullUp);
    gpio.SetMode(pin, GPIO::Mode::In);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    nice(-DHT22_READ_PROCESS_PRIORITY);
    gpio.SetMode(pin, GPIO::Mode::Out);
    auto start = std::chrono::high_resolution_clock::now();
    gpio.Set(pin, false);
    do {
        std::this_thread::sleep_for(std::chrono::microseconds(1));
    } while (std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start).count() < 1000);
    gpio.SetMode(pin, GPIO::Mode::In);
    nice(DHT22_READ_PROCESS_PRIORITY);

    unsigned long long previous = 0;
    uint64_t data = 0; unsigned count = 0;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    for (auto event : gpio.GetEvents()) {
        if (event.number == pin) {
            unsigned long long delta = event.time - previous;
            previous = event.time;
            switch (status) {
                case Communication::StartSignal:
                    if (event.high && check(delta, 950, 1100)) {
                        status = Communication::PullUpAfterStart;
                        continue;
                    }
                    break;
                case Communication::PullUpAfterStart:
                    if (!event.high && check(delta, 16, 44)) {
                        status = Communication::ResponseSignal;
                        continue;
                    }
                    break;
                case Communication::ResponseSignal:
                    if (event.high && check(delta, 76, 84)) {
                        status = Communication::PullUpAfterResponse;
                        continue;
                    }
                    break;
                case Communication::PullUpAfterResponse:
                    if (!event.high && check(delta, 76, 84)) {
                        status = Communication::DataStart;
                        continue;
                    }
                    break;
                case Communication::DataStart:
                    if (event.high && check(delta, 46, 74)) {
                        status = Communication::DataValue;
                        continue;
                    }
                    break;
                case Communication::DataValue:
                    if (!event.high && check(delta, 22, 32)) {
                        if (count < 40) {
                            data = (data << 0x01);
                        }
                        status = Communication::DataStart;
                        count++;
                        continue;
                    }
                    if (!event.high && check(delta, 66, 74)) {
                        if (count < 40) {
                            data = (data << 0x01) | 0x01;
                        }
                        status = Communication::DataStart;
                        count++;
                        continue;
                    }
                    break;
            }
        }
    }

    if (!count) {
        throw std::runtime_error("No signal received");
    }

    if (count < 40) {
        throw std::runtime_error("Corrupted data received");
    }

    if ((data & 0xff) != ((((data >> 32) & 0xff) + ((data >> 24) & 0xff) + ((data >> 16) & 0xff) + ((data >> 8) & 0xff)) & 0xff)) {
        throw std::runtime_error("Checksum error");
    }

    double temp = static_cast<double>((data >> 8) & 0x7fff) * 0.1;
    if ((data >> 23) & 0x01) {
        temp = -temp;
    }
    double humidity = static_cast<double>((data >> 24) & 0xffff) * 0.1;

    return { temp, humidity };
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
    try {
        auto read = ReadDHT22(pin);
        std::cout << "Temperature [C]: " << read.first << std::endl << "Humidity [%]: " << read.second << std::endl;
    } catch (std::exception &error) {
        std::cout << error.what() << std::endl;
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
