# Raspberry Pi GPIO real-time library

This simple library allows Raspberry Pi GPIO level reading in "real-time", like microcontrollers do. Produced output is vector of input level changes with timestamp included (2us accuracy).
It should work on any Pi (including 4), but I have tested it only on Zero so far.

In order to achieve time-accurate GPIO level change informations built-in peripherals are beeing used. The PWM delivers accurate synchronization signal to the DMA engine via DREQ line, and DMA transfers Block 0 GPIO state every two microseconds to buffered memory. Buffered memory can be easily read in user space and, as GPIO read timings are defined, level change relative timestamp may be computed.

## Supported functionalities

### GPIO time non-crucial operations

For simple GPIO operations like setting, reading values, see example code below:

```
auto &gpio = GPIO::Controller::GetInstance();
gpio.SetMode(GPIO4, GPIO::Mode::Out); // Switch GPIO 4 to output
gpio.Set(GPIO4, true); // Set GPIO 4 level high
...
gpio.SetMode(GPIO4, GPIO::Mode::In); // Switch GPIO 4 to input
bool level4 = gpio.Get(GPIO4); // Obtain GPIO 4 level
```

### GPIO time crucial operations

Time crucial operations are performed by operating on GPIO event lists. Single event is defined as structure:

```
struct Event {
    unsigned long long time; // Timestamp in microseconds, defines moment when event took/should take place
    unsigned number; // GPIO number
    bool high; // GPIO level, TRUE for high
};
```

In order to obtain events registered in past use GetEvents method. To perform level changes in strictly defined timing use SetSchedule method. See example code below:

```
auto &gpio = GPIO::Controller::GetInstance();
gpio.SetMode(GPIO4, GPIO::Mode::Out); // Switch GPIO 4 to output
gpio.SetMode(GPIO17, GPIO::Mode::In); // Switch GPIO 17 to input
gpio.SetSchedule(std::vector<GPIO::Event>({
    { 0, GPIO4, false }, // Set GPIO4 level low and...
    { 1000, GPIO4, true }, // after 1000us set GPIO4 level high and...
    { 3000, GPIO4, false } // after 3000us set GPIO4 level low
}), 0); // don't loop schedule, 0us loop interval
...
unsigned long long lastChange = 0;
for (auto event : gpio.GetEvents()) {
    if (event.number == GPIO17) { // get all events which occured on GPIO 17
       if (!event.high && (event.time - lastChange == 1000)) {
           // falling edge after 1000us of high level on GPIO 17 detected
       }
    }
}
```

## Examples

Four example programs are supplied: simple logic analyzer, RC servo controller, IRQ demo, and DHT22 temperature and humidity sensor client application.

### DHT22 client

For DHT22 no external pull-up resistor is required, simply connect VCC to 3.3V (pin 1), GND to GND (pin 6), and DATA to any available Raspberry Pi's GPIO (eg. GPIO 4 - pin 7). To install and run client application please follow these commands in terminal (use -p to define used GPIO number):
```
git clone https://github.com/markondej/rpi_gpio
cd rpi_gpio/dht22
make
sudo ./dht22 -p 4
```

### RC servo control

Connect RC servo to Raspberry Pi: VCC to +5V (pin 2), GNB to GNB (pin 6), signal to GPIO4 (pin 7). Then follow these commands in terminal (use -p to define used GPIO number):
```
git clone https://github.com/markondej/rpi_gpio
cd rpi_gpio/servo_ctrl
make
sudo ./servo_ctrl -p 4
```
