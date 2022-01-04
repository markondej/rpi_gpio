# Raspberry Pi GPIO real-time library

This simple library allows Raspberry Pi GPIO level reading in "real-time", like microcontrollers do. Produced output is vector of input level changes with timestamp included (2us accuracy).
It should work on any Pi (including 4), but I have tested it only on Zero so far.

In order to achieve time-accurate GPIO level change informations built-in peripherals are beeing used. The PWM delivers accurate synchronization signal to the DMA engine via DREQ line, and DMA transfers Block 0 GPIO state every microsecond to buffered memory. Buffered memory can be easily read in user space and, as GPIO read timings are defined, level change relative timestamp may be computed.

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
