# Raspberry Pi GPIO real-time library

This simple library allows Raspberry Pi GPIO level reading in "real-time", like microcontrollers do. Produced output is vector of input level changes with timestamp included (1us accuracy).
It should work on any Pi (including 4), but I have tested it only on Zero so far.

In order to achieve time-accurate GPIO level change informations built-in peripherals are beeing used. The PWM delivers accurate synchronization signal to the DMA engine via DREQ line, and DMA transfers Block 0 GPIO state every microsecond to buffered memory. Buffered memory can be easily read in user space and, as GPIO read timings are defined, level change relative timestamp may be computed.

## Examples

Three example programs are supplied: simple logic analyzer, IRQ demo, and DHT22 temperature and humidity sensor client application.

For DHT22 no external pull-up resistor is required, simply connect VCC to PIN 1, GND to GND, and DATA to any available Raspberry Pi's GPIO. To install this software please follow these commands in terminal (use -p to define used GPIO number):
```
git clone https://github.com/markondej/rpi_gpio
cd rpi_gpio/dht22
make
sudo ./dht22 -p 4
```
