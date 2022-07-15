import gpio;
import sys;

gpio.set_mode(4, 0);
gpio.set(4, 1);

i = 0;
for line in sys.stdin:
    gpio.set(4, i);
    if i != 0:
        i = 0;
    else:
        i = 1;
    if 'exit' == line.rstrip():
        break
