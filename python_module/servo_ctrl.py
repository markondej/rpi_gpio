import gpio;
import sys;

pin = 4;
schedule = [[0, pin, 1], [1000, pin, 0]];

gpio.set_mode(pin, 0);
gpio.set_schedule(schedule, 20000);

print("Servo angle (0-180):");
for line in sys.stdin:
    angle = int(line.rstrip());
    if angle < 0:
        angle = 0;
    if angle > 180:
        angle = 180;
    schedule[1][0] = 1000 + angle * 1000 / 180;
    print(schedule);
    gpio.set_schedule(schedule, 20000);
